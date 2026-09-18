"""
Electrical (SPICE) simulation of the SteeringControl LIN link, including wiring faults.

Runs ngspice from the local KiCad installation via its shared library (no extra install).
For every scenario it simulates one real LIN transaction on the analog level:
  Uno master sends break (0x00 @ 9600), 0x55, 0x3C  ->  RP2040 slave answers 6 bytes
and reports
  - DC levels at idle (what a multimeter shows): VBB-VSS per chip, LBUS, RXD at both MCUs
  - the lowest LBUS level while dominant
  - the bytes each MCU's UART actually decodes from the voltage on its RX pin

MCP2003 behavioural model (values from DS20002230G):
  - operation only with VBB-VSS > 5.5V, transmitter only with CS > 1.4V (VIL 0.8 / VIH 2.0)
  - driver ~30 ohm to VSS, current limit 200mA, slope control ~3us
  - internal 30k + diode pull-up LBUS->VBB, connected only when transmitter enabled
  - receiver threshold 0.5*VBB, RXD open drain ~200 ohm (VOL 0.4V @ 2mA)
  - RXD monitoring: RXD low while bus recessive -> transmitter off
  - TXD internal pull-up (~30k to the internal 4.3V rail)
  - quiescent path VBB->VSS ~100k
Arduino Uno: the 16U2 USB-serial chip drives D0 (RX) through 1k, idle HIGH - modelled.

Usage:  python lin_bus_sim.py            (all scenarios)
        python lin_bus_sim.py 0 4        (selected scenarios)
"""

import ctypes
import os
import sys
import tempfile

KICAD_BIN = r"C:\Program Files\KiCad\9.0\bin"

BAUD = 19200
BIT = 1.0 / BAUD
T_SETTLE = 200e-6
RESPONSE = [0x05, 0x40, 0x00, 0xBB, 0x80, 0x80]   # buttons=0x05, pot1=1024, pot2=3000, checksum

UNO_VIL, UNO_VIH = 1.5, 3.0        # ATmega328P @5V: 0.3/0.6 VCC
RP_VIL, RP_VIH = 0.8, 2.0          # RP2040 @3.3V


# ─── Scenario definitions ────────────────────────────────────────────────────
BASE = dict(
    vbb=12.0,
    master_pullup="vbb",       # "vbb" (1k+diode to VBB) | "gnd" (1k to GND, old README) | "none"
    master_bus_cap=220e-12,
    psu_gnd_ohm=0.01,          # lab supply minus <-> MCU ground
    rp_gnd_ohm=0.01,           # RP2040 ground <-> Uno ground (normally common via PC USB)
    slave_vss_ohm=0.01,        # MCP2003 #1 pin 5 <-> GND
    bus_wire_ohm=0.1,
    slave_rxd_pullup=True,
    slave_txrx_swapped=False,
    slave_cs_connected=True,
)

SCENARIOS = [
    ("Soll: neue README (12V, 1k->VBB, 220pF)", {}),
    ("Alt: VBB = 5V", dict(vbb=5.0)),
    ("Alt: Master 1k nach GND + 100nF", dict(master_pullup="gnd", master_bus_cap=100e-9)),
    ("Alt: README komplett (5V, 1k->GND, 100nF)", dict(vbb=5.0, master_pullup="gnd", master_bus_cap=100e-9)),
    ("Fehler: Netzteil-GND nicht mit MCU-GND verbunden", dict(psu_gnd_ohm=10e6)),
    ("Fehler: RP2040-GND nicht mit Uno-GND verbunden (z.B. Powerbank)", dict(rp_gnd_ohm=10e6)),
    ("Fehler: MCP2003 #1 (Slave) Pin 5 VSS offen", dict(slave_vss_ohm=10e6)),
    ("Fehler: LIN-Draht unterbrochen", dict(bus_wire_ohm=1e9)),
    ("Fehler: RXD-Pull-up am Slave fehlt", dict(slave_rxd_pullup=False)),
    ("Fehler: TX/RX am Slave vertauscht", dict(slave_txrx_swapped=True)),
    ("Fehler: CS am Slave nicht verbunden", dict(slave_cs_connected=False)),
    ("Variante: Master-Pull-up weggelassen", dict(master_pullup="none")),
]


# ─── Waveforms ───────────────────────────────────────────────────────────────
def uart_bits(byte):
    return [0] + [(byte >> i) & 1 for i in range(8)] + [1]


def pwl(segments, v_high, t_edge=50e-9):
    """segments: list of (duration, level) -> PWL string, idle high before and after."""
    pts = [(0.0, v_high)]
    t = T_SETTLE
    level = 1
    for dur, lvl in segments:
        if lvl != level:
            pts.append((t, v_high * level))
            pts.append((t + t_edge, v_high * lvl))
            level = lvl
        t += dur
    if level != 1:
        pts.append((t, 0.0))
        pts.append((t + t_edge, v_high))
    return "PWL(" + " ".join(f"{a:.9g} {b:.4g}" for a, b in pts) + ")", t


def master_segments():
    seg = [(1.0 / 9600, b) for b in uart_bits(0x00)]   # break: 0x00 @ 9600
    seg.append((250e-6, 1))                             # end()/begin()/delayMicroseconds(200)
    for byte in (0x55, 0x3C):
        seg += [(BIT, b) for b in uart_bits(byte)]
    return seg


def slave_segments(t_master_end):
    seg = [(t_master_end - T_SETTLE + 150e-6, 1)]       # answer 150us after PID stop bit
    for byte in RESPONSE:
        seg += [(BIT, b) for b in uart_bits(byte)]
    return seg


# ─── Netlist ─────────────────────────────────────────────────────────────────
def sig(x):
    return f"(0.5*(1+tanh({x})))"


def mcp2003(n, vbb, vss, txd, rxd, cs, lbus):
    """Behavioural MCP2003 instance n."""
    vs = f"V({vbb},{vss})"
    en = sig(f"({vs}-5.5)/0.05")
    cs_on = sig(f"(V({cs},{vss})-1.4)/0.05")
    tx_low = sig(f"(1.4-V({txd},{vss}))/0.05")
    rx_low = sig(f"(0.5*{vs}-V({lbus},{vss}))/(0.02*abs({vs})+0.01)")
    bus_rec = sig(f"(V({lbus},{vss})-0.6*{vs})/0.1")
    rxd_low = sig(f"(2.5-V({rxd},{vss}))/0.1")
    return f"""
* ---- MCP2003 #{n}
Rq{n} {vbb} {vss} 100k
Btxpu{n} {vss} {txd} I={en}*max(4.3-V({txd},{vss}),0)/30k
Bfault{n} flt{n} 0 V={en}*{cs_on}*{bus_rec}*{rxd_low}
Rflt{n} flt{n} fltf{n} 10k
Cflt{n} fltf{n} 0 1n
Bctl{n} ctl{n} 0 V={en}*{cs_on}*{tx_low}*(1-{sig(f"(V(fltf{n})-0.5)/0.05")})
Rctl{n} ctl{n} ctlf{n} 3k
Cctl{n} ctlf{n} 0 1n
Bdrv{n} {lbus} {vss} I=V(ctlf{n})*min(max(V({lbus},{vss}),0)/30,0.2)
Bpu{n} {vbb} {lbus} I={en}*{cs_on}*max(V({vbb},{lbus})-0.7,0)/30k
Brx{n} {rxd} {vss} I={en}*{rx_low}*max(V({rxd},{vss}),0)/200
"""


def netlist(p, t_end):
    m_pwl, t_master_end = pwl(master_segments(), 5.0)
    s_pwl, _ = pwl(slave_segments(t_master_end), 3.3)

    wire_tx, wire_rx = ("txd_s", "rxd_s") if not p["slave_txrx_swapped"] else ("rxd_s", "txd_s")

    lines = [f"* LIN link scenario",
             ".model DSIG D(IS=2.5n N=1.75 RS=0.6)",
             # supplies
             f"Rpsu gpsu 0 {p['psu_gnd_ohm']}",
             f"Vvbb vbb gpsu DC {p['vbb']}",
             f"Rrpgnd grp 0 {p['rp_gnd_ohm']}",
             "V5 v5 0 DC 5",
             "V33 v33 grp DC 3.3",
             # Uno master: TX, CS, RX with 16U2 (1k to 5V) and 4.7k pull-up
             f"Vmtx mtx_src 0 {m_pwl}",
             "Rmtx mtx_src txd_m 25",
             "Vmcs mcs_src 0 DC 5",
             "Rmcs mcs_src cs_m 25",
             "R16u2 v5 rxd_m 1k",
             "Rpu_m v5 rxd_m 4.7k",
             "Rin_m rxd_m 0 10Meg",
             "Cin_m rxd_m 0 5p",
             # RP2040 slave
             f"Vstx stx_src grp {s_pwl}",
             "Rstx stx_src rp_tx 25",
             "Rin_s rp_rx grp 10Meg",
             "Cin_s rp_rx grp 5p",
             f"Rw_tx rp_tx {wire_tx} 0.05",
             f"Rw_rx rp_rx {wire_rx} 0.05",
             ]
    if p["slave_rxd_pullup"]:
        lines.append("Rpu_s v33 rxd_s 4.7k")
    if p["slave_cs_connected"]:
        lines += ["Vscs scs_src grp DC 3.3", "Rscs scs_src cs_s 25"]
    else:
        lines += ["Rcs_pd cs_s vss_s 200k"]   # CS internal pull-down only

    # MCP2003 ground pins
    lines += ["Rvss_m vss_m 0 0.01", f"Rvss_s vss_s 0 {p['slave_vss_ohm']}"]

    # bus
    lines += [f"Rbus lbus_m lbus_s {p['bus_wire_ohm']}",
              f"Cbus_m lbus_m vss_m {p['master_bus_cap']}",
              "Cbus_s lbus_s vss_s 220p"]
    if p["master_pullup"] == "vbb":
        lines += ["Dmp vbb nmp DSIG", "Rmp nmp lbus_m 1k"]
    elif p["master_pullup"] == "gnd":
        lines += ["Rmp lbus_m vss_m 1k"]

    lines.append(mcp2003(2, "vbb", "vss_m", "txd_m", "rxd_m", "cs_m", "lbus_m"))
    lines.append(mcp2003(1, "vbb", "vss_s", "txd_s", "rxd_s", "cs_s", "lbus_s"))

    lines += [".options reltol=1e-3 abstol=1e-9 vntol=1e-5 itl4=200",
              f".tran 0.5u {t_end} 0 0.5u uic",
              ".end"]
    return "\n".join(lines).splitlines()


# ─── ngspice shared library ──────────────────────────────────────────────────
class Ngspice:
    def __init__(self):
        os.add_dll_directory(KICAD_BIN)
        self.lib = ctypes.CDLL(os.path.join(KICAD_BIN, "ngspice.dll"))
        self.log = []
        SendChar = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
        SendStat = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_char_p, ctypes.c_int, ctypes.c_void_p)
        Exit = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_int, ctypes.c_bool, ctypes.c_bool, ctypes.c_int, ctypes.c_void_p)
        BgRun = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_bool, ctypes.c_int, ctypes.c_void_p)
        self._cbs = (SendChar(lambda s, i, d: self.log.append(s.decode(errors="replace")) or 0),
                     SendStat(lambda s, i, d: 0),
                     Exit(lambda *a: 0),
                     BgRun(lambda *a: 0))
        self.lib.ngSpice_Init(self._cbs[0], self._cbs[1], self._cbs[2], None, None, self._cbs[3], None)

    def cmd(self, c):
        self.lib.ngSpice_Command(c.encode())

    def run(self, lines, vectors, out_file):
        self.log.clear()
        arr = (ctypes.c_char_p * (len(lines) + 1))(*[l.encode() for l in lines], None)
        self.lib.ngSpice_Circ(arr)
        self.cmd("run")
        self.cmd("set wr_singlescale")
        self.cmd("set wr_vecnames")
        self.cmd(f"wrdata {out_file} " + " ".join(vectors))
        self.cmd("remcirc")
        self.cmd("destroy all")


# ─── Analysis ────────────────────────────────────────────────────────────────
def load(path):
    with open(path) as f:
        header = f.readline().split()
        rows = [list(map(float, l.split())) for l in f if l.strip()]
    return header, rows


def at(t_arr, v_arr, t):
    lo, hi = 0, len(t_arr) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if t_arr[mid] < t:
            lo = mid + 1
        else:
            hi = mid
    return v_arr[lo]


def decode_uart(t_arr, v_arr, vil, vih, t_start):
    """Decode 8N1 @ BAUD. Returns (bytes, undefined_level_samples). Framing error -> 'FE'."""
    def level(v):
        return 1 if v >= vih else (0 if v <= vil else None)

    out, undefined = [], 0
    t = t_start
    t_last = t_arr[-1] - 10 * BIT
    prev = 1
    step = BIT / 8
    while t < t_last:
        v = level(at(t_arr, v_arr, t))
        if v is None:
            undefined += 1
            v = prev
        if prev == 1 and v == 0:
            bits = []
            for k in range(1, 10):
                s = level(at(t_arr, v_arr, t + (k + 0.5) * BIT - step / 2))
                if s is None:
                    undefined += 1
                    s = 0
                bits.append(s)
            byte = sum(b << i for i, b in enumerate(bits[:8]))
            out.append(f"{byte:02X}" if bits[8] == 1 else f"{byte:02X}!FE")
            t += 9.5 * BIT
            # after a framing error wait for the line to return high
            while t < t_last and level(at(t_arr, v_arr, t)) != 1:
                t += step
            prev = 1
            continue
        prev = v
        t += step
    return out, undefined


def analyse(name, header, rows):
    cols = {h: i for i, h in enumerate(header)}
    t = [r[cols["time"]] for r in rows]

    def col(n):
        return [r[cols[n]] for r in rows]

    idle_t = T_SETTLE * 0.9
    vals = {n: col(n) for n in header if n != "time"}
    vbb_m = at(t, vals["v(vbb)"], idle_t) - at(t, vals["v(vss_m)"], idle_t)
    vbb_s = at(t, vals["v(vbb)"], idle_t) - at(t, vals["v(vss_s)"], idle_t)
    lbus_idle = at(t, vals["v(lbus_s)"], idle_t)
    rxd_m_idle = at(t, vals["v(rxd_m)"], idle_t)
    rxd_s_idle = at(t, vals["v(rp_rx)"], idle_t) - at(t, vals["v(grp)"], idle_t)

    busy = [i for i, x in enumerate(t) if x > T_SETTLE]
    lbus_min = min(vals["v(lbus_m)"][i] - vals["v(vss_m)"][i] for i in busy)
    rxd_m_min = min(vals["v(rxd_m)"][i] for i in busy)

    rp_rx = [a - b for a, b in zip(vals["v(rp_rx)"], vals["v(grp)"])]
    uno_bytes, uno_undef = decode_uart(t, vals["v(rxd_m)"], UNO_VIL, UNO_VIH, T_SETTLE)
    rp_bytes, rp_undef = decode_uart(t, rp_rx, RP_VIL, RP_VIH, T_SETTLE)

    expected_uno = ["55", "3C"] + [f"{b:02X}" for b in RESPONSE]
    uno_tail = [b for b in uno_bytes if "!FE" not in b]
    ok_uno = uno_tail[-6:] == expected_uno[2:]
    ok_rp = "55" in rp_bytes and "3C" in rp_bytes and rp_bytes.index("3C") == rp_bytes.index("55") + 1

    print(f"\n=== {name}")
    print(f"  Idle (Multimeter):  VBB-VSS #2={vbb_m:5.2f}V  #1={vbb_s:5.2f}V   LBUS={lbus_idle:5.2f}V   "
          f"RXD Uno(D0)={rxd_m_idle:4.2f}V   RXD RP2040(GP1)={rxd_s_idle:4.2f}V")
    print(f"  Aktiv:              LBUS min={lbus_min:5.2f}V   RXD Uno min={rxd_m_min:4.2f}V")
    print(f"  Uno empfängt:       {' '.join(uno_bytes) or '-'}" + (f"   ({uno_undef} Samples im verbotenen Pegelbereich)" if uno_undef else ""))
    print(f"  RP2040 empfängt:    {' '.join(rp_bytes) or '-'}" + (f"   ({rp_undef} Samples im verbotenen Pegelbereich)" if rp_undef else ""))
    print(f"  Anfrage beim Slave: {'OK' if ok_rp else 'FEHLT'}    Antwort beim Master: {'OK' if ok_uno else 'FEHLT'}")
    return ok_rp, ok_uno


def main():
    picks = [int(a) for a in sys.argv[1:]] or range(len(SCENARIOS))
    spice = Ngspice()
    _, t_master_end = pwl(master_segments(), 5.0)
    t_end = t_master_end + 150e-6 + 6 * 10 * BIT + 300e-6
    vectors = ["v(vbb)", "v(vss_m)", "v(vss_s)", "v(lbus_m)", "v(lbus_s)", "v(rxd_m)", "v(rp_rx)", "v(grp)"]
    out = os.path.join(tempfile.gettempdir(), "lin_sim_out.txt").replace("\\", "/")
    summary = []
    for i in picks:
        name, over = SCENARIOS[i]
        p = dict(BASE, **over)
        if os.path.exists(out):
            os.remove(out)
        spice.run(netlist(p, t_end), vectors, out)
        if not os.path.exists(out):
            print(f"\n=== [{i}] {name}\n  SIMULATION FAILED")
            print("".join(l + "\n" for l in spice.log if "rror" in l or "arning" in l)[-2000:])
            summary.append((i, name, None, None))
            continue
        header, rows = load(out)
        ok_rp, ok_uno = analyse(f"[{i}] {name}", header, rows)
        summary.append((i, name, ok_rp, ok_uno))

    print("\n=== Zusammenfassung")
    for i, name, ok_rp, ok_uno in summary:
        state = "SIM-FEHLER" if ok_rp is None else ("✅ Kommunikation OK" if ok_rp and ok_uno else
                "❌ " + ", ".join(x for x, ok in (("Anfrage kommt nicht an", ok_rp), ("Antwort kommt nicht an", ok_uno)) if not ok))
        print(f"  [{i:2}] {name:55} {state}")


if __name__ == "__main__":
    main()
