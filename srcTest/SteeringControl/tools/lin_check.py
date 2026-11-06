"""Acceptance run for the LIN chain via the Uno debug master (env uno_master_debug).

Reads the master output (19200 baud), prints every button press, tracks pot/Hall
ranges, Hold, KEY state and gaps between frames (>0.3 s), then prints a summary.
Opening the port resets the Uno, so the first ~3 s are startup noise.

    python tools/lin_check.py                 # COM6, 180 s
    python tools/lin_check.py --port COM5 --seconds 300
"""
import argparse
import re
import time

import serial

LINE_RE = re.compile(r"BTN:(.*?) \| POT1: (\d+).*POT2: (\d+).*H1: (\d+).*H2: (\d+)"
                     r".*HOLD: (\d+)% (\d+)% \| KEY (\S+)")
ANALOG_NAMES = ["POT1", "POT2", "H1", "H2"]
GAP = 0.3


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM6")
    ap.add_argument("--seconds", type=float, default=180)
    args = ap.parse_args()

    ser = serial.Serial(args.port, 19200, timeout=1)
    t0 = time.time()
    prev, counts = set(), {}
    lo, hi, rest = [9999] * 4, [0] * 4, None
    frames = garbled = keyfail = 0
    holdmax = [0, 0]
    last, gaps = t0, []
    try:
        while time.time() - t0 < args.seconds:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode(errors="replace").strip()
            m = LINE_RE.search(line)
            if not m:
                if line and frames:
                    garbled += 1
                    print(f"[{time.time() - t0:6.1f}] ?? {line}", flush=True)
                continue
            now = time.time()
            if frames and now - last > GAP:
                gaps.append(round(now - last, 2))
                print(f"[{now - t0:6.1f}] LUECKE {now - last:.2f}s", flush=True)
            last = now
            frames += 1
            btns = set(m.group(1).split()) - {"-"}
            for b in sorted(btns - prev):
                counts[b] = counts.get(b, 0) + 1
                print(f"[{now - t0:6.1f}] DRUECK {b}", flush=True)
            prev = btns
            v = [int(m.group(i)) for i in range(2, 6)]
            rest = rest or v
            lo = [min(a, b) for a, b in zip(lo, v)]
            hi = [max(a, b) for a, b in zip(hi, v)]
            holdmax = [max(holdmax[0], int(m.group(6))), max(holdmax[1], int(m.group(7)))]
            if m.group(8) != "OK":
                keyfail += 1
    except KeyboardInterrupt:
        pass

    print("\n=== ERGEBNIS ===")
    print(f"Frames {frames}, unlesbar {garbled}, Luecken >{GAP}s {len(gaps)} {gaps[:10]}, KEY nicht OK {keyfail}")
    print("Knoepfe:", ", ".join(f"{k} {n}x" for k, n in sorted(counts.items())) or "keine")
    for i, name in enumerate(ANALOG_NAMES):
        print(f"  {name}: {lo[i]}-{hi[i]} (Ruhe {rest[i] if rest else '-'})")
    print(f"  HOLD max: {holdmax[0]}% {holdmax[1]}%")


if __name__ == "__main__":
    main()
