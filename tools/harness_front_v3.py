#!/usr/bin/env python3
"""Front harness v3 (2026-10-19): front node at the steering column switches the
front lights locally, X1 shrinks to the CAN pair, X2 (16 mm², Kl. 15) and X2b
(2.5 mm², Kl. 30) feed the front distributor (FV). Adds ABS T25, servo steering,
vacuum pump circuit (changeover relay, low side), brake switch, mirror switch,
diagnostic socket and the ABS rear sensor pass-through X6.

Rewrites Front.harness in place and rebuilds Gesamt.harness = Heck + Front.
Usage: python3 tools/harness_front_v3.py [dir]
"""
import json
import random
import string
import sys
from pathlib import Path

DIR = Path(sys.argv[1] if len(sys.argv) > 1 else "WiringDiagram/harness.design")


def main():
    front = json.load(open(DIR / "Front.harness"))
    heck = json.load(open(DIR / "Heck.harness"))
    used = set()

    def collect(o):
        if isinstance(o, dict):
            if isinstance(o.get("id"), str):
                used.add(o["id"])
            for v in o.values():
                collect(v)
        elif isinstance(o, list):
            for v in o:
                collect(v)

    collect(front)
    collect(heck)
    rnd = random.Random(2026101903)

    def nid():
        while True:
            s = "".join(rnd.choices(string.ascii_letters + string.digits + "_", k=6))
            if s not in used:
                used.add(s)
                return s

    conns = {c["label"]: c for c in front["connectors"] if c["label"] != "LightConn"}
    light_conn = sorted((c for c in front["connectors"] if c["label"] == "LightConn"),
                        key=lambda c: c["schematicPosition"]["y"])
    lc_l, lc_r = light_conn  # y -1860 = left, y -630 = right
    x1 = conns["X1 Front DT12"]
    m1 = next(t for t in front["terminals"] if t["signal"] == "M1 Rahmen vorn links")
    m2 = next(t for t in front["terminals"] if t["signal"] == "M2 Rahmen vorn rechts")

    # --- drop the v2 X1 wiring (lights via X1 + L/R splices, Zusatz-Fernlicht on pin 7)
    x1_split = {"18Sj7G", "_lDGbD", "MuH8r0", "5FcPAR"}
    gone = x1_split | {x1["id"]}
    front["wires"] = [w for w in front["wires"]
                      if w["source"]["id"] not in gone and w["target"]["id"] not in gone and w["id"] != "t9YvvK"]
    front["splices"] = [s for s in front["splices"] if s["id"] not in x1_split]

    # --- X1: CAN only, rest reserve
    for i, cav in enumerate(x1["cavities"]):
        cav.pop("signal", None)
        cav.pop("notConnected", None)
        if i == 0:
            cav["signal"] = "CAN-H"
        elif i == 1:
            cav["signal"] = "CAN-L"
        else:
            cav["notConnected"] = True

    new_conns, new_terms, new_splices, new_wires = [], [], [], []

    def connector(label, x, y, pins):
        """pins: list of signal names; None = not connected (reserve / unused pin)."""
        cavs = []
        for sig in pins:
            cav = {"id": nid()}
            if sig is None:
                cav["notConnected"] = True
            elif sig:
                cav["signal"] = sig
            cavs.append(cav)
        c = {"id": nid(), "label": label, "schematicPosition": {"x": x, "y": y}, "cavities": cavs}
        new_conns.append(c)
        return c

    def ring(signal, x, y):
        t = {"id": nid(), "schematicPosition": {"x": x, "y": y}, "signal": signal, "type": "Ring"}
        new_terms.append(t)
        return t

    def splice(x, y):
        s = {"id": nid(), "schematicPosition": {"x": x, "y": y}}
        new_splices.append(s)
        return {"id": s["id"], "handle": "Splice"}

    def pin(c, n):  # 1-based pin number = cavity order
        return {"id": c["id"], "handle": c["cavities"][n - 1]["id"]}

    def term(t):
        return {"id": t["id"], "handle": "Terminal"}

    def wire(color, a, b, stripe=None):
        w = {"id": nid(), "color": color, "source": a, "target": b}
        if stripe:
            w["stripeColor"] = stripe
        new_wires.append(w)

    # --- devices (harness-side connectors; group them per device in the app)
    fc = connector("Front-Controller (Nucleo)", 0, 0, [
        "+12 V Kl. 30 (V20 3 A)", "Masse", "", "",
        "Q1 Spule K4 Servo ein", "Q2 Spule K5 Abblend", "Q3 Spule K6 Fern", "Q4 Spule K7 Nebel",
        "Q5 Spule K8 Stand/TFL", "Q6 Spule K9 Blinker L", "Q7 Spule K10 Blinker R",
        "E1 Pumpe laeuft (Pull-up 1k)", "E2 Bremslicht (Teiler 10k/2k7)", "E3 ABS-Lampe"])
    fv_c = connector("FV-C Steuerung", 600, 0, [
        "K4 Spule -", "K5 Spule -", "K6 Spule -", "K7 Spule -", "K8 Spule -", "K9 Spule -", "K10 Spule -",
        "V20 Front-Controller 3 A", "V21 Lenkrad 1 A"])
    fv_a = connector("FV-A Leistung", 600, 420, [
        "V1 Servo +12 V 50 A (10 mm2)", "V2 ABS 30 A (4 mm2)", "V3 ABS 30 A (4 mm2)", "V4 ABS Zuendung 5 A",
        "V5 Pumpe +12 V 20 A (2,5 mm2)", "K3/87 Pumpe - (Low-Side)", "K3/87a Pumpe-Status",
        "K3 Spule - (V6 3 A)", "V7 Bremslichtschalter 3 A", "V8 Spiegelschalter 5 A",
        "V9 Servo Einschalten (K4)", "V27 Diagnose Kl. 30 5 A", "K3/30 Masse (2,5 mm2)"])
    fv_b = connector("FV-B Licht", 600, 900, [
        "V10 Abblend L 5 A", "V11 Abblend R 5 A", "V12 Fern L 5 A", "V13 Fern R 5 A",
        "V22 Stand/TFL L 3 A", "V23 Stand/TFL R 3 A", "V24 Blinker L 5 A", "V25 Blinker R 5 A",
        "V14 Nebel L 5 A", "V15 Nebel R 5 A"])
    servo = connector("Servolenkung (Corsa-C-Kit)", 1500, 0, ["", "Masse (10 mm2)", ""])
    t25_sig = {1: "G44 HR", 2: "G46 HL", 3: "G45 VR", 4: "G47 VL", 7: "Bruecke 7-22", 8: "Masse (4 mm2)",
               9: "", 10: "G46 HL", 11: "G47 VL", 12: "", 13: "K-Leitung", 16: "ABS-Lampe",
               17: "G44 HR", 18: "G45 VR", 22: "Bruecke 7-22", 23: "", 24: "Masse (4 mm2)", 25: ""}
    abs_ = connector("ABS J104 T25", 1500, 240, [t25_sig.get(n) for n in range(1, 26)])
    pump = connector("Unterdruckpumpe", 1500, 1140, ["", ""])
    s1 = connector("Unterdruckschalter S1", 1500, 1290, ["", "Masse"])
    brk = connector("Bremslichtschalter F", 1500, 1440, ["", "Bremslicht-Signal"])
    obd_sig = {4: "Masse", 7: "", 16: ""}
    obd = connector("Diagnosebuchse vorn", 1500, 1590, [obd_sig.get(n) for n in range(1, 17)])
    e43 = connector("Spiegelschalter E43", 1500, 2190, [
        "Spiegel L Sammel (K1)", "Beleuchtung 58b (optional)", "Spiegel R Sammel (K3)", "Spiegel L+R (K4)",
        "Spiegel L (K5)", "", "Spiegel R (K7)", "Masse"])
    e43["cavities"][1].pop("signal")
    e43["cavities"][1]["notConnected"] = True
    g47 = connector("ABS-Fuehler G47 VL", 2400, 240, ["", ""])
    g45 = connector("ABS-Fuehler G45 VR", 2400, 390, ["", ""])
    x6 = connector("X6 ABS-Fuehler hinten", 2400, 540, ["G46 HL a", "G46 HL b", "G44 HR a", "G44 HR b"])
    t5 = connector("Spiegel L T5", 2400, 2190, ["", "", "", "Heizung + (spaeter)", "Heizung - (spaeter)"])
    t5a = connector("Spiegel R T5a", 2400, 2430, ["", "", "", "Heizung + (spaeter)", "Heizung - (spaeter)"])
    box15 = ring("X2 Box F30 80 A Kl. 15", -600, 900)
    fv15 = ring("X2 FV-Bolzen 15 vorn", 90, 900)
    box30 = ring("X2b Box F31 20 A Kl. 30", -600, 990)
    fv30 = ring("X2b FV-Bolzen 30 vorn", 90, 990)

    # --- supplies
    wire("Red", term(box15), term(fv15))                  # 16 mm²
    wire("Red", term(box30), term(fv30), "White")         # 2.5 mm²
    # --- CAN + front controller
    wire("Yellow", pin(x1, 1), pin(fc, 3))
    wire("Green", pin(x1, 2), pin(fc, 4))
    wire("Red", pin(fv_c, 8), pin(fc, 1), "White")
    wire("Brown", pin(fc, 2), term(m1))
    for k in range(7):
        wire("Violet", pin(fv_c, 1 + k), pin(fc, 5 + k))
    # --- servo steering
    wire("Red", pin(fv_a, 1), pin(servo, 1))
    wire("Brown", pin(servo, 2), term(m1))
    wire("Black", pin(fv_a, 11), pin(servo, 3), "White")
    # --- ABS
    wire("Red", pin(fv_a, 2), pin(abs_, 9))
    wire("Red", pin(fv_a, 3), pin(abs_, 25))
    wire("Black", pin(fv_a, 4), pin(abs_, 23), "Yellow")
    wire("Brown", pin(abs_, 8), term(m1))
    wire("Brown", pin(abs_, 24), term(m1))
    wire("Blue", pin(abs_, 7), pin(abs_, 22))
    wire("Gray", pin(abs_, 16), pin(fc, 14), "White")
    wire("Gray", pin(abs_, 13), pin(obd, 7), "Blue")
    wire("Gray", pin(abs_, 11), pin(g47, 1))
    wire("Light Gray", pin(abs_, 4), pin(g47, 2))
    wire("Gray", pin(abs_, 18), pin(g45, 1))
    wire("Light Gray", pin(abs_, 3), pin(g45, 2))
    wire("Gray", pin(abs_, 2), pin(x6, 1))
    wire("Light Gray", pin(abs_, 10), pin(x6, 2))
    wire("Gray", pin(abs_, 1), pin(x6, 3))
    wire("Light Gray", pin(abs_, 17), pin(x6, 4))
    # --- vacuum pump: changeover relay K3 switches ground, NC contact = status for the controller
    wire("Red", pin(fv_a, 5), pin(pump, 1), "Green")
    wire("Brown", pin(pump, 2), pin(fv_a, 6), "White")
    wire("Green", pin(fv_a, 7), pin(fc, 12), "White")
    wire("Green", pin(fv_a, 8), pin(s1, 1))
    wire("Brown", pin(s1, 2), term(m1))
    wire("Brown", pin(fv_a, 13), term(m1))
    # --- brake switch -> ABS T25/12 + controller E2 (CAN 0x0405 to the rear)
    wire("Black", pin(fv_a, 9), pin(brk, 1), "Red")
    sp_brk = splice(1200, 1470)
    wire("Black", pin(brk, 2), sp_brk, "Red")
    wire("Black", sp_brk, pin(abs_, 12), "Red")
    wire("Black", sp_brk, pin(fc, 13), "Red")
    # --- diagnostic socket
    wire("Red", pin(fv_a, 12), pin(obd, 16))
    wire("Brown", pin(obd, 4), term(m1))
    # --- mirrors: switch fed from FV, motors switch-to-mirror only
    wire("Black", pin(fv_a, 10), pin(e43, 6), "Blue")
    wire("Brown", pin(e43, 8), term(m1))
    wire("Green", pin(e43, 5), pin(t5, 1))
    wire("Yellow", pin(e43, 1), pin(t5, 2))
    sp_k4 = splice(2100, 2280)
    wire("Violet", pin(e43, 4), sp_k4)
    wire("Violet", sp_k4, pin(t5, 3))
    wire("Violet", sp_k4, pin(t5a, 3))
    wire("Green", pin(e43, 7), pin(t5a, 1))
    wire("Yellow", pin(e43, 3), pin(t5a, 2))
    # --- lights: per-side fuses in FV, no splices
    lamp = [("Blue", None), ("White", None), ("Red", None), ("Red", "Black")]
    for i, (col, stripe) in enumerate(lamp):
        a, b = (2 * i + 1, 2 * i + 2)
        wire(col, pin(fv_b, a), pin(lc_l, i + 1), stripe)
        wire(col, pin(fv_b, b), pin(lc_r, i + 1), stripe)
    wire("Yellow", pin(fv_b, 9), pin(conns["FogFront1"], 1))
    wire("Yellow", pin(fv_b, 10), pin(conns["FogFront2"], 1))

    front["connectors"] += new_conns
    front["terminals"] += new_terms
    front["splices"] += new_splices
    front["wires"] += new_wires
    front["titleBlock"] = {"title": "Buggy Frontkabelbaum", "date": "2026-10-19", "drawnBy": "TheRealJourage + Claude"}

    ids = []

    def all_ids(o):
        if isinstance(o, dict):
            if isinstance(o.get("id"), str) and "handle" not in o:
                ids.append(o["id"])
            for v in o.values():
                all_ids(v)
        elif isinstance(o, list):
            for v in o:
                all_ids(v)

    all_ids(front)
    all_ids(heck)
    assert len(ids) == len(set(ids)), "duplicate ids"
    known = {c["id"]: {cv["id"] for cv in c["cavities"]} for c in front["connectors"]}
    known.update({t["id"]: {"Terminal"} for t in front["terminals"]})
    known.update({s["id"]: {"Splice"} for s in front["splices"]})
    nc = {cv["id"] for c in front["connectors"] for cv in c["cavities"] if cv.get("notConnected")}
    for w in front["wires"]:
        for end in (w["source"], w["target"]):
            assert end["handle"] in known.get(end["id"], set()), f"dangling {w['id']}"
            assert end["handle"] not in nc, f"wire on N/C cavity {w['id']}"

    gesamt = {k: v for k, v in front.items() if k not in ("connectors", "terminals", "splices", "wires", "titleBlock")}
    gesamt["titleBlock"] = {"title": "Buggy Gesamt (Heck + Front)", "date": "2026-10-19", "drawnBy": "TheRealJourage + Claude"}
    for k in ("connectors", "terminals", "splices", "wires"):
        items = heck.get(k, []) + front.get(k, [])
        if items:
            gesamt[k] = items

    for name, d in (("Front", front), ("Gesamt", gesamt)):
        p = DIR / f"{name}.harness"
        tmp = p.with_suffix(".tmp")
        tmp.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
        tmp.replace(p)
        print(f"{p}: {len(d.get('connectors', []))} connectors, {len(d.get('terminals', []))} terminals, "
              f"{len(d.get('splices', []))} splices, {len(d.get('wires', []))} wires")


if __name__ == "__main__":
    main()
