#!/usr/bin/env python3
"""Front harness v4 (2026-10-19): the relay box under the passenger seat switches the
front lights again; they run through X1 (DT 12-pol, 18 AWG). CAN gets its own
connector. The front keeps only the fuse block on X2 (servo, ABS, vacuum pump,
brake switch, mirror switch) and the front controller as a reading CAN node.

Edits the current Front.harness in place (keeps the user's layout) and rebuilds
Gesamt.harness = Heck + Front.
Usage: python3 tools/harness_front_v4.py [dir]
"""
import json
import random
import string
import sys
from pathlib import Path

DIR = Path(sys.argv[1] if len(sys.argv) > 1 else "WiringDiagram/harness.design")

# X1 pins: (signal, wire colour, stripe)
X1_PINS = [
    ("Abblendlicht L", "Blue", None),
    ("Abblendlicht R", "Blue", None),
    ("Fernlicht L", "White", None),
    ("Fernlicht R", "White", None),
    ("Standlicht/TFL L+R", "Red", None),
    ("Nebel L", "Yellow", None),
    ("Nebel R", "Yellow", None),
    ("Blinker L", "Red", "Black"),
    ("Blinker R", "Red", "Black"),
    ("Servolenkung Einschalten (Motor laeuft)", "Black", "White"),
    ("Kl. 30 Front-Controller/Lenkrad/Diagnose", "Red", "White"),
    (None, None, None),
]


def main():
    front = json.load(open(DIR / "Front.harness"))
    heck = json.load(open(DIR / "Heck.harness"))
    used = set()

    def collect(o):
        if isinstance(o, dict):
            if isinstance(o.get("id"), str) and "handle" not in o:
                used.add(o["id"])
            for v in o.values():
                collect(v)
        elif isinstance(o, list):
            for v in o:
                collect(v)

    collect(front)
    collect(heck)
    rnd = random.Random(2026101904)

    def nid():
        while True:
            s = "".join(rnd.choices(string.ascii_letters + string.digits + "_", k=6))
            if s not in used:
                used.add(s)
                return s

    by_label = {}
    for c in front["connectors"]:
        by_label.setdefault(c["label"], []).append(c)
    one = lambda label: by_label[label][0]
    lc_l, lc_r = sorted(by_label["LightConn"], key=lambda c: c["schematicPosition"]["y"])  # top = left
    x1 = one("X1 Front DT12")
    fc = one("Front-Controller (Nucleo)")
    fv_a = one("FV-A Leistung")
    servo = one("Servolenkung (Corsa-C-Kit)")
    obd = one("Diagnosebuchse vorn")
    fog_l, fog_r = one("FogFront1"), one("FogFront2")
    terms = {t["signal"]: t for t in front["terminals"]}

    # --- remove the front light distributor (FV-B), its control side (FV-C) and X2b
    drop_conns = {one("FV-B Licht")["id"], one("FV-C Steuerung")["id"]}
    drop_terms = {terms["X2b Box F31 20 A Kl. 30"]["id"], terms["X2b FV-Bolzen 30 vorn"]["id"]}
    # cavities that disappear: FC Q1..Q7 (5..11), FV-A V9 servo enable (11) and V27 diagnostics (12)
    drop_cavs = {cv["id"] for cv in fc["cavities"][4:11]} | {cv["id"] for cv in fv_a["cavities"][10:12]}
    gone_ids = drop_conns | drop_terms

    def keep(w):
        for end in (w["source"], w["target"]):
            if end["id"] in gone_ids or end["handle"] in drop_cavs:
                return False
            if end["id"] == x1["id"]:
                return False  # X1 is rewired completely
        return True

    front["wires"] = [w for w in front["wires"] if keep(w)]
    front["connectors"] = [c for c in front["connectors"] if c["id"] not in drop_conns]
    front["terminals"] = [t for t in front["terminals"] if t["id"] not in drop_terms]
    fc["cavities"] = [cv for cv in fc["cavities"] if cv["id"] not in drop_cavs]
    fv_a["cavities"] = [cv for cv in fv_a["cavities"] if cv["id"] not in drop_cavs]
    fv_a["label"] = "Front-Sicherungen (X2)"
    fc["cavities"][0]["signal"] = "+12 V Kl. 30 (X1/11)"
    terms["X2 FV-Bolzen 15 vorn"]["signal"] = "X2 Bolzen 15 vorn (Front-Sicherungen)"

    # --- X1: lights etc. from the relay box
    for cav, (sig, _, _) in zip(x1["cavities"], X1_PINS):
        cav.pop("signal", None)
        cav.pop("notConnected", None)
        if sig:
            cav["signal"] = sig
        else:
            cav["notConnected"] = True

    new_conns, new_splices, new_wires = [], [], []

    def pin(c, n):
        return {"id": c["id"], "handle": c["cavities"][n - 1]["id"]}

    def splice(x, y):
        s = {"id": nid(), "schematicPosition": {"x": x, "y": y}}
        new_splices.append(s)
        return {"id": s["id"], "handle": "Splice"}

    def wire(a, b, color, stripe=None):
        w = {"id": nid(), "color": color, "source": a, "target": b}
        if stripe:
            w["stripeColor"] = stripe
        new_wires.append(w)

    def x1wire(n, target):
        _, col, stripe = X1_PINS[n - 1]
        wire(pin(x1, n), target, col, stripe)

    x1wire(1, pin(lc_l, 1))
    x1wire(2, pin(lc_r, 1))
    x1wire(3, pin(lc_l, 2))
    x1wire(4, pin(lc_r, 2))
    sp_park = splice(3900, -300)
    x1wire(5, sp_park)
    wire(sp_park, pin(lc_l, 3), "Red")
    wire(sp_park, pin(lc_r, 3), "Red")
    x1wire(6, pin(fog_l, 1))
    x1wire(7, pin(fog_r, 1))
    x1wire(8, pin(lc_l, 4))
    x1wire(9, pin(lc_r, 4))
    x1wire(10, pin(servo, 3))
    sp_30 = splice(1110, -300)
    x1wire(11, sp_30)
    wire(sp_30, pin(fc, 1), "Red", "White")
    wire(sp_30, pin(obd, 16), "Red", "White")

    # --- CAN on its own connector (type still open)
    can = {"id": nid(), "label": "CAN Front (eigener Stecker)", "schematicPosition": {"x": 0, "y": -600},
           "cavities": [{"id": nid(), "signal": "CAN-H"}, {"id": nid(), "signal": "CAN-L"}]}
    new_conns.append(can)
    wire(pin(can, 1), pin(fc, 3), "Yellow")
    wire(pin(can, 2), pin(fc, 4), "Green")

    front["connectors"] += new_conns
    front["splices"] += new_splices
    front["wires"] += new_wires

    # --- checks: unique ids, every wire end resolves, nothing on N/C cavities
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
