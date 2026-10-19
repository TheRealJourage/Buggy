#!/usr/bin/env python3
"""Front harness v5 (2026-10-19): X1 pinout as crimped by the user.
1 Abblend L+R, 2 Fern L+R, 3 Stand/TFL L+R, 4 Blinker R, 5 Blinker L, 6 Nebel L+R (LED),
7 Servolenkung Einschalten, 8 Kl. 30 vorn, 9-12 Reserve. L+R pins split in the front harness.

Edits the current Front.harness in place; run tools/harness_layout.py afterwards
(places the new splices and rebuilds Gesamt.harness).
Usage: python3 tools/harness_front_v5.py [dir]
"""
import json
import random
import string
import sys
from pathlib import Path

DIR = Path(sys.argv[1] if len(sys.argv) > 1 else "WiringDiagram/harness.design")

X1_PINS = [
    ("Abblendlicht L+R", "Blue", None),
    ("Fernlicht L+R", "White", None),
    ("Standlicht/TFL L+R", "Red", None),
    ("Blinker R", "Red", "Black"),
    ("Blinker L", "Red", "Black"),
    ("Nebel L+R", "Yellow", None),
    ("Servolenkung Einschalten (Motor laeuft)", "Black", "White"),
    ("Kl. 30 Front-Controller/Lenkrad/Diagnose", "Red", "White"),
    (None, None, None), (None, None, None), (None, None, None), (None, None, None),
]


def main():
    path = DIR / "Front.harness"
    front = json.load(open(path))
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
    rnd = random.Random(2026101905)

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
    servo = one("Servolenkung (Corsa-C-Kit)")
    obd = one("Diagnosebuchse vorn")
    fog_l, fog_r = one("FogFront1"), one("FogFront2")

    # drop every wire on X1 and the splices that hung off it (v4: Stand split, Kl. 30 split)
    x1_splices = set()
    for w in front["wires"]:
        ends = (w["source"], w["target"])
        if any(e["id"] == x1["id"] for e in ends):
            x1_splices |= {e["id"] for e in ends if e["handle"] == "Splice"}
    front["wires"] = [w for w in front["wires"]
                      if x1["id"] not in (w["source"]["id"], w["target"]["id"])
                      and not ({w["source"]["id"], w["target"]["id"]} & x1_splices)]
    front["splices"] = [s for s in front["splices"] if s["id"] not in x1_splices]

    for cav, (sig, _, _) in zip(x1["cavities"], X1_PINS):
        cav.pop("signal", None)
        cav.pop("notConnected", None)
        if sig:
            cav["signal"] = sig
        else:
            cav["notConnected"] = True

    def pin(c, n):
        return {"id": c["id"], "handle": c["cavities"][n - 1]["id"]}

    def splice():
        s = {"id": nid(), "schematicPosition": {"x": 3810, "y": 0}}  # moved by harness_layout.py
        front["splices"].append(s)
        return {"id": s["id"], "handle": "Splice"}

    def wire(a, b, color, stripe=None):
        w = {"id": nid(), "color": color, "source": a, "target": b}
        if stripe:
            w["stripeColor"] = stripe
        front["wires"].append(w)

    def x1_to(n, *targets):
        _, col, stripe = X1_PINS[n - 1]
        if len(targets) == 1:
            wire(pin(x1, n), targets[0], col, stripe)
            return
        sp = splice()
        wire(pin(x1, n), sp, col, stripe)
        for t in targets:
            wire(sp, t, col, stripe)

    x1_to(1, pin(lc_l, 1), pin(lc_r, 1))
    x1_to(2, pin(lc_l, 2), pin(lc_r, 2))
    x1_to(3, pin(lc_l, 3), pin(lc_r, 3))
    x1_to(4, pin(lc_r, 4))
    x1_to(5, pin(lc_l, 4))
    x1_to(6, pin(fog_l, 1), pin(fog_r, 1))
    x1_to(7, pin(servo, 3))
    x1_to(8, pin(fc, 1), pin(obd, 16))
    fc["cavities"][0]["signal"] = "+12 V Kl. 30 (X1/8)"

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

    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(front, indent=2, ensure_ascii=False) + "\n")
    tmp.replace(path)
    print(f"{path}: {len(front['connectors'])} connectors, {len(front['splices'])} splices, {len(front['wires'])} wires")


if __name__ == "__main__":
    main()
