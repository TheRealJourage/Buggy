#!/usr/bin/env python3
"""Split BuggyElectrical.harness into Heck / Front / Gesamt and add the front
interface connector X1 (Deutsch DT 12-pol), L/R splices and the ground studs
M1/M2 as planned in viewer/hochstrom-front.html.

Usage: python3 tools/harness_split.py [src.harness] [outdir]
"""
import json
import random
import string
import sys
from copy import deepcopy
from pathlib import Path

SRC = Path(sys.argv[1] if len(sys.argv) > 1 else "WiringDiagram/harness.design/BuggyElectrical.harness")
OUT = Path(sys.argv[2] if len(sys.argv) > 2 else "WiringDiagram/harness.design")

REAR_LABELS = {"OffroadRearLeft", "ReversingExLeft", "RearLightLeft", "RearLightRight",
               "ReversingExRight", "OffroadRearRight", "RearHarnessCon"}
REAR_TERMINAL_SIGNALS = {"RingRL GND", "RingRR GND", "StudRL GND", "StudRR GND"}

# X1 pins: (signal, wire colour, stripe) – colours = Front-Seite Ader from the plan
X1_PINS = [
    ("Abblendlicht L+R", "Blue", None),
    ("Fernlicht L+R", "White", None),
    ("Standlicht/TFL L+R", "Red", None),
    ("Blinker L", "Red", "Black"),
    ("Blinker R", "Red", "Black"),
    ("Nebelscheinwerfer L+R", "Yellow", None),
    ("Zusatz-Fernlicht", "White", "Black"),
    ("Servolenkung Zuendung", "Black", "White"),
    ("Bremslicht", "Black", "Red"),
    ("ABS-Warnlampe", "Gray", "White"),
    ("ABS K-Leitung", "Gray", "Blue"),
    ("Unterdruck-Warnung", "Green", "Brown"),
]
# lamp pigtail colours (Superseal 4-pol, Bordnetz-Konzept 2026-10-18)
LAMP_COLORS = {"LowBeam": ("Blue", None), "HighBeam": ("White", None),
               "DrivngLight": ("Red", None), "Indicator": ("Red", "Black")}


def main():
    doc = json.load(open(SRC))
    used = set()

    def collect(o):
        if isinstance(o, dict):
            if "id" in o and isinstance(o["id"], str):
                used.add(o["id"])
            for v in o.values():
                collect(v)
        elif isinstance(o, list):
            for v in o:
                collect(v)

    collect(doc)
    rnd = random.Random(20261019)

    def nid():
        while True:
            s = "".join(rnd.choices(string.ascii_letters + string.digits + "_", k=6))
            if s not in used:
                used.add(s)
                return s

    conns = {c["label"]: c for c in doc["connectors"] if c["label"] not in ("LightConn",)}
    light_conn = [c for c in doc["connectors"] if c["label"] == "LightConn"]
    light_conn_l = min(light_conn, key=lambda c: c["schematicPosition"]["y"])  # y -1860 = left
    light_conn_r = max(light_conn, key=lambda c: c["schematicPosition"]["y"])
    terms = doc["terminals"]
    stud_l = next(t for t in terms if t["signal"] == "StudLightLeft GND")
    stud_r = next(t for t in terms if t["signal"] == "StudLightRight GND")
    stud_l["signal"] = "M1 Rahmen vorn links"
    stud_r["signal"] = "M2 Rahmen vorn rechts"
    ring_driver_left = next(t for t in terms if t["signal"] == "RingDriverLeft GND")

    # --- recolour lamp wires to the real pigtail colours
    for w in doc["wires"]:
        for end in (w["source"], w["target"]):
            for c in (conns.get("FrontLightLeft"), conns.get("FrontLightRight")):
                if c and end["id"] == c["id"]:
                    cav = next(cv for cv in c["cavities"] if cv["id"] == end["handle"])
                    key = cav["signal"].split(" ")[0]
                    if key in LAMP_COLORS:
                        w["color"], stripe = LAMP_COLORS[key]
                        if stripe:
                            w["stripeColor"] = stripe
                        else:
                            w.pop("stripeColor", None)

    new_connectors, new_terminals, new_splices, new_wires = [], [], [], []

    def wire(color, src, dst, stripe=None):
        w = {"id": nid(), "color": color, "source": src, "target": dst}
        if stripe:
            w["stripeColor"] = stripe
        new_wires.append(w)

    def cav_ep(conn, index):
        return {"id": conn["id"], "handle": conn["cavities"][index]["id"]}

    def term_ep(t):
        return {"id": t["id"], "handle": "Terminal"}

    def splice_ep(s):
        return {"id": s["id"], "handle": "Splice"}

    def splice(x, y):
        s = {"id": nid(), "schematicPosition": {"x": x, "y": y}}
        new_splices.append(s)
        return s

    def ring(signal, x, y):
        t = {"id": nid(), "schematicPosition": {"x": x, "y": y}, "signal": signal, "type": "Ring"}
        new_terminals.append(t)
        return t

    # --- X1
    x1 = {"id": nid(), "label": "X1 Front DT12", "schematicPosition": {"x": 3300, "y": -1320},
          "cavities": [{"id": nid(), "signal": sig} for sig, _, _ in X1_PINS]}
    new_connectors.append(x1)

    fl, fr = conns["FrontLightLeft"], conns["FrontLightRight"]
    fog1, fog2 = conns["FogFront1"], conns["FogFront2"]

    # pins 1-3: one wire from X1 to a splice, from there to LightConn L + R
    for pin, (lc_idx) in ((0, 0), (1, 1), (2, 2)):
        sig, col, stripe = X1_PINS[pin]
        s = splice(3900, -1500 + pin * 90)
        wire(col, cav_ep(x1, pin), splice_ep(s), stripe)
        wire(col, splice_ep(s), cav_ep(light_conn_l, lc_idx), stripe)
        wire(col, splice_ep(s), cav_ep(light_conn_r, lc_idx), stripe)
    # pins 4/5: indicators direct
    wire("Red", cav_ep(x1, 3), cav_ep(light_conn_l, 3), "Black")
    wire("Red", cav_ep(x1, 4), cav_ep(light_conn_r, 3), "Black")
    # pin 6: fog lights via splice
    s_fog = splice(3900, -1140)
    wire("Yellow", cav_ep(x1, 5), splice_ep(s_fog))
    wire("Yellow", splice_ep(s_fog), cav_ep(fog1, 0))
    wire("Yellow", splice_ep(s_fog), cav_ep(fog2, 0))
    # pin 7: Zusatz-Fernlicht -> existing 12V splice of HighBeamEx (S2 = the one wired to cavity 1 "12V")
    hb1 = conns["HighBeamEx1"]
    s2_id = next(w["source"]["id"] if w["source"]["handle"] == "Splice" else w["target"]["id"]
                 for w in doc["wires"]
                 if {w["source"]["id"], w["target"]["id"]} & {hb1["id"]}
                 and "Splice" in (w["source"]["handle"], w["target"]["handle"])
                 and hb1["cavities"][0]["id"] in (w["source"]["handle"], w["target"]["handle"]))
    s1_id = next(w["source"]["id"] if w["source"]["handle"] == "Splice" else w["target"]["id"]
                 for w in doc["wires"]
                 if {w["source"]["id"], w["target"]["id"]} & {hb1["id"]}
                 and "Splice" in (w["source"]["handle"], w["target"]["handle"])
                 and hb1["cavities"][1]["id"] in (w["source"]["handle"], w["target"]["handle"]))
    wire("White", cav_ep(x1, 6), {"id": s2_id, "handle": "Splice"}, "Black")

    # --- grounds: fog L -> M1, fog R -> M2, Zusatz-Fernlicht (S1) -> M2, Offroad front ring -> M1
    r_fog_l = ring("RingFogL GND", 5100, -1320)
    r_fog_r = ring("RingFogR GND", 5100, -1140)
    r_hbex = ring("RingHighBeamEx GND", -600, -960)
    wire("Brown", cav_ep(fog1, 1), term_ep(r_fog_l))
    wire("Brown", cav_ep(fog2, 1), term_ep(r_fog_r))
    wire("Brown", {"id": s1_id, "handle": "Splice"}, term_ep(r_hbex))
    wire("Black", term_ep(r_fog_l), term_ep(stud_l))
    wire("Black", term_ep(r_fog_r), term_ep(stud_r))
    wire("Black", term_ep(r_hbex), term_ep(stud_r))
    wire("Black", term_ep(ring_driver_left), term_ep(stud_l))

    # --- split
    rear_conn_ids = {c["id"] for c in doc["connectors"] if c["label"] in REAR_LABELS}
    rear_term_ids = {t["id"] for t in doc["terminals"] if t["signal"] in REAR_TERMINAL_SIGNALS}
    rear_ids = rear_conn_ids | rear_term_ids

    def is_rear_wire(w):
        return w["source"]["id"] in rear_ids or w["target"]["id"] in rear_ids

    base = {k: v for k, v in doc.items() if k not in ("wires", "connectors", "terminals", "splices")}

    def build(title, connectors, terminals, splices, wires):
        d = deepcopy(base)
        d["titleBlock"] = {"title": title, "date": "2026-10-19", "drawnBy": "TheRealJourage + Claude"}
        if connectors: d["connectors"] = connectors
        if terminals: d["terminals"] = terminals
        if splices: d["splices"] = splices
        if wires: d["wires"] = wires
        return d

    all_conns = doc["connectors"] + new_connectors
    all_terms = doc["terminals"] + new_terminals
    all_splices = doc["splices"] + new_splices
    all_wires = doc["wires"] + new_wires

    heck = build("Buggy Heckkabelbaum",
                 [c for c in all_conns if c["id"] in rear_ids],
                 [t for t in all_terms if t["id"] in rear_ids],
                 [], [w for w in all_wires if is_rear_wire(w)])
    front = build("Buggy Frontkabelbaum",
                  [c for c in all_conns if c["id"] not in rear_ids],
                  [t for t in all_terms if t["id"] not in rear_ids],
                  all_splices, [w for w in all_wires if not is_rear_wire(w)])
    gesamt = build("Buggy Gesamt (Heck + Front)", all_conns, all_terms, all_splices, all_wires)

    for name, d in (("Heck", heck), ("Front", front), ("Gesamt", gesamt)):
        p = OUT / f"{name}.harness"
        p.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
        print(f"{p}: {len(d.get('connectors', []))} connectors, {len(d.get('terminals', []))} terminals, "
              f"{len(d.get('splices', []))} splices, {len(d.get('wires', []))} wires")


if __name__ == "__main__":
    main()
