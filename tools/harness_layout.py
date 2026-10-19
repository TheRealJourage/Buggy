#!/usr/bin/env python3
"""Arrange the schematic of Front.harness like the vehicle seen from above:
rear/box on the left, front on the right, vehicle-left at the top.
Idempotent: only schematicPosition values change. Rebuilds Gesamt.harness = Heck + Front
(the rear harness keeps its own positions at negative x).

Usage: python3 tools/harness_layout.py [dir]
"""
import json
import sys
from pathlib import Path

DIR = Path(sys.argv[1] if len(sys.argv) > 1 else "WiringDiagram/harness.design")

# columns (x) from the box interface (left) to the lamps (right), 30-unit grid
A, B, C, D, E, F, G, H = 0, 750, 1500, 2400, 3300, 4200, 4500, 4800

CONNECTORS = {
    # A: interface to the box / rear
    "X1 Front DT12": (A, -240),
    "CAN Front (eigener Stecker)": (A, -600),
    "X6 ABS-Fuehler hinten": (A, 720),
    # B: middle of the vehicle (behind the cockpit)
    "HighBeamEx2": (B, -1500),
    "HighBeamEx1": (B, -1290),
    "OffroadFront1": (B, 840),
    "OffroadFront2": (B, 1050),
    # C: cockpit / steering column
    "Spiegel L T5": (C, -1800),
    "Front-Controller (Nucleo)": (C, -1200),
    "Diagnosebuchse vorn": (C, -600),
    "Spiegelschalter E43": (C, 60),
    "Bremslichtschalter F": (C, 480),
    "Spiegel R T5a": (C, 1500),
    # D: front fuse block on X2
    "Front-Sicherungen (X2)": (D, -600),
    # E: front units
    "ABS-Fuehler G47 VL": (E, -1800),
    "Servolenkung Hauptstrom (Corsa B/C, BoostWorks)": (E, -1500),
    "ABS J104 T25": (E, -1200),
    "Unterdruckpumpe (Hella UP28)": (E, -240),
    "Unterdruckschalter S1": (E, -60),
    "ABS-Fuehler G45 VR": (E, 1500),
    # F/G: lamps, left at the top, right at the bottom
    "FrontLightLeft": (G, -1650),
    "FogFront1": (G, -1200),
    "FogFront2": (G, 900),
    "FrontLightRight": (G, 1350),
}
# LightConn appears twice; distinguished by current y order (lower y = left)
LIGHT_CONN = [(F, -1800), (F, 1200)]

TERMINALS = {
    "X2 Box F30 80 A Kl. 15": (A, 360),
    "RingHighBeamEx GND": (B - 390, -1380),
    "RingDriverLeft GND": (B - 390, 900),
    "X2 Bolzen 15 vorn (Front-Sicherungen)": (D, -1260),
    "M1 Rahmen vorn links": (F, -1950),
    "M2 Rahmen vorn rechts": (F, 1500),
    "RingFogL GND": (H, -1200),
    "RingFogR GND": (H, 900),
}
# two plain "GND" rings (lamp pigtails); lower y = left
GND_RINGS = [(G, -1860), (G, 1290)]


def main():
    front = json.load(open(DIR / "Front.harness"))
    heck = json.load(open(DIR / "Heck.harness"))
    missing = []

    for c in front["connectors"]:
        if c["label"] == "LightConn":
            continue
        pos = CONNECTORS.get(c["label"])
        if pos is None:
            missing.append(c["label"])
            continue
        c["schematicPosition"] = {"x": pos[0], "y": pos[1]}
    lcs = sorted((c for c in front["connectors"] if c["label"] == "LightConn"),
                 key=lambda c: c["schematicPosition"]["y"])
    for c, pos in zip(lcs, LIGHT_CONN):
        c["schematicPosition"] = {"x": pos[0], "y": pos[1]}

    gnds = sorted((t for t in front["terminals"] if t["signal"] == "GND"),
                  key=lambda t: t["schematicPosition"]["y"])
    for t, pos in zip(gnds, GND_RINGS):
        t["schematicPosition"] = {"x": pos[0], "y": pos[1]}
    for t in front["terminals"]:
        if t["signal"] == "GND":
            continue
        pos = TERMINALS.get(t["signal"])
        if pos is None:
            missing.append(t["signal"])
            continue
        t["schematicPosition"] = {"x": pos[0], "y": pos[1]}

    # splices: place next to the component they serve (first non-splice neighbour)
    by_id = {c["id"]: c for c in front["connectors"]}
    for s in front["splices"]:
        neigh = []
        for w in front["wires"]:
            for a, b in ((w["source"], w["target"]), (w["target"], w["source"])):
                if a["id"] == s["id"] and b["id"] in by_id:
                    neigh.append(by_id[b["id"]])
        if not neigh:
            missing.append(f"splice {s['id']}")
            continue
        # leftmost neighbour, then 390 to the left of it, vertically on its first row
        # the box interface column never hosts splices (Gesamt has the rear harness left of it)
        inner = [c for c in neigh if c["schematicPosition"]["x"] > A] or neigh
        n = min(inner, key=lambda c: c["schematicPosition"]["x"])
        x = n["schematicPosition"]["x"] - 390
        y = n["schematicPosition"]["y"] + 60
        while any(o["schematicPosition"] == {"x": x, "y": y} for o in front["splices"] if o is not s):
            y += 60
        s["schematicPosition"] = {"x": x, "y": y}

    if missing:
        print("WARNING, no layout rule for:", ", ".join(missing))

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
