"""Clutch pedal switch: printed tunnel block with two micro switches and a sliding pin, parametric.

Concept (user 2026-10-31): Vault Buggy/Funktionsliste.md block 5 "Kupplung". The pin is fixed to the
pedal and slides in the tunnel; S1 trips shortly after the pedal leaves its rest, S2 only near full
travel. None = engaged, S1 only = moving, S1 + S2 = disengaged.
Switch: Lofair M102-011 (Omron SS-5 size), measured by the user 2026-10-31 (Vault Teile-Bestand.md).
Coordinates: X = pin travel (pedal pushes towards +X), Y across, Z up; tunnel floor at Z = 0.
The switches sit on top of the tunnel, lever tip facing the incoming pin, hinge towards +X.
Run with ~/.venvs/cad/bin/python -I tools/clutch_switch.py
"""
import json
import math
from pathlib import Path

from build123d import (Align, Axis, Box, BuildPart, BuildSketch, Cylinder, Locations, Mode, Plane, Polygon,
                       extrude, export_step, export_stl)

# --- switch (M102-011, measured) ---
SW_L, SW_H, SW_T = 19.8, 10.2, 6.4
SW_HOLE_D, SW_HOLE_PITCH, SW_HOLE_UP = 2.35, 9.5, 7.55   # hole centre above the plunger edge
LEVER_REST, LEVER_TRIP = 4.2, 1.0                        # lever tip below the plunger edge
# --- pin and tunnel ---
PIN = 8.0              # square pin, cannot turn
CLR = 0.25             # per side
TUN = PIN + 2 * CLR
GAP = 0.6              # plunger edge above the pin top -> lever pushed to 0.6, 0.4 beyond the trip point
CHAMFER_DEG = 35.0
WALL, FLOOR, CEIL = 3.0, 3.0, 2.0
# --- travel (hypothesis: pin on the pedal ~150 mm above the pivot -> ~40 mm, To-Do.md 2026-10-23) ---
STROKE = 40.0
S2_BEFORE_END = 4.0    # S2 trips this far before full travel
REST_GAP = 0.5         # pin tip ahead of the S1 lever tip at rest
ADJ = 4.0              # slots: each switch can move +-ADJ along X
X1 = 16.0              # S1 lever tip; pin is guided ~15 mm at rest
# --- mounting ---
BOLT_D = 6.2           # M6 at the pedal eye and the rear clevis
M2_NUT = (4.4, 1.8)    # across flats + clearance, depth of the nut channel

EDGE = PIN + GAP                                  # plunger edge height
TIP_Z = EDGE - (LEVER_REST + 0.5)                 # pin top at its very tip, clears the resting lever
TAN = math.tan(math.radians(CHAMFER_DEG))
CHAMFER_L = (PIN - TIP_Z) / TAN
D_TRIP = (EDGE - LEVER_TRIP - TIP_Z) / TAN        # pin tip past the lever tip when the switch trips
P0 = X1 - REST_GAP                                # pin tip at rest
S1_AT = REST_GAP + D_TRIP
S2_AT = STROKE - S2_BEFORE_END
X2 = P0 + S2_AT - D_TRIP
TUN_L = max(X2 + SW_L + ADJ + 3.0, P0 + STROKE + 3.0)
PIN_L = P0 + STROKE + 5.0                         # square part; the eye stays outside the tunnel at full travel
EYE_R = 7.0
CLEVIS_X = TUN_L + 7.0
OUT_Y = TUN / 2 + WALL
WALL_Y0 = SW_T / 2 + 0.15                         # switch face, window edge

assert X2 - (X1 + SW_L) >= 2 * ADJ + 1.5, f'stroke {STROKE} too short for two switches in a row'


def window_and_slots(xt):
    """Cuts for one switch whose lever tip (body front) is at xt."""
    xc = xt + SW_L / 2
    with BuildPart() as cut:
        with Locations((xt - ADJ - 0.3, -WALL_Y0, TUN - 0.1)):
            Box(SW_L + 2 * ADJ + 0.6, 2 * WALL_Y0, CEIL + SW_H + 2, align=(Align.MIN, Align.MIN, Align.MIN))
        for hx in (xc - SW_HOLE_PITCH / 2, xc + SW_HOLE_PITCH / 2):
            z = EDGE + SW_HOLE_UP
            with Locations((hx, WALL_Y0 - 0.1, z)):
                Box(2 * ADJ + 2.4, OUT_Y - WALL_Y0 + 0.2, 2.4, align=(Align.CENTER, Align.MIN, Align.CENTER))
                for dx in (-ADJ, ADJ):
                    with Locations((dx, 0, 0)):
                        Cylinder(1.2, OUT_Y - WALL_Y0 + 0.2, rotation=(-90, 0, 0),
                                 align=(Align.CENTER, Align.CENTER, Align.MIN))
            with Locations((hx, OUT_Y - M2_NUT[1], z)):
                Box(2 * ADJ + M2_NUT[0], M2_NUT[1] + 0.1, M2_NUT[0], align=(Align.CENTER, Align.MIN, Align.CENTER))
    return cut.part


def build_block():
    top = EDGE + SW_H
    with BuildPart() as p:
        with Locations((0, -OUT_Y, -FLOOR)):
            Box(TUN_L, 2 * OUT_Y, FLOOR + TUN + CEIL, align=(Align.MIN, Align.MIN, Align.MIN))
        with Locations((0, WALL_Y0, TUN)):
            Box(TUN_L, OUT_Y - WALL_Y0, top - TUN, align=(Align.MIN, Align.MIN, Align.MIN))
        for side in (-1, 1):
            y0 = TUN / 2 if side > 0 else -OUT_Y
            with Locations((TUN_L - 1, y0, -FLOOR)):
                Box(CLEVIS_X - TUN_L + 1, WALL, FLOOR + TUN + CEIL, align=(Align.MIN, Align.MIN, Align.MIN))
            r = (FLOOR + TUN + CEIL) / 2
            with Locations((CLEVIS_X, y0 + WALL / 2, -FLOOR + r)):
                Cylinder(r, WALL, rotation=(90, 0, 0))
        with Locations((-1, -TUN / 2, 0)):
            Box(TUN_L + 1, TUN, TUN, align=(Align.MIN, Align.MIN, Align.MIN), mode=Mode.SUBTRACT)
        with Locations((CLEVIS_X, 0, -FLOOR + (FLOOR + TUN + CEIL) / 2)):
            Cylinder(BOLT_D / 2, 2 * OUT_Y + 2, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    block = p.part
    for xt in (X1, X2):
        block = block - window_and_slots(xt)
    return block


def build_pin():
    """Pin with its tip at X = 0 (rest position is X = P0), bottom on the tunnel floor."""
    with BuildPart() as p:
        with BuildSketch(Plane.XZ):
            Polygon((-PIN_L, 0), (0, 0), (0, TIP_Z), (-CHAMFER_L, PIN), (-PIN_L, PIN), align=None)
        extrude(amount=PIN / 2, both=True)
        with Locations((-PIN_L - EYE_R + 2, 0, PIN / 2)):
            Cylinder(EYE_R, PIN, rotation=(90, 0, 0))
            Cylinder(BOLT_D / 2, PIN + 2, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    return p.part


def build_coupon(block):
    """Fit test: the S1 section of the block, to check switch, screws and pin before the full print."""
    with BuildPart() as box:
        with Locations((X1 - ADJ - 3, -50, -50)):
            Box(SW_L + 2 * ADJ + 6, 100, 100, align=(Align.MIN, Align.MIN, Align.MIN))
    return block & box.part


def lever_tip(s):
    """Lever tip below the plunger edge for switch tip xt at pedal travel s (pin tip at P0 + s)."""
    def at(xt):
        d = P0 + s - xt
        if d < 0:
            return LEVER_REST
        return min(LEVER_REST, max(GAP, EDGE - min(PIN, TIP_Z + d * TAN)))
    return at(X1), at(X2)


def meta():
    return {'sw': [SW_L, SW_H, SW_T], 'leverRest': LEVER_REST, 'leverTrip': LEVER_TRIP, 'pin': PIN, 'tun': TUN,
            'edge': EDGE, 'tipZ': TIP_Z, 'tan': TAN, 'gap': GAP, 'x1': X1, 'x2': X2, 'p0': P0, 'stroke': STROKE,
            's1At': S1_AT, 's2At': S2_AT, 'adj': ADJ, 'swY': WALL_Y0}


def export(part, path):
    export_step(part, str(path.with_suffix('.step')))
    export_stl(part, str(path.with_suffix('.stl')))


if __name__ == '__main__':
    out = Path(__file__).resolve().parent.parent / 'viewer' / 'downloads' / 'kupplung'
    out.mkdir(parents=True, exist_ok=True)
    block, pin = build_block(), build_pin()
    for name, part in (('block', block), ('stift', pin), ('probestueck', build_coupon(block))):
        assert len(part.solids()) == 1, f'{name}: {len(part.solids())} loose pieces'
        export(part, out / f'kupplungsschalter_{name}')
        bb = part.bounding_box().size
        print(f'{name}: {bb.X:.1f} x {bb.Y:.1f} x {bb.Z:.1f} mm, {part.volume / 1000:.1f} cm³')
    (out / 'kupplungsschalter.json').write_text(json.dumps(meta(), indent=1))
    print(f'S1 trips at {S1_AT:.1f} mm, S2 at {S2_AT:.1f} mm of {STROKE:.0f} mm; '
          f'slots move each trip point by +-{ADJ:.0f} mm; S1/S2 lever tips at X {X1:.1f} / {X2:.1f}')
    for s in (0, S1_AT - 0.5, S1_AT + 0.5, S2_AT - 0.5, S2_AT + 0.5, STROKE):
        a, b = lever_tip(s)
        print(f'  travel {s:5.1f}: lever S1 {a:.2f}  S2 {b:.2f}  -> S1 {"on" if a <= LEVER_TRIP else "off"}, '
              f'S2 {"on" if b <= LEVER_TRIP else "off"}')
