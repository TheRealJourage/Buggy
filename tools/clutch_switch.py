"""Clutch pedal switch: printed tunnel block with two micro switches and a sliding pin, parametric.

Concept (user 2026-10-31): Vault Buggy/Funktionsliste.md block 5 "Kupplung". The pin is fixed to the
pedal and slides in the tunnel; S1 trips shortly after the pedal leaves its rest, S2 only near full
travel. None = engaged, S1 only = moving, S1 + S2 = disengaged.
Switch: Lofair M102-011 (Omron SS-5 size), measured by the user 2026-10-31 (Vault Teile-Bestand.md).
Coordinates: X = pin travel (pedal pushes towards +X), Y across, Z up; tunnel floor at Z = 0.
The switches sit on top of the tunnel, hinge facing the incoming pin: the pin runs up the slope of the
lever instead of against its tip (user 2026-10-31). Both switches slide in one rail along the whole switch
zone, set on the vehicle. The tunnel reaches well in front of S1, so pin and block always stay plugged
together and form a strut between two pivots (pedal eye, rear clevis).
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
LEAD = (2.5, 30.0)     # chamfer on the pin's leading top edge (height, angle to the top): rides over a lever
                       # or switch that sits a bit low, or a step in the tunnel (user 2026-10-31)
LEAD_EDGE = 1.0        # 45 deg on the other three leading edges
WALL, FLOOR, CEIL = 3.0, 3.0, 2.0
TOP_EXTRA = 4.0         # outer rail wall raised above the switch top: only 0.45 mm were left over the nut channel,
                       # first print broke there (user 2026-11-02)
# --- travel (hypothesis: pin on the pedal ~150 mm above the pivot -> ~40 mm, To-Do.md 2026-10-23) ---
STROKE = 40.0
STROKE_MAX = 60.0      # pin and tunnel allow up to the travel at the pedal tip (55-60 mm), mount point free
S2_BEFORE_END = 4.0    # S2 trips this far before full travel (default position in the rail)
REST_MARGIN = 1.0      # at rest the pin tip stops this far before it would touch the S1 lever (hypothesis: lever
                       # runs straight from the hinge at the plunger edge to the tip)
ADJ = 16.0             # rail reaches this far beyond the default positions (+2 cm, user 2026-10-31)
P0 = 45.0              # pin tip at rest: guided length in the tunnel
# --- mounting ---
BOLT_D = 6.2           # M6 at the pedal eye and the rear clevis
M2_NUT = (4.4, 1.8)    # across flats + clearance, depth of the nut channel

EDGE = PIN + GAP                                  # plunger edge height
LEAD_TAN = math.tan(math.radians(LEAD[1]))
LEAD_L = LEAD[0] / LEAD_TAN


def lever_at(u):
    """Lever tip below the plunger edge with the pin tip u past the hinge: the straight lever rests on the
    highest point of the pin top (flat at GAP, chamfer lower towards the tip)."""
    if u <= 0:
        return LEVER_REST
    tip = LEVER_REST
    for i in range(1, 401):
        w = min(u, SW_L) * i / 400
        below = GAP + max(0.0, w - (u - LEAD_L)) * LEAD_TAN
        tip = min(tip, SW_L * below / w)
    return tip


def first_u(limit):
    u = 0.0
    while lever_at(u) > limit:
        u += 0.01
    return u


U_TOUCH = first_u(LEVER_REST - 1e-6)              # pin tip past the hinge when it first lifts the lever
U_TRIP = first_u(LEVER_TRIP)                      # ... when the switch trips
REST_IN = U_TOUCH - REST_MARGIN
X1 = P0 - REST_IN                                 # S1 hinge (front of the body)
S1_AT = U_TRIP - REST_IN
S2_AT = STROKE - S2_BEFORE_END
X2 = P0 + S2_AT - U_TRIP                          # S2 hinge
RAIL = (X1 - ADJ, X2 + SW_L + ADJ)
TUN_L = max(RAIL[1] + 3.0, P0 + STROKE_MAX + 3.0)
PIN_L = P0 + STROKE_MAX + 5.0                         # square part; the eye stays outside the tunnel at full travel
EYE_R = 7.0
CLEVIS_X = TUN_L + 7.0
OUT_Y = TUN / 2 + WALL
WALL_Y0 = SW_T / 2 + 0.15                         # switch face, window edge

assert REST_IN < U_TOUCH
assert X2 - (X1 + SW_L) >= 1.0, f'stroke {STROKE} too short for two switches in a row'


def rail():
    """One window for both switch bodies, one screw slot and one nut channel along the whole rail."""
    x0, x1 = RAIL
    h0 = x0 + (SW_L - SW_HOLE_PITCH) / 2              # first possible hole centre
    h1 = x1 - (SW_L - SW_HOLE_PITCH) / 2
    z = EDGE + SW_HOLE_UP
    with BuildPart() as cut:
        with Locations((x0 - 0.3, -WALL_Y0, TUN - 0.1)):
            Box(x1 - x0 + 0.6, 2 * WALL_Y0, CEIL + SW_H + 2, align=(Align.MIN, Align.MIN, Align.MIN))
        with Locations(((h0 + h1) / 2, WALL_Y0 - 0.1, z)):
            Box(h1 - h0 + 2.4, OUT_Y - WALL_Y0 + 0.2, 2.4, align=(Align.CENTER, Align.MIN, Align.CENTER))
        with Locations(((h0 + h1) / 2, OUT_Y - M2_NUT[1], z)):
            Box(h1 - h0 + M2_NUT[0], M2_NUT[1] + 0.1, M2_NUT[0], align=(Align.CENTER, Align.MIN, Align.CENTER))
    return cut.part


def build_block():
    top = EDGE + SW_H + TOP_EXTRA
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
    return p.part - rail()


def build_pin():
    """Pin with its tip at X = 0 (rest position is X = P0), bottom on the tunnel floor."""
    with BuildPart() as p:
        with BuildSketch(Plane.XZ):
            Polygon((-PIN_L, 0), (-LEAD_EDGE, 0), (0, LEAD_EDGE), (0, PIN - LEAD[0]), (-LEAD_L, PIN), (-PIN_L, PIN),
                    align=None)
        extrude(amount=PIN / 2, both=True)
        with BuildSketch(Plane.XY.offset(-1)):
            h = PIN / 2
            Polygon((-PIN_L - 1, -h), (-LEAD_EDGE, -h), (0, LEAD_EDGE - h), (0, h - LEAD_EDGE), (-LEAD_EDGE, h),
                    (-PIN_L - 1, h), align=None)
        extrude(amount=PIN + 2, mode=Mode.INTERSECT)
        with Locations((-PIN_L - EYE_R + 2, 0, PIN / 2)):
            Cylinder(EYE_R, PIN, rotation=(90, 0, 0))
            Cylinder(BOLT_D / 2, PIN + 2, rotation=(90, 0, 0), mode=Mode.SUBTRACT)
    return p.part


def build_coupon(block):
    """Fit test: the S1 section of the block, to check switch, screws and pin before the full print."""
    with BuildPart() as box:
        with Locations((X1 - ADJ - 3, -50, -50)):
            Box(SW_L + ADJ + 6, 100, 100, align=(Align.MIN, Align.MIN, Align.MIN))
    return block & box.part


def lever_tip(s):
    """Lever tips of S1 and S2 at pedal travel s (pin tip at P0 + s)."""
    return lever_at(P0 + s - X1), lever_at(P0 + s - X2)


def meta():
    return {'sw': [SW_L, SW_H, SW_T], 'leverRest': LEVER_REST, 'leverTrip': LEVER_TRIP, 'pin': PIN, 'tun': TUN,
            'edge': EDGE, 'gap': GAP, 'x1': X1, 'x2': X2, 'p0': P0, 'stroke': STROKE, 'strokeMax': STROKE_MAX, 'lead': [LEAD[0], LEAD_TAN],
            's1At': S1_AT, 's2At': S2_AT, 'rail': RAIL}


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
          f'S1/S2 hinge at X {X1:.1f} / {X2:.1f}, rail X {RAIL[0]:.1f}..{RAIL[1]:.1f}, tunnel {TUN_L:.1f}')
    for s in (0, S1_AT - 0.5, S1_AT + 0.5, S2_AT - 0.5, S2_AT + 0.5, STROKE):
        a, b = lever_tip(s)
        print(f'  travel {s:5.1f}: lever S1 {a:.2f}  S2 {b:.2f}  -> S1 {"on" if a <= LEVER_TRIP else "off"}, '
              f'S2 {"on" if b <= LEVER_TRIP else "off"}')
