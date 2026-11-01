"""ABS rear sensor spacer: sits between the front side plate of the wheel bearing cube and the sensor flange.

Geometry and open points: Vault Buggy/ABS.md "Fühler-Halter hinten", viewer/abs-fuehler.html.
Two heights for the two possible tone ring diameters (73.8 mm catalogue / ~81 mm inferred), 0.7 mm air gap,
sensor fitting depth 28 mm (3B0 927 807 G, ERA catalogue). Only one of them fits; it is used on both sides.
The screw (M6, placeholder) passes through a slot into a tapped hole in the side plate, so the print carries no
thread. Slot covers 14-26 mm from the sensor axis because the flange ear offset is not measured yet.
Marking: 1 notch = low variant, 2 notches = high variant.
Run with ~/.venvs/cad/bin/python -I tools/abs_spacer.py
"""
from pathlib import Path

from build123d import (Align, Box, BuildPart, BuildSketch, Circle, Cylinder, Locations, Mode, SlotCenterToCenter,
                       extrude, export_step, export_stl)

AXIS_TO_PLATE = 55.0          # outer face of the side plate from the axle axis (CAD: inner 47 + 8)
DEPTH = 28.0                  # sensor flange face to tip
GAP = 0.7                     # hypothesis 0.5-1.0 mm
RINGS = {'niedrig': 73.8, 'hoch': 81.0}
BORE = 18.4                   # sensor body fits 18 mm (user) + clearance
OUTER = 32.0
EAR_FROM, EAR_TO = 14.0, 26.0  # screw slot range from the sensor axis, placeholder until the flange is measured
SCREW_CLR = 6.6               # M6 clearance
LOBE = 16.0


def height(ring):
    return ring / 2 + GAP + DEPTH - AXIS_TO_PLATE


def build(h, notches):
    with BuildPart() as p:
        with BuildSketch():
            Circle(OUTER / 2)
            with Locations(((EAR_FROM + EAR_TO) / 2, 0)):
                SlotCenterToCenter(EAR_TO - EAR_FROM, LOBE)
        extrude(amount=h)
        Cylinder(BORE / 2, h, align=(Align.CENTER, Align.CENTER, Align.MIN), mode=Mode.SUBTRACT)
        with BuildSketch():
            with Locations(((EAR_FROM + EAR_TO) / 2, 0)):
                SlotCenterToCenter(EAR_TO - EAR_FROM, SCREW_CLR)
        extrude(amount=h, mode=Mode.SUBTRACT)
        for i in range(notches):
            with Locations((-OUTER / 2, (i - (notches - 1) / 2) * 5, h)):
                Box(3, 2, 3, mode=Mode.SUBTRACT)
    return p.part


if __name__ == '__main__':
    out = Path(__file__).resolve().parent.parent / 'viewer' / 'downloads' / 'abs'
    out.mkdir(parents=True, exist_ok=True)
    for n, (name, ring) in enumerate(RINGS.items(), start=1):
        h = round(height(ring), 1)
        part = build(h, n)
        assert len(part.solids()) == 1
        path = out / f'abs_abstandshalter_{name}_{str(h).replace(".", "_")}mm'
        export_step(part, str(path.with_suffix('.step')))
        export_stl(part, str(path.with_suffix('.stl')))
        bb = part.bounding_box().size
        print(f'{name}: ring {ring} -> {h} mm high, {bb.X:.1f} x {bb.Y:.1f} x {bb.Z:.1f} mm, {n} notch(es)')
