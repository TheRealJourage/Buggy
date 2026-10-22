"""Light panel housing for SLA printing (Anycubic M7 Max), parametric.

Layout and functions: Vault Buggy/Bordnetz-Konzept.md ("Lichtpanel", "Panel-Bau").
Measurements from the user (2026-10-22). Coordinates: X right, Y up (as seen by the
driver), Z towards the driver. Front face of the plate at Z = PLATE_T, back face at Z = 0,
the rim extends behind the plate to Z = -RIM_H. Run with ~/.venvs/cad/bin/python.
"""
import json
from pathlib import Path

from build123d import (Align, Box, BuildPart, BuildSketch, Cylinder, FontStyle, Locations,
                       Mode, Plane, RectangleRounded, Text, extrude, export_step, export_stl)

# --- toggle switch (all types identical) ---
HOLE_D = 12.2          # measured 12 mm, +0.2 for resin shrink / fit
KEY_W = 1.8            # groove in the thread: 2.0 wide, 0.8 deep -> nose slightly smaller
KEY_D = 0.7
FOOT_W, FOOT_H = 16.4, 29.4   # switch foot 16 x 29 (+0.4), long side vertical
FOOT_POCKET = 2.0      # foot sits this deep in the back of the plate (assumed centred on the thread)
# --- flip cover base plate ---
CAP_W, CAP_H, CAP_T = 17.4, 41.3, 0.9   # 17.1 x 41 x 0.9 (+0.3 in the plane)
CAP_TOP = 21.5 + HOLE_D / 2 - 0.1       # hole centre -> top edge of cap plate (21.5 from hole edge)
LED_D = 5.2            # 5 mm LED from the back
LED_UP = 6.0 + 16.0    # LED centre 16 mm above the hole edge
# --- panel ---
PLATE_T = 8.0          # 8 - 2 pocket = 6 mm in the thread, leaves 6 mm for nut + washer
PITCH_X, PITCH_Y = 36.0, 72.0   # glove-friendly spacing
LABEL_Y = -22.0        # label centre below the hole
LABEL_SIZE, LABEL_DEPTH = 4.2, 0.6
INNER_R = 0.5
MARGIN = 10.0
WALL, RIM_H = 3.0, 45.0
CORNER_R = 1.0         # angular box, only a small edge break
CABLE_D = 7.0          # 4 wires, exit towards the steering wheel (right side)
GROOVE = (1.0, 1.5, 5.0)  # silicone key groove in the inner wall: depth, height, distance behind plate
# --- KY-023 style joystick module (product picture: PCB 34 x 27, 32 high). ASSUMED, measure when it arrives ---
JOY_PCB = (27.0, 34.0, 1.6)     # X x Y x thickness, header pins at the bottom end
JOY_HOLES = (20.0, 27.0, 2.6)   # mounting hole spacing X, Y and pilot diameter for M3 self-tapping (assumed)
JOY_HOLE_D = 26.0               # front opening for the dome, must clear the tilted dome (assumed)
JOY_POCKET = (26.0, 26.0, 2.0)  # square pocket from the back for the gimbal frame, leaves this skin at the front
JOY_PCB_Z = -8.4                # PCB top face behind the plate (cap top then ~14 mm above the front)
JOY_DAM = (WALL, 15.0)          # wall around the module so the silicone stays out: thickness, depth behind plate

# column-wise pairs (top, bottom), mirrored 2026-10-22: light/fog is the rightmost column
# (label, has_cap, led_under_cap); None = empty place
CELLS = [
    [("SUCHLICHT", True, False), "JOY"],
    [("OFFROAD MITTE", True, False), ("OFFROAD HECK", True, False)],
    [("RUNDUM", False, False), ("NAHFELD", False, False)],
    [("ZUSATZ-FERN", True, False), ("RÜCKFAHR", True, True)],
    [("LICHT", False, False), ("NEBEL", True, True)],
]


def centres():
    cols = len(CELLS)
    for ci, col in enumerate(CELLS):
        x = (ci - (cols - 1) / 2) * PITCH_X
        for ri, cell in enumerate(col):
            y = PITCH_Y / 2 - ri * PITCH_Y
            if cell and cell != 'JOY':
                yield x, y, cell


def joystick_pos():
    for ci, col in enumerate(CELLS):
        for ri, cell in enumerate(col):
            if cell == 'JOY':
                return (ci - (len(CELLS) - 1) / 2) * PITCH_X, PITCH_Y / 2 - ri * PITCH_Y


def outline():
    xs = [(ci - (len(CELLS) - 1) / 2) * PITCH_X for ci in range(len(CELLS))]
    w = (max(xs) - min(xs)) + CAP_W + 2 * MARGIN
    top = PITCH_Y / 2 + CAP_TOP + MARGIN
    bottom = -PITCH_Y / 2 + LABEL_Y - LABEL_SIZE / 2 - MARGIN
    return w, top - bottom, (top + bottom) / 2


def cut_switch(x, y, cap, led):
    """Hole with key nose, foot pocket, cover recess and LED hole; call inside BuildPart."""
    # thread hole with key nose at the bottom
    with Locations((x, y, PLATE_T / 2)):
        Cylinder(HOLE_D / 2, PLATE_T + 1, mode=Mode.SUBTRACT)
    nose_top = PLATE_T - (CAP_T if cap else 0)
    nose_h = nose_top - FOOT_POCKET
    with Locations((x, y - HOLE_D / 2 + KEY_D / 2, FOOT_POCKET + nose_h / 2)):
        Box(KEY_W, KEY_D + 0.4, nose_h)
    # switch foot pocket in the back face
    with Locations((x, y, FOOT_POCKET / 2)):
        Box(FOOT_W, FOOT_H, FOOT_POCKET, mode=Mode.SUBTRACT)
    if cap:
        cap_cy = y + CAP_TOP - CAP_H / 2
        with Locations((x, cap_cy, PLATE_T - CAP_T / 2)):
            Box(CAP_W, CAP_H, CAP_T, mode=Mode.SUBTRACT)
    if led:
        with Locations((x, y + LED_UP, PLATE_T / 2)):
            Cylinder(LED_D / 2, PLATE_T + 1, mode=Mode.SUBTRACT)


def build(labels=True):
    w, h, yc = outline()
    with BuildPart() as p:
        # plate + rim, rounded outside corners
        with BuildSketch(Plane.XY.offset(-RIM_H)):
            with Locations((0, yc)):
                RectangleRounded(w, h, CORNER_R)
        extrude(amount=RIM_H + PLATE_T)
        with BuildSketch(Plane.XY.offset(-RIM_H)):
            with Locations((0, yc)):
                RectangleRounded(w - 2 * WALL, h - 2 * WALL, INNER_R)
        extrude(amount=RIM_H, mode=Mode.SUBTRACT)

        for x, y, (label, cap, led) in centres():
            cut_switch(x, y, cap, led)
            if not labels:
                continue
            # engraved label
            with BuildSketch(Plane.XY.offset(PLATE_T)) as sk:
                with Locations((x, y + LABEL_Y)):
                    Text(label, font_size=LABEL_SIZE, font_style=FontStyle.BOLD,
                         align=(Align.CENTER, Align.CENTER))
            extrude(sk.sketch, amount=-LABEL_DEPTH, mode=Mode.SUBTRACT)

        # joystick: dam (added) first, then opening, gimbal pocket, inside of the dam, standoffs
        jx, jy = joystick_pos()
        pw, ph, _ = JOY_PCB
        dw, dd = JOY_DAM
        with Locations((jx, jy, -dd / 2)):
            Box(pw + 2 + 2 * dw, ph + 2 + 2 * dw, dd)
            Box(pw + 2, ph + 2, dd + 0.01, mode=Mode.SUBTRACT)
        with Locations((jx, jy, PLATE_T / 2)):
            Cylinder(JOY_HOLE_D / 2, PLATE_T + 1, mode=Mode.SUBTRACT)
        qw, qh, skin = JOY_POCKET
        with Locations((jx, jy, (PLATE_T - skin) / 2)):
            Box(qw, qh, PLATE_T - skin, mode=Mode.SUBTRACT)
        hx, hy, hd = JOY_HOLES
        boss_h = -JOY_PCB_Z
        for sx in (-1, 1):
            for sy in (-1, 1):
                with Locations((jx + sx * hx / 2, jy + sy * hy / 2, -boss_h / 2)):
                    Cylinder(3.0, boss_h)
                    Cylinder(hd / 2, boss_h + 0.01, mode=Mode.SUBTRACT)

        # silicone key groove in the inner wall
        gd, gh, gz = GROOVE
        with BuildSketch(Plane.XY.offset(-gz - gh / 2)):
            with Locations((0, yc)):
                RectangleRounded(w - 2 * WALL + 2 * gd, h - 2 * WALL + 2 * gd, INNER_R + gd)
        extrude(amount=gh, mode=Mode.SUBTRACT)

        # cable exit towards the steering wheel (+X wall), mid depth
        with Locations((w / 2 - WALL / 2, yc, -RIM_H / 2)):
            Cylinder(CABLE_D / 2, WALL + 4, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    return p.part, (w, h)


def build_coupon():
    """Fit test: one place with cover + LED, one plain place, same plate thickness."""
    w = PITCH_X + CAP_W + 2 * 8
    top, bottom = CAP_TOP + 6, -FOOT_H / 2 - 6
    with BuildPart() as p:
        with Locations((0, (top + bottom) / 2, PLATE_T / 2)):
            Box(w, top - bottom, PLATE_T)
        cut_switch(-PITCH_X / 2, 0, True, True)
        cut_switch(PITCH_X / 2, 0, False, False)
    return p.part


if __name__ == '__main__':
    out = Path(__file__).resolve().parent.parent / 'viewer' / 'downloads' / 'panels'
    out.mkdir(parents=True, exist_ok=True)
    plain, _ = build(labels=False)
    export_step(plain, str(out / 'lichtpanel_ohne_schrift.step'))
    export_stl(plain, str(out / 'lichtpanel_ohne_schrift.stl'))
    part, (w, h) = build()
    coupon = build_coupon()
    export_stl(coupon, str(out / 'probestueck.stl'))
    export_step(coupon, str(out / 'probestueck.step'))
    cb = coupon.bounding_box()
    print(f'coupon {cb.size.X:.1f} x {cb.size.Y:.1f} x {cb.size.Z:.1f} mm, {coupon.volume / 1000:.1f} cm³')
    export_step(part, str(out / 'lichtpanel.step'))
    export_stl(part, str(out / 'lichtpanel.stl'))
    meta = {'plateT': PLATE_T, 'capTop': CAP_TOP, 'capW': CAP_W - 0.3, 'capH': CAP_H - 0.3, 'capT': CAP_T,
            'ledUp': LED_UP, 'thread': 12.0 - (PLATE_T - FOOT_POCKET), 'lever': 19.0,
            'cells': [{'x': x, 'y': y, 'label': c[0], 'cap': c[1], 'led': c[2]} for x, y, c in centres()],
            'joy': dict(zip(('x', 'y'), joystick_pos())) | {'pcbZ': JOY_PCB_Z, 'pcb': JOY_PCB}}
    (out / 'lichtpanel.json').write_text(json.dumps(meta, ensure_ascii=False, indent=1))
    bb = part.bounding_box()
    print(f'outline {w:.1f} x {h:.1f} mm, depth {bb.size.Z:.1f} mm, volume {part.volume / 1000:.1f} cm³')
