"""Cockpit panel housings for SLA printing (Anycubic M7 Max), parametric.

Layout and functions: Vault Buggy/Bordnetz-Konzept.md ("Lichtpanel", "Panel Motor & Zubehör",
"Panel Entertainment, Audio & Helligkeit", "Panel-Bau"). Measurements from the user (2026-10-22).
Coordinates: X right, Y up (as seen by the driver), Z towards the driver. Front face of the plate
at Z = PLATE_T, back face at Z = 0, the rim extends behind the plate to Z = -RIM_H.
Run with ~/.venvs/cad/bin/python -I tools/panels.py
"""
import json
from pathlib import Path

from build123d import (Align, Box, BuildPart, BuildSketch, Cylinder, FontStyle, Locations,
                       Mode, Plane, RectangleRounded, Text, extrude, export_step, export_stl)

# --- toggle switch (all types identical) ---
HOLE_D = 12.2          # measured 12 mm, +0.2 for resin shrink / fit
KEY_W = 1.8            # groove in the thread: 2.0 wide, 0.8 deep -> nose slightly smaller
KEY_D = 0.7
FOOT_W, FOOT_H = 16.4, 31.4   # switch foot 16 x 29 (+0.4), long side vertical; +1 at both ends after the test print (user 2026-10-22)
FOOT_POCKET = 2.0      # foot sits this deep in the back of the plate (assumed centred on the thread)
# --- LANBOO 12NZ2 momentary switch (T/T2), measured by the user 2026-10-30: no groove, a flat at the bottom of the thread ---
LB_THREAD = 6.25       # thread above the seal ring
LB_THREAD_FLAT = 5.22  # flat from the thread centre (11.84 across, 11.14 over the flat)
LB_SEAL = (12.4, 0.8, 0.1)    # seal ring 12.3 dia x 0.8: bore, ring height, squeeze (the seal is the stop)
LB_RING = (15.2, 2.29)        # metal ring 14.87 dia x 2.29 between seal and plate
LB_PLATE = (15.4, 30.6)       # metal plate 15.03 x 30.16 (+0.4), long side vertical; the pocket keeps it from turning
LB_SCREW_CLR = 1.8            # two screw heads 3.3 dia, 1.5 high: both ends beyond the ring are pocketed this much deeper
LB_LEVER = 17.45
# --- flip cover base plate ---
CAP_W, CAP_H, CAP_T = 17.4, 41.3, 0.9   # 17.1 x 41 x 0.9 (+0.3 in the plane)
CAP_TOP = 21.5 + HOLE_D / 2 - 0.1       # hole centre -> top edge of cap plate (21.5 from hole edge)
LED_D = 5.2            # 5 mm LED from the back
LED_UP = 6.0 + 16.0    # LED centre 16 mm above the hole edge
# --- potentiometer: thread 6.75 x 6, body 16.3 dia, 9 deep, pin 2.4 dia left 4 mm from the thread ---
POT_HOLE = 7.1
POT_BODY = 17.0        # back pocket and dam inside, body 16.3
POT_LUGS = (17.0, 6.75 / 2 + 14.0 + 0.5)  # solder lugs 16 wide, reach 14 mm below the thread edge: width, reach from centre
POT_EXTRA = 0.0        # extra room at both ends of the long side; pocket as test-printed fits (user 2026-10-30)
POT_SKIN = 3.0         # plate left in front of the pot, rest of the 6 mm thread takes the nut
POT_PIN = (7.0, 9.4, 2.8, 1.5)   # radial slot for the pin (left): r from, r to, width, depth into the skin
POT_DAM = (2.0, 12.0)  # tube around the pot so the silicone stays out: wall, depth behind plate
KNOB_D = 24.0          # preview only, knob not measured yet
# --- panel ---
PLATE_T = 10.0         # 10 - 2 pocket = 8 mm in the thread, leaves 4 mm for the 2 mm nut (user 2026-10-30)
PITCH_X, PITCH_Y = 36.0, 72.0   # glove-friendly spacing
LABEL_Y = -22.0        # label centre below the hole
LABEL_SIZE, LABEL_DEPTH = 4.2, 0.6
MARGIN = 10.0
WALL, RIM_H = 3.0, 45.0
CORNER_R, INNER_R = 1.0, 0.5   # angular box, only a small edge break
CABLE_D = 7.0          # 4 wires, exit towards the steering wheel
GROOVE = (1.0, 1.5, 5.0)  # silicone key groove in the inner wall: depth, height, distance behind plate
# --- KY-023 style joystick module (product picture: PCB 34 x 27, 32 high). ASSUMED, measure when found ---
JOY_PCB = (27.0, 34.0, 1.6)     # X x Y x thickness, header pins at the bottom end
JOY_HOLES = (20.0, 27.0, 2.6)   # mounting hole spacing X, Y and pilot diameter for M3 self-tapping (assumed)
JOY_HOLE_D = 26.0               # front opening for the dome, must clear the tilted dome (assumed)
JOY_POCKET = (26.0, 26.0, 2.0)  # square pocket from the back for the gimbal frame, leaves this skin at the front
JOY_PCB_Z = -6.4                # PCB top face behind the plate, 16.4 behind the front (cap top then ~14 mm above the front)
JOY_DAM = (WALL, 15.0)          # wall around the module so the silicone stays out: thickness, depth behind plate


def sw(label, cap=False, led=False, lb=False):
    return {'kind': 'sw', 'label': label, 'cap': cap, 'led': led, 'lb': lb}


def pot(label):
    return {'kind': 'pot', 'label': label}


JOY = {'kind': 'joy', 'label': ''}

# columns of (top, bottom) from left to right as seen by the driver; cable side = towards the wheel
PANELS = {
    'lichtpanel': {'cable': +1, 'cols': [
        [sw('SUCHLICHT', True), JOY],
        [sw('OFFROAD MITTE', True), sw('OFFROAD HECK', True)],
        [sw('RUNDUM'), sw('NAHFELD')],
        [sw('ZUSATZ-FERN', True), sw('RÜCKFAHR', True, True)],
        [sw('LICHT'), sw('NEBEL', True, True)],
    ]},
    # split again 2026-11-01 (user): everything needed to drive on the engine panel, incl. compressor
    'motorpanel': {'cable': -1, 'cols': [
        [sw('ZÜNDUNG', True), sw('START', True, lb=True)],
        [sw('MOTOR', True), sw('KOMPRESSOR', True)],
        [sw('LÜFTER'), pot('LÜFTER %')],
    ]},
    'zubehoerpanel': {'cable': -1, 'cols': [
        [sw('WINDE', True), sw('12 V FRONT')],
        [sw('SITZHEIZUNG'), sw('12 V SITZ')],
        [None, sw('12 V HECK')],
    ]},
    'audiopanel': {'cable': -1, 'cols': [
        [sw('CARPLAY')], [pot('CARPLAY')], [pot('INTERCOM')], [pot('FUNK')], [pot('AMBIENT')], [pot('TACHO/LED')],
    ]},
    # engine test bench (user 2026-10-30): pump on, injectors on, start; engine off = injectors off.
    # Plate only (no rim, print time), no potting; a full-thickness tab on the left for a screw clamp.
    'testpanel': {'cable': +1, 'rim': False, 'tab': (-1, 35.0), 'cols': [
        [sw('PUMPE', True)], [sw('DÜSEN')], [sw('START', True, lb=True)],
    ]},
}


def cells(panel):
    cols = panel['cols']
    rows = max(len(c) for c in cols)
    for ci, col in enumerate(cols):
        x = (ci - (len(cols) - 1) / 2) * PITCH_X
        for ri, cell in enumerate(col):
            if cell:
                yield x, (rows - 1) / 2 * PITCH_Y - ri * PITCH_Y, cell


def extent(cell):
    """half width, top above and bottom below the cell centre."""
    label_bottom = -LABEL_Y + LABEL_SIZE / 2
    if cell['kind'] == 'joy':
        hw = JOY_PCB[0] / 2 + 1 + JOY_DAM[0]
        hh = JOY_PCB[1] / 2 + 1 + JOY_DAM[0]
        return hw, hh, hh
    if cell['kind'] == 'pot':
        r = POT_BODY / 2 + POT_DAM[0]
        top = max(r, POT_BODY / 2 + POT_EXTRA + POT_DAM[0])
        return max(r, POT_LUGS[0] / 2 + POT_DAM[0]), top, max(label_bottom, POT_LUGS[1] + POT_EXTRA + POT_DAM[0])
    if cell['cap']:
        return CAP_W / 2, CAP_TOP, label_bottom
    if cell.get('lb'):
        return LB_PLATE[0] / 2, LB_PLATE[1] / 2, label_bottom
    return FOOT_W / 2, FOOT_H / 2, label_bottom


def outline(panel):
    cs = list(cells(panel))
    left = min(x - extent(c)[0] for x, y, c in cs) - MARGIN
    right = max(x + extent(c)[0] for x, y, c in cs) + MARGIN
    top = max(y + extent(c)[1] for x, y, c in cs) + MARGIN
    bottom = min(y - extent(c)[2] for x, y, c in cs) - MARGIN
    return right - left, top - bottom, (left + right) / 2, (top + bottom) / 2


def cut_switch(x, y, cap, led):
    """Hole with key nose, foot pocket, cover recess and LED hole; call inside BuildPart."""
    with Locations((x, y, PLATE_T / 2)):
        Cylinder(HOLE_D / 2, PLATE_T + 1, mode=Mode.SUBTRACT)
    nose_top = PLATE_T - (CAP_T if cap else 0)
    nose_h = nose_top - FOOT_POCKET
    with Locations((x, y - HOLE_D / 2 + KEY_D / 2, FOOT_POCKET + nose_h / 2)):
        Box(KEY_W, KEY_D + 0.4, nose_h)
    with Locations((x, y, FOOT_POCKET / 2)):
        Box(FOOT_W, FOOT_H, FOOT_POCKET, mode=Mode.SUBTRACT)
    if cap:
        with Locations((x, y + CAP_TOP - CAP_H / 2, PLATE_T - CAP_T / 2)):
            Box(CAP_W, CAP_H, CAP_T, mode=Mode.SUBTRACT)
    if led:
        with Locations((x, y + LED_UP, PLATE_T / 2)):
            Cylinder(LED_D / 2, PLATE_T + 1, mode=Mode.SUBTRACT)


def lb_stack(cap):
    """Z of the seal-bore top and of the metal plate face. Thread ends as high as on the other switches
    (12 - (PLATE_T - FOOT_POCKET) above the front); with a cover the cover plate takes the place of the plate."""
    in_plate = LB_THREAD - (12.0 - (PLATE_T - FOOT_POCKET))
    top = PLATE_T - max(in_plate, CAP_T if cap else 0)
    return top, top - (LB_SEAL[1] - LB_SEAL[2]) - LB_RING[1]


def cut_switch_lb(x, y, cap):
    """LANBOO 12NZ2: thread hole with a straight edge at the flat, seal bore, ring bore, plate pocket
    with deeper ends for the screw heads, cover recess; call inside BuildPart."""
    top, mp = lb_stack(cap)
    with Locations((x, y, PLATE_T / 2)):
        Cylinder(HOLE_D / 2, PLATE_T + 1, mode=Mode.SUBTRACT)
    flat = LB_THREAD_FLAT + 0.1   # straight edge at the bottom; only in the thin thread layer, the pocket does the real locating
    with Locations((x, y - flat - 2, (top + PLATE_T) / 2)):
        Box(HOLE_D, 4, PLATE_T - top)
    seal_d, seal_h, squeeze = LB_SEAL
    with Locations((x, y, top - (seal_h - squeeze) / 2)):
        Cylinder(seal_d / 2, seal_h - squeeze + 0.01, mode=Mode.SUBTRACT)
    ring_d, _ = LB_RING
    with Locations((x, y, (mp + top) / 2)):
        Cylinder(ring_d / 2, top - mp, mode=Mode.SUBTRACT)
    pw, ph = LB_PLATE
    with Locations((x, y, (mp - 1) / 2)):
        Box(pw, ph, mp + 1, mode=Mode.SUBTRACT)
    end = (ph - ring_d) / 2
    for sy in (-1, 1):
        with Locations((x, y + sy * (ph - end) / 2, mp + LB_SCREW_CLR / 2)):
            Box(pw, end, LB_SCREW_CLR + 0.01, mode=Mode.SUBTRACT)
    if cap:
        with Locations((x, y + CAP_TOP - CAP_H / 2, PLATE_T - CAP_T / 2)):
            Box(CAP_W, CAP_H, CAP_T, mode=Mode.SUBTRACT)


def cut_pot(x, y):
    """Thread hole, body pocket (round + slot down for the solder lugs) leaving POT_SKIN,
    pin slot on the left, dam of the same shape behind the plate."""
    wall, depth = POT_DAM
    lw, reach = POT_LUGS
    up, down = POT_BODY / 2 + POT_EXTRA, reach + POT_EXTRA   # long side, measured from the pot centre
    with Locations((x, y, -depth / 2)):
        Cylinder(POT_BODY / 2 + wall, depth)
    with Locations((x, y + (up - down) / 2, -depth / 2)):
        Box(lw + 2 * wall, up + down + 2 * wall, depth)
    with Locations((x, y, PLATE_T / 2)):
        Cylinder(POT_HOLE / 2, PLATE_T + 1, mode=Mode.SUBTRACT)
    floor = PLATE_T - POT_SKIN
    h = floor + depth + 0.01
    with Locations((x, y, (floor - depth) / 2)):
        Cylinder(POT_BODY / 2, h, mode=Mode.SUBTRACT)
    with Locations((x, y + (up - down) / 2, (floor - depth) / 2)):
        Box(lw, up + down, h, mode=Mode.SUBTRACT)
    r0, r1, pw, pd = POT_PIN
    with Locations((x - (r0 + r1) / 2, y, floor + pd / 2)):
        Box(r1 - r0, pw, pd, mode=Mode.SUBTRACT)


def cut_joystick(x, y):
    pw, ph, _ = JOY_PCB
    dw, dd = JOY_DAM
    with Locations((x, y, -dd / 2)):
        Box(pw + 2 + 2 * dw, ph + 2 + 2 * dw, dd)
        Box(pw + 2, ph + 2, dd + 0.01, mode=Mode.SUBTRACT)
    with Locations((x, y, PLATE_T / 2)):
        Cylinder(JOY_HOLE_D / 2, PLATE_T + 1, mode=Mode.SUBTRACT)
    qw, qh, skin = JOY_POCKET
    with Locations((x, y, (PLATE_T - skin) / 2)):
        Box(qw, qh, PLATE_T - skin, mode=Mode.SUBTRACT)
    hx, hy, hd = JOY_HOLES
    boss_h = -JOY_PCB_Z
    for sx in (-1, 1):
        for sy in (-1, 1):
            # boss plus a rib out to the dam wall, a face contact with the plate alone does not fuse
            inner = pw / 2 + 1
            with Locations((x + sx * (hx / 2 + inner) / 2, y + sy * hy / 2, -boss_h / 2)):
                Box(inner - hx / 2 + 0.5, 2.0, boss_h)
            with Locations((x + sx * hx / 2, y + sy * hy / 2, -boss_h / 2)):
                Cylinder(3.0, boss_h)
                Cylinder(hd / 2, boss_h + 0.01, mode=Mode.SUBTRACT)


def build(panel, labels=True):
    w, h, xc, yc = outline(panel)
    rim = RIM_H if panel.get('rim', True) else 0
    with BuildPart() as p:
        with BuildSketch(Plane.XY.offset(-rim)):
            with Locations((xc, yc)):
                RectangleRounded(w, h, CORNER_R)
        extrude(amount=rim + PLATE_T)
        if rim:
            with BuildSketch(Plane.XY.offset(-rim)):
                with Locations((xc, yc)):
                    RectangleRounded(w - 2 * WALL, h - 2 * WALL, INNER_R)
            extrude(amount=rim, mode=Mode.SUBTRACT)

        if 'tab' in panel:
            side, tl = panel['tab']
            with Locations((xc + side * (w + tl) / 2, yc, PLATE_T / 2)):
                Box(tl + 0.5, h, PLATE_T)

        for x, y, c in cells(panel):
            if c['kind'] == 'sw' and c['lb']:
                cut_switch_lb(x, y, c['cap'])
            elif c['kind'] == 'sw':
                cut_switch(x, y, c['cap'], c['led'])
            elif c['kind'] == 'pot':
                cut_pot(x, y)
            else:
                cut_joystick(x, y)
            if labels and c['label']:
                with BuildSketch(Plane.XY.offset(PLATE_T)) as sk:
                    with Locations((x, y + LABEL_Y)):
                        Text(c['label'], font_size=LABEL_SIZE, font_style=FontStyle.BOLD,
                             align=(Align.CENTER, Align.CENTER))
                extrude(sk.sketch, amount=-LABEL_DEPTH, mode=Mode.SUBTRACT)

        if not rim:
            return p.part, (w + panel.get('tab', (0, 0))[1], h)
        # groove only into the wall (a ring), so dams and bosses inside stay whole
        gd, gh, gz = GROOVE
        with BuildSketch(Plane.XY.offset(-gz - gh / 2)):
            with Locations((xc, yc)):
                RectangleRounded(w - 2 * WALL + 2 * gd, h - 2 * WALL + 2 * gd, INNER_R + gd)
                RectangleRounded(w - 2 * WALL, h - 2 * WALL, INNER_R, mode=Mode.SUBTRACT)
        extrude(amount=gh, mode=Mode.SUBTRACT)

        side = panel['cable']
        with Locations((xc + side * (w / 2 - WALL / 2), yc, -RIM_H / 2)):
            Cylinder(CABLE_D / 2, WALL + 4, rotation=(0, 90, 0), mode=Mode.SUBTRACT)
    return p.part, (w + panel.get('tab', (0, 0))[1], h)


def build_coupon():
    """Fit test: one place with cover + LED, one plain place, one pot with its dam."""
    pot_hw, _, pot_bottom = extent(pot(''))
    left, right = -PITCH_X - CAP_W / 2 - 8, PITCH_X + pot_hw + 6
    top, bottom = CAP_TOP + 6, -pot_bottom - 4
    with BuildPart() as p:
        with Locations(((left + right) / 2, (top + bottom) / 2, PLATE_T / 2)):
            Box(right - left, top - bottom, PLATE_T)
        cut_switch(-PITCH_X, 0, True, True)
        cut_switch(0, 0, False, False)
        cut_pot(PITCH_X, 0)
    assert len(p.part.solids()) == 1
    return p.part


def build_coupon_lb():
    """Fit test for the LANBOO switch: one place with cover (as START), one without."""
    cs = [(-PITCH_X / 2, 0, sw('MIT KAPPE', True, lb=True)), (PITCH_X / 2, 0, sw('OHNE KAPPE', lb=True))]
    left, right = -PITCH_X / 2 - CAP_W / 2 - 8, PITCH_X / 2 + LB_PLATE[0] / 2 + 8
    top, bottom = CAP_TOP + 6, -LB_PLATE[1] / 2 - 6
    with BuildPart() as p:
        with Locations(((left + right) / 2, (top + bottom) / 2, PLATE_T / 2)):
            Box(right - left, top - bottom, PLATE_T)
        for x, y, c in cs:
            cut_switch_lb(x, y, c['cap'])
    assert len(p.part.solids()) == 1
    return p.part, cs


def meta(cs):
    out = []
    for x, y, c in cs:
        m = {'x': x, 'y': y} | c
        if c.get('lb'):
            top, mp = lb_stack(c['cap'])
            m |= {'thread': LB_THREAD - (PLATE_T - top) + LB_SEAL[2], 'lever': LB_LEVER, 'mp': mp - PLATE_T}
        out.append(m)
    return {'plateT': PLATE_T, 'capTop': CAP_TOP, 'capW': CAP_W - 0.3, 'capH': CAP_H - 0.3, 'capT': CAP_T,
            'ledUp': LED_UP, 'thread': 12.0 - (PLATE_T - FOOT_POCKET), 'lever': 19.0,
            'potSkin': POT_SKIN, 'knobD': KNOB_D, 'joyPcbZ': JOY_PCB_Z, 'joyPcb': JOY_PCB, 'cells': out}


def export(part, path):
    export_step(part, str(path.with_suffix('.step')))
    export_stl(part, str(path.with_suffix('.stl')))


if __name__ == '__main__':
    out = Path(__file__).resolve().parent.parent / 'viewer' / 'downloads' / 'panels'
    out.mkdir(parents=True, exist_ok=True)
    coupon = build_coupon()
    export(coupon, out / 'probestueck')
    coupon_lb, cs = build_coupon_lb()
    for n in ('probestueck_lanboo', 'probestueck_lanboo_ohne_schrift'):
        export(coupon_lb, out / n)
    (out / 'probestueck_lanboo.json').write_text(json.dumps(meta(cs), ensure_ascii=False, indent=1))
    bb = coupon_lb.bounding_box().size
    print(f'probestueck_lanboo: {bb.X:.1f} x {bb.Y:.1f} x {bb.Z:.1f} mm, {coupon_lb.volume / 1000:.1f} cm³')
    for cap in (True, False):
        top, mp = lb_stack(cap)
        print(f'lanboo cap={cap}: seal bore top {PLATE_T - top:.2f} behind front, metal plate {PLATE_T - mp:.2f}, '
              f'thread out {LB_THREAD - (PLATE_T - top) + LB_SEAL[2]:.2f}')
    for name, panel in PANELS.items():
        export(build(panel, labels=False)[0], out / f'{name}_ohne_schrift')
        part, (w, h) = build(panel)
        export(part, out / name)
        (out / f'{name}.json').write_text(json.dumps(meta(cells(panel)), ensure_ascii=False, indent=1))
        assert len(part.solids()) == 1, f'{name}: {len(part.solids())} loose pieces'
        bb = part.bounding_box()
        print(f'{name}: {w:.1f} x {h:.1f} x {bb.size.Z:.1f} mm, {part.volume / 1000:.1f} cm³')
