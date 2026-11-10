"""HW-039 (BTS7960 / IBT-2 H-bridge module) as a simplified envelope model for enclosure planning.

Measured by the user 2026-11-10 (Vault Buggy/Teile-Bestand.md): board 50 x 50 x 1, holes 3 mm at 40 x 40 centre
pitch, heatsink 50 x 32 x 22 on the back (50 parallel to the terminal edge, fins across), screw terminal 14 high,
capacitor 17 high, total 40. Component positions and footprints are estimated from photos (hypothesis).
Coordinates match viewer/hw039.html: origin = board centre, Z = 0 board bottom, +Y = terminal edge.
Run with ~/.venvs/cad/bin/python -I tools/hw039.py
"""
from pathlib import Path

from build123d import Box, Cylinder, Polygon, Pos, export_step, export_stl, extrude

P, T, HOLE, PITCH, CH = 50.0, 1.0, 3.0, 40.0, 2.0
HS_W, HS_D, HS_H, HS_BASE, HS_FINS, HS_FIN = 50.0, 32.0, 22.0, 4.0, 9, 1.6
TERM_W, TERM_D, TERM_H, TERM_X, TERM_OVER = 21.0, 10.0, 14.0, 4.5, 1.5
CAP_D, CAP_H, CAP_X, CAP_Y = 10.0, 17.0, -13.5, P / 2 - 3.5
HDR_X, HDR_Y, HP = 10.5, -21.0, 2.54


def box(w, d, h, x, y, z):
    return Pos(x, y, z + h / 2) * Box(w, d, h)


def cyl(dia, h, x, y, z):
    return Pos(x, y, z + h / 2) * Cylinder(dia / 2, h)


def build():
    h = P / 2
    outline = Polygon((-h + CH, -h), (h - CH, -h), (h, -h + CH), (h, h - CH), (h - CH, h), (-h + CH, h),
                      (-h, h - CH), (-h, -h + CH), align=None)
    board = extrude(outline, T)
    for sx in (-1, 1):
        for sy in (-1, 1):
            board -= cyl(HOLE, T, sx * PITCH / 2, sy * PITCH / 2, 0)
    parts = [board, box(HS_W, HS_D, HS_BASE, 0, 0, -HS_BASE)]
    for i in range(HS_FINS):
        x = -HS_W / 2 + HS_FIN / 2 + i * (HS_W - HS_FIN) / (HS_FINS - 1)
        parts.append(box(HS_FIN, HS_D, HS_H - HS_BASE, x, 0, -HS_H))
    parts += [cyl(6, 2, x, -1.5, T) for x in (-13.5, 13.5)]
    parts += [box(10, 9, 4.4, x, 0.5, T) for x in (-5.5, 5.5)]
    parts.append(box(13, 7.5, 2.5, 17.5, -9, T))
    parts.append(box(TERM_W, TERM_D, TERM_H, TERM_X, h + TERM_OVER - TERM_D / 2, T))
    parts.append(cyl(CAP_D, CAP_H, CAP_X, CAP_Y, T))
    parts.append(box(4 * HP, 2 * HP, 2.5, HDR_X, HDR_Y, T))
    for i in range(4):
        for j in range(2):
            parts.append(box(0.64, 0.64, 6.0, HDR_X - 1.5 * HP + i * HP, HDR_Y - HP / 2 + j * HP, T + 2.4))
    return sum(parts[1:], parts[0])


if __name__ == '__main__':
    out = Path(__file__).resolve().parent.parent / 'viewer' / 'downloads' / 'hw039'
    out.mkdir(parents=True, exist_ok=True)
    part = build()
    export_step(part, str(out / 'hw039.step'))
    export_stl(part, str(out / 'hw039.stl'))
    bb = part.bounding_box()
    print(f'{bb.size.X:.1f} x {bb.size.Y:.1f} x {bb.size.Z:.1f} mm, Z {bb.min.Z:.1f}..{bb.max.Z:.1f}, solids {len(part.solids())}')
