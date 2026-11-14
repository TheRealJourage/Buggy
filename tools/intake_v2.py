"""Intake runner adapter v2: continue the original bend to 90 deg, spigots on K100 throttle pitch, split in two halves."""
import math, sys, json
import numpy as np
from build123d import *

SRC = "/srv/data/obsidian/vault/Buggy/Ansaugung/Ansaugung.step"
OUT = sys.argv[1] if len(sys.argv) > 1 else "out"

# original bend (fitted from STEP centreline, y-z plane, identical for all 6 channels)
CY, CZ, R = 125.55, -20.67, 133.19
PHI_CUT = 45.0            # deg from vertical where the original is cut
PHI_END = 90.0            # outlet direction: parallel to flange face
CUT_N = Vector(0, math.sin(math.radians(PHI_CUT)), math.cos(math.radians(PHI_CUT)))

# channels at the cut (measured hole centres) and port centres at the flange
CUT_CENTRES = [(67.90, 31.14, 73.76), (2.80, 30.97, 73.93), (-63.86, 31.14, 73.76),
               (-128.97, 30.98, 73.92), (-195.64, 31.14, 73.76), (-260.75, 30.98, 73.92)]
PORT_MID = -97.8          # centre between ports 3 and 4 (65.2 pitch)
THROTTLE_PITCH = 379.3 / 5  # K100 rail, measured 2026-11-02, mean 75.86
THROTTLE_X = [PORT_MID + (2.5 - k) * THROTTLE_PITCH for k in range(6)]

R_OUT = 24.65             # original body OD 49.3
R_IN = 17.8               # original bore ~35.6
STRAIGHT = 6.0            # straight body after the bend
CONE = 9.0                # shoulder body -> spigot (no sharp step)
SPIGOT_L = 22.0
SPIGOT_OD = 41.0          # as printed
SPIGOT_ID = 35.0          # = throttle bore; wall 3.0 (was 2.5)
BEAD_OD = 42.4
BEAD_POS = 4.0            # bead centre from spigot tip
BEAD_W = 3.0
SPLIT_X = PORT_MID

BOLT_BORES = [(-226.6, 16.5), (-164.6, 16.5), (-34.0, 16.5), (34.0, 16.5)]
BORE_R = 8.0


def smoothstep(u):
    return u * u * u * (u * (6 * u - 15) + 10)  # quintic: zero slope + curvature at both ends


def channel(start, x_end):
    x0, y0, z0 = start
    n = 60
    pts = []
    for i in range(n + 1):
        u = i / n
        phi = math.radians(PHI_CUT + (PHI_END - PHI_CUT) * u)
        y = CY - R * math.cos(phi)
        z = CZ + R * math.sin(phi)
        # blend measured start offset into the ideal arc
        w = 1 - smoothstep(min(1, u * 4))
        a0y, a0z = CY - R * math.cos(math.radians(PHI_CUT)), CZ + R * math.sin(math.radians(PHI_CUT))
        y += w * (y0 - a0y)
        z += w * (z0 - a0z)
        x = x0 + (x_end - x0) * smoothstep(u)
        pts.append(Vector(x, y, z))
    t0 = CUT_N
    t1 = Vector(0, 1, 0)
    bend = Spline(*pts, tangents=(t0, t1))
    p_end = pts[-1]
    def tube(r, back):
        p_start = pts[0] - t0 * back
        path = Wire([Line(p_start, pts[0]), bend, Line(p_end, p_end + Vector(0, STRAIGHT, 0))])
        return sweep(Plane(origin=p_start, z_dir=t0) * Circle(r), path=path, transition=Transition.ROUND)
    outer = tube(R_OUT, 1.5)
    inner = tube(R_IN, 4.0)
    # coaxial end: cone + spigot + bead, axis along +y
    ax0 = p_end + Vector(0, STRAIGHT, 0)
    prof = [(0, R_OUT), (CONE, SPIGOT_OD / 2), (CONE + SPIGOT_L - BEAD_POS - BEAD_W / 2 - 1.2, SPIGOT_OD / 2),
            (CONE + SPIGOT_L - BEAD_POS - BEAD_W / 2, BEAD_OD / 2),
            (CONE + SPIGOT_L - BEAD_POS + BEAD_W / 2, BEAD_OD / 2),
            (CONE + SPIGOT_L - BEAD_POS + BEAD_W / 2 + 1.2, SPIGOT_OD / 2),
            (CONE + SPIGOT_L, SPIGOT_OD / 2), (CONE + SPIGOT_L, 0), (0, 0)]
    # build in local XZ (X = axis along length, Z = radius) and revolve around local X
    sk = Polyline(*[(a, r) for a, r in prof], close=True)
    face = make_face(Plane.XY * sk)
    end_outer = revolve(face, Axis.X, 360)
    end_outer = end_outer.rotate(Axis.Z, 90)  # local X -> global +Y
    end_outer = end_outer.translate(ax0 - Vector(0, 0, 0))
    end_inner = Cylinder(SPIGOT_ID / 2, CONE + SPIGOT_L + 2, align=(Align.CENTER, Align.CENTER, Align.MIN))
    end_inner = end_inner.rotate(Axis.X, -90).translate(ax0 - Vector(0, 1, 0))
    # bore transition 35.6 -> 35 inside the cone region
    trans = Cone(R_IN, SPIGOT_ID / 2, CONE, align=(Align.CENTER, Align.CENTER, Align.MIN))
    trans = trans.rotate(Axis.X, -90).translate(ax0)
    return outer + end_outer, inner + end_inner + trans, ax0


def main():
    src = import_step(SRC)
    cut_plane = Plane(origin=(0, 31.4, 73.5), z_dir=CUT_N)
    base = src.split(cut_plane, keep=Keep.BOTTOM)
    adds, subs, outlets = [], [], []
    for c, xe in zip(CUT_CENTRES, THROTTLE_X):
        o, i, ax0 = channel(c, xe)
        adds.append(o); subs.append(i)
        outlets.append([round(ax0.X, 2), round(ax0.Y + CONE + SPIGOT_L, 2), round(ax0.Z, 2)])
    part = base
    for a in adds:
        part = part + a
    for s in subs:
        part = part - s
    for bx, by in BOLT_BORES:
        cyl = Cylinder(BORE_R, 300, align=(Align.CENTER, Align.CENTER, Align.MIN)).translate((bx, by, 70))
        part = part - cyl
    part = part.solids()[0] if len(part.solids()) == 1 else part
    print("solids", len(part.solids()), "vol cm3", round(part.volume / 1000, 1))
    bb = part.bounding_box(); print("bbox", bb.min, bb.max)
    export_step(part, f"{OUT}/Ansaugung_v2_komplett.step")
    export_stl(part, f"{OUT}/Ansaugung_v2_komplett.stl", tolerance=0.03, angular_tolerance=0.1)
    sp = Plane(origin=(SPLIT_X, 0, 0), z_dir=(1, 0, 0))
    right = part.split(sp, keep=Keep.TOP)
    left = part.split(sp, keep=Keep.BOTTOM)
    info = {"throttle_x": [round(x, 2) for x in THROTTLE_X], "outlet_tips": outlets}
    for name, p in (("Zyl-Haelfte_A_x-plus", right), ("Zyl-Haelfte_B_x-minus", left)):
        b = p.bounding_box()
        info[name] = {"solids": len(p.solids()), "vol_cm3": round(p.volume / 1000, 1),
                      "size": [round(b.size.X, 1), round(b.size.Y, 1), round(b.size.Z, 1)]}
        export_step(p, f"{OUT}/Ansaugung_v2_{name}.step")
        export_stl(p, f"{OUT}/Ansaugung_v2_{name}.stl", tolerance=0.03, angular_tolerance=0.1)
    print(json.dumps(info, indent=1))


main()
