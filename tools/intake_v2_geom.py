"""Intake runner adapter v2-SLS: same routing as v2, channels rebuilt from the flange top with a thin wall."""
import math, sys, json
from build123d import *

SRC = "/srv/data/obsidian/vault/Buggy/Ansaugung/Ansaugung.step"
OUT = sys.argv[1] if len(sys.argv) > 1 else "out_sls"

WALL = 4.0                # SLS wall in the runners (FDM v2: 6.8)
CY, CZ, R = 125.55, -20.67, 133.19          # original bend, fitted from the STEP centreline
FLANGE_TOP = 23.0
PHI0 = math.degrees(math.asin((FLANGE_TOP - CZ) / R))
PHI_SHIFT = 45.0          # lateral fan-out starts here (keeps socket bores clear)
PHI_END = 90.0
TRANSITION = 45.0         # mm of path length for port shape -> circle

# ports: (x, y, width_x, height_y, corner_r) measured in the STEP flange
PORTS = [(65.2, 1.3, 32.2, 37.8, 11.5), (0.0, 0.0, 38.2, 30.2, 8.5), (-65.2, 1.3, 32.2, 37.8, 11.5),
         (-130.4, 0.0, 38.2, 30.2, 8.5), (-195.6, 1.3, 32.2, 37.8, 11.5), (-260.8, 0.0, 38.2, 30.2, 8.5)]
PORT_MID = -97.8
THROTTLE_X = [PORT_MID + (2.5 - k) * 379.3 / 5 for k in range(6)]

R_IN = 17.8
R_OUT = R_IN + WALL
STRAIGHT = 6.0
CONE = 4.0
SPIGOT_L, SPIGOT_OD, SPIGOT_ID = 22.0, 41.0, 35.0
BEAD_OD, BEAD_POS, BEAD_W = 42.4, 4.0, 3.0
TIP_FLARE = 0.0
TEARDROP = None
SPLIT_X = PORT_MID
BOLT_BORES = [(-226.6, 16.5), (-164.6, 16.5), (-34.0, 16.5), (34.0, 16.5)]
NIPPLE_BOX = ((-143.0, -29.0, 38.0), (-117.0, -1.0, 86.0))
N_PTS = 48


def smoothstep(u):
    u = min(1.0, max(0.0, u))
    return u * u * u * (u * (6 * u - 15) + 10)


def rrect_radius(theta, w, h, r):
    """distance from centre to a rounded rectangle boundary along direction theta"""
    c, s = math.cos(theta), math.sin(theta)
    hx, hy = w / 2, h / 2
    lo, hi = 0.0, max(w, h)
    for _ in range(40):
        m = (lo + hi) / 2
        x, y = abs(c * m), abs(s * m)
        qx, qy = x - (hx - r), y - (hy - r)
        inside = (x <= hx and y <= hy) and not (qx > 0 and qy > 0 and qx * qx + qy * qy > r * r)
        lo, hi = (m, hi) if inside else (lo, m)
    return lo


def centreline(port, x_end, n=90):
    x0, y0 = port[0], port[1]
    a_y = CY - R * math.cos(math.radians(PHI0))
    pts = []
    for i in range(n + 1):
        phi = PHI0 + (PHI_END - PHI0) * i / n
        p = math.radians(phi)
        y = CY - R * math.cos(p) + (y0 - a_y) * (1 - smoothstep((phi - PHI0) / 15))
        z = CZ + R * math.sin(p)
        x = x0 + (x_end - x0) * smoothstep((phi - PHI_SHIFT) / (PHI_END - PHI_SHIFT))
        pts.append(Vector(x, y, z))
    return pts


def section(centre, tangent, port, blend, grow):
    w, h, r = port[2] + 2 * grow, port[3] + 2 * grow, port[4] + grow
    rc = R_IN + grow
    n = Vector(tangent).normalized()
    u = Vector(1, 0, 0) - n * n.X
    u = u.normalized()
    v = n.cross(u)
    # FDM: self-supporting roof (truncated 45 deg teardrop) on the inner bore where the runner lies flatter than 45 deg
    td_w = 0.0
    if TEARDROP:
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, n.Z))))
        td_w = smoothstep((tilt - 35.0) / 15.0) * smoothstep((TEARDROP["y_fade1"] - centre.Y) / (TEARDROP["y_fade1"] - TEARDROP["y_fade0"]))
    pts = []
    for k in range(N_PTS):
        th = 2 * math.pi * k / N_PTS
        rr = (1 - blend) * rrect_radius(th, w, h, r) + blend * rc
        if td_w > 0 and grow > 0:
            # outer: up to 2 mm extra wall over the teardrop roof, but never above z 108 (user limit)
            a = abs((th + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi)
            extra = min(2.0, max(0.0, 107.8 - (centre.Z + rc)))
            if a < math.pi / 3:
                rr += td_w * blend * extra * 0.5 * (1 + math.cos(a * 3))
        elif td_w > 0:
            a = abs((th + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi)  # angle from 'up' (= -v)
            if a < math.pi / 4:
                Hh = TEARDROP["H"]
                if "H_end" in TEARDROP:
                    Hh += (TEARDROP["H_end"] - TEARDROP["H"]) * smoothstep((centre.Y - TEARDROP["y_h0"]) / (TEARDROP["y_h1"] - TEARDROP["y_h0"]))
                td = min(rc / math.cos(math.pi / 4 - a), Hh / math.cos(a))
                rr += td_w * blend * (td - rc)
        pts.append(centre + u * (rr * math.cos(th)) + v * (rr * math.sin(th)))
    return Wire([Spline(*pts, periodic=True)])


def runner(port, x_end, grow, extra_below=0.0):
    pts = centreline(port, x_end)
    secs, s = [], 0.0
    last_s = -99
    for i, p in enumerate(pts):
        if i:
            s += (p - pts[i - 1]).length
        if i not in (0, len(pts) - 1) and s - last_s < 7.0:
            continue
        last_s = s
        t = Vector(0, 0, 1) if i == 0 else (pts[min(i + 1, len(pts) - 1)] - pts[i - 1])
        if i == len(pts) - 1:
            t = Vector(0, 1, 0)
        blend = smoothstep(s / TRANSITION)
        secs.append(section(p, t, port, blend, grow))
    body = loft([Face(w) for w in secs])
    if extra_below:
        w, h, r = port[2] + 2 * grow, port[3] + 2 * grow, port[4] + grow
        plug = Pos(port[0], port[1], FLANGE_TOP - extra_below) * extrude(RectangleRounded(w + 0.2, h + 0.2, r + 0.1), amount=extra_below + 0.4)
        return [body, plug], pts[-1]
    return body, pts[-1]


def spigot(ax0, solid):
    if solid:
        L = CONE + SPIGOT_L
        b0 = L - BEAD_POS - BEAD_W / 2
        prof = [(-0.5, R_OUT), (STRAIGHT, R_OUT), (STRAIGHT + CONE, SPIGOT_OD / 2), (STRAIGHT + b0 - 1.2, SPIGOT_OD / 2),
                (STRAIGHT + b0, BEAD_OD / 2), (STRAIGHT + b0 + BEAD_W, BEAD_OD / 2), (STRAIGHT + b0 + BEAD_W + 1.2, SPIGOT_OD / 2),
                (STRAIGHT + L, SPIGOT_OD / 2), (STRAIGHT + L, 0), (-0.5, 0)]
        face = make_face(Plane.XY * Polyline(*prof, close=True))
        return revolve(face, Axis.X, 360).rotate(Axis.Z, 90).translate(ax0)
    inner = Cylinder(R_IN, STRAIGHT + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)).rotate(Axis.X, -90).translate(ax0 - Vector(0, 1, 0))
    cone = Cone(R_IN, SPIGOT_ID / 2, CONE, align=(Align.CENTER, Align.CENTER, Align.MIN)).rotate(Axis.X, -90).translate(ax0 + Vector(0, STRAIGHT, 0))
    tube = Cylinder(SPIGOT_ID / 2, SPIGOT_L + 1, align=(Align.CENTER, Align.CENTER, Align.MIN)).rotate(Axis.X, -90).translate(ax0 + Vector(0, STRAIGHT + CONE, 0))
    if TIP_FLARE:
        # entry flare at the spigot mouth: softens the step from a larger hose bore into the spigot
        tip = Cone(SPIGOT_ID / 2, SPIGOT_ID / 2 + TIP_FLARE, TIP_FLARE * 1.5 + 0.01, align=(Align.CENTER, Align.CENTER, Align.MIN)).rotate(Axis.X, -90)
        tube = tube + tip.translate(ax0 + Vector(0, STRAIGHT + CONE + SPIGOT_L - TIP_FLARE * 1.5, 0))
    return inner + cone + tube


def main():
    src = import_step(SRC)
    flange = src.split(Plane.XY.offset(FLANGE_TOP), keep=Keep.BOTTOM)
    (bx0, by0, bz0), (bx1, by1, bz1) = NIPPLE_BOX
    nipple = src & Pos((bx0 + bx1) / 2, (by0 + by1) / 2, (bz0 + bz1) / 2) * Box(bx1 - bx0, by1 - by0, bz1 - bz0)
    part = flange + nipple
    voids, tips = [], []
    for port, xe in zip(PORTS, THROTTLE_X):
        outer, end = runner(port, xe, WALL)
        inner, _ = runner(port, xe, 0.0, extra_below=3.0)
        part = part + outer + spigot(end, True)
        voids += inner + [spigot(end, False)]
        tips.append([round(end.X, 2), round(end.Y + STRAIGHT + CONE + SPIGOT_L, 2), round(end.Z, 2)])
    for v in voids:
        part = part - v
    for bx, by in BOLT_BORES:
        part = part - Pos(bx, by, FLANGE_TOP) * Cylinder(8.0, 300, align=(Align.CENTER, Align.CENTER, Align.MIN))
    info = {"wall": WALL, "solids": len(part.solids()), "vol_cm3": round(part.volume / 1000, 1), "tips": tips}
    export_step(part, f"{OUT}/Ansaugung_v2_SLS_komplett.step")
    export_stl(part, f"{OUT}/Ansaugung_v2_SLS_komplett.stl", tolerance=0.03, angular_tolerance=0.1)
    sp = Plane(origin=(SPLIT_X, 0, 0), z_dir=(1, 0, 0))
    for name, keep in (("Haelfte_A_x-plus", Keep.TOP), ("Haelfte_B_x-minus", Keep.BOTTOM)):
        p = part.split(sp, keep=keep)
        b = p.bounding_box()
        info[name] = {"solids": len(p.solids()), "vol_cm3": round(p.volume / 1000, 1),
                      "size": [round(b.size.X, 1), round(b.size.Y, 1), round(b.size.Z, 1)]}
        export_step(p, f"{OUT}/Ansaugung_v2_SLS_{name}.step")
        export_stl(p, f"{OUT}/Ansaugung_v2_SLS_{name}.stl", tolerance=0.03, angular_tolerance=0.1)
    print(json.dumps(info, indent=1))


if __name__ == "__main__":
    main()
