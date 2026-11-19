"""Intake adapter v2 variants with mesh booleans (manifold3d).
sls : wall 4 mm, two halves (split between cyl 3/4)
cast: wall 6 mm, six single runners each with its own flange segment, shrink + machining allowance
usage: intake_v2_variants.py <fdm|fdm_kurz|fdm_opt|fdm_power|fdm_modular|sls|sls_power|cast> <outdir> [--step]
--step (fdm_modular only): additionally build the part as BREP with build123d and export STEP
run long jobs via run_watchdog.sh, e.g. run_watchdog.sh 900 300 out/log.txt -- python intake_v2_variants.py fdm_modular out --step
(normal: ~2.5 min mesh only, ~6 min with --step; OCC booleans can hang on near-coincident surfaces)
"""
import sys, os, json, math, tempfile
import numpy as np
import trimesh
import manifold3d as mf
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import intake_v2_geom as G
from build123d import (import_step, export_stl, export_step, Plane, Keep, Pos, Box, extrude, RectangleRounded,
                       Solid, Edge, Wire, Face, Axis, Vector)

MODE, OUT = sys.argv[1], sys.argv[2]
STEP = "--step" in sys.argv[3:]
os.makedirs(OUT, exist_ok=True)
CFG = {
    "sls": dict(wall=4.0, spigot_od=41.0, cone=4.0, tag="SLS", eyes=True),
    "fdm": dict(wall=6.8, spigot_od=41.0, cone=9.0, tag="FDM"),
    "fdm_opt": dict(wall=6.8, spigot_od=41.0, cone=9.0, tag="FDM_opt", opt=dict(r=78.4, straight=15.0)),
    "fdm_kurz": dict(wall=6.8, spigot_od=41.0, cone=9.0, tag="FDM_kurz", compact=dict(r=89.7, straight=25.0)),
    # power layout: bore 38 (~ head port area), spigot 48.3/40 for a 48 mm hose; K100 now via 48->40 reducer hose, 45 mm throttles later
    "fdm_power": dict(wall=6.8, r_in=19.0, spigot_od=48.3, spigot_id=40.0, bead_od=49.6, tip_flare=2.0, cone=9.0, tag="FDM_Leistung", opt=dict(r=77.0, straight=15.0)),
    "sls_power": dict(wall=4.0, r_in=19.0, spigot_od=48.3, spigot_id=40.0, bead_od=49.6, tip_flare=2.0, cone=6.0, tag="SLS_Leistung", opt=dict(r=77.0, straight=15.0),
                      eyes=dict(w=40.0, y0=68.0, y1=92.0, z0=84.0, z1=108.0, hole_d=8.4, nut_af=13.0, nut_h=7.0)),
    # modular: base with a small bolt flange per runner, screw-on spigot adapters (K100 41 mm now, 48 mm later)
    "fdm_modular": dict(wall=6.8, r_in=19.0, spigot_od=41.0, spigot_id=35.0, cone=9.0, tag="FDM_Modular", opt=dict(r=77.0, straight=15.0), modular=True,
                        plate=True),
    "cast": dict(wall=6.0, spigot_od=41.6, shrink=1.013, face_allow=1.0, hole_cast_d=11.2),
}[MODE]
G.SRC = "/srv/data/obsidian/vault/Buggy/Ansaugung/Ansaugung.step"
G.WALL = CFG["wall"]
G.R_IN = CFG.get("r_in", G.R_IN)
G.SPIGOT_ID = CFG.get("spigot_id", G.SPIGOT_ID)
G.BEAD_OD = CFG.get("bead_od", G.BEAD_OD)
G.TIP_FLARE = CFG.get("tip_flare", 0.0)
G.R_OUT = G.R_IN + G.WALL
G.SPIGOT_OD = CFG["spigot_od"]
G.CONE = CFG.get("cone", G.CONE)
TMP = tempfile.mkdtemp()
# mesh resolution: BREP tessellation and polygon count of the mesh-boolean cylinders
TESS_TOL, TESS_ANG = 0.01, 0.03      # mm chord error, rad (~1.7 deg)
SEG_LEN = 0.4                        # mm max facet length on round features


def nseg(r):
    return int(min(720, max(96, math.ceil(2 * math.pi * r / SEG_LEN))))


def to_mf(shape):
    p = os.path.join(TMP, "t.stl")
    export_stl(shape, p, tolerance=TESS_TOL, angular_tolerance=TESS_ANG)
    m = trimesh.load(p)
    m.merge_vertices()
    if not m.is_watertight:
        trimesh.repair.fill_holes(m)
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))


def save(man, path, keep_all=False):
    parts = man.decompose()
    if not keep_all:
        man = max(parts, key=lambda x: x.volume())
    me = man.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(me.vert_properties)[:, :3], faces=np.asarray(me.tri_verts), process=False)
    tm.export(path)
    chk = trimesh.load(path)
    nb = len(chk.split(only_watertight=False))
    if not chk.is_watertight or (not keep_all and nb != 1):
        raise RuntimeError(f"export check failed for {path}: watertight={chk.is_watertight}, bodies={nb}")
    b = tm.bounds
    return {"vol_cm3": round(tm.volume / 1000, 1), "watertight": bool(tm.is_watertight),
            "bodies": len(tm.split(only_watertight=False)), "size": [round(float(v), 1) for v in b[1] - b[0]]}


src = import_step(G.SRC)
flange_b = src.split(Plane.XY.offset(G.FLANGE_TOP), keep=Keep.BOTTOM)
flange = to_mf(flange_b)
(bx0, by0, bz0), (bx1, by1, bz1) = G.NIPPLE_BOX
nipple_b = src & Pos((bx0 + bx1) / 2, (by0 + by1) / 2, (bz0 + bz1) / 2) * Box(bx1 - bx0, by1 - by0, bz1 - bz0)
nipple = to_mf(nipple_b)



def centreline_compact(port, x_end, n=90):
    """tighter 90 deg bend from the flange top, then a straight run; lateral fan-out only once the runner
    has cleared the bolt access columns (ends 1/6: y >= 34, others: y >= 45.3)"""
    c = CFG["compact"]
    r, L = c["r"], c["straight"]
    phi0 = math.radians(G.PHI0)
    y0a = G.CY - G.R * math.cos(phi0)
    cy, cz = y0a + r * math.cos(phi0), G.FLANGE_TOP - r * math.sin(phi0)
    pts = []
    for i in range(n + 1):
        ph = phi0 + (math.pi / 2 - phi0) * i / n
        blend = 1 - G.smoothstep((math.degrees(ph) - G.PHI0) / 15)
        pts.append([port[0], cy - r * math.cos(ph) + (port[1] - y0a) * blend, cz + r * math.sin(ph)])
    for i in range(1, 26):
        pts.append([port[0], cy + L * i / 25, cz + r])
    s = [0.0]
    for a, b in zip(pts, pts[1:]):
        s.append(s[-1] + math.dist(a, b))
    y_thr = 34.0 if port[0] in (65.2, -260.8) else 45.3
    s0 = next(si for si, p in zip(s, pts) if p[1] >= y_thr)
    for si, p in zip(s, pts):
        p[0] = port[0] + (x_end - port[0]) * G.smoothstep((si - s0) / (s[-1] - s0))
    return [Vector(*p) for p in pts]


FACE_Z = 5.0
BORE_COLS = [(-226.6, 16.5), (-164.6, 16.5), (-34.0, 16.5), (34.0, 16.5)]
EAR_BOLTS = [(98.5, 1.3), (-297.1, 1.3)]


def _dists(pts, xs, port):
    """horizontal distances from bolt columns to sampled channel walls above the flange (inner->socket bores, outer->end bolt heads)"""
    out = []
    for i, (p, x) in enumerate(zip(pts, xs)):
        if p[2] < 23.0:
            continue
        a, b = pts[max(i - 1, 0)], pts[min(i + 1, len(pts) - 1)]
        t = np.array([b[0] - a[0], b[1] - a[1], b[2] - a[2]]); t /= np.linalg.norm(t)
        u = np.cross(t, [1.0, 0, 0]); u /= np.linalg.norm(u); v = np.cross(t, u)
        bl = G.smoothstep((p[2] - FACE_Z) / G.TRANSITION)
        r_in = (1 - bl) * max(port[2], port[3]) / 2 + bl * G.R_IN
        for rr, cols in ((r_in, BORE_COLS), (r_in + G.WALL, EAR_BOLTS)):
            for k in range(16):
                th = 2 * math.pi * k / 16
                q = np.array([x, p[1], p[2]]) + rr * (math.cos(th) * u + math.sin(th) * v)
                for bx, by in cols:
                    out.append(math.hypot(q[0] - bx, q[1] - by) if q[2] >= 23.0 else 1e9)
    return np.array(out)


def _clear(pts, xs, port, base):
    d = _dists(pts, xs, port)
    per = 16 * len(BORE_COLS) + 16 * len(EAR_BOLTS)
    need = np.array(([12.6] * (16 * len(BORE_COLS)) + [10.0] * (16 * len(EAR_BOLTS))) * (len(d) // per))
    return bool(np.all(d >= np.minimum(need, base - 0.05)))


def centreline_opt(port, x_end, n=90):
    """tangent-continuous bend starting at the flange face (vertical), radius r to 90 deg, short straight;
    lateral fan-out starts as early as the bolt access allows"""
    c = CFG["opt"]
    r, L = c["r"], c["straight"]
    pts = [[port[0], port[1], FACE_Z - 1.5]]
    for i in range(n + 1):
        ph = (math.pi / 2) * i / n
        blend = 1 - G.smoothstep(math.degrees(ph) / 15)
        pts.append([port[0], r * (1 - math.cos(ph)) + port[1] * blend, FACE_Z + r * math.sin(ph)])
    for i in range(1, 16):
        pts.append([port[0], r + L * i / 15, FACE_Z + r])
    s = [0.0]
    for a, b in zip(pts, pts[1:]):
        s.append(s[-1] + math.dist(a, b))
    base = _dists(pts, [port[0]] * len(pts), port)
    for s_a in np.arange(0, s[-1] - 30, 2.0):
        xs = [port[0] + (x_end - port[0]) * G.smoothstep((si - s_a) / (s[-1] - s_a)) for si in s]
        if _clear(pts, xs, port, base):
            break
    else:
        raise RuntimeError("no clear shift profile for port %s" % port[0])
    print("  shift start s=%.0f of %.0f mm" % (s_a, s[-1]), flush=True)
    return [Vector(x, p[1], p[2]) for x, p in zip(xs, pts)]


if CFG.get("compact"):
    G.centreline = centreline_compact
    G.STRAIGHT = 2.0
if CFG.get("opt"):
    G.centreline = centreline_opt
    G.STRAIGHT = 2.0
    # fill the old vertical port bores, the new curved bore is cut from the face upwards
    flange0 = flange
    for port in G.PORTS:
        # fill the old vertical bore only above the face-side pocket (z 10.5); the original port collars below stay untouched
        fill = Pos(port[0], port[1], 10.6) * extrude(RectangleRounded(port[2] + 0.4, port[3] + 0.4, port[4] + 0.2), amount=23.0 - 10.6)
        flange += to_mf(fill)
        flange_b = flange_b + fill
    G.FLANGE_TOP = FACE_Z
    FACE_POCKET = None
runners = []
runners_b = []
ENDS = []
for port, xe in zip(G.PORTS, G.THROTTLE_X):
    outer, end = G.runner(port, xe, G.WALL)
    inner, _ = G.runner(port, xe, 0.0, extra_below=0.0 if CFG.get("opt") else 3.0)
    if CFG.get("opt"):
        inner = [inner, mf.Manifold()]
    w, h, r = port[2] + 2 * G.WALL, port[3] + 2 * G.WALL, port[4] + G.WALL
    add = to_mf(outer)
    if CFG.get("opt"):
        add = add.trim_by_plane((0, 0, 1), 22.5)  # below the flange top the (filled) flange plate is the wall; avoids grazing the bolt bosses
    add += (mf.Manifold() if CFG.get("modular") else to_mf(G.spigot(end, True)))
    if not CFG.get("opt"):
        add += to_mf(Pos(port[0], port[1], G.FLANGE_TOP - 1.5) * extrude(RectangleRounded(w, h, r), amount=1.5))
    sub = to_mf(inner[0]) + (inner[1] if isinstance(inner[1], mf.Manifold) else to_mf(inner[1])) + (mf.Manifold() if CFG.get("modular") else to_mf(G.spigot(end, False)))
    runners.append((add, sub))
    runners_b.append((outer, inner[0]))
    ENDS.append((end.X, end.Y, end.Z))
    print("runner", port[0], "ok", flush=True)

bores = mf.Manifold()
for bx, by in G.BOLT_BORES:
    bores += mf.Manifold.cylinder(300, 8.0, 8.0, nseg(8.0)).translate((bx, by, 23.0))

# vacuum nipple on runner 4: re-drill through the new (thinner) wall along the original nipple axis
_t0, _t1 = 40.0, 74.0
nipple_bore = (mf.Manifold.cylinder(_t1 - _t0, 3.0, 3.0, nseg(3.0)).translate((0, 0, _t0))
               .rotate((55.5, 0, 0)).translate((-130.4, 48.3, 31.3)))
info = {"mode": MODE, "cfg": CFG}
# support eyes on the outer side of the bends (face away from the head, +z), bridging runner pairs 1-2, 3-4, 5-6.
# bolt from outside along +z into a captured M8 nut that sits in a hex pocket on the head side (open into the runner gap)
EYE = dict(w=40.0, y0=96.0, y1=120.0, z0=114.0, z1=138.0, hole_d=8.4, nut_af=13.0, nut_h=7.0)
if isinstance(CFG.get("eyes"), dict):
    EYE = CFG["eyes"]
EYE_X = [(G.THROTTLE_X[k] + G.THROTTLE_X[k + 1]) / 2 for k in (0, 2, 4)]
# ---- modular joint: base flange + screw-on spigot adapter (axis +y at each runner end)
# review 2026-11-13 (Codex + Fable): no centring ring (unprintable lying, no FDM margin) -> located by the 4 bolts;
# M5 heat-set inserts (hole 6.4) instead of downward-open nut pockets; plate 10 mm.
# 2026-11-19 (user): bore fully round everywhere, no teardrop roof; internal supports are accepted.
J = dict(fw=62.0, fd=52.0, ft=10.0, adp_ft=8.0, bore_r=19.0,
         groove=(21.0, 23.7, 1.5), bolt=(24.0, 18.0), hole_base=6.4, hole_adp=5.6, neck_l=14.0, spigot_l=22.0, strut_hole=6.6)


TAB = dict(w=24.0, zc=120.0, hole=6.6, fillet=20.0)  # hole at z 120 (12 mm beyond the plate edge), round end r 12, root fillets r 20


def ytab(x, y0, y1, z_edge, n=96):
    """support tab in the plate plane: concave root fillets blend into the plate edge, straight flanks, round end"""
    hw, R, zc = TAB["w"] / 2, TAB["fillet"], TAB["zc"]
    Rz = min(R, zc - z_edge)                       # elliptical fillet: R wide, never taller than the tab flank
    pts = [(-(hw + R) - 2.0, z_edge - 1.0), (-(hw + R), z_edge)]
    for k in range(1, n + 1):                      # left fillet (quarter ellipse), tangent to plate edge and tab flank
        t = (math.pi / 2) * k / n
        pts.append((-(hw + R) + R * math.sin(t), z_edge + Rz - Rz * math.cos(t)))
    if z_edge + Rz < zc:
        pts.append((-hw, zc))
    for k in range(1, 2 * n):                      # round end, centre (0, zc)
        t = math.pi - math.pi * k / (2 * n)
        pts.append((hw * math.cos(t), zc + hw * math.sin(t)))
    if z_edge + Rz < zc:
        pts.append((hw, zc))
    for k in range(n, -1, -1):                     # right fillet mirrored
        t = (math.pi / 2) * k / n
        pts.append(((hw + R) - R * math.sin(t), z_edge + Rz - Rz * math.cos(t)))
    pts.append(((hw + R) + 2.0, z_edge - 1.0))
    cs = mf.CrossSection([[(px, -(pz - z_edge)) for px, pz in pts]])
    if cs.area() < 0:
        cs = mf.CrossSection([[(px, -(pz - z_edge)) for px, pz in reversed(pts)]])
    return mf.Manifold.extrude(cs, y1 - y0).rotate((-90, 0, 0)).translate((x, y0, z_edge))


def ycyl(r0, y0, y1, x, z, r1=None):
    c = mf.Manifold.cylinder(y1 - y0, r0, r0 if r1 is None else r1, nseg(max(r0, r1 or 0)))
    return c.rotate((-90, 0, 0)).translate((x, y0, z))


def ybox(x, y0, y1, z, w, d):
    return mf.Manifold.cube((w, y1 - y0, d)).translate((x - w / 2, y0, z - d / 2))


def bolt_xy(x, z):
    bx, bz = J["bolt"]
    return [(x + sx * bx, z + sz * bz) for sx in (-1, 1) for sz in (-1, 1)]


def base_flange(x, y, z):
    add = ybox(x, y - J["ft"], y, z, J["fw"], J["fd"])
    sub = ycyl(J["bore_r"] + 0.05, y - J["ft"] - 1, y + 1, x, z)
    g0, g1, gd = J["groove"]
    sub += ycyl(g1, y - gd, y + 1, x, z) - ycyl(g0, y - gd - 1, y + 2, x, z)
    for bx, bz in bolt_xy(x, z):
        sub += ycyl(J["hole_base"] / 2, y - J["ft"] - 1, y + 1, bx, bz)
    return add, sub


def adapter(x, y, z, od, idd, bead_od, flare):
    """screw-on spigot: flat flange (prints face-down), 45 deg shoulder, neck, 45 deg step to the spigot, bead;
    round bore tapers to the spigot bore along the neck"""
    ft, nl, sl = J["adp_ft"], J["neck_l"], J["spigot_l"]
    neck_r = max(23.0, od / 2 + 1.5)
    a = ybox(x, y, y + ft, z, J["fw"], J["fd"])
    sh_r = min(neck_r + 4.0, J["fd"] / 2 - 0.5)                                  # shoulder never wider than the flange
    if sh_r > neck_r + 0.05:
        a += ycyl(sh_r, y + ft - 0.1, y + ft + (sh_r - neck_r), x, z, neck_r)     # 45 deg shoulder flange -> neck
    a += ycyl(neck_r, y + ft - 0.1, y + ft + nl, x, z)
    step = neck_r - od / 2
    a += ycyl(neck_r, y + ft + nl - 0.1, y + ft + nl + step, x, z, od / 2)       # 45 deg neck -> spigot
    a += ycyl(od / 2, y + ft + nl + step - 0.1, y + ft + nl + sl, x, z)
    tip = y + ft + nl + sl
    b0 = tip - 4.0 - 1.5
    a += ycyl(od / 2, b0 - 1.5, b0, x, z, bead_od / 2) + ycyl(bead_od / 2, b0, b0 + 3.0, x, z) + ycyl(bead_od / 2, b0 + 3.0, b0 + 4.5, x, z, od / 2)
    v = ycyl(J["bore_r"] + 0.05, y - 1, y + ft, x, z)
    v += ycyl(J["bore_r"], y + ft - 0.01, y + ft + nl, x, z, idd / 2)
    v += ycyl(idd / 2, y + ft + nl - 0.01, tip + 1, x, z)
    if flare:
        v += ycyl(idd / 2, tip - 1.5 * flare, tip + 0.01, x, z, idd / 2 + flare)
    for bx, bz in bolt_xy(x, z):
        v += ycyl(J["hole_adp"] / 2, y - 1, y + ft + 1, bx, bz)
    return a - v


if MODE in ("sls", "fdm", "fdm_kurz", "fdm_opt", "fdm_power", "sls_power", "fdm_modular"):
    part = flange + nipple
    if CFG.get("eyes"):
        for ex in EYE_X:
            part += mf.Manifold.cube((EYE["w"], EYE["y1"] - EYE["y0"], EYE["z1"] - EYE["z0"])).translate((ex - EYE["w"] / 2, EYE["y0"], EYE["z0"]))
    for a, _ in runners:
        part += a
    if CFG.get("modular"):
        for (ex, ey, ez) in ENDS:
            fa, _ = base_flange(ex, ey, ez)
            part += fa
        if CFG.get("plate"):
            # one continuous flange plate per half (runners 1-3 / 4-6), never across the split
            for grp in (ENDS[:3], ENDS[3:]):
                xs = [e[0] for e in grp]
                x0, x1 = min(xs) - J["fw"] / 2, max(xs) + J["fw"] / 2
                ey, ez = grp[0][1], grp[0][2]
                part += ybox((x0 + x1) / 2, ey - J["ft"], ey, ez, x1 - x0, J["fd"])
            # support tabs (user 2026-11-13): plate extended away from the head (+z) between the runners,
            # hole outside the runner/adapter envelope so a bar can be bolted on from outside
            TAB_X = [(ENDS[k][0] + ENDS[k + 1][0]) / 2 for k in (0, 1, 3, 4)]
            ey, ez = ENDS[0][1], ENDS[0][2]
            for tx in TAB_X:
                part += ytab(tx, ey - J["ft"], ey, ez + J["fd"] / 2)
    for _, s in runners:
        part -= s
    if CFG.get("modular"):
        for (ex, ey, ez) in ENDS:
            _, fs = base_flange(ex, ey, ez)
            part -= fs
        if CFG.get("plate"):
            for tx in TAB_X:
                part -= ycyl(TAB["hole"] / 2, ENDS[0][1] - J["ft"] - 1, ENDS[0][1] + 1, tx, TAB["zc"])
            info["support_tabs"] = dict(TAB, x=[round(x, 2) for x in TAB_X])
    part -= bores
    if CFG.get("eyes"):
        yc = (EYE["y0"] + EYE["y1"]) / 2
        for ex in EYE_X:
            part -= mf.Manifold.cylinder(60, EYE["hole_d"] / 2, EYE["hole_d"] / 2, nseg(EYE["hole_d"] / 2)).translate((ex, yc, EYE["z0"] - 20))
            nut = mf.Manifold.extrude(mf.CrossSection.circle(EYE["nut_af"] / 2 / 0.8660254 + 0.2, 6), EYE["nut_h"] + 20)
            part -= nut.rotate((0, 0, 30)).translate((ex, yc, EYE["z0"] - 20))
        info["eyes"] = dict(EYE, x=[round(x, 1) for x in EYE_X])
    part -= nipple_bore
    info["komplett"] = save(part, f"{OUT}/Ansaugung_v2_{CFG['tag']}_komplett.stl")
    if CFG.get("modular"):
        for name, od, idd, bead, flare in (("K100_Tuelle41", 41.0, 35.0, 43.0, 1.0), ("48er_Tuelle48", 48.3, 40.0, 50.3, 1.5)):
            allsix = mf.Manifold()
            for (ex, ey, ez) in ENDS:
                allsix += adapter(ex, ey, ez, od, idd, bead, flare)
            info["adapter_" + name + "_alle6"] = save(allsix, f"{OUT}/Ansaugung_v2_Adapter_{name}_alle6_Einbaulage.stl", keep_all=True)
            one = adapter(0.0, 0.0, 0.0, od, idd, bead, flare).rotate((90, 0, 0))
            info["adapter_" + name] = save(one, f"{OUT}/Ansaugung_v2_Adapter_{name}_1Stueck_Druck.stl")
        info["ends"] = [[round(v, 2) for v in e] for e in ENDS]
    a = part.trim_by_plane((1, 0, 0), G.SPLIT_X)
    b = part.trim_by_plane((-1, 0, 0), -G.SPLIT_X)
    info["A"] = save(a, f"{OUT}/Ansaugung_v2_{CFG['tag']}_Haelfte_A_x-plus.stl")
    info["B"] = save(b, f"{OUT}/Ansaugung_v2_{CFG['tag']}_Haelfte_B_x-minus.stl")
else:
    # flange segment borders = head bolts between the ports
    borders = [400, 34.0, -34.0, -96.2, -164.6, -226.6, -400]
    bolt_holes = [(-297.1, 1.3), (-226.6, -13.9), (-226.6, 16.5), (-164.6, 16.5), (-96.2, -13.9),
                  (-34.0, 16.5), (34.0, -13.9), (34.0, 16.5), (98.5, 1.3)]
    for i, (a, s) in enumerate(runners):
        hi, lo = borders[i], borders[i + 1]
        slab = mf.Manifold.cube((hi - lo, 400, 400)).translate((lo, -200, -100))
        seg = flange ^ slab
        # machining allowance on the head face: copy the bottom 1 mm slice 1 mm down
        sl = seg ^ mf.Manifold.cube((1000, 1000, CFG["face_allow"])).translate((-500, -500, 5.0))
        seg = seg + sl.translate((0, 0, -CFG["face_allow"]))
        # cast bolt holes undersize, drill to size afterwards
        for hx, hy in bolt_holes:
            if lo - 7 < hx < hi + 7:
                seg += (mf.Manifold.cylinder(18, 6.6, 6.6, nseg(6.6)).translate((hx, hy, 10.5)) ^ slab)
                seg -= mf.Manifold.cylinder(30, CFG["hole_cast_d"] / 2, CFG["hole_cast_d"] / 2, nseg(CFG["hole_cast_d"] / 2)).translate((hx, hy, 0))
        seg += a
        if i == 3:
            seg += nipple ^ slab
        seg -= s
        seg -= bores
        if i == 3:
            seg -= nipple_bore
        cx = (G.PORTS[i][0])
        seg = seg.translate((-cx, 0, 0)).scale((CFG["shrink"],) * 3).translate((cx, 0, 0))
        info[f"Rohr_{i + 1}"] = save(seg, f"{OUT}/Ansaugung_v2_Guss_Rohr{i + 1}_port_x{cx:+.1f}.stl")
# ---- BREP (STEP) build of fdm_modular: same construction as the mesh booleans above, with exact cylinders/cones
def _yplane(x, y, z):
    return Plane(origin=(x, y, z), x_dir=(1, 0, 0), z_dir=(0, 1, 0))


def b_ycyl(r0, y0, y1, x, z, r1=None):
    if r1 is None or abs(r1 - r0) < 1e-9:
        return Solid.make_cylinder(r0, y1 - y0, _yplane(x, y0, z))
    return Solid.make_cone(r0, r1, y1 - y0, _yplane(x, y0, z))


def b_ybox(x, y0, y1, z, w, d):
    return Pos(x, (y0 + y1) / 2, z) * Box(w, y1 - y0, d)


def b_ytab(x, y0, y1, z_edge):
    hw, R, zc = TAB["w"] / 2, TAB["fillet"], TAB["zc"]
    Rz = min(R, zc - z_edge)
    P = lambda px, pz: Vector(x + px, y0, pz)
    # ellipse plane: local x = world x, local y = world z
    ep = lambda cx, cz: Plane(origin=(x + cx, y0, cz), x_dir=(1, 0, 0), z_dir=(0, -1, 0))
    e = [Edge.make_line(P(-(hw + R) - 2.0, z_edge - 1.0), P(-(hw + R), z_edge)),
         Edge.make_ellipse(R, Rz, ep(-(hw + R), z_edge + Rz), -90, 0)]
    if z_edge + Rz < zc:
        e.append(Edge.make_line(P(-hw, z_edge + Rz), P(-hw, zc)))
    e.append(Edge.make_circle(hw, ep(0, zc), 0, 180))
    if z_edge + Rz < zc:
        e.append(Edge.make_line(P(hw, zc), P(hw, z_edge + Rz)))
    e += [Edge.make_ellipse(R, Rz, ep(hw + R, z_edge + Rz), 180, 270),
          Edge.make_line(P(hw + R, z_edge), P(hw + R + 2.0, z_edge - 1.0)),
          Edge.make_line(P(hw + R + 2.0, z_edge - 1.0), P(-(hw + R) - 2.0, z_edge - 1.0))]
    face = Face(Wire(e))
    return Solid.extrude(face, Vector(0, y1 - y0, 0))


def b_base_flange(x, y, z):
    add = b_ybox(x, y - J["ft"], y, z, J["fw"], J["fd"])
    sub = [b_ycyl(J["bore_r"] + 0.05, y - J["ft"] - 1, y + 1, x, z)]
    g0, g1, gd = J["groove"]
    sub.append(b_ycyl(g1, y - gd, y + 1, x, z) - b_ycyl(g0, y - gd - 1, y + 2, x, z))
    for bx, bz in bolt_xy(x, z):
        sub.append(b_ycyl(J["hole_base"] / 2, y - J["ft"] - 1, y + 1, bx, bz))
    return add, sub


def b_adapter(x, y, z, od, idd, bead_od, flare):
    ft, nl, sl = J["adp_ft"], J["neck_l"], J["spigot_l"]
    neck_r = max(23.0, od / 2 + 1.5)
    a = b_ybox(x, y, y + ft, z, J["fw"], J["fd"])
    sh_r = min(neck_r + 4.0, J["fd"] / 2 - 0.5)
    if sh_r > neck_r + 0.05:
        a += b_ycyl(sh_r, y + ft - 0.1, y + ft + (sh_r - neck_r), x, z, neck_r)
    a += b_ycyl(neck_r, y + ft - 0.1, y + ft + nl, x, z)
    step = neck_r - od / 2
    a += b_ycyl(neck_r, y + ft + nl - 0.1, y + ft + nl + step, x, z, od / 2)
    a += b_ycyl(od / 2, y + ft + nl + step - 0.1, y + ft + nl + sl, x, z)
    tip = y + ft + nl + sl
    b0 = tip - 4.0 - 1.5
    a += b_ycyl(od / 2, b0 - 1.5, b0, x, z, bead_od / 2) + b_ycyl(bead_od / 2, b0, b0 + 3.0, x, z) + b_ycyl(bead_od / 2, b0 + 3.0, b0 + 4.5, x, z, od / 2)
    v = b_ycyl(J["bore_r"] + 0.05, y - 1, y + ft, x, z)
    v += b_ycyl(J["bore_r"], y + ft - 0.01, y + ft + nl, x, z, idd / 2)
    v += b_ycyl(idd / 2, y + ft + nl - 0.01, tip + 1, x, z)
    if flare:
        v += b_ycyl(idd / 2, tip - 1.5 * flare, tip + 0.01, x, z, idd / 2 + flare)
    for bx, bz in bolt_xy(x, z):
        v += b_ycyl(J["hole_adp"] / 2, y - 1, y + ft + 1, bx, bz)
    return a - v


def b_export(shape, path):
    n = len(shape.solids())
    if n != 1 or not shape.is_valid:
        raise RuntimeError(f"STEP check failed for {path}: solids={n}, valid={shape.is_valid}")
    export_step(shape, path)
    b = shape.bounding_box()
    return {"vol_cm3": round(shape.volume / 1000, 1), "size": [round(b.size.X, 1), round(b.size.Y, 1), round(b.size.Z, 1)]}


def build_step():
    keep_up = Pos(0, 0, 22.5 + 500) * Box(2000, 2000, 1000)
    adds = [flange_b, nipple_b]
    subs = []
    for outer, inner in runners_b:
        adds.append(outer & keep_up)
        subs.append(inner)
    for (ex, ey, ez) in ENDS:
        fa, fs = b_base_flange(ex, ey, ez)
        adds.append(fa)
        subs += fs
    for grp in (ENDS[:3], ENDS[3:]):
        xs = [e[0] for e in grp]
        x0, x1 = min(xs) - J["fw"] / 2, max(xs) + J["fw"] / 2
        adds.append(b_ybox((x0 + x1) / 2, grp[0][1] - J["ft"], grp[0][1], grp[0][2], x1 - x0, J["fd"]))
    ey, ez = ENDS[0][1], ENDS[0][2]
    for tx in TAB_X:
        adds.append(b_ytab(tx, ey - J["ft"], ey, ez + J["fd"] / 2))
        subs.append(b_ycyl(TAB["hole"] / 2, ey - J["ft"] - 1, ey + 1, tx, TAB["zc"]))
    for bx, by in G.BOLT_BORES:
        subs.append(Solid.make_cylinder(8.0, 300, Plane(origin=(bx, by, 23.0))))
    subs.append(Solid.make_cylinder(3.0, _t1 - _t0, Plane(origin=(0, 0, _t0))).rotate(Axis.X, 55.5).translate((-130.4, 48.3, 31.3)))
    part = adds[0]
    for k, a in enumerate(adds[1:]):
        part = part + a
        print(f"STEP: union {k + 2}/{len(adds)}", flush=True)
    for k, v in enumerate(subs):
        part = part - v
        print(f"STEP: cut {k + 1}/{len(subs)}", flush=True)   # progress line per cut, read by run_watchdog.sh
    if len(part.solids()) > 1:
        part = max(part.solids(), key=lambda x: x.volume)
    res = {"komplett": b_export(part, f"{OUT}/Ansaugung_v2_{CFG['tag']}_komplett.step")}
    for name, keep in (("Haelfte_A_x-plus", Keep.TOP), ("Haelfte_B_x-minus", Keep.BOTTOM)):
        res[name] = b_export(part.split(Plane(origin=(G.SPLIT_X, 0, 0), z_dir=(1, 0, 0)), keep=keep),
                             f"{OUT}/Ansaugung_v2_{CFG['tag']}_{name}.step")
    for name, od, idd, bead, flare in (("K100_Tuelle41", 41.0, 35.0, 43.0, 1.0), ("48er_Tuelle48", 48.3, 40.0, 50.3, 1.5)):
        one = b_adapter(0.0, 0.0, 0.0, od, idd, bead, flare).rotate(Axis.X, 90)
        res["adapter_" + name] = b_export(one, f"{OUT}/Ansaugung_v2_Adapter_{name}_1Stueck_Druck.step")
    return res


if STEP:
    if MODE != "fdm_modular":
        raise SystemExit("--step is only implemented for fdm_modular")
    info["step"] = build_step()

print(json.dumps(info, indent=1))
