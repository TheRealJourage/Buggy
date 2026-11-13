"""Intake adapter v2 variants with mesh booleans (manifold3d).
sls : wall 4 mm, two halves (split between cyl 3/4)
cast: wall 6 mm, six single runners each with its own flange segment, shrink + machining allowance
usage: intake_v2_variants.py <fdm|sls|cast> <outdir>
"""
import sys, os, json, tempfile
import numpy as np
import trimesh
import manifold3d as mf
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import intake_v2_geom as G
from build123d import import_step, export_stl, Plane, Keep, Pos, Box, extrude, RectangleRounded

MODE, OUT = sys.argv[1], sys.argv[2]
os.makedirs(OUT, exist_ok=True)
CFG = {
    "sls": dict(wall=4.0, spigot_od=41.0, cone=4.0, tag="SLS", tie_plate=True),
    "fdm": dict(wall=6.8, spigot_od=41.0, cone=9.0, tag="FDM"),
    "cast": dict(wall=6.0, spigot_od=41.6, shrink=1.013, face_allow=1.0, hole_cast_d=11.2),
}[MODE]
G.SRC = "/srv/data/obsidian/vault/Buggy/Ansaugung/Ansaugung.step"
G.WALL = CFG["wall"]
G.R_OUT = G.R_IN + G.WALL
G.SPIGOT_OD = CFG["spigot_od"]
G.CONE = CFG.get("cone", G.CONE)
G.N_PTS = 32
TMP = tempfile.mkdtemp()


def to_mf(shape):
    p = os.path.join(TMP, "t.stl")
    export_stl(shape, p, tolerance=0.05, angular_tolerance=0.15)
    m = trimesh.load(p)
    m.merge_vertices()
    if not m.is_watertight:
        trimesh.repair.fill_holes(m)
    return mf.Manifold(mf.Mesh(vert_properties=np.asarray(m.vertices, np.float32), tri_verts=np.asarray(m.faces, np.uint32)))


def save(man, path):
    parts = man.decompose()
    man = max(parts, key=lambda x: x.volume())
    me = man.to_mesh()
    tm = trimesh.Trimesh(vertices=np.asarray(me.vert_properties)[:, :3], faces=np.asarray(me.tri_verts), process=False)
    tm.export(path)
    b = tm.bounds
    return {"vol_cm3": round(tm.volume / 1000, 1), "watertight": bool(tm.is_watertight),
            "bodies": len(tm.split(only_watertight=False)), "size": [round(float(v), 1) for v in b[1] - b[0]]}


src = import_step(G.SRC)
flange = to_mf(src.split(Plane.XY.offset(G.FLANGE_TOP), keep=Keep.BOTTOM))
(bx0, by0, bz0), (bx1, by1, bz1) = G.NIPPLE_BOX
nipple = to_mf(src & Pos((bx0 + bx1) / 2, (by0 + by1) / 2, (bz0 + bz1) / 2) * Box(bx1 - bx0, by1 - by0, bz1 - bz0))

runners = []
for port, xe in zip(G.PORTS, G.THROTTLE_X):
    outer, end = G.runner(port, xe, G.WALL)
    inner, _ = G.runner(port, xe, 0.0, extra_below=3.0)
    w, h, r = port[2] + 2 * G.WALL, port[3] + 2 * G.WALL, port[4] + G.WALL
    foot = to_mf(Pos(port[0], port[1], G.FLANGE_TOP - 1.5) * extrude(RectangleRounded(w, h, r), amount=1.5))
    add = to_mf(outer) + foot + to_mf(G.spigot(end, True))
    sub = to_mf(inner[0]) + to_mf(inner[1]) + to_mf(G.spigot(end, False))
    runners.append((add, sub))
    print("runner", port[0], "ok", flush=True)

bores = mf.Manifold()
for bx, by in G.BOLT_BORES:
    bores += mf.Manifold.cylinder(300, 8.0, 8.0, 64).translate((bx, by, G.FLANGE_TOP))

# vacuum nipple on runner 4: re-drill through the new (thinner) wall along the original nipple axis
_t0, _t1 = 40.0, 74.0
nipple_bore = (mf.Manifold.cylinder(_t1 - _t0, 3.0, 3.0, 32).translate((0, 0, _t0))
               .rotate((55.5, 0, 0)).translate((-130.4, 48.3, 31.3)))
info = {"mode": MODE, "cfg": CFG}
# tie plate across all runners just below the spigots: support strut mount + runner bracing
TIE = dict(y0=118.0, t=10.0, w=44.0, hole_d=6.4, x0=-330.0, x1=134.0)
tie_holes_x = [G.THROTTLE_X[0] + 21.8 + 9.5] + [(a + b) / 2 for a, b in zip(G.THROTTLE_X, G.THROTTLE_X[1:])] + [G.THROTTLE_X[-1] - 21.8 - 9.5]
if MODE in ("sls", "fdm"):
    part = flange + nipple
    if CFG.get("tie_plate"):
        part += mf.Manifold.cube((TIE["x1"] - TIE["x0"], TIE["t"], TIE["w"])).translate((TIE["x0"], TIE["y0"], 112.52 - TIE["w"] / 2))
    for a, _ in runners:
        part += a
    for _, s in runners:
        part -= s
    part -= bores
    if CFG.get("tie_plate"):
        for hx in tie_holes_x:
            part -= mf.Manifold.cylinder(40, TIE["hole_d"] / 2, TIE["hole_d"] / 2, 32).rotate((-90, 0, 0)).translate((hx, TIE["y0"] - 15, 112.52))
        info["tie_holes_x"] = [round(x, 1) for x in tie_holes_x]
    part -= nipple_bore
    info["komplett"] = save(part, f"{OUT}/Ansaugung_v2_{CFG['tag']}_komplett.stl")
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
                seg += (mf.Manifold.cylinder(18, 6.6, 6.6, 48).translate((hx, hy, 10.5)) ^ slab)
                seg -= mf.Manifold.cylinder(30, CFG["hole_cast_d"] / 2, CFG["hole_cast_d"] / 2, 48).translate((hx, hy, 0))
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
print(json.dumps(info, indent=1))
