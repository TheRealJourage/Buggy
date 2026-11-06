// Three exhaust layouts behind the real VR6 of the CAD model (Baugr2TUV.STEP, mm): X lateral (+X = left, Hypothese),
// Y up, Z forward (+Z = front). Frame and engine numbers come from the coarse 2022 model and are hypotheses.
import { bellLength, bellRpm } from './exhaust.js';

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const len = (a) => Math.hypot(a[0], a[1], a[2]);
const norm = (a) => mul(a, 1 / (len(a) || 1));
const dist = (a, b) => len(sub(a, b));
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

export const TUBE = { prim: 38, sec: 63.5, main: 76.2, down: 57, cast: 44 }; // OD; wall 1.5 → ID 35 / 60.5 / 73.2 / 54
const FR = 24.15; // frame tube radius (48.3 mm)

// frame members near the engine bay, centre lines from the STEP bounding boxes
export function frame(opt = {}) {
  const m = [];
  const seg = (name, a, b, r = FR, extra = {}) => m.push({ name, a, b, r, ...extra });
  for (const x of [630, 1410]) {
    for (const y of [965, 1365]) seg(`Längsrohr innen ${x < 1000 ? 'rechts' : 'links'} ${y < 1000 ? 'unten' : 'oben'}`, [x, y, -715], [x, y, -1415]);
    seg(`Pfosten hinter Motor ${x < 1000 ? 'rechts' : 'links'}`, [x, 965, -715], [x, 1365, -715]);
    seg(`Pfosten Heckkasten ${x < 1000 ? 'rechts' : 'links'}`, [x, 965, -1415], [x, 1365, -1415]);
    const xe = x < 1000 ? 830 : 1210;
    for (const y of [965, 1365]) seg(`Heck-Eckrohr ${x < 1000 ? 'rechts' : 'links'} ${y < 1000 ? 'unten' : 'oben'}`, [x, y, -1415], [xe, y, -1615]);
  }
  seg('Querrohr unter Motor', [630, 965, -715], [1410, 965, -715]);
  for (const y of [965, 1365]) seg(`Heck-Querrohr ${y < 1000 ? 'unten' : 'oben'}`, [830, y, -1615], [1210, y, -1615]);
  seg('Oberes Querrohr (Käfig)', [252, 1816, -1265], [1787, 1816, -1265]);
  seg('Käfig-Strebe rechts vorn', [630, 1370, -720], [305, 1815, -1265]);
  seg('Käfig-Strebe links vorn', [1410, 1370, -720], [1735, 1815, -1265]);
  seg('Käfig-Strebe rechts hinten', [630, 1370, -1415], [310, 1800, -1270]);
  seg('Käfig-Strebe links hinten', [1410, 1370, -1415], [1730, 1800, -1270]);
  // real rear beams (Halle 2026-11-05): 35 × 35 square, 50 mm clear gap in the middle; height and depth from the model
  for (const x of [1020 - 42.5, 1020 + 42.5]) seg('Heckbalken AHK (35 × 35 gemessen, Lage aus dem Modell)', [x, 965, -1615], [x, 1365, -1615], 25, { square: 35, hyp: true });
  // round uprights of the model at 830/1210 do not exist like this → shown as ghost only
  for (const x of [830, 1210]) seg('Heckpfosten im Modell (real nicht so)', [x, 965, -1615], [x, 1365, -1615], FR, { ghost: true });
  if (opt.upperBeam !== false) seg('Oberer Balken (fehlt im Modell, Lage geraten)', [1020, 1365, -1615], [1020, 1816, -1265], FR, { hyp: true, guess: true });
  return m;
}
// engine + gearbox envelope from the STEP surface bodies: one slab per 25 mm height band (x0, y0, zRear, x1, y1, zFront)
const SLABS = [[720, 925, -1213, 891, 950, -830], [702, 975, -1228, 720, 1000, -830], [648, 1000, -1230, 1113, 1025, -830], [647, 1025, -1161, 675, 1050, -830], [616, 1050, -1310, 1585, 1075, -830], [608, 1075, -1325, 1647, 1100, -830], [604, 1100, -1332, 1650, 1125, -830], [646, 1125, -1335, 1633, 1150, -830], [646, 1150, -1336, 1602, 1175, -830], [649, 1175, -1334, 1167, 1200, -830], [653, 1200, -1337, 1601, 1225, -830], [648, 1225, -1337, 1602, 1250, -830], [648, 1250, -1316, 1601, 1275, -830], [652, 1275, -1323, 1444, 1300, -830], [652, 1300, -1284, 1443, 1325, -830], [652, 1325, -1261, 1326, 1350, -830], [652, 1350, -1223, 1326, 1375, -830], [718, 1375, -1195, 1324, 1400, -830], [670, 1400, -1196, 1206, 1425, -830], [626, 1425, -1197, 1187, 1450, -830], [664, 1450, -1185, 1186, 1475, -830], [664, 1475, -1184, 1170, 1500, -830], [663, 1500, -1184, 1186, 1525, -830], [663, 1525, -1184, 1186, 1550, -830], [624, 1550, -1190, 1185, 1575, -830], [1168, 1575, -1190, 1185, 1600, -830]];
export const engine = (o = {}) => SLABS.map(([x0, y0, z0, x1, y1, z1]) => {
  const d = [o.dx ?? 0, o.dy ?? 0, o.dz ?? 0];
  return { name: 'Motor/Getriebe (STEP)', min: add([x0, y0, z0], d), max: add([x1, y1, z1], d) };
});
export const ENGINE = engine();
export const FLANGE = { z: -1185, x: 912, y: 1500, top: 1561 }; // exhaust flange face of the head in the model (plates 1443–1561 high, X 661–1164)
export const LIMITS = { floor: 989, xMin: 654, xMax: 1386, sideBelowY: 1400, rear: -1639, top: 1990 };
const CAGE = { y: 1816, r: FR, z: -1265 }, REAR_INNER = -1591; // cage cross tube; front face of the rear cross tubes

// Halle 2026-11-05 (User, confirmed): the frame is as in the model, the engine sits differently (lower, further in).
// Gaps: manifold top → cage cross tube (vertical), outermost runner → tube beside it (left/right in driving direction),
// engine → inner side of the frame end at manifold height.
export const HALLE = { top: 400, left: 220, right: 120, rear: 400 };
const CAST_HALF = TUBE.cast / 2;
const rearAt = (y, boxes) => Math.min(...boxes.filter((e) => e.min[1] <= y && e.max[1] >= y).map((e) => e.min[2]));
// engine shift that reproduces the Halle gaps (Hypothese: manifold top = top of the head flange plates, runner edge = port ± 22)
export function offsetFromHalle(pitch = 65) {
  const dy = Math.round(CAGE.y - CAGE.r - HALLE.top - FLANGE.top);
  const xRight = LIMITS.xMin + HALLE.right, xLeft = LIMITS.xMax - HALLE.left;
  const dx = Math.round((xRight + xLeft) / 2 - FLANGE.x);
  const dz = Math.round(REAR_INNER + HALLE.rear - rearAt(FLANGE.y, ENGINE));
  return { dx, dy, dz, span: xLeft - xRight, modelSpan: 5 * pitch + 2 * CAST_HALF };
}
export const OFFSET = (({ dx, dy, dz }) => ({ dx, dy, dz }))(offsetFromHalle());
// the same gaps measured back in the shifted model, plus engine boxes that cut a frame tube (model inconsistency)
export function halleCheck(o = {}) {
  const p = { yp: FLANGE.y, pitch: 65, cyl1: 'r', ...OFFSET, ...o };
  const boxes = engine(p), xc = FLANGE.x + p.dx, half = 2.5 * p.pitch + CAST_HALF;
  const rows = {
    top: CAGE.y - CAGE.r - (FLANGE.top + p.dy),
    right: xc - half - LIMITS.xMin,
    left: LIMITS.xMax - (xc + half),
    rear: rearAt(p.yp + p.dy, boxes) - REAR_INNER,
  };
  const cuts = new Set();
  for (const m of frame({ upperBeam: false })) {
    if (m.ghost) continue;
    for (let k = 0; k <= 40; k++) {
      const q = add(m.a, mul(sub(m.b, m.a), k / 40));
      if (boxes.some((e) => [0, 1, 2].every((i) => q[i] > e.min[i] - m.r && q[i] < e.max[i] + m.r))) { cuts.add(m.name.split(' (')[0]); break; }
    }
  }
  return { ...rows, cuts: [...cuts] };
}

// polyline with corner fillets (quadratic Bézier, tangent distance R), sampled every ~step mm
export function fillet(pts, R, step = 6) {
  const out = [pts[0]];
  const line = (a, b) => { const n = Math.max(1, Math.ceil(dist(a, b) / step)); for (let k = 1; k <= n; k++) out.push(add(a, mul(sub(b, a), k / n))); };
  const quad = (a, c, b) => { const n = Math.max(4, Math.ceil((dist(a, c) + dist(c, b)) / step)); for (let k = 1; k <= n; k++) { const t = k / n, s = 1 - t; out.push(add(add(mul(a, s * s), mul(c, 2 * s * t)), mul(b, t * t))); } };
  let prev = pts[0];
  for (let i = 1; i < pts.length - 1; i++) {
    const A = pts[i - 1], B = pts[i], C = pts[i + 1], la = dist(A, B), lc = dist(C, B);
    if (la < 1e-6 || lc < 1e-6) continue;
    const u1 = norm(sub(A, B)), u2 = norm(sub(C, B));
    if (dot(u1, u2) < -0.999) { line(prev, B); prev = B; continue; }
    const r = Math.min(R, la * (i === 1 ? 1 : 0.5), lc * (i === pts.length - 2 ? 1 : 0.5));
    const p1 = add(B, mul(u1, r)), p2 = add(B, mul(u2, r));
    line(prev, p1); quad(p1, B, p2); prev = p2;
  }
  line(prev, pts[pts.length - 1]);
  return out;
}
export const pathLen = (s) => s.reduce((acc, p, i) => (i ? acc + dist(p, s[i - 1]) : 0), 0);
function solve(fn, L, lo, hi) {
  if (pathLen(fn(hi)) < L) return { pts: fn(hi), short: true };
  for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (pathLen(fn(m)) < L) lo = m; else hi = m; }
  return { pts: fn((lo + hi) / 2), short: false };
}

// slots on a collector ring; pipes are matched by the angle of their approach point
function basis(axis) { const a = norm(axis), t = Math.abs(a[1]) < 0.9 ? [0, 1, 0] : [1, 0, 0]; const u = norm(cross(a, t)); return [u, cross(a, u)]; }
function ringSlots(c, axis, r, approach) {
  const [u, v] = basis(axis), n = approach.length;
  const phi = approach.map((p) => { const d = sub(p, c); return Math.atan2(dot(d, v), dot(d, u)); });
  const order = phi.map((_, i) => i).sort((a, b) => phi[a] - phi[b]);
  let sx = 0, sy = 0; order.forEach((pi, k) => { const a = phi[pi] - (2 * Math.PI * k) / n; sx += Math.cos(a); sy += Math.sin(a); });
  const th0 = Math.atan2(sy, sx), slots = [];
  order.forEach((pi, k) => { const th = th0 + (2 * Math.PI * k) / n; slots[pi] = add(c, add(mul(u, r * Math.cos(th)), mul(v, r * Math.sin(th)))); });
  return slots;
}

function xSlots(c, r, th0, xs) {
  const n = xs.length, sl = Array.from({ length: n }, (_, k) => { const t = ((th0 + (360 * k) / n) * Math.PI) / 180; return add(c, [r * Math.cos(t), 0, r * Math.sin(t)]); });
  sl.sort((a, b) => a[0] - b[0]);
  const order = xs.map((_, i) => i).sort((a, b) => xs[a] - xs[b]), res = [];
  order.forEach((pi, k) => { res[pi] = sl[k]; });
  return res;
}

// ---------------------------------------------------------------- layouts
// collector positions (frame coordinates), how far behind the flange the primaries climb and how far each group moves
// sideways off the middle beam. Found by a sweep (2026-11-06): no contact incl. the guessed middle beam at 4500/min,
// and still none when any value moves by ±20 mm.
export const LAYOUT = {
  A: { rise: 140, split: 95, aY: 1200, aZ: -1430, aDx: 50 },
  B: { rise: 110, split: 100, bY: 1230, bZ: -1440, bDx: 40 },
  C: {},
  N1: { yRun: 1050, yX: 0, straight: 150, katX: 120 },
  N2: { yRun: 1030, lift: 70, xY: 175, zY: -1500, straight: 150, yDmp: 1640 },
  N3: { yRun: 1030, lift: 64, xY: 230, zY: -1500, straight: 150, yDmp: 1640 },
};
export const VARIANTS = {
  A: { title: 'A · Fächer 6-2-1, Stack links' },
  B: { title: 'B · Fächer 6-1, Querdämpfer, 2 Rohre hinten' },
  C: { title: 'C · Serien-Guss + Y, Endrohr hinten-links' },
  N1: { title: 'N1 · Serien-Guss, Y mittig, Kat quer am Rahmenende' },
  N2: { title: 'N2 · Serien-Guss, Y stehend, Kat hoch, Querdämpfer oben' },
  N3: { title: 'N3 · Serien-Guss, gleich lange Hosenrohre, Y weiter links, Querdämpfer oben' },
};

export function build(v, o = {}) {
  const p = { yp: FLANGE.y, pitch: 65, cyl1: 'r', rpm: 4500, eo: 56, oDown: 170, oBack: 200, lkat: 300, dkat: 130, ldmp: 320, ddmp: 110, upperBeam: true, ...OFFSET, ...LAYOUT[v], ...o };
  const ED = 180 + p.eo, Lp = bellLength(p.rpm, ED);
  const s1 = p.cyl1 === 'r' ? 1 : -1; // cyl 1 at low X (= right, pulley side) when s1 = 1
  const Z0 = FLANGE.z + p.dz, XC = FLANGE.x + p.dx, YP = p.yp + p.dy; // yp = channel height in the model, dx/dy/dz = engine shift
  const ports = [0, 1, 2, 3, 4, 5].map((i) => [XC + s1 * (i - 2.5) * p.pitch, YP, Z0]);
  const parts = [], notes = [], marks = [];
  const part = (name, pts, od, kind, extra = {}) => { const q = { name, pts, od, kind, len: pathLen(pts), ...extra }; parts.push(q); return q; };
  const cyl = (name, a, b, d, kind, extra = {}) => part(name, [a, b], d, kind, { solid: true, ...extra });
  const cone = (name, a, b, r0, r1) => part(name, [a, b], 2 * Math.max(r0, r1), 'cone', { solid: true, r0, r1 });
  const grpOf = (ci) => (ci < 3 ? 0 : 1);
  const R = 1.5 * TUBE.prim, Rs = 1.5 * TUBE.sec;
  const out = { v, p, parts, notes, marks, ports, Lp, ED, engine: engine(p) };
  const YL = 1058;                               // low run in the well behind the block (floor 989 + r)
  const ZM = -1480, ZR = -1520, XR = FLANGE.x + 228; // Y slots, main run along X, riser (left of centre, clear of the corner tube)
  const gx = [0, 1].map((gi) => [[0, 1, 2], [3, 4, 5]][gi].reduce((a, ci) => a + ports[ci][0], 0) / 3);
  const sonde = (at, txt) => marks.push({ kind: 'sonde', at, name: `LSU 4.9 in M18×1,5-Muffe – ${txt}` });
  const vband = (at) => marks.push({ kind: 'vband', at, r: 45 });

  function primaries(slotsFor, viaFor, L, tMax) {
    return ports.map((P, ci) => {
      const slot = slotsFor(ci), pre = add(slot, [0, 150, 0]), pre2 = add(slot, [0, 60, 0]);
      const fn = (t) => fillet([P, add(P, [0, 0, -45]), ...viaFor(ci, t, P, slot), pre, pre2, slot], R);
      const s = solve(fn, L, 0, tMax);
      if (s.short) notes.push(`Primärrohr Zyl. ${ci + 1} erreicht nur ${pathLen(s.pts).toFixed(0)} statt ${L.toFixed(0)} mm.`);
      return part(`Primärrohr Zyl. ${ci + 1} (38 × 1,5)`, s.pts, TUBE.prim, 'p', { group: grpOf(ci), skipEnd: 70 });
    });
  }
  // over the head: up behind the flange, back over the block, down into the well
  // up behind the flange (each group shifted sideways by `split`, away from the middle beam), back over the head, down into the well
  const lane = (ci, P) => P[0] + s1 * (grpOf(ci) ? 1 : -1) * p.split;
  const overTop = (ci, t, P, slot) => [[lane(ci, P), YP + t, Z0 - p.rise], [lane(ci, P), YP + t, slot[2] + 70]];
  const tMax = CAGE.y - YP + 15;
  function castFront() {
    // OEM manifolds: outlet flange behind/below the head, clear of the block (shape and position: Hypothese)
    const O = gx.map((x) => [x, YP - p.oDown, Z0 - p.oBack]);
    const runners = ports.map((P, ci) => {
      const o = O[grpOf(ci)];
      return part(`Serien-Gusskrümmer ${ci < 3 ? '1–3 (022 253 033)' : '4–6 (022 253 034)'}, Kanal Zyl. ${ci + 1} (Form schematisch)`,
        fillet([P, add(P, [0, 0, -45]), [(P[0] + o[0]) / 2, YP - 20, (Z0 + o[2]) / 2 - 15], add(o, [0, 50, 0]), o], 35), TUBE.cast, 'p', { group: grpOf(ci), chain: `cast${grpOf(ci)}`, skipEnd: 140 });
    });
    O.forEach((o) => marks.push({ kind: 'flange', at: o, name: 'Krümmer-Ausgang 3-Loch (Dichtung 021 253 115), nach unten (Hypothese) – Ø, Lage, Richtung messen' }));
    return { O, runners };
  }
  // two pipes (secondaries / downpipes) down into the Y at the bottom of the well; Y points rearward
  function intoY(outl, od, name) {
    const ySl = [0, 1].map((gi) => [XC + s1 * (gi ? 33 : -33), YL, ZM]);
    const sec = [0, 1].map((gi) => { const O = outl[gi]; return part(name(gi), fillet([O, [O[0], YL, O[2]], add(ySl[gi], [0, 0, 50]), ySl[gi]], 85), od, 's', { group: gi, skipEnd: 50, skipStart: 40 }); });
    const yOut = [XC, YL, ZM - 40];
    cone('Y-Sammler 2-in-1 (Merge, kein T)', [XC, YL, ZM], yOut, 33 + od / 2, TUBE.main / 2);
    return { sec, yOut };
  }
  // main: along the well bottom to the left, sonde on top, then up as riser with the cat
  function mainToRiser(from, txt) {
    const kb = [XR, 1200, ZR];
    part('Sammelrohr 76,2 (alle 6 Zylinder) → Steigrohr', fillet([from, [from[0], YL, ZR], [XR, YL, ZR], kb], 100), TUBE.main, 'm');
    sonde([XC + 60, YL + 38, ZR], txt);
    const kt = add(kb, [0, p.lkat, 0]);
    cyl(`Kat (Platzhalter Ø ${p.dkat} × ${p.lkat}, Mustang-Kat – messen)`, kb, kt, p.dkat, 'kat');
    vband(kb); vband(kt);
    return kt;
  }
  const vertDamper = (kt) => { const d0 = add(kt, [0, 30, 0]), d1 = add(d0, [0, p.ldmp, 0]); part('Rohr 76,2', [kt, d0], TUBE.main, 'm'); cyl(`Dämpfer (Absorption, gerader Durchgang, Ø ${p.ddmp} × ${p.ldmp}, mit Marke + Typ kaufen)`, d0, d1, p.ddmp, 'dmp'); return d1; };

  if (v === 'A') {
    const C = gx.map((x, gi) => [x + s1 * (gi ? 1 : -1) * p.aDx, p.aY, p.aZ]);
    const slots = [];
    [[0, 1, 2], [3, 4, 5]].forEach((grp, gi) => { const sl = xSlots(C[gi], 27, 90, grp.map((ci) => ports[ci][0])); grp.forEach((ci, k) => { slots[ci] = sl[k]; }); });
    const prim = primaries((ci) => slots[ci], overTop, Lp, tMax);
    const outl = C.map((c, gi) => { const e = add(c, [0, -125, 0]); cone(`3-in-1-Sammler ${gi ? '{4,5,6}' : '{1,2,3}'}`, c, e, 27 + TUBE.prim / 2 + 2, TUBE.sec / 2); return e; });
    const { sec, yOut } = intoY(outl, TUBE.sec, (gi) => `Sekundärrohr ${gi ? '{4,5,6}' : '{1,2,3}'} (63,5 × 1,5)`);
    const d1 = vertDamper(mainToRiser(yOut, 'auf dem Sammelrohr unten, ≈ 100 mm hinter dem Y'));
    part('Endrohr (76,2, Mündung oben, leicht nach hinten)', fillet([d1, add(d1, [0, 60, 0]), add(d1, [0, 120, -35])], 90), TUBE.main, 't');
    out.primLen = prim.map((q) => q.len); out.secLen = sec.map((q) => q.len);
    out.sondePath = prim.map((q, ci) => q.len + 125 + sec[grpOf(ci)].len + 40 + 100);
  }

  if (v === 'B') {
    const C = [XC + p.bDx, p.bY, p.bZ];
    const sl = xSlots(C, 47, 15, ports.map((P) => P[0]));
    const prim = primaries((ci) => sl[ci], overTop, Lp, tMax);
    const o1 = add(C, [0, -150, 0]);
    cone('6-in-1-Sammler', C, o1, 47 + TUBE.prim / 2 + 2, TUBE.main / 2);
    const kt = mainToRiser(o1, 'auf dem Sammelrohr unten, ≈ 250 mm hinter dem 6-in-1');
    // pipe up to a cross muffler high behind the head, two tails straight back
    const dy = Math.max(kt[1] + 120, 1790), half = p.ldmp / 2, dx0 = XR - 90 - p.ldmp, dx1 = XR - 90;
    part('Rohr 76,2 → Querdämpfer', fillet([kt, [XR, dy, ZR], [dx1, dy, ZR]], 100), TUBE.main, 'm');
    cyl(`Querdämpfer (Einlass seitlich, 2 Auslässe hinten, Ø ${p.ddmp} × ${p.ldmp}, mit Marke + Typ)`, [dx0, dy, ZR], [dx1, dy, ZR], p.ddmp, 'dmp');
    for (const f of [0.25, 0.75]) {
      const x = dx0 + f * p.ldmp, e = [x, dy, ZR - p.ddmp / 2 + 10];
      part(`Endrohr ${f < 0.5 ? 'rechts' : 'links'} (57 × 1,5, waagrecht nach hinten)`, [e, [x, dy, -1630]], 57, 't');
    }
    out.primLen = prim.map((q) => q.len);
    out.sondePath = prim.map((q) => q.len + 150 + 250);
    if (dy + p.ddmp / 2 > 1990) notes.push('Querdämpfer höher als der Käfig (Y 1990).');
  }

  if (v === 'C') {
    const c = castFront();
    const { sec, yOut } = intoY(c.O, TUBE.down, (gi) => `Hosenrohr ${gi ? '4–6' : '1–3'} (57 × 1,5, Ø am Krümmer messen)`);
    const d1 = vertDamper(mainToRiser(yOut, 'auf dem Sammelrohr unten, ≈ 100 mm hinter dem Y'));
    const dirTip = norm([0.5, 0, -0.866]); // 30° to the left of straight back
    part('Endrohr (76,2, Mündung hinten-links 30°)', fillet([d1, add(d1, [0, 70, 0]), add(add(d1, [0, 70, 0]), mul(dirTip, 110))], 100), TUBE.main, 't');
    out.primLen = c.runners.map((q) => q.len); out.secLen = sec.map((q) => q.len);
    out.sondePath = c.runners.map((q, ci) => q.len + sec[grpOf(ci)].len + 40 + 100);
    out.cast = true;
  }
  // ---- N1–N3 (2026-11-06): stock cast manifolds, routing after .research-exhaust-flow.md / -practice.md:
  // change direction in the branches before the Y, straight run behind the Y (sonde there), only then wide bends (R/D ≥ 1.5)
  const RB = 1.5 * TUBE.down, RM = 1.5 * TUBE.main, YCONE = 110, LEFT = [1, 0, 0];
  const yMerge = (inl, dir, label) => {
    const a = add(inl, mul(dir, YCONE));
    cone(`Y-Sammler 2-in-1, Merge ≈ 15° (${label})`, inl, a, 33 + TUBE.down / 2, TUBE.main / 2);
    return a;
  };
  const straightSonde = (from, dir, txt) => {
    const e = add(from, mul(dir, p.straight));
    part(`Gerades Stück hinter dem Y (${p.straight} mm, Sonde)`, [from, e], TUBE.main, 'm');
    const up = dir[1] > 0.5 ? [0, 0, 1] : [0, 1, 0];
    sonde(add(add(from, mul(dir, p.straight / 2)), mul(up, 38)), txt);
    return e;
  };
  const katAt = (a, dir) => { const b = add(a, mul(dir, p.lkat)); cyl(`Kat (Platzhalter Ø ${p.dkat} × ${p.lkat}, Mustang-Kat – messen)`, a, b, p.dkat, 'kat'); vband(a); vband(b); return b; };
  const dmpAt = (a, dir, txt) => { const b = add(a, mul(dir, p.ldmp)); cyl(`${txt} (Platzhalter Ø ${p.ddmp} × ${p.ldmp}, mit Marke + Typ)`, a, b, p.ddmp, 'dmp'); return b; };
  const branchFin = (sec, c) => {
    out.primLen = c.runners.map((q) => q.len); out.secLen = sec.map((q) => q.len);
    out.cast = true;
  };

  if (v === 'N1') {
    // both outlets down → one bend each to the rear, converge, Y in the middle pointing rearward, straight, one bend left,
    // cat across the frame end, then up into a vertical straight-through damper in the left corner
    const c = castFront(), O = c.O, YR = p.yRun, Zr = O[0][2] - RB - 10;
    const yIn = [XC + p.yX, YR, Zr - 60];
    const sec = O.map((o, gi) => part(`Hosenrohr ${gi ? '4–6' : '1–3'} (57 × 1,5, 1 Bogen R 85 nach hinten)`,
      fillet([o, [o[0], YR, o[2]], [o[0], YR, Zr], add(yIn, [s1 * (gi ? 33 : -33), 0, 0])], RB), TUBE.down, 's', { group: gi, skipStart: 40, skipEnd: 40 }));
    const s0 = straightSonde(yMerge(yIn, [0, 0, -1], 'zeigt nach hinten'), [0, 0, -1], 'im geraden Stück hinter dem Y, oben');
    const k0 = [s0[0] + RM + p.katX, YR, s0[2] - RM];
    part('Bogen 90° nach links (R 114)', fillet([s0, [s0[0], YR, k0[2]], k0], RM), TUBE.main, 'm');
    const k1 = katAt(k0, LEFT);
    const d0 = [k1[0] + RM, YR + RM + 30, k1[2]];
    part('Bogen 90° hoch (R 114)', fillet([k1, [d0[0], YR, k1[2]], d0], RM), TUBE.main, 'm');
    const d1 = dmpAt(d0, [0, 1, 0], 'Dämpfer stehend, gerader Durchgang');
    part('Endrohr (76,2, Mündung hinten-links 30°)', fillet([d1, add(d1, [0, 70, 0]), add(add(d1, [0, 70, 0]), mul(norm([0.5, 0, -0.866]), 110))], 100), TUBE.main, 't');
    branchFin(sec, c);
    out.sondePath = c.runners.map((q, ci) => q.len + sec[grpOf(ci)].len + YCONE + p.straight / 2);
  }

  // standing Y behind the manifolds, fed from below: each branch goes down, runs flat (diagonal) to below its Y inlet, turns up
  const standingY = (O, t = 0) => {
    const YR = p.yRun, yIn = [XC + p.xY, YR + RB + 70, p.zY];
    const far = Math.abs(O[0][0] - yIn[0]) > Math.abs(O[1][0] - yIn[0]) ? 0 : 1;
    const slot = (gi) => add(yIn, [gi === far ? -33 : 33, 0, 0]);
    // the far branch runs `lift` higher so the two flat runs can cross; a higher run is shorter
    const route = (gi, lift = p.lift) => { const o = O[gi], sl = slot(gi), y = gi === far ? YR + lift : YR; return fillet([o, [o[0], y, o[2]], [sl[0], y, sl[2]], sl], RB); };
    return { yIn, far, route };
  };
  const castBranches = (O, sy, equal, label) => {
    const base = [0, 1].map((gi) => sy.route(gi));
    if (equal) {
      // equal length: raise the far branch until it is as long as the near one (bounded by the clearance between the runs)
      const near = 1 - sy.far, L = pathLen(base[near]);
      let lo = p.lift, hi = 160;
      if (pathLen(sy.route(sy.far, lo)) < L - 10) notes.push('Hosenrohre nicht gleich lang zu bekommen.');
      else if (pathLen(sy.route(sy.far, lo)) < L) { /* within 10 mm: equal enough */ }
      else { for (let i = 0; i < 40; i++) { const m = (lo + hi) / 2; if (pathLen(sy.route(sy.far, m)) > L) lo = m; else hi = m; } base[sy.far] = sy.route(sy.far, lo); }
    }
    return [0, 1].map((gi) => part(`Hosenrohr ${gi ? '4–6' : '1–3'} (57 × 1,5, ${label})`, base[gi], TUBE.down, 's', { group: gi, skipStart: 40, skipEnd: 40 }));
  };

  if (v === 'N2' || v === 'N3') {
    // standing Y left of the middle beam, straight up (sonde), cat upright, one bend into a transverse damper
    // behind the cage cross tube, two tailpipes to the rear. N3: Y further left and both branches equally long.
    const c = castFront(), sy = standingY(c.O), sec = castBranches(c.O, sy, v === 'N3', v === 'N3' ? 'gleich lang' : 'kurz, Bogen nach oben');
    const s0 = straightSonde(yMerge(sy.yIn, [0, 1, 0], 'steht senkrecht'), [0, 1, 0], 'im geraden Stück über dem Y, seitlich');
    const k1 = katAt(s0, [0, 1, 0]);
    const dy = Math.max(k1[1] + RM + 30, p.yDmp), dx1 = k1[0] - RM - 30, dx0 = dx1 - p.ldmp;
    part('Bogen 90° in den Querdämpfer (R 114)', fillet([k1, [k1[0], dy, k1[2]], [dx1, dy, k1[2]]], RM), TUBE.main, 'm');
    cyl(`Querdämpfer (Einlass seitlich, 2 Auslässe hinten, Platzhalter Ø ${p.ddmp} × ${p.ldmp}, mit Marke + Typ)`, [dx0, dy, k1[2]], [dx1, dy, k1[2]], p.ddmp, 'dmp');
    for (const f of [0.25, 0.75]) {
      const x = dx0 + f * p.ldmp, e = [x, dy, k1[2] - p.ddmp / 2 + 10];
      part(`Endrohr ${f < 0.5 ? 'rechts' : 'links'} (57 × 1,5, waagrecht nach hinten)`, [e, [x, dy, -1630]], 57, 't');
    }
    branchFin(sec, c);
    out.sondePath = c.runners.map((q, ci) => q.len + sec[grpOf(ci)].len + YCONE + p.straight / 2);
  }

  // total turning of each pipe (sum of direction changes along the polyline) and turning after the Y
  const turn = (q) => {
    const P = q.pts.filter((pt, k) => !k || dist(pt, q.pts[k - 1]) > 1e-6);
    let a = 0;
    for (let k = 2; k < P.length; k++) a += Math.acos(Math.max(-1, Math.min(1, dot(norm(sub(P[k - 1], P[k - 2])), norm(sub(P[k], P[k - 1]))))));
    return (a * 180) / Math.PI;
  };
  out.turnSec = parts.filter((q) => q.kind === 's').map(turn);
  out.turnMain = parts.filter((q) => q.kind === 'm' || q.kind === 't').reduce((a, q) => a + (q.solid ? 0 : turn(q)), 0);
  check(out, frame({ upperBeam: p.upperBeam }));
  if (out.cast) out.tunedRpm = null;
  return out;
}

// ---------------------------------------------------------------- checks
function segDist(q, a, b) {
  const ab = sub(b, a), t = Math.max(0, Math.min(1, dot(sub(q, a), ab) / dot(ab, ab)));
  return dist(q, add(a, mul(ab, t)));
}
function samples(q, step = 12) {
  if (q.pts.length > 2) return q.pts.filter((_, i) => i % 2 === 0).map((pt, i, arr) => ({ pt, s: (i / Math.max(1, arr.length - 1)) * q.len }));
  const n = Math.max(2, Math.ceil(q.len / step));
  return Array.from({ length: n + 1 }, (_, k) => ({ pt: add(q.pts[0], mul(sub(q.pts[1], q.pts[0]), k / n)), s: (k / n) * q.len }));
}
export function check(out, members) {
  const coll = new Map(), hyp = new Map(), viol = new Set();
  const S = out.parts.map((q) => samples(q));
  out.parts.forEach((q, i) => {
    const r = q.od / 2;
    for (const { pt, s } of S[i]) {
      if (q.kind === 'p' && s < 40) continue;
      if (pt[1] - r < LIMITS.floor - 1) viol.add(`unten (Y < ${LIMITS.floor}): ${q.name}`);
      if (pt[1] < LIMITS.sideBelowY && (pt[0] - r < LIMITS.xMin - 1 || pt[0] + r > LIMITS.xMax + 1)) viol.add(`seitlich (Rahmen innen): ${q.name}`);
      if (pt[2] - r < LIMITS.rear - 1 && q.kind !== 't') viol.add(`hinten über Rahmenende: ${q.name}`);
      if (q.kind === 't' && pt[2] < LIMITS.rear) viol.add(`Endrohr steht hinten über: ${q.name}`);
      if (q.kind !== 't' && pt[1] + r > LIMITS.top + 1) viol.add(`oben über dem Käfig (Y ${LIMITS.top}): ${q.name}`);
      for (const m of members) {
        if (m.ghost) continue;
        if (segDist(pt, m.a, m.b) < r + m.r + 5) (m.hyp ? hyp : coll).set(`${q.name.split(' (')[0]} ↔ ${m.name.split(' (')[0]}`, i);
      }
      if (!(q.kind === 'p' && s < 70)) for (const e of out.engine ?? ENGINE) {
        if (pt[0] > e.min[0] - r && pt[0] < e.max[0] + r && pt[1] > e.min[1] - r && pt[1] < e.max[1] + r && pt[2] > e.min[2] - r && pt[2] < e.max[2] + r) coll.set(`${q.name.split(' (')[0]} ↔ ${e.name.split(' (')[0]}`, i);
      }
    }
  });
  // pipe ↔ pipe, ignoring joined ends
  const skip = (q, s) => s < (q.skipStart ?? (q.kind === 'p' ? 0 : 50)) || q.len - s < (q.skipEnd ?? 50);
  for (let i = 0; i < out.parts.length; i++) for (let j = i + 1; j < out.parts.length; j++) {
    const a = out.parts[i], b = out.parts[j];
    if (a.solid || b.solid || (a.chain && a.chain === b.chain)) continue;
    let hit = false;
    for (const A of S[i]) { if (skip(a, A.s)) continue; for (const B of S[j]) { if (skip(b, B.s)) continue; if (dist(A.pt, B.pt) < a.od / 2 + b.od / 2 + 4) { hit = true; break; } } if (hit) break; }
    if (hit) coll.set(`${a.name.split(' (')[0]} ↔ ${b.name.split(' (')[0]}`, i);
  }
  let maxY = -1e9, minY = 1e9, minZ = 1e9;
  for (const q of out.parts) for (const pt of q.pts) { maxY = Math.max(maxY, pt[1] + q.od / 2); minY = Math.min(minY, pt[1] - q.od / 2); minZ = Math.min(minZ, pt[2] - (q.kind === 't' ? 0 : q.od / 2)); }
  Object.assign(out, { coll: [...coll.keys()], hyp: [...hyp.keys()], viol: [...viol], bad: new Set([...coll.values()]), maxY, minY, minZ });
  out.tunedRpm = out.primLen ? bellRpm(out.primLen.reduce((a, b) => a + b, 0) / out.primLen.length, out.ED) : null;
  return out;
}
