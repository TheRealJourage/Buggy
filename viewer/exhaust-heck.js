// Three exhaust layouts in the CAD frame (Baugr2TUV.STEP, mm): X lateral (+X = left, right-handed, Hypothese),
// Y up, Z forward (+Z = front). All frame numbers are read from the coarse 2022 model and are hypotheses.
import { bellLength, bellRpm } from './exhaust.js';

const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const mul = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const len = (a) => Math.hypot(a[0], a[1], a[2]);
const norm = (a) => mul(a, 1 / (len(a) || 1));
const dist = (a, b) => len(sub(a, b));
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];

export const TUBE = { prim: 38, sec: 63.5, main: 76.2 }; // OD; wall 1.5 → ID 35 / 60.5 / 73.2
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
  // real rear beams (User 2026-11-04): square, carry the tow hitch, ~40 mm apart in the middle; edge 50 mm assumed
  for (const x of [975, 1065]) seg('Heckbalken AHK (real quadratisch, Kante 50 angenommen)', [x, 965, -1615], [x, 1365, -1615], 35, { square: 50, hyp: true });
  // round uprights of the model at 830/1210 do not exist like this → shown as ghost only
  for (const x of [830, 1210]) seg('Heckpfosten im Modell (real nicht so)', [x, 965, -1615], [x, 1365, -1615], FR, { ghost: true });
  if (opt.upperBeam !== false) seg('Oberer Balken (fehlt im Modell, Lage geraten)', [1020, 1365, -1615], [1020, 1816, -1265], FR, { hyp: true, guess: true });
  return m;
}
export const ENGINE = [
  { name: 'Motor (Kiste im Modell, 681 × 400 × 200)', min: [660, 1220, -746], max: [1341, 1620, -546] },
  { name: 'Getriebe/Unterteil (Modell)', min: [657, 976, -746], max: [1341, 1220, -518] },
];
export const LIMITS = { floor: 989, xMin: 654, xMax: 1386, sideBelowY: 1400, rear: -1639 };

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
export const VARIANTS = {
  A: { title: 'A · 6-2-1-2 Twin-Stack', short: 'Zwei 3-in-1, Y, eine Sonde, Teilung auf zwei senkrechte Stränge mit je Kat + Dämpfer, zwei Stacks oben am Heck.' },
  B: { title: 'B · 6-1 Querdämpfer, Doppelrohr hinten', short: 'Ein 6-in-1, Sonde, Kat senkrecht in der Mitte, Querdämpfer oben, zwei Endrohre waagrecht nach hinten über den Heckkasten.' },
  C: { title: 'C · Tri-Y, Mono-Stack hinten-links', short: 'Kurze Primärrohre, zwei 3-in-1, lange Sekundärrohre, Y links hinten, ein Strang mit Kat + Dämpfer, Endrohr oben nach hinten-links gebogen.' },
};

export function build(v, o = {}) {
  const p = { yp: 1420, pitch: 65, cyl1: 'r', rpm: 5000, eo: 56, lkat: 300, dkat: 130, ldmp: 320, ddmp: 110, upperBeam: true, ...o };
  const ED = 180 + p.eo, Lp = bellLength(p.rpm, ED);
  const s1 = p.cyl1 === 'r' ? 1 : -1; // cyl 1 at low X (= right) when s1 = 1
  const ports = [0, 1, 2, 3, 4, 5].map((i) => [1000 + s1 * (i - 2.5) * p.pitch, p.yp, -746]);
  const parts = [], notes = [], marks = [];
  const part = (name, pts, od, kind, extra = {}) => { const q = { name, pts, od, kind, len: pathLen(pts), ...extra }; parts.push(q); return q; };
  const cyl = (name, a, b, d, kind, extra = {}) => part(name, [a, b], d, kind, { solid: true, ...extra });
  const cone = (name, a, b, r0, r1) => part(name, [a, b], 2 * Math.max(r0, r1), 'cone', { solid: true, r0, r1 });
  const grpOf = (ci) => (ci < 3 ? 0 : 1);
  const side = (x) => Math.sign(x - 1000) || 1;
  const R = 1.5 * TUBE.prim, Rs = 1.5 * TUBE.sec, Rm = 1.5 * TUBE.main;
  const out = { v, p, parts, notes, marks, ports, Lp, ED };

  // primaries: port → 54 straight → via(t) → pre-entry → collector slot; t solved for equal length L
  function primaries(slotsFor, viaFor, L, tMax, groupColor) {
    const res = [];
    ports.forEach((P, ci) => {
      const { slot, axis } = slotsFor(ci);
      const pre = sub(slot, mul(axis, 150)), pre2 = sub(slot, mul(axis, 60));
      const fn = (t) => fillet([P, add(P, [0, 0, -54]), ...viaFor(ci, t, P), pre, pre2, slot], R);
      const s = solve(fn, L, 0, tMax);
      if (s.short) notes.push(`Primärrohr Zyl. ${ci + 1} erreicht ${pathLen(s.pts).toFixed(0)} statt ${L.toFixed(0)} mm.`);
      res[ci] = part(`Primärrohr Zyl. ${ci + 1} (38 × 1,5)`, s.pts, TUBE.prim, 'p', { group: groupColor(ci), skipEnd: 70 });
    });
    return res;
  }
  function collector(name, c, axis, ringR, n, Lc, odOut) {
    const e = add(c, mul(axis, Lc));
    cone(name, c, e, ringR + TUBE.prim / 2 + 2, odOut / 2);
    return e;
  }
  const sonde = (at, dir, txt) => marks.push({ kind: 'sonde', at, dir, name: `LSU 4.9 in M18×1,5-Muffe – ${txt}` });
  const vband = (at, dir, r) => marks.push({ kind: 'vband', at, dir, r });

  const DOWN = [0, -1, 0], YL = 1058; // collectors point down; low runs at YL (floor 989 + r)
  if (v === 'A') {
    const g = [[0, 1, 2], [3, 4, 5]];
    const C = [0, 1].map((gi) => [1000 + s1 * (gi ? 1.5 : -1.5) * p.pitch, 1300, -1000]);
    const slots = [];
    g.forEach((grp, gi) => { const sl = xSlots(C[gi], 27, 90, grp.map((ci) => ports[ci][0])); grp.forEach((ci, k) => { slots[ci] = sl[k]; }); });
    // over the top: rise behind the head, then plunge into the downward collectors
    const prim = primaries((ci) => ({ slot: slots[ci], axis: DOWN }),
      (ci, t, P) => [[P[0], p.yp + t, -860], [P[0], p.yp + t, Math.min(-900, slots[ci][2] + 70)]], Lp, 500, grpOf);
    const outl = [0, 1].map((gi) => collector(`3-in-1-Sammler ${gi ? '{4,5,6}' : '{1,2,3}'}`, C[gi], DOWN, 27, 3, 150, TUBE.sec));
    const M = [1000, YL, -1380];
    const ySl = [0, 1].map((gi) => [1000 + s1 * (gi ? 33 : -33), YL, -1380]);
    const sec = [0, 1].map((gi) => { const O = outl[gi]; return part(`Sekundärrohr ${gi ? '{4,5,6}' : '{1,2,3}'} (63,5 × 1,5)`, fillet([O, [O[0], YL, O[2]], [O[0], YL, -1100], add(ySl[gi], [0, 0, 60]), ySl[gi]], Rs), TUBE.sec, 's', { group: gi, skipEnd: 60, skipStart: 40 }); });
    const yOut = [1000, YL, -1430];
    cone('Y-Sammler 2-in-1 (Merge, kein T)', M, yOut, 33 + TUBE.sec / 2, TUBE.main / 2);
    const split = [1000, YL, -1465];
    part('Sammelrohr 76,2 (alle 6 Zylinder)', [yOut, split], TUBE.main, 'm');
    sonde([1000, YL + 38, -1448], [0, 1, 0], '≈ 20 mm hinter dem Y-Sammler, oben senkrecht');
    [1000 - 140, 1000 + 140].forEach((x, k) => {
      const kb = [x, 1250, -1500], kt = [x, 1250 + p.lkat, -1500];
      part(`Hosenrohr-Teilung → Strang ${x > 1000 ? 'links' : 'rechts'} (63,5)`, fillet([split, [1000, YL, -1500], [x, YL, -1500], kb], 95), TUBE.sec, 'm', { skipStart: 80, chain: 'split' });
      cyl(`Kat ${k + 1} (Platzhalter Ø ${p.dkat} × ${p.lkat}, Mustang-Kat aus der X-Pipe – messen)`, kb, kt, p.dkat, 'kat');
      vband(kb, [0, 1, 0], 45); vband(kt, [0, 1, 0], 45);
      const d0 = add(kt, [0, 30, 0]), d1 = add(d0, [0, p.ldmp, 0]);
      part('Rohr 63,5', [kt, d0], TUBE.sec, 'm');
      cyl(`Dämpfer ${k + 1} (Absorption, gerader Durchgang, Ø ${p.ddmp} × ${p.ldmp}, mit E-Kennzeichnung kaufen)`, d0, d1, p.ddmp, 'dmp');
      part(`Stack ${x > 1000 ? 'links' : 'rechts'} (63,5, Mündung oben, schräg nach hinten)`, fillet([d1, add(d1, [0, 60, 0]), add(d1, [0, 110, -40])], 60), TUBE.sec, 't');
    });
    out.primLen = prim.map((q) => q.len); out.secLen = sec.map((q) => q.len);
    out.sondePath = prim.map((q, ci) => q.len + 150 + sec[grpOf(ci)].len + 50 + 18);
  }

  if (v === 'B') {
    const C = [1000, 1330, -1000];
    const sl = xSlots(C, 47, 15, ports.map((P) => P[0]));
    const prim = primaries((ci) => ({ slot: sl[ci], axis: DOWN }),
      (ci, t, P) => [[P[0], p.yp + t, -850], [P[0], p.yp + t, Math.min(-880, sl[ci][2] + 70)]], Lp, 500, grpOf);
    const o1 = collector('6-in-1-Sammler', C, DOWN, 47, 6, 180, TUBE.main);
    const zr = -1250, kb = [1000, 1250, zr], kt = [1000, 1250 + p.lkat, zr];
    part('Sammelrohr 76,2 (alle 6 Zylinder), Bogen nach hinten und hoch', fillet([o1, [1000, YL, -1000], [1000, YL, zr], kb], 110), TUBE.main, 'm');
    sonde([1000, YL + 38, -1110], [0, 1, 0], '≈ 200 mm hinter dem 6-in-1, oben senkrecht');
    cyl(`Kat (Platzhalter Ø ${p.dkat} × ${p.lkat}, Mustang-Kat – messen)`, kb, kt, p.dkat, 'kat');
    vband(kb, [0, 1, 0], 45); vband(kt, [0, 1, 0], 45);
    const dy = kt[1] + 30 + p.ddmp / 2, half = p.ldmp / 2;
    part('Rohr 76,2 → Querdämpfer', [kt, [1000, dy - p.ddmp / 2 + 5, zr]], TUBE.main, 'm');
    cyl(`Querdämpfer (Einlass Mitte, Auslass beidseitig, Ø ${p.ddmp} × ${p.ldmp}, mit E-Kennzeichnung)`, [1000 - half, dy, zr], [1000 + half, dy, zr], p.ddmp, 'dmp');
    for (const k of [-1, 1]) {
      const e = [1000 + k * half, dy, zr];
      part(`Endrohr ${k > 0 ? 'links' : 'rechts'} (57 × 1,5, waagrecht nach hinten über den Heckkasten)`, fillet([e, add(e, [k * 70, 0, 0]), add(e, [k * 70, 0, -90]), [e[0] + k * 70, dy, -1630]], 85), 57, 't', { skipStart: 30 });
    }
    out.primLen = prim.map((q) => q.len);
    out.sondePath = prim.map((q) => q.len + 180 + 230);
    if (dy + p.ddmp / 2 > 1792 - 10) notes.push('Querdämpfer zu hoch für das obere Querrohr.');
  }

  if (v === 'C') {
    const g = [[0, 1, 2], [3, 4, 5]];
    const C = [0, 1].map((gi) => [1000 + s1 * (gi ? 1.5 : -1.5) * p.pitch, 1300, -960]);
    const slots = [];
    g.forEach((grp, gi) => { const sl = xSlots(C[gi], 27, 90, grp.map((ci) => ports[ci][0])); grp.forEach((ci, k) => { slots[ci] = sl[k]; }); });
    const Lprim = p.cPrim ?? 480;
    const prim = primaries((ci) => ({ slot: slots[ci], axis: DOWN }),
      (ci, t, P) => [[P[0], p.yp + 20 + t, slots[ci][2] + 80]], Lprim, 300, grpOf);
    const outl = [0, 1].map((gi) => collector(`3-in-1-Sammler ${gi ? '{4,5,6}' : '{1,2,3}'}`, C[gi], DOWN, 27, 3, 140, TUBE.sec));
    // Y on the left (+X) rear, low; main turns up into the left stack
    const xs = 1000 + 180, M = [xs, YL, -1310];
    const ySl = [0, 1].map((gi) => [xs + (outl[gi][0] < outl[1 - gi][0] ? -33 : 33), YL, -1310]);
    const secFn = (gi) => (t) => { const O = outl[gi], S = ySl[gi]; return fillet([O, [O[0], YL, O[2]], [(O[0] + S[0]) / 2 + t, YL, -1150], add(S, [0, 0, 80]), S], Rs); };
    const Ls = Math.max(...[0, 1].map((gi) => pathLen(secFn(gi)(0))));
    const sec = [0, 1].map((gi) => {
      const s = solve(secFn(gi), Ls, 0, Math.max(0, LIMITS.xMax - 40 - (outl[gi][0] + ySl[gi][0]) / 2));
      if (s.short) notes.push(`Sekundärrohr ${gi + 1} erreicht ${pathLen(s.pts).toFixed(0)} statt ${Ls.toFixed(0)} mm.`);
      return part(`Sekundärrohr ${gi ? '{4,5,6}' : '{1,2,3}'} (63,5 × 1,5)`, s.pts, TUBE.sec, 's', { group: gi, skipEnd: 70, skipStart: 40 });
    });
    const yOut = add(M, [0, 0, -50]);
    cone('Y-Sammler 2-in-1', M, yOut, 33 + TUBE.sec / 2, TUBE.main / 2);
    const kb = [xs, 1250, -1470], kt = [xs, 1250 + p.lkat, -1470];
    part('Sammelrohr 76,2 (alle 6 Zylinder), Bogen nach oben', fillet([yOut, [xs, YL, -1470], kb], 110), TUBE.main, 'm');
    sonde([xs, YL + 38, -1385], [0, 1, 0], '≈ 25 mm hinter dem Y, oben senkrecht');
    cyl(`Kat (Platzhalter Ø ${p.dkat} × ${p.lkat}, Mustang-Kat – messen)`, kb, kt, p.dkat, 'kat');
    vband(kb, [0, 1, 0], 45); vband(kt, [0, 1, 0], 45);
    const d0 = add(kt, [0, 30, 0]), d1 = add(d0, [0, p.ldmp, 0]);
    part('Rohr 76,2', [kt, d0], TUBE.main, 'm');
    cyl(`Dämpfer (Absorption, gerader Durchgang, Ø ${p.ddmp} × ${p.ldmp}, mit E-Kennzeichnung)`, d0, d1, p.ddmp, 'dmp');
    const dirTip = norm([0.5, 0, -0.866]); // 30° to the left of straight back
    part('Endrohr (76,2, Hockeyschläger, Mündung hinten-links 30°)', fillet([d1, add(d1, [0, 70, 0]), add(add(d1, [0, 70, 0]), mul(dirTip, 140))], 110), TUBE.main, 't');
    out.primLen = prim.map((q) => q.len); out.secLen = sec.map((q) => q.len);
    out.sondePath = prim.map((q, ci) => q.len + 140 + sec[grpOf(ci)].len + 50 + 25);
  }
  check(out, frame({ upperBeam: p.upperBeam }));
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
      for (const m of members) {
        if (m.ghost) continue;
        if (segDist(pt, m.a, m.b) < r + m.r + 5) (m.hyp ? hyp : coll).set(`${q.name.split(' (')[0]} ↔ ${m.name.split(' (')[0]}`, i);
      }
      if (!(q.kind === 'p' && s < 70)) for (const e of ENGINE) {
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
