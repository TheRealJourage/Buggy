// Enclosure draft at the steering column (decisions user 2026-11-07, Kabelbaum-Übersicht.md).
// Same frame as lenkraum.js: u from measuring line "vorne" (= driver side) towards the servo,
// y lateral (+ = left in driving direction), z up from the bottom of the 45 mm free space.
// Every size not measured is a hypothesis and lives in HYP so it can be replaced after the Halle visit.
import { MEASURED as M, widths, colTopZ, colBottomZ } from './lenkraum.js';

export const HYP = {
  wall: 3,            // PETG wall
  sideGap: 5,         // clearance to frame/servo/column
  frontT: 10,         // SLA front plate, as the switch panels (thread needs 10 mm)
  alu: 3,             // aluminium bottom plate, recessed into the shell
  rdBoard: [45.5, 38], // GC9A01 board w x h (dealer 38 x 45.5), turned so 38 mm is vertical
  rdWindow: 35,       // visible round glass
  rdPitch: 55,
  swPitch: 36,        // PITCH_X in tools/panels.py
  capW: 17.4, capH: 41.3, capTop: 27.6, // cover plate and hole centre -> cover top (panels.py)
  swBody: [13, 13, 30], // toggle body behind the plate w x h x depth
  potBody: 17, knob: 24,
  nucleo: [82.5, 70, 30], pico: [51, 21, 10], xcvr: [40, 25, 10],
  disp: [193, 111], dispHousing: { l: 45, rim: 6, tilt: 20 },
  pi4: [85, 56, 25],
  tube: 16,
};

export const BEDS = {
  'Bambu Lab A1': [256, 256, 256],
  'Anycubic M7 Max (Hersteller, ungeprüft)': [298, 164, 300],
};

// Panel "Motor" in one row, from the left in driver view (Bordnetz-Konzept 2026-11-07).
export const MOTOR_ROW = [
  { label: 'Zündung', type: 'A', cap: true },
  { label: 'Motor', type: 'A', cap: true },
  { label: 'Lüfter A/M', type: 'A/C', cap: false },
  { label: 'Start', type: 'T', cap: true },
  { label: 'Kompressor', type: 'B', cap: true },
  { label: 'Lüfter-Poti', type: 'Poti', cap: false, pot: true },
];

export function sideGeom(side, p) {
  const s = side === 'left' ? 1 : -1;
  const inner = Math.max(M.steelW, p.colDia) / 2 + HYP.sideGap;
  const uMax = (side === 'left' ? M.lenLeft : M.lenRight) - HYP.sideGap;
  const outer = (u) => M.steelW / 2 + widths(u, p)[side] - HYP.sideGap;
  const us = [...new Set([0, Math.min(p.lineGap, uMax), uMax])];
  const outerPts = us.map((u) => [u, s * outer(u)]);
  const poly = [[0, s * inner], ...outerPts, [uMax, s * inner]];
  return { s, inner, uMax, outer, poly };
}

function wallPrisms(poly, t, z0, z1, skip) {
  const c = poly.reduce((a, q) => [a[0] + q[0] / poly.length, a[1] + q[1] / poly.length], [0, 0]);
  const out = [];
  for (let i = 0; i < poly.length; i++) {
    if (skip.includes(i)) continue;
    const a = poly[i], b = poly[(i + 1) % poly.length];
    const du = b[0] - a[0], dy = b[1] - a[1], L = Math.hypot(du, dy);
    let n = [-dy / L, du / L];
    if ((c[0] - a[0]) * n[0] + (c[1] - a[1]) * n[1] < 0) n = [-n[0], -n[1]];
    out.push([a, b, [b[0] + n[0] * t, b[1] + n[1] * t], [a[0] + n[0] * t, a[1] + n[1] * t]].map(([u, y]) => [u, y]));
  }
  return out.map((pts) => ({ kind: 'prism', pts, z0, z1 }));
}

const box = (o) => ({ kind: 'box', tilt: 0, ...o });

export function design(p) {
  const H = HYP, faceH = p.faceH, parts = [];
  const add = (group, name, o, extra = {}) => parts.push({ group, name, ...o, ...extra });

  for (const side of ['left', 'right']) {
    const g = sideGeom(side, p), tag = side === 'left' ? 'links' : 'rechts';
    // poly edges: 0 = front (SLA plate), last = inner wall at the column
    for (const w of wallPrisms(g.poly, H.wall, 0, faceH, [0]))
      add('schale', `PETG-Box ${tag}`, w, { color: '#4b5563', opacity: 0.45 });
    add('schale', `PETG-Box ${tag} Decke`, { kind: 'prism', pts: g.poly, z0: faceH - H.wall, z1: faceH }, { color: '#4b5563', opacity: 0.3 });
    add('alu', `Alu-Boden ${tag}`, { kind: 'prism', pts: g.poly, z0: 0, z1: H.alu }, { color: '#cbd5e1', opacity: 0.55, note: 'von unten mit Dichtmasse verschraubt' });
    const y0 = g.s * g.inner, y1 = g.s * g.outer(0);
    add('front', `SLA-Front ${tag}`, box({ u: H.frontT / 2, l: H.frontT, y: (y0 + y1) / 2, w: Math.abs(y1 - y0), z: 0, h: faceH }), { color: '#27272a', opacity: 0.9 });
  }

  // Left: 3 round displays, Pico, Nucleo, transceivers hanging from the ceiling.
  const L = sideGeom('left', p);
  const ceil = faceH - H.wall;
  const yc = (L.inner + L.outer(0)) / 2;
  const zc = faceH / 2;
  [-1, 0, 1].forEach((k, i) => {
    const y = yc + k * H.rdPitch;
    add('innen', `Rund-Display ${i + 1} (Platine)`, box({ u: H.frontT + 4, l: 8, y, w: H.rdBoard[0], z: zc - H.rdBoard[1] / 2, h: H.rdBoard[1] }), { color: '#f59e0b', check: true });
    add('front', `Rund-Display ${i + 1} Glas`, { kind: 'cyl', axis: 'u', u: -0.6, y, z: zc, r: H.rdWindow / 2, len: 1.2 }, { color: '#0b1220' });
  });
  const [pl, pw, ph] = H.pico;
  add('innen', 'Display-Pico', box({ u: 28 + pl / 2, l: pl, y: 80, w: pw, z: ceil - ph, h: ph }), { color: '#22c55e', check: true });
  const [nl, nw, nh] = H.nucleo;
  add('innen', 'Nucleo-F446RE', box({ u: 45 + nl / 2, l: nl, y: 165, w: nw, z: ceil - nh, h: nh }), { color: '#3b82f6', check: true, note: 'Höhe mit Stiftleisten Hypothese' });
  const [xl, xw, xh] = H.xcvr;
  add('innen', 'LIN/CAN-Transceiver', box({ u: 88 + xl / 2, l: xl, y: 80, w: xw, z: ceil - xh, h: xh }), { color: '#14b8a6', check: true, note: 'MCP2003 + SN65HVD230, Größe Hypothese' });

  // Right: panel "Motor" in one row.
  const R = sideGeom('right', p);
  const ycR = -(R.inner + R.outer(0)) / 2;
  const holeZ = faceH - H.capTop - 2;
  MOTOR_ROW.forEach((e, i) => {
    const y = ycR + ((MOTOR_ROW.length - 1) / 2 - i) * H.swPitch;
    if (e.pot) {
      add('front', `${e.label} Knopf`, { kind: 'cyl', axis: 'u', u: -8, y, z: holeZ, r: H.knob / 2, len: 16 }, { color: '#111827', label: e.label });
      add('innen', `${e.label} Körper`, box({ u: H.frontT + 10, l: 20, y, w: H.potBody, z: holeZ - H.potBody / 2, h: H.potBody }), { color: '#a3a3a3', check: true });
    } else {
      add('front', `${e.label} Schalter`, { kind: 'cyl', axis: 'u', u: -0.6, y, z: holeZ, r: 6.1, len: 1.2 }, { color: '#0b0b0b', label: e.label });
      if (e.cap) add('front', `${e.label} Kappe`, box({ u: -3, l: 6, y, w: H.capW, z: holeZ + H.capTop - H.capH, h: H.capH }), { color: '#dc2626', opacity: 0.85 });
      const [bw, bh, bd] = H.swBody;
      add('innen', `${e.label} Körper`, box({ u: H.frontT + bd / 2, l: bd, y, w: bw, z: holeZ - bh / 2, h: bh }), { color: '#a3a3a3', check: true });
    }
  });

  // Bridge over the column and the 7" display housing with the Pi 4 behind it.
  const D = H.dispHousing, hw = H.disp[0] / 2 + D.rim, hh = H.disp[1] + 2 * D.rim;
  const base = Math.max(faceH, colTopZ(p) + 5) + H.wall;
  add('oben', 'Brücke Deckplatte', box({ u: 30, l: 60, y: 0, w: 2 * hw, z: base - H.wall, h: H.wall }), { color: '#4b5563', opacity: 0.7 });
  if (base - H.wall > faceH + 0.5)
    for (const s of [1, -1])
      add('oben', 'Brücke Fuß', box({ u: 30, l: 60, y: s * (L.inner + 5), w: 10, z: faceH, h: base - H.wall - faceH }), { color: '#4b5563', opacity: 0.7 });
  const a = D.tilt * Math.PI / 180, uc = 8 + D.l / 2, zcH = base + hh / 2 + D.l / 2 * Math.sin(a);
  add('oben', '7"-Gehäuse (SLA-Rahmen + PETG-Rücken)', box({ u: uc, l: D.l, y: 0, w: 2 * hw, z: zcH - hh / 2, h: hh, tilt: D.tilt }), { color: '#3f3f46', opacity: 0.55 });
  add('oben', '7"-Display Glas', box({ u: uc - (D.l / 2 + 0.6) * Math.cos(a), l: 1.2, y: 0, w: H.disp[0], z: zcH + (D.l / 2 + 0.6) * Math.sin(a) - H.disp[1] / 2, h: H.disp[1], tilt: D.tilt }), { color: '#0b1220' });
  const [p4w, p4h, p4d] = H.pi4, off = D.l / 2 - p4d / 2 - 2;
  add('oben', 'Pi 4 + Passiv-Kühler', box({ u: uc + off * Math.cos(a), l: p4d, y: 0, w: p4w, z: zcH - off * Math.sin(a) - p4h / 2, h: p4h, tilt: D.tilt }), { color: '#a855f7', opacity: 0.9 });

  // Cable tube under the column between the two boxes.
  const cb = colBottomZ(p);
  add('kabel', 'Kabelrohr mit Tüllen', { kind: 'cyl', axis: 'y', u: 100, y: 0, z: Math.min(cb, faceH) / 2, r: H.tube / 2, len: 2 * L.inner + 2 * H.wall },
    { color: '#38bdf8', note: cb < H.tube + 2 ? `nur ${Math.round(cb)} mm unter der Säule (Hypothese)` : '' });

  return { parts, holeZ, base };
}

// Smallest rotation on the bed that fits a part with bounding dims d (mm); null if none.
export function bedFit(d, bed, margin = 3) {
  const [bx, by, bz] = bed.map((v) => v - margin);
  const orders = [[0, 1, 2], [0, 2, 1], [1, 2, 0]];
  const names = ['liegend', 'auf der Frontseite stehend', 'hochkant'];
  let best = null;
  orders.forEach(([i, j, k], n) => {
    if (d[k] > bz) return;
    for (let deg = 0; deg <= 90; deg++) {
      const r = deg * Math.PI / 180, c = Math.cos(r), s = Math.sin(r);
      if (d[i] * c + d[j] * s <= bx && d[i] * s + d[j] * c <= by) {
        if (!best || deg < best.deg) best = { pose: names[n], deg };
        return;
      }
    }
  });
  return best;
}

export function shellDims(side, p) {
  const g = sideGeom(side, p);
  return [Math.round(g.outer(0) - g.inner), Math.round(g.uMax), p.faceH];
}
