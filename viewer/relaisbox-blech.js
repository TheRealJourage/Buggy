// Relay box sheet-metal parts: flat patterns valid for both materials, cutting plan.
// Outer-dimension method: flat = sum(outer legs) - n * BD. One common BD is used for the cut;
// the wall leg is sized with the largest BD so the box never exceeds the fixed 115 mm height.
// Bend deduction after DIN 6935 (90 deg): k = 0.65 + 0.5 lg(r/s), BA = pi/2 (r + s/2 k), BD = 2 (r + s) - BA.

export const OUTER = { L: 500, W: 430, H: 115, FLANGE: 25 };
export const H_MAX = 115; // fixed by user 2026-11-01 (120 mm to the handbrake cable)

export const MATERIALS = {
  va: { name: 'Edelstahl 1.4301, 1,5 mm', t: 1.5, rho: 7.9, r: [1.5, 2] },
  alu: { name: 'Alu AlMg3, 2 mm', t: 2, rho: 2.7, r: [2, 3] },
};

export function bendDeduction(s, r) {
  const k = Math.min(1, 0.65 + 0.5 * Math.log10(r / s));
  const ba = (Math.PI / 2) * (r + (s / 2) * k);
  return 2 * (r + s) - ba;
}

const r1 = (v) => Math.round(v * 10) / 10;
const allBD = Object.values(MATERIALS).flatMap((m) => m.r.map((r) => bendDeduction(m.t, r)));
export const BD_MIN = Math.min(...allBD), BD_MAX = Math.max(...allBD);
export const BD = r1((BD_MIN + BD_MAX) / 2); // common value for the cut

export const GAP = 0.5;          // per edge at wall corners and flange mitres, closed by the weld
export const RELIEF_D = 8;       // corner relief hole at the crossing of the bottom bend lines
export const WEB_W = 50;         // centre web (lid sealing face), outer width
export const NOTCH = { w: WEB_W + 2 * GAP, extra: 2 }; // flange notch for the web, 2 mm relief into the wall

// ---------- Pos 1: tub, one blank (bottom + 4 walls + 4 inward flanges)
const wallFlat = Math.floor((H_MAX - BD_MAX - 0.3) * 2) / 2; // 0.3 mm reserve, rounded down to 0.5
export const TUB = {
  wall: wallFlat,
  flange: r1(OUTER.FLANGE - BD / 2),
  a: r1((OUTER.L - BD) / 2), // half bottom between bend lines, x (length)
  b: r1((OUTER.W - BD) / 2), // half bottom between bend lines, y (width)
};
TUB.ext = TUB.wall + TUB.flange;
TUB.flatL = r1(2 * (TUB.a + TUB.ext));
TUB.flatW = r1(2 * (TUB.b + TUB.ext));

// outline of the tub blank, centred on the bottom; front = +x, left = +y.
// Corner: the wall side edges stop GAP short of the neighbour's bend line; the relief hole
// (centre on the crossing of the bottom bend lines) is part of the contour.
export function tubOutline(arcSeg = 16) {
  const { a, b, wall: w, flange: f } = TUB;
  const R = RELIEF_D / 2, as = a - GAP, bs = b - GAP, n = NOTCH.w / 2, e = NOTCH.extra;
  const q = [[a + w - e, 0], [a + w - e, n], [a + w + f, n], [a + w + f, bs - f], [a + w, bs]];
  const a0 = -Math.asin(GAP / R), a1 = -Math.PI - a0 - Math.PI / 2; // -7.2 deg .. -262.8 deg (clockwise)
  for (let k = 0; k <= arcSeg; k++) {
    const t = a0 + ((a1 - a0) * k) / arcSeg;
    q.push([a + R * Math.cos(t), b + R * Math.sin(t)]);
  }
  q.push([as, b + w], [as - f, b + w + f], [0, b + w + f]);
  const Q2 = q.slice().reverse().map(([x, y]) => [-x, y]).slice(1);
  const Q3 = q.map(([x, y]) => [-x, -y]).slice(1);
  const Q4 = q.slice().reverse().map(([x, y]) => [x, -y]).slice(1, -1);
  return [...q, ...Q2, ...Q3, ...Q4];
}

export const tubBendLines = () => {
  const { a, b, wall: w } = TUB;
  return [
    { id: 'B1', kind: 'bottom', p: [[a, -b], [a, b]] }, { id: 'B2', kind: 'bottom', p: [[-a, -b], [-a, b]] },
    { id: 'B3', kind: 'bottom', p: [[-a, b], [a, b]] }, { id: 'B4', kind: 'bottom', p: [[-a, -b], [a, -b]] },
    { id: 'F1', kind: 'flange', p: [[a + w, -(b - GAP)], [a + w, -NOTCH.w / 2]] }, { id: 'F1', kind: 'flange', p: [[a + w, NOTCH.w / 2], [a + w, b - GAP]] },
    { id: 'F2', kind: 'flange', p: [[-a - w, -(b - GAP)], [-a - w, -NOTCH.w / 2]] }, { id: 'F2', kind: 'flange', p: [[-a - w, NOTCH.w / 2], [-a - w, b - GAP]] },
    { id: 'F3', kind: 'flange', p: [[-(a - GAP), b + w], [a - GAP, b + w]] }, { id: 'F4', kind: 'flange', p: [[-(a - GAP), -b - w], [a - GAP, -b - w]] },
  ];
};
export const reliefHoles = () => [[1, 1], [1, -1], [-1, 1], [-1, -1]].map(([sx, sy]) => [sx * TUB.a, sy * TUB.b]);

// ---------- Pos 2: centre web as an inverted U (web 50 = sealing face, two legs hang down)
export const U = { L: 495, web: WEB_W, leg: 100, notch: { w: 80, h: 40, r: 5, x: [-150, 0, 150] } };
U.webFlat = r1(U.web - BD);
U.legFlat = r1(U.leg - BD / 2);
U.flatW = r1(U.webFlat + 2 * U.legFlat);

const arc = (cx, cy, r, a0, a1, n = 6) => Array.from({ length: n + 1 }, (_, k) => {
  const t = a0 + ((a1 - a0) * k) / n;
  return [cx + r * Math.cos(t), cy + r * Math.sin(t)];
});

// U outline: u along the length (centred), w across the flat (0 .. flatW); cable notches with
// rounded corners in both leg edges (legs hang down, notches end up at the floor side)
export function uOutline() {
  const { L, flatW, notch: { w: nw, h: nh, r, x: nx } } = U, h = L / 2, P = Math.PI;
  const pts = [[-h, 0]];
  for (const x of nx) {
    const x0 = x - nw / 2, x1 = x + nw / 2;
    pts.push([x0, 0], ...arc(x0 + r, nh - r, r, P, P / 2), ...arc(x1 - r, nh - r, r, P / 2, 0), [x1, 0]);
  }
  pts.push([h, 0], [h, flatW]);
  for (const x of nx.slice().reverse()) {
    const x0 = x - nw / 2, x1 = x + nw / 2;
    pts.push([x1, flatW], ...arc(x1 - r, flatW - nh + r, r, 0, -P / 2), ...arc(x0 + r, flatW - nh + r, r, -P / 2, -P), [x0, flatW]);
  }
  pts.push([-h, flatW]);
  return pts;
}
export const uBendLines = () => [U.legFlat, U.legFlat + U.webFlat].map((w) => [[-U.L / 2, w], [U.L / 2, w]]);

// ---------- assembled result per material and bend radius
export function result(key, r) {
  const m = MATERIALS[key], bd = bendDeduction(m.t, r);
  return {
    key, r, bd,
    L: 2 * TUB.a + bd, W: 2 * TUB.b + bd,
    H: TUB.wall + bd, flange: TUB.flange + bd / 2,
    uWeb: U.webFlat + bd, uLeg: U.legFlat + bd / 2,
    innerL: 2 * TUB.a + bd - 2 * m.t,
    floorGap: TUB.wall + bd - m.t - (U.legFlat + bd / 2),
  };
}

// ---------- cutting plan: tub blank + U on one sheet, 10 mm edge, 10 mm between parts
export const KERF = 10;
export function cutPlan() {
  const tub = { w: TUB.flatW, h: TUB.flatL }; // tub rotated: width across the sheet
  const u = { w: U.flatW, h: U.L };
  const W = r1(KERF + tub.w + KERF + u.w + KERF), H = r1(KERF + tub.h + KERF);
  return {
    sheet: { w: Math.ceil(W / 10) * 10, h: Math.ceil(H / 10) * 10 },
    parts: [
      { pos: 1, x: KERF, y: KERF, w: tub.w, h: tub.h, rot: true },
      { pos: 2, x: KERF + tub.w + KERF, y: KERF, w: u.w, h: u.h, rot: true },
    ],
  };
}

export function polyArea(pts) {
  let s = 0;
  for (let i = 0; i < pts.length; i++) { const [x1, y1] = pts[i], [x2, y2] = pts[(i + 1) % pts.length]; s += x1 * y2 - x2 * y1; }
  return Math.abs(s) / 2;
}
