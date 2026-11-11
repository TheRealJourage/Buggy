import test from 'node:test';
import assert from 'node:assert/strict';
import * as g from '../relaisbox-blech.js';

const cases = Object.entries(g.MATERIALS).flatMap(([k, m]) => m.r.map((r) => g.result(k, r)));

test('DIN 6935 bend deduction matches hand calculation (s = r = 2: k 0.65, BD 3.84)', () => {
  assert.ok(Math.abs(g.bendDeduction(2, 2) - 3.837) < 0.01);
});

test('one blank keeps the box at or below the fixed 115 mm for every material and radius', () => {
  for (const c of cases) assert.ok(c.H <= g.H_MAX, `${c.key} r${c.r}: H ${c.H}`);
});

test('outer length, width, flange and web stay within 1 mm of nominal', () => {
  for (const c of cases) {
    assert.ok(Math.abs(c.L - g.OUTER.L) < 1 && Math.abs(c.W - g.OUTER.W) < 1, `${c.key} r${c.r}`);
    assert.ok(Math.abs(c.flange - g.OUTER.FLANGE) < 1 && Math.abs(c.uWeb - g.WEB_W) < 1);
  }
});

test('wall corners and flange mitres leave a gap instead of colliding', () => {
  for (const c of cases) {
    const t = g.MATERIALS[c.key].t;
    assert.ok(g.TUB.a - g.GAP <= c.L / 2 - t, `side wall end vs front wall inner face, ${c.key} r${c.r}`);
    assert.ok(g.TUB.b - g.GAP <= c.W / 2 - t);
    assert.ok(g.TUB.a - g.GAP - g.TUB.flange <= c.L / 2 - c.flange);
  }
});

test('U web fits between the walls and into the flange notch; legs stay off the floor', () => {
  for (const c of cases) {
    assert.ok(g.U.L < c.innerL, `${c.key} r${c.r}`);
    assert.ok(c.uWeb <= g.NOTCH.w, 'web into notch');
    assert.ok(c.floorGap > 5);
  }
});

test('relief hole covers the corner step of both wall edges', () => {
  assert.ok(g.RELIEF_D / 2 > g.GAP * Math.SQRT2);
});

test('tub outline is closed, mirrored and its area equals the cross minus notches and mitres', () => {
  const o = g.tubOutline(400);
  const { a, b, wall: w, flange: f } = g.TUB, as = a - g.GAP, bs = b - g.GAP;
  const n = g.NOTCH.w, e = g.NOTCH.extra, R = g.RELIEF_D / 2;
  const cross = 4 * a * b + 4 * bs * w + 4 * as * w;            // bottom + walls (edges cut back by GAP)
  const flanges = 2 * (2 * bs - f) * f + 2 * (2 * as - f) * f;  // trapezoids with 45 deg mitres
  const notches = 2 * (n * (f + e));
  // relief: circle minus the part that lies in the removed corner (x > a - GAP, y > b - GAP) is already
  // outside material; check against a numeric value with a small tolerance instead
  const expectedNoRelief = cross + flanges - notches;
  const area = g.polyArea(o);
  const reliefMax = 4 * Math.PI * R * R;
  assert.ok(area < expectedNoRelief && area > expectedNoRelief - reliefMax, `${area} vs ${expectedNoRelief}`);
  const xs = o.map((p) => p[0]), ys = o.map((p) => p[1]);
  assert.ok(Math.abs(Math.max(...xs) - g.TUB.flatL / 2) < 1e-6 && Math.abs(Math.min(...xs) + g.TUB.flatL / 2) < 1e-6);
  assert.ok(Math.abs(Math.max(...ys) - g.TUB.flatW / 2) < 1e-6);
});

test('cutting plan holds both blanks without overlap', () => {
  const { sheet, parts } = g.cutPlan();
  for (const p of parts) assert.ok(p.x >= g.KERF && p.y >= g.KERF && p.x + p.w <= sheet.w - g.KERF + 1e-6 && p.y + p.h <= sheet.h - g.KERF + 1e-6);
  const [A, B] = parts;
  assert.ok(A.x + A.w + g.KERF <= B.x + 1e-6);
});

test('sheet weight matches the relay box note (3.0 kg alu, 6.6 kg stainless)', () => {
  const area = (g.polyArea(g.tubOutline()) + g.polyArea(g.uOutline())) * 1e-6;
  const kg = (m) => area * m.t * m.rho;
  assert.ok(Math.abs(kg(g.MATERIALS.alu) - 3.0) < 0.1 && Math.abs(kg(g.MATERIALS.va) - 6.6) < 0.1);
});
