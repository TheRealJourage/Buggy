import test from 'node:test';
import assert from 'node:assert/strict';
import * as H from '../exhaust-heck.js';

for (const v of ['A', 'B', 'C']) {
  test(`layout ${v} fits behind the STEP engine without collisions (defaults, both cylinder-1 sides)`, () => {
    for (const cyl1 of ['r', 'l']) {
      const o = H.build(v, { cyl1, upperBeam: false });
      assert.deepEqual([...o.coll, ...o.viol, ...o.notes], [], `${v} ${cyl1}`);
    }
  });
  test(`layout ${v}: equal runners, sonde sees all 6 cylinders far enough downstream`, () => {
    const o = H.build(v);
    const L = o.primLen;
    if (v !== 'C') assert.ok(Math.max(...L) - Math.min(...L) < 2, L.join('/'));
    assert.ok(Math.abs(o.secLen?.[0] - o.secLen?.[1]) < 2 || !o.secLen, String(o.secLen));
    assert.ok(Math.min(...o.sondePath) >= 600, `sonde ${Math.min(...o.sondePath)}`);
    assert.ok(o.minZ >= H.LIMITS.rear);
  });
}

test('A and B primaries reach the Bell length for 4500/min', () => {
  for (const v of ['A', 'B']) {
    const o = H.build(v, { rpm: 4500 });
    assert.ok(Math.abs(o.tunedRpm - 4500) < 30, `${v} → ${o.tunedRpm}`);
  }
});

test('optimised layouts (2026-11-06): default 4500/min clear of everything incl. the guessed middle beam, ±20 mm reserve', () => {
  const clean = (o) => [...o.coll, ...o.viol, ...o.notes, ...o.hyp];
  for (const v of ['A', 'B']) {
    for (const cyl1 of ['r', 'l']) assert.deepEqual(clean(H.build(v, { cyl1 })), [], `${v} ${cyl1}`);
    for (const [k, x] of Object.entries(H.LAYOUT[v])) for (const d of [-20, 20]) {
      assert.deepEqual(clean(H.build(v, { [k]: x + d })), [], `${v} ${k}${d}`);
    }
  }
});

test('A and B: tuning far below the limit runs into the frame or the beam', () => {
  for (const v of ['A', 'B']) {
    const o = H.build(v, { rpm: 3800 });
    assert.ok(o.coll.length + o.hyp.length + o.notes.length > 0, v);
  }
});

test('nothing runs through the engine envelope', () => {
  for (const v of ['A', 'B', 'C']) assert.ok(!H.build(v).coll.some((c) => c.includes('Motor')), v);
});

test('default engine offset reproduces the Halle gaps (top/rear exact, sides within 15 mm)', () => {
  const c = H.halleCheck();
  assert.ok(Math.abs(c.top - H.HALLE.top) <= 1 && Math.abs(c.rear - H.HALLE.rear) <= 1, JSON.stringify(c));
  assert.ok(Math.abs(c.left - H.HALLE.left) <= 15 && Math.abs(c.right - H.HALLE.right) <= 15, JSON.stringify(c));
  assert.ok(H.OFFSET.dy < 0, 'engine sits lower than in the model');
});

test('offset zero reproduces the old model gaps (≈ 23 cm above the manifold)', () => {
  const c = H.halleCheck({ dx: 0, dy: 0, dz: 0 });
  assert.ok(Math.abs(c.top - 231) < 2, String(c.top));
});
