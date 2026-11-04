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

test('A and B primaries reach the Bell length for 5500/min', () => {
  for (const v of ['A', 'B']) {
    const o = H.build(v, { rpm: 5500 });
    assert.ok(Math.abs(o.tunedRpm - 5500) < 30, `${v} → ${o.tunedRpm}`);
  }
});

test('longer primaries (5000/min) run into the cage cross tube', () => {
  for (const v of ['A', 'B']) {
    const o = H.build(v, { rpm: 5000 });
    assert.ok(o.coll.some((c) => c.includes('Oberes Querrohr')), v);
  }
});

test('nothing runs through the engine envelope', () => {
  for (const v of ['A', 'B', 'C']) assert.ok(!H.build(v).coll.some((c) => c.includes('Motor')), v);
});
