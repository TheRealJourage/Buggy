import test from 'node:test';
import assert from 'node:assert/strict';
import * as H from '../exhaust-heck.js';

for (const v of ['A', 'B', 'C']) {
  test(`layout ${v} fits the CAD frame without collisions (defaults, both cylinder-1 sides)`, () => {
    for (const cyl1 of ['r', 'l']) {
      const o = H.build(v, { cyl1 });
      assert.deepEqual(o.coll, [], o.coll.join('; '));
      assert.deepEqual(o.viol, [], o.viol.join('; '));
      assert.deepEqual(o.hyp, [], o.hyp.join('; '));
      assert.deepEqual(o.notes, []);
    }
  });
  test(`layout ${v}: equal primaries, sonde sees all 6 cylinders far enough downstream`, () => {
    const o = H.build(v);
    const L = o.primLen;
    assert.ok(Math.max(...L) - Math.min(...L) < 2, L.join('/'));
    if (o.secLen) assert.ok(Math.abs(o.secLen[0] - o.secLen[1]) < 2, o.secLen.join('/'));
    assert.ok(Math.min(...o.sondePath) >= 600, `sonde ${Math.min(...o.sondePath)}`);
    assert.ok(o.minZ >= H.LIMITS.rear);
  });
}

test('A and B primaries hit the Bell length for the target rpm', () => {
  for (const v of ['A', 'B']) for (const rpm of [4500, 5000, 5500]) {
    const o = H.build(v, { rpm });
    assert.ok(Math.abs(o.tunedRpm - rpm) < 30, `${v} ${rpm} → ${o.tunedRpm}`);
  }
});

test('B reports the cross muffler hitting the cage tube with a too long cat', () => {
  const o = H.build('B', { lkat: 450 });
  assert.ok(o.coll.some((c) => c.includes('Oberes Querrohr')));
});
