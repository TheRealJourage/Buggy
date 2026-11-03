import test from 'node:test';
import assert from 'node:assert/strict';
import * as x from '../exhaust.js';

const close = (a, b, tol) => assert.ok(Math.abs(a - b) <= tol, `${a} != ${b} ±${tol}`);

test('Bell length matches the research report (ED 236)', () => {
  close(x.bellLength(5500, 236), 850, 5);
  close(x.bellLength(4500, 236), 1057, 5);
  close(x.bellLength(3200, 236), 1517, 5);
  close(x.bellRpm(x.bellLength(4000, 236), 236), 4000, 1e-6);
});

test('primary ID formula matches the research report', () => {
  close(x.primaryIdForRpm(3200), 29, 0.5);
  close(x.primaryIdForRpm(4500), 34.5, 0.5);
  close(x.primaryIdForRpm(6200), 40.5, 0.5);
  close(x.rpmForPrimaryId(x.primaryIdForRpm(5000)), 5000, 1e-6);
});

test('split keeps the flow area', () => {
  close(2 * x.area(x.splitId(60, 2)), x.area(60), 1e-6);
  close(4 * x.area(x.splitId(60, 4)), x.area(60), 1e-6);
});

test('groups {1,2,3} and {4,5,6} fire evenly, banks do not', () => {
  assert.deepEqual(x.firingGaps([1, 2, 3]), [240, 240, 240]);
  assert.deepEqual(x.firingGaps([4, 5, 6]), [240, 240, 240]);
  assert.deepEqual(x.firingGaps([1, 3, 5]), [120, 120, 480]);
});
