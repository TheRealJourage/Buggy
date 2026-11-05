import { test } from 'node:test';
import assert from 'node:assert/strict';
import { widths, check, defaultLayout, colBottomZ, DEFAULT_PARAMS as P, MEASURED } from '../lenkraum.js';

const box = (o) => ({ id: 99, name: 't', l: 40, w: 40, h: 20, u: 50, y: 100, z: 0, tilt: 0, ...o });

test('widths interpolate between the measuring lines', () => {
  assert.deepEqual(widths(0, P), { left: 290, right: 315 });
  assert.deepEqual(widths(P.lineGap, P), { left: 190, right: 235 });
  assert.deepEqual(widths(P.lineGap * 2, P), { left: 190, right: 235 });
});

test('box inside the side space is green', () => {
  assert.equal(check(box(), P).status, 'green');
});

test('box taller than 45 mm is yellow with sight height', () => {
  const r = check(box({ h: MEASURED.slabH + 20 }), P);
  assert.equal(r.status, 'yellow');
  assert.equal(r.sight, 20);
});

test('flat box under the column is green with cable info', () => {
  const r = check(box({ y: 0, h: colBottomZ(P) }), P);
  assert.equal(r.status, 'green');
  assert.match(r.info.join(), /Kabel/);
});

test('left side ends earlier at the servo motor', () => {
  assert.equal(check(box({ u: 130 }), P).status, 'red');
  assert.equal(check(box({ u: 130, y: -100 }), P).status, 'green');
});

test('box reaching into the column collides', () => {
  const r = check(box({ y: 0, h: colBottomZ(P) + 5 }), P);
  assert.equal(r.status, 'red');
  assert.match(r.errors.join(), /Lenksäule/);
});

test('box below the cross beam or sideways out is red', () => {
  assert.equal(check(box({ z: -5 }), P).status, 'red');
  assert.equal(check(box({ y: 300 }), P).status, 'red');
});

test('overlapping boxes are reported', () => {
  const a = box(), b = box({ id: 2, name: 'b', u: 60 });
  assert.match(check(a, P, [a, b]).errors.join(), /überlappt/);
});

test('tilt moves corners', () => {
  assert.equal(check(box({ l: 12, h: 111, tilt: -20, u: 140, y: -100 }), P).status, 'red');
});

test('default layout has no red boxes', () => {
  const L = defaultLayout(P);
  for (const b of L) assert.notEqual(check(b, P, L).status, 'red', b.name + ': ' + check(b, P, L).errors);
});
