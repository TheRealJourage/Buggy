import { test } from 'node:test';
import assert from 'node:assert/strict';
import { widths, check, defaultLayout, DEFAULT_PARAMS as P, MEASURED } from '../lenkraum.js';

const box = (o) => ({ id: 99, name: 't', l: 40, w: 40, h: 20, u: 50, y: 100, z: 0, tilt: 0, ...o });

test('widths interpolate between the measuring lines', () => {
  assert.deepEqual(widths(0, P), { left: 290, right: 315 });
  assert.deepEqual(widths(P.lineGap, P), { left: 190, right: 235 });
  assert.deepEqual(widths(P.lineGap * 2, P), { left: 190, right: 235 });
});

test('box inside the slab is green', () => {
  assert.equal(check(box(), P).status, 'green');
});

test('box sticking above the slab is yellow with sight height', () => {
  const r = check(box({ h: 60 }), P);
  assert.equal(r.status, 'yellow');
  assert.equal(r.sight, 60 - MEASURED.slabH);
});

test('left side ends earlier at the servo motor', () => {
  assert.equal(check(box({ u: 130 }), P).status, 'red');
  assert.equal(check(box({ u: 130, y: -100 }), P).status, 'green');
});

test('box in the centre strip above the slab collides with the column', () => {
  const r = check(box({ y: 0, z: 50 }), P);
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
