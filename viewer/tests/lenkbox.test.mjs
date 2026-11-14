import { test } from 'node:test';
import assert from 'node:assert/strict';
import { design, bedFit, sideGeom, shellDims, MOTOR_ROW, LEFT_BUTTONS, BEDS } from '../lenkbox.js';
import { check, widths, DEFAULT_PARAMS, MEASURED } from '../lenkraum.js';

const P = { ...DEFAULT_PARAMS, faceH: 45 };

test('parts inside the boxes fit the free space without collisions', () => {
  const inner = design(P).parts.filter((x) => x.check).map((x, i) => ({ ...x, id: i + 1 }));
  for (const b of inner) {
    const r = check(b, P, inner);
    assert.equal(r.status, 'green', `${b.name}: ${r.errors}`);
  }
});

test('shell outline stays inside the measured side space', () => {
  for (const side of ['left', 'right']) {
    const len = side === 'left' ? MEASURED.lenLeft : MEASURED.lenRight;
    for (const [u, y] of sideGeom(side, P).poly) {
      assert.ok(u >= 0 && u <= len, `${side} u=${u}`);
      assert.ok(Math.abs(y) <= MEASURED.steelW / 2 + widths(u, P)[side], `${side} y=${y}`);
      assert.ok(Math.abs(y) >= P.colDia / 2, `${side} hits the column`);
    }
  }
});

test('round displays left, motor row right in the decided order', () => {
  const { parts } = design(P);
  const rd = parts.filter((x) => /Rund-Display \d \(Platine\)/.test(x.name));
  assert.equal(rd.length, 3);
  assert.ok(rd.every((x) => x.y > 0));
  const sw = parts.filter((x) => x.label);
  assert.deepEqual(sw.map((x) => x.label), MOTOR_ROW.map((e) => e.label));
  assert.ok(sw.every((x) => x.y < 0));
  for (let i = 1; i < sw.length; i++) assert.ok(sw[i].y < sw[i - 1].y, 'left to right in driver view');
});

test('covers stay inside the front face', () => {
  for (const c of design(P).parts.filter((x) => /Kappe/.test(x.name))) {
    assert.ok(c.z >= 0 && c.z + c.h <= P.faceH, c.name);
  }
});

test('bed fit finds rotations and rejects parts that are too big', () => {
  assert.deepEqual(bedFit([200, 100, 40], [256, 256, 256]), { pose: 'liegend', deg: 0 });
  assert.equal(bedFit([270, 135, 45], [256, 256, 256]).pose, 'auf der Frontseite stehend');
  assert.equal(bedFit([400, 300, 300], [256, 256, 256]), null);
});

test('both shells fit the A1 in one piece', () => {
  for (const side of ['left', 'right']) assert.ok(bedFit(shellDims(side, P), BEDS['Bambu Lab A1']), side);
});

test('two buttons stacked on the left front, inside the face and clear of the displays', () => {
  const { parts } = design(P);
  const btn = parts.filter((x) => x.button);
  assert.deepEqual(btn.map((x) => x.button), LEFT_BUTTONS.map((b) => b.label));
  assert.equal(btn[0].y, btn[1].y);
  for (const b of btn) assert.ok(b.y > 0 && b.z - b.r >= 0 && b.z + b.r <= P.faceH, b.name);
  const rd = parts.filter((x) => /Rund-Display \d \(Platine\)/.test(x.name));
  const minRd = Math.min(...rd.map((x) => x.y - x.w / 2));
  assert.ok(btn[0].y + btn[0].r < minRd, 'button overlaps a display');
});
