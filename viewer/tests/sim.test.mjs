import test from 'node:test';
import assert from 'node:assert/strict';
import { createSim, SEQUENCE, SHIFT_ORDER, GEARS } from '../sim.js';

const DT = 0.0005;

function runUntilIdle(sim, maxT = 3) {
  const t0 = sim.state.t;
  do sim.step(DT);
  while (sim.state.state !== 'IDLE' && sim.state.t - t0 < maxT);
  return sim.state;
}

function shiftTo(sim, g) {
  assert.ok(sim.request(g), `request ${g} rejected in ${sim.state.state}`);
  return runUntilIdle(sim);
}

test('all gears reachable with default parameters', () => {
  const sim = createSim();
  for (const g of ['1', '2', '3', '4', '5', 'R', 'N', '3', '2', '5', '1']) {
    const s = shiftTo(sim, g);
    assert.equal(s.gear, g, `wanted ${g}, log: ${s.log.slice(-6).map((e) => e.msg).join(' | ')}`);
    assert.equal(s.message, '');
  }
});

test('2 -> 3 goes through neutral, gate move and flying engage', () => {
  const sim = createSim();
  shiftTo(sim, '2');
  sim.state.log.length = 0;
  shiftTo(sim, '3');
  const states = sim.state.log.map((e) => e.msg.split(' ')[0]);
  assert.deepEqual(states.slice(1), ['CLUTCH_OPEN', 'TO_NEUTRAL', 'GATE_MOVE', 'ROW_ENGAGE', 'DONE', 'CLUTCH_CLOSE', 'IDLE']);
  const pt = sim.poti();
  const w = sim.win(2);
  assert.ok(pt >= w.lo && pt <= w.hi, `poti ${pt}`);
});

test('3 -> 4 stays in gate, G never pressurized', () => {
  const sim = createSim();
  shiftTo(sim, '3');
  sim.request('4');
  let gPressed = false;
  do {
    sim.step(DT);
    if (sim.state.valve.G !== 'vent') gPressed = true;
  } while (sim.state.state !== 'IDLE');
  assert.equal(sim.state.gear, '4');
  assert.equal(gPressed, false);
});

test('too fast G overshoots 4 times, then faults to neutral', () => {
  const sim = createSim({ gSpeed: 700, gDecel: 1500 });
  shiftTo(sim, '2');
  sim.request('3');
  runUntilIdle(sim);
  assert.equal(sim.state.gear, 'N');
  assert.match(sim.state.message, /Fehlversuche|Timeout/);
  assert.equal(sim.rowOf(), 'neutral');
});

test('plus/minus follow R 1..5, N only via neutral()', () => {
  const sim = createSim();
  const seen = [];
  for (let i = 0; i < 6; i++) {
    sim.plus();
    runUntilIdle(sim);
    seen.push(sim.state.gear);
  }
  assert.deepEqual(seen, ['1', '2', '3', '4', '5', '5']);
  const down = [];
  for (let i = 0; i < 6; i++) {
    sim.minus();
    runUntilIdle(sim);
    down.push(sim.state.gear);
  }
  assert.deepEqual(down, ['4', '3', '2', '1', 'R', 'R']);
  sim.plus();
  runUntilIdle(sim);
  assert.equal(sim.state.gear, '1');
  sim.neutral();
  runUntilIdle(sim);
  assert.equal(sim.state.gear, 'N');
  sim.minus();
  runUntilIdle(sim);
  assert.equal(sim.state.gear, 'R');
  assert.deepEqual(SHIFT_ORDER, ['R', '1', '2', '3', '4', '5']);
});

test('layout R 1 3 5 top, 2 4 bottom, R and 5 at G end positions', () => {
  assert.deepEqual(Object.entries(GEARS).filter(([, v]) => v.row === 'top').map(([k]) => k).sort(), ['1', '3', '5', 'R']);
  const sim = createSim();
  shiftTo(sim, 'R');
  assert.ok(sim.poti() < 5);
  shiftTo(sim, '1');
  shiftTo(sim, '5');
  assert.ok(sim.poti() > 95);
  assert.equal(sim.rowOf(), 'top');
});

test('1/2 gate is also engaged on the fly from both sides', () => {
  const sim = createSim();
  shiftTo(sim, '5');
  shiftTo(sim, '2');
  assert.equal(sim.state.gear, '2');
  shiftTo(sim, 'N');
  shiftTo(sim, 'R');
  shiftTo(sim, 'N');
  shiftTo(sim, '1');
  assert.equal(sim.state.gear, '1');
  const w = sim.win(1);
  assert.ok(sim.poti() >= w.lo && sim.poti() <= w.hi);
});

test('auto clutch: pedal open during the whole shift, closed afterwards', () => {
  const sim = createSim();
  sim.request('3');
  let closedWhileShifting = false;
  do {
    sim.step(DT);
    if (['TO_NEUTRAL', 'GATE_MOVE', 'ROW_ENGAGE', 'CORRECT'].includes(sim.state.state) && !sim.clutch().open) closedWhileShifting = true;
  } while (sim.state.state !== 'IDLE');
  assert.equal(sim.state.gear, '3');
  assert.equal(closedWhileShifting, false);
  assert.ok(sim.clutch().closed);
});

test('manual clutch: command waits for the foot, then shifts', () => {
  const sim = createSim({ autoClutch: false });
  sim.request('3');
  for (let i = 0; i < 1000; i++) sim.step(DT); // 0.5 s without pedal
  assert.equal(sim.state.state, 'CLUTCH_OPEN');
  sim.setFoot(true);
  runUntilIdle(sim);
  assert.equal(sim.state.gear, '3');
});

test('manual clutch: no pedal -> command dropped, gear unchanged, no fault', () => {
  const sim = createSim({ autoClutch: false });
  sim.request('3');
  runUntilIdle(sim, 5);
  assert.equal(sim.state.gear, 'N');
  assert.match(sim.state.message, /nicht getreten/);
});

test('manual clutch released mid-shift -> fault to neutral', () => {
  const sim = createSim({ autoClutch: false });
  sim.setFoot(true);
  shiftTo(sim, '2');
  sim.request('5');
  while (sim.state.state !== 'GATE_MOVE') sim.step(DT);
  sim.setFoot(false);
  runUntilIdle(sim);
  assert.equal(sim.state.gear, 'N');
  assert.match(sim.state.message, /Kupplung/);
});
