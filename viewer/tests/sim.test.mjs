import test from 'node:test';
import assert from 'node:assert/strict';
import { createSim, SEQUENCE } from '../sim.js';

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
  assert.deepEqual(states.slice(1), ['TO_NEUTRAL', 'GATE_MOVE', 'ROW_ENGAGE', 'DONE', 'IDLE']);
  const pt = sim.poti();
  assert.ok(pt >= 45 && pt <= 55, `poti ${pt}`);
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

test('plus/minus follow R N 1..5', () => {
  const sim = createSim();
  const seen = [];
  for (let i = 0; i < 6; i++) {
    sim.plus();
    runUntilIdle(sim);
    seen.push(sim.state.gear);
  }
  assert.deepEqual(seen, ['1', '2', '3', '4', '5', '5']);
  for (let i = 0; i < 6; i++) {
    sim.minus();
    runUntilIdle(sim);
  }
  assert.equal(sim.state.gear, 'R');
  assert.deepEqual(SEQUENCE, ['R', 'N', '1', '2', '3', '4', '5']);
});
