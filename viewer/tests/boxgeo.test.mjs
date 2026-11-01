import test from 'node:test';
import assert from 'node:assert/strict';
import * as g from '../boxgeo.js';
import { GATE_TRAVEL, A_STROKE, ROW_TOP } from '../sim.js';

const near = (a, b) => assert.ok(Math.abs(a - b) < 1e-6, `${a} != ${b}`);

test('cable eyes stop EYE_CLEAR short of the rigid cable ends at full extension', () => {
  near(g.WALL_X - g.RIGID.G - (g.G_ROD_END0 + GATE_TRAVEL + g.CLEVIS), g.EYE_CLEAR);
  near(g.WALL_X - g.RIGID.R - (g.B_ROD_END0 + A_STROKE + Math.max(...ROW_TOP) + g.CLEVIS), g.EYE_CLEAR);
});

test('row chain: A rod -> FK-32 -> drive bracket -> gap -> B', () => {
  near(g.MITN_BACK0 - g.A_ROD_END0, g.FK32);
  near(g.B_REAR0 - g.MITN_FRONT0, g.MITN_GAP);
  near(g.A_ROD_END0 + g.B_OFFSET, g.B_REAR0);
});

test('rail carries the sled over the whole A stroke', () => {
  assert.ok(g.RAIL_X0 <= g.SLED.x0);
  assert.ok(g.RAIL_X0 + g.RAIL.len >= g.SLED.x1 + A_STROKE);
});

test('everything fits inside the box', () => {
  assert.ok(g.A_X0 > g.CB.x0 + g.WALL_T);
  assert.ok(g.A_BRACKET_X - g.BRACKET.foot > g.CB.x0 + g.WALL_T);
  assert.ok(g.RAIL_X0 + g.RAIL.len < g.WALL_X - g.RIGID.R);
  assert.ok(g.AX_Y + g.HM25.D / 2 < g.CB.h);
});
