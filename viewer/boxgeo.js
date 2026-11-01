// Cylinder box geometry (mm), shared by the 3D viewer (scene.js) and the build plan (zylinderbox.html).
// x along the cylinders (cable side = +x), y up from the underside of the base plate, z across the box.
import { GATE_TRAVEL, A_STROKE, A_CYL_STROKE, A_STOP, B_STROKE, ROW_TOP } from './sim.js';

// Airtec HM data sheet (Series HM, p. 9-37): X = rear pin -> head face, WF = head face -> rod end,
// nose thread M22 x 1.5 on both Ø20 and Ø25
export const HM20 = { X: 95, WF: 24, D: 27, rod: 8 };
export const HM25 = { X: 104, WF: 28, D: 30, rod: 10 };
export const G_BODY = HM20.X + GATE_TRAVEL;
export const A_BODY = HM25.X + A_CYL_STROKE;
export const B_BODY = HM20.X + B_STROKE;

export const WALL_X = 262;       // inner face of the end wall with the clip holes
export const WALL_T = 2;
export const FLOOR_T = 8;
export const FLOOR_EXT = 40;     // base plate runs on past the end wall under the gussets
// user notes: rigid cable end inside the box 70 mm (Wählseil) / 130 mm (Schaltseil), clip hole Ø 25 / Ø 15 mm
export const RIGID = { G: 70, R: 130 };
export const HOLE_D = { G: 25, R: 15 };
export const EYE_CLEAR = 10;     // assumed gap eye -> rigid part at full extension
export const CLEVIS = 22;        // rod end -> cable eye pin (RD-20 + Ø8 pin)
export const FK32 = 71.5;        // Airtec FK-32 compensating coupling A (M10x1.25) -> sled
export const AX_H = 40;          // common axis height above the base plate
export const AX_Y = FLOOR_T + AX_H;

// self-made brackets (user 2026-11-01): angle with Ø22.5 hole, cylinder clamped by its nose thread + RM-25
export const BRACKET = { w: 50, h: 60, t: 6, foot: 60, hole: 22.5, slot: 20, nutGap: 8 }; // angle 60x60x6
export const SLED_BRACKET = { w: 50, h: 40, t: 5, foot: 40 };                            // angle 40x40x5
export const RAIL = { len: 200, h: 13, w: 12, pitch: 25 };   // MGN12 + MGN12H, H = rail foot -> carriage top
export const CARRIAGE = { len: 45.4, w: 27, holes: 20 };     // MGN12H, 4 x M3 on 20 x 20
export const SLED_T = 6;
export const SLED_W = 50;
export const MITN_GAP = 10;      // drive bracket -> B rear end, room for the FK-32 stud end

export const LANE_W = 54;
const GAP = 6;
const CB_Z1 = 50;
export const ROW_Z = CB_Z1 - WALL_T - GAP - LANE_W / 2;
export const G_Z = ROW_Z - LANE_W - GAP;
export const POTI_Z = G_Z - LANE_W / 2 - GAP - 8;

// G: rod end at x = 0 (fully retracted)
export const G_ROD_END0 = WALL_X - RIGID.G - EYE_CLEAR - CLEVIS - GATE_TRAVEL;
export const G_HEAD = G_ROD_END0 - HM20.WF;
export const G_X0 = G_HEAD - G_BODY;
// row: B rod end at a = b = 0, B sits on the sled, A pushes the sled through FK-32 + drive bracket
export const ROW_TRAVEL = A_STROKE + Math.max(...ROW_TOP);
export const B_ROD_END0 = WALL_X - RIGID.R - EYE_CLEAR - CLEVIS - ROW_TRAVEL;
export const B_HEAD0 = B_ROD_END0 - HM20.WF;
export const B_REAR0 = B_HEAD0 - B_BODY;
export const MITN_FRONT0 = B_REAR0 - MITN_GAP;
export const MITN_BACK0 = MITN_FRONT0 - SLED_BRACKET.t;
export const B_OFFSET = FK32 + SLED_BRACKET.t + MITN_GAP;     // A rod end -> B rear end
export const A_ROD_END0 = B_REAR0 - B_OFFSET;
export const A_HEAD = A_ROD_END0 - HM25.WF - A_STOP;          // stop sleeve keeps the rod 12 mm out
export const A_X0 = A_HEAD - A_BODY;

// outer length rounded for fabrication (>= 5 mm air behind A, checked in the tests)
export const BOX_L = 660;
export const CB = { x0: WALL_X + WALL_T - BOX_L, x1: WALL_X + WALL_T, z0: POTI_Z - 8 - GAP - WALL_T, z1: CB_Z1, h: Math.ceil(AX_Y + HM25.D / 2 + 20) };
export const PLATE = { x0: CB.x0, x1: CB.x1 + FLOOR_EXT, z0: CB.z0, z1: CB.z1 };
export const LID_T = 1.5;
export const WALL_H = CB.h - FLOOR_T - LID_T - 0.5;   // end and rear wall, 0.5 mm below the cover
export const GUSSET = { t: 4, base: FLOOR_EXT, h: 55, z: [26, 87, 146].map((q) => CB.z0 + q) };

// brackets: front face sits nutGap behind the head face (nut in front), foot points to the rear
export const G_BRACKET_X = G_HEAD - BRACKET.nutGap;
export const A_BRACKET_X = A_HEAD - BRACKET.nutGap;
// sled at a = 0: from the drive bracket back face to the B bracket front face
export const SLED_TOP = FLOOR_T + RAIL.h + SLED_T;
export const SLED = { x0: MITN_BACK0, x1: B_HEAD0 - BRACKET.nutGap };
export const RAIL_X0 = PLATE.x0 + Math.round(SLED.x0 - (RAIL.len - (SLED.x1 - SLED.x0) - A_STROKE) / 2 - PLATE.x0);

// distance from the inner face of the end wall (how the plan is dimensioned)
export const fromWall = (x) => Math.round((WALL_X - x) * 10) / 10;
// distance from the poti-side outer edge
export const fromSide = (z) => Math.round((z - CB.z0) * 10) / 10;
