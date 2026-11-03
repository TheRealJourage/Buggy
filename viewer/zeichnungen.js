// Fabrication drawing set for the cylinder box (7 x A3), built from boxgeo.js.
// Plate coordinates: X from the rear edge of the base plate, Y from the poti-side edge, Z from its underside.
import * as g from './boxgeo.js';
import { sheet, view, poly, rect, line, circle, hole, tapped, slot, dim, leader, weld, balloon, datum, gtol, label, table, cross, el, fmt } from './drafting.js';

const root = document.getElementById('sheets');
const N = 7;
const BASE_NOTES = [
  'Maße in mm · Allgemeintoleranzen ISO 2768-mK',
  'Werkstückkanten ISO 13715: −0,3 (alle Kanten entgratet)',
  'Oberfläche roh, Sichtflächen ohne Kratzer',
  '!Orange mit * = Prüfmaß: vor Fertigung vom Auftraggeber bestätigen',
];
const sh = (k, info) => sheet(root, { sheet: `${k} / ${N}`, ...info, notes: [...(info.notes || []), ...BASE_NOTES] });

// ---------- fabrication geometry (rounded where a slot or the general tolerance allows it)
const r1 = (v) => Math.round(v * 10) / 10;
const PX = (x) => r1(x - g.PLATE.x0), PY = (z) => r1(z - g.PLATE.z0);
const L = PX(g.PLATE.x1), B = g.PLATE.z1 - g.PLATE.z0, T = g.FLOOR_T;
const XW = PX(g.WALL_X), WT = g.WALL_T, WH = g.WALL_H, AXH = g.AX_H, AXZ = T + AXH;
const QG = PY(g.G_Z), QR = PY(g.ROW_Z), QP = PY(g.POTI_Z);
const GQ = g.GUSSET.z.map(PY), GT = g.GUSSET.t, GB = g.GUSSET.base, GH = g.GUSSET.h, GCH = 5;
const RIV = { q: [40, 112], h: 66, d: 6 };
const BR = g.BRACKET, SB = g.SLED_BRACKET;
const SLOT_C = BR.t + (BR.foot - BR.t) / 2, SLOT_Q = 17;
const SB_HOLE = SB.t + (SB.foot - SB.t) / 2, SB_Q = 15;
const XG_SLOT = Math.round(PX(g.G_BRACKET_X) - SLOT_C), XA_SLOT = Math.round(PX(g.A_BRACKET_X) - SLOT_C);
const RAIL_X = PX(g.RAIL_X0), RAIL_E = 12.5;
const SLED_L = r1(g.SLED.x1 - g.SLED.x0), SLED_W = g.SLED_W, SLED_T = g.SLED_T, SLED_AX = r1(g.AX_Y - g.SLED_TOP);
const CAR_C = 23; // carriage centre from the sled ends
const LID = { t: g.LID_T, gap: 0.5, lip: 18 };
const LID_W = B + 2 * (LID.gap + LID.t), LID_L = XW + WT + 2 * (LID.gap + LID.t), LID_H = g.CB.h;
const REAR = { t: 1.5, flange: 15, grommet: { q: 57, h: 35, d: 30 }, gland: { q: 20, h: 35, d: 16.5 }, flangeHole: 9, flangeQ: [30, 122] };
const LASCHE = { l: 57, w: 16, t: 3, a: 8, pitch: r1(g.G_Z - g.POTI_Z) };

const plateHoles = [
  ['L1', XA_SLOT, QR - SLOT_Q, 'M6', 'durchgehend', 'Lagerbock A (Pos 4)'],
  ['L2', XA_SLOT, QR + SLOT_Q, 'M6', 'durchgehend', 'Lagerbock A (Pos 4)'],
  ['L3', XG_SLOT, QG - SLOT_Q, 'M6', 'durchgehend', 'Lagerbock G (Pos 4)'],
  ['L4', XG_SLOT, QG + SLOT_Q, 'M6', 'durchgehend', 'Lagerbock G (Pos 4)'],
  ...Array.from({ length: g.RAIL.len / g.RAIL.pitch }, (_, i) => [`S${i + 1}`, RAIL_X + RAIL_E + i * g.RAIL.pitch, QR, 'M3', 'durchgehend', 'Schiene MGN12*']),
  ['B1', 20, 20, 'M6', 'durchgehend', 'Befestigung Fahrzeug'],
  ['B2', 20, B - 20, 'M6', 'durchgehend', 'Befestigung Fahrzeug'],
  ['B3', L - 20, QG, 'M6', 'durchgehend', 'Befestigung Fahrzeug'],
  ['B4', L - 20, QR, 'M6', 'durchgehend', 'Befestigung Fahrzeug'],
  ['R1', REAR.flangeHole, REAR.flangeQ[0], 'M4', 'durchgehend', 'Rückwand (Pos 11)'],
  ['R2', REAR.flangeHole, REAR.flangeQ[1], 'M4', 'durchgehend', 'Rückwand (Pos 11)'],
];

// guards: rounded fabrication values must stay close to the design geometry
console.assert(L === 700 && XW === 658 && WH === 73, 'plate/wall', L, XW, WH);
console.assert(Math.abs(PX(g.A_BRACKET_X) - SLOT_C - XA_SLOT) < 1, 'A slot');
console.assert(GQ.every((q, i) => i === 0 || q - GQ[i - 1] > 40), 'gussets');

// ---------- shared drawing helpers
function bracketSide(v, xf, base, b, cls = 'k', dir = -1) { // L profile, upright at xf, foot towards dir
  const t = b.t * -dir, f = b.foot * dir;
  poly(v, [[xf, base], [xf, base + b.h], [xf + dir * b.t, base + b.h], [xf + dir * b.t, base + b.t], [xf + f, base + b.t], [xf + f, base]], cls);
  return t;
}
function bracketTop(v, xf, q, b, dir = -1, slots = false) {
  rect(v, xf, q - b.w / 2, xf + dir * b.t, q + b.w / 2);
  rect(v, xf + dir * b.t, q - b.w / 2, xf + dir * b.foot, q + b.w / 2);
  if (slots) for (const dq of [-SLOT_Q, SLOT_Q]) slot(v, xf + dir * SLOT_C, q + dq, BR.slot, 6.6, 'u');
}
const phantomCyl = (v, x0, x1, z, r) => rect(v, x0, z - r, x1, z + r, 'p');

// positions of the loose parts in the assembly (plate coordinates)
const A_ = {
  gBr: PX(g.G_BRACKET_X), aBr: PX(g.A_BRACKET_X), sled0: PX(g.SLED.x0), sled1: PX(g.SLED.x1),
  gX0: PX(g.G_X0), gHead: PX(g.G_HEAD), gRod: PX(g.G_ROD_END0),
  aX0: PX(g.A_X0), aHead: PX(g.A_HEAD), aRod: PX(g.A_ROD_END0),
  bRear: PX(g.B_REAR0), bHead: PX(g.B_HEAD0), bRod: PX(g.B_ROD_END0),
};
const SLED_Z = T + g.RAIL.h;

// ======================= 1: assembly
{
  const s = sh(1, {
    no: 'BGY-ZB-00', title: 'Zylinderbox – Zusammenbau', scale: '1:2', material: 'siehe Stückliste', stock: '1 Satz',
    notes: [
      'Pos 1–3 als Schweißbaugruppe, siehe BGY-ZB-01',
      'Pos 4–12 lose liefern, Montage durch Auftraggeber',
      'Strich-Zweipunkt: Kaufteile des Auftraggebers (Zylinder, Führung,',
      '  Poti, FK-32, Gabelköpfe, Seile), nicht im Lieferumfang',
    ],
  });
  const v = view(s, 45, 78, 0.5, [45, 26, 'Vorderansicht (von der Poti-Seite, Haube Pos 10 abgenommen)']);
  rect(v, 0, 0, L, T);
  rect(v, XW, T, XW + WT, T + WH);
  poly(v, [[XW + WT + GCH, T], [L, T], [XW + WT, T + GH], [XW + WT, T + GCH]]);
  poly(v, [[0, T], [0, T + WH], [REAR.t, T + WH], [REAR.t, T + REAR.t], [REAR.flange, T + REAR.t], [REAR.flange, T]]);
  rect(v, -LID.t - LID.gap, 0, XW + WT + LID.gap + LID.t, LID_H, 'p');
  bracketSide(v, A_.gBr, T, BR); bracketSide(v, A_.aBr, T, BR);
  rect(v, RAIL_X, T, RAIL_X + g.RAIL.len, T + 8, 'p');
  for (const c of [A_.sled0 + CAR_C, A_.sled1 - CAR_C]) rect(v, c - 22.7, T + 3, c + 22.7, SLED_Z, 'p');
  rect(v, A_.sled0, SLED_Z, A_.sled1, SLED_Z + SLED_T);
  bracketSide(v, A_.sled1, SLED_Z + SLED_T, SB);
  bracketSide(v, A_.sled0, SLED_Z + SLED_T, SB, 'k', 1);
  phantomCyl(v, A_.gX0, A_.gHead, AXZ, 13.5); phantomCyl(v, A_.aX0, A_.aHead, AXZ, 15); phantomCyl(v, A_.bRear, A_.bHead, AXZ, 13.5);
  rect(v, A_.aHead, AXZ - 8, A_.aRod, AXZ + 8);
  phantomCyl(v, A_.aRod, A_.sled0, AXZ, 11);
  for (const [k, n] of [['G', 70], ['R', 130]]) phantomCyl(v, XW - n, XW + 60, AXZ + (k === 'G' ? 0 : 0.01), 6);
  line(v, -10, AXZ, L + 10, AXZ, 'c');
  dim(v, [0, 0], [L, 0], 'h', -10);
  dim(v, [L, T], [L, AXZ], 'v', 8, { text: `${AXH}` });
  dim(v, [L, 0], [L, LID_H], 'v', 16);
  for (const [p, dx, dy, n] of [[[150, 4], 0, 14, '1'], [[XW + 1, 70], 10, -10, '2'], [[XW + 15, 35], 18, -8, '3'], [[A_.gBr - 3, 55], -6, -18, '4'], [[A_.sled1 - 2, SLED_Z + 30], 4, -22, '5'], [[A_.sled0 + 2, SLED_Z + 30], -6, -24, '6'], [[A_.sled0 + 60, SLED_Z + 3], -26, 20, '7'], [[A_.aHead + 20, AXZ + 8], 0, -20, '8'], [[200, LID_H], 0, -6, '10'], [[1, 60], -10, -14, '11']]) balloon(v, p, dx, dy, n);

  const t = view(s, 45, 188, 0.5, [45, 101, 'Draufsicht']);
  rect(t, 0, 0, L, B);
  rect(t, XW, 0, XW + WT, B);
  for (const q of GQ) rect(t, XW + WT, q - GT / 2, L, q + GT / 2);
  rect(t, 0, 0, REAR.t, B);
  bracketTop(t, A_.gBr, QG, BR, -1, true); bracketTop(t, A_.aBr, QR, BR, -1, true);
  rect(t, A_.sled0, QR - SLED_W / 2, A_.sled1, QR + SLED_W / 2);
  bracketTop(t, A_.sled1, QR, SB); bracketTop(t, A_.sled0, QR, SB, 1);
  phantomCyl(t, A_.gX0, A_.gHead, QG, 13.5); phantomCyl(t, A_.gHead, A_.gRod, QG, 4); phantomCyl(t, A_.gRod, A_.gRod + 22, QG, 8);
  phantomCyl(t, A_.aX0, A_.aHead, QR, 15); rect(t, A_.aHead, QR - 8, A_.aRod, QR + 8); phantomCyl(t, A_.aRod, A_.sled0, QR, 11);
  phantomCyl(t, A_.bRear, A_.bHead, QR, 13.5); phantomCyl(t, A_.bHead, A_.bRod, QR, 4); phantomCyl(t, A_.bRod, A_.bRod + 22, QR, 8);
  phantomCyl(t, A_.gRod - 120, A_.gRod + 65, QP, 6.5);
  rect(t, A_.gRod - LASCHE.t, QP - LASCHE.a, A_.gRod, QG + LASCHE.a);
  for (const q of [QG, QR]) { line(t, -10, q, L + 10, q, 'c'); phantomCyl(t, XW - (q === QG ? 70 : 130), XW, q, q === QG ? 10.5 : 5.5); }
  dim(t, [0, B], [XW, B], 'h', 6);
  dim(t, [L, 0], [L, B], 'v', 10);
  for (const [p, dx, dy, n] of [[[A_.aHead + 20, QR], 0, -18, '8'], [[A_.gRod - 1.5, QP + 4], 22, 4, '9'], [[2, B / 2], -10, 0, '11']]) balloon(t, p, dx, dy, n);
  label(s, 45 + (A_.gRod - 60) * 0.5, 188 - (QP - 9) * 0.5, 'Poti', { size: 2.5 });

  table(s, 22, 203, [['Pos', 9], ['Menge', 11], ['Benennung', 38], ['Werkstoff', 30], ['Halbzeug / Abmessung', 58], ['Zeichnung', 22], ['Bemerkung', 34]], [
    ['1', '1', 'Grundplatte', 'EN AW-5083', `Blech ${T} × ${B} × ${L}`, 'ZB-02', 'Schweißgruppe ZB-01'],
    ['2', '1', 'Stirnwand', 'EN AW-5754', `Blech ${WT} × ${B} × ${WH}`, 'ZB-03', 'Schweißgruppe ZB-01'],
    ['3', '3', 'Knotenblech', 'EN AW-5754', `Blech ${GT}, Dreieck ${GB} × ${GH}`, 'ZB-03', 'Schweißgruppe ZB-01'],
    ['4', '2', 'Lagerbock G / A', 'EN AW-6060 T66', `Winkel ${BR.h}×${BR.foot}×${BR.t}, L ${BR.w}`, 'ZB-04', ''],
    ['5', '1', 'Lagerbock B', 'EN AW-6060 T66', `Winkel ${SB.h}×${SB.foot}×${SB.t}, L ${SB.w}`, 'ZB-04', ''],
    ['6', '1', 'Mitnehmer', 'EN AW-6060 T66', `Winkel ${SB.h}×${SB.foot}×${SB.t}, L ${SB.w}`, 'ZB-04', ''],
    ['7', '1', 'Schlitten', 'EN AW-5083', `Blech ${SLED_T} × ${SLED_W} × ${SLED_L}`, 'ZB-05', ''],
    ['8', '1', 'Anschlaghülse', 'EN AW-6082 T6', 'Rund Ø16 × 40', 'ZB-05', ''],
    ['9', '1', 'Poti-Lasche', 'EN AW-5754', `Blech ${LASCHE.t} × ${LASCHE.w} × ${LASCHE.l}`, 'ZB-05', ''],
    ['10', '1', 'Haube', 'EN AW-5754', `Blech ${fmt(LID.t)}, gekantet`, 'ZB-06', ''],
    ['11', '1', 'Rückwand', 'EN AW-5754', `Blech ${fmt(REAR.t)}, gekantet`, 'ZB-03', ''],
    ['12', '4', 'Blindnietmutter M4', 'Alu oder Stahl verz.', 'Flachkopf, Klemmbereich 1,5–3', '–', 'in Pos 2 und 11'],
  ], { rh: 5.6, size: 2.7 });
}

// ======================= 2: weldment
{
  const s = sh(2, {
    no: 'BGY-ZB-01', title: 'Grundgerüst – Schweißbaugruppe', scale: '1:2', material: 'EN AW-5083 / EN AW-5754', stock: '1 Stück (Pos 1, 2, 3 × 3)',
    notes: [
      'Schweißverfahren 141 (WIG), Zusatz S Al 5356 (Vorschlag)',
      'Verzug gering halten, Grundplatte nach dem Schweißen eben (0,5)',
      'Bohrungen in Pos 2 nach dem Schweißen; Gewinde Pos 1 siehe ZB-02',
      'Spitzen der Knotenbleche 2 × 45° brechen',
    ],
  });
  const v = view(s, 45, 62, 0.5, [45, 18, 'Vorderansicht']);
  rect(v, 0, 0, L, T);
  rect(v, XW, T, XW + WT, T + WH);
  poly(v, [[XW + WT + GCH, T], [L, T], [XW + WT, T + GH], [XW + WT, T + GCH]]);
  dim(v, [0, 0], [L, 0], 'h', -10);
  dim(v, [XW + WT, T], [L, T], 'h', -18);
  dim(v, [XW, T + WH], [XW + WT, T + WH], 'h', 8);
  dim(v, [0, 0], [0, T], 'v', -6);
  dim(v, [XW, T], [XW, T + WH], 'v', -12);
  dim(v, [L, T], [L, T + GH], 'v', 8);
  datum(v, [300, T], 0, -10, 'A');
  gtol(v, [XW, T + 60], -150, -12, ['⊥', '0,3', 'A']);
  weld(v, [XW, T], -50, -8, 'a1,5', { left: true });
  weld(v, [XW + WT, T + 45], -45, -6, 'a1,5', { left: true });
  weld(v, [XW + WT + 22, T], -30, 12, 'a2,5', { left: true });
  // view arrow A
  const ax = v.X(L) + 9, ay = v.Y(AXZ);
  el(s, 'line', { x1: ax + 6, y1: ay, x2: ax, y2: ay, class: 'k' });
  el(s, 'path', { d: `M${ax} ${ay} l3 -1.2 l0 2.4 Z`, class: 'fill' });
  label(s, ax + 3, ay - 2.5, 'A', { size: 4, bold: true });

  const t = view(s, 45, 160, 0.5, [45, 81, 'Draufsicht']);
  rect(t, 0, 0, L, B);
  rect(t, XW, 0, XW + WT, B);
  for (const q of GQ) { rect(t, XW + WT, q - GT / 2, L, q + GT / 2); line(t, XW - 6, q, L + 4, q, 'c'); }
  let prev = 0;
  [...GQ, B].forEach((q, i) => { dim(t, [L, prev], [L, q], 'v', 8, { text: fmt(q - prev) }); prev = q; });
  dim(t, [0, 0], [0, B], 'v', -8);
  dim(t, [0, 0], [XW, 0], 'h', -7, { tol: '±0,5' });
  datum(t, [685, 0], 0, 9, 'B');

  // view A at 1:1 (poti side on the right)
  const a = view(s, 52, 283, 1, [215, 205, 'Ansicht A (1:1)']);
  const U = (q) => B - q;
  rect(a, 0, 0, B, T);
  rect(a, 0, T, B, T + WH);
  for (const q of GQ) rect(a, U(q) - GT / 2, T, U(q) + GT / 2, T + GH);
  for (const [q, d] of [[QG, g.HOLE_D.G], [QR, g.HOLE_D.R]]) hole(a, U(q), AXZ, d);
  for (const q of RIV.q) hole(a, U(q), T + RIV.h, RIV.d);
  dim(a, [B, AXZ], [U(QG), AXZ], 'h', -(g.HOLE_D.G / 2 + 6), { tol: '±0,2' });
  dim(a, [B, T + WH], [U(QR), T + WH], 'h', 6, { tol: '±0,2' });
  dim(a, [B, T + WH], [U(RIV.q[0]), T + WH], 'h', 13);
  dim(a, [B, T + WH], [U(RIV.q[1]), T + WH], 'h', 20);
  dim(a, [0, T], [0, AXZ], 'v', -7, { tol: '±0,2' });
  dim(a, [0, T], [0, T + RIV.h], 'v', -16);
  dim(a, [B, 0], [B, T + WH], 'v', 8, { text: `${T + WH}` });
  leader(a, [U(QG) + g.HOLE_D.G / 2 * 0.7, AXZ + g.HOLE_D.G / 2 * 0.7], 12, -10, `Ø${g.HOLE_D.G} +0,2/0*`, { chk: true });
  leader(a, [U(QR) - g.HOLE_D.R / 2 * 0.7, AXZ + g.HOLE_D.R / 2 * 0.7], -8, -12, `Ø${g.HOLE_D.R} +0,2/0*`, { chk: true });
  leader(a, [U(RIV.q[1]) - 2, T + RIV.h + 2], -6, -10, ['2× Ø6 für Blindnietmutter M4', '(Ø nach Hersteller)']);
  datum(a, [B, T / 2], 8, 0, 'B');
  datum(a, [40, T], 0, 7, 'A');
  label(s, 52 + B / 2, 289.5, 'Poti-Seite →', { size: 2.6, anchor: 'middle' });
}

// ======================= 3: base plate
{
  const s = sh(3, {
    no: 'BGY-ZB-02', title: 'Grundplatte (Pos 1)', scale: '1:2', material: 'EN AW-5083 H111', stock: `Blech ${T} × ${B} × ${L}, 1 Stück`,
    notes: [
      'Koordinaten X/Y ab Nullpunkt (Ecke Rückseite / Poti-Seite)',
      'Lage Pos 2 und Pos 3 strichpunktiert (Schweißgruppe ZB-01)',
      'Gewinde wahlweise vor oder nach dem Schweißen',
      '!S1–S8: Teilung 25 / Abstand 12,5 vor dem Bohren an Schiene prüfen',
    ],
  });
  const v = view(s, 45, 118, 0.5, [45, 34, 'Draufsicht']);
  rect(v, 0, 0, L, B);
  rect(v, XW, 0, XW + WT, B, 'p');
  for (const q of GQ) rect(v, XW + WT, q - GT / 2, L, q + GT / 2, 'p');
  for (const [id, x, y, th] of plateHoles) {
    tapped(v, x, y, +th.slice(1), { noCross: id.startsWith('S') && id !== 'S1' && id !== 'S8' });
    label(s, v.X(x) + 2.4, v.Y(y) - 2.4, id, { size: 2.3, color: id.startsWith('S') ? '#e06a00' : undefined });
  }
  line(v, RAIL_X, QR, RAIL_X + g.RAIL.len, QR, 'c');
  el(s, 'path', { d: `M${v.X(0)} ${v.Y(0)} m-4 0 l8 0 m-4 -4 l0 8`, class: 'k' });
  label(s, v.X(0) - 3, v.Y(0) + 6, '0 / 0', { size: 2.5 });
  dim(v, [0, 0], [L, 0], 'h', -10);
  dim(v, [0, 0], [0, B], 'v', -8);
  label(s, v.X(L / 2) + 20, v.Y(-14) + 2, 'X →', { size: 2.8 });
  label(s, v.X(-18), v.Y(B / 2) + 12, 'Y ↑', { size: 2.8, anchor: 'end' });
  const sv = view(s, 45, 152, 0.5, [45, 141, 'Seitenansicht']);
  rect(sv, 0, 0, L, T);
  dim(sv, [L, 0], [L, T], 'v', 6);
  table(s, 22, 162, [['Loch', 12], ['X', 16], ['Y', 16], ['Gewinde', 18], ['Tiefe', 24], ['für', 52]],
    plateHoles.map(([id, x, y, th, depth, use]) => [id, fmt(x) + (id.startsWith('S') ? '*' : ''), fmt(y), th, depth, use]), { rh: 5.1, size: 2.6 });
}

// ======================= 4: end wall, gusset, rear wall
{
  const s = sh(4, {
    no: 'BGY-ZB-03', title: 'Stirnwand, Knotenblech, Rückwand', scale: '1:1', material: 'EN AW-5754 H111', stock: 'Pos 2: 1 · Pos 3: 3 · Pos 11: 1',
    notes: [
      'Pos 2: Bohrungen erst nach dem Schweißen, siehe ZB-01',
      '!Pos 11: Ø30 nach Gummitülle (6 Schläuche Ø6) anpassen',
      'Pos 11: Biegeradius innen 1,5–2 (nach Werkzeug)',
    ],
  });
  label(s, 30, 22, 'Pos 2 – Stirnwand (Zuschnitt, t = 2)', { size: 3.5, bold: true });
  const w = view(s, 35, 102, 1);
  rect(w, 0, 0, B, WH);
  dim(w, [0, WH], [B, WH], 'h', 7);
  dim(w, [0, 0], [0, WH], 'v', -7);
  label(s, 35 + B / 2, 70, 'Bohrungen nach dem Schweißen (ZB-01)', { size: 2.8, anchor: 'middle' });
  const wt = view(s, 200, 102, 1);
  rect(wt, 0, 0, WT, WH);
  dim(wt, [0, WH], [WT, WH], 'h', 7);

  label(s, 232, 22, `Pos 3 – Knotenblech (t = ${GT}, 3 Stück)`, { size: 3.5, bold: true });
  const k = view(s, 250, 100, 1);
  poly(k, [[GCH, 0], [GB, 0], [0, GH], [0, GCH]]);
  dim(k, [0, 0], [GB, 0], 'h', -8);
  dim(k, [0, 0], [0, GH], 'v', -8);
  leader(k, [GCH / 2, GCH / 2], 22, -18, `${GCH} × 45° (Freischnitt Naht)`);
  label(s, 300, 50, 'Kante unten: auf Pos 1', { size: 2.6 });
  label(s, 300, 54, 'Kante links: an Pos 2', { size: 2.6 });

  label(s, 30, 117, `Pos 11 – Rückwand (t = ${fmt(REAR.t)}), Ansicht von außen`, { size: 3.5, bold: true });
  const r = view(s, 40, 232, 1);
  rect(r, 0, 0, B, WH);
  line(r, 0, REAR.t, B, REAR.t, 'h');
  hole(r, REAR.grommet.q, REAR.grommet.h, REAR.grommet.d);
  hole(r, REAR.gland.q, REAR.gland.h, REAR.gland.d);
  for (const q of RIV.q) hole(r, q, RIV.h, RIV.d);
  dim(r, [0, WH], [REAR.gland.q, WH], 'h', 6);
  dim(r, [0, WH], [RIV.q[0], WH], 'h', 12);
  dim(r, [0, WH], [REAR.grommet.q, WH], 'h', 18);
  dim(r, [0, WH], [RIV.q[1], WH], 'h', 24);
  dim(r, [0, WH], [B, WH], 'h', 30);
  dim(r, [B, 0], [B, REAR.grommet.h], 'v', 8);
  dim(r, [B, 0], [B, RIV.h], 'v', 16);
  dim(r, [B, 0], [B, WH], 'v', 24);
  leader(r, [REAR.grommet.q + 10.6, REAR.grommet.h - 10.6], 14, 14, `Ø${REAR.grommet.d}*`, { chk: true });
  leader(r, [REAR.gland.q - 5.8, REAR.gland.h - 5.8], -6, 16, `Ø${fmt(REAR.gland.d)} (M16)`);
  leader(r, [RIV.q[1] - 2.1, RIV.h - 2.1], -14, 12, '2× Ø6 Blindnietmutter M4');
  label(s, 40, 237, '← Poti-Seite', { size: 2.6 });
  const rs = view(s, 203, 232, 1, [200, 155, 'Seitenansicht']);
  poly(rs, [[0, 0], [0, WH], [REAR.t, WH], [REAR.t, REAR.t], [REAR.flange, REAR.t], [REAR.flange, 0]]);
  dim(rs, [0, 0], [REAR.flange, 0], 'h', -7);
  dim(rs, [0, 0], [REAR.flangeHole, 0], 'h', -14);
  line(rs, REAR.flangeHole, -2, REAR.flangeHole, REAR.t + 2, 'c');
  label(s, 207, 165, 'innen →', { size: 2.6 });
  const rb = view(s, 40, 263, 1, [100, 244, 'Lasche von unten']);
  rect(rb, 0, 0, B, REAR.flange);
  for (const q of REAR.flangeQ) hole(rb, q, REAR.flange - REAR.flangeHole, 4.5);
  dim(rb, [0, 0], [REAR.flangeQ[0], 0], 'h', -5);
  dim(rb, [0, 0], [REAR.flangeQ[1], 0], 'h', -11, { tx: 20 });
  leader(rb, [REAR.flangeQ[1] + 1.6, REAR.flange - REAR.flangeHole + 1.6], 10, -4, '2× Ø4,5');
}

// ======================= 5: brackets
{
  const s = sh(5, {
    no: 'BGY-ZB-04', title: 'Lagerböcke und Mitnehmer', scale: '1:1', material: 'EN AW-6060 T66', stock: 'Winkel EN 755-9 · Pos 4: 2 · Pos 5: 1 · Pos 6: 1',
    notes: [
      'Bohrungshöhe 40 bzw. 21 = Zylinderachse, Funktionsmaß',
      'Ø22,5 nimmt das Kopfgewinde M22×1,5 der Zylinder auf',
      'Innenradius der Winkel profilbedingt',
    ],
  });
  // Pos 4
  label(s, 30, 22, `Pos 4 – Lagerbock G / A (2 Stück), Winkel ${BR.h} × ${BR.foot} × ${BR.t}`, { size: 3.5, bold: true });
  const f = view(s, 40, 100, 1);
  rect(f, 0, 0, BR.w, BR.h);
  line(f, 0, BR.t, BR.w, BR.t, 'h');
  hole(f, BR.w / 2, AXH, BR.hole);
  dim(f, [0, BR.h], [BR.w, BR.h], 'h', 7);
  dim(f, [0, 0], [0, BR.h], 'v', -7);
  dim(f, [BR.w, 0], [BR.w, AXH], 'v', 8, { tol: '±0,1' });
  dim(f, [0, 0], [BR.w / 2, 0], 'h', -7);
  leader(f, [BR.w / 2 + 8, AXH + 8], 18, -16, `Ø${fmt(BR.hole)} +0,2/0`);
  const sv = view(s, 115, 100, 1);
  poly(sv, [[0, 0], [BR.foot, 0], [BR.foot, BR.h], [BR.foot - BR.t, BR.h], [BR.foot - BR.t, BR.t], [0, BR.t]]);
  for (const dw of [-BR.hole / 2, BR.hole / 2]) line(sv, BR.foot - BR.t, AXH + dw, BR.foot, AXH + dw, 'h');
  line(sv, BR.foot - BR.t - 2, AXH, BR.foot + 2, AXH, 'c');
  rect(sv, BR.foot - SLOT_C - BR.slot / 2, 0, BR.foot - SLOT_C + BR.slot / 2, BR.t, 'h');
  dim(sv, [0, 0], [BR.foot, 0], 'h', -7);
  dim(sv, [BR.foot - BR.t, BR.h], [BR.foot, BR.h], 'h', 7);
  dim(sv, [0, 0], [0, BR.t], 'v', -6);
  label(s, 115 + BR.foot, 106, 'Mutterseite', { size: 2.4, anchor: 'end' });
  const tv = view(s, 40, 185, 1);
  rect(tv, 0, 0, BR.w, BR.foot);
  line(tv, 0, BR.t, BR.w, BR.t, 't');
  for (const dq of [-SLOT_Q, SLOT_Q]) slot(tv, BR.w / 2 + dq, SLOT_C, BR.slot, 6.6, 'w');
  dim(tv, [BR.w, 0], [BR.w, SLOT_C], 'v', 8);
  dim(tv, [0, BR.foot], [BR.w / 2 - SLOT_Q, BR.foot], 'h', 6);
  dim(tv, [BR.w / 2 - SLOT_Q, BR.foot], [BR.w / 2 + SLOT_Q, BR.foot], 'h', 12);
  leader(tv, [BR.w / 2 + SLOT_Q + 3.3, SLOT_C + 4], 14, 8, ['2× Langloch', `6,6 × ${BR.slot}`]);
  label(s, 40, 190, 'Mutterseite', { size: 2.4 });

  // Pos 5 + 6
  label(s, 200, 22, `Pos 5 – Lagerbock B  ·  Pos 6 – Mitnehmer, Winkel ${SB.h} × ${SB.foot} × ${SB.t}`, { size: 3.5, bold: true });
  for (const [i, pos] of [[0, 'Pos 5'], [1, 'Pos 6']]) {
    const v5 = view(s, 205 + i * 70, 80, 1, [205 + i * 70, 89, pos]);
    rect(v5, 0, 0, SB.w, SB.h);
    line(v5, 0, SB.t, SB.w, SB.t, 'h');
    if (i === 0) { hole(v5, SB.w / 2, SLED_AX, BR.hole); leader(v5, [SB.w / 2 + 8, SLED_AX + 8], 6, -14, [`Ø${fmt(BR.hole)}`, '+0,2/0']); }
    else { tapped(v5, SB.w / 2, SLED_AX, 10); leader(v5, [SB.w / 2 + 3.5, SLED_AX + 3.5], 12, -16, 'M10×1,25 durchg.'); }
    dim(v5, [0, SB.h], [SB.w, SB.h], 'h', 7);
    dim(v5, [SB.w, 0], [SB.w, SLED_AX], 'v', 7, { tol: '±0,1' });
    if (i === 0) dim(v5, [0, 0], [0, SB.h], 'v', -6);
  }
  const s5 = view(s, 350, 80, 1);
  poly(s5, [[0, 0], [SB.foot, 0], [SB.foot, SB.h], [SB.foot - SB.t, SB.h], [SB.foot - SB.t, SB.t], [0, SB.t]]);
  line(s5, SB.foot - SB.t - 2, SLED_AX, SB.foot + 2, SLED_AX, 'c');
  for (const x of [SB.foot - SB_HOLE - 2.75, SB.foot - SB_HOLE + 2.75]) line(s5, x, 0, x, SB.t, 'h');
  dim(s5, [0, 0], [SB.foot, 0], 'h', -7);
  dim(s5, [SB.foot - SB_HOLE, 0], [SB.foot, 0], 'h', -15);
  dim(s5, [0, 0], [0, SB.t], 'v', -6);
  const t5 = view(s, 205, 152, 1, [205, 163, 'Fuß von oben (Pos 5 und 6 gleich)']);
  rect(t5, 0, 0, SB.w, SB.foot);
  line(t5, 0, SB.t, SB.w, SB.t, 't');
  for (const dq of [-SB_Q, SB_Q]) hole(t5, SB.w / 2 + dq, SB_HOLE, 5.5, { cs: 10.4 });
  dim(t5, [0, SB.foot], [SB.w / 2 - SB_Q, SB.foot], 'h', 6);
  dim(t5, [SB.w / 2 - SB_Q, SB.foot], [SB.w / 2 + SB_Q, SB.foot], 'h', 12);
  dim(t5, [SB.w, 0], [SB.w, SB_HOLE], 'v', 8);
  leader(t5, [SB.w / 2 + SB_Q + 3.7, SB_HOLE + 3.7], 18, -2, ['2× Ø5,5, Senkung 90° Ø10,4', 'von oben, für M5 ISO 10642']);
  label(s, 205, 156, 'Stegseite', { size: 2.4 });
  label(s, 205, 169, 'Fuß zeigt im Einbau unter Zylinder B.', { size: 2.6 });
}

// ======================= 6: sled, sleeve, poti strap
{
  const s = sh(6, {
    no: 'BGY-ZB-05', title: 'Schlitten, Anschlaghülse, Poti-Lasche', scale: '1:1 / 2:1', material: 'Pos 7: EN AW-5083 · Pos 8: EN AW-6082 T6 · Pos 9: EN AW-5754', stock: 'je 1 Stück',
    notes: [
      '!Pos 7: Bohrbild Wagen an MGN12H-Wagen prüfen (HIWIN: 20 × 20)',
      'Pos 7: Senkschrauben bündig, B liegt 7,5 mm darüber',
      '!Pos 9: Ø4,2 nach Augenbolzen des Potis',
    ],
  });
  label(s, 30, 22, `Pos 7 – Schlitten (t = ${SLED_T})`, { size: 3.5, bold: true });
  const v = view(s, 35, 95, 1);
  rect(v, 0, 0, SLED_L, SLED_W);
  for (const c of [CAR_C, SLED_L - CAR_C]) for (const dx of [-10, 10]) for (const dw of [-10, 10]) hole(v, c + dx, SLED_W / 2 + dw, 3.4, { cs: 6.5 });
  for (const x of [SB_HOLE, SLED_L - SB_HOLE]) for (const dw of [-SB_Q, SB_Q]) tapped(v, x, SLED_W / 2 + dw, 5);
  dim(v, [0, SLED_W], [SLED_L, SLED_W], 'h', 22);
  dim(v, [0, SLED_W], [CAR_C - 10, SLED_W], 'h', 8, { chk: true });
  dim(v, [CAR_C - 10, SLED_W], [CAR_C + 10, SLED_W], 'h', 8, { chk: true });
  dim(v, [0, SLED_W], [SB_HOLE, SLED_W], 'h', 15);
  dim(v, [SLED_L - SB_HOLE, SLED_W], [SLED_L, SLED_W], 'h', 15);
  dim(v, [SLED_L - CAR_C - 10, SLED_W], [SLED_L - CAR_C + 10, SLED_W], 'h', 8, { chk: true });
  dim(v, [SLED_L, 0], [SLED_L, SLED_W], 'v', 24);
  dim(v, [SLED_L, 0], [SLED_L, SLED_W / 2 - 10], 'v', 8, { chk: true });
  dim(v, [SLED_L, SLED_W / 2 - 10], [SLED_L, SLED_W / 2 + 10], 'v', 8, { chk: true });
  dim(v, [SLED_L, 0], [SLED_L, SLED_W / 2 - SB_Q], 'v', 16, { tx: 4 });
  leader(v, [CAR_C + 10 + 2.3, SLED_W / 2 - 10 - 2.3], 10, 18, ['8× Ø3,4, Senkung 90° Ø6,5 von oben*', 'für M3 ISO 10642'], { chk: true });
  leader(v, [SLED_L - SB_HOLE - 2, SLED_W / 2 - SB_Q - 2], -6, 32, '4× M5 durchgehend');
  label(s, v.X(SLED_L / 2), v.Y(SLED_W / 2) + 1, '← Mitnehmer (Pos 6)   ·   Lagerbock B (Pos 5), Seilseite →', { size: 2.4, anchor: 'middle' });
  const sv = view(s, 35, 145, 1);
  rect(sv, 0, 0, SLED_L, SLED_T);
  dim(sv, [0, 0], [0, SLED_T], 'v', -6);

  label(s, 230, 22, 'Pos 8 – Anschlaghülse, Schnitt (2:1)', { size: 3.5, bold: true });
  const h = view(s, 245, 75, 2);
  for (const sg of [1, -1]) {
    const p = poly(h, [[0, sg * 5.25], [40, sg * 5.25], [40, sg * 8], [0, sg * 8]], 'k');
    p.setAttribute('class', 'k'); p.setAttribute('style', 'fill:url(#hatch)');
  }
  line(h, -3, 0, 43, 0, 'c');
  dim(h, [0, 8], [40, 8], 'h', 8, { tol: '0/−0,1' });
  dim(h, [40, -8], [40, 8], 'v', 8, { pre: 'Ø' });
  dim(h, [0, -5.25], [0, 5.25], 'v', -8, { pre: 'Ø', text: 'Ø10,5 +0,2/0' });
  datum(h, [0, -6.5], -8, 8, 'A');
  gtol(h, [40, -6.5], 6, 10, ['∥', '0,05', 'A']);
  label(s, 245, 110, 'Kanten 0,5 × 45° gefast, Stirnflächen plan', { size: 2.6 });

  label(s, 230, 128, `Pos 9 – Poti-Lasche (t = ${LASCHE.t}, 2:1)`, { size: 3.5, bold: true });
  const l = view(s, 250, 175, 2);
  const R = LASCHE.w / 2, Lw = LASCHE.l;
  const X0 = l.X(R), X1 = l.X(Lw - R), Yt = l.Y(R), Yb = l.Y(-R), RR = R * 2;
  el(l.g, 'path', { d: `M${X0} ${Yt} L${X1} ${Yt} A${RR} ${RR} 0 0 1 ${X1} ${Yb} L${X0} ${Yb} A${RR} ${RR} 0 0 1 ${X0} ${Yt} Z`, class: 'k' });
  hole(l, LASCHE.a, 0, 8.5);
  hole(l, LASCHE.a + LASCHE.pitch, 0, 4.2);
  dim(l, [0, 0], [Lw, 0], 'h', -(R * 2 + 8));
  dim(l, [LASCHE.a, 0], [LASCHE.a + LASCHE.pitch, 0], 'h', R * 2 + 6, { tol: '±0,2' });
  dim(l, [Lw, -R], [Lw, R], 'v', 8);
  leader(l, [LASCHE.a + 3, 3], -10, -12, 'Ø8,5');
  leader(l, [LASCHE.a + LASCHE.pitch + 1.5, 1.5], 10, -12, 'Ø4,2*', { chk: true });
  leader(l, [Lw - R + R * 0.7, -R * 0.7], 8, 10, `R${R}`);
  label(s, 245, 207, 'Ø8,5 auf Kolbenstange G zwischen Kontermutter und Gabelkopf', { size: 2.6 });
}

// ======================= 7: cover
{
  const s = sh(7, {
    no: 'BGY-ZB-06', title: 'Haube (Pos 10)', scale: '1:2', material: 'EN AW-5754 H111', stock: `Blech ${fmt(LID.t)}, gekantet, 1 Stück`,
    notes: [
      `Innenmaß ${fmt(LID_W - 2 * LID.t)} × ${fmt(LID_H - LID.t)} über Grundplatte und Stirnwand`,
      'Biegeradius innen 1,5–2 (nach Werkzeug); Abwicklung durch Fertiger',
      'Ecken mit Ausklinkung, offen (Spritzschutz), nicht verschweißen',
      '4× Ø4,5 für M4 in Blindnietmuttern (Pos 12) an Pos 2 und Pos 11',
    ],
  });
  const v = view(s, 45, 72, 0.5, [45, 26, 'Vorderansicht (von der Poti-Seite)']);
  rect(v, 0, 0, LID_L, LID_H);
  line(v, LID.t, LID_H - LID.t, LID_L - LID.t, LID_H - LID.t, 'h');
  for (const x of [LID.t, LID_L - LID.t]) line(v, x, LID_H - LID.lip, x, LID_H - LID.t, 'h');
  for (const x of [0, LID_L - LID.t]) line(v, x, LID_H - LID.lip, x + LID.t, LID_H - LID.lip, 'h');
  dim(v, [0, 0], [LID_L, 0], 'h', -10);
  dim(v, [LID_L, 0], [LID_L, LID_H], 'v', 10);
  dim(v, [0, LID_H - LID.lip], [0, LID_H], 'v', -8);
  label(s, v.X(8), v.Y(LID_H / 2), 'Rückwand-Seite', { size: 2.6 });
  label(s, v.X(LID_L - 8), v.Y(LID_H / 2), 'Stirnwand-Seite (Seilseite)', { size: 2.6, anchor: 'end' });
  { const ax = v.X(LID_L) + 9, ay = v.Y(LID_H / 2) + 8; el(s, 'line', { x1: ax + 6, y1: ay, x2: ax, y2: ay, class: 'k' }); el(s, 'path', { d: `M${ax} ${ay} l3 -1.2 l0 2.4 Z`, class: 'fill' }); label(s, ax + 3, ay - 2.5, 'B', { size: 4, bold: true }); }
  const t = view(s, 45, 168, 0.5, [45, 87, 'Draufsicht']);
  rect(t, 0, 0, LID_L, LID_W);
  for (const w of [LID.t, LID_W - LID.t]) line(t, LID.t, w, LID_L - LID.t, w, 't');
  for (const x of [LID.t, LID_L - LID.t]) line(t, x, LID.t, x, LID_W - LID.t, 't');
  dim(t, [LID_L, 0], [LID_L, LID_W], 'v', 10);

  const b = view(s, 50, 280, 1, [230, 186, 'Ansicht B (1:1)']);
  label(s, 230, 191, 'Lasche Stirnwand-Seite, Rückwand-Seite gleich', { size: 2.6 });
  poly(b, [[0, 0], [0, LID_H], [LID_W, LID_H], [LID_W, 0], [LID_W - LID.t, 0], [LID_W - LID.t, LID_H - LID.t], [LID.t, LID_H - LID.t], [LID.t, 0]]);
  rect(b, LID.t, LID_H - LID.lip, LID_W - LID.t, LID_H - LID.t);
  const qh = RIV.q.map((q) => LID_W - (q + LID.gap + LID.t));
  const hz = T + RIV.h;
  for (const u of qh) hole(b, u, hz, 4.5);
  dim(b, [LID_W, LID_H], [qh[0], LID_H], 'h', 6);
  dim(b, [LID_W, LID_H], [qh[1], LID_H], 'h', 12);
  dim(b, [0, LID_H], [LID_W, LID_H], 'h', 18);
  dim(b, [LID_W, hz], [LID_W, LID_H], 'v', 8);
  dim(b, [LID_W, LID_H - LID.lip], [LID_W, LID_H], 'v', 16);
  dim(b, [0, 0], [0, LID_H], 'v', -7);
  dim(b, [0, 4], [LID.t, 4], 'h', -10, { text: fmt(LID.t) });
  leader(b, [qh[1] - 1.6, hz + 1.6], -10, -10, '2× Ø4,5');
  label(s, 50 + LID_W, 285, 'Poti-Seite', { size: 2.5, anchor: 'end' });
}

document.title = 'Buggy – Zylinderbox Zeichnungssatz (Entwurf)';
window.drawingsReady = true;
