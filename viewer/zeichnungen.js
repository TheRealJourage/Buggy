// Fabrication drawing set for the cylinder box (7 x A3), built from boxgeo.js.
// Index C (2026-11-11): all metal parts stainless, welded tub with inward flange + PC lid like the relay box.
// Plate coordinates: X from the rear edge of the base plate, Y from the poti-side edge, Z from its underside.
import * as g from './boxgeo.js';
import { sheet, view, poly, rect, line, circle, hole, tapped, slot, dim, leader, weld, balloon, datum, gtol, label, table, cross, el, fmt } from './drafting.js';

const root = document.getElementById('sheets');
const N = 7;
const BASE_NOTES = [
  'Maße in mm · Allgemeintoleranzen ISO 2768-mK',
  'Werkstückkanten ISO 13715: −0,3 (alle Kanten entgratet)',
  'Oberfläche roh, Schweißnähte gebeizt/passiviert, keine Alu- oder Stahl-Werkzeugspuren (Fremdrost)',
  '!Orange mit * = Prüfmaß: vor Fertigung vom Auftraggeber bestätigen',
];
const sh = (k, info) => sheet(root, { sheet: `${k} / ${N}`, date: '2026-11-11', rev: 'C', ...info, notes: [...(info.notes || []), ...BASE_NOTES] });

// ---------- fabrication geometry (rounded where a slot or the general tolerance allows it)
const r1 = (v) => Math.round(v * 10) / 10;
const PX = (x) => r1(x - g.PLATE.x0), PY = (z) => r1(z - g.PLATE.z0);
const L = PX(g.PLATE.x1), B = g.PLATE.z1 - g.PLATE.z0, T = g.FLOOR_T;
const XW = PX(g.WALL_X), WT = g.WALL_T, WH = g.WALL_H, AXH = g.AX_H, AXZ = T + AXH;
const QG = PY(g.G_Z), QR = PY(g.ROW_Z), QP = PY(g.POTI_Z);
const GQ = g.GUSSET.z.map(PY), GT = g.GUSSET.t, GB = g.GUSSET.base, GH = g.GUSSET.h, GCH = 5;
const BR = g.BRACKET, SB = g.SLED_BRACKET;
const SLOT_C = BR.t + (BR.foot - BR.t) / 2, SLOT_Q = 17;
const SB_HOLE = SB.t + (SB.foot - SB.t) / 2, SB_Q = 15;
const XG_SLOT = Math.round(PX(g.G_BRACKET_X) - SLOT_C), XA_SLOT = Math.round(PX(g.A_BRACKET_X) - SLOT_C);
const RAIL_X = PX(g.RAIL_X0), RAIL_E = 12.5;
const SLED_L = r1(g.SLED.x1 - g.SLED.x0), SLED_W = g.SLED_W, SLED_T = g.SLED_T, SLED_AX = r1(g.AX_Y - g.SLED_TOP);
const CAR_C = 23; // carriage centre from the sled ends
// tub (Index C): walls with a 25 mm flange inward, PC lid on EPDM tape, M5 into stainless rivet nuts
const FL = g.FLANGE, ST = g.SHELL_T, BL = XW + WT;                      // BL = box outer length
const TOP = T + WH, LID_H = g.CB.h, LID = { t: g.LID_T, L: BL, W: B, hole: 7 };
const LS = g.LID_SCREWS.map(([x, z]) => [r1(PX(x)), r1(PY(z))]);
const LSX = [...new Set(LS.map(([x]) => x))].sort((a, b) => a - b), LSY = [...new Set(LS.map(([, y]) => y))].sort((a, b) => a - b);
const SEAL_LEN = Math.round((2 * (BL - 2 * FL.seal) + 2 * (B - 2 * FL.seal)) / 100) / 10; // m
const REAR = { t: ST, glands: g.REAR_GLANDS.y.map((y, i) => ({ q: r1(PY(g.REAR_GLANDS.z)), h: y - T, d: g.REAR_GLANDS.d, what: g.REAR_GLANDS.what[i] })) };
const SIDE = { x0: ST, x1: XW, f0: FL.w, f1: BL - FL.w };               // side wall between rear and end wall, flange butts on theirs
const angle = (v, x0, y0, h, f, t, dir = 1) => poly(v, [[x0, y0], [x0, y0 + h], [x0 + dir * f, y0 + h], [x0 + dir * f, y0 + h - t], [x0 + dir * t, y0 + h - t], [x0 + dir * t, y0]]);
// valve stack (variant B, 2026-11-11): M4 tapped holes for the through bolts, from the scaled datasheet drawing (check on the valve)
const STK = { x0: PX(g.VSTACK.x0), x1: PX(g.VSTACK.x1), q0: PY(g.VSTACK.zOut), q1: PY(g.VSTACK.zIn), h: g.STACK_H };
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
  ...g.VALVE.holes.map(([dx, dz], i) => [`V${i + 1}`, PX(g.VSTACK.cx + dx), r1(PY(g.VSTACK.zOut + g.VALVE.body - dz)), 'M4', 'durchgehend', 'Ventilstapel*']),
  ['E1', PX(g.VSTACK.cx), r1(PY(g.CB.z0 + g.WALL_T + 10)), 'Ø6', 'durchgehend', 'Ablauf Wasser'],
];

// guards: rounded fabrication values must stay close to the design geometry
console.assert(L === 700 && XW === 658 && WH === 86 && B === 172 && LS.length === 18, 'plate/wall', L, XW, WH, B, LS.length);
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
      'Pos 1–3, 11, 13 als Schweißbaugruppe (Wanne), siehe BGY-ZB-01',
      'Pos 4–9 lose liefern, Pos 12 setzt der Fertiger nach dem Schweißen',
      'Pos 10 und 14 beschafft der Auftraggeber (Deckel erst an der Wanne messen)',
      'Strich-Zweipunkt: Kaufteile des Auftraggebers (Zylinder, Führung,',
      '  Poti, FK-32, Gabelköpfe, Seile, Ventile), nicht im Lieferumfang',
    ],
  });
  const v = view(s, 45, 80, 0.5, [45, 17, 'Vorderansicht (von der Poti-Seite, Seitenwand Pos 13 nicht dargestellt)']);
  rect(v, 0, 0, L, T);
  angle(v, XW + WT, T, WH, FL.w, WT, -1);
  poly(v, [[XW + WT + GCH, T], [L, T], [XW + WT, T + GH], [XW + WT, T + GCH]]);
  angle(v, 0, T, WH, FL.w, REAR.t);
  rect(v, 0, TOP + g.GASKET, BL, LID_H, 'p');
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
  rect(v, STK.x0, T + g.STACK.base, STK.x1, T + STK.h, 'p');
  label(s, 45 + (STK.x0 + 40) * 0.5, 80 - (T + STK.h + 3) * 0.5, 'Ventilstapel (Kaufteile)', { size: 2.5 });
  line(v, -10, AXZ, L + 10, AXZ, 'c');
  dim(v, [0, 0], [L, 0], 'h', -10);
  dim(v, [L, T], [L, AXZ], 'v', 8, { text: `${AXH}` });
  dim(v, [L, 0], [L, TOP], 'v', 16);
  dim(v, [L, 0], [L, LID_H], 'v', 24);
  for (const [p, dx, dy, n] of [[[150, 3], 0, 14, '1'], [[XW + 1, 70], 10, -10, '2'], [[XW + 15, 35], 18, -8, '3'], [[A_.gBr - 3, 55], -6, -18, '4'], [[A_.sled1 - 2, SLED_Z + 30], 4, -22, '5'], [[A_.sled0 + 2, SLED_Z + 30], -6, -24, '6'], [[A_.sled0 + 60, SLED_Z + 3], -26, 20, '7'], [[A_.aHead + 20, AXZ + 8], 0, -20, '8'], [[430, LID_H], 8, -3, '10'], [[1, 40], -10, 6, '11'], [[FL.screw, TOP], -14, -3, '12'], [[BL - FL.seal, TOP + 1.5], -16, -4, '14']]) balloon(v, p, dx, dy, n);

  const t = view(s, 45, 188, 0.5, [45, 101, 'Draufsicht (Deckel abgenommen)']);
  rect(t, 0, 0, L, B);
  rect(t, XW, 0, XW + WT, B);
  for (const q of GQ) rect(t, XW + WT, q - GT / 2, L, q + GT / 2);
  rect(t, FL.w, FL.w, BL - FL.w, B - FL.w);
  rect(t, 0, 0, BL, B);
  for (const [x, y] of LS) circle(t, x, y, LID.hole / 2);
  bracketTop(t, A_.gBr, QG, BR, -1, true); bracketTop(t, A_.aBr, QR, BR, -1, true);
  rect(t, A_.sled0, QR - SLED_W / 2, A_.sled1, QR + SLED_W / 2);
  bracketTop(t, A_.sled1, QR, SB); bracketTop(t, A_.sled0, QR, SB, 1);
  phantomCyl(t, A_.gX0, A_.gHead, QG, 13.5); phantomCyl(t, A_.gHead, A_.gRod, QG, 4); phantomCyl(t, A_.gRod, A_.gRod + 22, QG, 8);
  phantomCyl(t, A_.aX0, A_.aHead, QR, 15); rect(t, A_.aHead, QR - 8, A_.aRod, QR + 8); phantomCyl(t, A_.aRod, A_.sled0, QR, 11);
  phantomCyl(t, A_.bRear, A_.bHead, QR, 13.5); phantomCyl(t, A_.bHead, A_.bRod, QR, 4); phantomCyl(t, A_.bRod, A_.bRod + 22, QR, 8);
  phantomCyl(t, A_.gRod - 120, A_.gRod + 65, QP, 6.5);
  rect(t, STK.x0, STK.q0, STK.x1, STK.q1, 'p');
  label(s, 45 + (STK.x0 + 50) * 0.5, 188 - ((STK.q0 + STK.q1) / 2) * 0.5, '3 Ventile M-04', { size: 2.5 });
  rect(t, A_.gRod - LASCHE.t, QP - LASCHE.a, A_.gRod, QG + LASCHE.a);
  for (const q of [QG, QR]) { line(t, -10, q, L + 10, q, 'c'); phantomCyl(t, XW - (q === QG ? 70 : 130), XW, q, q === QG ? 10.5 : 5.5); }
  dim(t, [0, 0], [XW, 0], 'h', -6);
  dim(t, [L, 0], [L, B], 'v', 10);
  for (const [p, dx, dy, n] of [[[A_.aHead + 20, QR], 0, -18, '8'], [[A_.gRod - 1.5, QP + 4], 22, 4, '9'], [[2, B / 2], -10, 0, '11'], [[560, 1], 0, 10, '13'], [[560, B - 1], 6, -4, '13']]) balloon(t, p, dx, dy, n);
  label(s, 45 + (A_.gRod - 60) * 0.5, 188 - (QP - 9) * 0.5, 'Poti', { size: 2.5 });

  const W = '1.4301';
  table(s, 22, 199, [['Pos', 9], ['Menge', 11], ['Benennung', 38], ['Werkstoff', 30], ['Halbzeug / Abmessung', 58], ['Zeichnung', 22], ['Bemerkung', 34]], [
    ['1', '1', 'Grundplatte', W, `Blech ${T} × ${B} × ${L}`, 'ZB-02', 'Schweißgruppe ZB-01'],
    ['2', '1', 'Stirnwand', W, `Blech ${WT}, gekantet, Flansch ${FL.w}`, 'ZB-03', 'Schweißgruppe ZB-01'],
    ['3', '3', 'Knotenblech', W, `Blech ${GT}, Dreieck ${GB} × ${GH}`, 'ZB-03', 'Schweißgruppe ZB-01'],
    ['4', '2', 'Lagerbock G / A', W, `Winkel ${BR.h}×${BR.foot}×${BR.t}, L ${BR.w}`, 'ZB-04', 'Profil EN 10056-1*'],
    ['5', '1', 'Lagerbock B', W, `Winkel ${SB.h}×${SB.foot}×${SB.t}, L ${SB.w}`, 'ZB-04', 'Profil EN 10056-1*'],
    ['6', '1', 'Mitnehmer', W, `Winkel ${SB.h}×${SB.foot}×${SB.t}, L ${SB.w}`, 'ZB-04', 'Profil EN 10056-1*'],
    ['7', '1', 'Schlitten', W, `Blech ${SLED_T} × ${SLED_W} × ${SLED_L}`, 'ZB-05', ''],
    ['8', '1', 'Anschlaghülse', '1.4305', 'Rund Ø16 × 40', 'ZB-05', ''],
    ['9', '1', 'Poti-Lasche', W, `Blech ${LASCHE.t} × ${LASCHE.w} × ${LASCHE.l}`, 'ZB-05', ''],
    ['10', '1', 'Deckel', 'PC, glasklar, UV', `Platte ${LID.t} × ${B} × ${BL}`, 'ZB-06', 'Auftraggeber'],
    ['11', '1', 'Rückwand', W, `Blech ${fmt(ST)}, gekantet, Flansch ${FL.w}`, 'ZB-03', 'Schweißgruppe ZB-01'],
    ['12', String(LS.length), 'Einnietmutter M5', 'A2', 'Flachkopf, Klemmbereich ≥ 1,5–2*', 'ZB-01', 'nach dem Schweißen'],
    ['13', '2', 'Seitenwand', W, `Blech ${fmt(ST)}, gekantet, Flansch ${FL.w}`, 'ZB-06', 'Schweißgruppe ZB-01'],
    ['14', '1', 'Dichtband', 'EPDM-Zellkautschuk', `10 × 5 selbstklebend, ca. ${fmt(SEAL_LEN + 0.1)} m`, '–', 'Auftraggeber'],
  ], { rh: 5, size: 2.6 });
}

// ======================= 2: weldment
{
  const s = sh(2, {
    no: 'BGY-ZB-01', title: 'Wanne – Schweißbaugruppe', scale: '1:2', material: '1.4301 (X5CrNi18-10)', stock: '1 Stück (Pos 1, 2, 3 × 3, 11, 13 × 2)',
    notes: [
      'Schweißverfahren 141 (WIG), Zusatz W 19 9 L Si / ER308LSi (Vorschlag)',
      'Alle Wandstöße, Wand–Platte und Flanschstöße umlaufend dicht (Spritzwasser)',
      'Flanschoberseite = Dichtfläche: eben 0,5, Nähte dort bündig schleifen',
      `${LS.length}× Ø${LID.hole} für Pos 12 nach dem Schweißen, Ø nach Hersteller der Mutter`,
      'Bohrungen in Pos 2 nach dem Schweißen; Gewinde Pos 1 siehe ZB-02',
      'Spitzen der Knotenbleche 2 × 45° brechen',
    ],
  });
  const v = view(s, 45, 64, 0.5, [45, 15, 'Vorderansicht (von der Poti-Seite)']);
  rect(v, 0, 0, L, T);
  rect(v, 0, T, BL, TOP);
  line(v, FL.w, TOP - ST, BL - FL.w, TOP - ST, 'h');
  poly(v, [[XW + WT + GCH, T], [L, T], [XW + WT, T + GH], [XW + WT, T + GCH]]);
  dim(v, [0, 0], [L, 0], 'h', -8);
  dim(v, [0, 0], [0, T], 'v', -6);
  dim(v, [0, T], [0, TOP], 'v', -12);
  dim(v, [0, 0], [0, TOP], 'v', -20);
  dim(v, [L, T], [L, T + GH], 'v', 8);
  datum(v, [300, T], 0, -10, 'A');
  weld(v, [200, T], -40, -10, 'a1,5', { left: true });
  label(s, 45 + 160 * 0.5 - 2, 64 - (T - 14) * 0.5, 'umlaufend', { size: 2.3, anchor: 'end' });
  gtol(v, [XW, T + 60], -150, -12, ['⊥', '0,3', 'A']);
  weld(v, [XW + WT + 22, T], -30, 12, 'a2,5', { left: true });
  { const ax = v.X(L) + 9, ay = v.Y(AXZ); el(s, 'line', { x1: ax + 6, y1: ay, x2: ax, y2: ay, class: 'k' }); el(s, 'path', { d: `M${ax} ${ay} l3 -1.2 l0 2.4 Z`, class: 'fill' }); label(s, ax + 3, ay - 2.5, 'A', { size: 4, bold: true }); }

  const t = view(s, 45, 168, 0.5, [45, 80, 'Draufsicht, Flansch mit Löchern für Pos 12']);
  rect(t, 0, 0, L, B);
  rect(t, 0, 0, BL, B);
  rect(t, FL.w, FL.w, BL - FL.w, B - FL.w);
  for (const x of [ST, XW]) line(t, x, FL.w, x, B - FL.w, 'h');
  for (const y of [ST, B - ST]) line(t, FL.w, y, BL - FL.w, y, 'h');
  for (const q of GQ) { rect(t, BL, q - GT / 2, L, q + GT / 2); line(t, BL - 6, q, L + 4, q, 'c'); }
  for (const [x, y] of LS) circle(t, x, y, LID.hole / 2);
  for (const x of LSX) line(t, x, -3, x, 3, 'c');
  dim(t, [0, 0], [LSX[0], 0], 'h', -7);
  dim(t, [LSX[0], 0], [LSX[1], 0], 'h', -7, { text: `${LSX.length - 1}× ${fmt(LSX[1] - LSX[0])} (= ${fmt(LSX.at(-1) - LSX[0])})` });
  dim(t, [0, 0], [BL, 0], 'h', -14);
  dim(t, [BL, 0], [L, 0], 'h', -14);
  dim(t, [0, 0], [0, LSY[1]], 'v', -8);
  dim(t, [0, 0], [0, LSY[0]], 'v', -15, { tx: -4 });
  dim(t, [0, 0], [0, B], 'v', -22);
  dim(t, [BL, 0], [BL, FL.w], 'v', 6);
  leader(t, [LSX[2] + 2.5, LSY[0] + 2.5], 8, -12, `${LS.length}× Ø${LID.hole} (Pos 12)`);
  let prev = 0;
  [...GQ, B].forEach((q, i) => { dim(t, [L, prev], [L, q], 'v', 8, { text: fmt(q - prev) }); prev = q; });
  datum(t, [685, 0], 0, 9, 'B');

  // view A at 1:1 (poti side on the right)
  const a = view(s, 52, 283, 1, [215, 196, 'Ansicht A (1:1)']);
  const U = (q) => B - q;
  rect(a, 0, 0, B, T);
  rect(a, 0, T, B, TOP);
  line(a, 0, TOP - WT, B, TOP - WT, 'h');
  for (const q of GQ) rect(a, U(q) - GT / 2, T, U(q) + GT / 2, T + GH);
  for (const [q, d] of [[QG, g.HOLE_D.G], [QR, g.HOLE_D.R]]) hole(a, U(q), AXZ, d);
  dim(a, [B, AXZ], [U(QG), AXZ], 'h', -(g.HOLE_D.G / 2 + 6), { tol: '±0,2' });
  dim(a, [B, T + 72], [U(QR), T + 72], 'h', 0, { tol: '±0,2' });
  dim(a, [0, T], [0, AXZ], 'v', -7, { tol: '±0,2' });
  dim(a, [B, 0], [B, TOP], 'v', 8, { text: `${TOP}` });
  leader(a, [U(QG) + g.HOLE_D.G / 2 * 0.7, AXZ + g.HOLE_D.G / 2 * 0.7], 12, -10, `Ø${g.HOLE_D.G} +0,2/0*`, { chk: true });
  leader(a, [U(QR) - g.HOLE_D.R / 2 * 0.7, AXZ + g.HOLE_D.R / 2 * 0.7], -8, -12, `Ø${g.HOLE_D.R} +0,2/0*`, { chk: true });
  datum(a, [B, T / 2], 8, 0, 'B');
  datum(a, [40, T], 0, 7, 'A');
  label(s, 52 + B / 2, 289.5, 'Poti-Seite →', { size: 2.6, anchor: 'middle' });
}

// ======================= 3: base plate
{
  const s = sh(3, {
    no: 'BGY-ZB-02', title: 'Grundplatte (Pos 1)', scale: '1:2', material: '1.4301 (X5CrNi18-10)', stock: `Blech ${T} × ${B} × ${L}, 1 Stück`,
    notes: [
      'Koordinaten X/Y ab Nullpunkt (Ecke Rückseite / Poti-Seite)',
      'Lage Pos 2, 3, 11, 13 strichpunktiert (Schweißgruppe ZB-01)',
      'Gewinde in 1.4301: Schneidöl, Gewindebohrer für Edelstahl',
      'Gewinde wahlweise vor oder nach dem Schweißen',
      '!S1–S8: Teilung 25 / Abstand 12,5 vor dem Bohren an Schiene prüfen',
    ],
  });
  const v = view(s, 45, 118, 0.5, [45, 34, 'Draufsicht']);
  rect(v, 0, 0, L, B);
  rect(v, XW, 0, XW + WT, B, 'p');
  for (const q of GQ) rect(v, XW + WT, q - GT / 2, L, q + GT / 2, 'p');
  rect(v, 0, 0, REAR.t, B, 'p');
  for (const y of [0, B - ST]) rect(v, SIDE.x0, y, SIDE.x1, y + ST, 'p');
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
    plateHoles.map(([id, x, y, th, depth, use]) => [id, fmt(x) + (/^[SV]/.test(id) ? '*' : ''), fmt(y), th, depth, use]), { rh: 5.1, size: 2.6 });
}

// ======================= 4: end wall, gusset, rear wall
{
  const s = sh(4, {
    no: 'BGY-ZB-03', title: 'Stirnwand, Knotenblech, Rückwand', scale: '1:1', material: '1.4301 (X5CrNi18-10)', stock: 'Pos 2: 1 · Pos 3: 3 · Pos 11: 1',
    notes: [
      'Pos 2 und 11: Flansch 25 nach innen, Biegeradius innen 1,5–2 (nach Werkzeug)',
      'Abwicklung durch Fertiger; Außenmaße gelten',
      'Pos 2: Bohrungen erst nach dem Schweißen, siehe ZB-01',
      '!Pos 11: 3 × Ø16,5 für Kabelverschraubung M16 (Luft 8 mm, Sensorkabel, Spulenkabel) vor dem Schweißen; Klemmbereich 8-mm-Schlauch prüfen*',
    ],
  });
  label(s, 30, 22, `Pos 2 – Stirnwand (t = ${WT}), Ansicht von außen`, { size: 3.5, bold: true });
  const w = view(s, 35, 120, 1);
  rect(w, 0, 0, B, WH);
  line(w, 0, WH - WT, B, WH - WT, 'h');
  dim(w, [0, WH], [B, WH], 'h', 7);
  dim(w, [0, 0], [0, WH], 'v', -7);
  label(s, 35 + B / 2, 85, 'Bohrungen nach dem Schweißen (ZB-01)', { size: 2.8, anchor: 'middle' });
  const wt = view(s, 222, 120, 1, [255, 40, 'Schnitt']);
  angle(wt, 0, 0, WH, FL.w, WT);
  dim(wt, [0, WH], [FL.w, WH], 'h', 7);
  dim(wt, [0, 0], [WT, 0], 'h', -7, { text: fmt(WT) });
  label(s, 222 + FL.w + 2, 120 - WH / 2, '← innen', { size: 2.6 });

  label(s, 268, 22, `Pos 3 – Knotenblech (t = ${GT}, 3 Stück)`, { size: 3.5, bold: true });
  const k = view(s, 285, 100, 1);
  poly(k, [[GCH, 0], [GB, 0], [0, GH], [0, GCH]]);
  dim(k, [0, 0], [GB, 0], 'h', -8);
  dim(k, [0, 0], [0, GH], 'v', -8);
  leader(k, [GCH / 2, GCH / 2], 22, -18, `${GCH} × 45° (Freischnitt Naht)`);
  label(s, 335, 50, 'Kante unten: auf Pos 1', { size: 2.6 });
  label(s, 335, 54, 'Kante links: an Pos 2', { size: 2.6 });

  label(s, 30, 135, `Pos 11 – Rückwand (t = ${fmt(REAR.t)}), Ansicht von außen`, { size: 3.5, bold: true });
  const r = view(s, 52, 238, 1);
  rect(r, 0, 0, B, WH);
  line(r, 0, WH - REAR.t, B, WH - REAR.t, 'h');
  for (const gl of REAR.glands) hole(r, gl.q, gl.h, gl.d);
  dim(r, [0, WH], [REAR.glands[0].q, WH], 'h', 6);
  dim(r, [0, WH], [B, WH], 'h', 13);
  REAR.glands.forEach((gl, i) => dim(r, [0, 0], [0, gl.h], 'v', -6 - 7 * i));
  dim(r, [B, 0], [B, WH], 'v', 8);
  REAR.glands.forEach((gl) => leader(r, [gl.q + 5.8, gl.h + 5.8], 16, 0, `Ø${fmt(gl.d)} (M16) ${gl.what}`));
  label(s, 52, 243, '← Poti-Seite', { size: 2.6 });
  const rs = view(s, 232, 238, 1, [262, 160, 'Schnitt']);
  angle(rs, 0, 0, WH, FL.w, REAR.t);
  dim(rs, [0, WH], [FL.w, WH], 'h', 7);
  label(s, 232 + FL.w + 2, 238 - WH / 2, '← innen', { size: 2.6 });
}

// ======================= 5: brackets
{
  const s = sh(5, {
    no: 'BGY-ZB-04', title: 'Lagerböcke und Mitnehmer', scale: '1:1', material: '1.4301 (X5CrNi18-10)', stock: 'Winkel EN 10056-1 · Pos 4: 2 · Pos 5: 1 · Pos 6: 1',
    notes: [
      'Bohrungshöhe 40 bzw. 21 = Zylinderachse, Funktionsmaß',
      'Ø22,5 nimmt das Kopfgewinde M22×1,5 der Zylinder auf',
      'Innenradius der Winkel profilbedingt',
      '!Winkel 60×60×6 und 40×40×5 in 1.4301: Verfügbarkeit beim Händler prüfen*',
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
    no: 'BGY-ZB-05', title: 'Schlitten, Anschlaghülse, Poti-Lasche', scale: '1:1 / 2:1', material: 'Pos 7, 9: 1.4301 · Pos 8: 1.4305', stock: 'je 1 Stück',
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

// ======================= 7: lid, side wall, seal
{
  const s = sh(7, {
    no: 'BGY-ZB-06', title: 'Deckel (Pos 10), Seitenwand (Pos 13)', scale: '1:2 / 1:1', material: `Pos 10: PC ${LID.t} UV · Pos 13: 1.4301`, stock: 'Pos 10: 1 · Pos 13: 2',
    notes: [
      'Pos 10 beschafft der Auftraggeber: Maß an der fertigen Wanne nehmen,',
      `  ${LS.length}× Ø${LID.hole} selbst bohren (Spiel für Wärmedehnung PC)`,
      'Pos 10: kein Bremsenreiniger, keine flüssige Schraubensicherung am PC',
      'Pos 13: Flansch 25 nach innen, an beiden Enden ausgeklinkt;',
      '  Flansch stößt an die Flansche von Pos 2 und 11 (dicht schweißen)',
      'Pos 14 Dichtband innen neben der Schraubenreihe, Mitte 19 von außen',
    ],
  });
  const t = view(s, 45, 108, 0.5, [45, 17, 'Pos 10 – Deckel, Draufsicht']);
  rect(t, 0, 0, LID.L, LID.W);
  rect(t, FL.seal - 5, FL.seal - 5, LID.L - FL.seal + 5, LID.W - FL.seal + 5, 'p');
  for (const [x, y] of LS) hole(t, x, y, LID.hole);
  dim(t, [0, 0], [LID.L, 0], 'h', -14);
  dim(t, [0, 0], [LSX[0], 0], 'h', -7);
  dim(t, [LSX[0], 0], [LSX[1], 0], 'h', -7, { text: `${LSX.length - 1}× ${fmt(LSX[1] - LSX[0])} (= ${fmt(LSX.at(-1) - LSX[0])})` });
  dim(t, [LID.L, 0], [LID.L, LSY[0]], 'v', 7);
  dim(t, [LID.L, LSY[0]], [LID.L, LSY[1]], 'v', 7, { text: `${LSY.length - 1}× ${fmt(LSY[1] - LSY[0])}` });
  dim(t, [LID.L, 0], [LID.L, LID.W], 'v', 16);
  leader(t, [LSX[3] + 2.5, LSY[0] + 2.5], 10, -12, `${LS.length}× Ø${LID.hole}`);
  label(s, 45 + LID.L * 0.25, 112 - LID.W * 0.25, 'Strich-Zweipunkt: Lage Dichtband (Pos 14) auf dem Flansch', { size: 2.4 });
  label(s, 45, 128, '← Rückwand-Seite', { size: 2.5 });
  label(s, 45 + LID.L * 0.5, 128, 'Stirnwand-Seite →', { size: 2.5, anchor: 'end' });

  const SL = SIDE.x1 - SIDE.x0, F0 = SIDE.f0 - SIDE.x0, F1 = SIDE.f1 - SIDE.x0;
  const sv = view(s, 45, 194, 0.5, [45, 134, 'Pos 13 – Seitenwand (2 Stück, gleich), Ansicht von innen, links Rückwand-Seite']);
  poly(sv, [[0, 0], [SL, 0], [SL, WH - ST], [F1, WH - ST], [F1, WH], [F0, WH], [F0, WH - ST], [0, WH - ST]]);
  line(sv, F0, WH - ST, F1, WH - ST, 't');
  dim(sv, [0, WH], [SL, WH], 'h', 12);
  dim(sv, [0, WH], [F0, WH], 'h', 6);
  dim(sv, [F0, WH], [F1, WH], 'h', 6);
  dim(sv, [F1, WH], [SL, WH], 'h', 6);
  dim(sv, [SL, 0], [SL, WH], 'v', 8);
  dim(sv, [0, 0], [0, WH - ST], 'v', -6, { text: fmt(WH - ST) });

  const x = view(s, 60, 282, 1, [30, 212, 'Schnitt Flansch mit Deckel (1:1)']);
  const Y0 = 36, LT = Y0 + g.GASKET + LID.t;
  angle(x, 0, 0, Y0, FL.w, ST);
  rect(x, 0, Y0 + g.GASKET, FL.w + 12, LT);
  rect(x, FL.seal - 5, Y0, FL.seal + 5, Y0 + g.GASKET);
  line(x, FL.screw, Y0 - 10, FL.screw, LT + 4, 'c');
  rect(x, FL.screw - 3.5, Y0 - 10, FL.screw + 3.5, Y0 - ST, 'h');
  dim(x, [0, LT], [FL.screw, LT], 'h', 6);
  dim(x, [0, LT], [FL.seal, LT], 'h', 12);
  dim(x, [0, LT], [FL.w, LT], 'h', 18);
  dim(x, [FL.w + 12, Y0], [FL.w + 12, Y0 + g.GASKET], 'v', 6, { text: `${g.GASKET}` });
  dim(x, [FL.w + 12, Y0 + g.GASKET], [FL.w + 12, LT], 'v', 14);
  label(s, 60 + FL.w + 30, 282 - 40, 'Dichtband 10 × 5, gequetscht ≈ 3 (Vorschlag)', { size: 2.5 });
  label(s, 60 + FL.w + 30, 282 - 34, 'Einnietmutter M5 A2, Schraube M5 A2 + Scheibe', { size: 2.5 });
  label(s, 60 + FL.w + 30, 282 - 28, 'außen ←    → innen', { size: 2.5 });
}

document.title = 'Buggy – Zylinderbox Zeichnungssatz (Entwurf)';
window.drawingsReady = true;
