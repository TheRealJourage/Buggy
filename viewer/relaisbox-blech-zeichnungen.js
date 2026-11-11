// Relay box sheet-metal drawings (3 x A3): flat patterns valid for stainless 1.5 and alu 2, cutting plan.
import * as g from './relaisbox-blech.js';
import { sheet, view, poly, rect, line, circle, dim, leader, label, table, el, fmt } from './drafting.js';

const root = document.getElementById('sheets');
const N = 3, DATE = '2026-11-11';
const BASE_NOTES = [
  'Maße in mm · Allgemeintoleranzen ISO 2768-m · Kanten entgratet',
  '!Orange = Prüfpunkt: vor dem Zuschnitt mit dem Kollegen klären',
];
const PROJECT = 'Buggy – Relaisbox (Beifahrersitz) · Blechteile';
function sh(k, info) {
  const s = sheet(root, { sheet: `${k} / ${N}`, date: DATE, rev: 'A', material: 'Edelstahl 1.4301 1,5 (festgelegt) · gilt auch für AlMg3 2', ...info, notes: [...(info.notes || []), ...BASE_NOTES] });
  for (const t of s.querySelectorAll('text')) if (t.textContent.startsWith('Buggy – pneumatische')) t.textContent = PROJECT;
  return s;
}
const { TUB, U, BD, GAP, RELIEF_D, NOTCH } = g;
const cases = Object.entries(g.MATERIALS).flatMap(([k, m]) => m.r.map((r) => ({ m, ...g.result(k, r) })));

// ---------- sheet 1: tub flat pattern
{
  const s = sh(1, {
    title: 'Pos 1 Wanne – Abwicklung', scale: '1:5', no: 'RB-01', stock: `Blech ${fmt(TUB.flatL)} × ${fmt(TUB.flatW)} · 1 Stk`,
    notes: [
      `Abwicklung nach DIN 6935, gemeinsamer Ausgleichswert v = −${fmt(BD)} je 90°-Biegung`,
      `(Mittel aus Edelstahl 1,5 r1,5–2 und Alu 2 r2–3). Wandmaß ${fmt(TUB.wall)} mit dem größten v gerechnet,`,
      `damit die Bauhöhe 115 (festgelegt) bei keinem Werkstoff überschritten wird.`,
      'B1–B4: 90° hoch · F1–F4: 90° nach innen (Flansch zeigt zur Boxmitte).',
      `Ecken: Wandkanten ${fmt(GAP)} vor der Biegelinie, Freistich Ø${RELIEF_D}, Flansch-Gehrung 45°;`,
      'Ecken und Gehrungen WIG dicht durchschweißen (Edelstahl: Wurzel mit Formiergas, Kenntnisstand).',
      '!Probebiegung: Streifen 100 breit auf der Kantbank des Kollegen, v messen; weicht v um',
      `!mehr als 0,5 von ${fmt(BD)} ab, Wandmaß ${fmt(TUB.wall)} neu rechnen (Datei relaisbox-blech.js).`,
      '!Löcher für Einnietmuttern M5 erst nach dem Biegen bohren (9 von außen, zu nah an der Biegung).',
      '!Kabelverschraubungen, Druckausgleich, Laschen: Lage offen, Löcher vor dem Biegen ergänzen.',
    ],
  });
  const v = view(s, 125, 122, 0.2, [25, 22, 'Abwicklung Pos 1 (Ansicht auf die spätere Innenseite)']);
  const { a, b, wall: w, flange: f } = TUB;
  poly(v, g.tubOutline(24), 'k');
  for (const bl of g.tubBendLines()) line(v, ...bl.p[0], ...bl.p[1], 'p');
  // bend labels
  const bt = (u, wv, t, rot = 0) => el(v.g, 'text', { x: v.X(u), y: v.Y(wv), 'font-size': 2.4, 'text-anchor': 'middle', transform: rot ? `rotate(${rot} ${v.X(u)} ${v.Y(wv)})` : undefined }, t);
  bt(a + 4, 60, 'B1 90° hoch', -90); bt(-a - 2, 60, 'B2 90° hoch', -90);
  bt(-120, b + 2, 'B3 90° hoch'); bt(-120, -b - 9, 'B4 90° hoch');
  bt(a + w + 4, -110, 'F1 90° innen', -90); bt(-a - w - 2, -110, 'F2 90° innen', -90);
  bt(120, b + w + 2, 'F3 90° innen'); bt(120, -b - w - 9, 'F4 90° innen');
  bt(0, 0, 'Boden'); bt(a + w / 2, 6, 'Stirnwand vorn', -90); bt(-a - w / 2, 6, 'Stirnwand hinten', -90);
  bt(0, b + w / 2, 'Seitenwand links (Lastseite)'); bt(0, -b - w / 2, 'Seitenwand rechts (Steuerseite)');
  // dimension chains
  const top = TUB.flatW / 2, xs = [-(a + w + f), -(a + w), -a, a, a + w, a + w + f];
  for (let i = 0; i < xs.length - 1; i++) dim(v, [xs[i], top], [xs[i + 1], top], 'h', 7, { size: 2.6 });
  dim(v, [xs[0], top], [xs[5], top], 'h', 15);
  const left = -TUB.flatL / 2, ys = [-(b + w + f), -(b + w), -b, b, b + w, b + w + f];
  for (let i = 0; i < ys.length - 1; i++) dim(v, [left, ys[i]], [left, ys[i + 1]], 'v', -7, { size: 2.6 });
  dim(v, [left, ys[0]], [left, ys[5]], 'v', -15);
  leader(v, [a + w + f, NOTCH.w / 2], 6, 14, [`Ausschnitt ${fmt(NOTCH.w)} × ${fmt(f + NOTCH.extra)}`, 'für den Steg Pos 2'], { size: 2.5 });
  leader(v, [-a - w - f, -NOTCH.w / 2], -6, 16, [`Ausschnitt ${fmt(NOTCH.w)} × ${fmt(f + NOTCH.extra)}`], { size: 2.5 });
  leader(v, [a - RELIEF_D / 2 * 0.7, b - RELIEF_D / 2 * 0.7], 10, -24, [`Freistich Ø${RELIEF_D}, Mitte auf`, 'Kreuzung der Biegelinien', `Wandkanten ${fmt(GAP)} zurück`], { size: 2.5 });
  leader(v, [a + w + f / 2, b - GAP - f / 2], 8, -8, ['Gehrung 45°'], { size: 2.5 });

  label(s, 240, 22, 'Ergebnis nach dem Biegen (gerechnet, außen)', { bold: true, size: 3.2 });
  table(s, 240, 25, [['Werkstoff', 52], ['ri', 12], ['v', 12], ['L × B', 34], ['H', 16], ['Flansch', 18]],
    cases.map((c) => [c.m.name, fmt(c.r), fmt(Math.round(c.bd * 100) / 100), `${fmt(Math.round(c.L * 10) / 10)} × ${fmt(Math.round(c.W * 10) / 10)}`, fmt(Math.round(c.H * 10) / 10), fmt(Math.round(c.flange * 10) / 10)]), { size: 2.6 });
  label(s, 240, 61, 'Soll: 500 × 430, H max. 115 (festgelegt 2026-11-01), Flansch 25', { size: 2.6 });
  label(s, 240, 65, 'ri = Innenradius an der Kantbank (Werkzeug des Kollegen, offen).', { size: 2.6 });

  label(s, 240, 75, 'Biegefolge (Vorschlag)', { bold: true, size: 3.2 });
  ['1. F1–F4: alle Flansche 90°, solange das Blech flach ist', '2. B3/B4: lange Seitenwände hoch', '3. B1/B2: Stirnwände hoch (Oberwange muss zwischen die', '    Seitenwände passen: Innenmaß ca. 426 → Segment-Oberwange)',
    '!Prüfen: kann die Kantbank des Kollegen Schritt 3 mit', '!   nach innen stehendem Flansch (Schwenkbiege-/Segmentbank)?', '4. Ecken heften, Maß 500 × 430 prüfen, dann dicht schweißen']
    .forEach((t, k) => label(s, 240, 80 + k * 4, t.replace(/^!/, ''), { size: 2.6, color: t.startsWith('!') ? '#e06a00' : undefined }));
  window.drawingsReady = false;
}

// ---------- sheet 2: U web + assembly section
{
  const s = sh(2, {
    title: 'Pos 2 U-Steg + Zusammenbau', scale: '1:3 / 1:2 / 1:4', no: 'RB-02', stock: `Blech ${fmt(U.L)} × ${fmt(U.flatW)} · 1 Stk`,
    notes: [
      'Pos 2 ersetzt Mittelsteg + Mittelwand aus Entwurf v3 (Vorschlag Claude, 2026-11-11):',
      `umgedrehtes U, Oberseite ${U.web} = Dichtfläche beider Deckel, Schenkel ${U.leg} hängen frei,`,
      'Kabel laufen unten durch die 3 Ausschnitte 80 × 40 (Kante mit Kantenschutzprofil).',
      `Länge ${U.L}: ca. 0,7–1,1 Luft zu den Stirnwänden, Spalt beim Schweißen schließen.`,
      'Zusammenbau: Wanne kopfüber auf eine ebene Platte legen, Pos 2 einlegen →',
      'Flansch und Steg-Oberseite liegen bündig. Heften, dann Steg ↔ Flansch-Ausschnitt dicht',
      'schweißen und oben plan schleifen (Dichtung liegt darauf). Schenkel an die Stirnwände.',
      '!Steg-Löcher M5 (9 neben der Mitte) nach dem Einschweißen mit dem Flansch bohren.',
    ],
  });
  // flat U
  const v = view(s, 125, 122, 1 / 3, [25, 18, 'Abwicklung Pos 2 (1:3)']);
  poly(v, g.uOutline(), 'k');
  for (const [p1, p2] of g.uBendLines()) line(v, ...p1, ...p2, 'p');
  const h = U.L / 2;
  dim(v, [-h, 0], [h, 0], 'h', -12);
  dim(v, [h, 0], [h, U.legFlat], 'v', 8, { size: 2.6 });
  dim(v, [h, U.legFlat], [h, U.legFlat + U.webFlat], 'v', 8, { size: 2.6 });
  dim(v, [h, U.legFlat + U.webFlat], [h, U.flatW], 'v', 8, { size: 2.6 });
  dim(v, [h, 0], [h, U.flatW], 'v', 17);
  const nx = U.notch.x;
  dim(v, [-h, U.flatW], [nx[0] - U.notch.w / 2, U.flatW], 'h', 7, { size: 2.6 });
  dim(v, [nx[0] - U.notch.w / 2, U.flatW], [nx[0] + U.notch.w / 2, U.flatW], 'h', 7, { size: 2.6 });
  dim(v, [nx[0], U.flatW], [nx[1], U.flatW], 'h', 13, { size: 2.6 });
  dim(v, [nx[1], U.flatW], [nx[2], U.flatW], 'h', 13, { size: 2.6 });
  dim(v, [nx[2] + U.notch.w / 2, U.flatW], [nx[2] + U.notch.w / 2, U.flatW - U.notch.h], 'v', 6, { size: 2.6 });
  leader(v, [nx[1] + U.notch.w / 2 - 2, U.notch.h - 1], 10, 8, [`3 × Ausschnitt ${U.notch.w} × ${U.notch.h}, R${U.notch.r}`, 'je Schenkel'], { size: 2.5 });
  const bt = (u, wv, t) => el(v.g, 'text', { x: v.X(u), y: v.Y(wv) - 0.8, 'font-size': 2.4, 'text-anchor': 'middle' }, t);
  bt(-200, U.legFlat, 'S1 90°'); bt(-200, U.legFlat + U.webFlat, 'S2 90° (gleiche Richtung)');
  bt(-60, U.legFlat + U.webFlat / 2 - 1.5, 'Steg = Dichtfläche');

  // section of U at 1:1 (stainless 1.5 shown)
  const t = g.MATERIALS.va.t, vs = view(s, 300, 40, 0.5, [262, 22, 'Schnitt Pos 2 (1:2, Edelstahl 1,5)']);
  const W2 = U.web / 2;
  poly(vs, [[-W2, 0], [-W2, -U.leg], [-W2 + t, -U.leg], [-W2 + t, -t], [W2 - t, -t], [W2 - t, -U.leg], [W2, -U.leg], [W2, 0]], 'k');
  dim(vs, [-W2, 0], [W2, 0], 'h', 6);
  dim(vs, [W2, 0], [W2, -U.leg], 'v', 8);
  label(s, 262, 100, `Abwicklung gerechnet mit v = −${fmt(BD)}: Steg ${fmt(U.webFlat)} + 2 × ${fmt(U.legFlat)}`, { size: 2.5 });

  // assembly cross-section at 1:4 (y-z), stainless
  const va = cases.find((c) => c.key === 'va' && c.r === 2), alu = cases.find((c) => c.key === 'alu' && c.r === 3);
  const k = 0.25, va4 = view(s, 130, 230, k, [25, 150, 'Zusammenbau, Querschnitt (1:4, Edelstahl 1,5, ri 2)']);
  const Wo = va.W / 2, Ho = va.H, F = va.flange;
  poly(va4, [[-Wo + F, Ho], [-Wo, Ho], [-Wo, 0], [Wo, 0], [Wo, Ho], [Wo - F, Ho], [Wo - F, Ho - t], [Wo - t, Ho - t], [Wo - t, t], [-Wo + t, t], [-Wo + t, Ho - t], [-Wo + F, Ho - t]], 'k');
  poly(va4, [[-W2, Ho], [-W2, Ho - va.uLeg], [-W2 + t, Ho - va.uLeg], [-W2 + t, Ho - t], [W2 - t, Ho - t], [W2 - t, Ho - va.uLeg], [W2, Ho - va.uLeg], [W2, Ho]], 'k');
  rect(va4, -Wo, Ho + 3, -1, Ho + 8, 'h'); rect(va4, 1, Ho + 3, Wo, Ho + 8, 'h');
  label(s, va4.X(-Wo / 2), va4.Y(Ho + 10), 'Deckel Lastseite (PC 5)', { size: 2.4, anchor: 'middle' });
  label(s, va4.X(Wo / 2), va4.Y(Ho + 10), 'Deckel Steuerseite (PC 5)', { size: 2.4, anchor: 'middle' });
  dim(va4, [Wo, 0], [Wo, Ho], 'v', 8, { text: `${fmt(Math.round(va.H * 10) / 10)} (Alu ${fmt(Math.round(alu.H * 10) / 10)}) ≤ 115` , size: 2.5 });
  dim(va4, [-Wo, 0], [Wo, 0], 'h', -8, { text: `${fmt(Math.round(va.W * 10) / 10)} (Soll 430)`, size: 2.6 });
  dim(va4, [-Wo, Ho], [-Wo + F, Ho], 'h', 14, { size: 2.4, text: `${fmt(Math.round(F * 10) / 10)}` });
  dim(va4, [-W2, Ho], [W2, Ho], 'h', 14, { size: 2.4 });
  dim(va4, [-W2, t], [-W2, Ho - va.uLeg], 'v', -5, { size: 2.4, text: `${fmt(Math.round(va.floorGap * 10) / 10)}` });
  leader(va4, [-Wo / 2, t], -6, 10, 'Lastseite (links in Fahrtrichtung)', { size: 2.4, dot: true });
  leader(va4, [Wo / 2, t], 6, 10, 'Steuerseite', { size: 2.4, dot: true });
  leader(va4, [W2, Ho - 3], 34, -16, ['Schweißnaht Steg ↔ Flansch-Ausschnitt', 'oben dicht, plan geschliffen'], { size: 2.4 });
}

// ---------- sheet 3: cutting plan
{
  const plan = g.cutPlan(), { sheet: SH } = plan;
  const area = (g.polyArea(g.tubOutline()) + g.polyArea(g.uOutline())) * 1e-6;
  const s = sh(3, {
    title: 'Schnittplan', scale: '1:5 / 1:20', no: 'RB-03', stock: `Zuschnitt ${SH.w} × ${SH.h} · 1 Tafel`,
    notes: [
      `Beide Teile aus einem Zuschnitt ${SH.w} × ${SH.h} (10 Rand, 10 Abstand), Walzrichtung egal.`,
      'Schneiden: Laser oder Wasserstrahl mit den DXF (Layer CUT = Kontur, BEND = Biegelinie,',
      'nicht schneiden; Biegelinien als Körnung/Markierung am Rand). Alternativ Tafelschere +',
      'Stichsäge/Flex nach den Maßen auf Blatt 1 und 2.',
      'Edelstahl: Schutzfolie draufbehalten, mit Edelstahl-Werkzeug schneiden (Fremdrost, Kenntnisstand).',
      `Ecken der Wanne (4 × ${fmt(TUB.ext)} × ${fmt(TUB.ext)}) und der Rest reichen für Laschen und Halter.`,
    ],
  });
  const v = view(s, 35, 215, 0.2, [25, 22, `Zuschnitt ${SH.w} × ${SH.h} (1:5)`]);
  rect(v, 0, 0, SH.w, SH.h, 'k');
  const [P1, P2] = plan.parts;
  poly(v, g.tubOutline(16).map(([x, y]) => [P1.x + P1.w / 2 + y, P1.y + P1.h / 2 + x]), 'k');
  for (const bl of g.tubBendLines()) line(v, P1.x + P1.w / 2 + bl.p[0][1], P1.y + P1.h / 2 + bl.p[0][0], P1.x + P1.w / 2 + bl.p[1][1], P1.y + P1.h / 2 + bl.p[1][0], 'p');
  poly(v, g.uOutline().map(([u, w]) => [P2.x + w, P2.y + u + U.L / 2]), 'k');
  for (const [p1, p2] of g.uBendLines()) line(v, P2.x + p1[1], P2.y + p1[0] + U.L / 2, P2.x + p2[1], P2.y + p2[0] + U.L / 2, 'p');
  label(s, v.X(P1.x + P1.w / 2), v.Y(P1.y + P1.h / 2), 'Pos 1 Wanne', { size: 3.5, anchor: 'middle', bold: true });
  label(s, v.X(P2.x + P2.w / 2), v.Y(P2.y + P2.h / 2), 'Pos 2', { size: 3.5, anchor: 'middle', bold: true });
  label(s, v.X(P2.x + P2.w / 2), v.Y(P2.y + P2.h / 2) + 4.5, 'U-Steg', { size: 3, anchor: 'middle' });
  label(s, v.X(P2.x + P2.w / 2), v.Y(P2.h + 2 * g.KERF + 100), 'Rest', { size: 3, anchor: 'middle' });
  dim(v, [0, SH.h], [SH.w, SH.h], 'h', 8);
  dim(v, [SH.w, 0], [SH.w, SH.h], 'v', 8);
  dim(v, [0, 0], [P1.x, 0], 'h', -6, { size: 2.4 });
  dim(v, [P1.x + P1.w, 0], [P2.x, 0], 'h', -6, { size: 2.4 });
  dim(v, [P2.x, 0], [P2.x + P2.w, 0], 'h', -12, { size: 2.4 });
  dim(v, [P1.x, 0], [P1.x + P1.w, 0], 'h', -12, { size: 2.4 });

  // alternative: standard sheet 1000 x 2000 at 1:20
  const vt = view(s, 250, 140, 0.05, [240, 22, 'Alternative: Normtafel 1000 × 2000 (1:20)']);
  rect(vt, 0, 0, 1000, 2000, 'k');
  rect(vt, 0, 0, SH.w, SH.h, 't');
  el(vt.g, 'rect', { x: vt.X(0), y: vt.Y(SH.h), width: SH.w * vt.k, height: SH.h * vt.k, fill: '#ddd' });
  label(s, vt.X(500), vt.Y(SH.h / 2) + 1, 'Pos 1 + 2', { size: 2.6, anchor: 'middle' });
  label(s, vt.X(500), vt.Y((2000 + SH.h) / 2), `Rest 1000 × ${2000 - SH.h}`, { size: 2.6, anchor: 'middle' });
  dim(vt, [0, 2000], [1000, 2000], 'h', 5, { size: 2.4 });
  dim(vt, [1000, 0], [1000, 2000], 'v', 5, { size: 2.4 });

  label(s, 330, 30, 'Bedarf (gerechnet)', { bold: true, size: 3.2 });
  const z = (SH.w * SH.h) * 1e-6;
  table(s, 330, 33, [['', 28], ['Edelstahl 1,5', 26], ['Alu 2', 20]], [
    ['Zuschnitt', `${fmt(Math.round(z * 100) / 100)} m²`, `${fmt(Math.round(z * 100) / 100)} m²`],
    ['Teile (Blech)', `${fmt(Math.round(area * 1000) / 1000)} m²`, `${fmt(Math.round(area * 1000) / 1000)} m²`],
    ['Gewicht Teile', `${fmt(Math.round(area * 1.5 * 7.9 * 10) / 10)} kg`, `${fmt(Math.round(area * 2 * 2.7 * 10) / 10)} kg`],
    ['Preis Zuschnitt*', `ca. ${Math.round(z * 165)} €`, `ca. ${Math.round(z * 127.5)} €`],
  ], { size: 2.6 });
  label(s, 330, 66, '* Hypothese: Shop-Preise Metallshopper 2026-11-11', { size: 2.3, color: '#e06a00' });
  label(s, 330, 69.5, '  (VA K320 165 €/m², Alu roh 127,50 €/m², inkl. MwSt.),', { size: 2.3, color: '#e06a00' });
  label(s, 330, 73, '  ohne Laser, Versand, Arbeit. Händler des Kollegen fragen.', { size: 2.3, color: '#e06a00' });
}

// drafting.js styles `text { fill: #000 }`, which beats the fill attribute: move colours into style
for (const t of root.querySelectorAll('text[fill]')) if (t.getAttribute('fill') !== '#000') t.style.fill = t.getAttribute('fill');
window.drawingsReady = true;
