// Free space around the steering column (Halle 2026-11-05, Kabelbaum-Übersicht.md).
// Frame: u = mm from measuring line "vorne" towards the power-steering unit,
// y = mm lateral, + = left in driving direction, z = mm up from the bottom of the free space.
// The free space is 45 mm high (user 2026-11-06); the column runs through it in the middle.

export const MEASURED = {
  steelW: 30,
  front: { left: 290, right: 315 },
  back: { left: 190, right: 235 },
  lenLeft: 140,
  lenRight: 160,
  slabH: 45,
};

export const DEFAULT_PARAMS = {
  lineGap: 150,
  colDia: 50,
  colAxis: 45,
};

export function widths(u, p) {
  const t = Math.min(Math.max(u / p.lineGap, 0), 1);
  const m = MEASURED;
  return {
    left: m.front.left + (m.back.left - m.front.left) * t,
    right: m.front.right + (m.back.right - m.front.right) * t,
  };
}

export function colBottomZ(p) {
  return Math.max(0, p.colAxis - p.colDia / 2);
}

export function colTopZ(p) {
  return p.colAxis + p.colDia / 2;
}

export function corners(b) {
  const a = (b.tilt || 0) * Math.PI / 180;
  const c = Math.cos(a), s = Math.sin(a);
  const out = [];
  for (const du of [-b.l / 2, b.l / 2])
    for (const dy of [-b.w / 2, b.w / 2])
      for (const dz of [-b.h / 2, b.h / 2]) {
        out.push({
          u: b.u + du * c + dz * s,
          y: b.y + dy,
          z: b.z + b.h / 2 - du * s + dz * c,
        });
      }
  return out;
}

export function aabb(b) {
  const r = { u0: Infinity, u1: -Infinity, y0: Infinity, y1: -Infinity, z0: Infinity, z1: -Infinity };
  for (const k of corners(b)) {
    r.u0 = Math.min(r.u0, k.u); r.u1 = Math.max(r.u1, k.u);
    r.y0 = Math.min(r.y0, k.y); r.y1 = Math.max(r.y1, k.y);
    r.z0 = Math.min(r.z0, k.z); r.z1 = Math.max(r.z1, k.z);
  }
  return r;
}

function overlap1(a0, a1, b0, b1) {
  return Math.min(a1, b1) - Math.max(a0, b0) > 0.5;
}

export function overlaps(a, b) {
  const p = aabb(a), q = aabb(b);
  return overlap1(p.u0, p.u1, q.u0, q.u1) && overlap1(p.y0, p.y1, q.y0, q.y1) && overlap1(p.z0, p.z1, q.z0, q.z1);
}

// Column (with the flat steel on it) blocks the centre strip from its underside up to its top.
export function centreStrip(p) {
  const half = Math.max(MEASURED.steelW, p.colDia) / 2;
  return { y0: -half, y1: half, z0: colBottomZ(p), z1: colTopZ(p) };
}

export function check(b, p, others = []) {
  const m = MEASURED;
  const errors = [];
  for (const k of corners(b)) {
    const w = widths(k.u, p);
    const uMax = k.y > -m.steelW / 2 ? m.lenLeft : m.lenRight;
    if (k.z < -0.5) { errors.push('unter dem Freiraum (Querbalken/Fußraum)'); break; }
    if (k.u < -0.5) { errors.push('vor der Messlinie „vorne“'); break; }
    if (k.u > uMax + 0.5) { errors.push(`stößt an die Servolenkung (max. ${uMax} mm)`); break; }
    if (k.y > m.steelW / 2 + w.left + 0.5 || k.y < -m.steelW / 2 - w.right - 0.5) { errors.push('seitlich außerhalb'); break; }
  }
  const r = aabb(b), cs = centreStrip(p);
  if (overlap1(r.y0, r.y1, cs.y0, cs.y1) && overlap1(r.z0, r.z1, cs.z0, cs.z1)) errors.push('kollidiert mit der Lenksäule');
  for (const o of others) if (o.id !== b.id && overlaps(b, o)) errors.push(`überlappt „${o.name}“`);
  const info = [];
  if (overlap1(r.y0, r.y1, cs.y0, cs.y1) && r.z1 <= cs.z0 + 0.5) info.push('unter der Lenksäule (Kabelbereich)');
  const sight = Math.max(0, r.z1 - m.slabH);
  return { status: errors.length ? 'red' : sight > 0.5 ? 'yellow' : 'green', errors, info, sight: Math.round(sight) };
}

export const PRESETS = [
  { name: 'Front-Controller Nucleo-F446RE', l: 82.5, w: 70, h: 30, color: '#3b82f6', note: 'Platine 82,5 × 70 mm (Nucleo-64); Höhe mit Stiftleisten Hypothese' },
  { name: 'Pico (Rund-Displays)', l: 51, w: 21, h: 10, color: '#22c55e', note: 'Pico 51 × 21 mm; Höhe mit Steckern Hypothese' },
  { name: '7"-Display Waveshare DSI + Tacho-Pi hinten dran', l: 45, w: 193, h: 111, tilt: 20, color: '#a855f7', note: 'Pi mit Kühler sitzt direkt hinten am Display (User 2026-11-06). Front ca. 193 × 111 mm laut Händlern (Hypothese, Waveshare-Zeichnung prüfen); Tiefe 45 mm inkl. Pi + Kühler Hypothese' },
  { name: '3 Rund-Displays + SG90 (eingeklappt)', l: 50, w: 130, h: 25, color: '#f59e0b', note: 'je Platine 38 × 45,5 mm (Händler); Anordnung + Servo Hypothese' },
  { name: '5-V-Regler ≥ 1 A (Servo)', l: 45, w: 25, h: 15, color: '#94a3b8', note: 'Größe Hypothese' },
];

// z of a box is its bottom before tilting; u/y are its centre. tilt > 0 leans the top towards +u.
export function defaultLayout(p = DEFAULT_PARAMS) {
  const place = [
    { u: 50, y: 110, z: 0 },
    { u: 30, y: -80, z: 0 },
    { u: 55, y: 0, z: Math.max(MEASURED.slabH, colTopZ(p)) + 8 },
    { u: 120, y: -150, z: 0 },
    { u: 30, y: 200, z: 0 },
  ];
  return PRESETS.map((pr, i) => ({ id: i + 1, tilt: 0, ...pr, ...place[i] }));
}
