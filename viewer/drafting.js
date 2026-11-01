// Minimal technical-drawing kit for A3 sheets (mm units, ISO 128 line types, ISO 129 dimensions,
// ISO 2553 weld symbols, title block after ISO 7200). Used by zeichnungen.js.
const NS = 'http://www.w3.org/2000/svg';
export const CHK = '#e06a00'; // dimensions still to be confirmed (Prüfmaß)

export const fmt = (v) => (Math.round(v * 100) / 100).toLocaleString('de-DE');

export function el(parent, tag, attrs = {}, text) {
  const e = document.createElementNS(NS, tag);
  for (const [k, v] of Object.entries(attrs)) if (v !== undefined) e.setAttribute(k, v);
  if (text !== undefined) e.textContent = text;
  parent.appendChild(e);
  return e;
}

const STYLE = `
  .k { stroke: #000; stroke-width: .5; fill: none; stroke-linecap: round; stroke-linejoin: round }
  .t { stroke: #000; stroke-width: .25; fill: none }
  .h { stroke: #000; stroke-width: .3; fill: none; stroke-dasharray: 2 1 }
  .c { stroke: #000; stroke-width: .2; fill: none; stroke-dasharray: 6 1 1 1 }
  .p { stroke: #555; stroke-width: .25; fill: none; stroke-dasharray: 6 1 1 1 1 1 }
  .fill { fill: #000; stroke: none }
  .chk { stroke: ${CHK}; fill: ${CHK} }
  text { font-family: 'DejaVu Sans', 'Liberation Sans', Arial, sans-serif; fill: #000 }
  .hatch { fill: url(#hatch); stroke: none }
`;

// ---------- sheet with border, title block and general notes
export function sheet(root, info) {
  const s = el(root, 'svg', { xmlns: NS, width: '420mm', height: '297mm', viewBox: '0 0 420 297', class: 'sheet' });
  el(s, 'style', {}, STYLE);
  const defs = el(s, 'defs');
  const pat = el(defs, 'pattern', { id: 'hatch', width: 2, height: 2, patternUnits: 'userSpaceOnUse', patternTransform: 'rotate(45)' });
  el(pat, 'line', { x1: 0, y1: 0, x2: 0, y2: 2, stroke: '#000', 'stroke-width': 0.18 });
  el(s, 'rect', { x: 0, y: 0, width: 420, height: 297, fill: '#fff' });
  el(s, 'rect', { x: 20, y: 10, width: 390, height: 277, class: 'k', 'stroke-width': 0.7 });
  for (const x of [105, 210, 315]) { el(s, 'line', { x1: x, y1: 5, x2: x, y2: 10, class: 't' }); el(s, 'line', { x1: x, y1: 287, x2: x, y2: 292, class: 't' }); }
  titleBlock(s, info);
  const top = info.notes ? notesBox(s, info.notes) : 247;
  if (info.draft !== false) el(s, 'text', { x: 230, y: top - 3, 'font-size': 4.2, fill: '#c00000', 'font-weight': 'bold' }, 'ENTWURF – NICHT ZUR FERTIGUNG');
  return s;
}

function field(s, x, y, w, h, label, value, opt = {}) {
  el(s, 'rect', { x, y, width: w, height: h, class: 't' });
  el(s, 'text', { x: x + 1, y: y + 2.6, 'font-size': 1.9, fill: '#444' }, label);
  if (value !== undefined) el(s, 'text', { x: x + (opt.center ? w / 2 : 1.5), y: y + h - (opt.big ? 2.6 : 1.6), 'font-size': opt.size || 3, 'text-anchor': opt.center ? 'middle' : 'start', 'font-weight': opt.bold ? 'bold' : undefined, fill: opt.color }, value);
}

function titleBlock(s, i) {
  const x = 230, y = 247;
  el(s, 'rect', { x, y, width: 180, height: 40, class: 'k', 'stroke-width': 0.7 });
  field(s, x, y, 180, 8, 'Projekt', 'Buggy – pneumatische Gangschaltung · Zylinderbox');
  field(s, x, y + 8, 100, 8, 'Werkstoff', i.material || '—', { size: 2.7 });
  field(s, x + 100, y + 8, 80, 8, 'Halbzeug · Menge', i.stock || '—', { size: 2.7 });
  field(s, x, y + 16, 120, 12, 'Benennung', i.title, { size: 5, bold: true, big: true });
  field(s, x + 120, y + 16, 30, 12, 'Maßstab', i.scale, { size: 4.5, center: true, big: true });
  field(s, x + 150, y + 16, 30, 12, 'Projektion');
  projectionSymbol(s, x + 157, y + 23.5);
  field(s, x, y + 28, 60, 12, 'Zeichnungs-Nr.', i.no, { size: 4.5, bold: true, big: true });
  field(s, x + 60, y + 28, 22, 12, 'Blatt', i.sheet, { size: 3.6, center: true, big: true });
  field(s, x + 82, y + 28, 30, 12, 'Datum', i.date || '2026-11-01', { size: 3, center: true, big: true });
  field(s, x + 112, y + 28, 46, 12, 'Status', i.draft === false ? 'Freigegeben' : 'ENTWURF', { size: 3.6, center: true, big: true, bold: true, color: i.draft === false ? '#000' : '#c00000' });
  field(s, x + 158, y + 28, 22, 12, 'Index', i.rev || 'A', { size: 3.6, center: true, big: true });
}

// ISO 5456-2 first-angle symbol
function projectionSymbol(s, x, y) {
  el(s, 'path', { d: `M${x} ${y - 2.2} L${x + 6} ${y - 3.5} L${x + 6} ${y + 3.5} L${x} ${y + 2.2} Z`, class: 't' });
  el(s, 'line', { x1: x - 1, y1: y, x2: x + 7, y2: y, class: 'c' });
  el(s, 'circle', { cx: x + 12.5, cy: y, r: 3.5, class: 't' });
  el(s, 'circle', { cx: x + 12.5, cy: y, r: 2.2, class: 't' });
  el(s, 'line', { x1: x + 8, y1: y, x2: x + 17, y2: y, class: 'c' });
}

function notesBox(s, lines) {
  const x = 230, h = 4 + lines.length * 3.6, y = 247 - h - 2;
  el(s, 'rect', { x, y, width: 180, height: h, class: 't' });
  lines.forEach((l, k) => el(s, 'text', { x: x + 2, y: y + 4.5 + k * 3.6, 'font-size': 2.5, fill: l.startsWith('!') ? CHK : '#000' }, l.replace(/^!/, '')));
  return y;
}

// ---------- view: part coordinates (u right, w up) -> sheet
export function view(s, ox, oy, scale, label) {
  const g = el(s, 'g');
  const v = { s, g, ox, oy, k: scale, X: (u) => ox + u * scale, Y: (w) => oy - w * scale };
  if (label) el(s, 'text', { x: label[0], y: label[1], 'font-size': 3.5, 'font-weight': 'bold' }, label[2]);
  return v;
}

export function poly(v, pts, cls = 'k', closed = true) {
  const d = pts.map(([u, w], i) => `${i ? 'L' : 'M'}${v.X(u).toFixed(3)} ${v.Y(w).toFixed(3)}`).join(' ') + (closed ? ' Z' : '');
  return el(v.g, 'path', { d, class: cls });
}
export const rect = (v, u0, w0, u1, w1, cls = 'k') => poly(v, [[u0, w0], [u1, w0], [u1, w1], [u0, w1]], cls);
export const line = (v, u1, w1, u2, w2, cls = 'k') => el(v.g, 'line', { x1: v.X(u1), y1: v.Y(w1), x2: v.X(u2), y2: v.Y(w2), class: cls });
export const circle = (v, u, w, r, cls = 'k') => el(v.g, 'circle', { cx: v.X(u), cy: v.Y(w), r: r * v.k, class: cls });

export function cross(v, u, w, r, ext = 2) {
  const R = r * v.k + ext;
  el(v.g, 'line', { x1: v.X(u) - R, y1: v.Y(w), x2: v.X(u) + R, y2: v.Y(w), class: 'c' });
  el(v.g, 'line', { x1: v.X(u), y1: v.Y(w) - R, x2: v.X(u), y2: v.Y(w) + R, class: 'c' });
}
export function hole(v, u, w, d, opt = {}) {
  circle(v, u, w, d / 2, opt.hidden ? 'h' : 'k');
  if (opt.cs) circle(v, u, w, opt.cs / 2, 'k');
  if (!opt.noCross) cross(v, u, w, Math.max(d, opt.cs || 0) / 2);
}
// tapped hole in plan view: core circle thick, 3/4 thin arc at nominal size (ISO 6410)
export function tapped(v, u, w, d, opt = {}) {
  const core = d * 0.84, R = (d / 2) * v.k, cx = v.X(u), cy = v.Y(w);
  circle(v, u, w, core / 2, 'k');
  el(v.g, 'path', { d: `M${cx + R} ${cy} A${R} ${R} 0 1 0 ${cx} ${cy + R}`, class: 't' });
  if (!opt.noCross) cross(v, u, w, d / 2);
}
export function slot(v, u, w, len, wd, along = 'w', cls = 'k') {
  const r = wd / 2, h = (len - wd) / 2, k = v.k;
  const [dx, dy] = along === 'w' ? [0, h * k] : [h * k, 0];
  const cx = v.X(u), cy = v.Y(w), R = r * k;
  const d = along === 'w'
    ? `M${cx - R} ${cy - dy} A${R} ${R} 0 0 1 ${cx + R} ${cy - dy} L${cx + R} ${cy + dy} A${R} ${R} 0 0 1 ${cx - R} ${cy + dy} Z`
    : `M${cx - dx} ${cy - R} L${cx + dx} ${cy - R} A${R} ${R} 0 0 1 ${cx + dx} ${cy + R} L${cx - dx} ${cy + R} A${R} ${R} 0 0 1 ${cx - dx} ${cy - R} Z`;
  el(v.g, 'path', { d, class: cls });
  cross(v, u, w, 0, along === 'w' ? 1.5 : 1.5);
}

function arrow(g, x, y, ang, col) {
  const L = 2.6, W = 0.45, c = Math.cos(ang), s = Math.sin(ang);
  el(g, 'path', { d: `M${x} ${y} L${x - L * c - W * s} ${y - L * s + W * c} L${x - L * c + W * s} ${y - L * s - W * c} Z`, class: 'fill', style: col ? `fill:${col}` : undefined });
}

// linear dimension between two part points; dir 'h' or 'v'; off = sheet mm from the first point
// (h: positive = above, v: positive = right). opt: text, tol, pre, chk
export function dim(v, p1, p2, dir, off, opt = {}) {
  const g = v.g, col = opt.chk ? CHK : '#000';
  const a = [v.X(p1[0]), v.Y(p1[1])], b = [v.X(p2[0]), v.Y(p2[1])];
  const val = dir === 'h' ? Math.abs(p2[0] - p1[0]) : Math.abs(p2[1] - p1[1]);
  const txt = opt.text ?? `${opt.pre || ''}${fmt(val)}${opt.tol ? ' ' + opt.tol : ''}${opt.chk ? '*' : ''}`;
  const ls = { stroke: col, 'stroke-width': 0.25 };
  if (dir === 'h') {
    const y = a[1] - off, x1 = Math.min(a[0], b[0]), x2 = Math.max(a[0], b[0]);
    for (const p of [a, b]) el(g, 'line', { x1: p[0], y1: p[1] + Math.sign(y - p[1]) * 1, x2: p[0], y2: y + Math.sign(y - p[1]) * 1.5, ...ls });
    el(g, 'line', { x1, y1: y, x2, y2: y, ...ls });
    const out = x2 - x1 < 7;
    arrow(g, x1, y, out ? 0 : Math.PI, col); arrow(g, x2, y, out ? Math.PI : 0, col);
    if (out) { el(g, 'line', { x1: x1 - 4, y1: y, x2: x1, y2: y, ...ls }); el(g, 'line', { x1: x2, y1: y, x2: x2 + 4, y2: y, ...ls }); }
    const tx = opt.tx !== undefined ? (x1 + x2) / 2 + opt.tx : (x1 + x2) / 2;
    el(g, 'text', { x: tx, y: y - 0.9, 'font-size': opt.size || 3.2, 'text-anchor': 'middle', fill: col }, txt);
  } else {
    const x = a[0] + off, y1 = Math.min(a[1], b[1]), y2 = Math.max(a[1], b[1]);
    for (const p of [a, b]) el(g, 'line', { x1: p[0] + Math.sign(x - p[0]) * 1, y1: p[1], x2: x + Math.sign(x - p[0]) * 1.5, y2: p[1], ...ls });
    el(g, 'line', { x1: x, y1, x2: x, y2, ...ls });
    const out = y2 - y1 < 7;
    arrow(g, x, y1, out ? Math.PI / 2 : -Math.PI / 2, col); arrow(g, x, y2, out ? -Math.PI / 2 : Math.PI / 2, col);
    if (out) { el(g, 'line', { x1: x, y1: y1 - 4, x2: x, y2: y1, ...ls }); el(g, 'line', { x1: x, y1: y2, x2: x, y2: y2 + 4, ...ls }); }
    const ty = (y1 + y2) / 2 + (opt.tx || 0);
    el(g, 'text', { x: x - 0.9, y: ty, 'font-size': opt.size || 3.2, 'text-anchor': 'middle', fill: col, transform: `rotate(-90 ${x - 0.9} ${ty})` }, txt);
  }
}

// leader with arrow at a part point, text at sheet offset (dx, dy)
export function leader(v, p, dx, dy, txt, opt = {}) {
  const g = v.g, col = opt.chk ? CHK : '#000';
  const x = v.X(p[0]), y = v.Y(p[1]), ex = x + dx, ey = y + dy, sh = dx >= 0 ? 3 : -3;
  el(g, 'line', { x1: x, y1: y, x2: ex, y2: ey, stroke: col, 'stroke-width': 0.25 });
  el(g, 'line', { x1: ex, y1: ey, x2: ex + sh, y2: ey, stroke: col, 'stroke-width': 0.25 });
  if (opt.dot) el(g, 'circle', { cx: x, cy: y, r: 0.6, class: 'fill', style: `fill:${col}` });
  else arrow(g, x, y, Math.atan2(y - ey, x - ex), col);
  const lines = Array.isArray(txt) ? txt : [txt];
  lines.forEach((t, k) => el(g, 'text', { x: ex + sh + (dx >= 0 ? 0.8 : -0.8), y: ey + 1.1 + k * 3.8, 'font-size': opt.size || 3.2, 'text-anchor': dx >= 0 ? 'start' : 'end', fill: col }, t));
}

// ISO 2553 fillet weld symbol: arrow at the joint, reference line to the right (or left), both = both sides
export function weld(v, p, dx, dy, a, opt = {}) {
  const g = v.g, x = v.X(p[0]), y = v.Y(p[1]), ex = x + dx, ey = y + dy, dir = opt.left ? -1 : 1, L = 22;
  el(g, 'line', { x1: x, y1: y, x2: ex, y2: ey, class: 't' });
  arrow(g, x, y, Math.atan2(y - ey, x - ex));
  el(g, 'line', { x1: ex, y1: ey, x2: ex + dir * L, y2: ey, class: 't' });
  const tx = ex + dir * 9;
  const tri = (sgn) => el(g, 'path', { d: `M${tx} ${ey} L${tx} ${ey - sgn * 3} L${tx + 3} ${ey} Z`, class: 't', 'stroke-width': 0.35 });
  tri(1); if (opt.both !== false) tri(-1);
  el(g, 'text', { x: tx - 0.8, y: ey - 0.8, 'font-size': 2.8, 'text-anchor': 'end' }, a);
  if (opt.both !== false) el(g, 'text', { x: tx - 0.8, y: ey + 3.2, 'font-size': 2.8, 'text-anchor': 'end' }, a);
  const te = ex + dir * L;
  el(g, 'path', { d: `M${te} ${ey} L${te + dir * 2.5} ${ey - 2} M${te} ${ey} L${te + dir * 2.5} ${ey + 2}`, class: 't' });
  el(g, 'text', { x: te + dir * 3.2, y: ey + 1, 'font-size': 2.6, 'text-anchor': dir > 0 ? 'start' : 'end' }, opt.tail || '141');
}

export function balloon(v, p, dx, dy, n) {
  const x = v.X(p[0]), y = v.Y(p[1]), ex = x + dx, ey = y + dy;
  const ang = Math.atan2(y - ey, x - ex);
  el(v.g, 'line', { x1: x, y1: y, x2: ex + 4 * Math.cos(ang), y2: ey + 4 * Math.sin(ang), class: 't' });
  el(v.g, 'circle', { cx: x, cy: y, r: 0.6, class: 'fill' });
  el(v.g, 'circle', { cx: ex, cy: ey, r: 4, class: 't', fill: '#fff' });
  el(v.g, 'text', { x: ex, y: ey + 1.4, 'font-size': 4, 'text-anchor': 'middle' }, n);
}

// datum feature symbol (ISO 5459): filled triangle on the surface, letter in a box
export function datum(v, p, dx, dy, letter) {
  const x = v.X(p[0]), y = v.Y(p[1]), ex = x + dx, ey = y + dy;
  const ang = Math.atan2(ey - y, ex - x), c = Math.cos(ang), s = Math.sin(ang);
  el(v.g, 'path', { d: `M${x - 1.6 * s} ${y + 1.6 * c} L${x + 1.6 * s} ${y - 1.6 * c} L${x + 2.6 * c} ${y + 2.6 * s} Z`, class: 'fill' });
  el(v.g, 'line', { x1: x + 2.6 * c, y1: y + 2.6 * s, x2: ex, y2: ey, class: 't' });
  el(v.g, 'rect', { x: ex - 2.5, y: ey - 2.5, width: 5, height: 5, class: 't', fill: '#fff' });
  el(v.g, 'text', { x: ex, y: ey + 1.3, 'font-size': 3.5, 'text-anchor': 'middle' }, letter);
}

// geometric tolerance frame, e.g. ['⊥', '0,3', 'A'] at sheet position with a leader to a part point
export function gtol(v, p, dx, dy, cells) {
  const x = v.X(p[0]), y = v.Y(p[1]), ex = x + dx, ey = y + dy;
  el(v.g, 'line', { x1: x, y1: y, x2: ex, y2: ey + 2.5, class: 't' });
  arrow(v.g, x, y, Math.atan2(y - ey - 2.5, x - ex));
  let cx = ex;
  for (const c of cells) {
    const w = Math.max(6, c.length * 2.4 + 2.5);
    el(v.g, 'rect', { x: cx, y: ey, width: w, height: 5, class: 't', fill: '#fff' });
    el(v.g, 'text', { x: cx + w / 2, y: ey + 3.8, 'font-size': 3.3, 'text-anchor': 'middle' }, c);
    cx += w;
  }
}

export function label(s, x, y, t, opt = {}) {
  return el(s, 'text', { x, y, 'font-size': opt.size || 3.2, 'text-anchor': opt.anchor || 'start', 'font-weight': opt.bold ? 'bold' : undefined, fill: opt.color }, t);
}

// simple table: cols = [[header, width], ...], rows = arrays of strings; returns height
export function table(s, x, y, cols, rows, opt = {}) {
  const rh = opt.rh || 5.5, fs = opt.size || 2.8;
  let cx = x;
  const W = cols.reduce((a, c) => a + c[1], 0);
  el(s, 'rect', { x, y, width: W, height: rh * (rows.length + 1), class: 'k', 'stroke-width': 0.5 });
  cols.forEach(([h, w]) => {
    el(s, 'text', { x: cx + 1.2, y: y + rh - 1.7, 'font-size': fs, 'font-weight': 'bold' }, h);
    cx += w;
    el(s, 'line', { x1: cx, y1: y, x2: cx, y2: y + rh * (rows.length + 1), class: 't' });
  });
  rows.forEach((r, i) => {
    const ry = y + rh * (i + 1);
    el(s, 'line', { x1: x, y1: ry, x2: x + W, y2: ry, class: 't', 'stroke-width': i ? 0.2 : 0.4 });
    let tx = x;
    r.forEach((c, j) => {
      const chk = String(c).endsWith('*');
      el(s, 'text', { x: tx + 1.2, y: ry + rh - 1.7, 'font-size': fs, fill: chk ? CHK : '#000' }, c);
      tx += cols[j][1];
    });
  });
  return rh * (rows.length + 1);
}
