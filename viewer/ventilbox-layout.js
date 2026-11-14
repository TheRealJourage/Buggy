// Electronics case mounting plate layout (proposal 2026-11-11). Units mm, origin = plate corner, x right, y down in 2D.
// Shared by ventilbox.html (2D) and ventilbox3d.html (3D).
export const M = 10, GAP = 20, GLAND = 25, CABLE_Z = 25;


// Since 2026-11-11 the valves sit in the cylinder box (variant B) → the case holds only the electronics, two columns.
const C2 = M + 113 + 15;
export const electronics = [
  { id: 'nucleo', x: M, y: M, w: 105, h: 110, z: 45, label: 'Nucleo-F091RC auf Träger', sub: 'H ~45 (Hypothese)', hyp: true },
  { id: 'can', x: M + 107, y: M + 5, w: 16, h: 15, z: 10, label: 'CAN', sub: 'SN65HVD230 16,1×14,5', hyp: false, small: true },
  { id: 'pc817', x: M, y: M + 125, w: 113, h: 45, z: 20, label: 'PC817 8 Kanäle', sub: 'Maß offen (bestellt)', hyp: true },
  { id: 'lr', x: M, y: M + 180, w: 113, h: 60, z: 20, label: '6 × LR7843 + 1N4007', sub: 'Maß offen (bestellt)', hyp: true },
  { id: 'hwl', x: C2, y: M, w: 52, h: 52, z: 40, label: 'HW-039 L', sub: '50×51,5, H 40 (gemessen)', hyp: false },
  { id: 'hwr', x: C2 + 61, y: M, w: 52, h: 52, z: 40, label: 'HW-039 R', sub: '', hyp: false },
  { id: 'term', x: C2, y: M + 65, w: 113, h: 25, z: 25, label: 'Klemmen 12 V / Masse-Stern', sub: 'WAGO, Platz geschätzt', hyp: true },
];

export function layout() {
  const all = electronics;
  const W = Math.max(...all.map(b => b.x + b.w)) + M;
  const H = Math.max(...all.map(b => b.y + b.h)) + M;
  const zMax = Math.max(...all.map(e => e.z));
  const innerZ = Math.max(zMax + CABLE_Z, 70);
  return { all, W, H, zMax, innerZ, inner: [W + GLAND, H + GLAND, innerZ] };
}

// Minimal case (proposal 2026-11-11): same parts and placeholder sizes, packed in two columns,
// CAN transceiver on the Nucleo carrier, glands only on the three walls that need them.
export const WALL_CLEAR = 15; // extra inside clearance on a gland wall: gland nut + cable bend (hypothesis)
export const WALL_T = 1.5;    // sheet thickness (hypothesis, stainless like the relay box)
const B = M + 105 + 10;
export const minimalParts = [
  { id: 'nucleo', x: M, y: M, w: 105, h: 110, z: 45, label: 'Nucleo-F091RC auf Träger', sub: 'H ~45 (Hypothese)', hyp: true },
  { id: 'can', x: M + 88, y: M + 3, w: 16, h: 15, z: 10, label: 'CAN', sub: 'SN65HVD230 16,1×14,5, auf dem Träger', hyp: false, onCarrier: true },
  { id: 'term', x: M, y: M + 120, w: 105, h: 25, z: 25, label: 'Klemmen 12 V / Masse-Stern', sub: 'WAGO, Platz geschätzt', hyp: true },
  { id: 'hwl', x: B, y: M, w: 52, h: 52, z: 40, label: 'HW-039 L', sub: '50×51,5, H 40 (gemessen)', hyp: false },
  { id: 'hwr', x: B + 61, y: M, w: 52, h: 52, z: 40, label: 'HW-039 R', sub: '', hyp: false },
  { id: 'pc817', x: B, y: M + 62, w: 113, h: 45, z: 20, label: 'PC817 8 Kanäle (4 belegt)', sub: 'Maß offen (bestellt)', hyp: true },
  { id: 'lr', x: B, y: M + 117, w: 113, h: 60, z: 20, label: '6 × LR7843 + 1N4007', sub: 'Maß offen (bestellt)', hyp: true },
];
// wall: top = y 0 side, bottom = y H side, left = x 0 side; pos along the wall; to = part id the cable runs to
export const minimalGlands = [
  { wall: 'top', pos: B + 26, size: 16, label: 'Motor L', to: 'hwl' },
  { wall: 'top', pos: B + 56.5, size: 20, label: '12 V EPB + Masse', to: 'term' },
  { wall: 'top', pos: B + 87, size: 16, label: 'Motor R', to: 'hwr' },
  { wall: 'left', pos: M + 35, size: 16, label: 'CAN-Bündel', to: 'can' },
  { wall: 'left', pos: M + 132, size: 16, label: '12 V Steuerung', to: 'term' },
  // clutch S1/S2 go to the front controller and arrive via CAN (decided 2026-11-11) → no clutch gland
  { wall: 'bottom', pos: B + 30, size: 16, label: 'Sensorkabel', to: 'pc817' },
  { wall: 'bottom', pos: B + 83, size: 16, label: 'Spulenkabel', to: 'lr' },
];

export function minimalLayout() {
  const all = minimalParts;
  const W = Math.max(...all.map(b => b.x + b.w)) + M;
  const H = Math.max(...all.map(b => b.y + b.h)) + M;
  const zMax = Math.max(...all.map(e => e.z));
  const innerZ = Math.max(zMax + CABLE_Z, 70);
  const walls = new Set(minimalGlands.map(g => g.wall));
  const x0 = walls.has('left') ? -WALL_CLEAR : 0, x1 = W + (walls.has('right') ? WALL_CLEAR : 0);
  const y0 = walls.has('top') ? -WALL_CLEAR : 0, y1 = H + (walls.has('bottom') ? WALL_CLEAR : 0);
  const inner = [x1 - x0, y1 - y0, innerZ];
  const outer = inner.map(v => v + 2 * WALL_T);
  return { all, W, H, zMax, innerZ, x0, y0, x1, y1, inner, outer, glands: minimalGlands };
}
