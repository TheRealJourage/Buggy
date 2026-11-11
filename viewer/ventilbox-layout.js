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
