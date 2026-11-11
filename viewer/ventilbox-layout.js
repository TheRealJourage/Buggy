// Valve box mounting plate layout (proposal 2026-11-11). Units mm, origin = plate corner, x right, y down in 2D.
// Shared by ventilbox.html (2D) and ventilbox3d.html (3D).
export const M = 10, GAP = 20, GLAND = 25, CABLE_Z = 25;
export const ELX = M + 180 + GAP;

// Airtec M-04-53x-HN (datasheet M-04 p. 4-05): 180 incl. sockets, 161 body, 64 incl. coil+socket, 32 body, 22 wide
export const VALVE = { L: 180, body: 161, H: 64, bodyH: 32, W: 22 };
export const VALVE_PITCH = 115;
export const FITTING_ZONE = 35; // silencers + supply on 1/3/5 side, estimate

export const electronics = [
  { id: 'nucleo', x: ELX, y: M, w: 105, h: 110, z: 45, label: 'Nucleo-F091RC auf Träger', sub: 'H ~45 (Hypothese)', hyp: true },
  { id: 'can', x: ELX + 107, y: M + 5, w: 16, h: 15, z: 10, label: 'CAN', sub: 'SN65HVD230 16,1×14,5', hyp: false, small: true },
  { id: 'hwl', x: ELX, y: M + 125, w: 52, h: 52, z: 40, label: 'HW-039 L', sub: '50×51,5, H 40 (gemessen)', hyp: false },
  { id: 'hwr', x: ELX + 61, y: M + 125, w: 52, h: 52, z: 40, label: 'HW-039 R', sub: '', hyp: false },
  { id: 'pc817', x: ELX, y: M + 190, w: 113, h: 45, z: 20, label: 'PC817 8 Kanäle', sub: 'Maß offen (bestellt)', hyp: true },
  { id: 'lr', x: ELX, y: M + 245, w: 113, h: 60, z: 20, label: '6 × LR7843 + 1N4007', sub: 'Maß offen (bestellt)', hyp: true },
  { id: 'term', x: ELX, y: M + 315, w: 113, h: 25, z: 25, label: 'Klemmen 12 V / Masse-Stern', sub: 'WAGO, Platz geschätzt', hyp: true },
];

export function valvesFlat() {
  const out = [];
  ['G', 'A', 'B'].forEach((n, i) => {
    const y = M + i * VALVE_PITCH;
    out.push({ id: 'valve', name: n, x: M, y, w: VALVE.L, h: VALVE.H, z: VALVE.W, label: `Ventil ${n} (5/3)`, sub: '180×64, flach, H 22', hyp: false });
    out.push({ id: 'fit', x: M + 10, y: y + VALVE.H, w: 160, h: FITTING_ZONE, z: VALVE.W, label: '1/3/5: Schalldämpfer + Versorgung', sub: '~35 geschätzt', hyp: true });
  });
  return { items: out, z: VALVE.W };
}

// R-281/3 (datasheet p. 4-06): C = 94 incl. brackets, pitch 22.2, valve top 104 above bar axis, bar axis 32 above bracket foot
export function valvesManifold() {
  return { items: [
    { id: 'manifold', x: M, y: M, w: VALVE.L, h: 94, z: 136, label: '3 × Ventil stehend auf R-281/3', sub: '180×94, H ~136 (104 + 32)', hyp: false },
    { id: 'fit', x: M + 10, y: M + 94, w: 160, h: 30, z: 0, label: 'Schläuche 2/4 oben, Abluft 3/5 unten', sub: 'Platz geschätzt', hyp: true },
  ], z: 136 };
}

export function layout(variant) {
  const valves = variant === 'flat' ? valvesFlat() : valvesManifold();
  const all = [...valves.items, ...electronics];
  const W = Math.max(...all.map(b => b.x + b.w)) + M;
  const H = Math.max(...all.map(b => b.y + b.h)) + M;
  const zMax = Math.max(valves.z, ...electronics.map(e => e.z));
  const innerZ = Math.max(zMax + CABLE_Z, 70);
  return { valves, all, W, H, zMax, innerZ, inner: [W + GLAND, H + GLAND, innerZ] };
}
