import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { createSim, GATE_X, GATE_TRAVEL, GATE_NAMES, A_STROKE, B_STROKE, SEQUENCE, STATES, CLUTCH_TRAVEL } from './sim.js';

// ---- geometry constants (mm), HM-20 per Airtec catalog: body = 95 + stroke, WF 24, Ø27 / rod Ø8
const CYL_R = 13.5;
const ROD_R = 4;
const WF = 24;
const G_BODY = 95 + GATE_TRAVEL;
const A_BODY = 95 + A_STROKE;
const B_BODY = 95 + B_STROKE;

// ---- renderer / scene
const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const scene = new THREE.Scene();
scene.background = new THREE.Color(0x202228);
scene.add(new THREE.HemisphereLight(0xffffff, 0x404050, 1.4));
const sun = new THREE.DirectionalLight(0xffffff, 2.0);
sun.position.set(400, 800, 500);
scene.add(sun);

const camera = new THREE.PerspectiveCamera(40, window.innerWidth / window.innerHeight, 1, 10000);
camera.position.set(250, 1050, 1450);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(250, 60, -40);
controls.update();

const grid = new THREE.GridHelper(1600, 32, 0x3a3d44, 0x2c2f35);
grid.position.set(100, 0, -100);
scene.add(grid);

// ---- materials
const mat = {
  alu: new THREE.MeshStandardMaterial({ color: 0xb8bcc4, metalness: 0.6, roughness: 0.35 }),
  steel: new THREE.MeshStandardMaterial({ color: 0xe8e8e8, metalness: 0.9, roughness: 0.2 }),
  dark: new THREE.MeshStandardMaterial({ color: 0x3a3d44, metalness: 0.3, roughness: 0.6 }),
  frame: new THREE.MeshStandardMaterial({ color: 0x4a4f58, metalness: 0.4, roughness: 0.6 }),
  carriage: new THREE.MeshStandardMaterial({ color: 0x5d7a99, metalness: 0.4, roughness: 0.5 }),
  valve: new THREE.MeshStandardMaterial({ color: 0x9aa3ad, metalness: 0.5, roughness: 0.4 }),
  blue: new THREE.MeshStandardMaterial({ color: 0x2f6fd6, metalness: 0.2, roughness: 0.5 }),
  tank: new THREE.MeshStandardMaterial({ color: 0xc8ccd2, metalness: 0.7, roughness: 0.3 }),
  comp: new THREE.MeshStandardMaterial({ color: 0x222222, metalness: 0.3, roughness: 0.6 }),
  red: new THREE.MeshStandardMaterial({ color: 0xb02020, metalness: 0.3, roughness: 0.5 }),
  plate: new THREE.MeshStandardMaterial({ color: 0x30343b, metalness: 0.2, roughness: 0.7, transparent: true, opacity: 0.85 }),
  slot: new THREE.MeshBasicMaterial({ color: 0x0b0c0e }),
  knob: new THREE.MeshStandardMaterial({ color: 0x111111, metalness: 0.2, roughness: 0.4 }),
  supplyHose: new THREE.MeshStandardMaterial({ color: 0x2f6fd6, roughness: 0.6 }),
};
const hoseMat = {
  press: new THREE.MeshStandardMaterial({ color: 0xe0402a, roughness: 0.5, emissive: 0x401008 }),
  vent: new THREE.MeshStandardMaterial({ color: 0x6c7078, roughness: 0.7 }),
};

const hoverables = [];
const labels = [];

function info(mesh, title, part, note) {
  mesh.userData.info = { title, part, note };
  hoverables.push(mesh);
  return mesh;
}

function box(w, h, d, material, x = 0, y = 0, z = 0) {
  const m = new THREE.Mesh(new THREE.BoxGeometry(w, h, d), material);
  m.position.set(x, y, z);
  return m;
}

// cylinder along an axis ('x' | 'z' | 'y'), from a to b on that axis
function cyl(r, from, to, material, axis, c1 = 0, c2 = 0, seg = 24) {
  const len = Math.abs(to - from);
  const m = new THREE.Mesh(new THREE.CylinderGeometry(r, r, len, seg), material);
  const mid = (from + to) / 2;
  if (axis === 'x') { m.rotation.z = Math.PI / 2; m.position.set(mid, c1, c2); }
  else if (axis === 'z') { m.rotation.x = Math.PI / 2; m.position.set(c1, c2, mid); }
  else m.position.set(c1, mid, c2);
  return m;
}

function label(text, x, y, z, size = 14, color = '#e8e8e8') {
  const c = document.createElement('canvas');
  const ctx = c.getContext('2d');
  ctx.font = 'bold 44px system-ui, sans-serif';
  const w = Math.ceil(ctx.measureText(text).width) + 24;
  c.width = w; c.height = 64;
  ctx.font = 'bold 44px system-ui, sans-serif';
  ctx.fillStyle = 'rgba(0,0,0,0.55)';
  ctx.fillRect(0, 0, w, 64);
  ctx.fillStyle = color;
  ctx.textBaseline = 'middle';
  ctx.fillText(text, 12, 34);
  const tex = new THREE.CanvasTexture(c);
  tex.colorSpace = THREE.SRGBColorSpace;
  const s = new THREE.Sprite(new THREE.SpriteMaterial({ map: tex, depthTest: false }));
  s.scale.set((w / 64) * size, size, 1);
  s.position.set(x, y, z);
  s.renderOrder = 10;
  labels.push(s);
  return s;
}

const V = (x, y, z) => new THREE.Vector3(x, y, z);
function tube(points, material, r = 3) {
  return new THREE.Mesh(new THREE.TubeGeometry(new THREE.CatmullRomCurve3(points), 32, r, 8), material);
}

// ---- minimal cylinder box (mm): the 2 mm end wall IS the cable abutment (clip holes like the
// original shift housing); every cylinder is placed backwards from that wall at full extension.
const WALL_X = 262;              // inner face of the end wall with the clip holes
const WALL_T = 2;
const FLOOR_T = 8;
// user notes: rigid cable end inside the box 70 mm (Wählseil) / 130 mm (Schaltseil), clip hole Ø 25 / Ø 15 mm
const RIGID = { G: 70, R: 130 };
const HOLE_D = { G: 25, R: 15 };
const EYE_CLEAR = 10;            // assumed gap eye -> rigid part at full extension
const CLEVIS = 22;               // rod end -> cable eye pin (RD-20 + Ø8 pin)
const RO20 = 57;                 // Airtec FK-20 compensating coupling A -> B
const FOOT_W = 54;               // RA-25 foot width = lane pitch driver
const GAP = 6;
const AX_Y = FLOOR_T + 40;       // common axis height (B on MGN12 carriage, A/G on RA-25 + spacer)
const CB_Z1 = 50;
const ROW_Z = CB_Z1 - WALL_T - GAP - FOOT_W / 2;
const G_Z = ROW_Z - FOOT_W - GAP;
const POTI_Z = G_Z - FOOT_W / 2 - GAP - 8;
const G_ROD_END0 = WALL_X - RIGID.G - EYE_CLEAR - CLEVIS - GATE_TRAVEL;            // G rod end at x = 0
const G_X0 = G_ROD_END0 - WF - G_BODY;
const B_ROD_END0 = WALL_X - RIGID.R - EYE_CLEAR - CLEVIS - A_STROKE - B_STROKE;    // B rod end at a = b = 0
const B_OFFSET = RO20;
const A_ROD_END0 = B_ROD_END0 - WF - B_BODY - B_OFFSET;
const A_X0 = A_ROD_END0 - WF - A_BODY;
const CB = { x0: A_X0 - 5 - WALL_T, x1: WALL_X + WALL_T, z0: POTI_Z - 8 - GAP - WALL_T, z1: CB_Z1, h: Math.ceil(AX_Y + CYL_R + 20) };
const VB = { x0: -280, x1: 50, z0: 72, z1: 235, h: 110 };    // valve + electronics box
const HOSE_Y = 72;
const BULK = { y: AX_Y + 8, z0: CB.z0 + 12, dz: 12 };       // rear-wall bulkheads behind G
const ANCHOR = { G: WALL_X, R: WALL_X };
const VALVE_X = { G: -240, A: -205, B: -170 };
const VALVE_ZC = 150;
const GB = { x: 820, pivotY: 250, leverLen: 120 };       // gearbox shift tower (schematic)
const topGears = ['R', '1', '3', '5'];
const botGears = [null, '2', '4', null];

const world = new THREE.Group();
scene.add(world);

// two enclosures: rigid cylinder box (cables pull here) + separate valve/electronics box
const glass = new THREE.MeshStandardMaterial({ color: 0x8fb4d8, transparent: true, opacity: 0.1, depthWrite: false, side: THREE.DoubleSide });
const edgeMat = new THREE.LineBasicMaterial({ color: 0x7f97b0 });
const lids = [];
function enclosure(b, title, part, note, floorMat, floorT) {
  const w = b.x1 - b.x0, d = b.z1 - b.z0, cx = (b.x0 + b.x1) / 2, cz = (b.z0 + b.z1) / 2;
  world.add(info(box(w, floorT, d, floorMat, cx, floorT / 2, cz), title, part, note));
  const shell = new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.BoxGeometry(w, b.h, d)), edgeMat);
  shell.position.set(cx, b.h / 2, cz);
  world.add(shell);
  for (const [ww, dd, x, z] of [[w, 2, cx, b.z0], [w, 2, cx, b.z1], [2, d, b.x0, cz], [2, d, b.x1, cz]]) world.add(box(ww, b.h, dd, glass, x, b.h / 2, z));
  const lid = box(w, 3, d, glass, cx, b.h, cz);
  lid.visible = false;
  world.add(lid);
  lids.push(lid);
}
enclosure(CB, 'Zylinderbox (Minimalmaß) – Grundplatte', `Alu ${FLOOR_T} mm, außen ~${CB.x1 - CB.x0} × ${CB.z1 - CB.z0} × ${CB.h} mm`, 'Minimalbox: Stirnwand (2 mm) = Seil-Widerlager, Zylinder von der Wand aus rückwärts angeordnet. Kraftfluss Zylinderfüße → Grundplatte → Stirnwand, deshalb Stirnwand fest mit der Grundplatte verbinden und abstützen.', mat.frame, FLOOR_T);
enclosure(VB, 'Ventilbox – Boden', `Kunststoff/Alu-Gehäuse IP65, ~${VB.x1 - VB.x0} × ${VB.z1 - VB.z0} × ${VB.h} mm`, 'Ventile auf Verteiler, Steuergerät, Lufteingang, Strom/CAN. Getrennt von der Zylinderbox, verbunden über Schlauchbündel + Sensorkabel. Kann woanders im Fahrzeug sitzen (Schläuche kurz halten).', mat.dark, 4);
world.add(label('Zylinderbox', (CB.x0 + CB.x1) / 2, CB.h + 30, CB.z0, 16));
world.add(label('Ventilbox', (VB.x0 + VB.x1) / 2, VB.h + 30, VB.z1, 16));
// bulkheads: rear wall of the cylinder box (free space behind the shorter G), long wall of the valve box
world.add(info(box(8, 22, 6 * BULK.dz + 8, mat.blue, CB.x0, BULK.y, BULK.z0 + 2.5 * BULK.dz), 'Schottleiste Zylinderbox (Rückwand)', '6 × Schottverschraubung blue C-line (6 mm)', 'Sitzt hinter G – dort ist Platz, weil G kürzer baut als die Kaskade. Schläuche kurz halten (Hypothese: lange Schläuche machen G träger).'));
world.add(info(box(110, 22, 8, mat.blue, -205, HOSE_Y, VB.z0), 'Schottleiste Ventilbox', '6 × Schottverschraubung blue C-line (6 mm)', 'Schlauchbündel G/A/B (je 2 Schläuche) zur Zylinderbox.'));
world.add(info(cyl(7, CB.x0 - 10, CB.x0 + 2, mat.dark, 'x', BULK.y + 22, BULK.z0 + 2.5 * BULK.dz), 'Sensorstecker Zylinderbox', 'M12 8-polig (Vorschlag)', '4 Endlagensensoren + Poti (3 Adern) → Steuergerät in der Ventilbox.'));

// mounting blocks
function foot(x, z) { world.add(info(box(20, AX_Y - CYL_R - FLOOR_T, FOOT_W, mat.dark, x, (AX_Y - CYL_R + FLOOR_T) / 2, z), 'Fußbefestigung + Distanzplatte', 'Airtec RA-25 (für Ø 20/25) + Distanz auf Achshöhe 40 mm', 'Je Zylinder vorne + hinten, mit M6 auf die Grundplatte. Steif befestigt, weil die Seilkraft über den Zylinder läuft.')); }

// G cylinder (gate / Wählseil)
world.add(info(cyl(CYL_R, G_X0, G_X0 + G_BODY, mat.alu, 'x', AX_Y, G_Z), 'Zylinder G (Gasse → Wählseil)', `Airtec HM-20 (vorläufig), Hub ~${GATE_TRAVEL} mm`, 'Wählt die Gasse R · 1/2 · 3/4 · 5. R und 5 = Endlagen, 1/2 und 3/4 werden „fliegend“ eingelegt. Position über Linear-Poti.'));
foot(G_X0 + 10, G_Z); foot(G_X0 + G_BODY - 10, G_Z);
const gRod = cyl(ROD_R, 0, 80, mat.steel, 'x', AX_Y, G_Z);
world.add(gRod);
const gPortCap = V(G_X0 + 14, AX_Y + CYL_R, G_Z);
const gPortRod = V(G_X0 + G_BODY - 14, AX_Y + CYL_R, G_Z);
for (const p of [gPortCap, gPortRod]) {
  world.add(info(box(10, 12, 14, mat.blue, p.x, p.y + 6, p.z), 'Drosselrückschlagventil', 'Airtec 44-XSCF-60', 'Abluftdrosselung: stellt die Geschwindigkeit von G ein (Regler „G-Geschwindigkeit“).'));
}
world.add(label('G – Gasse', G_X0 + G_BODY / 2, AX_Y + 40, G_Z - 10));

// A cylinder (fixed) + B cylinder (on linear guide, pushed by A) → Schaltseil
world.add(info(cyl(CYL_R, A_X0, A_X0 + A_BODY, mat.alu, 'x', AX_Y, ROW_Z), 'Zylinder A (Reihe, Kaskade hinten)', 'Airtec HM-20 (vorläufig), Hub ~14 mm', 'Unten → Neutral. A aus + B ein = Neutral. Fest auf der Grundplatte.'));
foot(A_X0 + 10, ROW_Z); foot(A_X0 + A_BODY - 10, ROW_Z);
const aRod = cyl(ROD_R, 0, 80, mat.steel, 'x', AX_Y, ROW_Z);
world.add(aRod);
world.add(label('A', A_X0 + A_BODY / 2, AX_Y + 26, ROW_Z + 32));
const guideLen = B_BODY + A_STROKE + 40;
world.add(info(box(guideLen, 8, 20, mat.steel, A_ROD_END0 + B_OFFSET + guideLen / 2 - 10, FLOOR_T + 4, ROW_Z), 'Linearführung für B', 'Vorschlag: Profilschiene MGN12, ~250 mm + Wagen MGN12H', 'B sitzt auf einem Schlitten und wird von der Kolbenstange A mitgeschoben (Kaskade).'));

const bGroup = new THREE.Group();
world.add(bGroup);
bGroup.add(info(cyl(9, A_ROD_END0, A_ROD_END0 + RO20, mat.dark, 'x', AX_Y, ROW_Z), 'Ausgleichskupplung A → B', 'Airtec FK-20 (M8, 57 mm)', 'Verbindet Kolbenstange A mit dem Schlitten von B. Gleicht Winkel- und Parallelversatz aus, damit nichts klemmt.'));
bGroup.add(info(cyl(CYL_R, A_ROD_END0 + B_OFFSET, A_ROD_END0 + B_OFFSET + B_BODY, mat.alu, 'x', AX_Y, ROW_Z), 'Zylinder B (Reihe, Kaskade vorn)', 'Airtec HM-20 (vorläufig), Hub ~34 mm', 'Neutral → oben. Wird mit der Kolbenstange A mitgeschoben.'));
bGroup.add(box(B_BODY, AX_Y - CYL_R - 14, 30, mat.dark, A_ROD_END0 + B_OFFSET + B_BODY / 2, (AX_Y - CYL_R + 14) / 2, ROW_Z)); // carriage on guide
const bRod = cyl(ROD_R, 0, 80, mat.steel, 'x', AX_Y, ROW_Z);
bGroup.add(bRod);
bGroup.add(label('B', A_ROD_END0 + B_OFFSET + B_BODY / 2, AX_Y + 26, ROW_Z + 32));

// clevis + Ø8 pin + cable eye
function cableEnd(parent, z, title) {
  const g = new THREE.Group();
  g.add(info(box(CLEVIS, 18, 18, mat.dark, CLEVIS / 2, AX_Y, z), 'Gabelkopf', 'Airtec RD-20 (M8)', 'Kolbenstange → Seilende. Bolzen Ø 8 mm wie am originalen Schalthebel.'));
  g.add(cyl(4, AX_Y - 14, AX_Y + 14, mat.steel, 'y', CLEVIS, z));
  const eye = new THREE.Mesh(new THREE.TorusGeometry(7, 2.5, 8, 20), mat.knob);
  eye.rotation.x = Math.PI / 2;
  eye.position.set(CLEVIS, AX_Y, z);
  g.add(info(eye, title, 'Seilauge des Originalseils', 'Steckt auf dem Ø 8-mm-Bolzen.'));
  parent.add(g);
  return g;
}
const gEndGroup = cableEnd(world, G_Z, 'Wählseil-Auge');
const bEndGroup = cableEnd(bGroup, ROW_Z, 'Schaltseil-Auge');

// inner wires (scaled unit cylinders) + sleeve abutments
const wireMat = new THREE.MeshStandardMaterial({ color: 0xd0d0d0, metalness: 0.9, roughness: 0.3 });
function wire() {
  const m = new THREE.Mesh(new THREE.CylinderGeometry(1.5, 1.5, 1, 8), wireMat);
  m.rotation.z = Math.PI / 2;
  return m;
}
const gWire = wire(), rowWire = wire();
world.add(gWire, rowWire);
world.add(info(box(WALL_T, CB.h - FLOOR_T, CB.z1 - CB.z0, mat.alu, WALL_X + WALL_T / 2, FLOOR_T + (CB.h - FLOOR_T) / 2, (CB.z0 + CB.z1) / 2),
  'Stirnwand = Seil-Widerlager', `Blech ${WALL_T} mm mit Clip-Bohrungen Ø ${HOLE_D.G} (Wählseil) / Ø ${HOLE_D.R} (Schaltseil)`,
  'Die Seilenden clipsen direkt in die Boxwand ein wie im Original-Schaltgehäuse. Wand mit der Grundplatte verschrauben/verschweißen und außen mit zwei Knotenblechen abstützen – sie nimmt die volle Seilkraft auf.'));
for (const z of [CB.z0 + 6, CB.z1 - 6]) {
  world.add(info(box(40, 50, 4, mat.dark, WALL_X + WALL_T + 20, FLOOR_T + 25, z), 'Knotenblech', '4 mm, außen an der Stirnwand', 'Stützt die 2-mm-Stirnwand gegen die Seilkraft ab.'));
}
world.add(box(40, FLOOR_T, CB.z1 - CB.z0, mat.frame, WALL_X + WALL_T + 20, FLOOR_T / 2, (CB.z0 + CB.z1) / 2));
for (const [z, name, k] of [[G_Z, 'Wählseil', 'G'], [ROW_Z, 'Schaltseil', 'R']]) {
  const clip = new THREE.Mesh(new THREE.TorusGeometry(HOLE_D[k] / 2, 2, 8, 24), mat.knob);
  clip.rotation.y = Math.PI / 2;
  clip.position.set(WALL_X + WALL_T, AX_Y, z);
  world.add(info(clip, `Clip-Bohrung ${name}`, `Ø ${HOLE_D[k]} mm in ${WALL_T} mm Wand`, 'Original-Seilende rastet hier ein.'));
  world.add(info(cyl(HOLE_D[k] / 2 - 2, WALL_X - RIGID[k], WALL_X, mat.knob, 'x', AX_Y, z), `Festes Seilende ${name}`, `${RIGID[k]} mm in der Box (Notiz)`, 'Starres Endstück des Seilzugs, das Innenseil kommt vorne heraus zum Seilauge.'));
  world.add(cyl(6, WALL_X + WALL_T, WALL_X + 60, mat.knob, 'x', AX_Y, z));
}

// poti on G rod end
world.add(info(box(GATE_TRAVEL + 70, 16, 16, mat.dark, G_ROD_END0 + CLEVIS + GATE_TRAVEL / 2, AX_Y, POTI_Z), 'Linear-Poti G', `noch auswählen (nicht Airtec), ≥ ${GATE_TRAVEL} mm`, 'Misst die Gassenposition 0–100 %: < 10 % = R, 1/2 und 3/4 je Mitte ± 5 %, > 90 % = 5.'));
const potiArm = box(6, 6, G_Z - POTI_Z, mat.red, 0, AX_Y + 12, (G_Z + POTI_Z) / 2);
world.add(potiArm);
world.add(label('Poti', G_ROD_END0 + CLEVIS + GATE_TRAVEL / 2, AX_Y + 35, POTI_Z, 11));

// sensors (ZS-6701 on NT-550 strap), along X
function sensor(parent, x, z, title) {
  const m = new THREE.MeshStandardMaterial({ color: 0x1a1a1a, emissive: 0x000000 });
  parent.add(info(box(30, 5, 6, m, x, AX_Y + CYL_R + 3, z), title, 'Airtec ZS-6701 + NT-550 + KA-31-01', 'PNP 12 V → Spannungsteiler zum Mikrocontroller. Leuchtet grün, wenn der Kolben in dieser Endlage ist.'));
  const ring = new THREE.Mesh(new THREE.TorusGeometry(CYL_R + 1, 1.2, 8, 24), mat.dark);
  ring.rotation.y = Math.PI / 2;
  ring.position.set(x, AX_Y, z);
  parent.add(ring);
  return m;
}
const sensorMats = {
  Ain: sensor(world, A_X0 + 20, ROW_Z, 'Sensor A eingefahren'),
  Aout: sensor(world, A_X0 + A_BODY - 20, ROW_Z, 'Sensor A ausgefahren'),
  Bin: sensor(bGroup, A_ROD_END0 + B_OFFSET + 20, ROW_Z, 'Sensor B eingefahren'),
  Bout: sensor(bGroup, A_ROD_END0 + B_OFFSET + B_BODY - 20, ROW_Z, 'Sensor B ausgefahren'),
};

// valves on manifold (long axis along Z)
world.add(info(box(150, 20, 150, mat.frame, VALVE_X.A + 17, 16, VALVE_ZC), 'Ventilplatte / Verteiler', 'Airtec Reihenleiste (Option)', 'Gemeinsame Druckversorgung P (1), Abluft über Schalldämpfer 40-501-18.'));
const coilMats = {};
const valvePorts = {};
for (const [k, x] of Object.entries(VALVE_X)) {
  world.add(info(box(22, 30, 100, mat.valve, x, 41, VALVE_ZC), `Ventil ${k}`, 'Airtec M-04-533-HN-441', '5/3-Wege, Mittelstellung entlüftet, 12 V DC / 4,2 W, G1/8. Spule 14 (orange) = ausfahren, Spule 12 (cyan) = einfahren, keine Spule = drucklos.'));
  const c14 = new THREE.MeshStandardMaterial({ color: 0x2a2a2a, emissive: 0x000000 });
  const c12 = new THREE.MeshStandardMaterial({ color: 0x2a2a2a, emissive: 0x000000 });
  world.add(info(box(26, 30, 26, c14, x, 41, VALVE_ZC - 63), `Spule 14 Ventil ${k}`, 'Stecker 28-ST-01', 'Bestromt = Zylinder ausfahren.'));
  world.add(info(box(26, 30, 26, c12, x, 41, VALVE_ZC + 63), `Spule 12 Ventil ${k}`, 'Stecker 28-ST-01', 'Bestromt = Zylinder einfahren.'));
  coilMats[k] = { ext: c14, ret: c12 };
  valvePorts[k] = { cap: V(x, 56, VALVE_ZC - 15), rod: V(x, 56, VALVE_ZC + 15) };
  world.add(label(k, x, 80, VALVE_ZC, 12, '#ffd27a'));
}
world.add(info(cyl(7, VB.x0 - 15, VB.x0 + 10, mat.blue, 'x', 30, VALVE_ZC), 'Druckluft-Eingang (Schottverschraubung)', 'blue C-line', 'Vom Filterregler, Schaltdruck max. 10 bar.'));
world.add(tube([V(VB.x0 + 10, 30, VALVE_ZC), V(VB.x0 + 25, 25, VALVE_ZC), V(VALVE_X.G - 30, 18, VALVE_ZC)], mat.supplyHose, 3));

// ---- clutch: pedal outside the box, optional cylinder K + valve K (auto mode only)
const PEDAL = { x: 60, pivotY: 270, pivotZ: -360, arm: 220 }; // behind the box, out of the box view
const pedal = new THREE.Group();
pedal.position.set(PEDAL.x, PEDAL.pivotY, PEDAL.pivotZ);
pedal.add(cyl(8, -20, 20, mat.steel, 'x', 0, 0));
pedal.add(info(box(14, PEDAL.arm, 10, mat.dark, 0, -PEDAL.arm / 2, 0), 'Kupplungspedal', 'Original, Weg ~5 cm (Notiz)', 'Manuell: Fuß tritt vor Plus/Minus. Automatisiert: Zylinder K drückt das Pedal.'));
pedal.add(box(60, 40, 8, mat.knob, 0, -PEDAL.arm, -2));
world.add(pedal);
world.add(box(60, 20, 60, mat.frame, PEDAL.x, 10, PEDAL.pivotZ - 40));
world.add(cyl(6, 10, PEDAL.pivotY, mat.frame, 'y', PEDAL.x, PEDAL.pivotZ - 40));
world.add(label('Kupplungspedal', PEDAL.x, PEDAL.pivotY + 35, PEDAL.pivotZ, 12));

const kGroup = new THREE.Group();
world.add(kGroup);
const K_Y = PEDAL.pivotY - PEDAL.arm + 15;
const K_ROD0 = PEDAL.pivotZ + 6;            // rod end touches the pad at c = 0
const K_BODY0 = K_ROD0 + WF;
const K_BODY = 95 + CLUTCH_TRAVEL;
kGroup.add(info(cyl(CYL_R, K_BODY0, K_BODY0 + K_BODY, mat.alu, 'z', PEDAL.x, K_Y), 'Kupplungszylinder K (Option)', `Airtec HM-20, Hub ${CLUTCH_TRAVEL} mm (vorläufig)`, 'Drückt das Pedal. Trennen schnell, Einkuppeln über Drossel langsam (Regler fehlt noch, fest 150 mm/s).'));
kGroup.add(box(36, K_Y - CYL_R, 20, mat.dark, PEDAL.x, (K_Y - CYL_R) / 2, K_BODY0 + K_BODY - 10));
const kRod = cyl(ROD_R, 0, 80, mat.steel, 'z', PEDAL.x, K_Y);
kGroup.add(kRod);
kGroup.add(label('K', PEDAL.x, K_Y + 40, K_BODY0 + K_BODY / 2, 12));
const kValveX = -135;
const kCoil = new THREE.MeshStandardMaterial({ color: 0x2a2a2a, emissive: 0x000000 });
kGroup.add(info(box(22, 30, 100, mat.valve, kValveX, 41, VALVE_ZC), 'Ventil K (Kupplung)', 'Vorschlag: Airtec M-04-511-HN-441 (5/2 monostabil)', 'Spule an = Pedal getreten. Spule aus = Federrückstellung, Pedal kommt gedrosselt zurück.'));
kGroup.add(info(box(26, 30, 26, kCoil, kValveX, 41, VALVE_ZC - 63), 'Spule Ventil K', 'Stecker 28-ST-01', 'Bestromt = Kupplung trennen.'));
kGroup.add(label('K', kValveX, 80, VALVE_ZC, 12, '#ffd27a'));
const kHose = tube([V(kValveX, 56, VALVE_ZC - 15), V(kValveX, 100, VALVE_ZC - 80), V(-60, 135, CB.z0 + 10), V(PEDAL.x, 120, CB.z0 - 15), V(PEDAL.x, K_Y + 30, K_BODY0 + K_BODY - 14), V(PEDAL.x, K_Y + CYL_R, K_BODY0 + K_BODY - 14)], hoseMat.vent, 2.5);
kGroup.add(kHose);
kGroup.visible = false;

// controller
world.add(info(box(120, 40, 90, new THREE.MeshStandardMaterial({ color: 0x2b5d34, roughness: 0.6 }), -40, 26, 160), 'Steuergerät', 'offen (Mikrocontroller + 6 MOSFET-Treiber)', 'Liest Poti + 4 Endlagensensoren, schaltet 6 Spulen (je ~0,35 A bei 12 V), CAN SHIFT.SELECT 0x150.'));
world.add(label('Steuergerät', -40, 75, 160, 11));
world.add(info(cyl(8, VB.x0 - 15, VB.x0 + 10, mat.dark, 'x', 30, 200), 'Kabelverschraubung Strom/CAN', '—', '12 V, GND, CAN-H/L.'));

// cables (outer sleeves, static) to the gearbox shift tower
const sleeveMat = new THREE.MeshStandardMaterial({ color: 0x151515, roughness: 0.8 });
const LEVER = { G: { pivotZ: -150, sign: 1, tipZ: -30 }, R: { pivotZ: 150, sign: -1, tipZ: 30 } };
const sleeveEnd = { G: V(GB.x - 70, GB.pivotY, LEVER.G.tipZ), R: V(GB.x - 70, GB.pivotY, LEVER.R.tipZ) };
for (const [z, k, name] of [[G_Z, 'G', 'Wählseil'], [ROW_Z, 'R', 'Schaltseil']]) {
  const e = sleeveEnd[k];
  world.add(info(tube([V(CB.x1 + 4, AX_Y, z), V(CB.x1 + 120, AX_Y, z), V(560, 120, (z + e.z) / 2), V(e.x - 80, e.y, e.z), e], sleeveMat, 6),
    name, 'Original-Seilzug (wie VW-Set VGS2)', name === 'Wählseil' ? 'Überträgt die Gassenbewegung (Zylinder G) zum Wählhebel am Getriebe.' : 'Überträgt die Reihenbewegung (Kaskade A + B) zum Schalthebel am Getriebe.'));
}
world.add(label('Wählseil', 560, 150, -70, 12));
world.add(label('Schaltseil', 560, 150, 70, 12));

// gearbox with shift tower and two levers
world.add(info(box(220, 230, 440, mat.comp, GB.x + 40, 115, 0), 'Getriebe (schematisch)', '—', 'Nur Platzhalter. Wählhebel und Schalthebel wie beim Original, die Seile werden direkt eingehängt.'));
world.add(box(70, 20, 380, mat.dark, GB.x, GB.pivotY - 12, 0));
const levers = {};
const leverTips = {};
for (const [k, L] of Object.entries(LEVER)) {
  const g = new THREE.Group();
  g.position.set(GB.x, GB.pivotY, L.pivotZ);
  const len = Math.abs(L.tipZ - L.pivotZ);
  g.add(info(box(14, 10, len, mat.steel, 0, 6, (L.sign * len) / 2), k === 'G' ? 'Wählhebel am Getriebe' : 'Schalthebel am Getriebe', 'Original', 'Dreht sich mit der Seilbewegung.'));
  g.add(cyl(10, -8, 14, mat.dark, 'y', 0, 0));
  world.add(g);
  levers[k] = g;
  leverTips[k] = wire();
  world.add(leverTips[k]);
}
world.add(label('Getriebe', GB.x + 40, 300, 0, 16));

// ---- supply: compressor, check valve, tank, regulator
const supply = new THREE.Group();
supply.position.set(-150, 0, -100);
scene.add(supply);
supply.add(info(box(170, 120, 120, mat.comp, -760, 60, -420), 'Kompressor', 'T-Max, max. 10,3 bar', 'Maße geschätzt. Relais + Sicherung noch zu beschaffen.'));
supply.add(cyl(45, -845, -905, mat.comp, 'x', 60, -420));
supply.add(info(box(26, 20, 20, mat.blue, -640, 40, -420), 'Rückschlagventil', 'Airtec 44-XCVPU-80', 'Kompressor → Tank.'));
supply.add(info(cyl(75, -600, 0, mat.tank, 'x', 85, -430, 40), 'Drucklufttank', 'Mercedes W166 A1663200215 (PWO)', '10 L, 20 bar, AlMg3. Maße geschätzt (Ø150 × 600). Auf Gummi-Schwingungsdämpfern.'));
for (const x of [-500, -100]) supply.add(box(30, 20, 120, mat.dark, x, 10, -430));
supply.add(info(cyl(9, 160, 195, mat.red, 'y', -120, -430), 'Sicherheitsventil', '12 bar', 'ca. 20 % über Betriebsdruck.'));
supply.add(info(box(26, 34, 26, mat.dark, -220, 175, -430), 'Druckschalter (Pressostat)', 'Airtec PE-14-01 (Option)', '1–12 bar, Wechsler. Alternative: analoger Drucksensor + Relais per Mikrocontroller.'));
const gauge = new THREE.Mesh(new THREE.CylinderGeometry(30, 30, 10, 32), new THREE.MeshStandardMaterial({ color: 0xf0f0f0 }));
gauge.rotation.z = Math.PI / 2;
gauge.position.set(8, 85, -430);
supply.add(info(gauge, 'Manometer analog', '—', 'Am Tank, zusätzlich digital im Cockpit.'));
supply.add(info(box(50, 60, 50, mat.alu, -500, 95, -250), 'Filterregler', 'Airtec Baureihe FRY (Größe offen)', 'Begrenzt den Schaltdruck (Ventile/Zylinder max. 10 bar, Kompressor 10,3 bar) und scheidet Wasser ab.'));
supply.add(cyl(18, 30, 65, new THREE.MeshStandardMaterial({ color: 0xaad4ff, transparent: true, opacity: 0.5 }), 'y', -500, -250));
const fryGauge = new THREE.Mesh(new THREE.CylinderGeometry(16, 16, 6, 24), new THREE.MeshStandardMaterial({ color: 0xf0f0f0 }));
fryGauge.rotation.x = Math.PI / 2;
fryGauge.position.set(-500, 105, -222);
supply.add(fryGauge);
supply.add(label('Kompressor', -760, 150, -420));
supply.add(label('Tank W166 10 L', -300, 190, -430));
supply.add(label('Filterregler', -500, 150, -250));

supply.add(tube([V(-675, 40, -420), V(-650, 40, -420), V(-630, 40, -420), V(-605, 60, -430)], mat.supplyHose, 4));
supply.add(tube([V(-20, 120, -430), V(0, 180, -400), V(-300, 200, -330), V(-500, 140, -260), V(-500, 125, -250)], mat.supplyHose, 4));
const supplyLink = tube([V(-625, 95, -350), V(-560, 60, -250), V(-420, 30, 60), V(VB.x0 - 60, 30, VALVE_ZC), V(VB.x0 - 15, 30, VALVE_ZC)], mat.supplyHose, 4);
scene.add(supplyLink);

// ---- dynamic hoses valve -> cylinder
const hoses = {};
for (const k of ['G', 'A', 'B']) for (const port of ['cap', 'rod']) {
  const m = new THREE.Mesh(new THREE.BufferGeometry(), hoseMat.vent);
  scene.add(m);
  hoses[k + port] = m;
}
function routeHose(mesh, from, to, slot) {
  const bz = BULK.z0 + slot * BULK.dz;
  const pts = [from.clone(), V(from.x, HOSE_Y + 10, from.z - 30), V(from.x, HOSE_Y, VB.z0 + 4), V(CB.x0 - 30, HOSE_Y, VB.z0 - 12),
    V(CB.x0 - 30, BULK.y, bz), V(CB.x0 + 12, BULK.y, bz), V((CB.x0 + to.x) / 2, CB.h - 8, (bz + to.z) / 2), V(to.x, to.y + 12, to.z), to.clone()];
  mesh.geometry.dispose();
  mesh.geometry = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 48, 2.5, 6);
}

// ---- simulation + UI
let sim = createSim();
const $ = (id) => document.getElementById(id);

const gearsDiv = $('gears');
const gearButtons = {};
for (const g of SEQUENCE) {
  const b = document.createElement('button');
  b.textContent = g;
  b.onclick = () => sim.request(g);
  gearsDiv.appendChild(b);
  gearButtons[g] = b;
}
$('plus').onclick = () => sim.plus();
$('minus').onclick = () => sim.minus();
$('neutral').onclick = () => sim.neutral();
window.addEventListener('keydown', (e) => {
  if (e.key === '+' || e.key === 'ArrowUp') sim.plus();
  if (e.key === '-' || e.key === 'ArrowDown') sim.minus();
  if (SEQUENCE.includes(e.key.toUpperCase())) sim.request(e.key.toUpperCase());
});

const sliders = {
  timeScale: { fmt: (v) => `${v.toFixed(2)}×` },
  gSpeed: { param: true, fmt: (v) => `${v} mm/s` },
  gDecel: { param: true, fmt: (v) => `${v} mm/s²` },
  ventOffset: { param: true, fmt: (v) => `${v} %` },
};
let timeScale = 0.1;
function initSliders() {
  for (const [id, cfg] of Object.entries(sliders)) {
    const el = $(id);
    if (cfg.param) el.value = sim.params[id];
    const apply = () => {
      const v = parseFloat(el.value);
      if (cfg.param) sim.params[id] = v; else timeScale = v;
      $(id + 'V').textContent = cfg.fmt(v);
    };
    el.oninput = apply;
    apply();
  }
}
initSliders();
$('reset').onclick = () => {
  const keep = { gSpeed: sim.params.gSpeed, gDecel: sim.params.gDecel, ventOffset: sim.params.ventOffset };
  sim = createSim(keep);
  applyClutchMode($('autoClutch').checked);
};
$('labels').onchange = (e) => labels.forEach((l) => (l.visible = e.target.checked));
$('supply').onchange = (e) => (supply.visible = supplyLink.visible = e.target.checked);
$('lid').onchange = (e) => lids.forEach((l) => (l.visible = e.target.checked));
const VIEWS = {
  all: { pos: [250, 1050, 1450], target: [250, 60, -40] },
  box: { pos: [420, 330, -430], target: [20, 40, -20] },
  gearbox: { pos: [430, 620, 760], target: [760, 170, 0] },
  pedal: { pos: [560, 380, 40], target: [40, 120, -330] },
};
for (const [k, v] of Object.entries(VIEWS)) {
  $('view-' + k).onclick = () => {
    camera.position.set(...v.pos);
    controls.target.set(...v.target);
    controls.update();
  };
}

$('states').innerHTML = STATES.map((s) => `<span id="st-${s}">${s}</span>`).join('');

const vText = { ext: 'ausfahren (Spule 14)', ret: 'einfahren (Spule 12)', vent: 'drucklos' };
const kText = { ext: 'Kupplung trennen (Spule an)', vent: 'Feder, einkuppeln' };

function applyClutchMode(auto) {
  sim.params.autoClutch = auto;
  sim.setFoot(false);
  kGroup.visible = auto;
  $('manualClutch').style.display = auto ? 'none' : 'block';
}
$('autoClutch').onchange = (e) => applyClutchMode(e.target.checked);
const toggleFoot = () => sim.setFoot(!sim.state.foot);
$('foot').onclick = toggleFoot;
window.addEventListener('keydown', (e) => {
  if (e.code === 'Space' && !sim.params.autoClutch) { e.preventDefault(); toggleFoot(); }
});
applyClutchMode(false);

function drawPotiBar(pt) {
  const c = $('potiBar'), ctx = c.getContext('2d'), p = sim.params;
  const W = c.width, x0 = 8, w = W - 16, X = (v) => x0 + (v / 100) * w;
  ctx.clearRect(0, 0, W, c.height);
  ctx.fillStyle = '#2a2d33'; ctx.fillRect(x0, 14, w, 12);
  ctx.fillStyle = '#2d5a3a';
  ctx.fillRect(x0, 14, X(p.P_END_LO) - x0, 12);
  ctx.fillRect(X(p.P_END_HI), 14, x0 + w - X(p.P_END_HI), 12);
  const marks = [p.P_END_LO, p.P_END_HI];
  for (const g of [1, 2]) {
    const wn = sim.win(g);
    ctx.fillStyle = '#2d5a3a';
    ctx.fillRect(X(wn.lo), 14, X(wn.hi) - X(wn.lo), 12);
    ctx.fillStyle = '#c9822b';
    for (const v of [wn.lo + p.ventOffset, wn.hi - p.ventOffset]) ctx.fillRect(X(v) - 1, 10, 2, 20);
    marks.push(Math.round(wn.lo), Math.round(wn.hi));
  }
  ctx.fillStyle = '#aaa'; ctx.font = '10px system-ui';
  for (const v of marks) ctx.fillText(v, X(v) - 6, 42);
  GATE_X.forEach((gx, i) => ctx.fillText(GATE_NAMES[i], Math.min(X((gx / GATE_TRAVEL) * 100) - 6, x0 + w - 14), 9));
  ctx.fillStyle = '#ff5050';
  ctx.beginPath(); ctx.moveTo(X(pt), 28); ctx.lineTo(X(pt) - 5, 35); ctx.lineTo(X(pt) + 5, 35); ctx.fill();
  ctx.fillRect(X(pt) - 1, 12, 2, 16);
}

function drawH(s) {
  const c = $('hPattern'), ctx = c.getContext('2d');
  const W = c.width, H = c.height;
  const px = (x) => 40 + (x / GATE_TRAVEL) * (W - 90);
  const pz = (z) => H - 22 - (z / (A_STROKE + B_STROKE)) * (H - 44); // z = a + b, top row up
  ctx.clearRect(0, 0, W, H);
  ctx.strokeStyle = '#555'; ctx.lineWidth = 8; ctx.lineCap = 'round';
  ctx.beginPath();
  GATE_X.forEach((gx, i) => { ctx.moveTo(px(gx), pz(botGears[i] ? 0 : A_STROKE)); ctx.lineTo(px(gx), pz(A_STROKE + B_STROKE)); });
  ctx.moveTo(px(0), pz(A_STROKE)); ctx.lineTo(px(GATE_TRAVEL), pz(A_STROKE));
  ctx.stroke();
  ctx.font = 'bold 12px system-ui';
  GATE_X.forEach((gx, i) => {
    for (const [g, z, dy] of [[topGears[i], A_STROKE + B_STROKE, -8], [botGears[i], 0, 18]]) {
      if (!g) continue;
      ctx.fillStyle = g === s.target ? '#ffd27a' : g === s.gear ? '#5f5' : '#aaa';
      ctx.fillText(g, px(gx) - 4, pz(z) + dy);
    }
  });
  ctx.fillStyle = s.gear === 'N' ? '#5f5' : '#9ad';
  ctx.fillText('N', px(GATE_TRAVEL) + 14, pz(A_STROKE) + 4);
  ctx.fillStyle = '#ff5050';
  ctx.beginPath(); ctx.arc(px(s.x), pz(s.a + s.b), 6, 0, Math.PI * 2); ctx.fill();
}

let lastLogLen = -1;
function updateHud() {
  const s = sim.state;
  for (const st of STATES) $('st-' + st).classList.toggle('on', st === s.state);
  $('gear').textContent = s.target ? `${s.gear} → ${s.target}` : s.gear;
  const pt = sim.poti();
  $('poti').textContent = `${pt.toFixed(1)} %`;
  $('fails').textContent = `${s.fails} / ${sim.params.MAX_FAILS}`;
  $('msg').textContent = s.message ? `⚠ ${s.message} → Meldung ans Cockpit` : '';
  $('valves').innerHTML = ['G', 'A', 'B'].map((k) => `<tr><td>${k}</td><td class="v-${s.valve[k]}">${vText[s.valve[k]]}</td></tr>`).join('')
    + (sim.params.autoClutch ? `<tr><td>K</td><td class="v-${s.valve.K}">${kText[s.valve.K]}</td></tr>` : '');
  const cl = sim.clutch();
  $('clutch').innerHTML = `${cl.pct.toFixed(0)} % – ${cl.open ? '<span class="s-on">getrennt</span>' : cl.closed ? 'eingekuppelt' : 'schleift'}`
    + (sim.params.autoClutch ? '' : ` (Fuß ${s.foot ? 'getreten' : 'oben'})`);
  $('foot').classList.toggle('active', s.foot);
  $('foot').textContent = s.foot ? 'Kupplung loslassen' : 'Kupplung treten';
  const se = sim.sensors();
  $('sensors').innerHTML = [['A', 'Ain', 'Aout'], ['B', 'Bin', 'Bout']].map(([k, i, o]) =>
    `<tr><td>${k}</td><td class="${se[i] ? 's-on' : 's-off'}">● eingefahren</td><td class="${se[o] ? 's-on' : 's-off'}">● ausgefahren</td></tr>`).join('');
  for (const g of SEQUENCE) gearButtons[g].classList.toggle('active', g === s.gear);
  if (s.log.length !== lastLogLen || s.log.length === 200) {
    lastLogLen = s.log.length;
    $('log').textContent = s.log.slice(-40).map((e) => `${(e.t * 1000).toFixed(0).padStart(6)} ms  ${e.msg}`).join('\n');
    $('log').scrollTop = 1e9;
  }
  drawPotiBar(pt);
  drawH(s);
}

function updateScene() {
  const s = sim.state;
  const gEnd = G_ROD_END0 + s.x;
  gRod.position.x = gEnd - 40;
  gEndGroup.position.x = gEnd;
  potiArm.position.x = gEnd + CLEVIS;
  const aEnd = A_ROD_END0 + s.a;
  aRod.position.x = aEnd - 40;
  bGroup.position.x = s.a;
  const bEnd = B_ROD_END0 + s.b; // local to bGroup
  bRod.position.x = bEnd - 40;
  bEndGroup.position.x = bEnd;
  const setWire = (m, x0, x1, z) => { m.scale.y = Math.max(1, x1 - x0); m.position.set((x0 + x1) / 2, AX_Y, z); };
  setWire(gWire, gEnd + CLEVIS, ANCHOR.G - RIGID.G, G_Z);
  setWire(rowWire, B_ROD_END0 + s.a + s.b + CLEVIS, ANCHOR.R - RIGID.R, ROW_Z);

  // gearbox levers turn with the cable travel (centered on mid travel)
  const ang = { G: (s.x - GATE_TRAVEL / 2) / GB.leverLen, R: (s.a + s.b - (A_STROKE + B_STROKE) / 2) / GB.leverLen };
  for (const k of ['G', 'R']) {
    const L = LEVER[k];
    levers[k].rotation.y = L.sign * ang[k];
    const len = Math.abs(L.tipZ - L.pivotZ);
    const tipX = GB.x + L.sign * Math.sin(L.sign * ang[k]) * len;
    const m = leverTips[k];
    m.scale.y = Math.max(1, Math.abs(tipX - sleeveEnd[k].x));
    m.position.set((tipX + sleeveEnd[k].x) / 2, GB.pivotY + 6, L.tipZ);
  }

  const theta = Math.asin(Math.min(1, s.c / PEDAL.arm));
  pedal.rotation.x = theta;
  kRod.position.z = K_ROD0 - s.c + 40;
  kCoil.emissive.setHex(s.valve.K === 'ext' ? 0xff8a20 : 0x000000);
  kHose.material = s.valve.K === 'ext' ? hoseMat.press : hoseMat.vent;

  const se = sim.sensors();
  for (const [k, m] of Object.entries(sensorMats)) m.emissive.setHex(se[k] ? 0x22ff44 : 0x000000);
  for (const k of ['G', 'A', 'B']) {
    coilMats[k].ext.emissive.setHex(s.valve[k] === 'ext' ? 0xff8a20 : 0x000000);
    coilMats[k].ret.emissive.setHex(s.valve[k] === 'ret' ? 0x20c0ff : 0x000000);
  }

  const ports = {
    G: { cap: gPortCap.clone().add(V(0, 12, 0)), rod: gPortRod.clone().add(V(0, 12, 0)) },
    A: { cap: V(A_X0 + 14, AX_Y + CYL_R, ROW_Z), rod: V(A_X0 + A_BODY - 14, AX_Y + CYL_R, ROW_Z) },
    B: { cap: V(A_ROD_END0 + B_OFFSET + 14 + s.a, AX_Y + CYL_R, ROW_Z), rod: V(A_ROD_END0 + B_OFFSET + B_BODY - 14 + s.a, AX_Y + CYL_R, ROW_Z) },
  };
  for (const k of ['G', 'A', 'B']) {
    const i = ['G', 'A', 'B'].indexOf(k) * 2;
    routeHose(hoses[k + 'cap'], valvePorts[k].cap, ports[k].cap, i);
    routeHose(hoses[k + 'rod'], valvePorts[k].rod, ports[k].rod, i + 1);
    hoses[k + 'cap'].material = s.valve[k] === 'ext' ? hoseMat.press : hoseMat.vent;
    hoses[k + 'rod'].material = s.valve[k] === 'ret' ? hoseMat.press : hoseMat.vent;
  }
}

// hover info
const ray = new THREE.Raycaster();
const mouse = new THREE.Vector2();
renderer.domElement.addEventListener('pointermove', (e) => {
  mouse.set((e.clientX / window.innerWidth) * 2 - 1, -(e.clientY / window.innerHeight) * 2 + 1);
  ray.setFromCamera(mouse, camera);
  const hit = ray.intersectObjects(hoverables.filter((m) => m.parent.visible && (m.parent.parent?.visible ?? true)), false)[0];
  const el = $('hover');
  if (!hit) { el.style.display = 'none'; return; }
  const i = hit.object.userData.info;
  el.innerHTML = `<b>${i.title}</b><br><span style="color:#8ab">${i.part}</span><br>${i.note}`;
  el.style.display = 'block';
});

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});

const timer = new THREE.Timer();
const SUB = 0.0005;
renderer.setAnimationLoop(() => {
  timer.update();
  const dt = Math.min(timer.getDelta(), 0.05) * timeScale;
  const n = Math.max(1, Math.round(dt / SUB));
  for (let i = 0; i < n; i++) sim.step(dt / n);
  updateScene();
  updateHud();
  controls.update();
  renderer.render(scene, camera);
});
