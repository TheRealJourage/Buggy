import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { createSim, GATE_X, GATE_TRAVEL, GATE_NAMES, A_STROKE, B_STROKE, SEQUENCE, STATES } from './sim.js';

// ---- geometry constants (mm), HM-20 per Airtec catalog: body = 95 + stroke, WF 24, Ø27 / rod Ø8
const CYL_R = 13.5;
const ROD_R = 4;
const WF = 24;
const G_STROKE = GATE_TRAVEL;
const G_BODY = 95 + G_STROKE;
const A_BODY = 95 + A_STROKE;
const B_BODY = 95 + B_STROKE;
const ROW_Y = 64;          // axis height of A/B on the carriage
const G_Y = 20, G_Z = -60; // G axis
const G_BODY_END = -59;    // rod end = G_BODY_END + WF + x - ... = x - 35
const A_Z0 = -290;         // A cap end on carriage
const A_ROD_END0 = A_Z0 + A_BODY + WF;     // -157 at a = 0
const B_OFFSET = 6;       // B body starts 6 mm after the coupling
const B_ROD_END0 = A_ROD_END0 + B_OFFSET + B_BODY + WF; // 2 at a = b = 0
const PLATE_Y = 250;
const NEUTRAL_PIN_Z = B_ROD_END0 + A_STROKE;
const TOP_PIN_Z = B_ROD_END0 + A_STROKE + B_STROKE;
const VALVE_Z = { G: -160, A: -60, B: 40 };

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
camera.position.set(380, 480, 560);
const controls = new OrbitControls(camera, renderer.domElement);
controls.target.set(-90, 80, -80);
controls.update();

const grid = new THREE.GridHelper(1600, 32, 0x3a3d44, 0x2c2f35);
grid.position.set(-200, 0, -150);
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

// ---- fixed frame: rails, G cylinder, poti, gate plate
const world = new THREE.Group();
scene.add(world);

world.add(box(520, 8, 420, mat.frame, -150, 4, -110));
for (const z of [-250, 40]) {
  world.add(info(cyl(6, -90, GATE_TRAVEL + 100, mat.steel, 'x', 30, z), 'Führungsschiene', '—', 'Schematisch: der Schlitten trägt die Kaskade A+B und wird von G seitlich (Gasse) bewegt. Im Fahrzeug wirken die Zylinder über die Schaltzüge.'));
}

// G cylinder
const gBodyStart = G_BODY_END - G_BODY;
world.add(info(cyl(CYL_R, gBodyStart, G_BODY_END, mat.alu, 'x', G_Y, G_Z), 'Zylinder G (Gasse)', `Airtec HM-20 (vorläufig), Hub ~${GATE_TRAVEL} mm`, 'Wählt die Gasse R · 1/2 · 3/4 · 5. R und 5 = Endlagen. In 1/2 und 3/4 wird nicht angehalten: Reihe wird „fliegend“ eingelegt, G ab Fensteranfang drucklos.'));
world.add(box(14, 36, 36, mat.dark, gBodyStart - 4, G_Y, G_Z));
world.add(box(20, 24, 40, mat.frame, gBodyStart - 4, 8, G_Z));
const gRod = cyl(ROD_R, 0, 120, mat.steel, 'x', G_Y, G_Z);
world.add(gRod);
const gPortCap = new THREE.Vector3(gBodyStart + 12, G_Y + CYL_R, G_Z);
const gPortRod = new THREE.Vector3(G_BODY_END - 12, G_Y + CYL_R, G_Z);
for (const p of [gPortCap, gPortRod]) {
  world.add(info(box(10, 12, 14, mat.blue, p.x, p.y + 6, p.z), 'Drosselrückschlagventil', 'Airtec 44-XSCF-60', 'Abluftdrosselung: stellt die Geschwindigkeit von G ein (Regler „G-Geschwindigkeit“).'));
}
world.add(label('G – Gasse', (gBodyStart + G_BODY_END) / 2, G_Y + 45, G_Z));

// Poti
const POTI_Z = 10;
world.add(info(box(GATE_TRAVEL + 110, 14, 14, mat.dark, GATE_TRAVEL / 2, 20, POTI_Z), 'Linear-Poti G', `noch auswählen (nicht Airtec), ≥ ${GATE_TRAVEL} mm`, 'Misst die Gassenposition 0–100 %: < 10 % = R, 1/2 und 3/4 je Mitte ± 5 %, > 90 % = 5.'));
const potiWiper = box(6, 10, 18, mat.red, 0, 30, POTI_Z);
world.add(potiWiper);
world.add(label('Poti', GATE_TRAVEL / 2, 2, POTI_Z + 30, 11));

// Gate plate (H pattern)
world.add(info(box(GATE_TRAVEL + 70, 4, 110, mat.plate, GATE_TRAVEL / 2, PLATE_Y, 20), 'Schaltkulisse (Darstellung)', '—', 'Zeigt den Weg des Schaltfingers. Oben R · 1 · 3 · 5, unten 2 · 4.'));
const slotY = PLATE_Y + 2.2;
// slot per gate: R and 5 only neutral -> top, 1/2 and 3/4 full
const topGears = ['R', '1', '3', '5'];
const botGears = [null, '2', '4', null];
GATE_X.forEach((gx, i) => {
  const z0 = botGears[i] ? B_ROD_END0 : NEUTRAL_PIN_Z;
  world.add(box(8, 0.5, TOP_PIN_Z - z0 + 8, mat.slot, gx, slotY, (TOP_PIN_Z + z0) / 2));
  world.add(label(topGears[i], gx, PLATE_Y + 12, TOP_PIN_Z + 18, 13, '#ffd27a'));
  if (botGears[i]) world.add(label(botGears[i], gx, PLATE_Y + 12, B_ROD_END0 - 18, 13, '#ffd27a'));
});
world.add(box(GATE_TRAVEL + 8, 0.5, 8, mat.slot, GATE_TRAVEL / 2, slotY, NEUTRAL_PIN_Z));
world.add(label('N', GATE_TRAVEL + 24, PLATE_Y + 12, NEUTRAL_PIN_Z, 11, '#9ad'));

// ---- carriage with cascade A + B
const carriage = new THREE.Group();
scene.add(carriage);
carriage.add(info(box(70, 10, 370, mat.carriage, 0, 41, -120), 'Schlitten', '—', 'Wird von G in Gassenrichtung verschoben, trägt die Kaskade A + B.'));
for (const z of [-250, 40]) carriage.add(box(30, 14, 26, mat.dark, 0, 31, z));
carriage.add(box(10, 30, 20, mat.dark, -35, 26, G_Z)); // G rod bracket
carriage.add(box(6, 14, 6, mat.red, 0, 30, POTI_Z)); // poti wiper bracket

// A
carriage.add(info(cyl(CYL_R, A_Z0, A_Z0 + A_BODY, mat.alu, 'z', 0, ROW_Y), 'Zylinder A (Reihe, Kaskade hinten)', 'Airtec HM-20 (vorläufig), Hub ~14 mm', 'Unten → Neutral. A aus + B ein = Neutral.'));
carriage.add(box(36, 22, 14, mat.dark, 0, 52, A_Z0 - 4));
const aRod = cyl(ROD_R, 0, 60, mat.steel, 'z', 0, ROW_Y);
carriage.add(aRod);
carriage.add(label('A', 0, ROW_Y + 40, A_Z0 + A_BODY / 2));

// B (moves with A rod)
const bGroup = new THREE.Group();
carriage.add(bGroup);
const bZ0 = A_ROD_END0;
bGroup.add(box(30, 30, 10, mat.dark, 0, ROW_Y, bZ0 + 2)); // coupling A rod -> B body
bGroup.add(info(cyl(CYL_R, bZ0 + 6, bZ0 + 6 + B_BODY, mat.alu, 'z', 0, ROW_Y), 'Zylinder B (Reihe, Kaskade vorn)', 'Airtec HM-20 (vorläufig), Hub ~34 mm', 'Neutral → oben. Wird mit A-Kolbenstange mitgeschoben (Kaskade).'));
bGroup.add(box(20, 6, B_BODY, mat.dark, 0, 47, bZ0 + 6 + B_BODY / 2)); // slide shoe
const bRod = cyl(ROD_R, 0, 60, mat.steel, 'z', 0, ROW_Y);
bGroup.add(bRod);
bGroup.add(label('B', 0, ROW_Y + 40, bZ0 + B_BODY / 2));
const finger = new THREE.Group();
bGroup.add(finger);
finger.add(info(box(16, 16, 20, mat.dark, 0, ROW_Y, 0), 'Gabelkopf', 'Airtec RD-20 (M8)', 'Anlenkung Kolbenstange → Schaltzug / Schaltfinger (Ø 8 mm).'));
finger.add(cyl(ROD_R, ROW_Y, PLATE_Y + 20, mat.steel, 'y', 0, 0));
finger.add(info(new THREE.Mesh(new THREE.SphereGeometry(13, 24, 16), mat.knob), 'Schaltfinger / Knauf', '—', 'Position = eingelegter Gang.'));
finger.children[2].position.set(0, PLATE_Y + 26, 0);

// sensors (ZS-6701 on NT-550 strap)
const sensorMat = () => new THREE.MeshStandardMaterial({ color: 0x1a1a1a, emissive: 0x000000 });
function sensor(parent, z, title) {
  const g = new THREE.Group();
  const m = sensorMat();
  const body = info(box(6, 5, 30, m, 0, ROW_Y + CYL_R + 3, z), title, 'Airtec ZS-6701 + NT-550 + KA-31-01', 'Reed/induktiv, PNP 12 V → Spannungsteiler zum Mikrocontroller. Leuchtet grün, wenn der Kolben in dieser Endlage ist.');
  g.add(body);
  const ring = new THREE.Mesh(new THREE.TorusGeometry(CYL_R + 1, 1.2, 8, 24), mat.dark);
  ring.position.set(0, ROW_Y, z);
  g.add(ring);
  parent.add(g);
  return m;
}
const sensorMats = {
  Ain: sensor(carriage, A_Z0 + 18, 'Sensor A eingefahren'),
  Aout: sensor(carriage, A_Z0 + A_BODY - 18, 'Sensor A ausgefahren'),
  Bin: sensor(bGroup, bZ0 + B_OFFSET + 18, 'Sensor B eingefahren'),
  Bout: sensor(bGroup, bZ0 + B_OFFSET + B_BODY - 18, 'Sensor B ausgefahren'),
};

// ---- valves on manifold
world.add(info(box(150, 20, 290, mat.frame, -370, 10, -60), 'Ventilplatte / Verteiler', 'Airtec Reihenleiste (Option)', 'Gemeinsame Druckversorgung P (1), Abluft über Schalldämpfer 40-501-18.'));
const coilMats = {};
const valvePorts = {};
for (const [k, z] of Object.entries(VALVE_Z)) {
  world.add(info(box(100, 30, 22, mat.valve, -370, 35, z), `Ventil ${k}`, 'Airtec M-04-533-HN-441', '5/3-Wege, Mittelstellung entlüftet, 12 V DC / 4,2 W, G1/8. Spule 14 (orange) = ausfahren, Spule 12 (cyan) = einfahren, keine Spule = drucklos.'));
  const c14 = new THREE.MeshStandardMaterial({ color: 0x2a2a2a, emissive: 0x000000 });
  const c12 = new THREE.MeshStandardMaterial({ color: 0x2a2a2a, emissive: 0x000000 });
  world.add(info(box(26, 30, 26, c14, -433, 35, z), `Spule 14 Ventil ${k}`, 'Stecker 28-ST-01', 'Bestromt = Zylinder ausfahren.'));
  world.add(info(box(26, 30, 26, c12, -307, 35, z), `Spule 12 Ventil ${k}`, 'Stecker 28-ST-01', 'Bestromt = Zylinder einfahren.'));
  coilMats[k] = { ext: c14, ret: c12 };
  for (const dx of [-15, 15]) world.add(cyl(4, 20, 8, mat.steel, 'y', -370 + dx, z + 8));
  valvePorts[k] = { cap: new THREE.Vector3(-385, 50, z), rod: new THREE.Vector3(-355, 50, z) };
  world.add(label(k, -370, 70, z, 14, '#ffd27a'));
}

// ---- supply: compressor, check valve, tank, regulator
const SUPPLY_DZ = -260; // keep the tank out of the way of the shift unit
const supply = new THREE.Group();
supply.position.z = SUPPLY_DZ;
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

function tube(points, material, r = 3) {
  const curve = new THREE.CatmullRomCurve3(points);
  return new THREE.Mesh(new THREE.TubeGeometry(curve, 32, r, 8), material);
}
const V = (x, y, z) => new THREE.Vector3(x, y, z);
supply.add(tube([V(-675, 40, -420), V(-650, 40, -420), V(-630, 40, -420), V(-605, 60, -430)], mat.supplyHose, 4));
supply.add(tube([V(-20, 120, -430), V(0, 180, -400), V(-300, 200, -330), V(-500, 140, -260), V(-500, 125, -250)], mat.supplyHose, 4));
const supplyLink = tube([V(-475, 95, -250 + SUPPLY_DZ), V(-440, 60, -300), V(-400, 20, -230), V(-370, 10, -205)], mat.supplyHose, 4);
scene.add(supplyLink);

// ---- dynamic hoses valve -> cylinder
const hoses = {};
for (const k of ['G', 'A', 'B']) for (const port of ['cap', 'rod']) {
  const m = new THREE.Mesh(new THREE.BufferGeometry(), hoseMat.vent);
  scene.add(m);
  hoses[k + port] = m;
}
function routeHose(mesh, from, to) {
  const top = Math.max(from.y, to.y) + 90;
  const pts = [from.clone(), V(from.x, from.y + 40, from.z), V((from.x + to.x) / 2, top, (from.z + to.z) / 2), V(to.x, to.y + 40, to.z), to.clone()];
  mesh.geometry.dispose();
  mesh.geometry = new THREE.TubeGeometry(new THREE.CatmullRomCurve3(pts), 24, 2.5, 6);
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
};
$('labels').onchange = (e) => labels.forEach((l) => (l.visible = e.target.checked));
$('supply').onchange = (e) => (supply.visible = supplyLink.visible = e.target.checked);

$('states').innerHTML = STATES.map((s) => `<span id="st-${s}">${s}</span>`).join('');

const vText = { ext: 'ausfahren (Spule 14)', ret: 'einfahren (Spule 12)', vent: 'drucklos' };

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
  $('valves').innerHTML = ['G', 'A', 'B'].map((k) => `<tr><td>${k}</td><td class="v-${s.valve[k]}">${vText[s.valve[k]]}</td></tr>`).join('');
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
  carriage.position.x = s.x;
  potiWiper.position.x = s.x;
  // G rod end at carriage bracket (x - 35)
  const gEnd = s.x - 35;
  gRod.position.x = gEnd - 60;
  // A rod
  const aEnd = A_ROD_END0 + s.a;
  aRod.position.z = aEnd - 30;
  bGroup.position.z = s.a;
  const bEnd = bZ0 + B_OFFSET + B_BODY + WF + s.b;
  bRod.position.z = bEnd - 30;
  finger.position.z = bEnd;

  const se = sim.sensors();
  for (const [k, m] of Object.entries(sensorMats)) m.emissive.setHex(se[k] ? 0x22ff44 : 0x000000);
  for (const k of ['G', 'A', 'B']) {
    coilMats[k].ext.emissive.setHex(s.valve[k] === 'ext' ? 0xff8a20 : 0x000000);
    coilMats[k].ret.emissive.setHex(s.valve[k] === 'ret' ? 0x20c0ff : 0x000000);
  }

  const cx = s.x;
  const ports = {
    G: { cap: gPortCap.clone().add(V(0, 12, 0)), rod: gPortRod.clone().add(V(0, 12, 0)) },
    A: { cap: V(cx, ROW_Y + CYL_R, A_Z0 + 10), rod: V(cx, ROW_Y + CYL_R, A_Z0 + A_BODY - 10) },
    B: { cap: V(cx, ROW_Y + CYL_R, bZ0 + B_OFFSET + 10 + s.a), rod: V(cx, ROW_Y + CYL_R, bZ0 + B_OFFSET + B_BODY - 10 + s.a) },
  };
  for (const k of ['G', 'A', 'B']) {
    routeHose(hoses[k + 'cap'], valvePorts[k].cap, ports[k].cap);
    routeHose(hoses[k + 'rod'], valvePorts[k].rod, ports[k].rod);
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
