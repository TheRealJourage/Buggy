// Cockpit mock-up: switch assignment from Vault Buggy/Bordnetz-Konzept.md ("Bedienelemente & Panels", 2026-10-22).
import { SHIFT_ORDER } from './sim.js';

const W = 1920, H = 900;
const RED = '#ff3322', GREEN = '#33ff55', AMBER = '#ffaa00', BLUE = '#3aa0ff', YELLOW = '#ffe033';
const ILLEGAL = 'Kann im Straßenverkehr illegal sein → beleuchteter Schalter (Regel: alles, was vor der Straße aus muss, leuchtet).';

const PANELS = [
  { id: 'licht', title: 'Lichtpanel', x: 20, y: 190, cols: 5, items: [
    // mirrored 2026-10-22: light/fog is the rightmost column
    { id: 'suchAn', type: 'A', cover: true, led: RED, label: 'Suchlicht', pos: ['AUS', 'AN'], illegal: true, info: 'Suchscheinwerfer an/aus (Last-Ebene). Im Straßenverkehr nicht zulässig (mit TÜV klären). ' + ILLEGAL },
    { id: 'rundum', type: 'A', led: RED, label: 'Rundum', sub: 'orange', pos: ['AUS', 'AN'], illegal: true, info: 'Orange Rundumleuchte. Zulassung mit dem TÜV klären. ' + ILLEGAL },
    { id: 'offM', type: 'A', cover: true, led: RED, label: 'Offroad', sub: 'Mitte', pos: ['AUS', 'AN'], illegal: true, info: 'Offroad-Scheinwerfer Mitte (2,85 A, Last-Ebene). ' + ILLEGAL },
    { id: 'zusatz', type: 'C', cover: true, label: 'Zusatz-Fern', sub: 'Freigabe', pos: ['AUS', 'FREI'], info: 'Freigabe Zusatz-Fernlicht. Brennt nur zusammen mit dem Fernlicht (Lichthebel). Typ C, unbeleuchtet.' },
    { id: 'licht', type: 'B', label: 'Licht', pos: ['AUS', 'STAND', 'ABBLEND'], info: 'Aus / Standlicht / Abblendlicht inkl. Rücklicht + Kennzeichen. Standlicht-Stellung wird beim Abstellen im Parkmodus gehalten.' },
    { id: 'joy', type: 'joy', label: 'Joystick', sub: 'Pan / Tilt', info: 'Gamepad-Daumenstick X/Y als Ratenbefehl: Auslenkung = Drehgeschwindigkeit, loslassen = stehen bleiben. Wirkt nur, wenn das Suchlicht an ist. Presets/Home später.' },
    { id: 'nahfeld', type: 'A', led: RED, label: 'Nahfeld', sub: 'Unterboden weiß', pos: ['AUS', 'AN'], illegal: true, info: 'Nahfeldbeleuchtung: helle weiße LEDs unter dem Fahrzeug (Unterboden-Idee, wegen der Bodenfreiheit als Nahfeld). Zulassung mit TÜV klären. ' + ILLEGAL },
    { id: 'offH', type: 'A', cover: true, led: RED, label: 'Offroad', sub: 'Heck', pos: ['AUS', 'AN'], illegal: true, info: 'Offroad Heck (2,70 A, DT-Pin 7). ' + ILLEGAL },
    { id: 'rueck', type: 'B', cover: true, clearCap: true, capGlow: () => controls.rueck.st.pos === 'up', label: 'Rückfahr', sub: 'Zusatz weiß', pos: ['AUS', 'R-AUTO', 'AN'], info: 'Zusatz-Rückfahrscheinwerfer weiß (3,40 A, DT-Pin 8). Kappe zu = AUS. Mitte = Rangieren: an, solange R eingelegt ist. Oben = dauerhaft AN. Durchsichtige rote Kappe leuchtet bei AN (bei R-AUTO nicht, das ist straßenlegal). Zulassung mit TÜV klären.' },
    { id: 'nebel', type: 'B', cover: true, clearCap: true, capGlow: () => controls.nebel.st.pos === 'up' && controls.licht.st.pos !== 'down', label: 'Nebel', pos: ['AUS', 'VORN', 'V+SCHL'], info: 'Aus / Nebelscheinwerfer vorn / vorn + Nebelschlussleuchte. Wirkt nur mit Stand- oder Abblendlicht. Durchsichtige rote Kappe: leuchtet von unten (LED durch die Kappen-Platte), sobald die Nebelschlussleuchte an ist.' },
  ] },
  { id: 'motor', title: 'Motor & Zubehör', x: 1250, y: 190, cols: 5, items: [
    { id: 'zuendung', type: 'A', led: GREEN, label: 'Zündung', sub: 'Kl. 15', pos: ['AUS', 'AN'], info: 'Rastender Kippschalter, nur freigegeben bei gültigem Lenkrad-Code. LED leuchtet nur, wenn Kl. 15 wirklich anliegt.' },
    { id: 'start', type: 'T', cover: true, label: 'Start', sub: 'Kl. 50', pos: ['', 'START'], info: 'Tastend (EIN)-AUS mit Kappe. Wirkt nur bei BEREIT (N oder Kupplung getreten, Selbsttest ok).' },
    { id: 'kompressor', type: 'B', cover: true, label: 'Kompressor', pos: ['AUTO', 'AUS', 'AN'], info: 'Kappe zu = AUTO (läuft erst bei laufendem Motor). Mitte = AUS. Oben = AN: übergeht nur „Motor läuft“, der Druckschalter schaltet immer ab.' },
    { id: 'windeFrei', type: 'A', cover: true, led: YELLOW, label: 'Winde', sub: 'Freigabe', pos: ['AUS', 'FREI'], info: 'Freigabe für die Winde am Wischerhebel (herziehen = ein, wegdrücken = aus). Signal an den Controller, der die Winden-Relais schaltet. Vorschlag: zusätzlich in Reihe zur Spulenversorgung der Winden-Relais. Fernbedienung der Winde bleibt.' },
    { id: 'sitz', type: 'C', label: 'Sitzheizung', pos: ['AUS', 'AN'], info: 'Einstufig, normaler Kippschalter (Typ C).' },
    { id: 'luefter', type: 'A', led: BLUE, label: 'Lüfter', sub: 'Auto / Manuell', pos: ['AUTO', 'MAN'], info: 'Auto = Speeduino (CLT, Pin 47). Manuell = Tastgrad vom Poti daneben. Typ A oder C noch offen.' },
    { id: 'luefterPoti', type: 'pot', label: 'Lüfter', sub: 'Tastgrad', info: 'Tastgrad im Modus Manuell.' },
    { id: 'a12Front', type: 'A', led: BLUE, label: '12 V Front', pos: ['AUS', 'AN'], info: '12-V-Abnahmepunkt außen an der Front.' },
    { id: 'a12Sitz', type: 'A', led: BLUE, label: '12 V Sitz', sub: 'hinter Fahrer', pos: ['AUS', 'AN'], info: '12-V-Abnahmepunkt hinter dem Fahrersitz.' },
    { id: 'a12Heck', type: 'A', led: BLUE, label: '12 V Heck', pos: ['AUS', 'AN'], info: '12-V-Abnahmepunkt außen am Heck.' },
  ] },
  { id: 'audio', title: 'Entertainment, Audio & Helligkeit', x: 1250, y: 630, cols: 6, items: [
    { id: 'carplay', type: 'C', label: 'CarPlay-Pi', pos: ['AUS', 'AN'], info: 'Sound-/CarPlay-Pi an/aus (Typ C). Ohne den Pi läuft kein Audio: Musik, Telefon, Intercom und Funk gehen alle über ihn, deshalb sitzt der Schalter vorne bei den Lautstärkereglern.' },
    { id: 'volCar', type: 'pot', label: 'CarPlay-Pi', sub: 'Kopfhörer', info: 'Lautstärke Musik/Telefon im Kopfhörer.' },
    { id: 'volIc', type: 'pot', label: 'Intercom', sub: 'Kopfhörer', info: 'Lautstärke Beifahrer (Sena/Cardo) im Kopfhörer.' },
    { id: 'volFunk', type: 'pot', label: 'Funk', sub: 'CB / PMR', info: 'Lautstärke Funk im Kopfhörer. Sendepegel (CarPlay, Intercom, Funk, Megafon) im Pi-Menü (Vorschlag).' },
    { id: 'ambient', type: 'pot', label: 'Ambient', sub: 'Helligkeit', info: 'Ambient-Licht, Art noch offen.' },
    { id: 'tacho', type: 'pot', label: 'Tacho / LED', sub: 'Helligkeit', info: 'Helligkeit Tacho-Pi und Kontroll-LEDs.' },
  ] },
];

const WHEEL = [
  { id: 'hupe', x: 340, y: 300, r: 36, fill: '#444', text: 'HUPE', fn: 'Hupe', info: 'Hupe (GP8, Bit 2). Festgelegt.' },
  { id: 'gelb', x: 278, y: 350, r: 16, fill: '#e6c200', fn: 'Warnblinker', below: true, info: 'Gelb (GP11): Warnblinker ein/aus (Toggle). Kontrolle über die Blinkpfeile im Display. Hinweis: geht nur, solange das Lenkrad steckt und Kl. 30 + LIN-Kette wach sind.' },
  { id: 'blau', x: 236, y: 262, r: 16, fill: '#1e64d8', fn: 'PTT Funk', info: 'Blau (GP10): außen, beim Fahren gut erreichbar. Tendenz: Push-to-Talk Funk (CB/PMR).' },
  { id: 'minus', paddle: 'L', fn: 'runter', info: 'Große Schaltwippe links = Minus (GP12): runterschalten 5→…→1→R. Festgelegt.' },
  { id: 'gruen', x: 444, y: 262, r: 16, fill: '#1faa3a', fn: 'offen (PTT?)', info: 'Grün (GP5): außen, beim Fahren gut erreichbar. Wahrscheinlich ein zweites Push-to-Talk, Ziel noch offen.' },
  { id: 'rot', x: 402, y: 350, r: 16, fill: '#d42020', fn: 'Spiegelhzg.', below: true, info: 'Rot (GP6): Spiegelheizung mit Timer – drücken = 10 min an, nochmal drücken = aus.' },
  { id: 'plus', paddle: 'R', fn: 'hoch', info: 'Große Schaltwippe rechts = Plus (GP7): hochschalten R→1→…→5. Festgelegt.' },
  { id: 'schwarz', x: 340, y: 392, r: 16, fill: '#111', text: 'N', fn: 'Neutral', info: 'Schwarz (GP9), unten mittig: Gang N. Festgelegt 2026-10-22. Position am Lenkrad bestätigt der User noch.' },
];
const WHEEL_POTS = [
  { id: 'potL', x: 226, y: 410, fn: 'Servo', info: 'Lenkrad-Poti: Servo-Unterstützung (Tendenz), über CAN zum STM32, sobald das Kit-Poti ausgemessen ist. Welches Poti (links/rechts) ist noch offen.' },
  { id: 'potR', x: 454, y: 410, fn: 'Lautstärke', info: 'Lenkrad-Poti: Gesamtlautstärke (Tendenz).' },
];

const s = { gear: 'N', warn: false, winch: null, pan: 0, tilt: 0, blinkL: false, blinkR: false, fern: false, flash: false, spiegelUntil: 0, phase: false, log: [] };
const controls = {};
const dash = document.getElementById('dash');
dash.style.width = W + 'px';
dash.style.height = H + 'px';
const NS = 'http://www.w3.org/2000/svg';

function info(def) {
  const el = document.getElementById('info');
  const t = { A: 'Typ A · 2 Stellungen mit LED', B: 'Typ B · 3 Stellungen', C: 'Typ C · 2 Stellungen ohne LED', T: 'Typ T · tastend', T2: 'Typ T2 · tastend beide Richtungen', push: 'Druckknopf', pot: 'Drehpoti', reserve: 'Reserve', wheel: 'Lenkrad', stalk: 'Lenkstock' }[def.type] || '';
  el.innerHTML = `<b>${def.label || def.fn}${def.sub && def.type !== 'wheel' ? ' – ' + def.sub : ''}</b> <span style="color:#888">${t}${def.cover ? ' + Kappe' : ''}</span><br>${def.info || ''}`;
}

function log(msg) {
  s.log.unshift(msg);
  s.log.length = Math.min(s.log.length, 6);
}

// ---------- switch graphics ----------
const CX = 46, CY = 66;

function lever(pos, def, lit) {
  const led = def.led;
  const capFill = led ? (lit ? led : shade(led)) : 'url(#chrome)';
  const glow = led && lit ? ' filter="url(#glow)"' : '';
  if (pos === 'mid') {
    return `<circle cx="${CX}" cy="${CY}" r="8.5" fill="url(#chrome)" stroke="#222"/>
      <circle cx="${CX}" cy="${CY}" r="5.5" fill="${capFill}"${glow}/>`;
  }
  const tip = pos === 'up' ? CY - 44 : CY + 44;
  const shadow = pos === 'up' ? `<ellipse cx="${CX + 4}" cy="${CY - 10}" rx="5" ry="16" fill="#0006"/>` : `<ellipse cx="${CX + 4}" cy="${CY + 26}" rx="6" ry="20" fill="#0007"/>`;
  return `${shadow}<polygon points="${CX - 4},${CY} ${CX + 4},${CY} ${CX + 6},${tip} ${CX - 6},${tip}" fill="url(#chrome)" stroke="#333" stroke-width=".6"/>
    <ellipse cx="${CX}" cy="${tip}" rx="7.5" ry="5.5" fill="${capFill}" stroke="#222" stroke-width=".6"${glow}/>
    <ellipse cx="${CX - 2}" cy="${tip - 1.5}" rx="2.5" ry="1.4" fill="#fff8"/>`;
}

function shade(hex) {
  const n = parseInt(hex.slice(1), 16);
  const f = (v) => Math.round(((n >> v) & 255) * 0.28);
  return `rgb(${f(16)},${f(8)},${f(0)})`;
}

function nut() {
  const pts = [...Array(6)].map((_, i) => {
    const a = Math.PI / 6 + (i * Math.PI) / 3;
    return `${CX + 17 * Math.cos(a)},${CY + 17 * Math.sin(a)}`;
  }).join(' ');
  return `<circle cx="${CX}" cy="${CY}" r="22" fill="#0005"/>
    <polygon points="${pts}" fill="url(#nut)" stroke="#2a2a2a"/>
    <circle cx="${CX}" cy="${CY}" r="10.5" fill="#6c6f74" stroke="#222"/>
    <circle cx="${CX}" cy="${CY}" r="6" fill="#1a1a1a"/>`;
}

function cover(open, clear, glow) {
  if (open && clear) {
    return `<rect x="22" y="2" width="48" height="15" rx="4" fill="${glow ? '#ff3a2e' : '#a8201a99'}" stroke="#600"${glow ? ' filter="url(#glow)"' : ''}/>
      <rect x="25" y="4" width="42" height="3" rx="1.5" fill="#ffd0c8${glow ? 'cc' : '44'}"/>
      <circle cx="46" cy="23" r="2" fill="${glow ? '#ff6050' : '#3a0a08'}"/>
      <rect x="20" y="15" width="52" height="6" rx="2" fill="url(#chrome)" stroke="#333" stroke-width=".6"/>`;
  }
  if (open) {
    return `<rect x="22" y="2" width="48" height="15" rx="4" fill="#7a0c08" stroke="#400"/>
      <rect x="25" y="4" width="42" height="3" rx="1.5" fill="#ff5a4a66"/>
      <rect x="20" y="15" width="52" height="6" rx="2" fill="url(#chrome)" stroke="#333" stroke-width=".6"/>`;
  }
  return `<rect x="20" y="15" width="52" height="6" rx="2" fill="url(#chrome)" stroke="#333" stroke-width=".6"/>
    <rect x="22" y="19" width="48" height="98" rx="8" fill="url(#cover)" stroke="#500" stroke-width="1.2"/>
    <rect x="27" y="24" width="7" height="86" rx="3.5" fill="#ffffff2a"/>
    <rect x="38" y="106" width="16" height="7" rx="3" fill="#5a0805"/>`;
}

function posText(def) {
  const [down, mid, up] = def.type === 'B' || def.type === 'T2' ? def.pos : [def.pos[0], '', def.pos[1]];
  return `<text x="${CX}" y="11" text-anchor="middle" font-size="9.5" font-weight="700" fill="#d8d8d8">${up || ''}</text>
    <text x="${CX}" y="128" text-anchor="middle" font-size="9.5" font-weight="700" fill="#d8d8d8">${down || ''}</text>
    ${mid ? `<text x="91" y="${CY - 22}" text-anchor="end" font-size="8.5" font-weight="700" fill="#ddd" stroke="#000" stroke-width="2.5" paint-order="stroke">${mid}</text>` : ''}`;
}

function renderToggle(c) {
  const { def, st } = c;
  const lit = def.led && isLit(def, st);
  c.svg.innerHTML = nut() + lever(st.pos, def, lit) + (def.cover ? cover(st.open, def.clearCap, def.capGlow && def.capGlow()) : '') + posText(def);
}

function isLit(def, st) {
  if (def.type === 'push') return Date.now() < s.spiegelUntil;
  if (st.pos !== 'up') return false;
  if (def.blink) return s.phase;
  return true;
}

function renderPot(c) {
  const v = c.st.v;
  const a0 = -135, a = a0 + v * 270;
  let ticks = '';
  for (let i = 0; i <= 10; i++) {
    const t = ((a0 + i * 27) - 90) * Math.PI / 180;
    const r1 = 33, r2 = i % 5 === 0 ? 40 : 37;
    ticks += `<line x1="${CX + r1 * Math.cos(t)}" y1="${CY + r1 * Math.sin(t)}" x2="${CX + r2 * Math.cos(t)}" y2="${CY + r2 * Math.sin(t)}" stroke="#bbb" stroke-width="${i % 5 === 0 ? 2 : 1}"/>`;
  }
  c.svg.innerHTML = `${ticks}
    <circle cx="${CX}" cy="${CY + 3}" r="27" fill="#0007"/>
    <circle cx="${CX}" cy="${CY}" r="26" fill="url(#knob)" stroke="#222"/>
    <circle cx="${CX}" cy="${CY}" r="24.5" fill="none" stroke="#3a3c40" stroke-width="3" stroke-dasharray="1.5 1.8"/>
    <circle cx="${CX}" cy="${CY}" r="17" fill="#2b2d31" stroke="#555"/>
    <g transform="rotate(${a} ${CX} ${CY})"><rect x="${CX - 1.6}" y="${CY - 15}" width="3.2" height="12" rx="1.5" fill="#fff"/></g>
    <text x="${CX}" y="${CY + 50}" text-anchor="middle" font-size="10" fill="#9a9a9a">${Math.round(v * 100)} %</text>`;
}

function renderPush(c) {
  const lit = isLit(c.def, c.st);
  const left = Math.max(0, s.spiegelUntil - Date.now());
  const mm = Math.floor(left / 60000), ss = String(Math.floor(left / 1000) % 60).padStart(2, '0');
  c.svg.innerHTML = `<circle cx="${CX}" cy="${CY + 3}" r="27" fill="#0006"/>
    <circle cx="${CX}" cy="${CY}" r="26" fill="url(#nut)" stroke="#222"/>
    <circle cx="${CX}" cy="${CY}" r="20" fill="none" stroke="${lit ? c.def.led : shade(c.def.led)}" stroke-width="4"${lit ? ' filter="url(#glow)"' : ''}/>
    <circle cx="${CX}" cy="${CY}" r="16" fill="url(#knob)" stroke="#333"/>
    <text x="${CX}" y="${CY + 50}" text-anchor="middle" font-size="10" fill="#9a9a9a">${lit ? `${mm}:${ss}` : 'aus'}</text>`;
}

function renderJoy(c) {
  const { jx, jy } = c.st;
  const bx = CX, by = CY, kx = bx + jx * 13, ky = by + jy * 13;
  c.svg.innerHTML = `<rect x="16" y="${by - 30}" width="60" height="60" rx="10" fill="#151618" stroke="#000"/>
    <circle cx="${bx}" cy="${by}" r="23" fill="#0a0a0b" stroke="#2a2b2e"/>
    <circle cx="${kx + 1.5}" cy="${ky + 2.5}" r="17" fill="#000a"/>
    <circle cx="${kx}" cy="${ky}" r="17" fill="#2b2c30" stroke="#111"/>
    <circle cx="${kx}" cy="${ky}" r="13" fill="#1d1e21" stroke="#3a3c40" stroke-width="2" stroke-dasharray="2 2"/>
    <circle cx="${kx}" cy="${ky}" r="7" fill="#151618"/>
    <text x="${bx}" y="${by - 34}" text-anchor="middle" font-size="8" fill="#888">TILT ▲</text>
    <text x="${bx}" y="${by + 50}" text-anchor="middle" font-size="9" fill="#9a9a9a">${on('suchAn') ? `P ${Math.round(s.pan)}° T ${Math.round(s.tilt)}°` : 'Suchlicht aus'}</text>`;
}

function render(c) {
  if (c.def.type === 'joy') renderJoy(c);
  else if (c.def.type === 'pot') renderPot(c);
  else if (c.def.type === 'push') renderPush(c);
  else if (c.def.type !== 'reserve') renderToggle(c);
}

// ---------- interaction ----------
function attach(c) {
  const { def, st, svg } = c;
  const localY = (e) => {
    const r = svg.getBoundingClientRect();
    return ((e.clientY - r.top) / r.height) * 130;
  };
  svg.addEventListener('pointerenter', () => info(def));
  if (def.type === 'pot') {
    svg.addEventListener('pointerdown', (e) => {
      svg.setPointerCapture(e.pointerId);
      c.drag = { y: e.clientY, v: st.v };
    });
    svg.addEventListener('pointermove', (e) => {
      if (!c.drag) return;
      st.v = Math.min(1, Math.max(0, c.drag.v - (e.clientY - c.drag.y) / 160));
      update();
    });
    svg.addEventListener('pointerup', () => { c.drag = null; });
    return;
  }
  if (def.type === 'push') {
    svg.addEventListener('pointerdown', () => {
      if (Date.now() < s.spiegelUntil) { s.spiegelUntil = 0; log('Spiegelheizung aus'); }
      else { s.spiegelUntil = Date.now() + 10 * 60 * 1000; log('Spiegelheizung 10 min'); }
      update();
    });
    return;
  }
  if (def.type === 'joy') {
    const move = (e) => {
      const r = svg.getBoundingClientRect();
      let x = ((e.clientX - r.left) / r.width * 92 - CX) / 13, y = ((e.clientY - r.top) / r.height * 130 - CY) / 13;
      const m = Math.hypot(x, y);
      if (m > 1) { x /= m; y /= m; }
      st.jx = x; st.jy = y;
      render(c);
    };
    svg.addEventListener('pointerdown', (e) => { svg.setPointerCapture(e.pointerId); c.drag = true; move(e); });
    svg.addEventListener('pointermove', (e) => { if (c.drag) move(e); });
    const stop = () => { c.drag = false; st.jx = 0; st.jy = 0; render(c); };
    svg.addEventListener('pointerup', stop);
    svg.addEventListener('pointercancel', stop);
    return;
  }
  if (def.type === 'reserve') return;
  svg.addEventListener('pointerdown', (e) => {
    info(def);
    const y = localY(e);
    if (def.cover && !st.open) { st.open = true; update(); return; }
    if (def.cover && y < 18) { st.open = false; st.pos = def.type === 'B' || def.type === 'A' || def.type === 'C' || def.type === 'T' ? 'down' : st.pos; act(def, 'cover zu'); update(); return; }
    svg.setPointerCapture(e.pointerId);
    if (def.type === 'A' || def.type === 'C') st.pos = st.pos === 'up' ? 'down' : 'up';
    else if (def.type === 'B') {
      const order = ['down', 'mid', 'up'];
      const i = order.indexOf(st.pos) + (y < CY ? 1 : -1);
      st.pos = order[Math.min(2, Math.max(0, i))];
    } else if (def.type === 'T') st.pos = 'up';
    else if (def.type === 'T2') st.pos = y < CY ? 'up' : 'down';
    act(def);
    update();
  });
  const release = () => {
    if (def.type === 'T' && st.pos === 'up') { st.pos = 'down'; update(); }
    if (def.type === 'T2' && st.pos !== 'mid') { st.pos = 'mid'; act(def); update(); }
  };
  svg.addEventListener('pointerup', release);
  svg.addEventListener('pointercancel', release);
}

function label(def, pos) {
  if (def.type === 'B') return def.pos[['down', 'mid', 'up'].indexOf(pos)];
  return pos === 'up' ? def.pos[1] : def.pos[0];
}

function act(def, why) {
  const st = controls[def.id].st;
  if (def.id === 'start' && st.pos === 'up') log(on('zuendung') ? 'Start → Kl. 50 (nur bei BEREIT)' : 'Start ohne Zündung: keine Wirkung');
  else if (def.type !== 'T') log(`${def.label}${def.sub ? ' ' + def.sub : ''}: ${label(def, st.pos)}${why ? ' (' + why + ')' : ''}`);
}

const on = (id) => controls[id].st.pos === 'up';

// ---------- panels ----------
const saved = JSON.parse(localStorage.getItem('cockpitLayout') || '{}');

function buildPanel(p) {
  const el = document.createElement('div');
  el.className = 'panel' + (p.unlit ? ' unlit' : '');
  const pos = saved[p.id] || { x: p.x, y: p.y };
  el.style.left = pos.x + 'px';
  el.style.top = pos.y + 'px';
  el.innerHTML = `<h2>${p.title}</h2>`;
  for (const [l, t] of [['6px', '9px'], ['', '9px'], ['6px', ''], ['', '']]) {
    const sc = document.createElement('div');
    sc.className = 'screw';
    sc.style[l ? 'left' : 'right'] = '6px';
    sc.style[t ? 'top' : 'bottom'] = '6px';
    el.appendChild(sc);
  }
  const grid = document.createElement('div');
  grid.className = 'grid';
  grid.style.gridTemplateColumns = `repeat(${p.cols}, 92px)`;
  for (const def of p.items) {
    const cell = document.createElement('div');
    cell.className = 'cell' + (def.type === 'reserve' ? ' reserve' : '');
    const h = 130;
    const w = 92;
    const svg = document.createElementNS(NS, 'svg');
    svg.setAttribute('width', w);
    svg.setAttribute('height', h);
    svg.setAttribute('viewBox', `0 0 ${w} ${h}`);
    if (def.type === 'reserve') {
      svg.innerHTML = `<circle cx="${CX}" cy="${CY}" r="17" fill="#0a0a0a" stroke="#444" stroke-dasharray="3 3"/>`;
      svg.style.cursor = 'help';
    }
    cell.appendChild(svg);
    const plate = document.createElement('div');
    plate.className = 'plate';
    plate.innerHTML = `${def.label}${def.sub ? `<small>${def.sub}</small>` : ''}`;
    cell.appendChild(plate);
    const type = document.createElement('div');
    type.className = 'type';
    type.textContent = { A: 'A', B: 'B', C: 'C', T: 'T', T2: 'T2', push: 'Druckknopf', pot: 'Poti', joy: 'Gamepad-Stick', reserve: '' }[def.type] + (def.cover ? (def.clearCap ? ' + Kappe klar' : ' + Kappe') : '');
    cell.appendChild(type);
    grid.appendChild(cell);
    const st = { pos: def.type === 'T2' ? 'mid' : 'down', open: false, v: 0.4, jx: 0, jy: 0 };
    controls[def.id] = { def, st, svg };
    attach(controls[def.id]);
    if (def.type === 'reserve') svg.addEventListener('pointerenter', () => info(def));
  }
  el.appendChild(grid);
  draggable(el, el.querySelector('h2'), p.id);
  dash.appendChild(el);
}

let scale = 1;
function draggable(el, handle, key) {
  handle.addEventListener('pointerdown', (e) => {
    if (e.target.closest('.wbtn, .stalkbtn, #paddles')) return;
    handle.setPointerCapture(e.pointerId);
    const start = { x: e.clientX, y: e.clientY, l: parseFloat(el.style.left), t: parseFloat(el.style.top) };
    const move = (ev) => {
      el.style.left = start.l + (ev.clientX - start.x) / scale + 'px';
      el.style.top = start.t + (ev.clientY - start.y) / scale + 'px';
    };
    const up = () => {
      handle.removeEventListener('pointermove', move);
      handle.removeEventListener('pointerup', up);
      saved[key] = { x: parseFloat(el.style.left), y: parseFloat(el.style.top) };
      localStorage.setItem('cockpitLayout', JSON.stringify(saved));
    };
    handle.addEventListener('pointermove', move);
    handle.addEventListener('pointerup', up);
  });
}

// ---------- steering wheel + stalks ----------
function buildWheel() {
  const wrap = document.createElement('div');
  wrap.id = 'wheelWrap';
  const pos = saved.wheel || { x: 550, y: 180 };
  wrap.style.left = pos.x + 'px';
  wrap.style.top = pos.y + 'px';
  const cx = 340, cy = 300, r = 210;
  const a = Math.asin(180 / r), dx = r * Math.cos(a);
  const stalkBtn = (id, x, y, w, text) => `<g class="stalkbtn" data-id="${id}" transform="translate(${x} ${y})"><rect width="${w}" height="24" rx="5"/><text x="${w / 2}" y="16" text-anchor="middle" font-size="11" fill="#ddd">${text}</text></g>`;
  let svg = `<svg width="680" height="600" viewBox="0 0 680 600">
    <!-- stalks behind the wheel -->
    <g>
      <path d="M150 360 L18 400" stroke="#111" stroke-width="22" stroke-linecap="round"/>
      <path d="M150 360 L18 400" stroke="#2c2d30" stroke-width="16" stroke-linecap="round"/>
      <path d="M530 360 L662 400" stroke="#111" stroke-width="22" stroke-linecap="round"/>
      <path d="M530 360 L662 400" stroke="#2c2d30" stroke-width="16" stroke-linecap="round"/>
      <text x="20" y="432" font-size="12" font-weight="700" fill="#bbb">Lichthebel (Corsa C)</text>
      ${stalkBtn('blinkL', 10, 442, 78, '◀ Blinker')}${stalkBtn('blinkR', 94, 442, 78, 'Blinker ▶')}
      ${stalkBtn('fern', 10, 472, 78, 'Fernlicht')}${stalkBtn('flash', 94, 472, 78, 'Lichthupe')}
      <text x="660" y="432" font-size="12" font-weight="700" fill="#bbb" text-anchor="end">Wischer → Winde</text>
      ${stalkBtn('wWeg', 508, 442, 78, 'Winde aus')}${stalkBtn('wHer', 592, 442, 78, 'Winde ein')}
      ${stalkBtn('wStufe', 508, 472, 162, 'Stufe 1 (rastend)')}
    </g>
    <!-- shift paddles (big, plus/minus) and hall paddles (small, below) sit behind the rim -->
    ${WHEEL.filter((b) => b.paddle).map((b) => {
      const x = b.paddle === 'L' ? 22 : 563;
      const tx = b.paddle === 'L' ? 62 : 618;
      return `<g class="wbtn" data-id="${b.id}">
        <rect x="${x}" y="130" width="95" height="150" rx="22" fill="#000" transform="translate(0 3)"/>
        <rect x="${x}" y="130" width="95" height="150" rx="22" fill="#5a5d62" stroke="#111"/>
        <rect x="${x + 6}" y="136" width="83" height="20" rx="10" fill="#ffffff22"/>
        <text x="${tx}" y="215" text-anchor="middle" font-size="40" font-weight="700" fill="#fff">${b.paddle === 'L' ? '−' : '+'}</text>
        <text x="${tx}" y="242" text-anchor="middle" font-size="11" fill="#ddd">${b.fn}</text>
      </g>`;
    }).join('')}
    <g id="paddles" style="cursor:help">
      <rect x="44" y="290" width="70" height="58" rx="14" fill="#2e3034" stroke="#111"/>
      <rect x="566" y="290" width="70" height="58" rx="14" fill="#2e3034" stroke="#111"/>
      <text x="74" y="314" text-anchor="middle" font-size="10" fill="#aaa">Hall L</text><text x="74" y="328" text-anchor="middle" font-size="10" fill="#e8c547">unbelegt</text>
      <text x="606" y="314" text-anchor="middle" font-size="10" fill="#aaa">Hall R</text><text x="606" y="328" text-anchor="middle" font-size="10" fill="#e8c547">unbelegt</text>
    </g>
    <!-- rim + spokes -->
    <path d="M${cx} ${cy} L130 300 M${cx} ${cy} L550 300 M${cx} ${cy} L${cx} 480" stroke="#26272a" stroke-width="44"/>
    <path d="M${cx - dx} ${cy + 180} A${r} ${r} 0 1 1 ${cx + dx} ${cy + 180} Z" fill="none" stroke="#050505" stroke-width="52" stroke-linejoin="round"/>
    <path d="M${cx - dx} ${cy + 180} A${r} ${r} 0 1 1 ${cx + dx} ${cy + 180} Z" fill="none" stroke="#1d1d1f" stroke-width="44" stroke-linejoin="round"/>
    <path d="M${cx - dx} ${cy + 180} A${r} ${r} 0 1 1 ${cx + dx} ${cy + 180} Z" fill="none" stroke="#2a2a2d" stroke-width="44" stroke-dasharray="2 3" stroke-linejoin="round" opacity=".5"/>
    <rect x="${cx - 6}" y="${cy - r - 22}" width="12" height="44" fill="#ff7a00"/>
    <rect x="196" y="196" width="288" height="240" rx="34" fill="url(#carbon)" stroke="#000" stroke-width="2"/>
    <rect x="202" y="202" width="276" height="228" rx="30" fill="none" stroke="#ffffff10"/>`;
  for (const b of WHEEL.filter((w) => !w.paddle)) {
    svg += `<g class="wbtn" data-id="${b.id}">
      <circle cx="${b.x}" cy="${b.y + 2}" r="${b.r + 3}" fill="#000"/>
      <circle cx="${b.x}" cy="${b.y}" r="${b.r + 2}" fill="url(#chrome)"/>
      <circle cx="${b.x}" cy="${b.y}" r="${b.r}" fill="${b.fill}"/>
      <ellipse cx="${b.x - b.r * 0.3}" cy="${b.y - b.r * 0.4}" rx="${b.r * 0.45}" ry="${b.r * 0.22}" fill="#ffffff35"/>
      ${b.text ? `<text x="${b.x}" y="${b.y + (b.r > 20 ? 5 : 6)}" text-anchor="middle" font-size="${b.r > 20 ? 14 : 17}" font-weight="700" fill="#fff">${b.text}</text>` : ''}
      ${b.id !== 'hupe' ? (() => { const c = b.below || b.x === cx; return `<text x="${c ? b.x : b.x + (b.x < cx ? -24 : 24)}" y="${b.y + (c ? 32 : 4)}" text-anchor="${c ? 'middle' : b.x < cx ? 'end' : 'start'}" font-size="10.5" fill="${b.fn.startsWith('offen') ? '#e8c547' : '#ccc'}">${b.fn}</text>`; })() : ''}
    </g>`;
  }
  for (const p of WHEEL_POTS) {
    svg += `<g class="wbtn" data-id="${p.id}">
      <circle cx="${p.x}" cy="${p.y}" r="19" fill="url(#knob)" stroke="#111"/>
      <circle cx="${p.x}" cy="${p.y}" r="17" fill="none" stroke="#3a3c40" stroke-width="3" stroke-dasharray="1.5 1.8"/>
      <g class="ptr" transform="rotate(0 ${p.x} ${p.y})"><rect x="${p.x - 1.5}" y="${p.y - 15}" width="3" height="9" rx="1.5" fill="#fff"/></g>
      <text x="${p.x}" y="${p.y + 34}" text-anchor="middle" font-size="10.5" fill="#ccc">${p.fn}</text>
    </g>`;
  }
  svg += '</svg>';
  wrap.innerHTML = svg;
  dash.appendChild(wrap);

  wrap.querySelector('#paddles').addEventListener('pointerenter', () => info({ type: 'wheel', fn: 'Hall-Paddel L/R', info: 'Kleine Paddel unter den Schaltwippen: Simube Doppelpaddel (analog, Hall). Vorerst unbelegt – nicht zum Schalten, nicht zusammengelegt.' }));
  for (const g of wrap.querySelectorAll('.wbtn')) {
    const id = g.dataset.id;
    const b = WHEEL.find((x) => x.id === id) || WHEEL_POTS.find((x) => x.id === id);
    g.addEventListener('pointerenter', () => info({ ...b, type: 'wheel', label: b.fn }));
    if (id.startsWith('pot')) {
      const st = { v: 0.5 };
      const ptr = g.querySelector('.ptr');
      const draw = () => ptr.setAttribute('transform', `rotate(${-135 + st.v * 270} ${b.x} ${b.y})`);
      draw();
      g.addEventListener('pointerdown', (e) => { g.setPointerCapture(e.pointerId); g.drag = { y: e.clientY, v: st.v }; });
      g.addEventListener('pointermove', (e) => { if (g.drag) { st.v = Math.min(1, Math.max(0, g.drag.v - (e.clientY - g.drag.y) / 160)); draw(); } });
      g.addEventListener('pointerup', () => { if (g.drag) log(`${b.fn}: ${Math.round(st.v * 100)} %`); g.drag = null; update(); });
      continue;
    }
    g.addEventListener('pointerdown', () => wheelPress(b));
  }
  for (const g of wrap.querySelectorAll('.stalkbtn')) {
    const id = g.dataset.id;
    g.addEventListener('pointerenter', () => info(stalkInfo(id)));
    g.addEventListener('pointerdown', (e) => {
      if (id === 'blinkL') { s.blinkL = !s.blinkL; s.blinkR = false; }
      if (id === 'blinkR') { s.blinkR = !s.blinkR; s.blinkL = false; }
      if (id === 'fern') s.fern = !s.fern;
      if (id === 'flash') s.flash = true;
      if (id === 'wStufe') log('Wischerhebel Stufe 1: noch unbelegt');
      if (id === 'wWeg' || id === 'wHer') {
        g.setPointerCapture(e.pointerId);
        if (on('windeFrei')) { s.winch = id === 'wHer' ? 'EIN' : 'AUS'; g.classList.add('on'); log(`Winde ${s.winch}`); }
        else log('Winde gesperrt (Freigabe aus)');
      }
      update();
    });
    const release = () => {
      if (id === 'flash') { s.flash = false; update(); }
      if ((id === 'wWeg' || id === 'wHer') && s.winch) { s.winch = null; g.classList.remove('on'); log('Winde stopp'); update(); }
    };
    g.addEventListener('pointerup', release);
    g.addEventListener('pointercancel', release);
  }
  draggable(wrap, wrap.querySelector('svg'), 'wheel');
}

function stalkInfo(id) {
  const m = {
    blinkL: 'Blinker links. Antippen (< 400 ms) = 3× Spurwechsel-Blinken.', blinkR: 'Blinker rechts. Antippen = 3× Spurwechsel-Blinken.',
    fern: 'Fernlicht als Toggle in Software, nur bei Abblendlicht. Zusatz-Fernlicht folgt, wenn freigegeben.', flash: 'Lichthupe, geht immer.',
    wWeg: 'Wischerhebel wegdrücken (tastend) = Winde auslassen, nur mit Winden-Freigabe. Ersetzt den Kippschalter am Panel.', wHer: 'Wischerhebel herziehen (tastend) = Winde einziehen, nur mit Winden-Freigabe.',
    wStufe: 'Stufe 1 rastet: offen. Idee: Zusatz-Rückfahrlicht zum Rangieren.',
  };
  return { type: 'stalk', fn: id.startsWith('w') ? 'Wischerhebel' : 'Lichthebel', info: m[id] };
}

function wheelPress(b) {
  if (b.id === 'plus' || b.id === 'minus') {
    if (s.gear === 'N') s.gear = b.id === 'plus' ? '1' : 'R';
    else {
      const i = SHIFT_ORDER.indexOf(s.gear) + (b.id === 'plus' ? 1 : -1);
      s.gear = SHIFT_ORDER[Math.min(SHIFT_ORDER.length - 1, Math.max(0, i))];
    }
    log(`Gang ${s.gear}`);
  } else if (b.id === 'schwarz') { s.gear = 'N'; log('Gang N'); }
  else if (b.id === 'hupe') log('HUPE');
  else if (b.id === 'gelb') { s.warn = !s.warn; log(`Warnblinker ${s.warn ? 'AN' : 'AUS'}`); }
  else if (b.id === 'rot') {
    if (Date.now() < s.spiegelUntil) { s.spiegelUntil = 0; log('Spiegelheizung aus'); }
    else { s.spiegelUntil = Date.now() + 10 * 60 * 1000; log('Spiegelheizung 10 min'); }
  }
  else log(b.fn.startsWith('offen') ? `Lenkrad ${b.id}: noch unbelegt` : b.fn);
  update();
}

// ---------- status display ----------
function buildStatus() {
  const el = document.createElement('div');
  el.id = 'status';
  el.innerHTML = `<div id="road"></div><div id="events"></div><div class="arrow" id="arrL">◀</div><div id="gear">N</div><div class="arrow" id="arrR">▶</div>
    <div id="tells"></div>`;
  dash.appendChild(el);
  const i = document.createElement('div');
  i.id = 'info';
  i.innerHTML = '<b>Fahrersicht</b> – Lenkrad mittig, Lichtpanel links, Motor &amp; Zubehör und Entertainment/Audio rechts (Anordnung frei verschiebbar). Maus über ein Bedienelement zeigt Funktion und Status aus Bordnetz-Konzept.md.';
  dash.appendChild(i);
}

const TELLS = [
  ['ZÜND', GREEN, () => on('zuendung')],
  ['STAND', GREEN, () => controls.licht.st.pos !== 'down'],
  ['FERN', BLUE, () => fernActive()],
  ['NEBEL', GREEN, () => controls.nebel.st.pos !== 'down' && controls.licht.st.pos !== 'down'],
  ['NSL', AMBER, () => controls.nebel.st.pos === 'up' && controls.licht.st.pos !== 'down'],
  ['LÜFT M', BLUE, () => on('luefter')],
  ['KOMPR', YELLOW, () => controls.kompressor.st.pos !== 'mid'],
  ['WINDE', YELLOW, () => on('windeFrei')],
  ['SPIEGEL', AMBER, () => Date.now() < s.spiegelUntil],
  ['RÜCKF', RED, () => rueckActive()],
];
const rueckActive = () => { const p = controls.rueck.st.pos; return p === 'up' || (p === 'mid' && s.gear === 'R'); };
const fernActive = () => (s.fern && controls.licht.st.pos === 'up') || s.flash;

function updateStatus() {
  document.getElementById('gear').textContent = s.gear;
  const warn = s.warn;
  document.getElementById('arrL').classList.toggle('on', (s.blinkL || warn) && s.phase);
  document.getElementById('arrR').classList.toggle('on', (s.blinkR || warn) && s.phase);
  document.getElementById('tells').innerHTML = TELLS.map(([t, col, f], i) => {
    const lit = f();
    return `<span class="tell${lit ? ' on' : ''}" style="left:${6 + i * 51}px;${lit ? `background:${col};box-shadow:0 0 8px ${col}` : ''}">${t}</span>`;
  }).join('');
  const bad = PANELS.flatMap((p) => p.items).filter((d) => d.illegal && on(d.id)).map((d) => `${d.label} ${d.sub || ''}`.trim());
  if (rueckActive()) bad.push(controls.rueck.st.pos === 'up' ? 'Rückfahr weiß AN' : 'Rückfahr weiß (R-Auto, R drin)');
  const road = document.getElementById('road');
  road.classList.toggle('bad', bad.length > 0);
  road.innerHTML = bad.length ? `<b>Vor der Straße aus:</b><br>${bad.join('<br>')}` : '<b>Straße: ok</b><br>kein Offroad-/Sonderlicht an';
  const komp = { down: 'AUTO', mid: 'AUS', up: 'AN' }[controls.kompressor.st.pos];
  document.getElementById('events').innerHTML = `Kompressor ${komp}${on('zusatz') && fernActive() ? '<br>Zusatz-Fern AN' : ''}${s.winch ? `<br><b style="color:#ffe033">Winde ${s.winch}</b>` : ''}<br><span style="color:#666">${s.log.join('<br>')}</span>`;
}

function update() {
  for (const c of Object.values(controls)) render(c);
  updateStatus();
}

function fit() {
  const st = document.getElementById('stage');
  scale = Math.min(st.clientWidth / W, st.clientHeight / H);
  dash.style.transform = `scale(${scale})`;
}

for (const p of PANELS) buildPanel(p);
buildWheel();
buildStatus();
document.getElementById('reset').onclick = () => { localStorage.removeItem('cockpitLayout'); location.reload(); };
window.addEventListener('resize', fit);
fit();
update();
setInterval(() => { s.phase = !s.phase; update(); }, 333);
setInterval(() => {
  const j = controls.joy;
  if (!on('suchAn') || (!j.st.jx && !j.st.jy)) return;
  s.pan = Math.max(-180, Math.min(180, s.pan + j.st.jx * 60 * 0.05));
  s.tilt = Math.max(-30, Math.min(60, s.tilt - j.st.jy * 40 * 0.05));
  render(j);
}, 50);
