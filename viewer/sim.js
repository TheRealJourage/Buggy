// Shift controller state machine + simple cylinder physics.
// Model from Vault: Buggy/Druckluftanlage.md (4 gates R | 1/2 | 3/4 | 5; cascade A+B for rows).
// R and 5 sit at the outer gate stops of the gearbox and only have a top row; 1/2 and 3/4 are engaged on the fly.
// Units: mm, s. Travel = measured at the cable eye (Tafel 2026-10-15), sizing: "Neuberechnung 2026-10-24".

export const G_STROKE = 40; // standard stroke, gate range is 27 mm
const GATE_ABS = [13.7, 18.0, 27.5, 40.7]; // measured gate positions R, 1/2, 3/4, 5
const G_ZERO = (GATE_ABS[0] + GATE_ABS[3]) / 2 - G_STROKE / 2; // stroke centered, 6.5 mm reserve at R and 5
export const GATE_X = GATE_ABS.map((v) => Math.round((v - G_ZERO) * 10) / 10); // gearbox gate stops, not cylinder ends
export const GATE_TRAVEL = G_STROKE;
export const LAST_GATE = GATE_X.length - 1;
export const GATE_NAMES = ['R', '1/2', '3/4', '5'];
// Standard strokes only (User 2026-10-24): A HM-25-040 limited to 28 mm by a stop sleeve on the rod.
export const A_CYL_STROKE = 40;
export const A_STOP = 12; // stop sleeve takes 12 mm of the retraction
export const A_STROKE = A_CYL_STROKE - A_STOP; // bottom row (2/4 at 9.0 mm) -> neutral (mean 36.8 mm)
export const B_STROKE = 40; // neutral -> top row, top is stopped by the gearbox before B ends
export const ROW_TOP = [22.5, 21.8, 28.1, 25.2]; // B travel neutral -> engaged top gear per gate
export const BOUT_AT = 20.8; // sensor B "out" sits below the shortest top travel
export const NEUTRAL_Z = A_STROKE;

export const GEARS = {
  R: { gate: 0, row: 'top' },
  N: { gate: null, row: 'neutral' },
  1: { gate: 1, row: 'top' },
  2: { gate: 1, row: 'bottom' },
  3: { gate: 2, row: 'top' },
  4: { gate: 2, row: 'bottom' },
  5: { gate: 3, row: 'top' },
};
export const SEQUENCE = ['R', 'N', '1', '2', '3', '4', '5'];
// Plus/minus skip N so R <-> 1 is one click when manoeuvring; N has its own button.
export const SHIFT_ORDER = ['R', '1', '2', '3', '4', '5'];
export const STATES = ['IDLE', 'CLUTCH_OPEN', 'TO_NEUTRAL', 'GATE_MOVE', 'ROW_ENGAGE', 'CORRECT', 'DONE', 'CLUTCH_CLOSE', 'FAULT'];
export const CLUTCH_TRAVEL = 58; // mm at the pedal tip (55-60, Halle 2026-10-23)
const SHIFTING = ['TO_NEUTRAL', 'GATE_MOVE', 'ROW_ENGAGE', 'CORRECT'];

const pct = (x) => Math.round((x / G_STROKE) * 1000) / 10;

export const DEFAULTS = {
  P_END_LO: pct(GATE_X[0] + 1),          // poti % below = gate R reached
  P_END_HI: pct(GATE_X[LAST_GATE] - 1),  // poti % above = gate 5 reached
  WIN_HALF: pct(1.5),                    // middle gates: window = center ± 1.5 mm
  ventOffset: 0,     // % past the window edge before G is vented
  MAX_FAILS: 4,
  gSpeed: 100,       // mm/s, set by flow control on G
  gAccel: 8000,      // mm/s^2 while pressurized
  gDecel: 5000,      // mm/s^2 while vented (friction)
  rowSpeed: 300,     // mm/s for A and B
  valveDelay: 0.015, // s, M-04-533 switching time
  timeout: 1.0,      // s per state
  captureTol: 2.0,   // mm: row can leave neutral only this close to a gate (assumed)
  neutralTol: 1.5,   // mm: G can move only this close to neutral
  sensorTol: 0.5,    // mm
  holdTime: 0.05,    // s: keep row pressure after the sensor, top gears end on the gearbox stop
  autoClutch: true,  // false: driver presses the pedal, controller only checks it
  kSpeed: 400,       // mm/s clutch cylinder K pressing the pedal
  kRelease: 150,     // mm/s pedal return, throttled (engagement speed)
  pedalSpeed: 250,   // mm/s driver foot
  clutchWait: 3.0,   // s manual mode: wait this long for the pedal, then drop the command
  clutchTol: 3,      // mm: pedal sensors "getrennt" / "eingekuppelt"
};

const ROW_VALVES = {
  bottom: { A: 'ret', B: 'ret' },
  neutral: { A: 'ext', B: 'ret' },
  top: { A: 'ext', B: 'ext' },
};

const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

export function createSim(opts = {}) {
  const p = { ...DEFAULTS, ...opts };
  const s = {
    t: 0, state: 'IDLE', stateT: 0, gear: 'N', target: null, fails: 0, message: '',
    x: GATE_X[1], vx: 0, a: A_STROKE, b: 0, c: 0, foot: false, next: null, engagedAt: null,
    cmd: { G: 'vent', A: 'vent', B: 'vent', K: 'vent' },
    valve: { G: 'vent', A: 'vent', B: 'vent', K: 'vent' },
    pending: [], approach: 0, log: [],
  };

  const poti = () => (s.x / GATE_TRAVEL) * 100;
  const win = (g) => {
    const c = (GATE_X[g] / GATE_TRAVEL) * 100;
    return { c, lo: c - p.WIN_HALF, hi: c + p.WIN_HALF };
  };
  const gateOf = () => GATE_X.findIndex((g) => Math.abs(s.x - g) <= p.captureTol);
  const sensors = () => ({
    Ain: s.a <= p.sensorTol, Aout: s.a >= A_STROKE - p.sensorTol,
    Bin: s.b <= p.sensorTol, Bout: s.b >= BOUT_AT,
  });
  const clutch = () => ({
    pct: (s.c / CLUTCH_TRAVEL) * 100,
    open: s.c >= CLUTCH_TRAVEL - p.clutchTol,
    closed: s.c <= p.clutchTol,
  });
  const rowOf = () => {
    const se = sensors();
    if (se.Ain && se.Bin) return 'bottom';
    if (se.Aout && se.Bin) return 'neutral';
    if (se.Aout && se.Bout) return 'top';
    return null;
  };

  const log = (msg) => {
    s.log.push({ t: s.t, msg });
    if (s.log.length > 200) s.log.shift();
  };
  const setValve = (k, v) => {
    if (s.cmd[k] === v) return;
    s.cmd[k] = v;
    s.pending.push({ k, v, at: s.t + p.valveDelay });
  };
  const setRow = (row) => {
    setValve('A', ROW_VALVES[row].A);
    setValve('B', ROW_VALVES[row].B);
  };
  const ventAll = () => ['G', 'A', 'B'].forEach((k) => setValve(k, 'vent'));
  const enter = (st, note = '') => {
    s.state = st;
    s.stateT = 0;
    s.engagedAt = null;
    log(st + (note ? ` (${note})` : ''));
  };
  const fault = (why) => {
    s.message = why;
    enter('FAULT', why);
  };

  function request(g) {
    g = String(g);
    if (s.state !== 'IDLE' || !(g in GEARS) || g === s.gear) return false;
    s.target = g;
    s.fails = 0;
    s.message = '';
    s.approach = 0;
    log(`Befehl ${s.gear} → ${g}`);
    if (g === 'N') s.next = ['TO_NEUTRAL', ''];
    else if (gateOf() === GEARS[g].gate) s.next = ['ROW_ENGAGE', 'gleiche Gasse'];
    else if (rowOf() === 'neutral') s.next = ['GATE_MOVE', ''];
    else s.next = ['TO_NEUTRAL', ''];
    enter('CLUTCH_OPEN', p.autoClutch ? 'Zylinder K' : 'warte auf Fußkupplung');
    return true;
  }

  function control() {
    const pt = poti();
    const tgt = s.target ? GEARS[s.target] : null;
    if (SHIFTING.includes(s.state) && s.stateT > p.timeout) {
      fault(`Timeout in ${s.state}`);
      return;
    }
    if (SHIFTING.includes(s.state) && !clutch().open) {
      fault('Kupplung während der Schaltung eingekuppelt');
      return;
    }
    switch (s.state) {
      case 'IDLE':
        ventAll();
        setValve('K', 'vent');
        break;
      case 'CLUTCH_OPEN':
        if (p.autoClutch) setValve('K', 'ext');
        if (clutch().open) {
          enter(...s.next);
        } else if (p.autoClutch && s.stateT > p.timeout) {
          fault('Kupplung trennt nicht (Zylinder K)');
        } else if (!p.autoClutch && s.stateT > p.clutchWait) {
          s.message = 'Kupplung nicht getreten – Befehl verworfen';
          s.target = null;
          enter('IDLE', 'kein Gangwechsel');
        }
        break;
      case 'TO_NEUTRAL':
        setValve('G', 'vent');
        setRow('neutral');
        if (rowOf() === 'neutral') {
          if (s.target === 'N') enter('DONE');
          else enter('GATE_MOVE');
        }
        break;
      case 'GATE_MOVE': {
        const tg = tgt.gate;
        const w = win(tg);
        if (tg === 0) setValve('G', 'ret');
        else if (tg === LAST_GATE) setValve('G', 'ext');
        else {
          if (s.approach === 0) s.approach = s.x < GATE_X[tg] ? 1 : -1;
          setValve('G', s.approach > 0 ? 'ext' : 'ret');
        }
        const reached =
          tg === 0 ? pt < p.P_END_LO :
          tg === LAST_GATE ? pt > p.P_END_HI :
          s.approach > 0 ? pt >= w.lo : pt <= w.hi;
        if (reached) enter('ROW_ENGAGE', `Poti ${pt.toFixed(1)} %`);
        break;
      }
      case 'ROW_ENGAGE': {
        setRow(tgt.row);
        const engaged = rowOf() === tgt.row;
        const middle = tgt.gate > 0 && tgt.gate < LAST_GATE;
        if (middle && s.approach !== 0) {
          const w = win(tgt.gate);
          if (s.approach > 0 && pt >= w.lo + p.ventOffset) setValve('G', 'vent');
          if (s.approach < 0 && pt <= w.hi - p.ventOffset) setValve('G', 'vent');
          const over = s.approach > 0 ? pt > w.hi : pt < w.lo;
          if (over && !engaged) {
            s.fails += 1;
            log(`Überschwingen ${s.fails}/${p.MAX_FAILS} (Poti ${pt.toFixed(1)} %)`);
            if (s.fails >= p.MAX_FAILS) fault(`${p.MAX_FAILS} Fehlversuche`);
            else enter('CORRECT');
            break;
          }
        }
        if (!engaged) s.engagedAt = null;
        else if (s.engagedAt === null) s.engagedAt = s.t;
        else if (s.t - s.engagedAt >= p.holdTime) enter('DONE');
        break;
      }
      case 'CORRECT': {
        const w = win(tgt.gate);
        setRow('neutral');
        if (pt > w.hi) setValve('G', 'ret');
        else if (pt < w.lo) setValve('G', 'ext');
        else setValve('G', 'vent');
        if (pt >= w.lo && pt <= w.hi && rowOf() === 'neutral') {
          s.approach = s.vx >= 0 ? 1 : -1;
          enter('ROW_ENGAGE', 'neuer Versuch');
        }
        break;
      }
      case 'DONE':
        ventAll();
        s.gear = s.target;
        s.target = null;
        s.fails = 0;
        if (p.autoClutch) enter('CLUTCH_CLOSE', `Gang ${s.gear}`);
        else enter('IDLE', `Gang ${s.gear} – Fußkupplung loslassen`);
        break;
      case 'CLUTCH_CLOSE':
        setValve('K', 'vent'); // spring returns the pedal, speed set by the throttle
        if (clutch().closed) enter('IDLE', 'eingekuppelt');
        break;
      case 'FAULT':
        setValve('G', 'vent');
        setRow('neutral');
        if (rowOf() === 'neutral') {
          ventAll();
          setValve('K', 'vent');
          s.gear = 'N';
          s.target = null;
          enter('IDLE', 'Fehler → N, Meldung ans Cockpit');
        }
        break;
    }
  }

  function physics(dt) {
    for (const ev of s.pending.filter((e) => e.at <= s.t)) s.valve[ev.k] = ev.v;
    s.pending = s.pending.filter((e) => e.at > s.t);

    const z = s.a + s.b;
    if (Math.abs(z - NEUTRAL_Z) > p.neutralTol) {
      s.vx = 0; // a gear is (partly) engaged, gate is locked
    } else {
      const v = s.valve.G;
      if (v === 'ext') s.vx = Math.min(s.vx + p.gAccel * dt, p.gSpeed);
      else if (v === 'ret') s.vx = Math.max(s.vx - p.gAccel * dt, -p.gSpeed);
      else s.vx = Math.sign(s.vx) * Math.max(0, Math.abs(s.vx) - p.gDecel * dt);
    }
    s.x += s.vx * dt;
    if (s.x <= GATE_X[0] || s.x >= GATE_X[LAST_GATE]) {
      s.x = clamp(s.x, GATE_X[0], GATE_X[LAST_GATE]);
      s.vx = 0;
    }

    if (p.autoClutch) {
      s.c += (s.valve.K === 'ext' ? p.kSpeed : -p.kRelease) * dt;
    } else {
      s.c += (s.foot ? p.pedalSpeed : -p.pedalSpeed) * dt;
    }
    s.c = clamp(s.c, 0, CLUTCH_TRAVEL);

    const vel = (k) => (s.valve[k] === 'ext' ? p.rowSpeed : s.valve[k] === 'ret' ? -p.rowSpeed : 0);
    const na = clamp(s.a + vel('A') * dt, 0, A_STROKE);
    const g = gateOf();
    const top = g === -1 ? B_STROKE : ROW_TOP[g];
    const nb = clamp(s.b + vel('B') * dt, 0, Math.min(B_STROKE, top + NEUTRAL_Z - na));
    const dNew = Math.abs(na + nb - NEUTRAL_Z);
    const away = dNew > p.neutralTol && dNew > Math.abs(z - NEUTRAL_Z);
    const noBottom = (g === 0 || g === LAST_GATE) && na + nb < NEUTRAL_Z - p.neutralTol; // R and 5 have no bottom row
    const blocked = (g === -1 && away) || (noBottom && away);
    if (!blocked) {
      s.a = na;
      s.b = nb;
    }
  }

  function step(dt) {
    s.t += dt;
    s.stateT += dt;
    physics(dt);
    control();
  }

  function shift(dir) {
    if (s.gear === 'N') return request(dir > 0 ? '1' : 'R');
    const i = SHIFT_ORDER.indexOf(s.gear);
    const j = clamp(i + dir, 0, SHIFT_ORDER.length - 1);
    return j !== i && request(SHIFT_ORDER[j]);
  }

  return {
    params: p,
    state: s,
    step,
    request,
    plus: () => shift(1),
    minus: () => shift(-1),
    neutral: () => request('N'),
    poti,
    win,
    clutch,
    setFoot: (v) => { s.foot = v; },
    sensors,
    rowOf,
  };
}
