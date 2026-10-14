// Shift controller state machine + simple cylinder physics.
// Model from Vault: Buggy/Druckluftanlage.md (3 gates 1/2, 3/4, 5/R; cascade A+B for rows).
// Units: mm, s.

export const GATE_X = [0, 32, 64];
export const GATE_TRAVEL = 64;
export const A_STROKE = 14; // bottom row -> neutral
export const B_STROKE = 34; // neutral -> top row
export const NEUTRAL_Z = A_STROKE;

export const GEARS = {
  R: { gate: 2, row: 'bottom' },
  N: { gate: null, row: 'neutral' },
  1: { gate: 0, row: 'top' },
  2: { gate: 0, row: 'bottom' },
  3: { gate: 1, row: 'top' },
  4: { gate: 1, row: 'bottom' },
  5: { gate: 2, row: 'top' },
};
export const SEQUENCE = ['R', 'N', '1', '2', '3', '4', '5'];
export const STATES = ['IDLE', 'TO_NEUTRAL', 'GATE_MOVE', 'ROW_ENGAGE', 'CORRECT', 'DONE', 'FAULT'];

export const DEFAULTS = {
  P_12: 10,
  P_WIN_LO: 45,
  P_WIN_HI: 55,
  P_5R: 90,
  ventOffset: 0,     // % past the window edge before G is vented
  MAX_FAILS: 4,
  gSpeed: 250,       // mm/s, set by flow control on G
  gAccel: 8000,      // mm/s^2 while pressurized
  gDecel: 5000,      // mm/s^2 while vented (friction)
  rowSpeed: 300,     // mm/s for A and B
  valveDelay: 0.015, // s, M-04-533 switching time
  timeout: 1.0,      // s per state
  captureTol: 2.5,   // mm: row can leave neutral only this close to a gate
  neutralTol: 1.5,   // mm: G can move only this close to neutral
  sensorTol: 0.5,    // mm
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
    x: GATE_X[1], vx: 0, a: A_STROKE, b: 0,
    cmd: { G: 'vent', A: 'vent', B: 'vent' },
    valve: { G: 'vent', A: 'vent', B: 'vent' },
    pending: [], approach: 0, log: [],
  };

  const poti = () => (s.x / GATE_TRAVEL) * 100;
  const gateOf = () => GATE_X.findIndex((g) => Math.abs(s.x - g) <= p.captureTol);
  const sensors = () => ({
    Ain: s.a <= p.sensorTol, Aout: s.a >= A_STROKE - p.sensorTol,
    Bin: s.b <= p.sensorTol, Bout: s.b >= B_STROKE - p.sensorTol,
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
    if (g === 'N') enter('TO_NEUTRAL');
    else if (gateOf() === GEARS[g].gate) enter('ROW_ENGAGE', 'gleiche Gasse');
    else if (rowOf() === 'neutral') enter('GATE_MOVE');
    else enter('TO_NEUTRAL');
    return true;
  }

  function control() {
    const pt = poti();
    const tgt = s.target ? GEARS[s.target] : null;
    if (['TO_NEUTRAL', 'GATE_MOVE', 'ROW_ENGAGE', 'CORRECT'].includes(s.state) && s.stateT > p.timeout) {
      fault(`Timeout in ${s.state}`);
      return;
    }
    switch (s.state) {
      case 'IDLE':
        ventAll();
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
        if (tg === 0) setValve('G', 'ret');
        else if (tg === 2) setValve('G', 'ext');
        else {
          if (s.approach === 0) s.approach = s.x < GATE_X[1] ? 1 : -1;
          setValve('G', s.approach > 0 ? 'ext' : 'ret');
        }
        const reached =
          tg === 0 ? pt < p.P_12 :
          tg === 2 ? pt > p.P_5R :
          s.approach > 0 ? pt >= p.P_WIN_LO : pt <= p.P_WIN_HI;
        if (reached) enter('ROW_ENGAGE', `Poti ${pt.toFixed(1)} %`);
        break;
      }
      case 'ROW_ENGAGE': {
        setRow(tgt.row);
        const engaged = rowOf() === tgt.row;
        if (tgt.gate === 1 && s.approach !== 0) {
          if (s.approach > 0 && pt >= p.P_WIN_LO + p.ventOffset) setValve('G', 'vent');
          if (s.approach < 0 && pt <= p.P_WIN_HI - p.ventOffset) setValve('G', 'vent');
          const over = s.approach > 0 ? pt > p.P_WIN_HI : pt < p.P_WIN_LO;
          if (over && !engaged) {
            s.fails += 1;
            log(`Überschwingen ${s.fails}/${p.MAX_FAILS} (Poti ${pt.toFixed(1)} %)`);
            if (s.fails >= p.MAX_FAILS) fault(`${p.MAX_FAILS} Fehlversuche`);
            else enter('CORRECT');
            break;
          }
        }
        if (engaged) enter('DONE');
        break;
      }
      case 'CORRECT': {
        setRow('neutral');
        if (pt > p.P_WIN_HI) setValve('G', 'ret');
        else if (pt < p.P_WIN_LO) setValve('G', 'ext');
        else setValve('G', 'vent');
        if (pt >= p.P_WIN_LO && pt <= p.P_WIN_HI && rowOf() === 'neutral') {
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
        enter('IDLE', `Gang ${s.gear}`);
        break;
      case 'FAULT':
        setValve('G', 'vent');
        setRow('neutral');
        if (rowOf() === 'neutral') {
          ventAll();
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
    if (s.x <= 0 || s.x >= GATE_TRAVEL) {
      s.x = clamp(s.x, 0, GATE_TRAVEL);
      s.vx = 0;
    }

    const vel = (k) => (s.valve[k] === 'ext' ? p.rowSpeed : s.valve[k] === 'ret' ? -p.rowSpeed : 0);
    const na = clamp(s.a + vel('A') * dt, 0, A_STROKE);
    const nb = clamp(s.b + vel('B') * dt, 0, B_STROKE);
    const dNew = Math.abs(na + nb - NEUTRAL_Z);
    const blocked = gateOf() === -1 && dNew > p.neutralTol && dNew > Math.abs(z - NEUTRAL_Z);
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
    const i = SEQUENCE.indexOf(s.gear);
    const j = clamp(i + dir, 0, SEQUENCE.length - 1);
    return j !== i && request(SEQUENCE[j]);
  }

  return {
    params: p,
    state: s,
    step,
    request,
    plus: () => shift(1),
    minus: () => shift(-1),
    poti,
    sensors,
    rowOf,
  };
}
