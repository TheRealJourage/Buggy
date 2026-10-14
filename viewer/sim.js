// Shift controller state machine + simple cylinder physics.
// Model from Vault: Buggy/Druckluftanlage.md (4 gates R | 1/2 | 3/4 | 5; cascade A+B for rows).
// R and 5 sit at the G end positions and only have a top row; 1/2 and 3/4 are engaged on the fly.
// Units: mm, s.

export const GATE_PITCH = 32; // assumed, notes say ~3-3.4 cm per gate
export const GATE_X = [0, 1, 2, 3].map((i) => i * GATE_PITCH);
export const GATE_TRAVEL = GATE_X[3];
export const LAST_GATE = GATE_X.length - 1;
export const GATE_NAMES = ['R', '1/2', '3/4', '5'];
export const A_STROKE = 14; // bottom row -> neutral
export const B_STROKE = 34; // neutral -> top row
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
export const STATES = ['IDLE', 'TO_NEUTRAL', 'GATE_MOVE', 'ROW_ENGAGE', 'CORRECT', 'DONE', 'FAULT'];

export const DEFAULTS = {
  P_END_LO: 10,      // poti % below = gate R reached
  P_END_HI: 90,      // poti % above = gate 5 reached
  WIN_HALF: 5,       // middle gates: window = center ± 5 %
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
  const win = (g) => {
    const c = (GATE_X[g] / GATE_TRAVEL) * 100;
    return { c, lo: c - p.WIN_HALF, hi: c + p.WIN_HALF };
  };
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
        if (engaged) enter('DONE');
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
    const g = gateOf();
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
    win,
    sensors,
    rowOf,
  };
}
