"""Guided acceptance run for the steering wheel with rp2040_io_test.

Reads the io_test output (COM7, 115200), logs every line with a timestamp,
reports each button press by its as-built name, tracks the analog ranges and
notes the exact moment the Tiny drops off USB (plus the last lines before it).
Waits for the port to come back and keeps going. Ctrl+C prints the summary.

    python tools/io_check.py            # COM7
    python tools/io_check.py --replay old.log
"""
import argparse
import re
import sys
import time
from datetime import datetime
from pathlib import Path

import serial
from serial.tools import list_ports

# io_test prints GP6..GP12, then GP5 as 'H'. Names from lib/steering_buttons.
BUTTON_NAMES = ["rot", "plus", "hupe", "schwarz", "blau", "gelb", "minus", "gruen"]
ANALOG_NAMES = ["Poti L", "Poti R", "Hall L", "Hall R"]
LINE_RE = re.compile(r"BTN ([#-]{7}) ([H-]) \| POT +(\d+) +(\d+) \| HALL +(\d+) +(\d+)")

# Pass marks for this run (as built 2026-10-22: pots 20-4088, Hall +2400..2600)
POT_MIN, POT_MAX = 100, 4000
HALL_RISE = 2000


class Check:
    def __init__(self, log):
        self.log = log
        self.t0 = time.monotonic()
        self.pressed = [False] * 8
        self.press_count = [0] * 8
        self.lo = [None] * 4
        self.hi = [None] * 4
        self.rest = [None] * 4
        self.lines = 0
        self.garbled = 0
        self.history = []
        self.drops = []

    def t(self):
        return time.monotonic() - self.t0

    def say(self, msg):
        text = f"[{self.t():7.1f}s] {msg}"
        print(text, flush=True)
        self.log.write(f"# {text}\n")
        self.log.flush()

    def feed(self, line):
        self.log.write(f"{datetime.now():%H:%M:%S.%f}"[:-3] + f" {line}\n")
        self.history = (self.history + [line])[-10:]
        m = LINE_RE.search(line)
        if not m:
            if line.strip():
                self.garbled += 1
            return
        self.lines += 1
        bits = [c == "#" for c in m.group(1)] + [m.group(2) == "H"]
        for i, down in enumerate(bits):
            if down != self.pressed[i]:
                self.pressed[i] = down
                if down:
                    self.press_count[i] += 1
                self.say(f"{'DRUECK' if down else 'los   '} {BUTTON_NAMES[i]}")
        for i, v in enumerate(int(g) for g in m.groups()[2:]):
            if self.rest[i] is None:
                self.rest[i] = v
            self.lo[i] = v if self.lo[i] is None else min(self.lo[i], v)
            self.hi[i] = v if self.hi[i] is None else max(self.hi[i], v)

    def dropped(self, reason):
        self.drops.append(self.t())
        self.say(f"USB WEG ({reason}). Letzte Zeilen:")
        for h in self.history:
            print(f"           {h}", flush=True)

    def analog_ok(self, i):
        if self.lo[i] is None:
            return False
        if i < 2:
            return self.lo[i] <= POT_MIN and self.hi[i] >= POT_MAX
        return self.hi[i] - self.rest[i] >= HALL_RISE

    def summary(self):
        print("\n=== ERGEBNIS ===")
        print(f"Zeilen {self.lines}, unlesbar {self.garbled}, USB-Abbrueche {len(self.drops)}"
              + (f" bei {', '.join(f'{d:.1f}s' for d in self.drops)}" if self.drops else ""))
        for i, name in enumerate(BUTTON_NAMES):
            mark = "OK " if self.press_count[i] else "-- "
            print(f"  {mark} {name:8s} {self.press_count[i]}x gedrueckt")
        for i, name in enumerate(ANALOG_NAMES):
            mark = "OK " if self.analog_ok(i) else "-- "
            rng = f"{self.lo[i]}-{self.hi[i]} (Ruhe {self.rest[i]})" if self.lo[i] is not None else "keine Werte"
            print(f"  {mark} {name:8s} {rng}")
        self.log.write("# summary written\n")


def port_present(port):
    return any(p.device.upper() == port.upper() for p in list_ports.comports())


def run_serial(check, port, seconds):
    waiting = True
    while check.t() < seconds:
        if not port_present(port):
            if not waiting:
                check.dropped("Port verschwunden")
                waiting = True
            time.sleep(0.2)
            continue
        try:
            with serial.Serial(port, 115200, timeout=1) as ser:
                check.say(f"{port} offen")
                waiting = False
                silent_since = time.monotonic()
                while check.t() < seconds:
                    raw = ser.readline()
                    if raw:
                        silent_since = time.monotonic()
                        check.feed(raw.decode(errors="replace").rstrip())
                    elif time.monotonic() - silent_since > 2:
                        check.say("2 s keine Daten (Port noch da)")
                        silent_since = time.monotonic()
        except serial.SerialException as e:
            if not waiting:
                check.dropped(f"SerialException: {e}")
                waiting = True
            time.sleep(0.5)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM7")
    ap.add_argument("--replay", type=Path, help="feed a saved log instead of the port")
    ap.add_argument("--seconds", type=float, default=float("inf"), help="stop after this run time")
    ap.add_argument("--log", type=Path,
                    default=Path(__file__).parent / "logs" / f"io_check_{datetime.now():%Y%m%d_%H%M%S}.log")
    args = ap.parse_args()
    args.log.parent.mkdir(exist_ok=True)
    with open(args.log, "w", encoding="utf-8") as log:
        check = Check(log)
        print(f"Log: {args.log}", flush=True)
        try:
            if args.replay:
                for line in args.replay.read_text(encoding="utf-8").splitlines():
                    check.feed(line.split(" ", 1)[1] if line[:2].isdigit() else line)
            else:
                print(f"Warte auf {args.port} ...", flush=True)
                run_serial(check, args.port, args.seconds)
        except KeyboardInterrupt:
            pass
        check.summary()


if __name__ == "__main__":
    sys.exit(main())
