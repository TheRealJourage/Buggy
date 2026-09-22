"""Generates the STM32Wiper wiring page (steckplan.html) and a standalone SVG for Obsidian.

Drawing helpers are the same as in SteeringControl/docs/steckplan/gen.py.
"""
import os
import re

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "steckplan.html")

HOP = 6


class Fig:
    def __init__(self):
        self.wires = []      # (net, cls, [points])
        self.overlays = []   # svg strings drawn after wires
        self.under = []      # svg strings drawn before wires
        self.dots = []
        self.badges = []

    def wire(self, net, cls, pts):
        self.wires.append((net, cls, pts))

    def dot(self, x, y, cls):
        self.dots.append(f'<circle cx="{x}" cy="{y}" r="4" class="dot {cls}"/>')

    def badge(self, x, y, n, cls):
        self.badges.append(
            f'<g class="badge {cls}"><circle cx="{x}" cy="{y}" r="10"/>'
            f'<text x="{x}" y="{y + 4}" text-anchor="middle">{n}</text></g>')

    def render_wires(self):
        horiz = []
        for net, cls, pts in self.wires:
            for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                if y1 == y2:
                    horiz.append((net, min(x1, x2), max(x1, x2), y1))
        out = []
        for net, cls, pts in self.wires:
            d = f"M{pts[0][0]},{pts[0][1]}"
            for (x1, y1), (x2, y2) in zip(pts, pts[1:]):
                if x1 == x2 and y1 != y2:
                    lo, hi = min(y1, y2), max(y1, y2)
                    cross = sorted({hy for hn, hx1, hx2, hy in horiz
                                    if hn != net and hx1 + 1 < x1 < hx2 - 1 and lo + HOP < hy < hi - HOP},
                                   reverse=y2 < y1)
                    step = 1 if y2 > y1 else -1
                    for cy in cross:
                        d += f" L{x1},{cy - step * HOP} A{HOP},{HOP} 0 0 {1 if step > 0 else 0} {x1},{cy + step * HOP}"
                d += f" L{x2},{y2}"
            out.append(f'<path d="{d}" class="wc"/><path d="{d}" class="w {cls}"/>')
        return "\n".join(out)

    def svg(self, vb, label):
        return (f'<svg viewBox="{vb}" role="img" aria-label="{label}" xmlns="http://www.w3.org/2000/svg">'
                + "\n".join(self.under) + self.render_wires() + "\n".join(self.overlays)
                + "\n".join(self.dots) + "\n".join(self.badges) + "</svg>")


BAND = {"schwarz": "#111111", "braun": "#7a4a1f", "rot": "#d42a1e"}
# 5-band metal film (blue body)
CODE5 = {"220 Ω": ("rot", "rot", "schwarz", "schwarz", "braun")}


def resistor(f, x, y, vertical, value, label_side=1):
    w, h = (12, 38) if vertical else (38, 12)
    f.overlays.append(f'<rect x="{x - w / 2}" y="{y - h / 2}" width="{w}" height="{h}" rx="3" class="part rbody"/>')
    # first band at the left/top end, tolerance band set apart
    for off, name in zip((-13, -8, -3, 2, 11), CODE5[value]):
        if vertical:
            f.overlays.append(f'<rect x="{x - w / 2 + 0.9}" y="{y + off - 1.4}" width="{w - 1.8}" height="2.8" fill="{BAND[name]}"/>')
        else:
            f.overlays.append(f'<rect x="{x + off - 1.4}" y="{y - h / 2 + 0.9}" width="2.8" height="{h - 1.8}" fill="{BAND[name]}"/>')
    if vertical:
        f.overlays.append(f'<text x="{x + label_side * 12}" y="{y + 4}" class="val" text-anchor="{"start" if label_side > 0 else "end"}">{value}</text>')
    else:
        f.overlays.append(f'<text x="{x}" y="{y - 12 * label_side + (4 if label_side < 0 else 0)}" class="val" text-anchor="middle">{value}</text>')


def button(f, x, y, label):
    """Pushbutton on a horizontal wire, centered at x."""
    f.overlays.append(f'<rect x="{x - 16}" y="{y - 14}" width="32" height="20" class="cut"/>')
    f.overlays.append(f'<circle cx="{x - 14}" cy="{y}" r="2.5" class="term"/><circle cx="{x + 14}" cy="{y}" r="2.5" class="term"/>'
                      f'<line x1="{x - 14}" y1="{y - 8}" x2="{x + 14}" y2="{y - 8}" class="plate"/>'
                      f'<line x1="{x}" y1="{y - 8}" x2="{x}" y2="{y - 15}" class="plate"/>'
                      f'<line x1="{x - 5}" y1="{y - 15}" x2="{x + 5}" y2="{y - 15}" class="plate"/>')
    f.overlays.append(f'<text x="{x}" y="{y + 20}" class="val" text-anchor="middle">{label}</text>')


def pad(f, x, y, label, anchor, dx=0, dy=0, used=False):
    u = " used" if used else ""
    f.overlays.append(f'<rect x="{x - 4.5}" y="{y - 4.5}" width="9" height="9" class="pad{u}"/>')
    f.overlays.append(f'<text x="{x + dx}" y="{y + dy}" class="lbl{u}" text-anchor="{anchor}">{label}</text>')


# ═══ FIGURE: stalk switches -> Nucleo ══════════════════════════════════════════
f = Fig()

# ── Nucleo-F446RE, portrait, ST-LINK USB on top ──
NX, NY, NW, NH = 520, 90, 240, 590
LX, RX = NX + 20, NX + NW - 20          # inner Arduino headers
f.under.append(f'<rect x="{NX}" y="{NY}" width="{NW}" height="{NH}" rx="10" class="board nucleo"/>'
               f'<rect x="{NX + 90}" y="{NY - 12}" width="60" height="26" rx="3" class="port"/>'
               f'<text x="{NX + 120}" y="{NY + 6}" class="porttxt" text-anchor="middle">USB</text>'
               f'<rect x="{NX + 14}" y="{NY + 24}" width="{NW - 28}" height="100" rx="4" class="stlink"/>'
               f'<text x="{NX + NW / 2}" y="{NY + 80}" class="hdr" text-anchor="middle">ST-LINK</text>'
               f'<rect x="{NX + 90}" y="{NY + 330}" width="60" height="60" rx="3" class="mcu"/>'
               f'<text x="{NX + 120}" y="{NY + 364}" class="porttxt" text-anchor="middle">F446RE</text>'
               f'<text x="{NX + NW / 2}" y="{NY + 150}" class="boardname big" text-anchor="middle">Nucleo-F446RE</text>'
               f'<text x="{NX + NW / 2}" y="{NY + 168}" class="chipsub" text-anchor="middle">USB vom PC</text>'
               f'<text x="{LX + 12}" y="{NY + 188}" class="hdr">CN6</text>'
               f'<text x="{RX - 12}" y="{NY + 188}" class="hdr" text-anchor="end">CN5</text>')

left = {}
y = 300
for n in ["NC", "IOREF", "RESET", "+3V3", "+5V", "GND", "GND₂", "VIN"]:
    left[n] = y; y += 20
y += 20
for n in ["A0", "A1", "A2", "A3", "A4", "A5"]:
    left[n] = y; y += 20
right = {}
y = 300
for n in ["D15", "D14", "AREF", "GND", "D13", "D12", "D11", "D10", "D9", "D8"]:
    right[n] = y; y += 20
y += 20
for n in ["D7", "D6", "D5", "D4", "D3", "D2", "D1", "D0"]:
    right[n] = y; y += 20

f.under.append(f'<text x="{LX + 12}" y="{left["A0"] - 16}" class="hdr">CN8</text>'
               f'<text x="{RX - 12}" y="{right["D7"] - 16}" class="hdr" text-anchor="end">CN9</text>')

used_l = {"+3V3", "GND", "A0", "A1", "A2", "A3"}
for n, y in left.items():
    pad(f, LX, y, n.replace("₂", ""), "start", dx=10, dy=4, used=n in used_l)
for n, y in right.items():
    pad(f, RX, y, n, "end", dx=-10, dy=4, used=n == "D7")

# ── two stalks, 3 wires each: common + two resistor-coded signal lines ──
SX, SW = 40, 150
TX = SX + SW
stalks = [("Lichthebel", "Blinker · Fernlicht · Lichthupe", 370, {"GND": 400, "A0": 450, "A2": 500}),
          ("Wischerhebel", "Front · Heck · Waschen", 560, {"GND": 590, "A1": 640, "A3": 690})]
wire_col = {"GND": "weiß (Masse?)", "A0": "weiß-schwarz", "A2": "gelb-schwarz", "A1": "weiß-schwarz", "A3": "gelb-schwarz"}
term = {}
for name, sub, sy, t in stalks:
    f.under.append(f'<rect x="{SX}" y="{sy}" width="{SW}" height="150" rx="8" class="board stalk"/>'
                   f'<text x="{SX}" y="{sy - 24}" class="boardname">{name}</text>'
                   f'<text x="{SX}" y="{sy - 8}" class="chipsub">{sub}</text>')
    for k, y in t.items():
        f.overlays.append(f'<circle cx="{TX}" cy="{y}" r="4" class="term"/>'
                          f'<text x="{TX - 10}" y="{y + 4}" class="val" text-anchor="end">{wire_col[k]}</text>')
    term[name] = t

# ── 1: +3V3 -> 3.3 V rail (vertical, x = RAIL) ──
RAIL = 300
JX = 380                      # pull-up junction x on each signal line
f.wire("3v3", "c33", [(LX, left["+3V3"]), (RAIL, left["+3V3"]), (RAIL, 665)])
f.badge(440, left["+3V3"], 1, "c33")
f.under.append(f'<text x="{RAIL + 8}" y="{left["+3V3"] - 8}" class="rail">3,3-V-Schiene</text>')

# ── 2/3: GND -> common wire of each stalk ──
f.wire("gnd", "cg", [(LX, left["GND"]), (TX, 400)])
f.badge(440, 400, 2, "cg")
f.wire("gnd", "cg", [(230, 400), (230, 590), (TX, 590)]); f.dot(230, 400, "cg")
f.badge(230, 560, 3, "cg")

# ── 4-7: signals; 8-11: 220 R pull-ups ──
sig = [("A0", "crx", 4, 8, [(470, 480), (470, 450)], 450),
       ("A2", "clin", 5, 9, [(450, 520), (450, 500)], 500),
       ("A1", "ctx", 6, 10, [(505, 500), (505, 640)], 640),
       ("A3", "ccs", 7, 11, [(515, 540), (515, 690)], 690)]
for a, cls, n, rn, fan, ty in sig:
    f.wire(a, cls, [(LX, left[a])] + fan + [(TX, ty)])
    f.badge(420, ty, n, cls)
    f.wire(a, cls, [(JX, ty), (JX, ty - 25)])
    f.wire("3v3", "c33", [(JX, ty - 25), (RAIL, ty - 25)])
    f.dot(JX, ty, cls); f.dot(RAIL, ty - 25, "c33")
    resistor(f, (JX + RAIL) / 2, ty - 25, False, "220 Ω", 1)
    f.badge(JX + 18, ty - 25, rn, "c33")

# ── 12/13: calibration button 3V3 -> button -> D7 ──
f.wire("3v3", "c33", [(470, left["+3V3"]), (470, 40), (900, 40), (900, right["D7"])])
f.dot(470, left["+3V3"], "c33")
f.badge(900, 200, 12, "c33")
f.wire("d7", "csig", [(RX, right["D7"]), (900, right["D7"])])
button(f, 850, right["D7"], "Kalibrier-Taster")
f.badge(790, right["D7"], 13, "csig")

FIG = f.svg("0 0 960 740", "Verkabelung Lenkstockschalter an Nucleo-F446RE: zwei Hebel mit je drei Adern, vier ADC-Eingänge mit 220-Ohm-Pull-ups, Kalibrier-Taster")

tpl = open(os.path.join(HERE, "template.html"), encoding="utf-8").read()
html = tpl.replace("{{FIG1}}", FIG)
html = ('<!doctype html><html lang="de"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">\n'
        + html.replace("</style>\n", "</style>\n</head><body>\n", 1) + "</body></html>\n")
open(OUT, "w", encoding="utf-8").write(html)
print("written", OUT, len(html))

# ─── standalone SVG for Obsidian (light theme baked in) ────────────────────────
css = tpl[tpl.index("<style>") + 7:tpl.index("</style>")]
root = css[css.index(":root {") + 7:css.index("}", css.index(":root {"))]
rules = [l for l in css.splitlines() if l.startswith("svg ") or l.startswith(".c")]
style = "svg{" + root.replace("\n", "") + "}" + "\n".join(rules)
style = style.replace('"IBM Plex Mono", ', "").replace('"IBM Plex Sans", ', "")
vb = re.search(r'viewBox="0 0 (\d+) (\d+)"', FIG)
w, h = vb.group(1), vb.group(2)
body = FIG.replace("<svg ", f'<svg width="{w}" height="{h}" ', 1)
head_end = body.index(">") + 1
body = body[:head_end] + f'<style>{style}</style><rect width="{w}" height="{h}" fill="#fbfcfc"/>' + body[head_end:]
open(os.path.join(HERE, "steckplan-lenkstock.svg"), "w", encoding="utf-8").write('<?xml version="1.0" encoding="UTF-8"?>\n' + body)
print("svg written")
