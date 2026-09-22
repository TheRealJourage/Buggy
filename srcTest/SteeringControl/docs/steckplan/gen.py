"""Generates the SteeringControl wiring page (wiring.html)."""
import os

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "steckplan.html")

HOP = 6


class Fig:
    def __init__(self):
        self.wires = []      # (net, cls, [points], hops_allowed)
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

    def svg(self, vb, label, extra_top=""):
        return (f'<svg viewBox="{vb}" role="img" aria-label="{label}" xmlns="http://www.w3.org/2000/svg">'
                + extra_top + "\n".join(self.under) + self.render_wires() + "\n".join(self.overlays)
                + "\n".join(self.dots) + "\n".join(self.badges) + "</svg>")


# ─── symbols (drawn over wires, bg-filled) ─────────────────────────────────────
BAND = {"schwarz": "#111111", "braun": "#7a4a1f", "rot": "#d42a1e", "gelb": "#f2c200",
        "violett": "#7d3cc4", "gold": "#c9a227"}
CODE4 = {"220 Ω": ("rot", "rot", "braun", "gold"),
         "1 kΩ": ("braun", "schwarz", "rot", "gold"),
         "4,7 kΩ": ("gelb", "violett", "rot", "gold")}


def resistor(f, x, y, vertical, value, label_side=1):
    w, h = (12, 38) if vertical else (38, 12)
    f.overlays.append(f'<rect x="{x - w / 2}" y="{y - h / 2}" width="{w}" height="{h}" rx="3" class="part rbody"/>')
    # 4-band code, first band at the left/top end, tolerance band set apart
    for off, name in zip((-13, -7, -1, 10), CODE4[value]):
        if vertical:
            f.overlays.append(f'<rect x="{x - w / 2 + 0.9}" y="{y + off - 1.6}" width="{w - 1.8}" height="3.2" fill="{BAND[name]}"/>')
        else:
            f.overlays.append(f'<rect x="{x + off - 1.6}" y="{y - h / 2 + 0.9}" width="3.2" height="{h - 1.8}" fill="{BAND[name]}"/>')
    if vertical:
        f.overlays.append(f'<text x="{x + label_side * 12}" y="{y + 4}" class="val" text-anchor="{"start" if label_side > 0 else "end"}">{value}</text>')
    else:
        f.overlays.append(f'<text x="{x}" y="{y - 12 * label_side + (4 if label_side < 0 else 0)}" class="val" text-anchor="middle">{value}</text>')


def cap(f, x, y, vertical, value, label_side=1):
    """vertical=True: current flows vertically -> plates horizontal."""
    if vertical:
        f.overlays.append(f'<rect x="{x - 9}" y="{y - 3}" width="18" height="6" class="cut"/>')
        f.overlays.append(f'<line x1="{x - 9}" y1="{y - 3}" x2="{x + 9}" y2="{y - 3}" class="plate"/>'
                          f'<line x1="{x - 9}" y1="{y + 3}" x2="{x + 9}" y2="{y + 3}" class="plate"/>')
        f.overlays.append(f'<text x="{x + label_side * 13}" y="{y + 4}" class="val" text-anchor="{"start" if label_side > 0 else "end"}">{value}</text>')
    else:
        f.overlays.append(f'<rect x="{x - 3}" y="{y - 9}" width="6" height="18" class="cut"/>')
        f.overlays.append(f'<line x1="{x - 3}" y1="{y - 9}" x2="{x - 3}" y2="{y + 9}" class="plate"/>'
                          f'<line x1="{x + 3}" y1="{y - 9}" x2="{x + 3}" y2="{y + 9}" class="plate"/>')
        f.overlays.append(f'<text x="{x}" y="{y + 22}" class="val" text-anchor="middle">{value}</text>')


def diode_down(f, x, y, label, led=False, cls=""):
    """Anode at top, cathode (bar) at bottom."""
    f.overlays.append(f'<rect x="{x - 9}" y="{y - 9}" width="18" height="18" class="cut"/>')
    f.overlays.append(f'<polygon points="{x - 8},{y - 7} {x + 8},{y - 7} {x},{y + 6}" class="dio {cls}"/>'
                      f'<line x1="{x - 8}" y1="{y + 7}" x2="{x + 8}" y2="{y + 7}" class="plate"/>')
    f.overlays.append(f'<text x="{x + 14}" y="{y + 4}" class="val">{label}</text>')


def diode_left(f, x, y, label, led_cls):
    """LED, current flows right -> left (anode right, cathode left)."""
    f.overlays.append(f'<rect x="{x - 9}" y="{y - 9}" width="18" height="18" class="cut"/>')
    f.overlays.append(f'<polygon points="{x + 7},{y - 8} {x + 7},{y + 8} {x - 6},{y}" class="dio {led_cls}"/>'
                      f'<line x1="{x - 7}" y1="{y - 8}" x2="{x - 7}" y2="{y + 8}" class="plate"/>'
                      f'<path d="M{x - 2},{y - 11} l5,-7 M{x + 4},{y - 11} l5,-7" class="rays"/>')
    f.overlays.append(f'<text x="{x}" y="{y + 22}" class="val" text-anchor="middle">{label}</text>')


def button(f, x, y, label):
    """Pushbutton on a horizontal wire, centered at x."""
    f.overlays.append(f'<rect x="{x - 16}" y="{y - 14}" width="32" height="20" class="cut"/>')
    f.overlays.append(f'<circle cx="{x - 14}" cy="{y}" r="2.5" class="term"/><circle cx="{x + 14}" cy="{y}" r="2.5" class="term"/>'
                      f'<line x1="{x - 14}" y1="{y - 8}" x2="{x + 14}" y2="{y - 8}" class="plate"/>'
                      f'<line x1="{x}" y1="{y - 8}" x2="{x}" y2="{y - 15}" class="plate"/>'
                      f'<line x1="{x - 5}" y1="{y - 15}" x2="{x + 5}" y2="{y - 15}" class="plate"/>')
    f.overlays.append(f'<text x="{x}" y="{y + 20}" class="val" text-anchor="middle">{label}</text>')


def pot(f, x, y1, y2, label):
    """Vertical potentiometer body between y1..y2, wiper arrow from the right."""
    ym = (y1 + y2) / 2
    f.overlays.append(f'<rect x="{x - 7}" y="{y1}" width="14" height="{y2 - y1}" class="part"/>'
                      f'<polygon points="{x + 7},{ym} {x + 15},{ym - 5} {x + 15},{ym + 5}" class="arrowhead"/>')
    f.overlays.append(f'<text x="{x - 10}" y="{y1 - 6}" class="val" text-anchor="end">{label}</text>')


def pad(f, x, y, label, anchor, dx=0, dy=0, used=False, cls="lbl"):
    f.overlays.append(f'<rect x="{x - 4.5}" y="{y - 4.5}" width="9" height="9" class="pad{" used" if used else ""}"/>')
    u = " used" if used else ""
    if "vertup" in cls:      # bottom header: label reads upward, starts above the pad
        tx, ty = x + 3.5, y - 9
        f.overlays.append(f'<text x="{tx}" y="{ty}" class="lbl{u}" text-anchor="start" transform="rotate(-90 {tx} {ty})">{label}</text>')
    elif "vert" in cls:      # top header: label reads upward, ends below the pad
        tx, ty = x + 3.5, y + 9
        f.overlays.append(f'<text x="{tx}" y="{ty}" class="lbl{u}" text-anchor="end" transform="rotate(-90 {tx} {ty})">{label}</text>')
    else:
        f.overlays.append(f'<text x="{x + dx}" y="{y + dy}" class="{cls}{u}" text-anchor="{anchor}">{label}</text>')


def nc(f, x, y, side, text=True):
    """'not connected' cross on a lead end."""
    f.overlays.append(f'<path d="M{x - 5},{y - 5} l10,10 M{x + 5},{y - 5} l-10,10" class="ncx"/>')
    if text:
        f.overlays.append(f'<text x="{x + 10 * side}" y="{y + 4}" class="nc" text-anchor="{"start" if side > 0 else "end"}">frei</text>')


def dip(f, x, y, name, sub, left, right, pitch):
    """DIP-8. x = left lead end, body from x+20 .. x+140. left/right: labels top->bottom."""
    bx, bw = x + 20, 120
    top = y - 18
    h = pitch * 3 + 36
    f.under.append(f'<rect x="{bx}" y="{top}" width="{bw}" height="{h}" rx="3" class="chip"/>'
                   f'<path d="M{bx + bw / 2 - 9},{top} a9,9 0 0 0 18,0" class="notch"/>'
                   f'<circle cx="{bx + 10}" cy="{top + 10}" r="2.5" class="pin1"/>')
    f.under.append(f'<text x="{bx + bw / 2}" y="{top - 22}" class="chipname" text-anchor="middle">{name}</text>'
                   f'<text x="{bx + bw / 2}" y="{top - 8}" class="chipsub" text-anchor="middle">{sub}</text>')
    for i, lab in enumerate(left):
        py = y + i * pitch
        f.under.append(f'<line x1="{x}" y1="{py}" x2="{bx}" y2="{py}" class="lead"/>'
                       f'<text x="{bx + 6}" y="{py + 4}" class="pinname">{i + 1} {lab}</text>')
    for i, lab in enumerate(right):
        py = y + i * pitch
        f.under.append(f'<line x1="{bx + bw}" y1="{py}" x2="{bx + bw + 20}" y2="{py}" class="lead"/>'
                       f'<text x="{bx + bw - 6}" y="{py + 4}" class="pinname" text-anchor="end">{lab} {8 - i}</text>')


# ═══ FIGURE 1: LIN link ════════════════════════════════════════════════════════
f1 = Fig()
R12, RG = 120, 748           # rails
P = 36                       # DIP pitch
CY = 300                     # DIP pin 1 y
C2 = 640                     # MCP #2 left lead x   (right lead = 760)
C1 = 1200                    # MCP #1 left lead x   (right lead = 1320)
pins_y = [CY + i * P for i in range(4)]     # pin1..4 left / pin8..5 right

# rails
f1.wire("12v", "c12", [(150, R12), (1480, R12)])
f1.wire("gnd", "cg", [(22, RG), (1480, RG)])
f1.under.append(f'<text x="1480" y="{R12 - 8}" class="rail" text-anchor="end">+12 V · Plus-Schiene</text>'
                f'<text x="1480" y="{RG + 20}" class="rail" text-anchor="end">GND · Masse-Schiene (gemeinsam für alles)</text>')

# lab supply
f1.under.append('<rect x="30" y="18" width="200" height="62" rx="6" class="board psu"/>'
                '<text x="130" y="44" class="boardname" text-anchor="middle">Labornetzteil</text>'
                '<text x="130" y="62" class="chipsub" text-anchor="middle">12,0 V · Limit ~100 mA</text>')
f1.overlays.append('<circle cx="60" cy="80" r="6" class="term minus"/><circle cx="200" cy="80" r="6" class="term plus"/>'
                   '<text x="60" y="84" class="termtxt" text-anchor="middle">−</text><text x="200" y="84" class="termtxt" text-anchor="middle">+</text>')
f1.wire("12v", "c12", [(200, 86), (200, R12)]); f1.dot(200, R12, "c12"); f1.badge(214, 102, 1, "c12")
f1.wire("gnd", "cg", [(60, 86), (60, 100), (22, 100), (22, RG)]); f1.badge(22, 420, 2, "cg")

# ── Arduino Uno R3 ──
UX, UY, UW, UH = 60, 220, 470, 300
f1.under.append(f'<rect x="{UX}" y="{UY}" width="{UW}" height="{UH}" rx="10" class="board uno"/>'
                f'<rect x="{UX - 22}" y="{UY + 30}" width="56" height="50" rx="4" class="port"/>'
                f'<text x="{UX + 6}" y="{UY + 60}" class="porttxt" text-anchor="middle">USB</text>'
                f'<rect x="{UX - 16}" y="{UY + 210}" width="48" height="46" rx="4" class="port"/>'
                f'<text x="{UX + 8}" y="{UY + 238}" class="porttxt" text-anchor="middle">DC</text>'
                f'<text x="{UX + UW / 2 + 20}" y="{UY + 150}" class="boardname big" text-anchor="middle">Arduino Uno R3</text>'
                f'<text x="{UX + UW / 2 + 20}" y="{UY + 170}" class="chipsub" text-anchor="middle">Master · USB vom PC</text>'
                f'<text x="{UX + UW - 12}" y="{UY + 92}" class="hdr" text-anchor="end">DIGITAL (PWM ~)</text>'
                f'<text x="{UX + 100}" y="{UY + UH - 50}" class="hdr">POWER</text>'
                f'<text x="{UX + 316}" y="{UY + UH - 50}" class="hdr">ANALOG IN</text>')
TH = UY + 16
top_hdr = {}
xs = 510
for name in ["0", "1", "2", "3", "4", "5", "6", "7"]:
    top_hdr[name] = xs; xs -= 18
xs -= 12
for name in ["8", "9", "10", "11", "12", "13", "GND", "AREF", "SDA", "SCL"]:
    top_hdr[name] = xs; xs -= 18
used_top = {"0", "1", "2", "12", "13", "GND"}
for name, x in top_hdr.items():
    lab = {"0": "RX 0", "1": "TX 1"}.get(name, name)
    pad(f1, x, TH, lab if name in ("0", "1") else name, "end", dx=3, dy=16, used=name in used_top, cls="lbl vert")
BH = UY + UH - 16
bot_hdr = {}
xs = 200
for name in ["NC", "IOREF", "RESET", "3.3V", "5V", "GND", "GND₂", "VIN"]:
    bot_hdr[name] = xs; xs += 18
xs += 18
for name in ["A0", "A1", "A2", "A3", "A4", "A5"]:
    bot_hdr[name] = xs; xs += 18
for name, x in bot_hdr.items():
    pad(f1, x, BH, name.replace("₂", ""), "start", dx=-3, dy=-12, used=name in ("5V", "GND"), cls="lbl vertup")

# ── MCP2003 #2 (master) ──
dip(f1, C2, CY, "MCP2003 #2", "Master-Seite", ["RXD", "CS", "WAKE", "TXD"], ["VREN", "VBB", "LBUS", "VSS"], P)
# ── MCP2003 #1 (slave) ──
dip(f1, C1, CY, "MCP2003 #1", "Slave-Seite", ["RXD", "CS", "WAKE", "TXD"], ["VREN", "VBB", "LBUS", "VSS"], P)

# ── RP2040-Tiny ──
TX0, TY0, TW, THt = 980, 250, 160, 250
f1.under.append(f'<rect x="{TX0}" y="{TY0}" width="{TW}" height="{THt}" rx="10" class="board tiny"/>'
                f'<rect x="{TX0 + 48}" y="{TY0 - 6}" width="64" height="18" rx="2" class="port"/>'
                f'<text x="{TX0 + 80}" y="{TY0 + 7}" class="porttxt" text-anchor="middle">FPC</text>'
                f'<rect x="{TX0 + 52}" y="{TY0 + 80}" width="56" height="56" rx="3" class="mcu"/>'
                f'<text x="{TX0 + 80}" y="{TY0 + 112}" class="porttxt" text-anchor="middle">RP2040</text>'
                f'<text x="{TX0 + 80}" y="{TY0 - 28}" class="boardname" text-anchor="middle">RP2040-Tiny</text>'
                f'<text x="{TX0 + 80}" y="{TY0 - 13}" class="chipsub" text-anchor="middle">Slave · USB über Adapter</text>')
tiny_r = {f"GP{i}": TY0 + 30 + i * 20 for i in range(10)}
tiny_l = {n: TY0 + 30 + i * 20 for i, n in enumerate(["5V", "GND", "3V3", "GP29", "GP28", "GP27", "GP26", "GP15", "GP14"])}
tiny_b = {n: TX0 + 40 + i * 25 for i, n in enumerate(["GP13", "GP12", "GP11", "GP10"])}
for n, y in tiny_r.items():
    pad(f1, TX0 + TW, y, n, "end", dx=-9, dy=4, used=n in ("GP0", "GP1", "GP2"))
for n, y in tiny_l.items():
    pad(f1, TX0, y, n, "start", dx=9, dy=4, used=n in ("GND", "3V3"))
for n, x in tiny_b.items():
    pad(f1, x, TY0 + THt, n.replace("GP", ""), "middle", dy=-10)

# ── wires Uno ↔ MCP2003 #2 ──
p1, p2, p3, p4 = pins_y
# 5  D0 (RX) <- RXD
f1.wire("d0", "crx", [(top_hdr["0"], TH), (top_hdr["0"], 186), (620, 186), (620, p1), (C2, p1)]); f1.badge(566, 186, 5, "crx")
# 6  D2 -> CS
f1.wire("d2", "ccs", [(top_hdr["2"], TH), (top_hdr["2"], 196), (605, 196), (605, p2), (C2, p2)]); f1.badge(540, 196, 6, "ccs")
# 4  D1 (TX) -> TXD
f1.wire("d1", "ctx", [(top_hdr["1"], TH), (top_hdr["1"], 206), (590, 206), (590, p4), (C2, p4)]); f1.badge(560, 206, 4, "ctx")
# 7  pull-up 4.7k: RXD #2 -> Uno 5V
f1.wire("d0", "c5", [(628, p1), (628, 582), (bot_hdr["5V"], 582), (bot_hdr["5V"], BH)]); f1.dot(628, p1, "crx")
resistor(f1, 470, 582, False, "4,7 kΩ", 1); f1.badge(400, 582, 7, "c5")
# 3  Uno GND -> rail
f1.wire("gnd", "cg", [(bot_hdr["GND"], BH), (bot_hdr["GND"], RG)]); f1.dot(bot_hdr["GND"], RG, "cg"); f1.badge(bot_hdr["GND"], 680, 3, "cg")
nc(f1, C2, p3, -1, False)
# status LEDs D12 / D13 -> 220R -> LED -> GND pin on the top header
for pin, yy, txt, cls, num in (("12", 150, "LED grün", "led-g", 23), ("13", 178, "LED rot", "led-r", 24)):
    f1.wire(f"led{pin}", "csig", [(top_hdr[pin], TH), (top_hdr[pin], yy), (112, yy)])
    resistor(f1, 200, yy, False, "220 Ω", 1 if pin == "12" else -1)
    diode_left(f1, 140, yy, "", cls)
    f1.overlays.append(f'<text x="88" y="{yy + 4}" class="val" text-anchor="end">{txt}</text>')
    f1.badge(top_hdr[pin] + 13, yy - 10 if pin == "12" else yy + 14, num, "csig")
f1.wire("gnd", "cg", [(112, 150), (96, 150), (96, 206), (top_hdr["GND"], 206), (top_hdr["GND"], TH)])
f1.wire("gnd", "cg", [(112, 178), (96, 178)]); f1.dot(96, 178, "cg")

# ── MCP2003 #2 right side ──
R2 = C2 + 160
nc(f1, R2, p1, 1)
f1.wire("12v", "c12", [(R2, p2), (850, p2), (850, R12)]); f1.dot(850, R12, "c12"); f1.badge(850, 250, 8, "c12")
f1.wire("gnd", "cg", [(R2, p4), (830, p4), (830, RG)]); f1.dot(830, RG, "cg"); f1.badge(830, 680, 9, "cg")
f1.wire("12v", "c12", [(830, p2), (830, (p2 + p3) / 2)]); f1.wire("gnd", "cg", [(830, (p2 + p3) / 2), (830, p4)]); f1.dot(830, p2, "c12"); f1.dot(830, p4, "cg")
cap(f1, 830, (p2 + p3) / 2, True, "", -1); f1.badge(812, (p2 + p3) / 2, 10, "cg")
f1.wire("lin", "clin", [(R2, p3), (870, p3), (870, 692), (1430, 692), (1430, p3), (C1 + 160, p3)])
f1.badge(1180, 692, 13, "clin")
f1.overlays.append('<text x="1180" y="714" class="val strong" text-anchor="middle">LIN-Draht</text>')
# 12 220pF LBUS -> GND at #2
f1.wire("c220a", "clin", [(870, 612), (830, 612)]); f1.dot(870, 612, "clin"); f1.dot(830, 612, "cg")
cap(f1, 850, 612, False, "220 pF"); f1.badge(850, 588, 12, "clin")
# 11 master pull-up 12V -> 1N4148 -> 1k -> LBUS
f1.wire("pu", "c12", [(920, R12), (920, 520), (870, 520)]); f1.dot(920, R12, "c12"); f1.dot(870, 520, "clin")
diode_down(f1, 920, 160, "1N4148, Ring unten")
f1.overlays.append('<text x="934" y="180" class="val strong">Master-Pull-up</text>')
resistor(f1, 920, 262, True, "1 kΩ", -1)
f1.badge(920, 420, 11, "c12")

# ── MCP2003 #1 right side ──
R1 = C1 + 160
nc(f1, R1, p1, 1)
f1.wire("12v", "c12", [(R1, p2), (1410, p2), (1410, R12)]); f1.dot(1410, R12, "c12"); f1.badge(1410, 250, 14, "c12")
f1.wire("gnd", "cg", [(R1, p4), (1390, p4), (1390, RG)]); f1.dot(1390, RG, "cg"); f1.badge(1390, 660, 15, "cg")
f1.wire("12v", "c12", [(1390, p2), (1390, (p2 + p3) / 2)]); f1.wire("gnd", "cg", [(1390, (p2 + p3) / 2), (1390, p4)]); f1.dot(1390, p2, "c12"); f1.dot(1390, p4, "cg")
cap(f1, 1390, (p2 + p3) / 2, True, "", -1); f1.badge(1372, (p2 + p3) / 2, 16, "cg")
f1.wire("c220b", "clin", [(1430, 612), (1390, 612)]); f1.dot(1430, 612, "clin"); f1.dot(1390, 612, "cg")
cap(f1, 1410, 612, False, "220 pF"); f1.badge(1410, 588, 17, "clin")
f1.dot(1430, 692, "clin")
nc(f1, C1, p3, -1, False)

# ── Tiny ↔ MCP2003 #1 ──
TR = TX0 + TW
f1.wire("gp1", "crx", [(TR, tiny_r["GP1"]), (C1, p1)]); f1.badge(1172, p1 - 12, 19, "crx")
f1.wire("gp2", "ccs", [(TR, tiny_r["GP2"]), (1150, tiny_r["GP2"]), (1150, p2), (C1, p2)]); f1.badge(1178, p2 + 12, 20, "ccs")
f1.wire("gp0", "ctx", [(TR, tiny_r["GP0"]), (1162, tiny_r["GP0"]), (1162, p4), (C1, p4)]); f1.badge(1182, p4 - 12, 18, "ctx")
f1.wire("gp1", "c33", [(TX0, tiny_l["3V3"]), (945, tiny_l["3V3"]), (945, 196), (1188, 196), (1188, p1)]); f1.dot(1188, p1, "crx")
resistor(f1, 1130, 196, False, "4,7 kΩ", 1); f1.badge(975, 196, 21, "c33")
f1.wire("gnd", "cg", [(TX0, tiny_l["GND"]), (958, tiny_l["GND"]), (958, RG)]); f1.dot(958, RG, "cg"); f1.badge(958, 600, 22, "cg")

FIG1 = f1.svg("0 0 1500 780", "Verkabelung der LIN-Strecke: Uno, zwei MCP2003, RP2040-Tiny, 12-V-Netzteil")

# ═══ FIGURE 2: RP2040-Tiny inputs ═══════════════════════════════════════════════
f2 = Fig()
X0, Y0, W, H = 400, 120, 160, 250
f2.under.append(f'<rect x="{X0}" y="{Y0}" width="{W}" height="{H}" rx="10" class="board tiny"/>'
                f'<rect x="{X0 + 48}" y="{Y0 - 6}" width="64" height="18" rx="2" class="port"/>'
                f'<text x="{X0 + 80}" y="{Y0 + 7}" class="porttxt" text-anchor="middle">FPC</text>'
                f'<rect x="{X0 + 52}" y="{Y0 + 80}" width="56" height="56" rx="3" class="mcu"/>'
                f'<text x="{X0 + 80}" y="{Y0 + 112}" class="porttxt" text-anchor="middle">RP2040</text>'
                f'<text x="{X0 + 80}" y="{Y0 - 14}" class="boardname" text-anchor="middle">RP2040-Tiny</text>')
r = {f"GP{i}": Y0 + 30 + i * 20 for i in range(10)}
l = {n: Y0 + 30 + i * 20 for i, n in enumerate(["5V", "GND", "3V3", "GP29", "GP28", "GP27", "GP26", "GP15", "GP14"])}
b = {n: X0 + 40 + i * 25 for i, n in enumerate(["GP13", "GP12", "GP11", "GP10"])}
for n, y in r.items():
    pad(f2, X0 + W, y, n, "end", dx=-9, dy=4, used=n in ("GP6", "GP7", "GP8", "GP9"))
for n, y in l.items():
    pad(f2, X0, y, n, "start", dx=9, dy=4, used=n in ("GND", "3V3", "GP27", "GP26", "GP14"))
for n, x in b.items():
    pad(f2, x, Y0 + H, n.replace("GP", ""), "middle", dy=-10, used=n in ("GP10", "GP11", "GP12"))

GX = 880
# GND: Tiny GND -> top bus -> buttons (right) and pots/LED (left)
f2.wire("gnd", "cg", [(X0, l["GND"]), (380, l["GND"]), (380, 80), (100, 80), (100, 470)])
f2.wire("gnd", "cg", [(380, 80), (GX, 80), (GX, 494)]); f2.dot(380, 80, "cg")
f2.badge(240, 80, "G", "cg")
f2.under.append('<text x="630" y="72" class="rail" text-anchor="middle">GND (vom GND-Pin des Tiny)</text>')
# buttons B1..B4 on GP6..GP9 (right edge)
for i, (gp, by) in enumerate((("GP6", 150), ("GP7", 200), ("GP8", 250), ("GP9", 300))):
    xv = 620 + i * 20
    f2.wire(f"b{i}", "cbtn", [(X0 + W, r[gp]), (xv, r[gp]), (xv, by), (GX, by)])
    button(f2, 760, by, f"Taster {i + 1}"); f2.dot(GX, by, "cg"); f2.badge(xv, by + (r[gp] - by) / 2, f"T{i + 1}", "cbtn")
# buttons B5..B7 on GP10..GP12 (bottom edge)
for i, (gp, by) in enumerate((("GP10", 410), ("GP11", 452), ("GP12", 494))):
    f2.wire(f"b{i + 4}", "cbtn", [(b[gp], Y0 + H), (b[gp], by), (GX, by)])
    button(f2, 760, by, f"Taster {i + 5}")
    if i < 2:
        f2.dot(GX, by, "cg")
    f2.badge(640, by, f"T{i + 5}", "cbtn")
# pots: 3V3 bus, GND bus, wipers
f2.wire("3v3", "c33", [(X0, l["3V3"]), (300, l["3V3"]), (300, 210), (150, 210), (150, 250)])
f2.wire("3v3", "c33", [(230, 210), (230, 250)]); f2.dot(230, 210, "c33"); f2.badge(340, l["3V3"], "V", "c33")
f2.wire("gnd", "cg", [(150, 310), (150, 400), (100, 400)]); f2.dot(100, 400, "cg")
f2.wire("gnd", "cg", [(230, 310), (230, 400), (150, 400)]); f2.dot(150, 400, "cg")
pot(f2, 150, 250, 310, "POT 1"); pot(f2, 230, 250, 310, "POT 2")
f2.wire("w1", "cpot", [(165, 280), (190, 280), (190, 350), (290, 350), (290, l["GP26"]), (X0, l["GP26"])]); f2.badge(330, l["GP26"], "P1", "cpot")
f2.wire("w2", "cpot", [(245, 280), (270, 280), (270, l["GP27"]), (X0, l["GP27"])]); f2.badge(330, l["GP27"], "P2", "cpot")

# status LED on GP14
f2.wire("led", "csig", [(X0, l["GP14"]), (340, l["GP14"]), (340, 470), (100, 470)])
resistor(f2, 290, 470, False, "220 Ω", 1); diode_left(f2, 200, 470, "Status-LED", "led-g")
f2.badge(340, 420, "L", "csig")

FIG2 = f2.svg("0 0 960 540", "Eingaben am RP2040-Tiny: 7 Taster, 2 Potis, Status-LED")

html = open(os.path.join(os.path.dirname(OUT), "template.html"), encoding="utf-8").read()
html = html.replace("{{FIG1}}", FIG1).replace("{{FIG2}}", FIG2)
html = ('<!doctype html><html lang="de"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">\n'
        + html.replace("</style>\n", "</style>\n</head><body>\n", 1) + "</body></html>\n")
open(OUT, "w", encoding="utf-8").write(html)
print("written", OUT, len(html))


# ─── standalone SVGs for Obsidian (light theme baked in) ────────────────────────
import re as _re
tpl = open(os.path.join(os.path.dirname(OUT), "template.html"), encoding="utf-8").read()
css = tpl[tpl.index("<style>") + 7:tpl.index("</style>")]
root = css[css.index(":root {") + 7:css.index("}", css.index(":root {"))]
rules = [l for l in css.splitlines() if l.startswith("svg ") or l.startswith(".c")]
style = "svg{" + root.replace("\n", "") + "}" + "\n".join(rules)
style = style.replace('"IBM Plex Mono", ', "").replace('"IBM Plex Sans", ', "")


def standalone(svg_markup, path):
    vb = _re.search(r'viewBox="0 0 (\d+) (\d+)"', svg_markup)
    w, h = vb.group(1), vb.group(2)
    body = svg_markup.replace('<svg ', f'<svg width="{w}" height="{h}" ', 1)
    head_end = body.index(">") + 1
    body = body[:head_end] + f'<style>{style}</style><rect width="{w}" height="{h}" fill="#fbfcfc"/>' + body[head_end:]
    open(path, "w", encoding="utf-8").write('<?xml version="1.0" encoding="UTF-8"?>\n' + body)


standalone(FIG1, os.path.join(os.path.dirname(OUT), "steckplan-teil1-lin.svg"))
standalone(FIG2, os.path.join(os.path.dirname(OUT), "steckplan-teil2-eingaben.svg"))
print("svgs written")
