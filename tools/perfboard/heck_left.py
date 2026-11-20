"""Relay box Nucleo (rear controller): plug-on board for CN7 (left side) on a 6 x 8 cm perfboard (22 x 28 holes).
Right side (CN10, relay bundles) is built. Fan PWM stage and ZL180 driver sit on a second board (cable J9).
Hole coordinates as in router.py: CN7 odd = col 13 (outer), even = col 14 (inner), pin 1 in row 12."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from router import Board

C0, R0 = 5, 10          # board = cols C0..C0+21, rows R0..R0+26 (22 x 27 holes, rows A..AA)


def jst(b, ref, title, pins, at, card, vertical=False):
    c, r = at
    holes = [(c, r + k) if vertical else (c + k, r) for k in range(len(pins))]
    b.part(ref, 'JST', f'JST XH {len(pins)}-polig', [(h, net, str(k + 1)) for k, (h, (net, lab)) in enumerate(zip(holes, pins))], card,
           title, names=[lab for net, lab in pins], title=title)


def build():
    b = Board('L', (C0, C0 + 21), (R0, R0 + 26), 'CN7')
    P = b.pin
    P(1, 'TX3'); P(2, 'RX3'); P(6, 'E5V'); P(8, 'GND'); P(16, '3V3'); P(17, 'PWM'); P(19, 'GNDB'); P(21, 'ZLG')
    P(28, 'D47'); P(30, 'ZS'); P(32, 'WB'); P(34, 'K15'); P(36, 'BAT'); P(38, 'TANK')

    # ---------- Serial3: level shifter on sockets left of CN7, channel 4 (LV4 sits right at PC10) ----------
    lv = ['LV1', 'LV2', 'LV', 'GND', 'LV3', 'LV4']
    hv = ['HV1', 'HV2', 'HV', 'GND', 'HV3', 'HV4']
    lvn = [None, None, '3V3', None, None, 'TX3']
    hvn = [None, None, 'E5V', 'GNDB', None, 'D15']
    b.part('U1', 'MOD', 'Pegelwandler 4 Kanäle (gesteckt)', [((6 + k, 12), lvn[k], lv[k]) for k in range(6)] + [((6 + k, 17), hvn[k], hv[k]) for k in range(6)],
           'ser', 'auf 2 × Buchsenleiste 6-polig, LV-Reihe oben (an PC10), Kanal 4. LV-GND bleibt frei: GND kommt über HV-GND (beide GND auf dem Modul verbunden = Hypothese, mit Durchgangsprüfer prüfen). Pinfolge und Reihenabstand (5 Raster) am Modul prüfen',
           names=lv + hv)
    b.preroute('TX3', [(13, 12), (11, 12)])
    b.preroute('RX3', [(14, 12), (14, 11), (13, 11)])
    b.part('R8', 'R', '10 kΩ', [((13, 11), 'RX3', 'a'), ((7, 11), 'D14', 'b')], 'ser', 'liegend, springt über die 3,3-V-Bahn zum Pegelwandler')
    b.preroute('3V3', [(14, 19), (19, 19), (19, 10), (8, 10), (8, 12)])
    # BR1: flat insulated wire bridge; the inner end is under the Nucleo
    for h in ((15, 14), (8, 18)):
        b.take(h, 'E5V', 'Drahtbrücke BR1')
    b.jumpers.append(dict(net='E5V', a=[15, 14], b=[8, 18], plug=True))
    b.preroute('E5V', [(8, 17), (8, 18)])
    W = lambda h, net, label, card: b.pad(h, net, label, 'W', card)
    W((5, 13), 'D14', 'Speeduino D14 (TX3, rot)', 'ser')
    W((5, 14), 'D15', 'Speeduino D15 (RX3, weiß)', 'ser')
    # 5 V feed right at E5V, under the Nucleo (flat parts only)
    b.preroute('GND', [(14, 15), (16, 15)])
    b.part('C2', 'E', '100 µF/25 V (liegend)', [((16, 14), 'E5V', '+'), ((16, 15), 'GND', '-')], 'e5v', 'liegend unter dem Nucleo, Körper nach oben (Reihe B–E); Minus (Streifen) unten')
    b.part('C1', 'C', '100 nF', [((17, 14), 'E5V', 'a'), ((17, 15), 'GND', 'b')], 'e5v', 'stehend, flache Bauform (unter dem Nucleo)')
    b.preroute('E5V', [(15, 14), (17, 14)])
    W((18, 14), 'E5V', '+5 V kleine Schiene', 'e5v')
    W((18, 15), 'GND', 'GND (5-V-Zuleitung)', 'e5v')

    # ---------- fan PWM (BC547B) + ZL180 driver (IRLZ44N) left of CN7, outside the outline ----------
    b.part('R1', 'R', '1 kΩ', [((12, 20), 'PWM', 'a'), ((8, 20), 'BASE', 'b')], 'pwm', 'liegend, Basiswiderstand')
    b.part('R2', 'R', '10 kΩ', [((8, 21), 'BASE', 'a'), ((12, 21), 'GNDB', 'b')], 'pwm', 'liegend, Basis nach GND')
    b.part('Q1', 'TO92', 'BC547B', [((7, 20), 'COL', 'C'), ((7, 21), 'BASE', 'B'), ((7, 22), 'GNDB', 'E')], 'pwm',
           'stehend, flache Seite nach links; C-B-E vor dem Einlöten messen', names=['C', 'B', 'E'])
    b.part('R3', 'R', '2 kΩ', [((6, 19), 'COL', 'a'), ((5, 19), 'KL15P', 'b')], 'pwm', 'stehend; mit R4 parallel = 1 kΩ an Kl. 15')
    b.part('R4', 'R', '2 kΩ', [((6, 18), 'COL', 'a'), ((5, 18), 'KL15P', 'b')], 'pwm', 'stehend, parallel zu R3')
    b.part('R5', 'R', '100 Ω', [((6, 20), 'COL', 'a'), ((5, 20), 'PWMO', 'b')], 'pwm', 'stehend, Reihenwiderstand zur Leitung')
    b.part('D1', 'TVS', '1.5KE24A', [((5, 22), 'PWMO', 'k'), ((5, 28), 'GNDB', 'a')], 'pwm', 'liegend senkrecht, Ring oben (Leitung); Löcher ggf. auf 1,3 mm aufbohren')
    W((5, 17), 'KL15P', 'Kl. 15 (12 V) für PWM-Pull-up', 'pwm')
    W((5, 21), 'PWMO', 'Lüfter-PWM → J293', 'pwm')
    b.part('R6', 'R', '1 kΩ', [((12, 23), 'ZLG', 'a'), ((12, 27), 'GATE', 'b')], 'zl', 'liegend senkrecht, Gate-Widerstand')
    b.preroute('ZLG', [(13, 22), (12, 22), (12, 23)])
    b.part('Q2', 'TO220', 'IRLZ44N', [((11, 27), 'GATE', 'G'), ((10, 27), 'DRN', 'D'), ((9, 27), 'SRC', 'S')], 'zl',
           'stehend (außerhalb des Nucleo) direkt neben dem Ende von R6, Metallfahne nach oben, Schrift nach unten (zu J2); G-D-S von der Schrift aus = links → rechts', names=['G', 'D', 'S'])
    b.part('R7', 'R', '10 kΩ', [((12, 29), 'GATE', 'a'), ((8, 29), 'SRC', 'b')], 'zl', 'liegend zwischen Q2 und J2 (über J2), Gate nach Source; S/D-Bahnen laufen darunter durch')
    # J2 right under Q2 (User 2026-11-20): coil (-), source and board GND to the ground point in one 3-pin plug
    jst(b, 'J2', 'ZL180 + Masse', [('SRC', 'Source → Massepunkt (eigene Ader)'), ('DRN', 'ZL180-Spule (−)'), ('GNDB', 'GND → Massepunkt')], (9, 31), 'zl')

    # ---------- inputs: strips below CN7, rails GND (row 33) and 3V3 (row 34) under the series parts ----------
    b.preroute('GNDB', [(6, 23), (6, 33), (22, 33)])
    b.preroute('3V3', [(6, 34), (26, 34)])
    b.preroute('3V3', [(19, 19), (26, 19), (26, 30)])

    def series(ref, t, v, c, top, bot, card, note, r=31):
        b.part(ref, t, v, [((c, r), top, 'a'), ((c, r + 4), bot, 'k' if t == 'D' else 'b')], card, note)

    def strip(net, row, c):
        b.preroute(net, [(15, row), (c, row), (c, 31)] if c != 15 else [(15, row), (15, 31)])

    strip('TANK', 30, 15)
    series('R9', 'R', '1 kΩ', 15, 'TANK', 'TANKL', 'tank', 'senkrecht, über beide Schienen')
    b.part('D2', 'D', '1N5819', [((16, 30), 'TANK', 'a'), ((16, 34), '3V3', 'k')], 'tank', 'senkrecht, Ring unten (3,3 V)')
    b.part('R10', 'R', '1 kΩ', [((14, 35), 'TANKL', 'a'), ((14, 34), '3V3', 'b')], 'tank', 'Pull-up für den Geber, stehend (außerhalb des Nucleo)')
    strip('BAT', 29, 17)
    series('R11', 'R', '10 kΩ', 17, 'BAT', 'BATL', 'bat', 'senkrecht, über beide Schienen')
    b.part('D3', 'D', '1N5819', [((18, 30), 'BAT', 'a'), ((18, 34), '3V3', 'k')], 'bat', 'senkrecht, Ring unten (3,3 V)')
    b.part('R12', 'R', '2 kΩ', [((19, 29), 'BAT', 'a'), ((19, 33), 'GNDB', 'b')], 'bat', 'senkrecht')
    strip('K15', 28, 20)
    series('R13', 'R', '10 kΩ', 20, 'K15', 'K15L', 'k15', 'senkrecht, über beide Schienen')
    b.part('D4', 'D', '1N5819', [((21, 30), 'K15', 'a'), ((21, 34), '3V3', 'k')], 'k15', 'senkrecht, Ring unten (3,3 V)')
    b.part('R14', 'R', '2 kΩ', [((22, 29), 'K15', 'a'), ((22, 33), 'GNDB', 'b')], 'k15', 'senkrecht')
    strip('WB', 27, 23); strip('ZS', 26, 24); strip('D47', 25, 25)
    # pull-ups as single 1/4 W resistors, lying and long: from each lane up to the 3V3 lane (row 19), jumping over the lanes above
    for ref, net, row, c, card in (('R15', 'D47', 25, 18, 'd47'), ('R16', 'ZS', 26, 20, 'zs'), ('R17', 'WB', 27, 22, 'wb')):
        b.part(ref, 'R', '10 kΩ', [((c, row), net, 'a'), ((c, 19), '3V3', 'b')], card, f'liegend senkrecht, lang ({row - 19} Raster), springt über die Bahnen darüber; Pull-up an 3,3 V')
    for ref, c, net, card in (('D5', 23, 'WB', 'wb'), ('D6', 24, 'ZS', 'zs'), ('D7', 25, 'D47', 'd47')):
        series(ref, 'D', '1N4148 (DO-35)', c, net, net + 'L', card, 'senkrecht, Ring unten (zur Leitung); dünne Bauform, weil die drei Dioden im 2,54-mm-Raster nebeneinander liegen')

    for h, net, lab, card in (((15, 36), 'TANKL', 'Tankgeber (1. Ader)', 'tank'), ((13, 32), 'GNDB', 'Tankgeber GND (2. Ader)', 'tank'),
                              ):
        W(h, net, lab, card)
    # JST housings would sit on the lower leads of the series parts (row 35) -> soldered cables
    for h, net, lab, card in (((17, 36), 'BATL', 'Batterie +12 V', 'bat'), ((20, 36), 'K15L', 'Kl. 15 (ZL180-Ausgang)', 'k15'),
                              ((23, 36), 'WBL', 'Warnblinkschalter', 'wb'), ((24, 36), 'ZSL', 'Zündschalter', 'zs'), ((25, 36), 'D47L', 'Speeduino D47', 'd47')):
        W(h, net, lab, card)

    # ---------- CAN module standing in a socket, bottom left (outside), wires from CN10 ----------
    can = ['3V3', 'GND', 'CTX', 'CRX', 'CANH', 'CANL']
    b.part('U2', 'MOD', 'SN65HVD230-Modul (gesteckt)', [((6 + k, 35), n if n != 'GND' else 'GNDB', n) for k, n in enumerate(can)], 'can',
           'in Buchsenleiste 6-polig, Modul steht senkrecht (außerhalb des Nucleo); Pinfolge am eigenen Modul prüfen', names=can)
    W((8, 36), 'CTX', 'PA12 ← Draht von CN10-12', 'can')
    W((9, 36), 'CRX', 'PA11 ← Draht von CN10-14', 'can')
    W((10, 36), 'CANH', 'CAN-H (Bus)', 'can')
    W((11, 36), 'CANL', 'CAN-L (Bus)', 'can')

    order = ['TX3', 'RX3', 'D14', 'D15', 'PWM', 'ZLG', 'CRX', 'CTX', 'CANH', 'CANL',
             'TANK', 'TANKL', 'BAT', 'BATL', 'K15', 'K15L', 'WB', 'WBL', 'ZS', 'ZSL', 'D47', 'D47L', 'BASE', 'COL', 'KL15P', 'PWMO', 'GATE', 'DRN', 'SRC', 'E5V', '3V3', 'GND', 'GNDB']
    return b, order


LEG = {'GND': 'g', 'BASE': 'b', 'COL': 'c', 'KL15P': 'k', 'PWMO': 'o', 'GATE': 'q', 'DRN': 'd', 'SRC': 's', 'GNDB': 'G', '3V3': 'V', 'E5V': '5', 'TX3': 't', 'RX3': 'r', 'D14': 'x', 'D15': 'y', 'PWM': 'p', 'ZLG': 'z',
       'CRX': 'R', 'CTX': 'T', 'CANH': 'H', 'CANL': 'L', 'TANK': '1', 'TANKL': '!', 'BAT': '2', 'BATL': '@', 'K15': '3', 'K15L': '#',
       'WB': '4', 'WBL': '$', 'ZS': '5', 'ZSL': '%', 'D47': '6', 'D47L': '^'}

if __name__ == '__main__':
    b, order = build()
    for n in order:
        try:
            b.route(n)
        except RuntimeError as e:
            print('FAIL', e)
    print(b.ascii(LEG))
    print('jumpers', [j for j in b.jumpers])
    print('bad', b.check(order))
