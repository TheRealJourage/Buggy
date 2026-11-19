"""Relay box Nucleo (rear controller): plug-on board for CN7 (left side). Right side (CN10, relay bundles) is built.
Hole coordinates as in router.py: CN7 odd = col 13 (outer), even = col 14 (inner), pin 1 in row 12."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from router import Board


def jst(b, ref, title, pins, at, card, vertical=False):
    c, r = at
    holes = [(c, r + k) if vertical else (c + k, r) for k in range(len(pins))]
    b.part(ref, 'JST', f'JST XH {len(pins)}-polig', [(h, net, str(k + 1)) for k, (h, (net, lab)) in enumerate(zip(holes, pins))], card,
           title, names=[lab for net, lab in pins], title=title)


def build():
    b = Board('L', (1, 33), (5, 40), 'CN7')
    P = b.pin
    P(1, 'TX3'); P(2, 'RX3'); P(6, 'E5V'); P(8, 'GND'); P(16, '3V3'); P(17, 'PWM'); P(19, 'GNDB'); P(21, 'ZLG')
    P(28, 'D47'); P(30, 'ZS'); P(32, 'WB'); P(34, 'K15'); P(36, 'BAT'); P(38, 'TANK')

    # ---------- left of the Nucleo (outside the outline, height free): fan PWM + ZL180, GND rail col 11 ----------
    b.part('R1', 'R', '1 kΩ', [((12, 20), 'PWM', 'a'), ((8, 20), 'BASE', 'b')], 'pwm', 'Basiswiderstand, liegt über der GND-Schiene')
    b.part('R2', 'R', '10 kΩ', [((8, 21), 'BASE', 'a'), ((12, 21), 'GNDB', 'b')], 'pwm', 'Basis nach GND')
    b.part('Q1', 'TO92', 'BC547B', [((8, 18), 'COL', 'C'), ((9, 18), 'BASE', 'B'), ((10, 18), 'GNDB', 'E')], 'pwm',
           'stehend, flache Seite nach unten; C-B-E vor dem Einlöten messen', names=['C', 'B', 'E'])
    b.part('R3', 'R', '2 kΩ', [((8, 17), 'COL', 'a'), ((4, 17), 'KL15P', 'b')], 'pwm', 'Pull-up an Kl. 15, mit R4 parallel = 1 kΩ')
    b.part('R4', 'R', '2 kΩ', [((8, 16), 'COL', 'a'), ((4, 16), 'KL15P', 'b')], 'pwm', 'parallel zu R3')
    b.part('R5', 'R', '100 Ω', [((8, 15), 'COL', 'a'), ((4, 15), 'PWMO', 'b')], 'pwm', 'Reihenwiderstand zur Leitung')
    b.part('D1', 'TVS', '1.5KE24A', [((4, 13), 'PWMO', 'k'), ((10, 13), 'GNDB', 'a')], 'pwm', 'liegend, Ring links (Leitung); Löcher ggf. auf 1,3 mm aufbohren')
    jst(b, 'J1', 'Lüfter-PWM (J293)', [('PWMO', 'PWM → J293'), ('KL15P', 'Kl. 15 (12 V)')], (1, 15), 'pwm', vertical=True)

    b.part('R6', 'R', '1 kΩ', [((12, 22), 'ZLG', 'a'), ((8, 22), 'GATE', 'b')], 'zl', 'Gate-Widerstand, liegt über der GND-Schiene')
    b.part('R7', 'R', '10 kΩ', [((8, 23), 'GATE', 'a'), ((4, 23), 'SRC', 'b')], 'zl', 'Gate nach Source')
    b.part('Q2', 'TO220', 'IRLZ44N', [((8, 24), 'GATE', 'G'), ((7, 24), 'DRN', 'D'), ((6, 24), 'SRC', 'S')], 'zl',
           'stehend (außerhalb des Nucleo), Metallfahne nach oben; G-D-S von vorn = rechts → links', names=['G', 'D', 'S'])
    jst(b, 'J2', 'ZL180 + Masse', [('SRC', 'Source → Massepunkt'), ('DRN', 'ZL180-Spule (−)'), ('GNDB', 'GND → Massepunkt')], (1, 26), 'zl', vertical=True)

    # ---------- level shifter on sockets, left of the Nucleo; channel 3 so 3V3 / GND / TX arrive in order ----------
    lv = ['LV1', 'LV2', 'LV', 'GND', 'LV3', 'LV4']
    hv = ['HV1', 'HV2', 'HV', 'GND', 'HV3', 'HV4']
    lvn = [None, None, '3V3', 'GND', 'TX3', None]
    hvn = [None, None, 'E5V', 'GND', 'D15', None]
    b.part('U1', 'MOD', 'Pegelwandler 4 Kanäle (gesteckt)', [((10, 7 + k), lvn[k], lv[k]) for k in range(6)] + [((5, 7 + k), hvn[k], hv[k]) for k in range(6)],
           'ser', 'auf 2 × Buchsenleiste 6-polig, LV-Seite zum Nucleo; Kanal 3 benutzt. Pinfolge und Abstand der Reihen (5 Raster) = Hypothese, am Modul prüfen',
           names=lv + hv)
    b.part('R8', 'R', '10 kΩ', [((14, 11), 'RX3', 'a'), ((14, 7), 'D14', 'b')], 'ser', 'senkrecht, liegt über der 3,3-V- und GND-Bahn')
    b.preroute('TX3', [(13, 12), (12, 12), (12, 11), (10, 11)])
    b.preroute('3V3', [(14, 19), (17, 19), (17, 9), (10, 9)])
    b.preroute('GND', [(14, 15), (16, 15), (16, 10), (5, 10)])
    jst(b, 'J3', 'Speeduino Serial3 + 5 V', [('D14', 'D14 = TX3 (rot)'), ('E5V', '+5 V kleine Schiene'), ('GND', 'GND'), ('D15', 'D15 = RX3 (weiß)')], (1, 8), 'ser', vertical=True)
    b.part('C1', 'C', '100 nF', [((3, 9), 'E5V', 'a'), ((3, 10), 'GND', 'b')], 'e5v', 'an der Einspeisung')
    b.part('C2', 'E', '100 µF/25 V (stehend)', [((2, 9), 'E5V', '+'), ((2, 10), 'GND', '-')], 'e5v', 'stehend außerhalb des Nucleo; Minus (Streifen) unten')
    for h in ((15, 14), (4, 9)):
        b.take(h, 'E5V', 'Steckbrücke BR1')
    b.jumpers.append(dict(net='E5V', a=[15, 14], b=[4, 9], plug=True))
    b.part('C3', 'E', '10 µF/25 V (stehend)', [((13, 34), '3V3', '+'), ((13, 35), 'GNDB', '-')], 'v3', 'stehend (außerhalb des Nucleo), Minus unten')

    # ---------- CAN module standing in a socket below the Nucleo, on the rails; two wires from CN10 ----------
    can = ['3V3', 'GND', 'CTX', 'CRX', 'CANH', 'CANL']
    b.part('U2', 'MOD', 'SN65HVD230-Modul (gesteckt)', [((33, 34 + k), n if n != 'GND' else 'GNDB', n) for k, n in enumerate(can)], 'can',
           'in Buchsenleiste 6-polig, Modul steht senkrecht (außerhalb des Nucleo); Pinfolge am eigenen Modul prüfen', names=can)
    b.pad((32, 36), 'CTX', 'PA12 → Draht von CN10-12', 'R', 'can')
    b.pad((32, 37), 'CRX', 'PA11 ← Draht von CN10-14', 'R', 'can')
    jst(b, 'J5', 'CAN-Bus', [('CANH', 'CAN-H'), ('CANL', 'CAN-L')], (32, 40), 'can')

    # ---------- inputs: one vertical strip per pin, rails 3V3 (row 34) and GND (row 35) under the series parts ----------
    b.preroute('GNDB', [(11, 13), (11, 35), (33, 35)])
    b.preroute('3V3', [(14, 34), (33, 34)])
    b.preroute('3V3', [(17, 19), (33, 19), (33, 33)])

    def series(ref, t, v, c, top, bot, card, note):
        b.part(ref, t, v, [((c, 33), top, 'a'), ((c, 37), bot, 'k' if t == 'D' else 'b')], card, note)

    def strip(net, row, c, card):
        b.preroute(net, [(15, row), (c, row), (c, 33)] if c != 15 else [(15, row), (15, 33)])

    # tank (PC0, row 30)
    strip('TANK', 30, 15, 'tank')
    series('R9', 'R', '1 kΩ', 15, 'TANK', 'TANKL', 'tank', 'senkrecht, über beide Schienen')
    b.part('D2', 'D', '1N5819', [((16, 30), 'TANK', 'a'), ((16, 34), '3V3', 'k')], 'tank', 'senkrecht, Ring unten (3,3 V)')
    b.part('C4', 'C', '100 nF', [((17, 33), 'TANK', 'a'), ((17, 35), 'GNDB', 'b')], 'tank')
    b.part('R10', 'R', '1 kΩ', [((14, 38), 'TANKL', 'a'), ((14, 34), '3V3', 'b')], 'tank', 'Pull-up für den Geber, senkrecht')
    # battery (PC1, row 29)
    strip('BAT', 29, 19, 'bat')
    series('R11', 'R', '10 kΩ', 19, 'BAT', 'BATL', 'bat', 'senkrecht, über beide Schienen')
    b.part('D3', 'D', '1N5819', [((20, 30), 'BAT', 'a'), ((20, 34), '3V3', 'k')], 'bat', 'senkrecht, Ring unten (3,3 V)')
    b.part('R12', 'R', '2 kΩ', [((21, 31), 'BAT', 'a'), ((21, 35), 'GNDB', 'b')], 'bat', 'senkrecht')
    b.part('C5', 'C', '100 nF', [((22, 33), 'BAT', 'a'), ((22, 35), 'GNDB', 'b')], 'bat')
    # Kl. 15 feedback (PB0, row 28)
    strip('K15', 28, 23, 'k15')
    series('R13', 'R', '10 kΩ', 23, 'K15', 'K15L', 'k15', 'senkrecht, über beide Schienen')
    b.part('D4', 'D', '1N5819', [((24, 30), 'K15', 'a'), ((24, 34), '3V3', 'k')], 'k15', 'senkrecht, Ring unten (3,3 V)')
    b.part('R14', 'R', '2 kΩ', [((25, 31), 'K15', 'a'), ((25, 35), 'GNDB', 'b')], 'k15', 'senkrecht')
    b.part('C6', 'C', '100 nF', [((26, 33), 'K15', 'a'), ((26, 35), 'GNDB', 'b')], 'k15')
    # digital inputs: pull-up + series diode (ring towards the connector)
    for ref, rd, net, row, c, card in (('R15', 'D5', 'WB', 27, 27, 'wb'), ('R16', 'D6', 'ZS', 26, 29, 'zs'), ('R17', 'D7', 'D47', 25, 31, 'd47')):
        strip(net, row, c, card)
        b.part(ref, 'R', '10 kΩ', [((c + 1, 30), net, 'a'), ((c + 1, 34), '3V3', 'b')], card, 'Pull-up, senkrecht')
        series(rd, 'D', '1N5819', c, net, net + 'L', card, 'senkrecht, Ring unten (zur Leitung)')

    jst(b, 'J6', 'Tankgeber', [('TANKL', 'Geber (1. Ader)'), ('GNDB', 'GND (2. Ader)')], (15, 39), 'tank')
    jst(b, 'J7', 'Box intern', [('BATL', 'Batterie +12 V'), ('K15L', 'Kl. 15 (ZL180)'), ('WBL', 'Warnblinkschalter')], (21, 39), 'k15')
    jst(b, 'J8', 'Kabelbaum', [('ZSL', 'Zündschalter'), ('D47L', 'Speeduino D47')], (29, 39), 'zs')

    order = ['TX3', 'RX3', 'D14', 'D15', 'PWM', 'BASE', 'COL', 'KL15P', 'PWMO', 'ZLG', 'GATE', 'DRN', 'SRC', 'CRX', 'CTX', 'CANH', 'CANL',
             'TANK', 'TANKL', 'BAT', 'BATL', 'K15', 'K15L', 'WB', 'WBL', 'ZS', 'ZSL', 'D47', 'D47L', 'E5V', '3V3', 'GND', 'GNDB']
    return b, order


LEG = {'GND': 'g', 'GNDB': 'G', '3V3': 'v', '3V3': 'V', 'E5V': '5', 'TX3': 't', 'RX3': 'r', 'D14': 'x', 'D15': 'y', 'PWM': 'p', 'BASE': 'b',
       'COL': 'c', 'KL15P': 'k', 'PWMO': 'o', 'ZLG': 'z', 'GATE': 'q', 'DRN': 'd', 'SRC': 's', 'CRX': 'R', 'CTX': 'T', 'CANH': 'H', 'CANL': 'L',
       'TANK': '1', 'TANKL': '!', 'BAT': '2', 'BATL': '@', 'K15': '3', 'K15L': '#', 'WB': '4', 'WBL': '$', 'ZS': '5', 'ZSL': '%', 'D47': '6', 'D47L': '^'}

if __name__ == '__main__':
    b, order = build()
    for n in order:
        try:
            b.route(n)
        except RuntimeError as e:
            print('FAIL', e)
    print(b.ascii(LEG))
    print('jumpers', b.jumpers)
    print('bad', b.check(order))
