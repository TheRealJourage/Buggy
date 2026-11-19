"""Relay box Nucleo (rear controller): plug-on board for CN7 (left side) on a 6 x 8 cm perfboard (22 x 28 holes).
Right side (CN10, relay bundles) is built. Fan PWM stage and ZL180 driver sit on a second board (cable J9).
Hole coordinates as in router.py: CN7 odd = col 13 (outer), even = col 14 (inner), pin 1 in row 12."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from router import Board

C0, R0 = 5, 10          # board = cols C0..C0+21, rows R0..R0+27


def jst(b, ref, title, pins, at, card, vertical=False):
    c, r = at
    holes = [(c, r + k) if vertical else (c + k, r) for k in range(len(pins))]
    b.part(ref, 'JST', f'JST XH {len(pins)}-polig', [(h, net, str(k + 1)) for k, (h, (net, lab)) in enumerate(zip(holes, pins))], card,
           title, names=[lab for net, lab in pins], title=title)


def build():
    b = Board('L', (C0, C0 + 21), (R0, R0 + 27), 'CN7')
    P = b.pin
    P(1, 'TX3'); P(2, 'RX3'); P(6, 'E5V'); P(16, '3V3'); P(17, 'PWM'); P(19, 'GNDB'); P(21, 'ZLG')
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
    b.preroute('3V3', [(14, 19), (16, 19), (16, 10), (8, 10), (8, 12)])
    # BR1: flat insulated wire bridge; the inner end is under the Nucleo (pin + Dupont plug would be ~14 mm, only ~8 mm free)
    for h in ((15, 14), (8, 18)):
        b.take(h, 'E5V', 'Drahtbrücke BR1')
    b.jumpers.append(dict(net='E5V', a=[15, 14], b=[8, 18], plug=True))
    b.preroute('E5V', [(8, 17), (8, 18)])
    jst(b, 'J3', 'Speeduino Serial3', [('D14', 'D14 = TX3 (rot)'), ('D15', 'D15 = RX3 (weiß)')], (5, 13), 'ser', vertical=True)
    jst(b, 'J4', '5 V (kleine Schiene)', [('E5V', '+5 V'), ('GNDB', 'GND')], (5, 20), 'e5v', vertical=True)
    b.part('C1', 'C', '100 nF', [((8, 20), 'E5V', 'a'), ((8, 21), 'GNDB', 'b')], 'e5v', 'an der Einspeisung, stehend')
    b.part('C2', 'E', '100 µF/25 V (liegend)', [((7, 20), 'E5V', '+'), ((7, 21), 'GNDB', '-')], 'e5v', 'liegend, Körper nach unten (Reihe 23–27 frei); Minus (Streifen) unten')

    # ---------- cable to the second board: fan PWM + ZL180 ----------
    jst(b, 'J9', 'Kabel zur 2. Platine (PWM + ZL180)', [('PWM', 'PA15 Lüfter-PWM'), ('GNDB', 'GND'), ('ZLG', 'PB7 ZL180')], (10, 20), 'pwm', vertical=True)

    # ---------- inputs: strips below CN7, rails GND (row 33) and 3V3 (row 34) under the series parts ----------
    b.preroute('GNDB', [(9, 21), (9, 33), (22, 33)])
    b.preroute('3V3', [(12, 34), (26, 34)])
    b.preroute('3V3', [(16, 19), (26, 19), (26, 32)])

    def series(ref, t, v, c, top, bot, card, note, r=32):
        b.part(ref, t, v, [((c, r), top, 'a'), ((c, r + 4), bot, 'k' if t == 'D' else 'b')], card, note)

    def strip(net, row, c):
        b.preroute(net, [(15, row), (c, row), (c, 32)] if c != 15 else [(15, row), (15, 32)])

    strip('TANK', 30, 15)
    series('R9', 'R', '1 kΩ', 15, 'TANK', 'TANKL', 'tank', 'senkrecht, über beide Schienen')
    b.part('D2', 'D', '1N5819', [((16, 30), 'TANK', 'a'), ((16, 34), '3V3', 'k')], 'tank', 'senkrecht, Ring unten (3,3 V)')
    b.part('R10', 'R', '1 kΩ', [((14, 36), 'TANKL', 'a'), ((14, 35), '3V3', 'b')], 'tank', 'Pull-up für den Geber, stehend (außerhalb des Nucleo)')
    strip('BAT', 29, 17)
    series('R11', 'R', '10 kΩ', 17, 'BAT', 'BATL', 'bat', 'senkrecht, über beide Schienen')
    b.part('D3', 'D', '1N5819', [((18, 30), 'BAT', 'a'), ((18, 34), '3V3', 'k')], 'bat', 'senkrecht, Ring unten (3,3 V)')
    b.part('R12', 'R', '2 kΩ', [((19, 29), 'BAT', 'a'), ((19, 33), 'GNDB', 'b')], 'bat', 'senkrecht')
    strip('K15', 28, 20)
    series('R13', 'R', '10 kΩ', 20, 'K15', 'K15L', 'k15', 'senkrecht, über beide Schienen')
    b.part('D4', 'D', '1N5819', [((21, 30), 'K15', 'a'), ((21, 34), '3V3', 'k')], 'k15', 'senkrecht, Ring unten (3,3 V)')
    b.part('R14', 'R', '2 kΩ', [((22, 29), 'K15', 'a'), ((22, 33), 'GNDB', 'b')], 'k15', 'senkrecht')
    strip('WB', 27, 23); strip('ZS', 26, 24); strip('D47', 25, 25)
    b.part('RN1', 'SIP', '3 × 10 kΩ (SIP-4, gemeinsamer Pin)', [((26, 31), '3V3', '1'), ((25, 31), 'D47', '2'), ((24, 31), 'ZS', '3'), ((23, 31), 'WB', '4')], 'd47',
           'gemeinsamer Pin 1 (Punkt) rechts an 3,3 V; Pull-ups für D47, Zündschalter, Warnblinker', names=['3V3', 'D47', 'ZS', 'WB'])
    for ref, c, net, card in (('D5', 23, 'WB', 'wb'), ('D6', 24, 'ZS', 'zs'), ('D7', 25, 'D47', 'd47')):
        series(ref, 'D', '1N4148 (DO-35)', c, net, net + 'L', card, 'senkrecht, Ring unten (zur Leitung); dünne Bauform, weil die drei Dioden im 2,54-mm-Raster nebeneinander liegen')

    jst(b, 'J6', 'Tankgeber', [('GNDB', 'GND (2. Ader)'), ('TANKL', 'Geber (1. Ader)')], (6, 37), 'tank')
    jst(b, 'J7', 'Box intern', [('BATL', 'Batterie +12 V'), ('K15L', 'Kl. 15 (ZL180)')], (18, 37), 'k15')
    jst(b, 'J8', 'Schalter + Speeduino', [('WBL', 'Warnblinkschalter'), ('ZSL', 'Zündschalter'), ('D47L', 'Speeduino D47')], (23, 37), 'zs')

    # ---------- CAN module standing in a socket, bottom left (outside), wires from CN10 ----------
    can = ['3V3', 'GND', 'CTX', 'CRX', 'CANH', 'CANL']
    b.part('U2', 'MOD', 'SN65HVD230-Modul (gesteckt)', [((6 + k, 35), n if n != 'GND' else 'GNDB', n) for k, n in enumerate(can)], 'can',
           'in Buchsenleiste 6-polig, Modul steht senkrecht (außerhalb des Nucleo); Pinfolge am eigenen Modul prüfen', names=can)
    b.pad((8, 36), 'CTX', 'PA12 → Draht von CN10-12', 'L', 'can')
    b.pad((9, 36), 'CRX', 'PA11 ← Draht von CN10-14', 'L', 'can')
    jst(b, 'J5', 'CAN-Bus', [('CANL', 'CAN-L'), ('CANH', 'CAN-H')], (12, 35), 'can', vertical=True)

    order = ['TX3', 'RX3', 'D14', 'D15', 'PWM', 'ZLG', 'CRX', 'CTX', 'CANH', 'CANL',
             'TANK', 'TANKL', 'BAT', 'BATL', 'K15', 'K15L', 'WB', 'WBL', 'ZS', 'ZSL', 'D47', 'D47L', 'E5V', '3V3', 'GNDB']
    return b, order


LEG = {'GNDB': 'G', '3V3': 'V', 'E5V': '5', 'TX3': 't', 'RX3': 'r', 'D14': 'x', 'D15': 'y', 'PWM': 'p', 'ZLG': 'z',
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
