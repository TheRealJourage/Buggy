from router import Board

def build():
    b = Board('R', (37, 56), (0, 41), 'CN10')
    P = b.pin
    P(2, 'ABS'); P(4, 'LED'); P(6, 'WPR'); P(12, 'CTX'); P(14, 'CRX'); P(16, 'KAN'); P(9, 'GND')
    P(22, 'FAN'); P(24, 'WPF'); P(26, 'KAU'); P(28, 'S2'); P(30, 'S1'); P(34, 'POT')
    # rails: GND from the inner pin CN10-9 under the Nucleo to the top row and down the right edge; 3V3 inside
    b.preroute('GND', [(38, 16), (37, 16), (37, 0), (56, 0), (56, 41)])
    b.preroute('3V3', [(54, 2), (54, 40)])
    # fan-out staircase (upper group turns up, lower group turns down)
    b.preroute('ABS', [(39, 12), (40, 12), (40, 2), (50, 2)])
    b.preroute('LED', [(39, 13), (41, 13), (41, 10), (45, 10), (45, 9)])
    b.preroute('WPR', [(39, 14), (42, 14), (42, 11), (50, 11)])
    b.preroute('CTX', [(39, 17), (48, 17)])
    b.preroute('CRX', [(39, 18), (48, 18)])
    b.preroute('KAN', [(39, 19), (46, 19), (46, 21), (52, 21)])
    b.preroute('FAN', [(39, 22), (45, 22), (45, 24), (52, 24)])
    b.preroute('WPF', [(39, 23), (44, 23), (44, 27), (50, 27)])
    b.preroute('KAU', [(39, 24), (43, 24), (43, 29), (52, 29)])
    b.preroute('S2', [(39, 25), (42, 25), (42, 32), (52, 32)])
    b.preroute('S1', [(39, 26), (41, 26), (41, 35), (52, 35)])
    b.preroute('POT', [(39, 28), (40, 28), (40, 38), (52, 38)])
    # ABS: divider, clamp, 1 k to Kl. 15
    b.part('D1', 'D', '1N5819', [((54, 2), '3V3', 'k'), ((50, 2), 'ABS', 'a')], 'abs', 'Ring rechts an der 3,3-V-Schiene')
    b.part('R1', 'R', '2,7 kΩ', [((41, 1), 'GND', 'a'), ((45, 1), 'ABS', 'b')], 'abs')
    b.part('C1', 'C', '100 nF', [((47, 1), 'GND', 'a'), ((49, 1), 'ABS', 'b')], 'abs')
    b.part('R2', 'R', '10 kΩ', [((42, 4), 'ABSL', 'a'), ((46, 4), 'ABS', 'b')], 'abs', 'Reihenwiderstand Leitung → Pin')
    b.part('R3', 'R', '1 kΩ 0,5 W', [((42, 5), 'ABSL', 'a'), ((46, 5), 'KL15', 'b')], 'abs', '0,5-W-Typ (0,21 W gerechnet)')
    b.pad((41, 4), 'ABSL', 'J104 Pin 16 (ABS-Lampe)', 'R', 'abs')
    b.pad((47, 5), 'KL15', 'Kl. 15 (V4)', 'R', 'abs')
    # LED driver 74AHCT125, notch left, pins 1-7 bottom row (row 9), 14-8 top row (row 6)
    names = ['1OE', '1A', '1Y', '2OE', '2A', '2Y', 'GND', '3Y', '3A', '3OE', '4Y', '4A', '4OE', 'VCC']
    nets = ['GND', 'LED', 'LEDY', 'GND', 'GND', None, 'GND', None, 'V5', 'V5', None, 'V5', 'V5', 'V5']
    holes = [(44 + k, 9) for k in range(7)] + [(50 - k, 6) for k in range(7)]
    b.part('U1', 'DIP', '74AHCT125 (DIP-14, Sockel)', [(h, nets[k], str(k + 1)) for k, h in enumerate(holes)], 'led',
           'Kerbe links; Pinfolge im Datenblatt prüfen. Gatter 2 an Masse, Gatter 3/4 an 5 V (aus)', notch='L', names=names)
    b.part('R4', 'R', '330 Ω', [((48, 10), 'LEDY', 'a'), ((52, 10), 'DIN', 'b')], 'led')
    b.pad((53, 10), 'DIN', 'DIN LED-Leiste', 'R', 'led')
    b.part('C2', 'C', '100 nF', [((52, 7), 'V5', 'a'), ((52, 8), 'GND', 'b')], 'led', 'dicht am Chip, 5 V–GND')
    b.pad((51, 7), 'V5', '+5 V große 5-V-Schiene', 'R', 'led')
    # wiper stalk rear
    b.part('R5', 'R', '220 Ω', [((50, 11), 'WPR', 'a'), ((54, 11), '3V3', 'b')], 'stalk')
    b.pad((47, 11), 'WPR', 'Wischerhebel Signal hinten', 'R', 'stalk')
    b.pad((54, 13), '3V3', '3,3 V von der linken Platine', 'R', 'v3')
    # CAN module standing, pins in col 48 rows 15-20
    b.part('U2', 'CANM', 'SN65HVD230-Modul (stehend)', [((48, 15), '3V3', '3V3'), ((48, 16), 'GND', 'GND'), ((48, 17), 'CTX', 'CTX'),
           ((48, 18), 'CRX', 'CRX'), ((48, 19), 'CANH', 'CANH'), ((48, 20), 'CANL', 'CANL')], 'can', 'stehend eingelötet, Bauteilseite nach rechts; Pinfolge am Modul prüfen')
    b.pad((49, 19), 'CANH', 'CAN-H → Stecker CAN Front', 'R', 'can')
    b.pad((49, 20), 'CANL', 'CAN-L → Stecker CAN Front', 'R', 'can')
    # panel + clutch inputs: R to 3V3 on the main row, C to GND one row below
    def pullup(net, row, card, label, diode=None):
        b.part(f'R_{net}', 'R', '10 kΩ', [((50, row), net, 'a'), ((54, row), '3V3', 'b')], card, 'Pull-up, rechts in die 3,3-V-Schiene')
        b.part(f'C_{net}', 'C', '100 nF', [((52, row + 1), net, 'a'), ((56, row + 1), 'GND', 'b')], card, 'springt über die 3,3-V-Schiene in die GND-Schiene')
        if diode:
            k, a, padh = diode
            b.part(f'D_{net}', 'D', '1N5819', [(k, net + 'L', 'k'), (a, net, 'a')], card, 'Ring links (zur Leitung)')
            b.pad(padh, net + 'L', label, 'R', card)
        else:
            b.pad((47, row), net, label, 'R', card)
    pullup('KAN', 21, 'comp', 'Kompressor AN')
    pullup('FAN', 24, 'fan', 'Lüfter Auto/manuell')
    b.part('R6', 'R', '220 Ω', [((50, 27), 'WPF', 'a'), ((54, 27), '3V3', 'b')], 'stalk')
    b.pad((47, 27), 'WPF', 'Wischerhebel Signal vorn', 'R', 'stalk')
    b.pad((56, 26), 'GND', 'Wischerhebel Masse', 'R', 'stalk')
    pullup('KAU', 29, 'comp', 'Kompressor AUTO')
    b.pad((56, 28), 'GND', 'Kompressor Mittelkontakt (Masse)', 'R', 'comp')
    pullup('S2', 32, 'clutch', 'Kupplung S2', ((44, 33), (48, 33), (43, 33)))
    pullup('S1', 35, 'clutch', 'Kupplung S1', ((43, 36), (47, 36), (42, 36)))
    b.part('C_POT', 'C', '100 nF', [((52, 39), 'POT', 'a'), ((56, 39), 'GND', 'b')], 'poti', 'springt über die 3,3-V-Schiene')
    b.pad((47, 38), 'POT', 'Lüfter-Poti Schleifer', 'R', 'poti')
    b.pad((54, 40), '3V3', 'Lüfter-Poti Ende 3,3 V', 'R', 'poti')
    b.pad((56, 37), 'GND', 'Lüfter-Poti Ende GND', 'R', 'poti')
    order = ['ABS', 'ABSL', 'KL15', 'LED', 'LEDY', 'DIN', 'V5', 'WPR', 'CTX', 'CRX', 'CANH', 'CANL', 'KAN', 'FAN', 'WPF', 'KAU', 'S2', 'S2L', 'S1', 'S1L', 'POT', '3V3', 'GND']
    return b, order

if __name__ == '__main__':
    b, order = build()
    for n in order:
        b.route(n)
    print(b.ascii({'GND': 'g', '3V3': 'v', 'V5': '5', 'ABS': 'a', 'ABSL': 'A', 'KL15': 'k', 'LED': 'l', 'LEDY': 'y', 'DIN': 'd', 'WPR': 'w', 'WPF': 'W',
                   'CTX': 't', 'CRX': 'r', 'CANH': 'H', 'CANL': 'L', 'KAN': 'n', 'FAN': 'f', 'KAU': 'u', 'S2': '2', 'S2L': '@', 'S1': '1', 'S1L': '!', 'POT': 'p'}))
    print('bad', b.check(order), 'jumpers', b.jumpers)
