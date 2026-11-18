import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from router import Board

def build():
    b = Board('R', (27, 39), (0, 29), 'CN10')
    P = b.pin
    P(1, 'ABS'); P(2, 'LED'); P(3, 'CRX'); P(5, 'CTX'); P(9, 'GND')
    P(13, 'KAN'); P(17, 'KAU'); P(21, 'FAN'); P(25, 'PMP'); P(29, 'S2'); P(33, 'S1')
    # CAN module lying flat, pins in col 36 rows 11-16 (CANL top ... 3V3 bottom), body inward over cols 30-36
    b.part('U2', 'CANM', 'SN65HVD230-Modul (flach)', [((36, 11), 'CANL', 'CANL'), ((36, 12), 'CANH', 'CANH'), ((36, 13), 'CRX', 'CRX'),
           ((36, 14), 'CTX', 'CTX'), ((36, 15), 'GND', 'GND'), ((36, 16), '3V3', '3V3')], 'can',
           'flach aufgesteckt, Platine nach innen (über Spalte 30–36); Pinfolge am eigenen Modul prüfen')
    b.pad((37, 11), 'CANL', 'CAN-L → Stecker CAN Front', 'R', 'can')
    b.pad((37, 12), 'CANH', 'CAN-H → Stecker CAN Front', 'R', 'can')
    b.preroute('GND', [(38, 16), (37, 16), (37, 15), (36, 15)])
    b.preroute('3V3', [(36, 16), (30, 16), (30, 28)])
    b.pad((31, 17), '3V3', '3,3 V von der linken Platine', 'R', 'v3')
    # simple pull-up inputs (row r: R to the 3V3 trunk in col 30, pad on the trace)
    for r, net, lab, card in ((18, 'KAN', 'Kompressor AN', 'comp'), (20, 'KAU', 'Kompressor AUTO', 'comp'), (22, 'FAN', 'Lüfter Auto/manuell', 'fan')):
        b.part(f'R_{net}', 'R', '10 kΩ', [((34, r), net, 'a'), ((30, r), '3V3', 'b')], card, 'Pull-up in die 3,3-V-Bahn')
        b.pad((36, r), net, lab, 'R', card)
    # diode inputs: R on row r, Schottky on row r+1 (ring towards the line pad)
    for r, net, lab, card in ((24, 'PMP', 'K3/87a Pumpe-Status', 'pump'), (26, 'S2', 'Kupplung S2', 'clutch'), (28, 'S1', 'Kupplung S1', 'clutch')):
        b.part(f'R_{net}', 'R', '10 kΩ', [((34, r), net, 'a'), ((30, r), '3V3', 'b')], card, 'Pull-up in die 3,3-V-Bahn')
        b.part(f'D_{net}', 'D', '1N5819', [((33, r + 1), net + 'L', 'k'), ((37, r + 1), net, 'a')], card, 'Ring links (zur Leitung)')
        b.pad((32, r + 1), net + 'L', lab, 'R', card)
    b.pad((37, 15), 'GND', 'Kompressor Mittelkontakt (Masse)', 'R', 'comp')
    # top area: LED driver 74AHCT125 (rows 0/3) and ABS (rows 5-10); ABS leaves PC9 upwards in col 38, LED (PC8, outer) in col 39
    b.preroute('ABS', [(38, 12), (38, 8)])
    b.preroute('LED', [(39, 12), (39, 2), (33, 2), (33, 3)])
    names = ['1OE', '1A', '1Y', '2OE', '2A', '2Y', 'GND', '3Y', '3A', '3OE', '4Y', '4A', '4OE', 'VCC']
    nets = ['GND', 'LED', 'LEDY', 'GND', 'GND', 'GND', 'GND', 'V5', 'V5', 'V5', 'V5', 'V5', 'V5', 'V5']
    holes = [(32 + k, 3) for k in range(7)] + [(38 - k, 0) for k in range(7)]
    b.part('U1', 'DIP', '74AHCT125 (DIP-14)', [(h, nets[k], str(k + 1)) for k, h in enumerate(holes)], 'led',
           'Kerbe links; flach direkt eingelötet; Pinfolge im Datenblatt prüfen. Gatter 2 an Masse (2Y mit an Masse), Gatter 3/4 an 5 V (Ausgänge hochohmig, mit an 5 V)', notch='L', names=names)
    b.part('C2', 'C', '100 nF', [((31, 0), 'V5', 'a'), ((31, 3), 'GND', 'b')], 'led', '5 V–GND dicht am Chip')
    b.pad((30, 0), 'V5', '+5 V große 5-V-Schiene', 'R', 'led')
    b.part('R4', 'R', '330 Ω', [((34, 4), 'LEDY', 'a'), ((30, 4), 'DIN', 'b')], 'led')
    b.pad((29, 4), 'DIN', 'DIN LED-Leiste', 'R', 'led')
    b.part('C1', 'C', '100 nF', [((37, 6), 'ABS', 'a'), ((35, 6), 'GND', 'b')], 'abs')
    b.part('R2', 'R', '10 kΩ', [((37, 7), 'ABS', 'a'), ((33, 7), 'ABSL', 'b')], 'abs', 'Reihenwiderstand')
    b.part('D1', 'D', '1N5819', [((32, 8), '3V3', 'k'), ((36, 8), 'ABS', 'a')], 'abs', 'Ring links an 3,3 V')
    b.part('R1', 'R', '2,7 kΩ', [((37, 9), 'ABS', 'a'), ((33, 9), 'GND', 'b')], 'abs')
    b.part('R3', 'R', '1 kΩ 0,5 W', [((32, 6), 'ABSL', 'a'), ((28, 6), 'KL15', 'b')], 'abs', '0,5-W-Typ (0,21 W gerechnet)')
    b.pad((32, 7), 'ABSL', 'J104 Pin 16 (ABS-Lampe)', 'R', 'abs')
    b.pad((27, 6), 'KL15', 'Kl. 15 (V4)', 'R', 'abs')
    order = ['ABS', 'ABSL', 'KL15', 'LED', 'LEDY', 'DIN', 'V5', 'CRX', 'CTX', 'CANH', 'CANL', 'KAN', 'KAU', 'FAN', 'PMP', 'PMPL', 'S2', 'S2L', 'S1', 'S1L', '3V3', 'GND']
    return b, order

LEG = {'GND': 'g', '3V3': 'v', 'V5': '5', 'ABS': 'a', 'ABSL': 'A', 'KL15': 'k', 'LED': 'l', 'LEDY': 'y', 'DIN': 'd', 'CTX': 't', 'CRX': 'r',
       'CANH': 'H', 'CANL': 'L', 'KAN': 'n', 'KAU': 'u', 'FAN': 'f', 'PMP': 'p', 'PMPL': 'P', 'S2': '2', 'S2L': '@', 'S1': '1', 'S1L': '!'}
if __name__ == '__main__':
    b, order = build()
    for n in order:
        b.route(n)
    print(b.ascii(LEG)); print('bad', b.check(order), 'jumpers', b.jumpers)
