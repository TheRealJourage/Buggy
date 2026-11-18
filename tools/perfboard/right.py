import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from router import Board
from left import jst

def build():
    b = Board('R', (23, 43), (-4, 44), 'CN10')
    P = b.pin
    P(1, 'ABS'); P(2, 'LED'); P(3, 'CRX'); P(5, 'CTX'); P(9, 'GND')
    P(13, 'POT'); P(15, 'BRK'); P(17, 'KAN'); P(19, 'KAU'); P(21, 'FAN'); P(23, 'PMP'); P(25, 'S2'); P(27, 'S1')
    # ---- CAN module flat (pins col 36 rows 11-16), GND from CN10-9 ----
    b.part('U2', 'CANM', 'SN65HVD230-Modul (flach)', [((36, 11), 'CANL', 'CANL'), ((36, 12), 'CANH', 'CANH'), ((36, 13), 'CRX', 'CRX'),
           ((36, 14), 'CTX', 'CTX'), ((36, 15), 'GND', 'GND'), ((36, 16), '3V3', '3V3')], 'can',
           'flach aufgesteckt, Platine nach innen (über Spalte 30–36); Pinfolge am eigenen Modul prüfen')
    b.preroute('GND', [(38, 16), (37, 16), (37, 15), (36, 15)])
    # ---- top: LED driver + ABS as before, connectors above the Nucleo edge ----
    b.preroute('ABS', [(38, 12), (38, 8)])
    b.preroute('LED', [(39, 12), (39, 2), (33, 2), (33, 3)])
    names = ['1OE', '1A', '1Y', '2OE', '2A', '2Y', 'GND', '3Y', '3A', '3OE', '4Y', '4A', '4OE', 'VCC']
    nets = ['GND', 'LED', 'LEDY', 'GND', 'GND', 'GND', 'GND', 'V5', 'V5', 'V5', 'V5', 'V5', 'V5', 'V5']
    holes = [(32 + k, 3) for k in range(7)] + [(38 - k, 0) for k in range(7)]
    b.part('U1', 'DIP', '74AHCT125 (DIP-14)', [(h, nets[k], str(k + 1)) for k, h in enumerate(holes)], 'led',
           'Kerbe links; flach direkt eingelötet; Pinfolge im Datenblatt prüfen. Gatter 2 an Masse, Gatter 3/4 an 5 V', notch='L', names=names)
    b.part('C2', 'C', '100 nF', [((31, 0), 'V5', 'a'), ((31, 3), 'GND', 'b')], 'led', '5 V–GND dicht am Chip')
    b.part('R4', 'R', '330 Ω', [((34, 4), 'LEDY', 'a'), ((30, 4), 'DIN', 'b')], 'led')
    b.part('C1', 'C', '100 nF', [((37, 6), 'ABS', 'a'), ((35, 6), 'GND', 'b')], 'abs')
    b.part('R2', 'R', '10 kΩ', [((37, 7), 'ABS', 'a'), ((33, 7), 'ABSL', 'b')], 'abs', 'Reihenwiderstand')
    b.part('D1', 'D', '1N5819', [((32, 8), '3V3', 'k'), ((36, 8), 'ABS', 'a')], 'abs', 'Ring links an 3,3 V')
    b.part('R1', 'R', '2,7 kΩ', [((37, 9), 'ABS', 'a'), ((33, 9), 'GND', 'b')], 'abs')
    b.part('R3', 'R', '1 kΩ 0,5 W', [((32, 6), 'ABSL', 'a'), ((28, 6), 'KL15', 'b')], 'abs', '0,5-W-Typ (0,21 W gerechnet)')
    jst(b, 'J5', 'LED-Leiste', [('DIN', 'DIN'), ('GND', 'GND'), ('V5', '+5 V groß')], -2, 30, 'led')
    jst(b, 'J6', 'ABS-Lampe / Kl. 15', [('KL15', 'Kl. 15 (V4)'), ('ABSL', 'J104 Pin 16')], -2, 27, 'abs')
    # ---- lower group: staircase, lowest pin turns first (closest to the header) ----
    b.preroute('S1', [(38, 25), (37, 25), (37, 31)])
    b.preroute('S2', [(38, 24), (36, 24), (36, 31)])
    b.preroute('PMP', [(38, 23), (35, 23), (35, 31)])
    b.preroute('FAN', [(38, 22), (34, 22), (34, 31)])
    b.preroute('KAU', [(38, 21), (33, 21), (33, 31)])
    b.preroute('KAN', [(38, 20), (32, 20), (32, 31)])
    b.preroute('BRK', [(38, 19), (31, 19), (31, 26)])
    b.preroute('POT', [(38, 18), (30, 18), (30, 31)])
    # pull-ups: bussed 6 x 10 kOhm network below the header end, common pin in col 38
    b.part('RN1', 'SIP', '6 × 10 kΩ (SIP-7, gemeinsamer Pin)', [((38, 32), '3V3', '1'), ((37, 32), 'S1', '2'), ((36, 32), 'S2', '3'),
           ((35, 32), 'PMP', '4'), ((34, 32), 'FAN', '5'), ((33, 32), 'KAU', '6'), ((32, 32), 'KAN', '7')], 'comp',
           'gemeinsamer Pin 1 (Punkt) rechts an 3,3 V', names=['3V3', 'S1', 'S2', 'Pu', 'Lü', 'Au', 'An'])
    # series diodes (pump, clutch) in the target columns of connector J9, staggered, ring towards the connector
    b.part('D_PMP', 'D', '1N5819', [((40, 41), 'PMPL', 'k'), ((40, 37), 'PMP', 'a')], 'pump', 'senkrecht, Ring unten (zur Leitung)')
    b.part('D_S2', 'D', '1N5819', [((41, 40), 'S2L', 'k'), ((41, 36), 'S2', 'a')], 'clutch', 'senkrecht, Ring unten (zur Leitung)')
    b.part('D_S1', 'D', '1N5819', [((42, 41), 'S1L', 'k'), ((42, 37), 'S1', 'a')], 'clutch', 'senkrecht, Ring unten (zur Leitung)')
    # brake divider (left of the poti line), poti filter
    b.part('R5', 'R', '10 kΩ', [((31, 27), 'BRK', 'a'), ((31, 31), 'BRKL', 'b')], 'brake', 'senkrecht in der Bahn (Reihenwiderstand)')
    b.part('R6', 'R', '2,7 kΩ', [((31, 26), 'BRK', 'a'), ((27, 26), 'GNDB', 'b')], 'brake', 'springt über die Poti-Bahn')
    b.part('C3', 'C', '100 nF', [((31, 25), 'BRK', 'a'), ((29, 25), 'GNDB', 'b')], 'brake', 'springt über die Poti-Bahn')
    b.part('D2', 'D', '1N5819', [((27, 24), '3V3', 'k'), ((31, 24), 'BRK', 'a')], 'brake', 'Ring links an 3,3 V, springt über die Poti-Bahn')
    b.part('C4', 'C', '100 nF', [((30, 33), 'POT', 'a'), ((28, 33), 'GNDB', 'b')], 'poti')
    # CAN to the top-left connector, under the module
    b.preroute('CANL', [(36, 11), (35, 11), (25, 11), (25, -2)])
    b.preroute('CANH', [(36, 12), (35, 12), (24, 12), (24, -2)])
    jst(b, 'J7', 'CAN Front', [('CANH', 'CAN-H'), ('CANL', 'CAN-L')], -2, 24, 'can')
    # 3V3 cable connector beside the header (outside the Nucleo); 3V3 up the right edge, along the top, down the left edge
    jst(b, 'J8', '3,3 V von links', [('3V3', '3,3 V'), ('GNDB', 'GND')], 25, 42, 'v3')
    b.preroute('3V3', [(38, 32), (41, 32), (41, -4), (23, -4), (23, 41)])
    b.preroute('3V3', [(41, 25), (42, 25)])
    b.preroute('3V3', [(23, 16), (35, 16), (36, 16)])
    b.preroute('GNDB', [(43, 25), (43, 44), (26, 44)])
    # bottom connectors (row 42), one free hole between housings
    jst(b, 'J11', 'Lüfter-Poti', [('3V3', '3,3 V'), ('GNDB', 'GND'), ('POT', 'Schleifer')], 42, 28, 'poti')
    jst(b, 'J12', 'Bremslicht', [('BRKL', 'Bremslicht'), ('GNDB', 'GND')], 42, 32, 'brake')
    jst(b, 'J10', 'Panel Motor (Schalter)', [('KAN', 'Kompr. AN'), ('KAU', 'Kompr. AUTO'), ('FAN', 'Lüfter A/M'), ('GNDB', 'GND')], 42, 35, 'comp')
    jst(b, 'J9', 'Fußraum (Pumpe, Kupplung)', [('PMPL', 'Pumpe K3/87a'), ('S2L', 'Kupplung S2'), ('S1L', 'Kupplung S1'), ('GNDB', 'GND')], 42, 40, 'clutch')
    order = ['ABS', 'ABSL', 'KL15', 'LED', 'LEDY', 'DIN', 'V5', 'CRX', 'CTX', 'CANH', 'CANL', 'KAN', 'KAU', 'FAN', 'PMP', 'PMPL', 'S2', 'S2L', 'S1', 'S1L',
             'BRK', 'BRKL', 'POT', '3V3', 'GND', 'GNDB']
    return b, order

LEG = {'GNDB': 'G', 'GND': 'g', '3V3': 'v', 'V5': '5', 'ABS': 'a', 'ABSL': 'A', 'KL15': 'k', 'LED': 'l', 'LEDY': 'y', 'DIN': 'd', 'CTX': 't', 'CRX': 'r',
       'CANH': 'H', 'CANL': 'L', 'KAN': 'n', 'KAU': 'u', 'FAN': 'f', 'PMP': 'p', 'PMPL': 'P', 'S2': '2', 'S2L': '@', 'S1': '1', 'S1L': '!', 'BRK': 'b', 'BRKL': 'B', 'POT': 'o'}
