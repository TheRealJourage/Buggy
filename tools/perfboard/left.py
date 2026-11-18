import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from router import Board

def build():
    b = Board('L', (13, 22), (0, 33), 'CN7')
    P = b.pin
    P(1, 'TXD'); P(2, 'RXD'); P(4, 'CS'); P(6, 'E5V'); P(8, 'GNDT'); P(16, '3V3'); P(20, 'GND')
    P(28, 'BLK'); P(30, 'WPF'); P(32, 'FRN'); P(34, 'WPR'); P(36, 'POT'); P(38, 'BRK')
    b.preroute('3V3', [(14, 19), (20, 19)])
    b.preroute('3V3', [(20, 17), (20, 30)])
    b.preroute('GND', [(14, 21), (18, 21), (18, 32)])
    # LIN: MCP2003 horizontal, notch left: bottom row (row 10) pins 1-4, top row (row 7) pins 8-5
    names = ['RXD', 'CS', 'WAKE', 'TXD', 'VSS', 'LBUS', 'VBB', 'VREN']
    nets = ['RXD', 'CS', None, 'TXD', 'GNDT', 'LBUS', 'V12', None]
    holes = [(14, 10), (15, 10), (16, 10), (17, 10), (17, 7), (16, 7), (15, 7), (14, 7)]
    b.part('U1', 'DIP', 'MCP2003 (DIP-8)', [(h, nets[k], str(k + 1)) for k, h in enumerate(holes)], 'lin',
           'Kerbe links; flach direkt eingelötet. Pin 3 WAKE und Pin 8 VREN bleiben frei. RXD-Pull-up = interner Pull-up des STM32', notch='L', names=names)
    b.preroute('TXD', [(13, 12), (13, 9), (17, 9), (17, 10)])
    b.part('C1', 'C', '100 nF', [((15, 5), 'V12', 'a'), ((17, 5), 'GNDT', 'b')], 'lin', 'VBB–VSS dicht am Chip')
    b.part('C2', 'C', '220 pF', [((16, 4), 'LBUS', 'a'), ((18, 4), 'GNDT', 'b')], 'lin', 'nie 100 nF an LBUS')
    b.part('R2', 'R', '1 kΩ', [((16, 3), 'LBUS', 'a'), ((20, 3), 'PU', 'b')], 'lin', 'Master-Pull-up')
    b.part('D1', 'D', '1N4148', [((20, 2), 'PU', 'k'), ((16, 2), 'V12', 'a')], 'lin', 'Ring rechts (zum 1 kΩ)')
    b.pad((14, 5), 'V12', '12 V vom Zündschalter', 'L', 'lin')
    b.pad((14, 3), 'LBUS', 'LIN-Draht zum Lenkrad', 'L', 'lin')
    # 5 V and GND top
    b.part('C3', 'C', '100 nF', [((16, 14), 'E5V', 'a'), ((16, 15), 'GNDT', 'b')], 'e5v', 'dicht am Pin')
    b.part('C4', 'E', '100 µF/25 V (liegend)', [((18, 14), 'E5V', '+'), ((18, 15), 'GNDT', '-')], 'e5v', 'liegend, Körper nach rechts; Plus oben')
    b.pad((17, 13), 'E5V', '+5 V kleine 5-V-Leitung', 'L', 'e5v')
    b.pad((15, 16), 'GNDT', 'GND → Massepunkt', 'L', 'gnd')
    b.part('C5', 'E', '10 µF/25 V (liegend)', [((20, 20), '3V3', '+'), ((18, 22), 'GND', '-')], 'v3', 'liegend')
    b.pad((21, 19), '3V3', '3,3 V → rechte Platine', 'L', 'v3')
    for r, net, lab in ((25, 'BLK', 'Lichthebel weiß-schwarz (Blinker)'), (26, 'WPF', 'Wischerhebel Signal vorn'),
                        (27, 'FRN', 'Lichthebel weiß (Lichthupe/Fernlicht)'), (28, 'WPR', 'Wischerhebel Signal hinten')):
        b.part(f'R_{net}', 'R', '220 Ω', [((16, r), net, 'a'), ((20, r), '3V3', 'b')], 'stalk', 'springt über die GND-Bahn')
        b.pad((15, r), net, lab, 'L', 'stalk')
    b.pad((19, 23), 'GND', 'Lichthebel gelb-braun (Masse)', 'L', 'stalk')
    b.pad((19, 24), 'GND', 'Wischerhebel Masse', 'L', 'stalk')
    b.part('C6', 'C', '100 nF', [((16, 29), 'POT', 'a'), ((18, 29), 'GND', 'b')], 'poti')
    b.pad((15, 29), 'POT', 'Lüfter-Poti Schleifer', 'L', 'poti')
    b.pad((21, 30), '3V3', 'Lüfter-Poti Ende 3,3 V', 'L', 'poti')
    b.pad((19, 31), 'GND', 'Lüfter-Poti Ende GND', 'L', 'poti')
    b.part('D2', 'D', '1N5819', [((20, 30), '3V3', 'k'), ((16, 30), 'BRK', 'a')], 'brake', 'Ring rechts an der 3,3-V-Bahn')
    b.part('C7', 'C', '100 nF', [((16, 31), 'BRK', 'a'), ((18, 31), 'GND', 'b')], 'brake')
    b.part('R3', 'R', '2,7 kΩ', [((14, 32), 'BRK', 'a'), ((18, 32), 'GND', 'b')], 'brake')
    b.part('R4', 'R', '10 kΩ', [((16, 33), 'BRK', 'a'), ((20, 33), 'BRKL', 'b')], 'brake', 'Reihenwiderstand, springt über die GND-Bahn')
    b.pad((21, 33), 'BRKL', 'Bremslicht (nach V7)', 'L', 'brake')
    order = ['TXD', 'RXD', 'CS', 'V12', 'LBUS', 'PU', 'E5V', 'BLK', 'WPF', 'FRN', 'WPR', 'POT', 'BRK', 'BRKL', 'GNDT', '3V3', 'GND']
    return b, order

LEG = {'GNDT': 'G', 'GND': 'g', '3V3': 'v', '3V3T': 'V', 'E5V': '5', 'TXD': 't', 'RXD': 'r', 'CS': 'c', 'V12': 'k', 'LBUS': 'l', 'PU': 'u',
       'BLK': 'x', 'WPF': 'w', 'FRN': 'y', 'WPR': 'W', 'POT': 'o', 'BRK': 'b', 'BRKL': 'B'}
if __name__ == '__main__':
    b, order = build()
    for n in order:
        b.route(n)
    print(b.ascii(LEG)); print('bad', b.check(order), 'jumpers', b.jumpers)
