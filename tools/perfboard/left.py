import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from router import Board

def jst(b, ref, title, pins, row, col0, card):
    b.part(ref, 'JST', f'JST XH {len(pins)}-polig', [((col0 + k, row), net, str(k + 1)) for k, (net, lab) in enumerate(pins)], card,
           title, names=[lab for net, lab in pins], title=title)

def build():
    b = Board('L', (13, 21), (-3, 38), 'CN7')
    P = b.pin
    P(1, 'TXD'); P(2, 'RXD'); P(4, 'CS'); P(6, 'E5V'); P(8, 'GND'); P(16, '3V3')
    P(28, 'BLK'); P(30, 'WPF'); P(32, 'FRN'); P(34, 'WPR')
    # ---- top: LIN + 5 V; connectors above the Nucleo edge ----
    names = ['RXD', 'CS', 'WAKE', 'TXD', 'VSS', 'LBUS', 'VBB', 'VREN']
    nets = ['RXD', 'CS', None, 'TXD', 'GND', 'LBUS', 'V12', None]
    holes = [(14, 10), (15, 10), (16, 10), (17, 10), (17, 7), (16, 7), (15, 7), (14, 7)]
    b.part('U1', 'DIP', 'MCP2003 (DIP-8)', [(h, nets[k], str(k + 1)) for k, h in enumerate(holes)], 'lin',
           'Kerbe links; flach direkt eingelötet. Pin 3 WAKE und Pin 8 VREN bleiben frei. RXD-Pull-up = interner Pull-up des STM32', notch='L', names=names)
    b.preroute('TXD', [(13, 12), (13, 9), (17, 9), (17, 10)])
    b.part('C1', 'C', '100 nF', [((15, 5), 'V12', 'a'), ((17, 5), 'GND', 'b')], 'lin', 'VBB–VSS dicht am Chip')
    b.part('C2', 'C', '220 pF', [((16, 4), 'LBUS', 'a'), ((18, 4), 'GND', 'b')], 'lin', 'nie 100 nF an LBUS')
    b.part('R1', 'R', '1 kΩ', [((16, 3), 'LBUS', 'a'), ((20, 3), 'PU', 'b')], 'lin', 'Master-Pull-up')
    b.part('D1', 'D', '1N4148', [((20, 2), 'PU', 'k'), ((16, 2), 'V12', 'a')], 'lin', 'Ring rechts (zum 1 kΩ)')
    jst(b, 'J1', 'Lenkrad / LIN', [('V12', '12 V Zündschalter'), ('LBUS', 'LIN'), ('GND', 'GND')], -2, 14, 'lin')
    jst(b, 'J2', '5 V (kleine Leitung)', [('GND', 'GND'), ('E5V', '+5 V')], -2, 19, 'e5v')
    b.part('C3', 'C', '100 nF', [((16, 14), 'E5V', 'a'), ((16, 15), 'GND', 'b')], 'e5v', 'dicht am Pin')
    b.part('C4', 'E', '100 µF/25 V (liegend)', [((18, 14), 'E5V', '+'), ((18, 15), 'GND', '-')], 'e5v', 'liegend, Körper nach rechts; Plus oben')
    # ---- power: GND from CN7-8 down the right edge (col 21) + bus under the bottom connectors; 3V3 inside (col 20) ----
    b.preroute('GND', [(14, 15), (21, 15), (21, 38), (14, 38)])
    b.preroute('3V3', [(14, 19), (20, 19), (20, 36)])
    b.part('C5', 'E', '10 µF/25 V (liegend)', [((20, 21), '3V3', '+'), ((21, 21), 'GND', '-')], 'v3', 'liegend, Körper nach links')
    # ---- bottom: staircase to the stalk connector, highest pin turns furthest right ----
    b.preroute('BLK', [(14, 25), (18, 25), (18, 36)])
    b.preroute('WPF', [(14, 26), (17, 26), (17, 36)])
    b.preroute('FRN', [(14, 27), (16, 27), (16, 36)])
    b.preroute('WPR', [(14, 28), (15, 28), (15, 36)])
    b.part('RN1', 'SIP', '4 × 220 Ω (SIP-5, gemeinsamer Pin)', [((19, 31), '3V3', '1'), ((18, 31), 'BLK', '2'), ((17, 31), 'WPF', '3'),
           ((16, 31), 'FRN', '4'), ((15, 31), 'WPR', '5')], 'stalk', 'gemeinsamer Pin 1 (Punkt) rechts an 3,3 V', names=['3V3', 'Bl', 'Wv', 'Fl', 'Wh'])
    jst(b, 'J3', 'Lenkstockhebel (Licht + Wischer)', [('GND', 'GND'), ('WPR', 'Wischer hinten'), ('FRN', 'Fernlicht'), ('WPF', 'Wischer vorn'), ('BLK', 'Blinker')], 36, 14, 'stalk')
    jst(b, 'J4', '3,3 V → rechte Platine', [('3V3', '3,3 V'), ('GND', 'GND')], 36, 20, 'v3')
    order = ['TXD', 'RXD', 'CS', 'V12', 'LBUS', 'PU', 'E5V', 'BLK', 'WPF', 'FRN', 'WPR', '3V3', 'GND']
    return b, order

LEG = {'GND': 'g', '3V3': 'v', 'E5V': '5', 'TXD': 't', 'RXD': 'r', 'CS': 'c', 'V12': 'k', 'LBUS': 'l', 'PU': 'u',
       'BLK': 'x', 'WPF': 'w', 'FRN': 'y', 'WPR': 'W'}
