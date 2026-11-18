from router import Board, hdr

def build():
    b = Board('L', (0, 21), (9, 41), 'CN7')
    P = b.pin
    P(1, 'TXD'); P(2, 'RXD'); P(3, 'CS'); P(6, 'E5V'); P(22, 'GND'); P(16, '3V3')
    P(17, 'BRK'); P(21, 'PMP'); P(35, 'BLK'); P(37, 'FRN')
    # inner channels under the Nucleo (solder side only), staircase so nothing crosses
    b.preroute('CS', [(13, 13), (12, 13), (12, 10), (21, 10), (21, 36), (13, 36), (13, 37)])
    b.preroute('TXD', [(13, 12), (13, 11), (20, 11), (20, 35), (12, 35)])
    b.preroute('RXD', [(14, 12), (19, 12), (19, 34), (10, 34), (10, 38), (12, 38)])
    b.preroute('E5V', [(14, 14), (18, 14), (18, 33), (4, 33)])
    b.preroute('3V3', [(14, 19), (17, 19), (17, 32), (3, 32), (3, 12)])
    b.preroute('GND', [(14, 22), (16, 22), (16, 31), (5, 31), (5, 12)])
    # LIN: MCP2003 DIP-8, notch at the bottom; MCU side right (col 12), bus side left (col 9)
    b.part('U1', 'DIP', 'MCP2003 (DIP-8, Sockel)', [((12, 38), 'RXD', '1'), ((12, 37), 'CS', '2'), ((12, 36), None, '3'), ((12, 35), 'TXD', '4'),
            ((9, 35), 'GND', '5'), ((9, 36), 'LBUS', '6'), ((9, 37), 'V12', '7'), ((9, 38), None, '8')], 'lin',
           'Kerbe unten (Pin 1 unten rechts). Pin 3 WAKE und Pin 8 VREN bleiben frei', notch='B', names=['RXD', 'CS', 'WAKE', 'TXD', 'VSS', 'LBUS', 'VBB', 'VREN'])
    b.part('R1', 'R', '5,1 kΩ', [((11, 32), '3V3', 'a'), ((11, 34), 'RXD', 'b')], 'lin', 'stehend/kurz: RXD-Pull-up, springt über die 5-V-Bahn')
    b.part('C1', 'C', '100 nF', [((8, 37), 'V12', 'a'), ((8, 35), 'GND', 'b')], 'lin', 'senkrecht VBB–VSS, dicht am Chip')
    b.part('C2', 'C', '220 pF', [((7, 36), 'LBUS', 'a'), ((7, 35), 'GND', 'b')], 'lin', 'nie 100 nF an LBUS')
    b.part('R2', 'R', '1 kΩ', [((4, 36), 'LBUS', 'a'), ((4, 40), 'PU', 'b')], 'lin', 'senkrecht, Master-Pull-up')
    b.part('D1', 'D', '1N4148', [((5, 40), 'PU', 'k'), ((9, 40), 'V12', 'a')], 'lin', 'Ring links (zum 1 kΩ)')
    b.pad((5, 36), 'LBUS', 'LIN-Draht zum Lenkrad', 'L', 'lin')
    b.pad((6, 37), 'V12', '12 V vom Zündschalter', 'L', 'lin')
    # E5V
    b.part('C3', 'E', '100 µF/25 V', [((6, 33), 'E5V', '+'), ((6, 31), 'GND', '-')], 'e5v', 'Plus unten (5-V-Bahn), Minus oben (GND), springt über die 3,3-V-Bahn')
    b.part('C4', 'C', '100 nF', [((9, 33), 'E5V', 'a'), ((9, 31), 'GND', 'b')], 'e5v', 'springt über die 3,3-V-Bahn')
    b.pad((4, 33), 'E5V', '+5 V kleine 5-V-Leitung', 'L', 'e5v')
    b.part('C5', 'E', '10 µF/25 V', [((3, 21), '3V3', '+'), ((5, 21), 'GND', '-')], 'v3', 'zwischen den Schienen: Plus links (3,3 V), Minus rechts (GND)')
    # brake (divider + clamp), node col 7
    b.part('D2', 'D', '1N5819', [((3, 16), '3V3', 'k'), ((7, 16), 'BRK', 'a')], 'brake', 'Ring links an der 3,3-V-Schiene')
    b.part('R3', 'R', '2,7 kΩ', [((9, 17), 'BRK', 'a'), ((5, 17), 'GND', 'b')], 'brake')
    b.part('C6', 'C', '100 nF', [((7, 18), 'BRK', 'a'), ((5, 18), 'GND', 'b')], 'brake')
    b.part('R4', 'R', '10 kΩ', [((2, 19), 'BRKL', 'a'), ((6, 19), 'BRK', 'b')], 'brake', 'springt über beide Schienen')
    b.pad((0, 19), 'BRKL', 'Bremslicht (nach V7)', 'L', 'brake')
    # pump (diode input), node col 7
    b.part('R5', 'R', '10 kΩ', [((3, 23), '3V3', 'a'), ((7, 23), 'PMP', 'b')], 'pump', 'Pull-up, springt über die GND-Schiene')
    b.part('C7', 'C', '100 nF', [((7, 24), 'PMP', 'a'), ((5, 24), 'GND', 'b')], 'pump')
    b.part('D3', 'D', '1N5819', [((2, 25), 'PMPL', 'k'), ((6, 25), 'PMP', 'a')], 'pump', 'Ring links (zur Leitung), springt über beide Schienen')
    b.pad((0, 25), 'PMPL', 'K3/87a Pumpe-Status', 'L', 'pump')
    # light stalk
    b.part('R6', 'R', '220 Ω', [((3, 29), '3V3', 'a'), ((7, 29), 'BLK', 'b')], 'stalk', 'springt über die GND-Schiene')
    b.part('R7', 'R', '220 Ω', [((3, 30), '3V3', 'a'), ((7, 30), 'FRN', 'b')], 'stalk', 'springt über die GND-Schiene')
    b.pad((9, 29), 'BLK', 'Lichthebel weiß-schwarz (Blinker)', 'L', 'stalk')
    b.pad((10, 30), 'FRN', 'Lichthebel weiß (Lichthupe/Fernlicht)', 'L', 'stalk')
    b.pad((5, 27), 'GND', 'Lichthebel gelb-braun (Masse)', 'L', 'stalk')
    b.pad((3, 14), '3V3', '3,3 V → rechte Platine', 'L', 'v3')
    b.pad((5, 12), 'GND', 'GND → Massepunkt', 'L', 'gnd')
    b.occ[(7, 34)] = '_keepout'  # next to C3: no jumper end here
    order = ['BRK', 'PMP', 'BLK', 'FRN', 'BRKL', 'PMPL', 'TXD', 'CS', 'RXD', 'E5V', 'V12', 'LBUS', 'PU', '3V3', 'GND']
    return b, order

if __name__ == '__main__':
    b, order = build()
    for n in order:
        b.route(n)
    print(b.ascii({'GND': 'g', '3V3': 'v', 'E5V': '5', 'TXD': 't', 'RXD': 'r', 'CS': 'c', 'V12': 'k', 'LBUS': 'l', 'PU': 'u',
                   'BRK': 'b', 'BRKL': 'B', 'PMP': 'p', 'PMPL': 'P', 'BLK': 'x', 'FRN': 'y'}))
    print('bad', b.check(order), 'jumpers', b.jumpers)
