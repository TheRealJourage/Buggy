import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import json, left, right
CARD = {'TXD': 'lin', 'RXD': 'lin', 'CS': 'lin', 'V12': 'lin', 'LBUS': 'lin', 'PU': 'lin', 'E5V': 'e5v', '3V3': 'v3', 'GND': 'gnd', 'GNDT': 'gnd', 'GNDB': 'gnd',
        'BRK': 'brake', 'BRKL': 'brake', 'PMP': 'pump', 'PMPL': 'pump', 'BLK': 'stalk', 'FRN': 'stalk', 'WPR': 'stalk', 'WPF': 'stalk',
        'ABS': 'abs', 'ABSL': 'abs', 'KL15': 'abs', 'LED': 'led', 'LEDY': 'led', 'DIN': 'led', 'V5': 'led',
        'CTX': 'can', 'CRX': 'can', 'CANH': 'can', 'CANL': 'can', 'KAN': 'comp', 'KAU': 'comp', 'FAN': 'fan', 'S1': 'clutch', 'S2': 'clutch',
        'S1L': 'clutch', 'S2L': 'clutch', 'POT': 'poti'}
KIND = {'GND': 'g', 'GNDT': 'g', 'GNDB': 'g', '3V3': 'v', 'E5V': '5', 'V5': '5', 'V12': 'k', 'KL15': 'k'}
out = {}
for mod in (left, right):
    b, order = mod.build()
    for n in order:
        b.route(n)
    bad = b.check(order)
    assert not bad, (b.name, bad)
    L = sum(abs(q[0] - p[0]) + abs(q[1] - p[1]) for pa in b.paths for p, q in zip(pa['pts'], pa['pts'][1:]))
    out[b.name] = dict(cols=[b.c0, b.c1], rows=[b.r0, b.r1], header=b.header,
                       paths=[dict(p, card=CARD[p['net']], kind=KIND.get(p['net'], 's')) for p in b.paths],
                       jumpers=[dict(j, card=CARD[j['net']]) for j in b.jumpers], parts=b.parts, pads=b.pads)
    print(b.name, len(b.paths), 'paths', len(b.jumpers), 'jumpers', len(b.parts), 'parts', len(b.pads), 'pads', 'bridge length', L, 'holes')
json.dump(out, open('boards.json', 'w'), ensure_ascii=False)
