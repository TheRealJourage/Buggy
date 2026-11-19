import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import json, heck_left
CARD = {'TX3': 'ser', 'RX3': 'ser', 'D14': 'ser', 'D15': 'ser', 'E5V': 'e5v', '3V3': 'v3', 'GND': 'gnd', 'GNDB': 'gnd',
        'PWM': 'pwm', 'BASE': 'pwm', 'COL': 'pwm', 'KL15P': 'pwm', 'PWMO': 'pwm', 'ZLG': 'zl', 'GATE': 'zl', 'DRN': 'zl', 'SRC': 'zl',
        'CRX': 'can', 'CTX': 'can', 'CANH': 'can', 'CANL': 'can', 'TANK': 'tank', 'TANKL': 'tank', 'BAT': 'bat', 'BATL': 'bat',
        'K15': 'k15', 'K15L': 'k15', 'WB': 'wb', 'WBL': 'wb', 'ZS': 'zs', 'ZSL': 'zs', 'D47': 'd47', 'D47L': 'd47'}
KIND = {'GND': 'g', 'GNDB': 'g', '3V3': 'v', 'E5V': '5', 'KL15P': 'k'}
b, order = heck_left.build()
for n in order:
    b.route(n)
bad = b.check(order)
assert not bad, bad
L = sum(abs(q[0] - p[0]) + abs(q[1] - p[1]) for pa in b.paths for p, q in zip(pa['pts'], pa['pts'][1:]))
out = {'L': dict(cols=[b.c0, b.c1], rows=[b.r0, b.r1], header=b.header,
                 paths=[dict(p, card=CARD[p['net']], kind=KIND.get(p['net'], 's')) for p in b.paths],
                 jumpers=[dict(j, card=CARD[j['net']]) for j in b.jumpers], parts=b.parts, pads=b.pads)}
print(b.name, len(b.paths), 'paths', len(b.jumpers), 'jumpers', len(b.parts), 'parts', len(b.pads), 'pads', 'bridge length', L, 'holes')
json.dump(out, open(sys.argv[1] if len(sys.argv) > 1 else 'heck.json', 'w'), ensure_ascii=False)
