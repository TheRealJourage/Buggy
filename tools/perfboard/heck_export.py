import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import json, heck_left
CARD = {'TX3': 'ser', 'RX3': 'ser', 'D14': 'ser', 'D15': 'ser', 'E5V': 'e5v', '3V3': 'v3', 'GND': 'gnd', 'GNDB': 'gnd',
        'PWM': 'pwm', 'BASE': 'pwm', 'COL': 'pwm', 'KL15P': 'pwm', 'PWMO': 'pwm', 'ZLG': 'zl', 'GATE': 'zl', 'DRN': 'zl', 'SRC': 'zl',
        'CRX': 'can', 'CTX': 'can', 'CANH': 'can', 'CANL': 'can', 'TANK': 'tank', 'TANKL': 'tank', 'BAT': 'bat', 'BATL': 'bat',
        'K15': 'k15', 'K15L': 'k15', 'WB': 'wb', 'WBL': 'wb', 'ZS': 'zs', 'ZSL': 'zs', 'D47': 'd47', 'D47L': 'd47'}
KIND = {'GND': 'g', 'GNDB': 'g', '3V3': 'v', 'E5V': '5', 'KL15P': 'k'}
def export(mod, dc=0, dr=0):
    b, order = mod.build()
    for n in order:
        b.route(n)
    bad = b.check(order)
    assert not bad, (b.name, bad)
    L = sum(abs(q[0] - p[0]) + abs(q[1] - p[1]) for pa in b.paths for p, q in zip(pa['pts'], pa['pts'][1:]))
    print(b.name, len(b.paths), 'paths', len(b.jumpers), 'jumpers', len(b.parts), 'parts', len(b.pads), 'pads', 'bridge length', L, 'holes')
    mv = lambda h: [h[0] + dc, h[1] + dr]
    return dict(cols=[b.c0 + dc, b.c1 + dc], rows=[b.r0 + dr, b.r1 + dr], header=b.header if mod is heck_left else None,
                paths=[dict(p, pts=[mv(q) for q in p['pts']], card=CARD[p['net']], kind=KIND.get(p['net'], 's')) for p in b.paths],
                jumpers=[dict(j, a=mv(j['a']), b=mv(j['b']), card=CARD[j['net']]) for j in b.jumpers],
                parts=[dict(p, leads=[mv(l[:2]) + l[2:] for l in p['leads']]) for p in b.parts],
                pads=[dict(p, at=mv(p['at'])) for p in b.pads])


# CN7 plug-on board on a 6 x 8 cm perfboard (22 x 28 holes), all cables soldered to pads
out = {'L': export(heck_left)}
json.dump(out, open(sys.argv[1] if len(sys.argv) > 1 else 'heck.json', 'w'), ensure_ascii=False)
