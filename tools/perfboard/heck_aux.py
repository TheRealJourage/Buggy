"""Relay box: small aux board (fan PWM stage + ZL180 low-side driver), cable J10 from the CN7 plug-on board (J9).
Outside the Nucleo, height free. Fits a piece of perfboard 14 x 20 holes."""
import sys, os; sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from router import Board
from heck_left import jst


def build():
    b = Board('A', (0, 13), (0, 17), 'CN10')   # header CN10 lies outside this board (cols 38/39): no header here
    jst(b, 'J10', 'Kabel von J9 (Steckplatine)', [('PWM', 'PA15 Lüfter-PWM'), ('GNDB', 'GND'), ('ZLG', 'PB7 ZL180')], (0, 8), 'pwm', vertical=True)
    # fan PWM: BC547B pulls the J293 line low, pull-up 2 x 2 kOhm to Kl. 15, 100 Ohm + TVS at the line
    b.part('R1', 'R', '1 kΩ', [((1, 8), 'PWM', 'a'), ((5, 8), 'BASE', 'b')], 'pwm', 'Basiswiderstand')
    b.part('R2', 'R', '10 kΩ', [((5, 9), 'BASE', 'a'), ((1, 9), 'GNDB', 'b')], 'pwm', 'Basis nach GND')
    b.part('Q1', 'TO92', 'BC547B', [((6, 6), 'GNDB', 'E'), ((7, 6), 'BASE', 'B'), ((8, 6), 'COL', 'C')], 'pwm',
           'stehend, flache Seite nach unten; C-B-E vor dem Einlöten messen', names=['E', 'B', 'C'])
    b.part('R3', 'R', '2 kΩ', [((8, 4), 'COL', 'a'), ((12, 4), 'KL15P', 'b')], 'pwm', 'Pull-up an Kl. 15, mit R4 parallel = 1 kΩ')
    b.part('R4', 'R', '2 kΩ', [((8, 3), 'COL', 'a'), ((12, 3), 'KL15P', 'b')], 'pwm', 'parallel zu R3')
    b.part('R5', 'R', '100 Ω', [((8, 2), 'COL', 'a'), ((12, 2), 'PWMO', 'b')], 'pwm', 'Reihenwiderstand zur Leitung')
    b.part('D1', 'TVS', '1.5KE24A', [((13, 1), 'PWMO', 'k'), ((13, 7), 'GNDB', 'a')], 'pwm', 'liegend senkrecht, Ring oben (Leitung); Löcher ggf. auf 1,3 mm aufbohren')
    jst(b, 'J1', 'Lüfter-PWM (J293)', [('KL15P', 'Kl. 15 (12 V)'), ('PWMO', 'PWM → J293')], (11, 0), 'pwm')
    # ZL180 coil low side: IRLZ44N standing, source on its own wire to the ground point
    b.part('R6', 'R', '1 kΩ', [((1, 10), 'ZLG', 'a'), ((5, 10), 'GATE', 'b')], 'zl', 'Gate-Widerstand')
    b.part('R7', 'R', '10 kΩ', [((5, 11), 'GATE', 'a'), ((9, 11), 'SRC', 'b')], 'zl', 'Gate nach Source')
    b.part('Q2', 'TO220', 'IRLZ44N', [((5, 13), 'GATE', 'G'), ((6, 13), 'DRN', 'D'), ((7, 13), 'SRC', 'S')], 'zl',
           'stehend, Metallfahne nach hinten (Reihe 12); G-D-S von vorn = links → rechts', names=['G', 'D', 'S'])
    jst(b, 'J2', 'ZL180', [('DRN', 'ZL180-Spule (−)'), ('SRC', 'Source → Massepunkt')], (6, 16), 'zl')
    order = ['PWM', 'BASE', 'COL', 'KL15P', 'PWMO', 'ZLG', 'GATE', 'DRN', 'SRC', 'GNDB']
    return b, order


LEG = {'PWM': 'p', 'GNDB': 'G', 'ZLG': 'z', 'BASE': 'b', 'COL': 'c', 'KL15P': 'k', 'PWMO': 'o', 'GATE': 'q', 'DRN': 'd', 'SRC': 's'}

if __name__ == '__main__':
    b, order = build()
    for n in order:
        try:
            b.route(n)
        except RuntimeError as e:
            print('FAIL', e)
    print(b.ascii(LEG))
    print('jumpers', b.jumpers, 'bad', b.check(order))
