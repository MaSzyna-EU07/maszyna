"""Writes old.scn, the old scenery of the takeover tests (test_takeover_scenery.cpp): tracks as an old scenery has them, made by hand of
Bezier curves with no design behind them, and the lineside objects along them.

  line a  a straight of 200 m, a curve to the left of R 500 m with clothoids of 60 m and 80 mm of cant turning by 30 degree, a straight on;
          level for 300 m, then rising at 5 per mille with no vertical curve; cut into tracks 20 to 45 m long, a few of no name, a few
          written against the line, one calling an event of a signal, three in an isolated circuit
  zwr_1   a switch at the end of line a, its diverging track to the left (an old one, of no type the catalogue has)
  line b  on from the main track of the switch, straight; line c on from its diverging track, straight
  along a: a signal named by the events of a track and its memory cell, catenary poles every 50 m with the wires between them, hectometre
  posts; a tree 25 m off; a cabinet between b and c

Run with python 3, no arguments; the file is written next to this one. The coordinates are written to the millimetre and the rolls to
the tenth of a degree, as old sceneries have them.
"""
import math
import os
import random

STEP = 0.01
R = 500.0
L = 60.0
TURN = math.radians(30.0)
CANT = 80.0  # mm
ARC = R * TURN - L  # the arc between the clothoids: the turn of the clothoids is L / R for both
STRAIGHT_IN = 200.0
STRAIGHT_OUT = 220.0
LENGTH = STRAIGHT_IN + 2 * L + ARC + STRAIGHT_OUT


def curvature(s):
    s1 = STRAIGHT_IN
    s2 = s1 + L
    s3 = s2 + ARC
    s4 = s3 + L
    if s < s1 or s >= s4:
        return 0.0
    if s < s2:
        return (s - s1) / L / R
    if s < s3:
        return 1.0 / R
    return (s4 - s) / L / R


def cant(s):
    s1 = STRAIGHT_IN
    s2 = s1 + L
    s3 = s2 + ARC
    s4 = s3 + L
    if s < s1 or s >= s4:
        return 0.0
    if s < s2:
        return CANT * (s - s1) / L
    if s < s3:
        return CANT
    return CANT * (s4 - s) / L


def height(s):
    return 100.0 if s < 300.0 else 100.0 + 0.005 * (s - 300.0)


def grade(s):
    return 0.0 if s < 300.0 else 0.005


# the plan of line a by its curvature, integrated finely: x, z and the heading at each step
def integrate():
    points = [(0.0, 0.0, 0.0)]
    x = z = heading = 0.0
    n = int(LENGTH / STEP) + 1
    for i in range(n):
        s = i * STEP
        k = curvature(s + STEP * 0.5)
        mid = heading + k * STEP * 0.5
        x += math.cos(mid) * STEP
        z += math.sin(mid) * STEP
        heading += k * STEP
        points.append((x, z, heading))
    return points


PLAN = integrate()


def at(s):
    i = min(int(round(s / STEP)), len(PLAN) - 1)
    x, z, h = PLAN[i]
    return x, z, h


def roll(c):
    return round(math.degrees(math.asin(c / 1500.0)), 1)


def text(v):
    return f'{v:.3f}'.rstrip('0').rstrip('.') if abs(v) >= 0.0005 else '0'


def bezier(a, b, ya, yb, ga, gb):
    xa, za, ha = a
    xb, zb, hb = b
    chord = math.hypot(xb - xa, zb - za)
    turn = hb - ha
    handle = chord / (3.0 * math.cos(turn / 4.0) ** 2)
    if abs(turn) < 1e-9 and abs(ga - gb) < 1e-12:
        return (0, 0, 0), (0, 0, 0)
    c1 = (math.cos(ha) * handle, ga * handle, math.sin(ha) * handle)
    c2 = (-math.cos(hb) * handle, -gb * handle, -math.sin(hb) * handle)
    return c1, c2


SURFACE = '1.435 0.25 25.0 20 0 flat vis rail_screw_used1 4 tpbps-new2 0.2 0.5 1.1'


def track(name, p0, r0, c1, c2, p3, r3, attributes, length):
    point = lambda p: ' '.join(text(v) for v in p)
    lines = [f'node -1 0 {name} track normal {text(length)} {SURFACE}',
             f'{point(p0)} {text(r0)} //point 1', f'{point(c1)} //control vector 1', f'{point(c2)} //control vector 2',
             f'{point(p3)} {text(r3)} //point 2', '0']
    lines += attributes
    lines.append('endtrack')
    return '\n'.join(lines)


def main():
    random.seed(20261123)
    out = ['// the old scenery of the takeover tests, written by make_old.py: tracks of no design, as old sceneries have them', '']
    # line a: the cuts 20 to 45 m apart and where the clothoids begin and end (where an old scenery has its tracks of the transition curves
    # and their cant ramps), the joints on the line to the millimetre
    bounds = [STRAIGHT_IN, STRAIGHT_IN + L, STRAIGHT_IN + L + ARC, STRAIGHT_IN + 2 * L + ARC]
    cuts = [0.0]
    while cuts[-1] + 45.0 < LENGTH:
        cuts.append(cuts[-1] + random.uniform(20.0, 45.0))
    cuts = sorted(set([c for c in cuts if all(abs(c - b) > 5.0 for b in bounds)] + bounds))
    cuts.append(LENGTH)
    names = []
    for i in range(len(cuts) - 1):
        names.append('none' if i in (5, 11) else f'a_{i + 1}')
    reversed_ = {2, 7, 12}
    for i in range(len(cuts) - 1):
        a, b = cuts[i], cuts[i + 1]
        pa, pb = at(a), at(b)
        p0 = (round(pa[0], 3), round(height(a), 3), round(pa[1], 3))
        p3 = (round(pb[0], 3), round(height(b), 3), round(pb[1], 3))
        c1, c2 = bezier(pa, pb, height(a), height(b), grade(a + 1e-6), grade(b - 1e-6))
        c1 = tuple(round(v, 4) for v in c1)
        c2 = tuple(round(v, 4) for v in c2)
        r0, r3 = roll(cant(a + 1e-6)), roll(cant(b - 1e-6))
        attributes = ['velocity 100']
        if names[i] == 'a_2':
            attributes.append('event2 sem_a_light')
        if names[i] == 'a_3':
            attributes.append('event1 sem_a_set')
        if names[i] in ('a_4', 'a_5', 'a_6'):
            attributes.append('isolated obw_a')
        if i in reversed_:
            # written against the line: its ends swapped, the control vectors with them, the rolls of the other sign
            out.append(track(names[i], p3, -r3, c2, c1, p0, -r0, attributes, b - a))
        else:
            out.append(track(names[i], p0, r0, c1, c2, p3, r3, attributes, b - a))
    # the switch at the end of line a: the main path straight on, the diverging one to the left by 1:9 over 33 m
    xe, ze, he = at(LENGTH)
    ye = height(LENGTH)
    d = (math.cos(he), math.sin(he))
    left = (-d[1], d[0])
    main_end = (xe + d[0] * 33.23, ye + 0.005 * 33.23, ze + d[1] * 33.23)
    angle = math.atan(1.0 / 9.0)
    div_end = (xe + d[0] * 32.9 + left[0] * 1.85, ye + 0.005 * 33.0, ze + d[1] * 32.9 + left[1] * 1.85)
    div_heading = (math.cos(he + angle), math.sin(he + angle))
    start = (round(xe, 3), round(ye, 3), round(ze, 3))
    main_end = tuple(round(v, 3) for v in main_end)
    div_end = tuple(round(v, 3) for v in div_end)
    point = lambda p: ' '.join(text(v) for v in p)
    h = 11.0
    out.append('\n'.join([
        f'node -1 0 zwr_1 track switch 33.23 {SURFACE}',
        f'{point(start)} 0', '0 0 0', '0 0 0', f'{point(main_end)} 0', '0',
        f'{point(start)} 0', f'{text(d[0] * h)} {text(0.005 * h)} {text(d[1] * h)}', f'{text(-div_heading[0] * h)} {text(-0.005 * h)} {text(-div_heading[1] * h)}',
        f'{point(div_end)} 0', '300',
        'velocity 100', 'endtrack']))
    # line b on from the main end, line c on from the diverging end: straights at 5 per mille
    for prefix, origin, heading, count in (('b', main_end, d, 3), ('c', div_end, div_heading, 2)):
        for k in range(count):
            a = (origin[0] + heading[0] * 50.0 * k, origin[1] + 0.005 * 50.0 * k, origin[2] + heading[1] * 50.0 * k)
            b = (origin[0] + heading[0] * 50.0 * (k + 1), origin[1] + 0.005 * 50.0 * (k + 1), origin[2] + heading[1] * 50.0 * (k + 1))
            a = tuple(round(v, 3) for v in a)
            b = tuple(round(v, 3) for v in b)
            out.append(track(f'{prefix}_{k + 1}', a, 0, (0, 0, 0), (0, 0, 0), b, 0, ['velocity 60'], 50.0))
    # the objects along line a
    def beside(s, offset, up):
        x, z, hh = at(s)
        lx, lz = -math.sin(hh), math.cos(hh)
        return (round(x + lx * offset, 3), round(height(s) + up, 3), round(z + lz * offset, 3)), math.degrees(math.atan2(math.cos(hh), math.sin(hh)))
    out.append('')
    out.append('// the signal of line a, named by the events of a_2 and a_3, and its memory cell')
    p, yaw = beside(150.0, -3.0, 0.0)
    out.append(f'node -1 0 sem_a model {point(p)} {text(round(yaw, 1))} sygnalizacja/semafor_sbl.t3d none endmodel')
    p, yaw = beside(120.0, -4.0, 0.0)
    out.append(f'node -1 0 sem_a_mem memcell {point(p)} sbl 0 0 none endmemcell')
    out.append('event sem_a_light lights 0 sem_a 1 0 0 endevent')
    out.append('event sem_a_set updatevalues 0 sem_a_mem sbl 1 0 endevent')
    out.append('')
    out.append('// the catenary of line a: a pole every 50 m on its left, the wires between them over the track')
    poles = [s for s in range(25, int(LENGTH) - 20, 50)]
    for k, s in enumerate(poles):
        p, yaw = beside(float(s), 3.5, 0.0)
        out.append(f'node -1 0 slup_{k + 1} model {point(p)} {text(round(yaw, 1))} slupy/slup_k.t3d none endmodel')
    for k in range(len(poles) - 1):
        a, b = float(poles[k]), float(poles[k + 1])
        stagger = 0.2 if k % 2 == 0 else -0.2
        p1, _ = beside(a, stagger, 5.6)
        p2, _ = beside(b, -stagger, 5.6)
        p3, _ = beside(a, stagger, 7.2)
        p4, _ = beside(b, -stagger, 7.2)
        out.append(f'node -1 0 none traction zasilanie 3000 2000 0.01 cu 1 0 {point(p1)} {point(p2)} {point(p3)} {point(p4)} 1.6 5 4 0.3 vis endtraction')
    out.append('')
    out.append('// the hectometre posts of line a, on its right')
    for s in range(100, int(LENGTH), 100):
        p, yaw = beside(float(s), -2.5, 0.0)
        out.append(f'node -1 0 none model {point(p)} {text(round(yaw, 1))} znaki/hektometr.t3d none endmodel')
    out.append('')
    out.append('// a tree far off line a, a cabinet between lines b and c')
    p, yaw = beside(400.0, 25.0, 0.0)
    out.append(f'node -1 0 none model {point(p)} 0 drzewa/tree1.t3d none endmodel')
    on_b = (main_end[0] + d[0] * 10.0, main_end[2] + d[1] * 10.0)
    on_c = (div_end[0] + div_heading[0] * 10.0, div_end[2] + div_heading[1] * 10.0)
    cab = ((on_b[0] + on_c[0]) * 0.5, main_end[1], (on_b[1] + on_c[1]) * 0.5)
    out.append(f'node -1 0 szafa_1 model {point(tuple(round(v, 3) for v in cab))} 0 ludzie/szafa_kablowa.t3d none endmodel')
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'old.scn')
    with open(path, 'w', newline='\n') as file:
        file.write('\n'.join(out) + '\n')


if __name__ == '__main__':
    main()
