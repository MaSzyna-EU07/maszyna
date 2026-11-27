"""Writes the eu07_td_off_track_curve_* cases and their fragments (cases/td_off_track_curve_*.scn): a track of R 300 m
laid on its own in the fields of td.scn, on level ground, on an embankment 4 m high and in a cutting whose banks rise
4 m at 1:1.5 and on at 1:5, with an EU07 without a driver overturning on it (physics.derailment new).

    python tests/scenarios/make_off_track_curves.py
"""
import json
import math
from pathlib import Path

CASES = str(Path(__file__).resolve().parent / "cases") + "/"
R = 300.0
CENTRE = (-600.0, 400.0)  # x, z
THETA = 0.5  # rad of the track, 150 m
RAIL = {'flat': 0.2, 'embankment': 4.2, 'cutting': 0.2}
# the cross section of the ground: (offset from the axis, height), outwards from the centre of the curve positive;
# the grass of td.scn lies at 0 around it
SECTIONS = {
    'embankment': [(-9.5, 0.0), (-3.5, 4.0), (3.5, 4.0), (9.5, 0.0)],
    'cutting': [(-150.0, 32.1), (-9.5, 4.0), (-3.5, 0.0), (3.5, 0.0), (9.5, 4.0), (150.0, 32.1)],
}
BACKSLASH = chr(92)
NEWLINE = chr(10)


def point(phi, offset, height):
    # outwards: away from the centre of the curve
    r = R + offset
    return (CENTRE[0] + r * math.cos(phi), height, CENTRE[1] + r * math.sin(phi))


def triangles(kind):
    # a triangle strip for each band of the cross section, along the curve every 0.03 rad (9 m)
    nodes = []
    section = SECTIONS[kind]
    phis = [-0.3 + 0.03 * i for i in range(41)]  # 90 m before the track to 120 m past it
    for (o1, h1), (o2, h2) in zip(section, section[1:]):
        lines = []
        for phi in phis:
            for offset, height in ((o1, h1), (o2, h2)):
                x, y, z = point(phi, offset, height)
                lines.append(f"{x:.2f} {y:.2f} {z:.2f} 0 1 0 0 0")
        nodes.append("node -1 0 none triangle_strip grass" + NEWLINE + (" end" + NEWLINE).join(lines) + NEWLINE + "endtri")
    return nodes


def track(kind):
    y = RAIL[kind]
    p1 = point(0.0, 0.0, y)
    p2 = point(THETA, 0.0, y)
    k = 4.0 / 3.0 * math.tan(THETA / 4.0) * R
    t1 = (-math.sin(0.0), math.cos(0.0))
    t2 = (-math.sin(THETA), math.cos(THETA))
    return NEWLINE.join([
        f"node -1 0 autotest_curve track normal {R * THETA:.4f} 1.435 0.15 25.0 20 0 flat vis",
        " rail_screw_used1 6 1435mm/tpbps-new2 0.2 0.5 1.1",
        f"{p1[0]:.4f} {y} {p1[2]:.4f}  0.0  //point 1",
        f"{t1[0] * k:.4f} 0.0 {t1[1] * k:.4f}  //control vector 1",
        f"{-t2[0] * k:.4f} 0.0 {-t2[1] * k:.4f}  //control vector 2",
        f"{p2[0]:.4f} {y} {p2[2]:.4f}  0.0  //point 2",
        f"{R}",
        "endtrack"])


WORDS = {
    'flat': ("on level ground", "the grass of td.scn",
             "it slides on the grass out of the curve, falls on its side and stops"),
    'embankment': ("on an embankment 4 m high with slopes of 1:1.5", "triangles of the fragment",
                   "it falls on its side over the edge of the embankment, slides down its slope and on over the level ground at its foot, 4 m lower"),
    'cutting': ("in a cutting with banks of 1:1.5 4 m high, the ground rising on beyond them at 1:5", "triangles of the fragment",
                "it runs into the bank on the outside of the curve, climbs it and stops sooner than on level ground"),
}
CHECKS = {
    'flat': {"min_offset": 3, "min_distance": 50, "max_distance": 250, "lying": True},
    'embankment': {"min_offset": 5, "min_distance": 30, "max_distance": 300, "lying": True},
    'cutting': {"min_offset": 3, "min_distance": 20, "max_distance": 230},
}
HEADER = [
    "// added by tests/scenarios/run.py to td.scn, whose own trainsets were removed (physics.derailment new, the vehicles",
    "// off the track moving on the ground, vehicle/derailmotion.h): a track of R 300 m without cant laid on its own in the",
    "// fields of td.scn (autotest_curve), {where}; written by a script, see the README.",
    "// an EU07 with an AI driver and no timetable stands on track 1 of Stacja1, where td.scn keeps its timetable train, so",
    "// it stays put and logs its physics; the EU07 of the case has no driver and runs at {{velocity}} km/h from 20 m past",
    "// point 1 of the curve towards its point 2",
]
TRAINS = [
    "trainset none tdo_n1734 0.0 0.0",
    "node -1 0 {{vehicle}} dynamic pkp" + BACKSLASH + "4e_v1 4e-004 4e 0 headdriver 0 0 enddynamic",
    "endtrainset",
    "",
    "trainset none autotest_curve 20.0 {{velocity}}",
    "node -1 0 {{vehicle}}_off dynamic pkp" + BACKSLASH + "4e_v1 4e-004 4e 0 nobody 0 0 enddynamic",
    "endtrainset",
]
for kind in ('flat', 'embankment', 'cutting'):
    nodes = ([] if kind == 'flat' else triangles(kind)) + [track(kind)]
    where, source, what = WORDS[kind]
    lines = [line.replace("{where}", where) for line in HEADER] + nodes + TRAINS
    with open(CASES + f"td_off_track_curve_{kind}.scn", "w", encoding="utf-8", newline=NEWLINE) as out:
        out.write(NEWLINE.join(lines) + NEWLINE)
    case = {
        "description": f"physics.derailment new: an EU07 without a driver runs at 150 km/h into a curve of R 300 m without cant, a track of 150 m laid on its own in the fields of td.scn (autotest_curve), {where} ({source}). Over its overturning speed (about 120 km/h) it overturns out of the curve: {what}.",
        "base_scenery": "td.scn",
        "time": "10:34",
        "fragment": f"td_off_track_curve_{kind}.scn",
        "vars": {"velocity": "150"},
        "ini": ["gfxrenderer null", "physics.derailment new", "couplerphysics all"],
        "expect": "timeout",
        "duration": 60,
        "timeout": 600,
        "checks": [
            {"type": "log_contains", "pattern": "derailment: autotest_" + BACKSLASH + "S+_off leaves the track"},
            dict({"type": "leaves_track", "vehicle": "{{vehicle}}_off", "cause": "overturned"}, **CHECKS[kind]),
        ],
    }
    with open(CASES + f"eu07_td_off_track_curve_{kind}.json", "w", encoding="utf-8", newline=NEWLINE) as out:
        out.write(json.dumps(case, indent=2, ensure_ascii=False) + NEWLINE)
print("written")
