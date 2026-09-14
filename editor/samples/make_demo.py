"""Writes editor/samples/demo.m0s: the plan the editor is handed to open.

It is deliberately small — two parallel tracks and a trapez between them — because
that is the case worth having in front of you: a przejście rozjazdowe is the one
piece of trackwork whose two halves have to agree with each other, and everything
about how the editor ties one turnout to another shows up here.

The numbers are not drawn by eye. A 1:9 turnout leaves its frog 1,469 m off the
track it stands on, so between tracks 4,75 m apart the wstawka has 1,811 m left to
cover, which at the crossing angle is 16,405 m of straight and nothing else.
"""

import math
import pathlib

ORIGIN = (643640.5594274866, 493212.77532093093)   # what the scenery's zero stands for
EAST = math.pi / 2.0                               # azimuth: radians clockwise from north

ALPHA = math.atan(1.0 / 9.0)                                              # the crossing angle
FROG_LAT = 190.0 * (1.0 - math.cos(ALPHA)) + 2.7808 * math.sin(ALPHA)     # KR off the track
FROG_ALONG = 3.3108 + 190.0 * math.sin(ALPHA) + 2.7808 * math.cos(ALPHA)  # and along it

X0, Y0 = 643400.0, 493100.0
LENGTH = 1200.0
SPACING = 4.75

GAP = (SPACING - 2.0 * FROG_LAT) / math.sin(ALPHA)   # the wstawka between the two frogs
GAP_ALONG = GAP * math.cos(ALPHA)


def g(value):
    return repr(float(value))


ids = iter(range(1, 200))
tracks = []
turnouts = []


def nid():
    return next(ids)


def track(name, anchor, elems):
    tid = nid()
    tracks.append((tid, name, anchor, elems))
    return tid


def elem(kind, radius, hand, length, hold=None):
    eid = nid()
    out = ["elem %d %s %s %d %s" % (eid, kind, g(radius), hand, g(length))]
    if hold is not None:
        out.append("epar %d %s" % (hold[0], g(hold[1])))
    return eid, out


def turnout(on, station, side, facing=1, opposite=0, kind="Rz 1:9 R190"):
    """`side` is which way the branch comes out in the world: +1 left of the
    kilometrage, -1 right. A trailing turnout is laid on a reversed pose, so the
    same side of the ground is the opposite hand."""
    tid = nid()
    turnouts.append((tid, on, station, side if facing else -side, facing, 1, 0.0, opposite, kind))
    return tid


# --- the two tracks. the second is held at międzytorze against the first, so it lies
# exactly parallel and stays that way when the first one is edited
main_elems = []
MAIN_STRAIGHT, l = elem("line", 0, 0, LENGTH)
main_elems += l
TOR1 = track("tor 1", "anchor pose %s %s %s" % (g(X0), g(Y0), g(EAST)), main_elems)

second_elems = []
_, l = elem("line", 0, 0, LENGTH, hold=(MAIN_STRAIGHT, SPACING))
second_elems += l
TOR2 = track("tor 2", "anchor pose %s %s %s" % (g(X0), g(Y0 + SPACING), g(EAST)), second_elems)


def crossover(name, from_track, from_station, side, from_begin, to_track, to_begin):
    """A turnout on one track, the wstawka off its frog, and the turnout standing
    against it on the other. The far one is trailing: it opens back toward the
    near one, which is what makes the two of them one przejście rozjazdowe."""
    near = turnout(from_track, from_station, side)
    # the far one stands against the near one: where it stands is worked out from that
    # one's frog, so moving either of them carries the other with it. the station written
    # here is only where it would go if the two were ever unpaired
    meets = (from_begin + from_station + FROG_ALONG + GAP_ALONG) - to_begin
    far = turnout(to_track, meets + FROG_ALONG, -side, facing=0, opposite=near)
    _, elems = elem("line", 0, 0, GAP)
    track(name, "anchor port %d frog" % near, elems)
    return near, far


# the trapez: one crossover up onto tor 2, the other back down onto tor 1
crossover("trapez 1-2 (wstawka)", TOR1, 400.0, 1, 0.0, TOR2, 0.0)
crossover("trapez 3-4 (wstawka)", TOR2, 600.0, -1, 0.0, TOR1, 0.0)

# --- the file
curve = 190.0 * ALPHA - 7.188
out = ["m0s 9", "crs 2180", "units m rad 1/m", "next %d" % (next(ids) + 50)]
out += ["view %s %s 260" % (g(X0 + LENGTH * 0.5), g(Y0))]
out += ["origin 1 1 %s %s" % (g(ORIGIN[0]), g(ORIGIN[1]))]
out += ["scn scenery/plan_demo.scn"]
out += ["types 1", "type 9 27.138 0.005 0.07 0 4",
        "tseg 0 3.3108 0 0 0", "tseg 1 7.188 190 190 0",
        "tseg 2 %s 190 190 0" % g(curve), "tseg 3 2.7808 0 0 0", "tname Rz 1:9 R190"]
out += ["turnouts %d" % len(turnouts)]
for tid, on, station, hand, facing, bft, bend, opposite, kind in turnouts:
    out += ["turnout %d %d %s %d %d %d %s %d" % (tid, on, g(station), hand, facing, bft, g(bend), opposite),
            "tutype " + kind]
out += ["tracks %d" % len(tracks)]
for tid, name, anchor, elems in tracks:
    out += ["track %d" % tid, "tkname " + name, anchor,
            "elems %d" % sum(1 for e in elems if e.startswith("elem "))]
    out += elems

pathlib.Path(__file__).with_name("demo.m0s").write_text("\n".join(out) + "\n", encoding="utf-8")
print("tory: %d, rozjazdy: %d, wstawka: %.3f m" % (len(tracks), len(turnouts), GAP))
