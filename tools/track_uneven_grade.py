#!/usr/bin/env python3
"""Grades the tracks of any scenery for the uneven attribute, from what the scenery shows: textures, grass on the track,
curves, speed and road crossings. The originals stay as they are, graded copies of the files with tracks go to --out.

    python track_uneven_grade.py scenery/td.scn
    python track_uneven_grade.py scenery/baltyk_skm1.scn --out graded
    python track_uneven_grade.py scenery/baltyk_skm1.scn --out graded --route -107,2300 -4120,2491 --straight tor0290,tor0906
    python track_uneven_grade.py scenery/mytracks.scm --root scenery --out graded --gain 1.2

With no --out it only reports. See track_uneven_doc.md for the attribute. Python 3.8 or later, no other packages.
"""
import argparse
import collections
import heapq
import math
import pathlib
import re
import sys
import zlib

# PLK Id-14 appendix 1, class C: vertical limit in mm by speed in km/h
LIMITS = [(20, 50), (30, 40), (40, 35), (50, 30), (60, 25), (70, 21), (80, 18), (90, 16), (100, 14), (110, 12), (120, 10), (130, 9), (140, 8), (150, 7), (160, 6)]
# how worn a texture looks, 0-1: the first pattern found in its name counts, anything else is 0.5
BALLAST_WEAR = [("grass|traw", 0.95), ("old|star", 0.9), ("oil|smar", 0.6), ("new|nowa", 0.0), ("bps|beton|conc|tpb", 0.2), ("drew|wood|tpd", 0.45)]
RAIL_WEAR = [("unused2", 1.0), ("unused", 0.85), ("rused2", 0.8), ("rused", 0.65), ("used2", 0.45), ("used", 0.2), ("new", 0.0), ("old|rust", 0.8)]
WOOD = re.compile("drew|wood|tpd")
GRASS = re.compile("grass|traw|chwast|weed")
NO_SPEED = 40.0  # km/h, a stretch with no velocity anywhere: station and side tracks
SWITCH_MOST = 35.0  # the limit of 40 km/h
DIVERGING = 40.0  # km/h through the diverging path
MOST, MOST_UP = 50.0, 60.0  # of uneven and of its vertical part
REACH = 0.3  # how near the ends of tracks count as joined, m
SHORT = 2.0  # the pieces inside a slip don't tell the quality of the line
PIECE = 100.0  # a track without a speed shorter than this can take it through a switch, m
GRASS_NEAR, GRASS_FULL = 2.5, 14.0  # grass this near the middle grows on the track; plants per 100 m of an overgrown one
NEAR_LEAST, NEAR_MOST = 0.5, 0.95  # how far a chosen track goes towards the limit
CURVE_STEP = 1.0 / 2500.0  # a difference of curvature that counts as a change, 1/m
RISE = 0.0273 * 4.5  # how fast pressure lifts a pantograph, m/s, by TDynamicObject::Update
GAP_MOST = 0.006  # the gap to the wire allowed; at 0.01 m the voltage goes and the main breaker trips
OVERSPEED = 1.1  # a driver can go this much over the speed of the track
# as up_waves and the dip of the joints in world/TrackUneven.h
STRAY_PER_MM, DIP_SHARE, DIP_REACH = 0.00145, 0.4, 3.0
GOLDEN = math.pi * (3.0 - math.sqrt(5.0))
UP_WAVES = [(2 * math.pi / length * math.cos(GOLDEN * i), 2 * math.pi / length * math.sin(GOLDEN * i), GOLDEN * (i + 1), share) for i, (length, share) in enumerate(zip((8.3, 11.9, 16.7, 23.9, 37.1, 47.3, 61.9, 83.3), (.14, .19, .22, .22, .23, .23, .23, .23)))]
TOKEN = re.compile(r"//[^\n]*|\S+")
NUMBER = re.compile(r"^[-+]?(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?$")


def limit(speed):
    if speed <= LIMITS[0][0]:
        return float(LIMITS[0][1])
    for (v0, n0), (v1, n1) in zip(LIMITS, LIMITS[1:]):
        if speed <= v1:
            return n0 + (n1 - n0) * (speed - v0) / (v1 - v0)
    return float(LIMITS[-1][1])


def wear(name, table):
    name = (name or "").lower()
    for pattern, value in table:
        if re.search(pattern, name):
            return value
    return 0.5


def hashed(salt, name):
    return zlib.crc32(salt + name.encode("latin-1", "replace")) / 0xffffffff


class Scenery:
    """All that's found by following the includes from the given files."""

    def __init__(self, root):
        self.root = root
        self.tracks, self.roads, self.plants = [], [], collections.defaultdict(list)
        self.seen = set()
        self.notes = collections.Counter()
        self.texts = {}

    def read(self, path, offset=(0.0, 0.0, 0.0)):
        path = path.resolve()
        if path not in self.seen:
            self.seen.add(path)
            FileReader(self, path, offset).run()

    def include(self, name, params, origin):
        path = self.root / name.replace("\\", "/")
        # a plant takes its place in the parameters; a file with grass in its name and none of them just lists plants
        numbers = [float(w) for w in params if NUMBER.match(w)]
        if GRASS.search(name.lower()) and len(numbers) >= 3:
            self.plant(numbers[0] + origin[0], numbers[2] + origin[2])
        elif not path.is_file():
            self.notes["brak pliku " + name] += 1
        elif not params:
            self.read(path, origin)
        elif path.stat().st_size < 4000000 and re.search(rb"\btrack\s+(normal|switch)\b", path.read_bytes(), re.I):
            # the same file can come many times with other parameters: its tracks can't be told apart in one copy
            self.notes["pliki z parametrami, tory pominiete"] += 1

    def plant(self, x, z):
        self.plants[(round(x / 10), round(z / 10))].append((x, z))


class FileReader:
    """Goes through the words of one file. Each handler takes the place of its word and gives the place of the last word it used."""

    def __init__(self, scenery, path, offset):
        self.scenery, self.path = scenery, path
        text = path.read_bytes().decode("latin-1")
        scenery.texts[path] = text
        self.matches = [m for m in TOKEN.finditer(text) if not m.group(0).startswith("//")]
        self.words = [m.group(0) for m in self.matches]
        self.lower = [w.lower() for w in self.words]
        self.origins = [offset]
        self.handlers = {"include": self.include, "origin": self.origin, "endorigin": self.endorigin, "rotate": self.rotate, "node": self.node}

    def run(self):
        i = 0
        while i < len(self.words):
            handler = self.handlers.get(self.lower[i])
            i = (handler(i) if handler else i) + 1

    def include(self, i):
        if i + 1 >= len(self.words):
            return i
        end = i + 1
        while end < len(self.words) and self.lower[end] != "end":
            end += 1
        self.scenery.include(self.words[i + 1], self.words[i + 2:end], self.origins[-1])
        return end

    def origin(self, i):
        values = self.words[i + 1:i + 4]
        if len(values) < 3 or not all(NUMBER.match(w) for w in values):
            return i
        self.origins.append(tuple(o + float(w) for o, w in zip(self.origins[-1], values)))
        return i + 3

    def endorigin(self, i):
        if len(self.origins) > 1:
            self.origins.pop()
        return i

    def rotate(self, i):
        if i + 3 < len(self.words) and any(w.strip("0.-") for w in self.words[i + 1:i + 4]):
            self.scenery.notes["rotate"] += 1
        return i

    def node(self, i):
        if i + 4 >= len(self.words):
            return i
        kind = self.lower[i + 4]
        if kind == "track":
            return self.track(i)
        if kind == "model" and i + 9 < len(self.words) and GRASS.search(self.lower[i + 9]) and all(NUMBER.match(w) for w in self.words[i + 5:i + 8]):
            self.scenery.plant(float(self.words[i + 5]) + self.origins[-1][0], float(self.words[i + 7]) + self.origins[-1][2])
        return i

    def paths_from(self, cursor, count):
        paths = []
        origin = self.origins[-1]
        for _ in range(count):
            values = self.words[cursor:cursor + 15]
            if len(values) < 15 or not all(NUMBER.match(v) for v in values):
                return None
            n = [float(v) for v in values]
            part = {"p1": [n[j] + origin[j] for j in range(3)], "c1": n[4:7], "c2": n[7:10], "p2": [n[10 + j] + origin[j] for j in range(3)], "radius": abs(n[14])}
            part["line"] = line_of(part)
            part["length"] = sum(math.dist(a, b) for a, b in zip(part["line"], part["line"][1:]))
            paths.append(part)
            cursor += 15
        return paths

    def track(self, i):
        words, lower = self.words, self.lower
        at = i + 4
        end = at
        while end < len(words) and lower[end] != "endtrack":
            end += 1
        if end >= len(words) or at + 10 >= end:
            return i
        kind = lower[at + 1]
        if kind not in ("normal", "switch", "road"):
            return end
        visible = lower[at + 9] == "vis"
        paths = self.paths_from(at + (16 if visible else 10), 2 if kind == "switch" else 1)
        if paths is None:
            self.scenery.notes["tory z parametrami, pominiete"] += 1
        elif kind == "road":
            self.scenery.roads.append(paths[0])
        else:
            self.add_track(i, at, end, kind, visible, paths)
        return end

    def add_track(self, i, at, end, kind, visible, paths):
        words = self.words
        block = self.lower[at:end]
        velocity = 0.0
        if "velocity" in block and NUMBER.match(words[at + block.index("velocity") + 1]):
            velocity = float(words[at + block.index("velocity") + 1])
        self.scenery.tracks.append({
            "name": words[i + 3], "kind": kind,
            "rail": self.lower[at + 10] if visible else None, "ballast": self.lower[at + 12] if visible else None,
            "sound": float(words[at + 5]) if NUMBER.match(words[at + 5]) else -1.0,
            "paths": paths, "speed": velocity if velocity > 0 else None, "source": "wpis" if velocity > 0 else None,
            "file": self.path, "offset": self.matches[end].start(),
            "taken": any(w in block for w in ("uneven", "unevenup", "unevenside", "unevencross", "jointdip"))})


def bezier(part, t):
    a = [part["p1"][i] + part["c1"][i] for i in range(3)]
    b = [part["p2"][i] + part["c2"][i] for i in range(3)]
    u = 1 - t
    return [u * u * u * part["p1"][i] + 3 * u * u * t * a[i] + 3 * u * t * t * b[i] + t * t * t * part["p2"][i] for i in range(3)]


def line_of(part, step=5.0):
    if not any(part["c1"]) and not any(part["c2"]):
        return [list(part["p1"]), list(part["p2"])]
    count = max(4, int(math.dist(part["p1"], part["p2"]) / step))
    return [bezier(part, i / count) for i in range(count + 1)]


def plan_radius(line):
    least = math.inf
    for a, b, c in zip(line, line[1:], line[2:]):
        area = abs((b[0] - a[0]) * (c[2] - a[2]) - (c[0] - a[0]) * (b[2] - a[2])) / 2
        if area > 1e-6:
            least = min(least, math.dist((a[0], a[2]), (b[0], b[2])) * math.dist((b[0], b[2]), (c[0], c[2])) * math.dist((c[0], c[2]), (a[0], a[2])) / (4 * area))
    return least


def end_curve(part, key):
    line = part["line"]
    if len(line) < 3:
        return 1.0 / part["radius"] if part["radius"] > 0 else 0.0
    a, b, c = line[:3] if key == "p1" else line[-3:]
    sides = math.dist(a, b) * math.dist(b, c) * math.dist(c, a)
    return 2.0 * ((b[0] - a[0]) * (c[2] - b[2]) - (b[2] - a[2]) * (c[0] - b[0])) / sides if sides > 0 else 0.0


class Joints:
    def __init__(self, tracks):
        self.ends = collections.defaultdict(list)
        for t in tracks:
            for part in t["paths"]:
                for key in ("p1", "p2"):
                    self.ends[(round(part[key][0]), round(part[key][2]))].append((t, part, key))

    def at(self, point):
        found = []
        for cx in (round(point[0]) - 1, round(point[0]), round(point[0]) + 1):
            for cz in (round(point[2]) - 1, round(point[2]), round(point[2]) + 1):
                for item in self.ends.get((cx, cz), ()):
                    if abs(item[1][item[2]][0] - point[0]) <= REACH and abs(item[1][item[2]][2] - point[2]) <= REACH:
                        found.append(item)
        return found


def plain_neighbours(t, joints):
    return [other for key in ("p1", "p2") for other, _, _ in joints.at(t["paths"][0][key]) if other is not t and other["kind"] == "normal"]


def joined_at(t, point, joints, seen):
    # the plain tracks joined at a point, and the far ends of the straight paths of the switches joined there
    plain, further = [], []
    for other, part, side in joints.at(point):
        if other is t or id(part) in seen:
            continue
        seen.add(id(part))
        if other["kind"] != "switch":
            plain.append(other)
        elif other["paths"].index(part) != other["bent"]:
            further.append(part["p2" if side == "p1" else "p1"])
    return plain, further


def plain_past_switches(t, point, joints):
    # the plain tracks reached from a point of a track along the straight paths of switches
    points, seen, found = [point], set(), []
    for _ in range(6):
        onward = []
        for reached in points:
            plain, further = joined_at(t, reached, joints, seen)
            found += plain
            onward += further
        points = onward
    return found


def short_pieces_past_switches(t, joints):
    return [o for key in ("p1", "p2") for o in plain_past_switches(t, t["paths"][0][key], joints) if o["paths"][0]["length"] < PIECE]


def spread_speed(tracks, joints, reached):
    heads = collections.deque(t for t in tracks if t["kind"] == "normal" and t["speed"])
    while heads:
        t = heads.popleft()
        for other in reached(t, joints):
            if other["speed"] is None:
                other["speed"], other["source"] = t["speed"], "sasiad"
                heads.append(other)


def settle_speeds(tracks, joints):
    # from the nearest track on the stretch between switches, a short piece also along the straight path of a switch
    spread_speed(tracks, joints, plain_neighbours)
    spread_speed(tracks, joints, short_pieces_past_switches)
    for t in tracks:
        if t["speed"] is None:
            t["speed"], t["source"] = NO_SPEED, "brak"


def plain_ends(tracks):
    for t in tracks:
        if t["kind"] == "normal":
            for part in t["paths"]:
                yield part["p1"]
                yield part["p2"]


def distances(tracks, joints, station):
    best, heap = {}, []
    for point in plain_ends(tracks):
        gap = math.dist((point[0], point[2]), station)
        node = (round(point[0], 1), round(point[2], 1))
        if gap < 300 and gap < best.get(node, math.inf):
            best[node] = gap
            heapq.heappush(heap, (gap, node, point))
    while heap:
        distance, node, point = heapq.heappop(heap)
        if distance > best.get(node, math.inf):
            continue
        for _, part, key in joints.at(point):
            other = part["p2" if key == "p1" else "p1"]
            onward = (round(other[0], 1), round(other[2], 1))
            if distance + part["length"] < best.get(onward, math.inf):
                best[onward] = distance + part["length"]
                heapq.heappush(heap, (best[onward], onward, other))
    return best


def grass_on(part, plants):
    near, count = set(), max(1, int(part["length"] / 3))
    line = part["line"]
    for i in range(count + 1):
        position = i / count * (len(line) - 1)
        low = min(int(position), len(line) - 2)
        share = position - low
        x, z = line[low][0] + (line[low + 1][0] - line[low][0]) * share, line[low][2] + (line[low + 1][2] - line[low][2]) * share
        for cx in (round(x / 10) - 1, round(x / 10), round(x / 10) + 1):
            for cz in (round(z / 10) - 1, round(z / 10), round(z / 10) + 1):
                for plant in plants.get((cx, cz), ()):
                    if (plant[0] - x) ** 2 + (plant[1] - z) ** 2 < GRASS_NEAR * GRASS_NEAR:
                        near.add(plant)
    return len(near) * 100.0 / max(part["length"], 5.0)


def cells_under(a, b):
    for x in range(math.floor(min(a[0], b[0]) / 50), math.floor(max(a[0], b[0]) / 50) + 1):
        for z in range(math.floor(min(a[2], b[2]) / 50), math.floor(max(a[2], b[2]) / 50) + 1):
            yield x, z


def road_cells(roads):
    cells = collections.defaultdict(list)
    for road in roads:
        for a, b in zip(road["line"], road["line"][1:]):
            for cell in cells_under(a, b):
                cells[cell].append((a, b))
    return cells


def cross_in_plan(a, b, c, d):
    rx, rz, sx, sz = b[0] - a[0], b[2] - a[2], d[0] - c[0], d[2] - c[2]
    k = rx * sz - rz * sx
    if abs(k) < 1e-9:
        return False
    t = ((c[0] - a[0]) * sz - (c[2] - a[2]) * sx) / k
    u = ((c[0] - a[0]) * rz - (c[2] - a[2]) * rx) / k
    return 0 <= t <= 1 and 0 <= u <= 1


def hits_road(part, cells):
    line = part["line"]
    return any(cross_in_plan(a, b, c, d) for a, b in zip(line, line[1:]) for cell in cells_under(a, b) for c, d in cells.get(cell, ()))


def heights_per_mm(t):
    # the height of the rail under a pantograph frame by frame, per mm of the vertical part
    step = t["speed"] * OVERSPEED / 3.6 / 60.0
    heights, done = [], 0.0
    line = t["paths"][0]["line"]
    for a, b in zip(line, line[1:]):
        length = math.dist(a, b)
        count = max(1, int(length / step))
        for i in range(count):
            share = i / count
            x, z = a[0] + (b[0] - a[0]) * share, a[2] + (b[2] - a[2]) * share
            height = STRAY_PER_MM * sum(s * math.sin(wx * x + wz * z + phase) for wx, wz, phase, s in UP_WAVES)
            if t["sound"] > 2.0 * DIP_REACH:
                phase = math.fmod(done + length * share, t["sound"])
                joint = min(phase, t["sound"] - phase)
                if joint < DIP_REACH:
                    height -= 0.001 * DIP_SHARE * math.cos(math.pi / 2 * joint / DIP_REACH) ** 2
            heights.append(height)
        done += length
    return heights


def largest_gap(heights, up):
    gap = worst = 0.0
    climb = RISE / 60.0
    for before, after in zip(heights, heights[1:]):
        gap = max(0.0, gap + (before - after) * up - climb)
        worst = max(worst, gap)
    return worst


def ceiling(t, product):
    # the worst track of a speed comes up to it and the better ones stay under it, so the state of a track still shows
    return product / t["speed"] * (0.7 + 0.3 * min(1.0, max(0.0, (t["state"] - 0.5) / 0.7)))


def path_radius(part):
    return min(plan_radius(part["line"]), part["radius"] if part["radius"] > 0 else math.inf)


def mark_diverging_paths(switches):
    # the path that bends more
    for t in switches:
        radii = [path_radius(p) for p in t["paths"]]
        t["bent"] = 1 if len(radii) < 2 or radii[1] <= radii[0] else 0


class Route:
    """How far along from the first point to the second a place lies, measured along the tracks."""

    def __init__(self, tracks, joints, first, last):
        self.first, self.last = first, last
        self.from_first, self.from_last = distances(tracks, joints, first), distances(tracks, joints, last)
        ends = [self.from_last[n] for n in self.from_last if math.dist(n, first) < 300]
        self.length = min(ends) if ends else math.dist(first, last)
        if not ends:
            print("uwaga: po torach nie ma polaczenia miedzy punktami --route, licze w linii prostej")

    def share(self, part):
        shares = []
        for key in ("p1", "p2"):
            place = (part[key][0], part[key][2])
            node = (round(place[0], 1), round(place[1], 1))
            a, b = self.from_first.get(node), self.from_last.get(node)
            if a is None or b is None:
                a, b = math.dist(place, self.first), math.dist(place, self.last)
            shares.append(min(1.0, max(0.0, (a - b + self.length) / (2 * self.length))))
        return 0.5 * sum(shares)


def grade_states(scenery, route, gain):
    cells = road_cells(scenery.roads)
    for t in scenery.tracks:
        part = t["paths"][0]
        radius = path_radius(part)
        curve = 0.0 if radius >= 6000 else min(1.0, max(0.0, (1200 - radius) / 950))
        plain = t["kind"] == "normal"
        density = grass_on(part, scenery.plants) if plain else 0.0
        crossing = plain and hits_road(part, cells)
        progress = route.share(part) if route else 0.5
        state = 0.50 + 0.20 * wear(t["ballast"], BALLAST_WEAR) + 0.10 * wear(t["rail"], RAIL_WEAR) + 0.12 * min(1.0, density / GRASS_FULL) + 0.06 * curve + 0.22 * progress + 0.18 * crossing + 0.06 * (hashed(b"", t["name"]) - 0.5)
        t.update(state=min(state, 1.2) * gain, curve=curve, crossing=crossing, grass=density)
        if plain:
            value = min(limit(t["speed"]) * t["state"], MOST)
            t.update(value=value, up=min(value * 1.25, MOST_UP), side=min(value * (0.9 + 0.4 * curve), MOST))


def fit_under_pantograph(normal):
    # the largest product of the vertical part and the speed with the gap small on every track
    low, high = 100, 4000
    while high - low > 25:
        middle = (low + high) // 2
        if all(largest_gap(t["heights"], min(t["up"], ceiling(t, middle))) <= GAP_MOST for t in normal):
            low = middle
        else:
            high = middle
    for t in normal:
        up = ceiling(t, low)
        if up < t["up"]:
            ratio = up / t["up"]
            t["up"], t["value"], t["side"] = up, t["value"] * ratio, t["side"] * ratio
    return low


def joint_curve_changes(t, part, key, joints):
    own = end_curve(part, key)
    for other, theirs, side in joints.at(part[key]):
        if other is not t and other["kind"] == "normal":
            their = end_curve(theirs, side)
            if abs(own - (-their if side == key else their)) > CURVE_STEP:
                return True
    return False


def curve_changes(t, joints):
    part = t["paths"][0]
    return abs(end_curve(part, "p1") - end_curve(part, "p2")) > CURVE_STEP or any(joint_curve_changes(t, part, key, joints) for key in ("p1", "p2"))


def move_near_limit(normal, joints, share):
    # on wooden sleepers and where the curve changes, a share of the tracks goes nearer the limit, chosen for good by the place
    chosen = 0
    for t in normal:
        if not (WOOD.search(t["ballast"] or "") or curve_changes(t, joints)):
            continue
        part = t["paths"][0]
        name = f"{t['name']}:{part['p1'][0]:.0f}:{part['p1'][2]:.0f}"
        if hashed(b"near", name) >= share:
            continue
        value = t["value"] + (limit(t["speed"]) - t["value"]) * (NEAR_LEAST + (NEAR_MOST - NEAR_LEAST) * hashed(b"how", name))
        if value <= t["value"]:
            continue
        # sideways and the cross level don't move a pantograph, up and down only as far as it keeps to the wire on this track
        up = value
        while up > t["up"] and largest_gap(t["heights"], up) > GAP_MOST:
            up -= 0.5
        t["value"], t["up"], t["side"] = value, max(up, t["up"]), min(value * (0.9 + 0.4 * t["curve"]), MOST)
        chosen += 1
    return chosen


def value_from_plain(t, index, part, joints, product):
    found = [o for key in ("p1", "p2") for o, theirs, _ in joints.at(part[key]) if o["kind"] == "normal" and theirs["length"] >= SHORT]
    if not found:
        return None
    speed = max(o["speed"] for o in found)
    if index == t["bent"]:
        speed = min(speed, DIVERGING)
    return min(limit(speed) * max(o["state"] for o in found), SWITCH_MOST, product / speed)


def value_from_switches(t, part, joints):
    found = []
    for key in ("p1", "p2"):
        for other, theirs, _ in joints.at(part[key]):
            if other["kind"] == "switch" and other is not t:
                found += [other["values"][i] for i, q in enumerate(other["paths"]) if q is theirs and other["values"][i] is not None]
    return max(found) if found else None


def grade_switches(switches, joints, product):
    # each path by the plain tracks it leads to, the worst of them; those only among switches from their neighbours
    for t in switches:
        t["values"] = [value_from_plain(t, index, part, joints, product) for index, part in enumerate(t["paths"])]
    for _ in range(5):
        for t in switches:
            t["values"] = [value_from_switches(t, part, joints) if value is None else value for value, part in zip(t["values"], t["paths"])]
    for t in switches:
        t["values"] = [min(SWITCH_MOST, product / NO_SPEED) if v is None else v for v in t["values"]]


def grade(scenery, options):
    tracks = scenery.tracks
    normal = [t for t in tracks if t["kind"] == "normal"]
    switches = [t for t in tracks if t["kind"] == "switch"]
    joints = Joints(tracks)
    mark_diverging_paths(switches)
    settle_speeds(tracks, joints)
    grade_states(scenery, Route(tracks, joints, *options.route) if options.route else None, options.gain)
    for t in normal:
        t["heights"] = heights_per_mm(t)
    product = fit_under_pantograph(normal)
    chosen = move_near_limit(normal, joints, options.near)
    worst = max((largest_gap(t["heights"], t["up"]) for t in normal), default=0.0)
    for t in normal:
        del t["heights"]
        if t["name"].lower() in options.straight:
            t["value"] = t["up"] = t["side"] = 0.0
    grade_switches(switches, joints, product)
    return product, chosen, worst


def report(scenery, product, chosen, worst):
    normal = [t for t in scenery.tracks if t["kind"] == "normal"]
    switches = [t for t in scenery.tracks if t["kind"] == "switch"]
    print(f"pliki: {len(scenery.seen)}, tory: {len(normal)}, rozjazdy: {len(switches)}, drogi: {len(scenery.roads)}, trawa: {sum(len(v) for v in scenery.plants.values())}")
    print(f"pion razy km/h najwyzej {product}, blizej granicy {chosen} torow, najwieksza przerwa do drutu {worst * 100:.2f} cm (bez wznosu samego drutu)")
    groups = collections.defaultdict(list)
    for t in normal:
        groups[(t["source"], t["speed"])].append(t)
    print("predkosc  skad      torow     km  limit   uneven min/mediana/max   pion max")
    for key in sorted(groups, key=lambda k: (-k[1], k[0])):
        rows = groups[key]
        values = sorted(t["value"] for t in rows)
        print(f"{key[1]:8.0f}  {key[0]:8} {len(rows):6} {sum(t['paths'][0]['length'] for t in rows) / 1000:6.1f} {limit(key[1]):6.1f}   {values[0]:6.1f} {values[len(values) // 2]:6.1f} {values[-1]:6.1f}   {max(t['up'] for t in rows):8.1f}")
    if switches:
        values = sorted(v for t in switches for v in t["values"])
        print(f"rozjazdy: {values[0]:.1f}-{values[-1]:.1f} mm, mediana {values[len(values) // 2]:.1f}")
    for note, count in sorted(scenery.notes.items()):
        print(f"uwaga: {note}: {count}" if not note.startswith("brak") else f"uwaga: {note}")
    if scenery.notes["rotate"]:
        print("uwaga: bloki rotate nie sa uwzgledniane, polaczenia torow z nich moga byc nierozpoznane")
    taken = sum(1 for t in scenery.tracks if t["taken"])
    if taken:
        print(f"uwaga: {taken} torow ma juz nierownosci, zostaja bez zmian")


def attribute(t):
    if t["kind"] == "normal":
        return f"uneven {t['value']:.1f};{t['up']:.1f};{t['side']:.1f} " if t["value"] > 0 else ""
    return "uneven " + ";".join(f"{v:.1f}" for v in t["values"]) + " "


def write(scenery, out):
    by_file = collections.defaultdict(list)
    for t in scenery.tracks:
        if not t["taken"]:
            by_file[t["file"]].append(t)
    written = 0
    for path, tracks in by_file.items():
        text = scenery.texts[path]
        pieces, last = [], 0
        for t in sorted(tracks, key=lambda t: t["offset"]):
            pieces += [text[last:t["offset"]], attribute(t)]
            last = t["offset"]
        pieces.append(text[last:])
        target = out / path.relative_to(scenery.root)
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes("".join(pieces).encode("latin-1"))
        written += 1
    print(f"zapisane pliki: {written}, w {out}")


def point(text):
    x, z = text.split(",")
    return float(x), float(z)


def main():
    parser = argparse.ArgumentParser(description="Ocena torow dowolnej scenerii dla atrybutu uneven.")
    parser.add_argument("files", nargs="+", type=pathlib.Path, help=".scn, .scm albo .inc; includy sa czytane dalej")
    parser.add_argument("--root", type=pathlib.Path, help="folder scenery, wzgledem ktorego ida includy; domyslnie folder pierwszego pliku")
    parser.add_argument("--out", type=pathlib.Path, help="folder na kopie plikow z torami; bez tego tylko raport")
    parser.add_argument("--route", nargs=2, type=point, metavar="X,Z", help="tory pogarszaja sie od pierwszego punktu do drugiego, liczac po torach")
    parser.add_argument("--gain", type=float, default=1.0, help="mnoznik stanu torow, domyslnie 1")
    parser.add_argument("--near", type=float, default=0.6, help="jaka czesc torow na drewnie i przy zmianach krzywizny idzie blizej granicy Id-14, domyslnie 0.6")
    parser.add_argument("--straight", default="", help="nazwy torow bez nierownosci, po przecinku")
    options = parser.parse_args()
    options.straight = {name.strip().lower() for name in options.straight.split(",") if name.strip()}
    root =(options.root or options.files[0].parent).resolve()
    if options.out:
        options.out = options.out.resolve()
        if options.out == root or root in options.out.parents:
            sys.exit("--out nie moze lezec w folderze scenerii: oryginaly maja zostac nietkniete")
    scenery = Scenery(root)
    for path in options.files:
        scenery.read(path)
    if not scenery.tracks:
        sys.exit("nie znaleziono torow")
    product, chosen, worst = grade(scenery, options)
    report(scenery, product, chosen, worst)
    if options.out:
        write(scenery, options.out)


if __name__ == "__main__":
    main()
