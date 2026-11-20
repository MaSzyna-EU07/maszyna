#!/usr/bin/env python3
"""Simulation snapshots (eu07 --snapshot-at / --load-snapshot): reading them and telling two of them apart.

A snapshot is a text file in the syntax of the scenery files: entries of the form

    <type> <name> <key> <value> <key> <value> ... end<type>

with // and /* */ comments, quoted values for texts with spaces, `<>` for an empty text. The first entry is
the header, `snapshot <version> scenery <file> ... endsnapshot`. See simulation/simulationsnapshot.h.

Used by run.py for the cases with a "snapshot" section, and by hand:

    python tests/scenarios/snapshots.py a.snapshot b.snapshot            # what differs, with the numbers
    python tests/scenarios/snapshots.py a.snapshot b.snapshot --exact    # every key which isn't the same text
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

BREAKS = " \t\r\n;"


@dataclass
class Entry:
    type: str
    name: str
    fields: dict[str, str] = field(default_factory=dict)

    def number(self, key: str, default: float = 0.0) -> float:
        try:
            return float(self.fields[key])
        except (KeyError, ValueError):
            return default

    def text(self, key: str, default: str = "") -> str:
        return self.fields.get(key, default)


def tokens(text: str) -> list[str]:
    """The tokens of the file, the way the simulator's parser cuts them: comments out, quotes off."""
    out, token, i, n = [], None, 0, len(text)
    while i < n:
        c = text[i]
        if c == '"':
            token = token or ""
            i += 1
            while i < n and text[i] != '"':
                if text[i] == "\\" and i + 1 < n:
                    i += 1
                token += text[i]
                i += 1
            i += 1
            continue
        if text.startswith("//", i):
            end = text.find("\n", i)
            i = n if end < 0 else end
            continue
        if text.startswith("/*", i):
            end = text.find("*/", i + 2)
            i = n if end < 0 else end + 2
            continue
        if c in BREAKS:
            if token is not None:
                out.append(token)
                token = None
            i += 1
            continue
        token = (token or "") + c
        i += 1
    if token is not None:
        out.append(token)
    return out


def decode(token: str) -> str:
    if token == "<>":
        return ""
    return token[1:] if token.startswith("<") else token


def parse(text: str) -> list[Entry]:
    entries, items, i = [], tokens(text), 0
    while i + 1 < len(items):
        entry = Entry(items[i], decode(items[i + 1]))
        end = "end" + entry.type
        i += 2
        while i < len(items) and items[i] != end:
            if i + 1 >= len(items) or items[i + 1] == end:
                raise ValueError(f"entry {entry.type} {entry.name}: key {items[i]} has no value")
            entry.fields[items[i]] = decode(items[i + 1])
            i += 2
        if i >= len(items):
            raise ValueError(f"entry {entry.type} {entry.name} isn't closed with {end}")
        i += 1
        entries.append(entry)
    if not entries or entries[0].type != "snapshot":
        raise ValueError("not a snapshot: the file doesn't start with a snapshot entry")
    return entries


def load(path: Path | str) -> list[Entry]:
    # the names of a scenery are cp1250; latin-1 keeps every byte apart
    return parse(Path(path).read_bytes().decode("latin-1"))


def by_type(entries: list[Entry], kind: str) -> dict[str, Entry]:
    return {e.name: e for e in entries if e.type == kind}


# ---------------------------------------------------------------- exact comparison

# what a snapshot saved right after it was loaded may hold differently from the one loaded, by key of an
# entry type: nothing, so far
ROUND_TRIP_ALLOWED: dict[str, set[str]] = {}


def exact_differences(first: list[Entry], second: list[Entry], allowed=None) -> list[str]:
    """Every entry and key whose text isn't the same in both: "vehicle eu07 pipe: 5.01 / 5.02"."""
    allowed = ROUND_TRIP_ALLOWED if allowed is None else allowed
    out = []
    a = {(e.type, e.name): e for e in first}
    b = {(e.type, e.name): e for e in second}
    for key in a.keys() | b.keys():
        if key not in a or key not in b:
            out.append(f"{key[0]} {key[1]}: only in the {'first' if key in a else 'second'}")
            continue
        fa, fb = a[key].fields, b[key].fields
        for name in fa.keys() | fb.keys():
            if name in allowed.get(key[0], ()):
                continue
            if fa.get(name) != fb.get(name):
                out.append(f"{key[0]} {key[1]} {name}: {fa.get(name, '(none)')} / {fb.get(name, '(none)')}")
    return sorted(out)


# ---------------------------------------------------------------- comparison with tolerances


@dataclass
class Comparison:
    """How far two snapshots of the same moment are apart."""
    vehicles: int = 0
    position: float = 0.0  # [m] the farthest a vehicle is from where the other snapshot has it
    position_vehicle: str = ""
    velocity: float = 0.0  # [m/s] the largest difference of speed
    velocity_vehicle: str = ""
    pipe: float = 0.0  # [bar] the largest difference of brake pipe pressure
    pipe_vehicle: str = ""
    cylinder: float = 0.0  # [bar] the largest difference of brake cylinder pressure
    cylinder_vehicle: str = ""
    tracks: list[str] = field(default_factory=list)  # vehicles on another track
    couplings: list[str] = field(default_factory=list)  # vehicles coupled another way
    memcells: list[str] = field(default_factory=list)  # cells with other content
    switches: list[str] = field(default_factory=list)  # switches set another way
    isolated: list[str] = field(default_factory=list)  # isolated sections with other counts of axles
    drivers: list[str] = field(default_factory=list)  # drivers elsewhere in their orders or timetables
    events: list[str] = field(default_factory=list)  # events queued in one of them only
    missing: list[str] = field(default_factory=list)  # vehicles in one of them only
    time: float = 0.0  # [s] difference of the clocks

    def lines(self) -> list[str]:
        out = [
            f"vehicles compared: {self.vehicles}",
            f"clock: {self.time:.3f} s apart",
            f"position: {self.position:.3f} m at most ({self.position_vehicle})",
            f"speed: {self.velocity * 3.6:.3f} km/h at most ({self.velocity_vehicle})",
            f"brake pipe: {self.pipe:.3f} bar at most ({self.pipe_vehicle})",
            f"brake cylinders: {self.cylinder:.3f} bar at most ({self.cylinder_vehicle})",
        ]
        for label, items in (("vehicles in one snapshot only", self.missing), ("on another track", self.tracks),
                             ("coupled another way", self.couplings), ("memory cells", self.memcells),
                             ("switches", self.switches), ("isolated sections", self.isolated), ("drivers", self.drivers),
                             ("queued events", self.events)):
            shown = ", ".join(items[:8]) + (f", ... ({len(items)})" if len(items) > 8 else "")
            out.append(f"{label}: {shown if items else 'the same'}")
        return out


def _greater(result: Comparison, what: str, value: float, vehicle: str) -> None:
    if value > getattr(result, what):
        setattr(result, what, value)
        setattr(result, what + "_vehicle", vehicle)


COUPLER_KEYS = ("coupler0", "coupler0end", "coupler0flags", "coupler1", "coupler1end", "coupler1flags")
# of a driver: where it is in its orders and its timetable. the timers and what it saw ahead aren't compared
DRIVER_KEYS = ("order", "orders", "orderpos", "train", "station", "nextstop", "ai", "direction")


def compare(first: list[Entry], second: list[Entry], cell_tolerance: float = 1e-6) -> Comparison:
    result = Comparison()
    ta, tb = by_type(first, "time").get("now"), by_type(second, "time").get("now")
    if ta and tb:
        def seconds(t):
            return ((t.number("day") * 24 + t.number("hour")) * 60 + t.number("minute")) * 60 + t.number("second") \
                + (t.number("milliseconds") % 1000.0) / 1000.0
        result.time = abs(seconds(ta) - seconds(tb))

    va, vb = by_type(first, "vehicle"), by_type(second, "vehicle")
    result.missing = sorted(va.keys() ^ vb.keys())
    for name in sorted(va.keys() & vb.keys()):
        a, b = va[name], vb[name]
        if a.text("removed") or b.text("removed"):
            if a.text("removed") != b.text("removed"):
                result.missing.append(name)
            continue
        result.vehicles += 1
        _greater(result, "position", math.dist([a.number(k) for k in "xyz"], [b.number(k) for k in "xyz"]), name)
        _greater(result, "velocity", abs(a.number("v") - b.number("v")), name)
        _greater(result, "pipe", abs(a.number("pipe") - b.number("pipe")), name)
        _greater(result, "cylinder", abs(a.number("brakepress") - b.number("brakepress")), name)
        if (a.text("track0"), a.text("track1")) != (b.text("track0"), b.text("track1")):
            result.tracks.append(name)
        if any(a.text(k) != b.text(k) for k in COUPLER_KEYS):
            result.couplings.append(name)

    ca, cb = by_type(first, "memcell"), by_type(second, "memcell")
    for name in sorted(ca.keys() | cb.keys()):
        a, b = ca.get(name), cb.get(name)
        if a is None or b is None or a.text("text") != b.text("text") \
                or abs(a.number("value1") - b.number("value1")) > cell_tolerance \
                or abs(a.number("value2") - b.number("value2")) > cell_tolerance:
            result.memcells.append(name)

    sa, sb = by_type(first, "switch"), by_type(second, "switch")
    result.switches = sorted(n for n in sa.keys() | sb.keys()
                             if n not in sa or n not in sb or sa[n].text("state") != sb[n].text("state"))

    ia, ib = by_type(first, "isolated"), by_type(second, "isolated")
    result.isolated = sorted(n for n in ia.keys() | ib.keys()
                             if n not in ia or n not in ib or ia[n].text("axles") != ib[n].text("axles"))

    da, db = by_type(first, "driver"), by_type(second, "driver")
    for name in sorted(da.keys() | db.keys()):
        a, b = da.get(name), db.get(name)
        if a is None or b is None:
            result.drivers.append(name)
            continue
        changed = [k for k in DRIVER_KEYS if a.text(k) != b.text(k)]
        if changed:
            result.drivers.append(f"{name} ({', '.join(changed)})")

    def queued(entries):
        return {(e.name, e.text("sibling", "0")) for e in entries if e.type == "event"}
    result.events = sorted(f"{n}" for n, _ in queued(first) ^ queued(second))
    return result


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("first")
    ap.add_argument("second")
    ap.add_argument("--exact", action="store_true", help="list every key whose text differs")
    args = ap.parse_args()
    first, second = load(args.first), load(args.second)
    if args.exact:
        lines = exact_differences(first, second)
        print("\n".join(lines) if lines else "the same")
        return 1 if lines else 0
    print("\n".join(compare(first, second).lines()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
