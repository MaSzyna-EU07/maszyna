#!/usr/bin/env python3
"""Scenario tests: run the real simulator on a scenery and check what a vehicle did.

A case (cases/<name>.json) names a base scenery from the game install, a fragment
added to it (usually a trainset with an AI driver), the result the simulator's test
mode should end with, and checks on the vehicle's physics log. The runner:

  1. builds scenery/autotest_<case>.scn in the game dir: the base scenery with its own
     trainsets removed, optionally a different start time, and the case fragment added,
  2. starts eu07 in test mode (--test: every train driven by AI, nothing drawn, fixed
     time steps as fast as it can) with that scenery and an extra settings file (-ini)
     that turns on physicslog,
  3. waits until the simulator quits by itself and reads the test result from its exit
     code: pass, fail, signal passed at danger, timeout (after `duration` seconds of
     simulation, if the case gives one), standstill or error,
  4. compares the result with the one the case expects, evaluates the checks on the
     physicslog and prints PASS/FAIL per check.

Scenery and vehicle data are not part of this repo, so this runs against a local game
install only (--game-dir or EU07_GAME_DIR), not in CI. See README.md.

Usage:
  python tests/scenarios/run.py --game-dir "E:/.../MaSzyna" -j 4       # all cases
  python tests/scenarios/run.py --game-dir ... eu07_td_timetable       # chosen cases
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CASES_DIR = HERE / "cases"
OUT_DIR = HERE / "out"

# physicslog columns, see TController::PhysicsLog() in vehicle/Driver.cpp.
# Velocity is in km/h and pressures in bar, despite the header saying m/s and MPa.
COL_TIME = 0
COL_VELOCITY = 1  # wheel speed: stays at 0 while the wheels are locked in a skid
COL_BRAKE_PRESS = 8
COL_COMMAND = 16  # text, read as None
COL_WHEELSLIP = 20
COL_VEHICLE_VELOCITY = 25  # signed vehicle speed, km/h; only in builds which log it

# exit codes of the simulator's test mode (eu07 --test), see simulation/simulationtestrules.h
EXIT_RESULTS = {
    0: "pass",
    10: "fail",
    11: "signal_passed_at_danger",
    12: "timeout",
    13: "standstill",
    14: "error",
}
# the timeout of the simulator's real time watchdog (test.wall_timeout) has the timeout code too; told
# apart by its verdict, so a run that hung doesn't pass a case that expects the simulation time to run out
WALL_TIMEOUT = "wall_timeout"
RESULTS = (*EXIT_RESULTS.values(), WALL_TIMEOUT)

# settings layered over the game's eu07.ini for a test run (test mode doesn't read the player's own
# in %APPDATA%); the window ones only matter
# with --watch, test mode keeps its window hidden and turns the sound off by itself
TEST_INI = """\
fullscreen no
fullscreenwindowed no
width 1280
height 720
inactivepause no
soundenabled no
physicslog yes
multiplelogs yes
timespeed {timespeed}
"""
# multiplelogs: log and errors go to logs/log (<scenery>) <date>.txt instead of the shared
# log.txt, so runs of different cases can go in parallel


# ---------------------------------------------------------------- scenery


def split_trainsets(text: str) -> tuple[str, str]:
    """Splits text into (everything else, the trainset ... endtrainset blocks)."""
    rest, trainsets = [], []
    inside = False
    for line in text.splitlines(keepends=True):
        tokens = line.split("//", 1)[0].split()
        if not inside and tokens and tokens[0].lower() == "trainset":
            inside = True
        if inside:
            trainsets.append(line)
            if any(t.lower() == "endtrainset" for t in tokens):
                inside = False
            continue
        rest.append(line)
    return "".join(rest), "".join(trainsets)


def set_start_time(text: str, hhmm: str) -> str:
    return re.sub(r"(?im)^(\s*time\s+)\d{1,2}:\d{2}", lambda m: m.group(1) + hhmm, text, count=1)


def add_track_events(text: str, events: dict) -> str:
    """Adds lines (e.g. "event0 test_pass") to tracks named in events, before their endtrack."""
    for track, lines in events.items():
        lines = [lines] if isinstance(lines, str) else lines
        m = re.search(rf"(?im)^\s*node\s+\S+\s+\S+\s+{re.escape(track)}\s+track\b.*?^(\s*endtrack\b)", text, re.S)
        if not m:
            sys.exit(f"track_events: no track {track} in the base scenery itself (not in its includes)")
        text = text[: m.start(1)] + "".join(line + "\n" for line in lines) + text[m.start(1):]
    return text


def set_track_friction(text: str, friction: float) -> str:
    """Sets the friction of every track: node <range> <range> <name> track <type> <length> <gauge> <friction> ..."""
    return re.sub(r"(?im)^(\s*node\s+\S+\s+\S+\s+\S+\s+track\s+\S+\s+\S+\s+\S+\s+)[-\d.]+",
                  lambda m: m.group(1) + f"{friction}", text)


def fill_vars(fragment: str, values: dict, keep=frozenset()) -> str:
    """Replaces {{name}} with the case's value; a list value becomes one line per item."""
    def value(m):
        if m.group(1) in keep:
            return m.group(0)
        v = values.get(m.group(1))
        if v is None:
            sys.exit(f"fragment uses {{{{{m.group(1)}}}}} but the case sets no such var")
        return "\n".join(v) if isinstance(v, list) else str(v)
    return re.sub(r"\{\{(\w+)\}\}", value, fragment)


def scenery_cache(game_dir: Path, case_name: str) -> Path:
    """The simulator's geometry cache of the case scenery."""
    return game_dir / "scenery" / f"autotest_{case_name}.sbt"


def build_scenery(case: dict, game_dir: Path, case_name: str, vehicle: str) -> str:
    """Writes the case scenery into the game's scenery dir and returns its file name."""
    scenery_dir = game_dir / "scenery"
    # sceneries are cp1250; latin-1 round-trips every byte, so nothing gets mangled
    text = (scenery_dir / case["base_scenery"]).read_bytes().decode("latin-1")
    if case.get("strip_trainsets", True):
        text, _ = split_trainsets(text)
    if "time" in case:
        text = set_start_time(text, case["time"])
    if "track_friction" in case:
        text = set_track_friction(text, case["track_friction"])
    if "track_events" in case:
        text = add_track_events(text, case["track_events"])
    fragment = (CASES_DIR / case["fragment"]).read_bytes().decode("latin-1")
    # vars first: {{vehicle}} inside a var's value gets filled in the second pass
    fragment = fill_vars(fragment, case.get("vars", {}), keep={"vehicle"})
    fragment = fill_vars(fragment, {"vehicle": vehicle})
    # FirstInit binds event launchers to their events and links the tracks. so launchers and
    # events of the case go in before it (read after it they never fire), trainsets after it
    # (placed before it they stand on unlinked track and derail at once)
    before, trainsets = split_trainsets(fragment)
    # settings of the test mode, read like any other config block: the case's vehicle is the train under test,
    # whose consist the pass and the standstill wait for, and the case's own (test.timeout and so on)
    config = [f"test.vehicle {vehicle}", *case.get("test_config", [])]
    before = "config\n" + "".join(line + "\n" for line in config) + "endconfig\n" + before
    header = f"\n// ---- autotest: {case_name} ----\n"
    firstinit = re.search(r"(?im)^\s*firstinit\b", text)
    if firstinit:
        text = text[: firstinit.start()] + header + before + "\n" + text[firstinit.start():]
    else:
        text += header + before + "\n"
    text += header + trainsets + "\n"
    name = f"autotest_{case_name}.scn"
    (scenery_dir / name).write_bytes(text.encode("latin-1"))
    # the simulator caches the scenery's static geometry in <name>.sbt and reads it instead of
    # the terrain in the .scn (tracks, events and trainsets always come from the .scn), so
    # drop the cache once the base scenery has changed
    cache = scenery_cache(game_dir, case_name)
    if cache.exists() and cache.stat().st_mtime < (scenery_dir / case["base_scenery"]).stat().st_mtime:
        cache.unlink()
    return name


# ---------------------------------------------------------------- run


def read_physicslog(path: Path) -> list[list[float]]:
    """Returns rows of columns, None for the text ones; skips the header and partial lines."""
    rows = []
    try:
        lines = path.read_text(errors="replace").splitlines()
    except FileNotFoundError:
        return rows
    for line in lines[1:]:
        parts = line.split()
        if len(parts) <= COL_BRAKE_PRESS:
            continue  # line still being written
        try:
            row = [float(p) for p in parts[: COL_BRAKE_PRESS + 1]]
        except ValueError:
            continue
        for p in parts[COL_BRAKE_PRESS + 1:]:
            try:
                row.append(float(p))
            except ValueError:
                row.append(None)
        rows.append(row)
    return rows


def find_logs(game_dir: Path, scenery: str, since: float) -> dict[str, Path]:
    """The log.txt / errors.txt of one run; with multiplelogs each scenery gets its own files."""
    stem = scenery.rsplit(".", 1)[0]
    found = {}
    for kind in ("log", "errors"):
        files = [p for p in (game_dir / "logs").glob(f"{kind} ({stem}) *.txt") if p.stat().st_mtime >= since]
        if files:
            found[kind] = max(files, key=lambda p: p.stat().st_mtime)
    return found


def read_text(path: Path | None) -> str:
    try:
        return path.read_text(errors="replace") if path else ""
    except FileNotFoundError:
        return ""


# load problems that leave the case without its vehicle; no point waiting for the timeout
FATAL_LOG_LINES = ("Bad vehicle:", "Bad trainset:")
# wall seconds from start to the first physicslog row before giving up. "Scenario loading
# time" in the log is not the end of loading (the trainsets after FirstInit still follow),
# and with several simulators loading at once the first rows took well over 15 s
STARTUP_TIMEOUT = 120.0
# wall seconds past a case's timeout before the runner kills a simulator that didn't end itself
KILL_GRACE = 30.0


@dataclass
class RunResult:
    rows: list[list[float]] = field(default_factory=list)
    exit_code: int | None = None  # None: still running when we closed it (the normal end with --watch)
    verdict: str = ""  # the simulator's own line about the test result
    failure: str = ""  # why the run was cut short
    wall_seconds: float = 0.0
    logs: dict[str, Path] = field(default_factory=dict)


def run_simulator(exe: Path, game_dir: Path, scenery: str, ini: Path, physicslog: Path,
                  duration: float | None, timeout: float, watch: bool, say) -> RunResult:
    """Runs the simulator on the scenery. In test mode it quits by itself and the exit code
    is the result; with watch it runs in a window and gets closed after `duration`."""
    physicslog.unlink(missing_ok=True)
    if watch:
        cmd = [str(exe), "-s", scenery, "-v", "ghostview", "-ini", str(ini)]
    else:
        # the simulator's own watchdog ends a run that takes too long, with the timeout code and its verdict
        cmd = [str(exe), "--test", "-s", scenery, "-ini", str(ini), "--test-wall-timeout", f"{timeout:g}"]
        if duration is not None:
            cmd += ["--test-timeout", f"{duration:g}"]
    say("  $ " + " ".join(f'"{c}"' if " " in c else c for c in cmd), progress=True)
    since = time.time() - 1
    started = time.monotonic()
    # the test result line goes to stdout; a file, not a pipe, so a chatty run can't block on it
    stdout = (ini.parent / "stdout.txt").open("w+b")
    proc = subprocess.Popen(cmd, cwd=game_dir, stdout=stdout, stderr=subprocess.STDOUT)
    result = RunResult()
    last_report = 0.0
    try:
        while True:
            time.sleep(0.25)  # at timespeed 20 that is 5 s of simulation
            wall = time.monotonic() - started
            result.rows = read_physicslog(physicslog)
            sim_t = result.rows[-1][COL_TIME] if result.rows else 0.0
            if wall - last_report >= 15:
                v = result.rows[-1][COL_VELOCITY] if result.rows else 0.0
                say(f"  ... {wall:5.0f}s wall, sim {sim_t:6.1f}s, v={v:6.1f} km/h", progress=True)
                last_report = wall
            if proc.poll() is not None:
                result.exit_code = proc.returncode
                break
            if watch and sim_t >= duration:
                break
            # in test mode a little later than the simulator's own limit, for a run that can't even end itself
            if wall > timeout + (0 if watch else KILL_GRACE):
                result.failure = f"timed out after {wall:.0f}s before {duration}s of simulation"
                break
            if not result.rows:
                result.logs = find_logs(game_dir, scenery, since)
                fatal = [l.strip() for l in read_text(result.logs.get("errors")).splitlines()
                         if any(k in l for k in FATAL_LOG_LINES)]
                if fatal:
                    result.failure = "vehicle failed to load: " + " | ".join(fatal)
                    break
                if wall > STARTUP_TIMEOUT:
                    result.failure = f"no physicslog rows after {STARTUP_TIMEOUT:.0f}s"
                    break
    finally:
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
        result.wall_seconds = time.monotonic() - started
        stdout.seek(0)
        result.verdict = next((line.strip() for line in stdout.read().decode(errors="replace").splitlines()
                               if line.startswith("test:")), "")
        stdout.close()
    result.rows = read_physicslog(physicslog)
    result.logs = find_logs(game_dir, scenery, since)
    return result


# ---------------------------------------------------------------- checks


def check_starts(rows, min_speed, within):
    for r in rows:
        if r[COL_TIME] > within:
            break
        if r[COL_VELOCITY] >= min_speed:
            return True, f"reached {min_speed} km/h at t={r[COL_TIME]:.1f}s"
    top = max((r[COL_VELOCITY] for r in rows if r[COL_TIME] <= within), default=0.0)
    return False, f"top speed in the first {within}s was {top:.1f} km/h, wanted {min_speed}"


def check_max_speed(rows, limit, tolerance=0.0):
    worst = max(rows, key=lambda r: r[COL_VELOCITY])
    v, t = worst[COL_VELOCITY], worst[COL_TIME]
    ok = v <= limit + tolerance
    return ok, f"top speed {v:.1f} km/h at t={t:.1f}s (limit {limit} +{tolerance})"


def _first_moving(rows, moving_speed):
    return next((i for i, r in enumerate(rows) if r[COL_VELOCITY] >= moving_speed), None)


def _distances(rows):
    """Metres travelled up to each row, from the speed; the physicslog has no position."""
    out, d = [0.0], 0.0
    for a, b in zip(rows, rows[1:]):
        d += (a[COL_VELOCITY] + b[COL_VELOCITY]) / 2 / 3.6 * (b[COL_TIME] - a[COL_TIME])
        out.append(d)
    return out


def check_brakes(rows, min_pressure, moving_speed=20):
    start = _first_moving(rows, moving_speed)
    if start is None:
        return False, "never moved, so never braked"
    for r in rows[start:]:
        if r[COL_BRAKE_PRESS] >= min_pressure:
            return True, f"brake cylinder {r[COL_BRAKE_PRESS]:.2f} bar at t={r[COL_TIME]:.1f}s, v={r[COL_VELOCITY]:.1f} km/h"
    return False, f"brake cylinder pressure never reached {min_pressure} bar after moving off"


def check_stops(rows, moving_speed, stop_speed, hold, min_distance=None, max_distance=None):
    start = _first_moving(rows, moving_speed)
    if start is None:
        return False, f"never reached {moving_speed} km/h"
    dist = _distances(rows)
    stopped_at = None
    for i in range(start, len(rows)):
        r = rows[i]
        if r[COL_VELOCITY] <= stop_speed:
            if stopped_at is None:
                stopped_at = i
            if r[COL_TIME] - rows[stopped_at][COL_TIME] >= hold:
                where = dist[stopped_at]
                detail = f"came to a stop at t={rows[stopped_at][COL_TIME]:.1f}s after {where:.0f} m"
                if (min_distance is not None and where < min_distance) or (max_distance is not None and where > max_distance):
                    return False, f"{detail}, wanted between {min_distance} and {max_distance} m"
                return True, detail
        else:
            stopped_at = None
    return False, f"did not stay below {stop_speed} km/h for {hold}s after moving off"


def check_stays_stopped(rows, moving_speed, stop_speed):
    """Once stopped after moving off, never moves again (e.g. waiting at a red signal)."""
    start = _first_moving(rows, moving_speed)
    if start is None:
        return False, f"never reached {moving_speed} km/h"
    stop = next((i for i in range(start, len(rows)) if rows[i][COL_VELOCITY] <= stop_speed), None)
    if stop is None:
        return False, f"never came down to {stop_speed} km/h"
    moved = next((r for r in rows[stop:] if r[COL_VELOCITY] > stop_speed), None)
    if moved is not None:
        return False, f"stopped at t={rows[stop][COL_TIME]:.1f}s but moved again at t={moved[COL_TIME]:.1f}s, v={moved[COL_VELOCITY]:.1f} km/h"
    return True, f"stopped at t={rows[stop][COL_TIME]:.1f}s and stayed until t={rows[-1][COL_TIME]:.1f}s"


def check_distance(rows, max=None, min=None):
    """Total distance travelled, in metres."""
    d = _distances(rows)[-1]
    ok = (max is None or d <= max) and (min is None or d >= min)
    bounds = " ".join(s for s in (f">= {min}" if min is not None else "", f"<= {max}" if max is not None else "") if s)
    return ok, f"travelled {d:.0f} m (wanted {bounds})"


def check_slips(rows, within=None):
    """The lead vehicle's wheels slip (or lock in a skid) at some point, e.g. to make sure a slippery rail case does what it's meant to."""
    for r in rows:
        if within is not None and r[COL_TIME] > within:
            break
        if len(r) > COL_WHEELSLIP and r[COL_WHEELSLIP]:
            return True, f"wheels slipping at t={r[COL_TIME]:.1f}s, v={r[COL_VELOCITY]:.1f} km/h"
    return False, "the wheels never slipped"


def check_holds_still(rows, moving_speed, stop_speed, max_distance):
    """Once stopped after moving off, the vehicle doesn't creep. Uses the vehicle speed rather than the
    wheel speed, so it also catches a vehicle sliding on locked wheels."""
    if not rows or len(rows[0]) <= COL_VEHICLE_VELOCITY or rows[0][COL_VEHICLE_VELOCITY] is None:
        return False, "the physicslog has no vehicle speed column, the build is too old"
    speed = [abs(r[COL_VEHICLE_VELOCITY]) for r in rows]
    start = next((i for i, v in enumerate(speed) if v >= moving_speed), None)
    if start is None:
        return False, f"never reached {moving_speed} km/h"
    stop = next((i for i in range(start, len(rows)) if speed[i] <= stop_speed), None)
    if stop is None:
        return False, f"never came down to {stop_speed} km/h"
    d, top, top_t = 0.0, 0.0, rows[stop][COL_TIME]
    for i in range(stop + 1, len(rows)):
        d += (speed[i - 1] + speed[i]) / 2 / 3.6 * (rows[i][COL_TIME] - rows[i - 1][COL_TIME])
        if speed[i] > top:
            top, top_t = speed[i], rows[i][COL_TIME]
    held = rows[-1][COL_TIME] - rows[stop][COL_TIME]
    detail = f"stopped at t={rows[stop][COL_TIME]:.1f}s, then moved {d:.3f} m in {held:.0f}s, top {top:.4f} km/h at t={top_t:.1f}s"
    return d <= max_distance, detail


def check_ends_moving(rows, min_speed):
    """The run ended with the vehicle still moving, e.g. a pass that didn't wait for the trains to stop."""
    if not rows:
        return False, "the physicslog is empty"
    last = rows[-1]
    ok = last[COL_VELOCITY] >= min_speed
    return ok, f"last row t={last[COL_TIME]:.1f}s, v={last[COL_VELOCITY]:.1f} km/h (wanted >= {min_speed})"


def check_slip_clears(rows, moving_speed, stop_speed, within):
    """Once the vehicle has stopped after moving off, its wheels stop slipping within `within` seconds
    (a vehicle left skidding on locked wheels keeps its brakes capped by the sliding adhesion)."""
    if not rows or len(rows[0]) <= COL_VEHICLE_VELOCITY or rows[0][COL_VEHICLE_VELOCITY] is None:
        return False, "the physicslog has no vehicle speed column, the build is too old"
    start = next((i for i, r in enumerate(rows) if abs(r[COL_VEHICLE_VELOCITY]) >= moving_speed), None)
    if start is None:
        return False, f"never reached {moving_speed} km/h"
    stop = next((i for i in range(start, len(rows)) if abs(rows[i][COL_VEHICLE_VELOCITY]) <= stop_speed), None)
    if stop is None:
        return False, f"never came down to {stop_speed} km/h"
    deadline = rows[stop][COL_TIME] + within
    late = [r for r in rows[stop:] if r[COL_TIME] > deadline and r[COL_WHEELSLIP]]
    if late:
        return False, f"stopped at t={rows[stop][COL_TIME]:.1f}s, wheels still slipping at t={late[0][COL_TIME]:.1f}s ({len(late)} rows after the {within}s grace)"
    return True, f"stopped at t={rows[stop][COL_TIME]:.1f}s, no slipping after t={deadline:.1f}s"


CHECKS = {
    "slips": check_slips,
    "slip_clears": check_slip_clears,
    "holds_still": check_holds_still,
    "starts": check_starts,
    "max_speed": check_max_speed,
    "brakes": check_brakes,
    "stops": check_stops,
    "stays_stopped": check_stays_stopped,
    "distance": check_distance,
    "ends_moving": check_ends_moving,
}


def check_log_contains(log, pattern):
    """A line of the run's log.txt matches the regex, e.g. the "Bad driving" message of a collision."""
    found = next((line.strip() for line in log.splitlines() if re.search(pattern, line)), None)
    if found is None:
        return False, f"no log line matches /{pattern}/"
    return True, f"log: {found}"


def check_log_lacks(log, pattern):
    """No line of the run's log.txt matches the regex."""
    found = [line.strip() for line in log.splitlines() if re.search(pattern, line)]
    if found:
        return False, f"{len(found)} log line(s) match /{pattern}/, first: {found[0]}"
    return True, f"no log line matches /{pattern}/"


# checks on the text of the run's log.txt rather than on the physicslog rows
LOG_CHECKS = {
    "log_contains": check_log_contains,
    "log_lacks": check_log_lacks,
}


# ---------------------------------------------------------------- main


def find_exe(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit).resolve()
    candidates = sorted(REPO.glob("cmake-build-*/bin/Release/eu07*.exe"), key=lambda p: p.stat().st_mtime)
    if not candidates:
        sys.exit("no eu07 executable found under cmake-build-*/bin/Release, pass --exe")
    return candidates[-1]


def count_lines(path: Path) -> int:
    try:
        return sum(1 for line in path.read_text(errors="replace").splitlines() if line.strip())
    except FileNotFoundError:
        return 0


def describe_exit(code: int | None) -> str:
    if code is None:
        return "none, still running"
    name = EXIT_RESULTS.get(code)
    return f"{code} ({name})" if name else f"{code} (0x{code & 0xFFFFFFFF:08X}, a crash?)"


def run_case(name: str, exe: Path, game_dir: Path, timespeed: float, watch: bool, say) -> bool:
    case = json.loads((CASES_DIR / f"{name}.json").read_text(encoding="utf-8"))
    say(f"\n=== {name}: {case.get('description', '')}")
    out = OUT_DIR / name
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)

    # the lead vehicle is named after the case, so parallel runs never share a physicslog
    vehicle = f"autotest_{name}"
    scenery = build_scenery(case, game_dir, name, vehicle)
    ini = out / "autotest.ini"
    # the case's own settings go last, over the common ones
    ini.write_text(TEST_INI.format(timespeed=timespeed) + "".join(line + "\n" for line in case.get("ini", [])),
                   encoding="ascii")
    physicslog = game_dir / "physicslog" / f"{vehicle}.dat"

    duration = float(case["duration"]) if "duration" in case else None
    # a case that only records `duration` seconds ends on the test timeout; one that says
    # how the test ends (a pass event, a signal passed at danger...) expects that result
    expect = case.get("expect", "timeout" if duration is not None else "pass")
    if expect not in RESULTS:
        sys.exit(f"{name}: expect must be one of {', '.join(RESULTS)}")
    if watch and duration is None:
        sys.exit(f"{name}: --watch needs a case with duration")
    result = run_simulator(exe, game_dir, scenery, ini, physicslog,
                           duration, float(case.get("timeout", 600)), watch, say)

    # the logs are ours alone (named after the case scenery), move them out of the game dir
    for kind, path in result.logs.items():
        shutil.move(path, out / f"{kind}.txt")
    if physicslog.exists():
        shutil.move(physicslog, out / physicslog.name)
    # other trains of the case with a driver, named <something>_<vehicle>, log too
    for other in physicslog.parent.glob(f"*_{vehicle}.dat"):
        shutil.move(other, out / other.name)

    ok = True
    sim_t = result.rows[-1][COL_TIME] if result.rows else 0.0
    say(f"  ran {result.wall_seconds:.0f}s wall, {sim_t:.1f}s of physicslog, results in {out.relative_to(REPO)}")
    if watch:
        if result.exit_code is not None:
            say(f"  FAIL  simulator exited on its own with code {result.exit_code}")
            ok = False
    else:
        if result.verdict:
            say(f"  {result.verdict}")
        got = EXIT_RESULTS.get(result.exit_code) if result.exit_code is not None else None
        if got == "timeout" and "of real time" in result.verdict:
            got = WALL_TIMEOUT
        passed = got == expect
        watchdog = ", from the real time watchdog" if got == WALL_TIMEOUT else ""
        say(f"  {'PASS' if passed else 'FAIL'}  exit code {describe_exit(result.exit_code)}{watchdog}, expected {expect}")
        ok &= passed
    # a run cut short (the real time watchdog, an error, a crash, or closed by us) could have been writing
    # the scenery's geometry cache; the simulator checks only the header of it, so the next runs would read
    # a truncated one. it is written again on the next run
    if result.failure or (not watch and got in (None, "error", WALL_TIMEOUT)):
        scenery_cache(game_dir, name).unlink(missing_ok=True)
    if result.failure:
        say(f"  FAIL  {result.failure}")
        ok = False
    if not result.rows and case.get("checks"):
        if not result.failure:
            say(f"  FAIL  no physicslog for '{vehicle}'")
        return False

    log = read_text(out / "log.txt")
    for check in case.get("checks", []):
        params = {k: v for k, v in check.items() if k != "type"}
        if check["type"] in LOG_CHECKS:
            # with no log text a log check proves nothing, log_lacks would pass vacuously
            if log.strip():
                passed, detail = LOG_CHECKS[check["type"]](log, **params)
            else:
                passed, detail = False, "log.txt is missing or empty"
        else:
            passed, detail = CHECKS[check["type"]](result.rows, **params)
        say(f"  {'PASS' if passed else 'FAIL'}  {check['type']}: {detail}")
        ok &= passed

    errors = count_lines(out / "errors.txt")
    if errors:
        say(f"  note  errors.txt has {errors} line(s)")
    if case.get("fail_on_errors") and errors:
        ok = False
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases", nargs="*", help="case names from cases/ (default: all)")
    ap.add_argument("--game-dir", default=os.environ.get("EU07_GAME_DIR"),
                    help="game install with scenery/ and dynamic/ (or EU07_GAME_DIR)")
    ap.add_argument("--exe", help="simulator executable (default: newest cmake-build-*/bin/Release/eu07*.exe)")
    ap.add_argument("--watch", action="store_true",
                    help="run in a visible window in real time (see --timespeed) instead of the test mode, "
                         "closed after the case's duration; the exit code isn't checked")
    ap.add_argument("--timespeed", type=float, default=1.0,
                    help="with --watch: simulation time multiplier, the simulator offers 1, 5, 20 and 60 (default 1)")
    ap.add_argument("-j", "--jobs", type=int, default=1,
                    help="cases run at once, each in a simulator of its own (default 1)")
    args = ap.parse_args()

    if not args.game_dir:
        sys.exit("pass --game-dir or set EU07_GAME_DIR")
    game_dir = Path(args.game_dir).resolve()
    exe = find_exe(args.exe)
    print(f"exe:  {exe}\ngame: {game_dir}")

    names = args.cases or sorted(p.stem for p in CASES_DIR.glob("*.json"))
    missing = [n for n in names if not (CASES_DIR / f"{n}.json").exists()]
    if missing:
        sys.exit("no such case: " + ", ".join(missing))

    started = time.monotonic()
    results = {}
    if args.jobs <= 1:
        def say(line, progress=False):
            print(line, flush=True)
        for n in names:
            results[n] = run_case(n, exe, game_dir, args.timespeed, args.watch, say)
    else:
        # each case prints its block when it is done, so parallel output does not interleave
        def job(n):
            lines = []
            def say(line, progress=False):
                if not progress:
                    lines.append(line)
            ok = run_case(n, exe, game_dir, args.timespeed, args.watch, say)
            return n, ok, lines
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            for n in names:
                print(f"started {n}", flush=True)
            for fut in as_completed([pool.submit(job, n) for n in names]):
                n, ok, lines = fut.result()
                print("\n".join(lines), flush=True)
                results[n] = ok

    print(f"\n=== summary ({time.monotonic() - started:.0f}s)")
    for n in names:
        print(f"  {'PASS' if results[n] else 'FAIL'}  {n}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
