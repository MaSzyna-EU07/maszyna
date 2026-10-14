#!/usr/bin/env python3
"""Scenario tests: run the real simulator on a scenery and check what a vehicle did.

A case (cases/<name>.json) names a base scenery from the game install, a fragment
added to it (usually a trainset with an AI driver), and checks on the vehicle's
physics log. The runner:

  1. builds scenery/autotest_<case>.scn in the game dir: the base scenery with its own
     trainsets removed, optionally a different start time, and the case fragment added,
  2. starts eu07 with that scenery in ghostview (every train driven by AI) and an extra
     settings file (-ini) that turns on physicslog and runs in a window,
  3. waits until the vehicle's physicslog covers `duration` seconds of simulation,
     then closes the simulator,
  4. evaluates the checks on the log and prints PASS/FAIL per check.

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

# settings layered over the user's eu07.ini for a test run
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
    fragment = (CASES_DIR / case["fragment"]).read_bytes().decode("latin-1")
    # vars first: {{vehicle}} inside a var's value gets filled in the second pass
    fragment = fill_vars(fragment, case.get("vars", {}), keep={"vehicle"})
    fragment = fill_vars(fragment, {"vehicle": vehicle})
    # FirstInit binds event launchers to their events and links the tracks. so launchers and
    # events of the case go in before it (read after it they never fire), trainsets after it
    # (placed before it they stand on unlinked track and derail at once)
    before, trainsets = split_trainsets(fragment)
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
    cache = scenery_dir / f"autotest_{case_name}.sbt"
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


@dataclass
class RunResult:
    rows: list[list[float]] = field(default_factory=list)
    exit_code: int | None = None  # None: still running when we closed it (the normal end)
    failure: str = ""  # why the run was cut short
    wall_seconds: float = 0.0
    logs: dict[str, Path] = field(default_factory=dict)


def run_simulator(exe: Path, game_dir: Path, scenery: str, ini: Path, physicslog: Path,
                  duration: float, timeout: float, say) -> RunResult:
    physicslog.unlink(missing_ok=True)
    cmd = [str(exe), "-s", scenery, "-v", "ghostview", "-ini", str(ini)]
    say("  $ " + " ".join(f'"{c}"' if " " in c else c for c in cmd), progress=True)
    since = time.time() - 1
    started = time.monotonic()
    proc = subprocess.Popen(cmd, cwd=game_dir)
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
            if sim_t >= duration:
                break
            if wall > timeout:
                result.failure = f"timed out after {timeout:.0f}s before {duration}s of simulation"
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


def check_stops(rows, moving_speed, stop_speed, hold):
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
                return True, f"came to a stop at t={rows[stopped_at][COL_TIME]:.1f}s after {dist[stopped_at]:.0f} m"
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


def run_case(name: str, exe: Path, game_dir: Path, timespeed: float, say) -> bool:
    case = json.loads((CASES_DIR / f"{name}.json").read_text(encoding="utf-8"))
    say(f"\n=== {name}: {case.get('description', '')}")
    out = OUT_DIR / name
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)

    # the lead vehicle is named after the case, so parallel runs never share a physicslog
    vehicle = f"autotest_{name}"
    scenery = build_scenery(case, game_dir, name, vehicle)
    ini = out / "autotest.ini"
    ini.write_text(TEST_INI.format(timespeed=timespeed), encoding="ascii")
    physicslog = game_dir / "physicslog" / f"{vehicle}.dat"

    result = run_simulator(exe, game_dir, scenery, ini, physicslog,
                           float(case["duration"]), float(case.get("timeout", 600)), say)

    # the logs are ours alone (named after the case scenery), move them out of the game dir
    for kind, path in result.logs.items():
        shutil.move(path, out / f"{kind}.txt")
    if physicslog.exists():
        shutil.move(physicslog, out / physicslog.name)

    ok = True
    sim_t = result.rows[-1][COL_TIME] if result.rows else 0.0
    say(f"  ran {result.wall_seconds:.0f}s wall, {sim_t:.1f}s of physicslog, results in {out.relative_to(REPO)}")
    if result.exit_code is not None:
        say(f"  FAIL  simulator exited on its own with code {result.exit_code}")
        ok = False
    if result.failure:
        say(f"  FAIL  {result.failure}")
        ok = False
    if not result.rows:
        if not result.failure:
            say(f"  FAIL  no physicslog for '{vehicle}'")
        return False

    for check in case["checks"]:
        params = {k: v for k, v in check.items() if k != "type"}
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
    # 20 gave the same speed profile as 1 within a few km/h and seconds on eu07_td_timetable,
    # in 20 s instead of 4 min; 1 is for watching a run
    ap.add_argument("--timespeed", type=float, default=20.0,
                    help="simulation time multiplier, the simulator offers 1, 5, 20 and 60 (default 20)")
    ap.add_argument("-j", "--jobs", type=int, default=1,
                    help="cases run at once, each in its own simulator window (default 1)")
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
            results[n] = run_case(n, exe, game_dir, args.timespeed, say)
    else:
        # each case prints its block when it is done, so parallel output does not interleave
        def job(n):
            lines = []
            def say(line, progress=False):
                if not progress:
                    lines.append(line)
            ok = run_case(n, exe, game_dir, args.timespeed, say)
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
