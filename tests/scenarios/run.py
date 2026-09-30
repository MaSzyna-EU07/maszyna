#!/usr/bin/env python3
"""Scenario tests: run the real simulator on a scenery and check what a vehicle did.

A case (cases/<name>.json) names a base scenery from the game install, a fragment
appended to it (usually a trainset with an AI driver), and checks on the vehicle's
physics log. The runner:

  1. builds scenery/autotest_<case>.scn in the game dir: the base scenery with its own
     trainsets removed, optionally a different start time, and the case fragment appended,
  2. starts eu07 with that scenery in ghostview (every train driven by AI) and an extra
     settings file (-ini) that turns on physicslog and runs in a window,
  3. waits until the vehicle's physicslog covers `duration` seconds of simulation,
     then closes the simulator,
  4. evaluates the checks on the log and prints PASS/FAIL per check.

Scenery and vehicle data are not part of this repo, so this runs against a local game
install only (--game-dir or EU07_GAME_DIR), not in CI.

Usage:
  python tests/scenarios/run.py --game-dir "E:/.../MaSzyna"            # all cases
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
from dataclasses import dataclass, field
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parents[1]
CASES_DIR = HERE / "cases"
OUT_DIR = HERE / "out"

# physicslog columns, see TController::PhysicsLog() in vehicle/Driver.cpp.
# Velocity is in km/h and pressures in bar, despite the header saying m/s and MPa.
COL_TIME = 0
COL_VELOCITY = 1
COL_BRAKE_PRESS = 8

# settings layered over the user's eu07.ini for a test run
TEST_INI = """\
fullscreen no
fullscreenwindowed no
width 1280
height 720
inactivepause no
soundenabled no
physicslog yes
multiplelogs no
timespeed {timespeed}
"""


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


def build_scenery(case: dict, game_dir: Path, case_name: str) -> str:
    """Writes the case scenery into the game's scenery dir and returns its file name."""
    scenery_dir = game_dir / "scenery"
    # sceneries are cp1250; latin-1 round-trips every byte, so nothing gets mangled
    text = (scenery_dir / case["base_scenery"]).read_bytes().decode("latin-1")
    if case.get("strip_trainsets", True):
        text, _ = split_trainsets(text)
    if "time" in case:
        text = set_start_time(text, case["time"])
    fragment = (CASES_DIR / case["fragment"]).read_bytes().decode("latin-1")
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
    return name


# ---------------------------------------------------------------- run


def read_physicslog(path: Path) -> list[list[float]]:
    """Returns rows of numeric columns up to BrakePress; skips the header and partial lines."""
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
            rows.append([float(p) for p in parts[: COL_BRAKE_PRESS + 1]])
        except ValueError:
            continue
    return rows


@dataclass
class RunResult:
    rows: list[list[float]] = field(default_factory=list)
    exit_code: int | None = None  # None: still running when we closed it (the normal end)
    timed_out: bool = False
    wall_seconds: float = 0.0


def run_simulator(exe: Path, game_dir: Path, scenery: str, ini: Path, physicslog: Path,
                  duration: float, timeout: float) -> RunResult:
    physicslog.unlink(missing_ok=True)
    cmd = [str(exe), "-s", scenery, "-v", "ghostview", "-ini", str(ini)]
    print("  $", " ".join(f'"{c}"' if " " in c else c for c in cmd))
    started = time.monotonic()
    proc = subprocess.Popen(cmd, cwd=game_dir)
    result = RunResult()
    last_report = 0.0
    try:
        while True:
            time.sleep(0.25)  # at timespeed 20 that is 5 s of simulation
            result.rows = read_physicslog(physicslog)
            sim_t = result.rows[-1][COL_TIME] if result.rows else 0.0
            wall = time.monotonic() - started
            if wall - last_report >= 15:
                v = result.rows[-1][COL_VELOCITY] if result.rows else 0.0
                print(f"  ... {wall:5.0f}s wall, sim {sim_t:6.1f}s, v={v:6.1f} km/h")
                last_report = wall
            if proc.poll() is not None:
                result.exit_code = proc.returncode
                break
            if sim_t >= duration:
                break
            if wall > timeout:
                result.timed_out = True
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
    stopped_at = None
    for r in rows[start:]:
        if r[COL_VELOCITY] <= stop_speed:
            if stopped_at is None:
                stopped_at = r[COL_TIME]
            if r[COL_TIME] - stopped_at >= hold:
                return True, f"came to a stop at t={stopped_at:.1f}s"
        else:
            stopped_at = None
    return False, f"did not stay below {stop_speed} km/h for {hold}s after moving off"


CHECKS = {
    "starts": check_starts,
    "max_speed": check_max_speed,
    "brakes": check_brakes,
    "stops": check_stops,
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


def run_case(name: str, exe: Path, game_dir: Path, timespeed: float) -> bool:
    case = json.loads((CASES_DIR / f"{name}.json").read_text(encoding="utf-8"))
    print(f"\n=== {name}: {case.get('description', '')}")
    out = OUT_DIR / name
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)

    scenery = build_scenery(case, game_dir, name)
    ini = out / "autotest.ini"
    ini.write_text(TEST_INI.format(timespeed=timespeed), encoding="ascii")
    physicslog = game_dir / "physicslog" / f"{case['vehicle']}.dat"

    result = run_simulator(exe, game_dir, scenery, ini, physicslog,
                           float(case["duration"]), float(case.get("timeout", 600)))

    for f in ("log.txt", "errors.txt"):
        if (game_dir / f).exists():
            shutil.copy2(game_dir / f, out / f)
    if physicslog.exists():
        shutil.copy2(physicslog, out / physicslog.name)

    ok = True
    sim_t = result.rows[-1][COL_TIME] if result.rows else 0.0
    print(f"  ran {result.wall_seconds:.0f}s wall, {sim_t:.1f}s of physicslog, results in {out.relative_to(REPO)}")
    if result.exit_code is not None:
        print(f"  FAIL  simulator exited on its own with code {result.exit_code} (crash?)")
        ok = False
    if result.timed_out:
        print(f"  FAIL  timed out before {case['duration']}s of simulation")
        ok = False
    if not result.rows:
        print(f"  FAIL  no physicslog for '{case['vehicle']}' ({physicslog})")
        return False

    for check in case["checks"]:
        params = {k: v for k, v in check.items() if k != "type"}
        passed, detail = CHECKS[check["type"]](result.rows, **params)
        print(f"  {'PASS' if passed else 'FAIL'}  {check['type']}: {detail}")
        ok &= passed

    errors = count_lines(out / "errors.txt")
    if errors:
        print(f"  note  errors.txt has {errors} line(s)")
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
    args = ap.parse_args()

    if not args.game_dir:
        sys.exit("pass --game-dir or set EU07_GAME_DIR")
    game_dir = Path(args.game_dir).resolve()
    exe = find_exe(args.exe)
    print(f"exe:  {exe}\ngame: {game_dir}")

    names = args.cases or sorted(p.stem for p in CASES_DIR.glob("*.json"))
    results = {n: run_case(n, exe, game_dir, args.timespeed) for n in names}

    print("\n=== summary")
    for n, ok in results.items():
        print(f"  {'PASS' if ok else 'FAIL'}  {n}")
    return 0 if all(results.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
