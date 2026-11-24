#!/usr/bin/env python3
"""Self-test scripts of the track editor: run the editor on a copy of a test scenery and check what its expectations say.

A case (cases/<name>.txt) is a script of the editor's self-test commands (application/editormodeselftest*.cpp), with
`expect` commands that log SELFTEST PASS or SELFTEST FAIL and make the editor end with a non-zero exit code when any of
them failed. For each case the runner:

  1. copies the fixture (scenery/autotest_editor.scn) into the game's scenery folder as autotest_editor_<case>.scn, and the
     scenarios of its map (scenery/autotest_editor.<part>.scn) as autotest_editor_<case>_<part>.scn, naming that copy,
  2. starts the editor (eu07 -edit <scenery>) with the script in EU07_EDITOR_SELFTEST, its window hidden, a settings
     file of its own (-ini) and the one given with --ini over it, a limit of wall time and of memory,
  3. reads the exit code and the SELFTEST lines of log.txt, and prints PASS or FAIL for the case,
  4. deletes the copy of the scenery and whatever the editor wrote next to it under its name.

A line `#> reopen` in a script ends the editor there (the part before it ends with quit) and starts it again on the
scenery as the part before saved it, for what has to be seen after a save and a load.

The editor has to be built with -DWITH_EDITOR_SELFTESTS=ON; a build without it runs no script and is stopped at the
time limit. The game data isn't in this repository, so this runs against a local game install only, not in CI.
One editor runs at a time. See README.md.

Usage:
  python tests/editor/run.py --game-dir C:/MaSzyna [--ini my.ini] [--exe build/bin/RelWithDebInfo/eu07-....exe] [cases...]
"""

from __future__ import annotations

import argparse
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
FIXTURE = HERE / "scenery" / "autotest_editor.scn"
# the scenarios of the map of the fixture, autotest_editor.<part>.scn: copied next to the copy of the fixture as
# autotest_editor_<case>_<part>.scn, the name of the fixture in them made the name of its copy
COMPANIONS = sorted((HERE / "scenery").glob("autotest_editor.*.scn"))
OUT_DIR = HERE / "out"

# a part of a script ends with quit before this line, the editor starts again on the scenery saved
REOPEN = "#> reopen"
# the exit code of a run whose expectations failed, editor_selftest::failed_exit_code
FAILED_EXIT_CODE = 2

# settings of the runner, read over the player's eu07.ini and under the file given with --ini: a small window, no sound,
# nothing written into the data of the game, log.txt in the game folder
RUNNER_INI = """\
fullscreen no
fullscreenwindowed no
width 640
height 480
inactivepause no
soundenabled no
convertmodels 0
multiplelogs no
debuglog 1
"""


@dataclass
class StageResult:
    exit_code: int | None = None
    failure: str = ""
    seconds: float = 0.0
    peak_mb: float = 0.0
    lines: list[str] = field(default_factory=list)  # the SELFTEST PASS / FAIL / result lines of its log


# ---------------------------------------------------------------- the process and its memory


if os.name == "nt":
    import ctypes
    from ctypes import wintypes

    class _MemoryCounters(ctypes.Structure):
        _fields_ = [("cb", wintypes.DWORD), ("PageFaultCount", wintypes.DWORD), ("PeakWorkingSetSize", ctypes.c_size_t),
                    ("WorkingSetSize", ctypes.c_size_t), ("QuotaPeakPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaPagedPoolUsage", ctypes.c_size_t), ("QuotaPeakNonPagedPoolUsage", ctypes.c_size_t),
                    ("QuotaNonPagedPoolUsage", ctypes.c_size_t), ("PagefileUsage", ctypes.c_size_t),
                    ("PeakPagefileUsage", ctypes.c_size_t), ("PrivateUsage", ctypes.c_size_t)]

    class _IoCounters(ctypes.Structure):
        _fields_ = [(name, ctypes.c_ulonglong) for name in ("ReadOperationCount", "WriteOperationCount", "OtherOperationCount",
                                                            "ReadTransferCount", "WriteTransferCount", "OtherTransferCount")]

    class _BasicLimits(ctypes.Structure):
        _fields_ = [("PerProcessUserTimeLimit", ctypes.c_longlong), ("PerJobUserTimeLimit", ctypes.c_longlong),
                    ("LimitFlags", wintypes.DWORD), ("MinimumWorkingSetSize", ctypes.c_size_t),
                    ("MaximumWorkingSetSize", ctypes.c_size_t), ("ActiveProcessLimit", wintypes.DWORD),
                    ("Affinity", ctypes.c_size_t), ("PriorityClass", wintypes.DWORD), ("SchedulingClass", wintypes.DWORD)]

    class _ExtendedLimits(ctypes.Structure):
        _fields_ = [("BasicLimitInformation", _BasicLimits), ("IoInfo", _IoCounters), ("ProcessMemoryLimit", ctypes.c_size_t),
                    ("JobMemoryLimit", ctypes.c_size_t), ("PeakProcessMemoryUsed", ctypes.c_size_t),
                    ("PeakJobMemoryUsed", ctypes.c_size_t)]

    _kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    _kernel32.CreateJobObjectW.restype = wintypes.HANDLE
    _kernel32.CreateJobObjectW.argtypes = [ctypes.c_void_p, wintypes.LPCWSTR]
    _kernel32.SetInformationJobObject.argtypes = [wintypes.HANDLE, ctypes.c_int, ctypes.c_void_p, wintypes.DWORD]
    _kernel32.AssignProcessToJobObject.argtypes = [wintypes.HANDLE, wintypes.HANDLE]
    _kernel32.K32GetProcessMemoryInfo.argtypes = [wintypes.HANDLE, ctypes.c_void_p, wintypes.DWORD]
    _kernel32.CloseHandle.argtypes = [wintypes.HANDLE]
    _JOB_EXTENDED_LIMITS = 9
    _JOB_LIMIT_PROCESS_MEMORY = 0x100
    _JOB_LIMIT_KILL_ON_JOB_CLOSE = 0x2000

    def guard_process(proc: subprocess.Popen, limit_mb: int):
        """A job object over the editor: its memory can't grow past the limit, and it ends with the runner. None if the
        system doesn't allow it (the polling of the memory still stops the editor then)."""
        job = _kernel32.CreateJobObjectW(None, None)
        if not job:
            return None
        limits = _ExtendedLimits()
        limits.BasicLimitInformation.LimitFlags = _JOB_LIMIT_PROCESS_MEMORY | _JOB_LIMIT_KILL_ON_JOB_CLOSE
        limits.ProcessMemoryLimit = limit_mb * 1024 * 1024
        if not _kernel32.SetInformationJobObject(job, _JOB_EXTENDED_LIMITS, ctypes.byref(limits), ctypes.sizeof(limits)) \
                or not _kernel32.AssignProcessToJobObject(job, int(proc._handle)):
            _kernel32.CloseHandle(job)
            return None
        return job

    def release_guard(job) -> None:
        if job:
            _kernel32.CloseHandle(job)

    def memory_mb(proc: subprocess.Popen) -> float:
        counters = _MemoryCounters()
        counters.cb = ctypes.sizeof(counters)
        if not _kernel32.K32GetProcessMemoryInfo(int(proc._handle), ctypes.byref(counters), counters.cb):
            return 0.0
        return counters.PrivateUsage / (1024 * 1024)
else:
    def guard_process(proc: subprocess.Popen, limit_mb: int):
        return None

    def release_guard(job) -> None:
        pass

    def memory_mb(proc: subprocess.Popen) -> float:
        try:
            for line in Path(f"/proc/{proc.pid}/status").read_text().splitlines():
                if line.startswith("VmRSS:"):
                    return int(line.split()[1]) / 1024
        except (OSError, ValueError):
            pass
        return 0.0


def run_editor(exe: Path, game_dir: Path, scenery: str, script: Path, inis: list[Path], timeout: float, limit_mb: int,
               show: bool) -> StageResult:
    """Runs the editor on the scenery with the script until it quits, or until the limits stop it."""
    # the settings files before the scenery: one naming a sceneryfile of its own would take its place otherwise
    cmd = [str(exe)]
    for ini in inis:
        cmd += ["-ini", str(ini)]
    cmd += ["-edit", scenery]
    env = dict(os.environ)
    env["EU07_EDITOR_SELFTEST"] = str(script)
    if show:
        env.pop("EU07_EDITOR_SELFTEST_HIDDEN", None)
    else:
        # the window is never shown nor focused, the computer stays free for whatever else it's used for
        env["EU07_EDITOR_SELFTEST_HIDDEN"] = "1"
    result = StageResult()
    started = time.monotonic()
    proc = subprocess.Popen(cmd, cwd=game_dir, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    job = guard_process(proc, limit_mb)
    try:
        while proc.poll() is None:
            time.sleep(0.25)
            used = memory_mb(proc)
            result.peak_mb = max(result.peak_mb, used)
            if used > limit_mb:
                result.failure = f"stopped at {used:.0f} MB, over the limit of {limit_mb} MB"
                break
            if time.monotonic() - started > timeout:
                result.failure = f"stopped after {timeout:.0f} s, the time limit"
                break
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait(timeout=30)
        release_guard(job)
    result.exit_code = proc.returncode
    result.seconds = time.monotonic() - started
    return result


# ---------------------------------------------------------------- the scripts, the log and the copies


def split_script(text: str) -> list[str]:
    """The parts of a script, each run in an editor of its own."""
    parts = re.split(rf"(?m)^{re.escape(REOPEN)}\s*$", text)
    return [part.strip("\r\n") + "\n" for part in parts]


def read_log(log: Path) -> str:
    try:
        return log.read_bytes().decode("utf-8", errors="replace")
    except FileNotFoundError:
        return ""


def selftest_lines(text: str) -> list[str]:
    """The lines of the expectations and the result in the log of the editor."""
    keep = ("SELFTEST PASS", "SELFTEST FAIL", "SELFTEST result")
    return [line.strip() for line in text.splitlines() if any(k in line for k in keep)]


def snapshot(folder: Path) -> dict[str, float]:
    """The entries of the folder with the times they were changed, to tell what a run added or changed."""
    return {entry.name: entry.stat().st_mtime for entry in os.scandir(folder)}


def clean_up(scenery_dir: Path, stem: str, before: dict[str, float], keep: bool, say) -> None:
    """Deletes the copy of the scenery and what the editor wrote under its name; says what else changed there."""
    after = snapshot(scenery_dir)
    for name in sorted(after):
        path = scenery_dir / name
        if stem.lower() in name.lower():
            if keep:
                say(f"  kept  {path}")
            elif path.is_dir():
                shutil.rmtree(path)
            else:
                path.unlink()
        elif name not in before:
            say(f"  note  the run added {path}, left in place")
        elif after[name] != before[name]:
            say(f"  note  the run changed {path}")


def describe_exit(code: int | None) -> str:
    if code is None:
        return "none"
    if code == FAILED_EXIT_CODE:
        return f"{code} (expectations failed)"
    if code == 1:
        return "1 (the start failed, or an error ended the editor)"
    return f"{code} (0x{code & 0xFFFFFFFF:08X}, a crash?)" if code else "0"


def run_case(name: str, source: Path, exe: Path, game_dir: Path, user_ini: Path | None, timeout: float, limit_mb: int, show: bool,
             keep: bool, say) -> tuple[bool, int, float]:
    """Runs the case; returns whether it passed, the number of its expectations and the seconds it took."""
    scenery_dir = game_dir / "scenery"
    stem = f"autotest_editor_{name}".lower()
    scenery = f"{stem}.scn"
    out = OUT_DIR / name
    shutil.rmtree(out, ignore_errors=True)
    out.mkdir(parents=True)
    runner_ini = out / "editor.ini"
    runner_ini.write_text(RUNNER_INI)
    inis = [runner_ini] + ([user_ini] if user_ini else [])
    parts = split_script(source.read_text(encoding="utf-8"))
    before = snapshot(scenery_dir)
    if scenery in (entry.lower() for entry in before):
        sys.exit(f"{scenery_dir / scenery} is there already: a run before this one left it, delete it first")
    shutil.copyfile(FIXTURE, scenery_dir / scenery)
    for companion in COMPANIONS:
        part = companion.name[len("autotest_editor."):-len(".scn")]
        text = companion.read_text(encoding="utf-8").replace(FIXTURE.name, scenery)
        (scenery_dir / f"{stem}_{part}.scn").write_text(text, encoding="utf-8")
    passed = True
    expectations = 0
    seconds = 0.0
    try:
        for index, part in enumerate(parts, 1):
            script = out / f"part{index}.txt"
            script.write_text(part, encoding="utf-8", newline="\n")
            log = game_dir / "log.txt"
            since = time.time() - 1
            result = run_editor(exe, game_dir, scenery, script, inis, timeout, limit_mb, show)
            seconds += result.seconds
            # the log of this run, not one left from before by an editor that didn't start
            if log.exists() and log.stat().st_mtime >= since:
                shutil.copyfile(log, out / f"part{index}.log")
            text = read_log(out / f"part{index}.log")
            result.lines = selftest_lines(text)
            # the copy of the fixture and no other scenery: the script may save
            if not result.failure and f'loading scenario "{scenery}"' not in text.lower():
                result.failure = f"the editor didn't load {scenery} (see {out / f'part{index}.log'})"
            label = f"part {index} of {len(parts)}" if len(parts) > 1 else "run"
            say(f"  {label}: exit code {describe_exit(result.exit_code)}, {result.seconds:.0f} s, at most {result.peak_mb:.0f} MB")
            for line in result.lines:
                if "SELFTEST FAIL" in line or "SELFTEST result" in line:
                    say("    " + line[line.index("SELFTEST"):])
            summary = next((line for line in result.lines if "SELFTEST result" in line), "")
            counted = re.search(r"result: (\d+) failed of (\d+)", summary)
            if counted:
                expectations += int(counted.group(2))
            ok = not result.failure and result.exit_code == 0 and counted is not None and counted.group(1) == "0"
            if result.failure:
                say(f"    {result.failure}")
            elif counted is None:
                say(f"    no SELFTEST result line in the log ({out / f'part{index}.log'}): the script didn't reach quit")
            if not ok:
                passed = False
                break
        if passed and expectations == 0:
            say("    the case checks nothing: no expect in it")
            passed = False
    finally:
        clean_up(scenery_dir, stem, before, keep, say)
    return passed, expectations, seconds


def find_exe(explicit: str | None) -> Path:
    if explicit:
        return Path(explicit).resolve()
    patterns = ("build/bin/RelWithDebInfo/eu07*.exe", "build/bin/Release/eu07*.exe", "cmake-build-*/bin/Release/eu07*.exe", "build/bin/eu07*")
    candidates = sorted((p for pattern in patterns for p in REPO.glob(pattern) if p.is_file()), key=lambda p: p.stat().st_mtime)
    if not candidates:
        sys.exit("no eu07 executable found under build/bin, pass --exe")
    return candidates[-1]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cases", nargs="*", help="case names from cases/, or paths of scripts elsewhere (default: all of cases/)")
    ap.add_argument("--game-dir", default=os.environ.get("EU07_GAME_DIR"), help="game install with scenery/ (or EU07_GAME_DIR)")
    ap.add_argument("--exe", help="editor executable, built with -DWITH_EDITOR_SELFTESTS=ON (default: the newest eu07*.exe under build/bin)")
    ap.add_argument("--ini", help="settings file read over the runner's own ones, for example a low quality of the picture")
    ap.add_argument("--timeout", type=float, default=240.0, help="wall time limit of a run of the editor, seconds (default 240)")
    ap.add_argument("--memory-mb", type=int, default=8192, help="memory limit of the editor, MB (default 8192)")
    ap.add_argument("--show", action="store_true", help="show the window of the editor")
    ap.add_argument("--keep", action="store_true", help="keep the copies of the scenery, to look at what the scripts saved")
    args = ap.parse_args()

    if not args.game_dir:
        sys.exit("pass --game-dir or set EU07_GAME_DIR")
    game_dir = Path(args.game_dir).resolve()
    exe = find_exe(args.exe)
    user_ini = Path(args.ini).resolve() if args.ini else None
    print(f"exe:  {exe}\ngame: {game_dir}")

    # a case by its name in cases/, or a script elsewhere by its path (one being written, say)
    sources = {}
    for given in args.cases or sorted(p.stem for p in CASES_DIR.glob("*.txt")):
        path = Path(given) if given.endswith(".txt") else CASES_DIR / f"{given}.txt"
        if not path.is_file():
            sys.exit(f"no such case: {given}")
        sources[path.stem] = path.resolve()

    def say(line):
        print(line, flush=True)

    started = time.monotonic()
    results = {}
    for name, source in sources.items():
        say(f"\n{name}")
        results[name] = run_case(name, source, exe, game_dir, user_ini, args.timeout, args.memory_mb, args.show, args.keep, say)
        passed, expectations, seconds = results[name]
        say(f"{'PASS' if passed else 'FAIL'}  {name} ({expectations} expectations, {seconds:.0f} s)")

    print(f"\n=== summary ({time.monotonic() - started:.0f} s)")
    for name, (passed, expectations, _) in results.items():
        print(f"  {'PASS' if passed else 'FAIL'}  {name} ({expectations})")
    failed = sum(1 for passed, _, _ in results.values() if not passed)
    print(f"{len(results) - failed} passed, {failed} failed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
