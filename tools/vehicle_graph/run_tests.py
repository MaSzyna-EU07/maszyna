#!/usr/bin/env python3
"""Tests of the vehicle graph core, run through the simulator's -graph-run command.

usage: run_tests.py <simulator executable> [--update]

- every tests/*.vhg file starts with a line "// expect: ok", "// expect: error <text>" or "// expect: warning <text>",
  and is checked against the output of -graph-run;
- every <name>.vhg with a <name>.script, in both directories, is run twice, and both results have to be identical (determinism)
  and equal to <name>.expected.csv. --update writes the current results as the expected ones.
"""

import pathlib
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parent


def run(executable, *arguments):
    result = subprocess.run([str(executable), "-graph-run", *map(str, arguments)], capture_output=True, text=True)
    return result.returncode, result.stdout + result.stderr


def check_file(executable, path):
    first = path.read_text(encoding="utf-8").splitlines()[0]
    if not first.startswith("// expect:"):
        return f"{path.name}: missing '// expect:' line"
    expectation = first[len("// expect:"):].strip()
    kind, _, text = expectation.partition(" ")
    code, output = run(executable, path)
    if kind == "ok":
        if code != 0 or "error" in output:
            return f"{path.name}: expected no errors, got:\n{output}"
    elif kind == "error":
        if code == 0 or text not in output:
            return f"{path.name}: expected error containing '{text}', got (exit {code}):\n{output}"
    elif kind == "warning":
        if code != 0 or "warning" not in output or text not in output:
            return f"{path.name}: expected warning containing '{text}', got (exit {code}):\n{output}"
    else:
        return f"{path.name}: unknown expectation '{expectation}'"
    return None


def check_example(executable, path, update):
    script = path.with_suffix(".script")
    expected = path.with_suffix(".expected.csv")
    results = []
    with tempfile.TemporaryDirectory() as directory:
        for attempt in range(2):
            output = pathlib.Path(directory) / f"run{attempt}.csv"
            code, log = run(executable, path, "-script", script, "-every", 10, "-o", output)
            if code != 0:
                return f"{path.name}: run failed:\n{log}"
            # git may check the expected files out with windows line endings
            results.append(output.read_bytes().replace(b"\r\n", b"\n"))
    if results[0] != results[1]:
        return f"{path.name}: two runs gave different results"
    if update:
        expected.write_bytes(results[0])
        return None
    if not expected.exists():
        return f"{path.name}: missing {expected.name}, run with --update to create it"
    if expected.read_bytes().replace(b"\r\n", b"\n") != results[0]:
        return f"{path.name}: results differ from {expected.name}"
    return None


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    executable = pathlib.Path(sys.argv[1]).resolve()
    update = "--update" in sys.argv[2:]
    failures = []
    count = 0
    for path in sorted((ROOT / "tests").glob("*.vhg")):
        count += 1
        problem = check_file(executable, path)
        if problem:
            failures.append(problem)
    for path in sorted((ROOT / "tests").glob("*.vhg")) + sorted((ROOT / "examples").glob("*.vhg")):
        if path.with_suffix(".script").exists():
            count += 1
            problem = check_example(executable, path, update)
            if problem:
                failures.append(problem)
    for problem in failures:
        print("FAIL", problem)
    print(f"{count - len(failures)} of {count} passed")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
