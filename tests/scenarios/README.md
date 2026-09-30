# Scenario tests

End-to-end checks that run the real simulator on a scenery and look at what a vehicle
did: did it start, did it keep to the speed limit, did it brake and stop.

Scenery and vehicle data are not in this repo, so these run against a local game install
only, not in CI.

## Running

```
python tests/scenarios/run.py --game-dir "E:/Program Files (x86)/Steam/steamapps/common/MaSzyna"
```

Python 3.10+, no extra packages. Options:

- `EU07_GAME_DIR` can replace `--game-dir`.
- `--exe path/to/eu07.exe` picks the build to test; the default is the newest
  `cmake-build-*/bin/Release/eu07*.exe`. The build needs the `-ini` command line option.
- name one or more cases to run only those, e.g. `run.py ... eu07_td_timetable`.
- `--timespeed N` sets the simulation time multiplier, default 20: a 4-minute case runs in
  about 20 s. Physics still steps finely (the physicslog keeps its 0.05 s rows) and the speed
  profile at 20 and 60 matched a real-time run within a few km/h and seconds. Use
  `--timespeed 1` to watch a run.

Exit code is 0 when every case passed. Logs of each run (`log.txt`, `errors.txt`,
the vehicle's physicslog) land in `tests/scenarios/out/<case>/`.

## How it works

For each case the runner:

1. writes `scenery/autotest_<case>.scn` into the game dir: the base scenery with its own
   trainsets removed, the start time changed if the case says so, and the case's `.scn`
   fragment added: its trainsets after `FirstInit`, everything else (event launchers standing
   in for keys the scenery expects a player to press, extra events) before it,
2. starts the simulator with `-s autotest_<case>.scn -v ghostview -ini out/<case>/autotest.ini`;
   ghostview leaves every train to the AI, and the extra ini turns on `physicslog`,
   runs in a 1280x720 window and does not pause when the window loses focus,
3. waits until `physicslog/<vehicle>.dat` covers `duration` seconds, then closes the simulator,
4. runs the checks on the physicslog.

## Writing a case

`cases/<name>.json`:

| key | meaning |
| --- | --- |
| `base_scenery` | scenery file in the game's `scenery/` dir |
| `strip_trainsets` | drop the base scenery's trainsets (default `true`) |
| `time` | replace the scenery start time, `HH:MM` |
| `fragment` | `.scn` file in `cases/` added to the scenery: trainsets after `FirstInit` (before it they derail on unlinked track), the rest before it (launchers read after it never fire) |
| `vehicle` | node name of the vehicle whose physicslog is checked |
| `duration` | seconds of simulation to record |
| `timeout` | wall-clock seconds before giving up, loading included |
| `fail_on_errors` | fail when `errors.txt` is not empty (default `false`) |
| `checks` | list of checks, below |

Checks (speeds in km/h, times in seconds of physicslog time):

- `starts` `min_speed`, `within`: reaches `min_speed` within `within` seconds.
- `max_speed` `limit`, `tolerance`: never goes faster than `limit + tolerance`.
- `brakes` `min_pressure`: after moving off, brake cylinder pressure reaches `min_pressure` bar
  (physicslog pressures are in bar whatever its header says; the brake pipe runs at 5.0).
- `stops` `moving_speed`, `stop_speed`, `hold`: after reaching `moving_speed`, stays at or
  below `stop_speed` for `hold` seconds.

New check types go in `CHECKS` in `run.py`; each gets the physicslog rows and the check's
parameters and returns `(passed, detail)`.
