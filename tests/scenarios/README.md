# Scenario tests

End-to-end checks that run the real simulator on a scenery and look at what a vehicle
did: did it start, did it keep to the speed limit, did it brake and stop.

Scenery and vehicle data are not in this repo, so these run against a local game install
only, not in CI.

## Running

```
python tests/scenarios/run.py --game-dir "E:/Program Files (x86)/Steam/steamapps/common/MaSzyna" -j 4
```

Python 3.10+, no extra packages. Options:

- `EU07_GAME_DIR` can replace `--game-dir`.
- `--exe path/to/eu07.exe` picks the build to test; the default is the newest
  `cmake-build-*/bin/Release/eu07*.exe`. The build needs the `-ini` command line option.
- name one or more cases to run only those, e.g. `run.py ... eu07_td_timetable`.
- `-j N` runs N cases at once, each in its own simulator window. Each case prints its
  results when it is done. All 8 cases take about a minute with `-j 4`.
- `--timespeed N` sets the simulation time multiplier, default 20: a 4-minute case runs in
  about 20 s. Physics still steps finely (the physicslog keeps its 0.05 s rows) and the speed
  profile at 20 and 60 matched a real-time run within a few km/h and seconds. Use
  `--timespeed 1` to watch a run.

Exit code is 0 when every case passed. Logs of each run (`log.txt`, `errors.txt`,
the vehicle's physicslog) are moved to `tests/scenarios/out/<case>/`.

## How it works

For each case the runner:

1. writes `scenery/autotest_<case>.scn` into the game dir: the base scenery with its own
   trainsets removed, the start time changed if the case says so, and the case's `.scn`
   fragment added: its trainsets after `FirstInit`, everything else (event launchers standing
   in for keys the scenery expects a player to press, extra events) before it,
2. starts the simulator with `-s autotest_<case>.scn -v ghostview -ini out/<case>/autotest.ini`;
   ghostview leaves every train to the AI, and the extra ini turns on `physicslog`, runs in a
   1280x720 window, does not pause when the window loses focus, and writes the logs to
   `logs/log (autotest_<case>) <date>.txt` rather than the shared `log.txt` (`multiplelogs`),
3. waits until `physicslog/autotest_<case>.dat` covers `duration` seconds, then closes the
   simulator. It gives up early when the log reports `Bad vehicle` / `Bad trainset`, or when
   the scenario has loaded and no physicslog appeared,
4. runs the checks on the physicslog.

The lead vehicle is named `autotest_<case>`, so every case has its own physicslog and log
files and cases can run in parallel. The `autotest_*` sceneries (and the simulator's `.sbt`
geometry cache of them) stay in the game's `scenery/` dir; the cache is dropped when the base
scenery is newer.

## Writing a case

`cases/<name>.json`:

| key | meaning |
| --- | --- |
| `base_scenery` | scenery file in the game's `scenery/` dir |
| `strip_trainsets` | drop the base scenery's trainsets (default `true`) |
| `time` | replace the scenery start time, `HH:MM` |
| `fragment` | `.scn` file in `cases/` added to the scenery: trainsets after `FirstInit` (before it they derail on unlinked track), the rest before it (launchers read after it never fire) |
| `vars` | values for `{{name}}` in the fragment; a list becomes one line per item |
| `duration` | seconds of simulation to record |
| `timeout` | wall-clock seconds before giving up, loading included |
| `fail_on_errors` | fail when `errors.txt` is not empty (default `false`) |
| `checks` | list of checks, below |

`{{vehicle}}` is always set, also inside `vars` values: the node name of the lead vehicle,
the one whose physicslog is checked. Other vehicles of the consist take names made from it,
e.g. `{{vehicle}}_s`. The td.scn cases share two fragments, `td_timetable.scn` (route set,
train runs to Przystanek) and `td_red_signal.scn` (no route, exit signal at stop), and differ
only in `vars.consist`.

Checks (speeds in km/h, times in seconds of physicslog time, distances in metres):

- `starts` `min_speed`, `within`: reaches `min_speed` within `within` seconds.
- `max_speed` `limit`, `tolerance`: never goes faster than `limit + tolerance`.
- `brakes` `min_pressure`: after moving off, brake cylinder pressure reaches `min_pressure` bar
  (physicslog pressures are in bar whatever its header says; the brake pipe runs at 5.0).
- `stops` `moving_speed`, `stop_speed`, `hold`: after reaching `moving_speed`, stays at or
  below `stop_speed` for `hold` seconds.
- `stays_stopped` `moving_speed`, `stop_speed`: after moving off and stopping, never goes
  above `stop_speed` again.
- `distance` `min`, `max`: total distance travelled. The physicslog has no position, so the
  distance comes from integrating the speed; on td.scn every vehicle's first stop after
  Stacja1 came out at 1400 m (+-3), and the red signal stop at 140 m.

New check types go in `CHECKS` in `run.py`; each gets the physicslog rows and the check's
parameters and returns `(passed, detail)`.
