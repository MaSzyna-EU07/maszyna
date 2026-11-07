# Scenario tests

End-to-end checks that run the real simulator on a scenery and look at how the run ended
and what a vehicle did: did it start, did it keep to the speed limit, did it brake and stop,
did it pass a signal at stop.

Scenery and vehicle data are not in this repo, so these run against a local game install
only, not in CI.

## The simulator's test mode

The runner drives the simulator's test mode (eu07/maszyna#541), which works without the
runner too and is meant for scenery authors as well:

```
eu07 --test -s <scenery.scn> [--test-timeout <seconds>] [-seed <number|word>] [-ini <settings file>]
```

With `--test` the simulator:

- needs `-s`; `-v` is ignored and every train is driven by AI (ghostview),
- draws nothing: the null renderer loads the 3D models (the physics needs some of their
  data) but no textures, and the window stays hidden,
- has no sound, no network, no python, no pause (not even one set by the scenery), and
  no crash dialog,
- runs the simulation as fast as it can in fixed steps of 0.03 s of simulation time per
  frame (three 10 ms physics steps), whatever the real time, the time speed or the machine,
- seeds its random numbers with a fixed seed unless `-seed` or `randomseed` gives one, so two
  runs of a scenery come out the same (the physicslogs of two runs are byte for byte equal),
- quits when the test ends, and says how it ended in the exit code and in a `test: ...`
  line on stdout and in the log.

A scenery reports its result to the memory cell `__test.result`, which the simulator creates
(in every run, so a test scenery loads normally too). Write value 1 to it for a pass and 0
for a failure, with an `UpdateValues` event; other values are ignored, and the text, if
given, goes to the log as the reason:

```
event test_pass UpdateValues 0 __test.result reached_the_stop 1 * endevent
event test_fail UpdateValues 0 __test.result wrong_track 0 * endevent
```

(`PutValues` isn't the event for it: it hands a command to a vehicle, it doesn't write a cell.)
A good place for the pass event is `event0` of a track the train should stop on: it runs
while a train stands there, and the length of the track is the margin of the stop.

The test ends with the first of these:

| exit code | result | when |
| --- | --- | --- |
| 0 | pass | `__test.result` was set to 1 and then every vehicle stood still for 1 s, so a train still moving towards a signal at stop has to stop first. Also the standstill end with `test.pass_on_standstill yes` |
| 1 | fail | `__test.result` was set to 0; ends at once |
| 2 | signal passed at danger | a train went past a signal showing stop (`SetVelocity 0 0`; `SetVelocity 0 x` on automatic block, which may be passed after a stop, doesn't count); ends at once, unless `test.fail_if_stop_overran no` |
| 3 | timeout | `test.timeout` seconds of simulation time went by (default 3600) |
| 4 | standstill | every vehicle stood still for `test.standstill` seconds (default 300) and nothing reported a result. Trains waiting at a station or for a signal count as standing, so the limit has to be longer than such waits |
| 5 | error | the run could not start: no `-s`, the scenery failed to load or has no vehicles, or the simulator threw |

A crash ends with the system's code instead (on Windows e.g. `0xC0000005`, -1073741819).

The limits go in a `config` block of the scenery:

```
config
test.timeout 600
test.standstill 120
test.pass_on_standstill no
test.fail_if_stop_overran yes
endconfig
```

`--test-timeout` on the command line wins over `test.timeout` of the scenery.

A signal is passed at danger when the head of a train goes past a signal which its driver
has in its speed table, showing stop. So it's the driver's view that counts: a train without
a driver isn't watched, and neither is a signal the driver never scanned (a train placed at
100 km/h 150 m before a signal at stop got past it before its AI first looked ahead; at
60 km/h it was caught). Outside the test mode the same is written to the log, nothing more.

## Running

```
python tests/scenarios/run.py --game-dir "C:/Steam/steamapps/common/MaSzyna" -j 4
```

Python 3.10+, no extra packages. Options:

- `EU07_GAME_DIR` can replace `--game-dir`.
- `--exe path/to/eu07.exe` picks the build to test; the default is the newest
  `cmake-build-*/bin/Release/eu07*.exe`. The build needs the `--test` and `-ini` command
  line options.
- name one or more cases to run only those, e.g. `run.py ... eu07_td_timetable`.
- `-j N` runs N cases at once. Each case prints its results when it is done. All 14 cases
  take about 30 s with `-j 4`, loading included.
- `--watch` runs a case the old way, in a visible window in real time (or faster with
  `--timespeed N`), closed after the case's `duration`; the exit code isn't checked then. For
  watching what a case does.

Exit code is 0 when every case passed. Logs of each run (`log.txt`, `errors.txt`,
`stdout.txt`, the vehicle's physicslog) are moved to `tests/scenarios/out/<case>/`.

## How it works

For each case the runner:

1. writes `scenery/autotest_<case>.scn` into the game dir: the base scenery with its own
   trainsets removed, the start time changed if the case says so, the case's track events
   added, and the case's `.scn` fragment added: its trainsets after `FirstInit`, everything
   else (the case's `test_config`, event launchers standing in for keys the scenery expects a
   player to press, extra events) before it,
2. starts the simulator with `--test -s autotest_<case>.scn -ini out/<case>/autotest.ini`,
   plus `--test-timeout <duration>` when the case gives a `duration`; the extra ini turns on
   `physicslog` and writes the logs to `logs/log (autotest_<case>) <date>.txt` rather than the
   shared `log.txt` (`multiplelogs`),
3. waits until the simulator quits by itself. It gives up early when the log reports
   `Bad vehicle` / `Bad trainset`, when the scenario has loaded and no physicslog appeared, or
   after the case's `timeout` of wall-clock time,
4. compares the result from the exit code with the case's `expect`, then runs the checks on
   the physicslog.

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
| `track_friction` | set the friction of every track of the base scenery (default in the sceneries is 0.15), e.g. for slippery rail |
| `track_events` | lines added to tracks of the base scenery, by track name, e.g. `{ "tor1k": "event0 test_pass" }`; only tracks in the scenery file itself, not in its includes |
| `test_config` | lines of a `config` block added to the scenery, e.g. `["test.timeout 90"]` |
| `fragment` | `.scn` file in `cases/` added to the scenery: trainsets after `FirstInit` (before it they derail on unlinked track), the rest before it (launchers read after it never fire) |
| `vars` | values for `{{name}}` in the fragment; a list becomes one line per item |
| `expect` | the result the test mode should end with: `pass`, `fail`, `signal_passed_at_danger`, `timeout`, `standstill` or `error`. Default `timeout` for a case with `duration`, else `pass` |
| `duration` | seconds of simulation to record: the test timeout of the run (`--test-timeout`) |
| `timeout` | wall-clock seconds before giving up, loading included |
| `fail_on_errors` | fail when `errors.txt` is not empty (default `false`) |
| `checks` | list of checks, below |

`{{vehicle}}` is always set, also inside `vars` values: the node name of the lead vehicle,
the one whose physicslog is checked. Other vehicles of the consist take names made from it,
e.g. `{{vehicle}}_s`. The td.scn cases share two fragments, `td_timetable.scn` (route set,
train runs to Przystanek) and `td_red_signal.scn` (no route, exit signal at stop), and differ
only in `vars.consist`. `td_red_signal_moving.scn` is the red signal one with the train
starting at `vars.velocity` km/h; with `track_friction` 0.03 it locks the wheels when braking
and slides past the signal, so those cases set `test.fail_if_stop_overran no`.
`td_red_signal_pass.scn` adds the `test_pass` event, which `test_mode_pass` puts as `event0`
on `tor1k`, the track in front of the signal.

The `test_mode_*` cases show the ends of the test mode: `test_mode_pass` (exit 0, the pass
event), `test_mode_signal_passed_at_danger` (exit 2), `test_mode_timeout` (exit 3, from
`test.timeout`) and `test_mode_standstill` (exit 4, from `test.standstill`).

Checks (speeds in km/h, times in seconds of physicslog time, distances in metres). Most use
the physicslog Velocity column, which is the wheel speed: it stays at 0 while the wheels are
locked in a skid. The ones marked *vehicle speed* use the vehicle speed column at the end of
the row instead, logged by builds from this branch on.

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
- `slips` `within`: the lead vehicle's wheels slip or lock at some point (within `within`
  seconds if given), so a slippery rail case does what it's meant to.
- `slip_clears` `moving_speed`, `stop_speed`, `within` (*vehicle speed*): once stopped after
  moving off, the wheels stop slipping within `within` seconds.
- `holds_still` `moving_speed`, `stop_speed`, `max_distance` (*vehicle speed*): once stopped
  after moving off, moves no more than `max_distance` metres, e.g. no creeping on locked wheels.

New check types go in `CHECKS` in `run.py`; each gets the physicslog rows and the check's
parameters and returns `(passed, detail)`.
