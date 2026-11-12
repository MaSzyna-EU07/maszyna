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
eu07 --test -s <scenery.scn> [--test-timeout <seconds>] [--test-wall-timeout <seconds>] [-seed <number|word>] [-ini <settings file>]
```

With `--test` the simulator:

- needs `-s`; `-v` is ignored and every train is driven by AI (ghostview),
- reads the settings of the game folder's `eu07.ini` and of the files given with `-ini` (in that
  order), not the player's own `eu07.ini` in `%APPDATA%\MaSzyna` (`~/.config/MaSzyna` on Linux),
  so a test runs the same for whoever runs it,
- draws nothing: the null renderer loads the 3D models (the physics needs some of their
  data) but no textures, and the window stays hidden,
- has no sound, no network, no python, no pause (not even one set by the scenery), and
  no crash dialog,
- takes no input from outside: keys and mouse sent to its window, gamepad, UART (a cab
  desk), ZeroMQ and commands sent by other programs (`WM_COPYDATA`) are all ignored, and
  motion telemetry is off,
- gives nothing to the outside either: no keyboard leds, LPT or PoKeys (`feedbackmode`), no
  messages to a multiplayer server (`multiplayer`), no program run at the end (`execonexit`),
  no console window of its own (`showsystemconsole`), no Discord presence, and the game's `imgui.ini`
  stays as it is,
- keeps all of the above whatever the `config` block of the scenery says: the settings of the
  test mode are applied again after it,
- runs the simulation as fast as it can in fixed steps of 0.03 s of simulation time per
  frame (three 10 ms physics steps), whatever the real time, the time speed or the machine,
- seeds its random numbers with a fixed seed unless `-seed` or `randomseed` gives one, so two
  runs of a scenery come out the same (the physicslogs of two runs are byte for byte equal),
- quits when the test ends, and says how it ended in the exit code and in a `test: ...`
  line on stdout and in the log. A command line it can't take, a start that fails, or an
  exception thrown by the simulator (of any type, on Linux too) ends the same way, with the
  `error` code. That line is all there is on stdout: the log goes to
  the log file only (`debuglog` doesn't print it on the console in test mode).

On Windows the simulator is a windowed program, which the `cmd` prompt and PowerShell don't wait
for. In a `.bat` file `eu07 --test ... > result.txt` waits and sets `%errorlevel%`; at the prompt
`start /wait eu07 --test ...` waits too but doesn't pass the output on. In PowerShell use
`(Start-Process eu07 '--test ...' -Wait -PassThru -RedirectStandardOutput result.txt).ExitCode`.
Scripts (Python's `subprocess`, bash) wait for it as for any other program.

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
| 0 | pass | `__test.result` was set to 1 and then the tested trains stood still for 1 s, so a train still moving towards a signal at stop has to stop first (with `test.no_movement_before_pass no` the pass ends the test at once). Also the standstill end with `test.pass_on_standstill yes` |
| 10 | fail | `__test.result` was set to 0; ends at once |
| 11 | signal passed at danger | a train went past a signal showing stop (`SetVelocity 0 0`; `SetVelocity 0 x` on automatic block, which may be passed after a stop, doesn't count); ends at once, unless `test.fail_if_stop_overran no` |
| 12 | timeout | `test.timeout` seconds of simulation time went by (default 3600), or `test.wall_timeout` seconds of real time since the start (see below) |
| 13 | standstill | the tested trains stood still for `test.standstill` seconds (default 300) and nothing reported a result. Trains waiting at a station or for a signal count as standing, so the limit has to be longer than such waits |
| 14 | error | the run could not start: no `-s`, the scenery failed to load or has no vehicles, or the simulator threw |

A crash ends with the system's code instead (on Windows e.g. `0xC0000005`, -1073741819). The
codes of the test mode keep clear of the ones a process gets when it goes wrong on its own: 1
(killed with `TerminateProcess`, or a plain failure of most programs), 3 (`abort()` on Windows),
126-127 (a shell that can't run the program) and 128 + a signal number (killed, on Linux).

The limits go in a `config` block of the scenery:

```
config
test.timeout 600
test.wall_timeout 900
test.standstill 120
test.pass_on_standstill no
test.fail_if_stop_overran yes
test.no_movement_before_pass yes
test.vehicle ep07-424
endconfig
```

| key | default | meaning |
| --- | --- | --- |
| `test.timeout` | 3600 | seconds of simulation time before the test ends as `timeout`; 0 = never |
| `test.wall_timeout` | `test.timeout` + 600 | seconds of real time since the start before the test ends as `timeout`; 0 = never |
| `test.standstill` | 300 | seconds with the tested trains still and no result before the test ends as `standstill`; 0 = never |
| `test.pass_on_standstill` | no | the standstill end passes the test instead |
| `test.fail_if_stop_overran` | yes | a signal passed at danger ends the test; with `no` it's only logged |
| `test.no_movement_before_pass` | yes | a pass waits until the tested trains stand still; with `no` it ends the test at once |
| `test.vehicle` | none | a vehicle of a train under test, by its name in the scenery (once for each train under test). The tested trains are the consists of these vehicles as they are at the moment (a train the tested one couples to joins it), and only they count for the pass and the standstill: other trains, AI traffic, may run on. A name the scenery doesn't have ends the test with `error`. Not given, every vehicle of the scenery counts. A signal passed at danger counts for every train either way |

`--test-timeout` and `--test-wall-timeout` on the command line win over `test.timeout` and
`test.wall_timeout` of the scenery and the settings files.

`test.wall_timeout` is a watchdog in real time, for a run that hangs or crawls: when the
simulator still runs that many seconds after its start, loading included, it says `test: timeout`
and quits with the timeout code, whatever it is doing at the moment, with no dialog. Not given,
it is `test.timeout` + 600 s: ten minutes for the loading, and a simulation that runs at least as
fast as real time is never cut short by it. With `test.timeout 0` there is no watchdog unless
`test.wall_timeout` is given; `test.wall_timeout 0` turns it off. A test that has ended but whose
simulator doesn't close is ended by the watchdog too, with its own result.

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
- `-j N` runs N cases at once. Each case prints its results when it is done. All 22 cases
  take about 30 s with `-j 4`, loading included.
- `--watch` runs a case the old way, in a visible window in real time (or faster with
  `--timespeed N`), closed after the case's `duration`; the exit code isn't checked then. For
  watching what a case does.

Exit code is 0 when every case passed. Logs of each run (`log.txt`, `errors.txt`,
`stdout.txt`, the physicslogs of the vehicle and of other trains of the case, the black box as
`autotest_<case>.blackbox`) are moved to `tests/scenarios/out/<case>/`.

## How it works

For each case the runner:

1. writes `scenery/autotest_<case>.scn` into the game dir: the base scenery with its own
   trainsets removed, the start time changed if the case says so, the case's track events
   added, and the case's `.scn` fragment added: its trainsets after `FirstInit`, everything
   else (the case's `test_config`, event launchers standing in for keys the scenery expects a
   player to press, extra events) before it,
2. starts the simulator with `--test -s autotest_<case>.scn -ini out/<case>/autotest.ini
   --test-wall-timeout <timeout>`, plus `--test-timeout <duration>` when the case gives a
   `duration`; the extra ini turns on
   `physicslog` (and with it the black box of the run, `physicslog/autotest_<case>_<date>.blackbox`)
   and writes the logs to `logs/log (autotest_<case>) <date>.txt` rather than the
   shared `log.txt` (`multiplelogs`),
3. waits until the simulator quits by itself. It gives up early when the log reports
   `Bad vehicle` / `Bad trainset`, or when the scenario has loaded and no physicslog appeared.
   After the case's `timeout` of wall-clock time the simulator ends itself with the timeout code;
   one that doesn't is killed 30 s later,
4. compares the result from the exit code with the case's `expect`, then runs the checks on
   the physicslog, the log and the black box.

The lead vehicle is named `autotest_<case>`, so every case has its own physicslog and log
files and cases can run in parallel. The `autotest_*` sceneries (and the simulator's `.sbt`
geometry cache of them) stay in the game's `scenery/` dir; the cache is dropped when the base
scenery is newer, and after a run cut short (`wall_timeout`, `error`, a crash, or killed by run.py),
which could have been writing it.

## Writing a case

`cases/<name>.json`:

| key | meaning |
| --- | --- |
| `base_scenery` | scenery file in the game's `scenery/` dir |
| `strip_trainsets` | drop the base scenery's trainsets (default `true`) |
| `time` | replace the scenery start time, `HH:MM` |
| `track_friction` | set the friction of every track of the base scenery (default in the sceneries is 0.15), e.g. for slippery rail |
| `track_events` | lines added to tracks of the base scenery, by track name, e.g. `{ "tor1k": "event0 test_pass" }`; only tracks in the scenery file itself, not in its includes |
| `test_config` | lines of a `config` block added to the scenery, e.g. `["test.timeout 90"]`; the block always has `test.vehicle` with the case's lead vehicle, so only its train counts for the pass and the standstill |
| `ini` | extra `eu07.ini` lines for the run, after the common ones of the runner's `-ini` file, e.g. `["crashdamage yes"]` |
| `fragment` | `.scn` file in `cases/` added to the scenery: trainsets after `FirstInit` (before it they derail on unlinked track), the rest before it (launchers read after it never fire) |
| `vars` | values for `{{name}}` in the fragment; a list becomes one line per item |
| `timetable` | a timetable file in `cases/` for a trainset of the fragment, for a train whose timetable no scenery of the game has: copied to the game's `scenery/` dir as `autotest_<case>.txt` for the run and removed after it; the fragment names it `{{timetable}}` |
| `expect` | the result the test mode should end with: `pass`, `fail`, `signal_passed_at_danger`, `timeout`, `standstill` or `error`, or `wall_timeout` for the timeout of the real time watchdog (the timeout code with a verdict saying it was the real time; a case expecting `timeout` fails on it). Default `timeout` for a case with `duration`, else `pass` |
| `duration` | seconds of simulation to record: the test timeout of the run (`--test-timeout`) |
| `timeout` | wall-clock seconds before giving up, loading included: the simulator's `--test-wall-timeout` (default 600) |
| `fail_on_errors` | fail when `errors.txt` is not empty (default `false`) |
| `checks` | list of checks, below |

`{{vehicle}}` is always set, also inside `vars` values: the node name of the lead vehicle,
the one whose physicslog is checked, and the train under test (`test.vehicle`). Other vehicles
of the consist take names made from it, e.g. `{{vehicle}}_s`; another train with a driver is named
`<something>_{{vehicle}}`, so its physicslog is moved out of the game dir too. The td.scn cases share two fragments, `td_timetable.scn` (route set,
train runs to Przystanek) and `td_red_signal.scn` (no route, exit signal at stop), and differ
only in `vars.consist`. Przystanek is a technical stop in that timetable (`pt`), where the AI
doesn't wait for the departure time: it sets off as soon as it counts the train as standing at
the W4, which is below 1 km/h, so the train stands there for a few seconds at most, or doesn't
quite stop (since the AI braking of #264 the SN61 turns back at 0.4 km/h). The
timetable cases therefore check that the train comes down to 1 km/h, with no hold, 1380-1420 m
from the start (each of the seven trains stops at 1397-1401 m). `td_red_signal_moving.scn` is the red signal one with the train
starting at `vars.velocity` km/h; with `track_friction` 0.03 it locks the wheels when braking
and slides past the signal, so those cases set `test.fail_if_stop_overran no`.
`td_red_signal_pass.scn` adds the `test_pass` event, which `test_mode_pass` puts as `event0`
on `tor1k`, the track in front of the signal.

`eu07_td_departure_station_capitals` runs an EU07 from where `td_timetable.scn` puts it, with the W4 of Stacja1 about
35 m ahead, on a timetable of its own (`td_departure_station_capitals.txt`, fragment `td_departure_station.scn`):
its first row, the station of departure, gives no arrival time and is written in capitals (`STACJA1`), the relation
says `Stacja1`, as in some timetables of the game (`Os33733` of linia_107_poludnie.scn starts at `ZAGORZ`). The
train has to stop there and leave at its departure time, 10:35, which the log shows as `as TEST24: at 10:35 next
Przystanek`; taking Stacja1 for a station it passes through, the driver logs `passed STACJA1` at once and leaves as
soon as the route is set at 10:34, and the train manager gives no departure signal. `eu07_td_departure_station` is the
same with the row written as in the relation. `eu07_td_timetable_past_w4` (fragment `td_timetable_past_w4.scn`) puts
the td/rozklad train, an EU07 with three coaches, with its head 80 m past the W4 of Stacja1, on `tor1k`: the W4 stays
under the consist, behind the head, and the driver still counts Stacja1 and leaves at 10:35, since the scan of the
route begins at the last vehicle and keeps a W4 behind the head.

The `test_mode_*` cases show the ends of the test mode: `test_mode_pass` (exit 0, the pass
event), `test_mode_signal_passed_at_danger` (exit 11), `test_mode_timeout` (exit 12, from
`test.timeout`), `test_mode_standstill` (exit 13, from `test.standstill`) and
`test_mode_wall_timeout` (exit 12 from the real time watchdog, which ends it while it loads).
`test_mode_standstill_traffic` (fragment `td_traffic.scn`) has a train under test standing on
track 2 of Stacja1 and AI traffic, `traffic_{{vehicle}}`, leaving track 1 on the timetable: the
standstill of 100 s ends the test although the traffic runs on (counting every vehicle, it runs
into the test timeout).
`test_mode_pass_while_moving` reports the pass from `event2` of `tor1k` (the direction the train runs), as the train runs onto
it, with `test.no_movement_before_pass no`, so the test ends with the train still moving.

The `en57_2000_automatic_coupler_*` cases (issue #86) share `td_automatic_coupler_hit.scn`: an
EN57-2000 unit without a driver runs at `vars.velocity` km/h into a standing one, from
`35.6 - vars.offset` m behind it, with `crashdamage yes` in `ini`. At 50 km/h the automatic
couplers mustn't catch and the log has to show the collision and the derailment; at 4 km/h they
couple and neither shows up: the black box has to have the last car of the standing unit,
`{{vehicle}}_rb`, and the first of the rolling one, `{{vehicle}}_hit_ra`, coupled (the simulator
writes nothing to the log when couplers catch). Nothing reports a result, so both end on the 60 s
test timeout.

Checks (speeds in km/h, times in seconds of physicslog time, distances in metres). Most use
the physicslog Velocity column, which is the wheel speed: it stays at 0 while the wheels are
locked in a skid. The ones marked *vehicle speed* use the vehicle speed column at the end of
the row instead, logged by builds from this branch on.

- `starts` `min_speed`, `within`: reaches `min_speed` within `within` seconds.
- `max_speed` `limit`, `tolerance`: never goes faster than `limit + tolerance`.
- `brakes` `min_pressure`: after moving off, brake cylinder pressure reaches `min_pressure` bar
  (physicslog pressures are in bar whatever its header says; the brake pipe runs at 5.0).
- `stops` `moving_speed`, `stop_speed`, `hold`, optional `min_distance`, `max_distance`: after
  reaching `moving_speed`, stays at or below `stop_speed` for `hold` seconds; with the distances,
  that stop has to be between them (metres travelled, as in `distance`).
- `stays_stopped` `moving_speed`, `stop_speed`: after moving off and stopping, never goes
  above `stop_speed` again.
- `distance` `min`, `max`: total distance travelled. The physicslog has no position, so the
  distance comes from integrating the speed; on td.scn every vehicle's first stop after
  Stacja1 came out at 1400 m (+-4), and the red signal stop at 140 m.
- `slips` `within`: the lead vehicle's wheels slip or lock at some point (within `within`
  seconds if given), so a slippery rail case does what it's meant to.
- `slip_clears` `moving_speed`, `stop_speed`, `within` (*vehicle speed*): once stopped after
  moving off, the wheels stop slipping within `within` seconds.
- `holds_still` `moving_speed`, `stop_speed`, `max_distance` (*vehicle speed*): once stopped
  after moving off, moves no more than `max_distance` metres, e.g. no creeping on locked wheels.
- `ends_moving` `min_speed`: the last row of the physicslog is at `min_speed` or faster, e.g.
  the test ended without waiting for the train to stop.
- `log_contains` `pattern`: a line of the run's `log.txt` matches the regex, e.g. the
  `Bad driving: ... collided` message of a collision.
- `log_lacks` `pattern`: no line of the run's `log.txt` matches the regex.
- `couples` `vehicle`, `other`: the black box of the run has the two vehicles coupled to each
  other (the coupler itself, whatever else of the hoses and the cables) at some time after the
  start, and still coupled at the end. The vehicles go by their node names, with `{{vehicle}}`
  filled in as in the fragment, e.g. `{{vehicle}}_rb`. Fails when the two were coupled from the
  start, came apart again, or the run left no black box. The black box is looked at once a
  second, so the time given is that of the first look after the couplers caught.

New check types go in `CHECKS` in `run.py`; each gets the physicslog rows and the check's
parameters and returns `(passed, detail)`. Checks on the log go in `LOG_CHECKS` and get the
text of `log.txt` instead of the rows. Checks on the black box (`blackbox_doc.md` in the root of
the repo; the simulator writes it along with the physicslogs) go in `BLACKBOX_CHECKS` and get
what `read_blackbox()` read of it: the names of the vehicles and the coupling records. A check
on another kind of record has to add it there.
