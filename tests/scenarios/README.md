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
- seeds its random numbers with a fixed seed unless `-seed` or `randomseed` gives one, so,
  with the fixed step above, two runs of a scenery come out the same (the physicslogs of two
  runs are byte for byte equal; a seed without the fixed step isn't enough, see
  [A seed alone doesn't make a run repeatable](#a-seed-alone-doesnt-make-a-run-repeatable)),
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
- `-j N` runs N cases at once. Each case prints its results when it is done. The 42 cases
  that aren't `slow` take about 30 s with `-j 4` and 2 minutes one at a time, loading included.
- `--watch` runs a case the old way, in a visible window in real time (or faster with
  `--timespeed N`), closed after the case's `duration`; the exit code isn't checked then. For
  watching what a case does.
- `--slow` runs the cases marked `slow` too. Without it, and with no cases named, they are
  skipped (and listed): each is a whole mission on a big scenery and takes many minutes. A
  slow case named on the command line always runs. Run them one at a time (`-j 1`): each
  simulator loads a whole scenery. A case on a mission of the base scenery with physicslog on
  (`vehicle` given) removes every physicslog written in the game dir meanwhile, so with `-j N`
  it runs alone, after the others.
- `--seed N` (a number or a word) runs the cases with another seed of the simulator's random
  numbers (`randomseed`) than the fixed one of the test mode, for a bug that shows up only
  sometimes: run the case with a few seeds. A seed repeats a run only in the test mode, see
  below.

### A seed alone doesn't make a run repeatable

The same seed gives the same run only when every frame lasts the same simulation time. Each
frame the simulator splits the time since the last one into physics steps of at most 10 ms
(`driver_mode::update()`); outside the test mode that time comes from the wall clock (times
`timespeed`), so the number and the length of the steps follow the frame rate, and with them
the speeds, the pressures and the moments the AI decides things. Two runs of one scenery with
one seed then drift apart, the more so on a busy machine.

- The test mode (`--test`, what `run.py` runs by default) sets a fixed step itself: 0.03 s of
  simulation time per frame, three 10 ms steps, whatever the wall clock
  (`Timer::set_fixed_step()` in `driver_mode::enter()`). A case run twice with the same
  `--seed`, or with none, comes out the same. This is the way to repeat a run.
- `--watch` doesn't use the test mode: it runs on the wall clock, so a watched run isn't
  repeatable, seed or not. Use it to look at a case, not to repeat one.
- Outside the test mode (`--watch`, or the simulator started by hand with `-seed`) the setting
  `deltaoverride N` makes every frame last 1/N s of simulation time, whatever the wall clock
  and `timespeed` (`Timer::set_delta_override()`). It goes in `eu07.ini`, a `-ini` file or a
  case's `ini` list. In the test mode it takes the place of the 0.03 s step. Whether a watched
  run with it repeats byte for byte hasn't been checked.

Exit code is 0 when every case passed, or failed as its `xfail` mark says it does (`XFAIL` in
the summary); a case marked `xfail` that passes (`XPASS`) fails the run, so that the mark gets
removed. Logs of each run (`log.txt`, `errors.txt`,
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
which could have been writing it. A case with `file.binary.terrain no` in its `ini` has no cache
written, and one left by an earlier run is removed.

A case with `vehicle` runs a train of the base scenery under its own name: its physicslog is
`physicslog/<vehicle>.dat` (one of that name already in the game dir is replaced), and with the
physicslogs on, every train of the scenery writes one, over any of the same name; those written
during the run are deleted after it. Don't run such a case along with another one in the same game dir.

Apart from these the game dir loses its `log.txt` and `errors.txt`, those of the last game
session: the simulator deletes them at every start, before it knows that the run's log goes
to `logs/`.

## Writing a case

`cases/<name>.json`:

| key | meaning |
| --- | --- |
| `base_scenery` | scenery file in the game's `scenery/` dir |
| `strip_trainsets` | drop the base scenery's trainsets (default `true`); `false` keeps them, for a case on one of the scenery's own missions (below) |
| `vehicle` | the train under test is this vehicle of the base scenery (its node name), instead of the `autotest_<case>` one of the fragment: `test.vehicle`, the physicslog checked and `{{vehicle}}` are this one. With `strip_trainsets` `false` |
| `replace` | literal replacements in the text of the base scenery, a list of `[old, new]`, e.g. to put another locomotive on a mission; an `old` the scenery doesn't have is an error |
| `physicslog` | `false` turns the physicslogs (and the black box) off for the run: on a scenery with dozens of AI trains each of them would write 3-4 kB per second of simulation into the game dir. Such a case has the result of the test mode and the log checks only (default `true`) |
| `slow` | `true` marks a case that takes many minutes: skipped unless named or run with `--slow` |
| `xfail` | marks a case that fails because of a known, reported bug, with the reference and the reason as its text, e.g. `"eu07/maszyna#524: the AI never couples"`. Its failure is reported as `XFAIL` and doesn't fail the run; when it passes (`XPASS`) the run fails until the mark is removed |
| `time` | replace the scenery start time, `HH:MM` |
| `track_friction` | set the friction of every track of the base scenery (default in the sceneries is 0.15), e.g. for slippery rail |
| `track_events` | lines added to tracks of the base scenery, by track name, e.g. `{ "tor1k": "event0 test_pass" }`; only tracks in the scenery file itself, not in its includes |
| `test_config` | lines of a `config` block added to the scenery, e.g. `["test.timeout 90"]`; the block always has `test.vehicle` with the case's lead vehicle, so only its train counts for the pass and the standstill |
| `ini` | extra `eu07.ini` lines for the run, after the common ones of the runner's `-ini` file, e.g. `["crashdamage yes"]` |
| `fragment` | `.scn` file in `cases/` added to the scenery: trainsets after `FirstInit` (before it they derail on unlinked track), the rest before it (launchers read after it never fire). Optional when the case keeps the scenery's trainsets |
| `vars` | values for `{{name}}` in the fragment; a list becomes one line per item |
| `timetable` | a timetable file in `cases/` for a trainset of the fragment, for a train whose timetable no scenery of the game has: copied to the game's `scenery/` dir as `autotest_<case>.txt` for the run and removed after it; the fragment names it `{{timetable}}` |
| `vehicle_variants` | vehicles of the case's own, made from those of the game, e.g. with a `.fiz` key the game's data doesn't have yet: a list of `{ "name": ..., "from": "<dir>/<type>", "replace": [[old, new], ...] }`. For the run the vehicle's `.fiz`, with the literal replacements (an `old` it doesn't have is an error), and its `.mmd` are copied beside it in the game's `dynamic/` dir as `autotest_<case>_<name>` and removed after it; the fragment names the type `{{variant_<name>}}`, e.g. `dynamic pkp/et42_v2 112e-014-a {{variant_a}} ...`. The copy of the `.mmd` has the name of the vehicle it is made from in place of `(p2)`, the type name the simulator hands it, which some pass on to the include that names the model |
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
`td_shunt_red_signal.scn` is the same with no timetable, so the AI shunts (Shunt) towards the signal at
stop, starting `vars.offset` m into `tdo_n1734`: `eu07_td_shunt_red_signal` alone and
`eu07_td_shunt_push_red_signal` pushing three coaches, both from 30 km/h, have to stop before the signal,
151 m from the start (the shunting AI used to coast up to it and pass it at about 28 km/h, fixed
by #409). They stop after 145 m and 141 m.

The `traxx_*` cases run a Traxx (E186, `p160dc`), whose driver's brake handle (MHZ_6P) has a time
chamber, like the MHZ_K5P, M394, Knorr, St113, FVel6 and FVE408 handles, which no other case has: the
AI getting such a locomotive ready has a rule of its own for releasing the train brake (5775493b2,
#444). `traxx_td_timetable` is the timetable run from a trainset ready to depart,
`traxx_td_timetable_cold` the same from a cold start (`td_cold_timetable.scn`, the handle in its
holding position, the brake pipe low), and `traxx_td_crew_add` the locomotive standing ready to depart
with nobody in it, the handle in its holding position and the brake pipe charged, until a driver is
put in (`td_crew.scn`, as `eu07_td_crew_add`). In all three the driver puts the handle to running
within a second of starting and leaves.

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

`eu07_coaches_td_timetable` is the timetable case with the EU07 and three coaches, a train that brakes with the train
brake, beside the light EU07 and the SN61 alone, which brake with the independent brake. The `*_td_downgrade_w4_signal`
cases (fragment `td_downgrade_w4_signal.scn`, timetable `td_downgrade.txt`) run on a line the fragment adds beside the
tracks of td.scn, at x = -3000: 300 m level, then 3000 m down a gradient of 20 per mille, under an overhead wire fed
by a substation of its own (`gr_pwr`; the wire on the section `pwr01` of td.scn gave the EU07 no voltage). The train
starts on the level at the W4 of Gora, stops at the W4 of Dol 1500 m down the gradient, leaves at 10:38 and stops before
`gr_sem`, a signal at stop 1100 m further down (a memory cell with `SetVelocity 0 0` and the event that reads it, no
model), about 1590 m and 2690 m from the start. The three trains are the light EU07, the SN61 alone and the EU07 with
three coaches.

### Heavy trains

The heavy train cases (#187, #199, #263, #265) run on lines of their own beside the tracks of td.scn, at x = -3000, as
the `*_td_downgrade_w4_signal` cases of #479 do: 100 m tracks, an overhead wire fed by a substation of its own (none for
the diesel case) and signals at stop that are a memory cell with `SetVelocity 0 0` and the event that reads it, with no
model. Each train starts on a level stretch long enough for the whole consist, with its head 10 m before the W4 of Gora,
which is where the distances in the checks start from. A freight train leaves a station of its timetable as soon as it
stands there (`IsCargoTrain` in `TController`), so the freight trains leave Gora at once, not at 10:35. The checks that
look for a train that comes to a stand use the vehicle speed (`keeps_moving`), as the wheels of a heavy train starting
on poor rails spin while it stands; `starts_again` checks a start from a signal that clears at a given time, and how far
the train rolls back meanwhile.

| case | fragment | train | what it shows |
| --- | --- | --- | --- |
| `et41_coal_td_downgrade_restriction` | `td_downgrade_restriction.scn`: 600 m level, 3500 m down 20 per mille, 20 km/h from 1500 to 1900 m down, the signal 3000 m down | ET41, 25 coal wagons 412W with 40 t each, G (about 1670 t) | passes: through the restriction at 10.5 km/h at the lowest, stops at the signal (3078 m) |
| `et22_764_td_downgrade_restriction` | the same | ET22, 20 heavy wagons 764 with 84 t each (about 2650 t) | `xfail`: dips to 8.5 km/h in the 20 km/h restriction (below the 10 km/h #263 counts as a defect); it doesn't stop |
| `st43_steel_td_downgrade12_long` | `td_downgrade12_long.scn`: 900 m level, 6500 m down 12 per mille, no wire, the signal 6000 m down | ST43, 45 coil wagons 425S, G | passes: holds 35-37.5 km/h of the 40 all the way down, never below 20 km/h, stops at the signal |
| `et41_coal_td_upgrade_start` | `td_upgrade_signal_start.scn`: 600 m level, 2500 m up 10 per mille, a signal 1000 m up that clears at 10:42, another 2300 m up | the ET41 with 25 coal wagons | passes: stops at the first signal with the whole train on the gradient, moves off 4.7 s after it clears, 10 km/h after 52 s, rolls back 0.04 m |
| `eu07a_coaches_td_upgrade_long` | `td_upgrade_long.scn`: 600 m level, 5500 m up 10 per mille, the signal 5300 m up | EU07A (303E), 8 coaches (about 360 t), passenger timetable at 80 km/h | passes: 80 km/h at 1430 m, never below 73.8 km/h on the climb |
| `et41_coal_td_slippery_start` | `td_level_signal.scn`: 2100 m level, the signal 1000 m ahead | ET41, 25 coal wagons with 60 t each (about 2170 t), `friction 0.3` | passes with the fix of this branch: gets going and stops at the signal (1082 m, at 392 s); before it, 395 slips in 1500 s and 75 m |

What the AI still does with heavy trains, without a case that fails on it:

- On a steep descent into a restriction a heavy G train holds the speed by applying and releasing the train brake in
  turn. Its brake takes 10-15 s to release, so it falls well below the limit before the brake lets go: 10.5-12 km/h
  for the ET41 with coal wagons, 8.5 km/h for the ET22 with the 764 wagons (the `xfail` above), in a 20 km/h
  restriction. #263 reported this train stopping before the restriction in 10 of 12 runs; on staging it no longer
  does, also with `friction 0.5` and with the wagons fully loaded.
- Starting uphill, a heavy train pulls against its train brake while the brake releases (#265): in
  `et41_coal_td_upgrade_start` about 13 s at about 200 kN with the head moving by the slack of the couplers alone. It
  starts at full power, without taking up the slack first.
- The AI doesn't know the adhesion of the rails: it plans braking with `Global.FrictionWeatherFactor` alone, and the
  result of its running brake test (`adjust_desired_speed_for_braking_test`) is shown in the debug panel only. It sands
  ahead of a slip only at a high motor current, and, since the fix of `et41_coal_td_slippery_start`, through the rest of
  a start that has slipped (`control_wheelslip()`); not before a hard brake, nor before the first slip of a start.

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
`test_mode_standing_1m_past_signal` and `test_mode_standing_4m_past_signal` (fragment `td_past_red_signal.scn`) set
the td/rozklad train at a stand with its head 1 m and 4 m past the exit signal at stop (the memory cell of the
signal is 51 m into `tor1k`). The driver finds the signal behind the head at its first look ahead, 13.6 s in, and a
standing train passes nothing, so both run into their timeout. When a signal found up to 5 m behind the head
still counted as one the head had ahead, standing or not, both ended as signals passed at danger at 13.6 s (a
train set 6 m or 20 m past didn't). Starting at 0 km/h the driver doesn't wake up within the case, so they start at
0.1 km/h, as `td_red_signal.scn` does.

The `en57_2000_automatic_coupler_*` cases (issue #86) share `td_automatic_coupler_hit.scn`: an
EN57-2000 unit without a driver runs at `vars.velocity` km/h into a standing one, from
`35.6 - vars.offset` m behind it, with `crashdamage yes` in `ini`. At 50 km/h the automatic
couplers mustn't catch and the log has to show the collision and the derailment; at 4 km/h they
couple and neither shows up: the black box has to have the last car of the standing unit,
`{{vehicle}}_rb`, and the first of the rolling one, `{{vehicle}}_hit_ra`, coupled (the simulator
writes nothing to the log when couplers catch). Nothing reports a result, so both end on the 60 s
test timeout.

`et21_td_no_wire_reports` (issue #421, fragment `td_no_wire.scn`) adds a 200 m track of its own,
with no overhead wire, 400 m off the line of td.scn (every track of td.scn is under the wire), with
an ET21 and its AI driver on it. The pantographs go up and find no wire, so the locomotive can't
start: the driver has to say so in the log, once, naming the locomotive and where it stands, and
not mistake it for a wire without voltage. It ends on the standstill.

`eu07_td_cold_connect` (fragment `td_cold_connect.scn`) has an EU07 with a hand brake
(`4E-ZEZ_GREY`; the plain `4e` has none) standing cold on track 1 of Stacja1 (trainset velocity 0,
so it starts with the hand brake on) and two coaches standing 45 m ahead of it. Its driver sleeps
until `event0` of its track gives it, once, `Shunt -1 3`: start the engine, couple to the consist
ahead (Connect, the first order it drives under), go on shunting. It passes when the first coach,
pushed, runs onto `tor1k` (`eventall2`), with the train still moving, and the black box has the
locomotive coupled to the coaches (it couples about 70 s after the start). A driver that leaves its
hand brake on under Connect pulls against it for good and the case ends as a standstill after 300 s.

The `eu07_td_buckle_*` cases (fragment `td_buckle_event.scn`) run the timetable train of
`td_timetable.scn` and buckle the curve `tdo_n222`, about 950 m ahead of it, with a `trackbuckle`
event of `vars.amplitude` at 10:35. Both have `physics.derailment new` in `ini`: the default,
`monitor`, leaves the decision to the legacy draw and only logs what the criteria would do
(`derailment monitor: ... overturned at 46 km/h on track tdo_n222`), so the train runs on to
Przystanek whatever the buckle. With 600 mm over a main lobe of 8 m the locomotive overturns on
the buckle at 46 km/h, 1036 m from the start (`Bad driving: ... derailed on buckled track`); with
30 mm it keeps on the rails and stops at Przystanek after 1400 m.

`sm42_td_uncouple_past_signal` (fragment `td_uncouple_past_signal.scn`) is the uncoupling of the SM42 at Debica on
L053 poranek (step 10 of `deb_sm42_stat`) on td.scn. An SM42 with no timetable stands turned at the far end of `tor1k`,
three wagons loaded with gravel (51 m of train) behind it towards the exit signal `stacja1_j`, which stays at stop (no
route). `tdo_n1734`, the track behind `tor1k`, is closed (`trackvel 0`) from the start. `event0` of `tor1k` gives the
driver `Shunt -1 0`: it pulls the wagons back for 47 m and stops in front of the closed track, with the far end of the
last wagon 2 m past the signal's memory cell (the `distance` check keeps it 0-4 m past). The pull leaves the screw
coupler stretched, 0.019 m apart at the buffers, and a screw coupler is let go only with the buffers pressed together
(under 0.01 m). At the first stand `event0` gives `Shunt 0 0`: brake the wagons, change direction, push the buffers in,
uncouple. After the direction change the driver counts `stacja1_j` as passed at its first move, so its speed is capped
at 0. The driver used to work the brake releaser only while it wanted to speed up, so the locomotive stayed braked by
the wagons' brake application and never closed the gap: no `uncoupled` in the log, and the case ran into its timeout.
Now the locomotive is released, eases into the wagons and uncouples within a second. The wagons are loaded so that the
stop leaves the gap above 0.01 m: with a coach and an empty wagon the buffers came to rest 0.009 m apart and the
coupler let go at once.

The `*_td_crew_*` cases (eu07/maszyna#510, fragment `td_crew.scn`) are about the crew command of a
`putvalues` event, `Crew:<vehicle> <cab> <flags>`: cab 1 or -1 puts an AI driver in cab A or B of
the vehicle named, 0 takes its AI driver off. The vehicle is named in the text, so the events are
set off by event launchers, at an hour or with a delay; `vars.trainsets` are the vehicles and
`vars.crew` the events of the case:

- `eu07_td_crew_add`: an EU07 standing cold with nobody in it gets a driver in cab A 20 s after
  the start. A driver put in a standing vehicle waits for orders, as the one of a standing
  trainset does, so `event0` of its track gives the order to go (`Shunt -1 0`), once: the driver
  starts the locomotive and leaves through the exit signal, whose aspect makes it a train run.
  `eu07_td_crew_add_rear` is the same with cab B: the locomotive leaves the other way, its rear
  end leading (`moves`, from the black box).
- `eu07_td_crew_remove`: the EU07 of `eu07_td_timetable` loses its driver at 10:36, on the run at
  59 km/h. It is left with the power off and the brake valve in the emergency position: it
  stands 16 s and 168 m later and doesn't move in the 164 s to the end (`stops_driverless`; the
  physicslog ends when the driver goes). `en57_td_crew_remove` (driven from the head car, the
  motors in the middle one), `vectron_td_crew_remove` (induction motors and an integrated handle)
  and `sm42_td_crew_remove` (diesel) are the same with other vehicles.
- `eu07_td_crew_swap`: the driver waiting at Stacja1 is taken off after 20 s, which leaves the
  locomotive braked, and a new one put in 20 s later, who gets the order to go, releases the
  brakes and leaves.
- `golf_td_crew_swap`: a car running along the road `d3` at 50 km/h loses its driver, which
  leaves it with its own brake applied in full: it stands 3 s and 17 m later. A road vehicle has
  no cab to tell by and takes a driver all the same: the new one, put in 40 s later, drives on
  along the road, at up to 100 km/h, having no timetable to cap the speed as the first one had
  (the trainset's `50`).
- `eu07_td_crew_refused`: the run of `eu07_td_timetable` with crew events which must be refused,
  at the start and again on the run: a second driver for the locomotive, a driver for a coach
  (no cab), a cab which is none of 1, -1, 0, a driver taken off a coach which has none, and a
  vehicle the scenery lacks. Each leaves its line in the log and the train runs its timetable to
  the stop at Przystanek as it does without them.
- `en57_td_crew_parted`: an EN57-2000 unit with an AI driver and no timetable stands on track 1,
  another one with nobody in it runs into its rear at 100 km/h (`crashdamage yes`). The hit
  breaks the coupler between the middle and the rear car (`parts`, at 2 s), which leaves the rear
  car with the driver in charge of it. The driver taken off at 10:35 had all three cars in charge,
  the parted one too, and none of them keeps it (the log line `the driver was in charge of 3
  vehicles`; the cars still coupled to the head car are two).

A vehicle with a user in it is left alone by the command, which the test mode can't show: nobody
sits in a vehicle there.

`eu07_td_hauls_dead_vectron` is the timetable run with a dead locomotive in the train: behind the EU07 a
Vectron with nobody in it, coupled with the hook and the brake hose alone (the EU07's coupling `3`) and
prepared for hauling, the parameter `H` after its own coupling (`0.H`: the pullee mode of
MaSzyna-EU07/maszyna#64). The checks are those of the timetable cases; the train takes 300 s to
Przystanek and on (20 km/h after 79 s, the stop after 220 s and 1405 m). Without the `H` the Vectron,
which applies the emergency brake while none of its cabs is active, empties the brake pipe of the train
in two seconds, both locomotives stand with their brake cylinders full and the train never moves.
`eu07_td_hauls_dead_vectron_cold` (fragment `td_cold_timetable.scn`) is the same train from a cold start:
trainset velocity 0, so both locomotives stand braked (brake pipe at 3.5 bar, batteries off, the spring
brake of the Vectron switched on), and `event0` of the track gives the driver `Obey_train`, once. The EU07
starts up, charges the brake pipe and leaves on time (the stop after 209 s and 1393 m); without the `H`
the pipe stays below 1 bar and the case ends as a standstill after 300 s.

`eu07_td_reinspect_brakes_after_coupling` (fragment `td_reinspect_brakes.scn`, eu07/maszyna#350 and #282) is about the
wagon inspector of the AI (`TController::AutoRewident()`) on a train formed by coupling. An EU07 stands cold on the line
from Stacja1 (`koniec`) and a cut stands 45 m ahead of it, braked, with nobody in it: 16 coaches with P alone (BDhpumn)
and next to the locomotive 8 empty wagons with G and P (401W), 541 m of wagons. `event0` of the locomotive's track gives
the driver `Shunt -3 3`, once: couple to the cut with the coupler and the brake hose and run it on as a train
(`Obey_train`), waiting for the way, which `event2` of `tdo_n231`, in front of the cut, gives as a `SetVelocity 40 40`
(no signal of the line is in sight). Over 500 m of wagons make it a train on G (Cw-1 §19.1), where the brake of a wagon
without G is cut out (§19.9). The inspector looks at the train as the AI couples, at 4 km/h (t=69 s), too fast for it to
cut a brake out, and leaves that for the next standstill: `reinspect_brakes()` cuts the 16 coaches out as the train stands
a moment later, its brakes applied by the empty pipe of the cut. Neither the physicslog nor the black box has the brakes
of the wagons, so the case tells by how the train brakes: `event2` of `tdo_n240` takes the driver off at 38 km/h, which
leaves the brake valve in the emergency position, and braked by the locomotive and the 8 wagons alone the train stops 29 s
and 211 m later, a mean deceleration of 0.27 m/s² (`stops_driverless` with `max_deceleration` 0.40). With every brake
working it stops at 0.57 m/s², 99 m: that is `eu07_td_reinspect_brakes_legacy_inspector`, the same run with the inspector
of the earlier releases (`ai.brakesetting.cw1 no`), which cuts no brake out, and also what a build whose
`reinspect_brakes()` does nothing gives for the first case, which then fails. The test passes 150 s after the driver goes.

The `*_section*` and `*_converter_interlock*` cases are about the line breaker closed ahead of the pantographs
(`PantRequiresMain=Yes` in `Cntrl.`, MaSzyna-EU07/maszyna#272) and the line contactors that wait for the
converter of their own vehicle (`LineContactorsInterlock=Converter`, #276), driven by the AI on the timetable
run of td.scn. The overhead wire of td.scn is changed with `replace`: one span of it, 61 m some 400 m from
the start (`0.7764 5.845 1561.2417` onwards, over the way out of Stacja1, which the train runs under power at
50-55 km/h), gets no supply and 0 V (`traction none 0`), and in `ep05_td_section_insulator` the span before
it is split around a section insulator 1 m long (`traction * 0`, as sceneries hang insulators). The EP05 has
`PantRequiresMain=Yes` in the game's data; the ET42 has no `LineContactorsInterlock` yet (SVN data of #276),
so those cases run variants of the vehicles made for the run (`vehicle_variants`).

- `ep05_td_timetable`: the EP05 on the plain timetable run, the reference: 20 km/h after 73 s, the stop
  after 205 s and 1400 m.
- `ep05_td_dead_section`: the EP05 runs into the span without voltage under power, nothing on the track
  telling its driver to coast. The traction force drops to nothing at once and the AI puts the master
  controller to 0; the line breaker stays closed (the log
  says the vehicle lost power for 4.05 s, which it says only while the line breaker is closed). Behind the
  section the AI takes power again (535 m, 36 kN by 595 m) and stops at Przystanek (1399 m).
- `ep05_td_neutral_section`: the same with the track under that span, `koniec`, marked as one to coast
  through (`overhead 0`, which `track_events` adds as it adds events): the AI puts the master controller to
  0 as the locomotive runs onto it, 25 m ahead of the dead wire, and takes power again behind it.
- `eu07_td_dead_section`: the old behaviour for reference, an EU07 (pantographs first) into the same span:
  its line breaker opens after 0.2 s (`lost power for 0.21 sec`, no longer), and behind the section the AI
  closes it again and takes power (493 m).
- `ep05_td_section_insulator`: the EP05 runs under the insulator under power at 50 km/h. The pantograph is
  without voltage for some 0.07 s, which the vehicle bridges by holding the last voltage for 0.2 s
  (`TDynamicObject::Update`): the traction force stays between 50 and 58 kN past it and nothing is logged.
  Without the hold the undervoltage relay of a `PantRequiresMain` vehicle would open the line contactors,
  which close again only with the master controller back at its first position. With the insulator 15 m long
  instead (a trial run) the vehicle does lose power there (logged, the force drops to 0), which shows the
  pantograph runs under the piece of wire.
- `et42_td_converter_interlock_cold`: an ET42 with `LineContactorsInterlock=Converter` in both sections
  stands cold (`td_cold_timetable.scn`) and is ordered to run the timetable: the AI starts the converters,
  leaves (20 km/h after 74 s) and stops at Przystanek (1393 m). It and `et42_td_converter_interlock`, the
  same from the timetable start, check that the AI gets such a train going; a plain ET42 passes them too.
- `et42_td_converter_interlock_slow_converter`: the check that the interlock takes effect. The same cold ET42
  with a converter that starts 60 s after it is switched on (`ConverterStartDelay=60`): the line contactors
  wait for it, and the train reaches 20 km/h at 80.0 s; `starts` with `after: 77` fails it earlier. With the
  same converter and no interlock it gets there at 73.8 s, as fast as without the slow converter (a copy of
  the case without the key, run once, not kept).
- `ep05_td_interlock_dead_section_cold`: #272 and #276 together, an EP05 with `LineContactorsInterlock=Converter`
  added, standing cold, with the span without voltage on its way: the pantographs can go up only through the
  closed line breaker, so the AI has to close it first, then start the converter; it leaves (20 km/h after
  77 s). Under the dead wire the converter stops with the main circuit, behind it the converter starts again
  and the AI takes power (573 m, 4 s later than the EP05 without the interlock) and stops at Przystanek
  (1393 m).

What these can't reach: the local converter switch of the ET42 (`converterlocal_sw:`), which the AI has to put
back on (`LineContactorsHeldByLocalConverterSwitch()`, the part of #276 in `determine_consist_state()` and
the `converteron` hint), is in the cab only: no command of a scenery or of the train's cables sets
`ConverterAllowLocal`, and nobody sits in a cab in the test mode, so the switch is always on in a case.

The `eu07_td_switch_*` cases (fragment `td_switch_thrown.scn`, the review of #289, point 2b) throw a switch of
Stacja1 by an event under an EU07 with three coaches and check that the train keeps to the rails (`on_track`) and that
the AI keeps to the route it is really on. Both switches of the station have their points towards the line and the
way to track 2 straight (position 0), to track 1 curved (position 1); the ends of the two ways are 1.9 m apart, the
tracks 4.6 m. `zwr01` at the west end is run through facing by a train running into the station, `zwr02` at the east
end trailing by one leaving it. The throw is an event set off once, by the rear axle of the locomotive running onto a
track (`event1`/`event2` of it, which only the vehicle with the driver sets off) or by a launcher 5 s after the start;
a driver that stands cold gets its order (`vars.order`) from `event0` of its track, after the throw where the train
stands over the switch. On track 1 a signal at stop of the case's own stands at z 1100: the train has to stop before
it (`event0` of `tdo_n1725` passes the test, running onto `tdo_n1734` fails it); on track 2 the exit signal `stacja1_k`
at stop. What they showed on `staging` of 2026-11-22 (f15ea405), where an axle on a switch keeps the way it ran onto
(the segment it took when it got onto the switch) but leaves the switch by the way it is set to now, and the AI does
not scan its route again for a switch that changed behind the head of the train:

| case | throw | result |
| --- | --- | --- |
| `eu07_td_switch_facing_control` | none, set to track 1 from the start | passes: stops before the signal of track 1 |
| `eu07_td_switch_thrown_facing_head` | `zwr01` to track 1 with the locomotive on the switch | `xfail`: the locomotive jumps sideways at the end of the switch (0.78 m off the track in the black box), the whole train goes to track 1, and the AI, still on the route of track 2, runs past the signal of track 1 at 40 km/h |
| `eu07_td_switch_thrown_facing_behind` | `zwr01` to track 1 with the locomotive just past it | `xfail`: the locomotive runs on along track 2 and the coaches coupled to it along track 1, side by side, for 900 m |
| `eu07_td_switch_thrown_standing_facing` | `zwr01` to track 1 under the standing train, which then leaves facing | `xfail`: the second coach runs with a bogie on each track, its body 2.3 m off both; the AI, which scans from the last vehicle, reads the signals of track 1 while it runs on track 2 (a signal passed at danger, `stacja1_j`) |
| `eu07_td_switch_thrown_standing_trailing` | `zwr02` to track 1 under the standing train, which then leaves trailing | passes: the first wheels coming from track 2 push the switch back (as through any switch set against them from the frog end) |
| `eu07_td_switch_thrown_trailing` | `zwr02` to track 2 under the coaches of the timetable train leaving track 1 | passes: the switch is pushed back, the train stops at Przystanek after 1402 m |

Nothing derails in any of them with `physics.derailment monitor` (the default; the monitor reported nothing either),
and the three `xfail` cases run with `physics.derailment new` came out the same to the metre: the criteria of
derailment look at the curve, the cant and the speed of a vehicle, not at its bogies being on two tracks. Nor does the
coupler between a locomotive and a coach 4.6 m beside it break.

### Cases on a mission of a scenery

The cases made from players' reports (eu07/maszyna#421, #506, #524, #525) run a whole mission
of a scenery of the game, among its own traffic: `strip_trainsets` `false`, `vehicle` the
mission's own locomotive or unit, every train driven by AI as always in the test mode. They are
`slow`, have `physicslog` `false` unless a check needs the black box, and `file.binary.terrain no`
in `ini`, so that no geometry cache of the copied scenery (hundreds of MB for these) is written
into the game dir. What they add to the scenery:

- nothing, when standing too long is the bug: `test.standstill` is set above the longest wait
  the mission has, and the run ends with `duration` (`baltyk_skm1_*`),
- `mission_state_probe.scn`: these sceneries steer each station with a state cell per train
  (text and two values, e.g. `deb_tme222158_stat` = `a 6 *`: the route of step 6 is set). The
  probe is a loop of two events that looks at the cell `vars.cell` every 5 s and reports the
  pass once it holds `vars.state` (a `memcompare`: `* 6 *`, `d * *`), with `vars.reason` as the
  text. No fail event: a train that never gets there ends the test with the standstill or the
  timeout. A periodic event launcher wouldn't do, the simulator polls those near the camera only,
- events named `<track>:event0`, `:event1`, `:event2` or `:eventall1`...: the simulator binds an
  event with such a name to the track by itself, so a fragment can hang events on tracks that
  live in the scenery's includes, where `track_events` doesn't reach (`wrzosy_pxe336080_couple.scn`).
  The name must not be one the scenery has already: a second event of the same name replaces the first.

A train whose trainset has velocity 0 has a driver that sleeps in `Wait_for_orders` when nobody
is in the cab: the player would drive it, so the fragment has to give the orders the player's
hands stand for, with `PutValues` from an event the vehicle itself sets off (`event0` of the
track it stands on). The first of them may be the one to couple (`Shunt -2 -3`): a driver whose
first order is `Connect` releases the hand brake a cold locomotive starts with
(`eu07_td_cold_connect`). The mission trains of velocity 0.1 wait for their signal and need nothing.

The cases and what they showed on master of 2026-11-11 (the two L053 poranek rows also on
staging f15ea405 of 2026-11-23):

| case | report | result |
| --- | --- | --- |
| `wrzosy_pxe336080_couples` | #524 | passes: ordered to, the locomotive couples to the rake (`couples`, 262 s) and takes it out of the depot (696 s). The scenery itself gives no such order |
| `baltyk_skm1_roj55324_shunting_signal` | #525 | passes, with four seeds: the unit on the siding is let into the platforms 4 minutes after the start, once the shunter beside it has done its first moves, and leaves at 10:42 |
| `baltyk_skm1_roj55321_departure` | #525 | passes, with three seeds: the unit in the platform leaves at 10:34 |
| `l053_poranek_tme222158_debica` | #421 | passes: TME222158 gets the exit signal to Sandomierz (step 6 of `deb_tme222158_stat`) after 4086.2 s. On master of 2026-11-11 the station jammed 4 minutes after the start, the AI of EP07-338'16 passed the shunting signal Tm4 at stop at 29 km/h; #409 fixed that. On staging f15ea405 (2026-11-23) the station's SM42-1096 jammed it next: ordered to uncouple at step 10 of `deb_sm42_stat`, with its cut standing across `debica_k_sem_info` at stop, it stayed braked and never got the coupler off (standstill after 4347 s), until the AI kept the locomotive released while it pushes the buffers in to uncouple (`sm42_td_uncouple_past_signal`) |
| `l053_poranek_tme222158_debica_et21` | #421 | passes after 4131.6 s. On master of 2026-11-11, with an ET21 as in the report, the locomotive stopped for good at Debica Towarowa, on the headshunt 20 m past the end of the overhead wire, and its driver never got out of `Prepare_engine`. Since #409 the ET21 gets through Debica Towarowa (steps 1-5 of `dt_tme222158_stat`) to Debica; there it waited for the SM42 as `l053_poranek_tme222158_debica` did, until the same fix |
| `l053_poludnie_roj32134_debica` | #506 | passes: both trains run and the unit ends at Debica after 6134 s; no train in its way |

In the L053 sceneries the test mode logs a signal passed at danger for several shunting
locomotives within minutes, so these cases have `test.fail_if_stop_overran no`.

The simulation ran at 15 times real time on Baltyk and at 5-8 times on L053 (with another
build going on on the machine) with the physicslogs off, and at 3-4 times on Wrzosy with them
on (250 drivers, the road vehicles included: 740 MB of physicslogs in 900 s), after 30-150 s of
loading.

Checks (speeds in km/h, times in seconds of physicslog time, distances in metres). Most use
the physicslog Velocity column, which is the wheel speed: it stays at 0 while the wheels are
locked in a skid. The ones marked *vehicle speed* use the vehicle speed column at the end of
the row instead, logged by builds from this branch on.

- `starts` `min_speed`, `within` (`after`): reaches `min_speed` within `within` seconds (and not before `after` seconds,
  when given).
- `max_speed` `limit`, `tolerance`: never goes faster than `limit + tolerance`.
- `brakes` `min_pressure`: after moving off, brake cylinder pressure reaches `min_pressure` bar
  (physicslog pressures are in bar whatever its header says; the brake pipe runs at 5.0).
- `stops` `moving_speed`, `stop_speed`, `hold`, optional `min_distance`, `max_distance`, `max_time`,
  `nth`: after reaching `moving_speed`, stays at or below `stop_speed` for `hold` seconds; with the
  distances, that stop has to be between them (metres travelled, as in `distance`), with `max_time`
  it has to begin by then (seconds of the physicslog). `nth` (default 1) takes a later stop: each
  stop after the first counts once the train has reached `moving_speed` again.
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
- `keeps_moving` `moving_speed`, `min_speed`, `until_distance`, optional `from_distance` (*vehicle speed*): once at
  `moving_speed`, the speed stays at or above `min_speed` between the two distances (metres travelled, from the vehicle
  speed), e.g. a heavy train doesn't come to a stand on a descent before a restriction. Reports the lowest speed.
- `starts_again` `after`, `min_speed`, `within`, optional `max_rollback` (*vehicle speed* for the rollback): standing
  at the time `after` (a signal that clears then), the train reaches `min_speed` within `within` seconds, and rolls back
  no more than `max_rollback` metres meanwhile.
- `log_contains` `pattern`: a line of the run's `log.txt` matches the regex, e.g. the
  `Bad driving: ... collided` message of a collision.
- `log_lacks` `pattern`: no line of the run's `log.txt` matches the regex.
- `couples` `vehicle`, `other`: the black box of the run has the two vehicles coupled to each
  other (the coupler itself, whatever else of the hoses and the cables) at some time after the
  start, and still coupled at the end. The vehicles go by their node names, with `{{vehicle}}`
  filled in as in the fragment, e.g. `{{vehicle}}_rb`. Fails when the two were coupled from the
  start, came apart again, or the run left no black box. The black box is looked at once a
  second, so the time given is that of the first look after the couplers caught.
- `parts` `vehicle`, `other`, optional `before`: the black box has the two vehicles, coupled to
  each other, come apart (the coupler itself let go: broken, or uncoupled), by the time `before`
  of the recording when given.
- `driver_changes` `vehicle`, `to`, optional `after`, `before`: the black box has the vehicle get
  the driver `to` (`none`, `ai` or `user`) at a time of the recording between the two, having had
  another one before, and keep it to the end.
- `stops_driverless` `vehicle`, `min_speed`, `within`, optional `stop_speed` (0.5), `max_creep`
  (1 m), `hold` (60 s): the black box has the vehicle lose its driver at `min_speed` or faster,
  stand `within` seconds later and stay within `max_creep` of where it stopped for the rest of
  the recording, or until it gets a driver again, at least `hold` seconds. The physicslog can't
  tell: the driver writes it, so it ends when the driver goes. Optional `min_deceleration`,
  `max_deceleration` (m/s²): the mean deceleration of that stop, from the speed the driver left at
  and the way to the stop (v²/2s), has to be within them, e.g. a train some of whose brakes are
  cut out stops slower.
- `moves` `vehicle`, `min_speed`, optional `direction` (`front`, or `rear`): the black box has the
  vehicle reach `min_speed` with that end leading (the physicslog speed has no direction).
- `traction` `from_distance`, `to_distance`, and one or more of `reaches`, `at_most`, `at_least`: the
  traction force in kN (the physicslog has it in N, whatever its header says) while the vehicle runs between
  the two distances (metres travelled, as in `distance`): its highest there reaches `reaches`, e.g. the power
  taken again behind a section without voltage; it stays at or below `at_most`, e.g. none under a dead wire;
  it stays at or above `at_least`, e.g. no power lost under an insulator.
- `master_controller` `from_distance`, `to_distance`, `at_most`: the master controller stays at or below the
  position between the two distances, e.g. at 0 where the scenery tells the driver to coast.

- `on_track` `vehicles`, optional `max_offset` (0.75 m), `max_detour` (5 m): every vehicle whose name starts with
  `vehicles` (e.g. `{{vehicle}}`, the train of the case) keeps to the rails in the black box, in each of its states:
  its middle is within `max_offset` of the line of a track (level distance, the map of the tracks of the black box),
  and of two vehicles coupled to each other the way along the tracks from one to the other (through the ends of the
  tracks that meet) is no more than `max_detour` longer than the straight line between them. A vehicle with its
  bogies on two tracks that part, or thrown sideways, is off the line; a coach running on beside the locomotive it is
  coupled to, on the next track, has a way to it hundreds of metres long. A coach on a curve of 300 m, through the
  S of a switch, was up to 0.38 m off. The middle of a vehicle is the middle of the chord between its pivots, so on
  a curve of radius R a vehicle running right is a^2 / 8R off the track, a being the distance of the pivots: a 19.5 m
  coach 0.59 m at R 80, 0.79 m at R 60. The default suits curves down to about R 65 for such a coach; a case with
  tighter curves sets more. The black box lists a vehicle once a second, so a jump between two lists shows up only
  as it is at the next one.

New check types go in `CHECKS` in `run.py`; each gets the physicslog rows and the check's
parameters and returns `(passed, detail)`. Checks on the log go in `LOG_CHECKS` and get the
text of `log.txt` instead of the rows. Checks on the black box (`blackbox_doc.md` in the root of
the repo; the simulator writes it along with the physicslogs) go in `BLACKBOX_CHECKS` and get
what `read_blackbox()` read of it: the names of the vehicles, the coupling records, and the
states of each vehicle from the `vehicles` records (position, speed, driver; a vehicle is listed
when it moved, changed speed or got another driver, and all of them every 60 s). A check
on another kind of record has to add it there.
