# Self-tests of the track editor

Scripts that drive the track editor through the commands of its self-test (`application/editormodeselftest*.cpp`) and check
what came out with `expect` commands. A failed expectation makes the run fail: the editor ends with a non-zero exit code.

The game data isn't in this repository, so these run against a local game install only, not in CI.

## Running

The editor has to be built with the self-tests, which a normal build leaves out (a script takes over the input of the editor):

```
cmake -S . -B build -DWITH_EDITOR_SELFTESTS=ON
cmake --build build --config RelWithDebInfo
```

Then, with the game folder (one with `scenery/`, `models/` and `dynamic/` of the game data):

```
python tests/editor/run.py --game-dir C:/MaSzyna                       # all cases
python tests/editor/run.py --game-dir C:/MaSzyna 1_inspector_cant      # chosen cases
python tests/editor/run.py --game-dir C:/MaSzyna --ini low.ini         # with settings of your own over the runner's
```

- `--exe` names the executable; by default it's the newest `eu07*.exe` under `build/bin`.
- `--ini` is read over the runner's settings (a 640x480 window, no sound, `convertmodels 0`, `log.txt` in the game folder),
  which are read over the player's own `eu07.ini`. A file with low settings of the picture makes the runs quicker.
- `--timeout` (240 s) and `--memory-mb` (8192 MB) limit each run of the editor. On Windows the editor runs in a job object
  which keeps its memory under the limit and ends it with the runner; the runner also stops it when it goes over the limit.
- `--show` shows the window of the editor; without it the window is never shown nor focused (`EU07_EDITOR_SELFTEST_HIDDEN`),
  so the computer can be used meanwhile.
- `--keep` keeps the copies of the scenery, to look at what a script saved.

The cases run one after another, one editor at a time. Each prints the exit code of the editor, its time and memory, the
`SELFTEST FAIL` lines, and `PASS` or `FAIL`; a summary follows. The runner exits with 1 when any case failed. The scripts as
they were run and the logs of the editor are in `tests/editor/out/<case>/`.

## How it works

For each case the runner:

1. copies the fixture, `scenery/autotest_editor.scn`, into the `scenery` folder of the game as `autotest_editor_<case>.scn`,
2. starts `eu07 -ini <runner's settings> [-ini <--ini>] -edit autotest_editor_<case>.scn` in the game folder, with the script in
   `EU07_EDITOR_SELFTEST` (the settings first: a `sceneryfile` in them would take the place of the scenery otherwise),
3. waits until the editor quits, or stops it at the limits of time and memory,
4. reads the exit code and the `SELFTEST` lines of `log.txt`,
5. deletes the copy of the scenery and whatever the editor wrote next to it under its name, and says what else the run added
   to the `scenery` folder or changed there.

A case passes when every run of the editor ended with the exit code 0, its log has `SELFTEST result: 0 failed of N`, and the
case checked something (N above 0 over all its runs). The editor ends with 2 when an expectation failed or a command of the
script wasn't taken by any group of the self-test (a typo would otherwise let the expectations after it pass for nothing),
with 1 when it couldn't start or an error ended it, and with the system's code on a crash.

The fixture holds plain tracks only, no terrain: lines of straights (`a`, `s`, `d`, the pair `p` and `q` 4.5 m apart), a track
of 1000 mm gauge (`g1`) beside one of the standard gauge (`g2`), a turntable of 16 m with `velocity 10` (`tt1`, with the
include of its model and events), two curves of 600 and 604.5 m side by side with a cant of 60 mm (`i0`..`i9`, `o0`..`o9`,
the straights at their ends taking the cant on and off), a straight taking a cant of 60 mm on along its middle track (`r1`..`r3`),
a curve without cant of 60 m of R 2000 m and 60 m of R 400 m turning the same way between straights (`k0`..`k3`), and a track
apart with a vehicle (`v1`), without which the editor would put one of its own onto a track of the cases on saving. It uses only what every install of the game has.

## Writing a case

A case is `cases/<name>.txt`: one command a line, `#` for comments, as for `EU07_EDITOR_SELFTEST`. The commands are listed at
the top of `application/editormodeselftest.cpp`. Start with a `wait` of a few frames and end with `quit`, which logs
`SELFTEST result: <failed> failed of <expectations>`.

The expectations (`expect <what> ...`), each logged as `SELFTEST PASS <what>` or `SELFTEST FAIL <what>: got <value>, expected <value>`:

| expectation | checks |
| --- | --- |
| `expect cant <track> <start\|end> <mm> <tolerance mm>` | the cant at the end of the first path, as the inspector shows it (1500 mm between the rail axes) |
| `expect joint <track> <start\|end> <gap m> <angle deg> <cant step mm>` | the end of the first path against the nearest end of another track (within 1 m): the gap, the kink and the step of the cant between them, each at most the value given |
| `expect records <name> <count> [kind]` | the records of the switches (`//$r`) of the name, with the connections having the switch among theirs; only those of the kind (`switch`, `slip`, `crossing`, `double`, `connection`) when given |
| `expect speed <track> <km/h> [tolerance]` | the speed of the track as the editor shows it (`editor_track::velocity`) |
| `expect gauge <track> <m> [tolerance]` | the gauge of the track |
| `expect track <name> <present\|removed>` | a track of the name is in the scenery, or isn't |
| `expect route <track> <in\|out>` | the track is a part of the line open in the route tool, or isn't |
| `expect norms gauge <mm> [tolerance]` | the rail axes distance of the norms the tracks are laid to |
| `expect height <track> <start\|end> <m> <tolerance>` | the height of the end of the first path |
| `expect lift <track> <start\|end> <label> <m> <tolerance>` | how far the end is above the height `mark <label> <track> <start\|end>` kept |
| `expect includes <track> <count>` | the includes whose parameters name the track: the model and the events of a turntable |
| `expect status <text>` | the status of the switch tool, as its panel shows it, holds the text (the rest of the line) |
| `expect statuslacks <text>` | the status of the switch tool doesn't hold the text |
| `expect refusal <holds\|lacks> <text>` | what the cursor tells while it's shown (a refusal or a warning) holds the text, or doesn't |
| `expect statusline <holds\|lacks> <text>` | the status line of the editor (also shown in the layers window) holds the text, or doesn't |
| `expect laycursor <x> <z> <tolerance>` | the point the line of the lay tool goes to from its last point clicked, in the plan |
| `expect terrain <grow\|area\|prompt> <0\|1>` | the terrain model is made under the tracks as they're laid / around the camera within the download area / the question of the terrain of a new scenery is open |
| `expect laypanel <ready\|clicking\|laid\|extending> <shown\|folded>` | the state of the panel of the lay tool, as the step at its top tells it, and whether its settings are shown or folded to the line of their summary |
| `expect switchpanel <ready\|dragging\|placed\|selected>` | the state of the panel of the switch tool, as the step at its top tells it |
| `expect steps <count>` | the steps of the history Ctrl+Z takes back, the ones made together counted once: none made by a change refused |
| `expect tool <name>` | the tool of the surroundings in use: `select`, `insert`, `brush`, `fill`, `copy`, or `track` while the field of work of the tracks is |
| `expect window <shortcuts\|settings\|weather\|orthophoto> <0\|1>` | the window of the shortcuts (F1), of the settings of the editor, of the weather and the time, of the settings of the orthophoto is open |
| `expect selected <x> <y> <z> <tolerance>` | the node selected stands at the point |

With the editing on the model (`modelediting 1`, the setting for the run only), a track may be named `@<line>.<index>`: the track of
the line of the model at the index from its start. The commands `modelplan <track>` (the route design opened on the line),
`routeradius|routecant <vertex> <value>`, `routemove <vertex> <x> <z>`, `modelsplit <m>`, `modeljoin <x> <z> <x> <z>`, `modeldelete`
(of the line open in the route design), `modelselect|modeldel <track>`,
`modelanswer <yes|no>` (the window asking), `modelmark <label>`, `modeltracks`; and the expectations:

| expectation | checks |
| --- | --- |
| `expect managed <track> <line\|none\|any>` | the track is made by the model of the line, or isn't |
| `expect modellines <count>` | the lines of the model |
| `expect projectfile <present\|absent>`, `expect projectfile lines <count>` | the project file next to the copy of the scenery, the lines it holds |
| `expect modelends` | the tracks of each line end at the numbers of its nodes and at each other, to the bit |
| `expect modelrefused <0\|1>`, `expect modelmessage <holds\|lacks> <text>` | the last command of the model was refused / what it told, its notes of the norms with it |
| `expect modelvertex <line> <index> <radius\|in\|out\|cant> <value>` | a value of a curve of a line of the model |
| `expect modelsame <label>` | the lines and nodes of the model are the ones `modelmark` kept |
| `expect hand <line> <0\|1>` | the line is taken for changed by hand |
| `expect question <none\|hand> [line]` | the window asking the user |

A line `#> reopen` ends a run of the editor there (the part before it saves and ends with `quit`) and starts the editor again
on the scenery as it was saved, for what has to be checked after a save and a load. The editor's own `openscenery` starts a
new editor without the script, so it isn't for the cases.

## The cases

The cases check the fixes of PR #450 (the numbers are those of its test plan):

| case | fix |
| --- | --- |
| `1_inspector_cant` | the cant typed into the inspector goes on into the track joined at that end, in the same undo step |
| `3_norms_after_delete` | deleting a track of the line open in the route tool keeps the laying norms |
| `4_gauge_paste_group` | the gauge is a paste group of its own, off by default |
| `5_turntable_speed` | the speed of a turntable set in the editor is the one saved |
| `6_set_delete_records` | deleting a set of tracks takes the records of its switches, and the model and the events of a turntable |
| `6b_set_delete_whole` | a double switch, a slip and a crossover in a set go whole, with their records |
| `7_crossover_delete` | deleting a switch of a crossover on a curve takes the crossover out whole: the outer track comes down, the record goes |
| `7b_bent_switch_delete` | a switch bent out of the shape of its record still takes the record along when deleted |

and the quick fixes of the switches (Q2 of the plan of the track editing):

| case | fix |
| --- | --- |
| `q2_1_switch_speed` | a switch laid, or given another template, gets the speed of its diverging track written: of its type, bent into a curve with the cant taken with its sign and the cant deficiency of its frog |
| `q2_2_switch_refusals` | a switch with the cant changing within it or a ramp ending nearer than 6 m, a double switch or a slip bent, or the curvature of its diverging track turning from one side to the other isn't laid; 1:9.403 bent is only warned of |
| `q2_3_switch_move` | a switch moved along its curve or its straight is judged as when it's laid: moved too near a cant ramp, or onto one, it stays |

and the first steps in the editor (P1 of the plan of the editor's interface):

| case | fix |
| --- | --- |
| `fs_1_preview_over_panel` | while the pointer is over a panel the line the lay tool shows stays as it was, no leg following the pointer under the panel; back over the view it follows again |
| `fs_2_switch_refusal_cleared` | a switch refused is told in the panel of the switch tool and by the cursor, not in the status line as well, and what it told goes once the next switch is laid |
| `fs_3_without_terrain` | the answer Without terrain to the question of the terrain of a new scenery turns off the terrain made of the terrain model by itself: under the tracks as they're laid and around the camera within the download area |

and the consistency of the interface of the editor (P2 of the plan of the editor's interface):

| case | fix |
| --- | --- |
| `c_1_bank_card_arms_insert` | a model of the node bank clicked with the select tool of the surroundings arms the insert tool with it; inserted where there's no ground under the cursor it stands at the height 0 where the cursor points, and the status says where |
| `c_2_esc_closes_shortcuts` | Esc closes the window of the shortcuts, as F1 does, also in the tracks |

and the flow of the lay and the switch tools, a step at a time:

| case | fix |
| --- | --- |
| `lf_1_lay_panel_states` | the panel of the lay tool goes from ready, its settings shown, to clicking, its settings folded to a line, to the line laid told over the settings, and back to ready when it's taken back; the same in the mode of a curve traced |
| `lf_2_switch_panel_states` | the panel of the switch tool stays ready after a switch refused, is placed after a switch laid, and tells a switch selected again after another one was laid |

and the editing on the model of the track geometry (E4a of the plan of the track editing), with the setting on:

| case | checks |
| --- | --- |
| `m_0_setting_off` | the setting off, a line laid is an old track and no project file is written |
| `m_1_lay_managed` | a line laid in the straights is a line of the model, its tracks marked as its, the project file written with the scenery, the ends to the bit before and after a reopen |
| `m_2_plan_refusal` | a curve changed in the route design past the norms is refused with the value, the limit and the paragraph, nothing changed; within them, laid in one step |
| `m_3_model_undo` | Ctrl+Z takes a command of the model back with its tracks, Ctrl+Y brings it again, a split taken back; the project saved after an undo has the line as it was |
| `m_4_hand_change` | a track of a line changed by another tool asks before the line is laid anew: kept, nothing changes; overwritten, laid anew; saved, found after a reopen |
| `m_6_split_join` | two lines joined with a new one, split and deleted from the route design, each a step |
| `m_7_plan_converts` | the route design applied to old tracks makes them a line of the model; Ctrl+Z brings the old tracks back |
