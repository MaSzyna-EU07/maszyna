# Coverage of the simulation core by the unit tests

These tests characterise the physics of the vehicles (`McZapkie/Mover.cpp`, `hamulce.cpp`, `Oerlikon_ESt.cpp`, `friction.cpp`). They exist to show that refactors of those files, such as splitting `TractionForce` or `RunCommand`, leave the behaviour as it was. This page lists:

- what the tests reach;
- what nothing in the game ever runs;
- what only data no game vehicle has would run;
- what is left to test;
- the behaviour the tests found that may deserve a look.

## How it was measured

Coverage was measured on Windows with MSVC's SanitizerCoverage (`/fsanitize-coverage=trace-pc-guard`) on the unit test binary. The counts are basic blocks, which comes close to branch coverage: both sides of a condition are blocks of their own. The tool is local and not part of the build, by decision (2026-10). Any block-coverage tool run over `eu07_unit_tests` gives comparable numbers.

| File | Blocks run, at the start | Now |
|---|---|---|
| `Mover.cpp` | 8.9% | 80.7% (3323 of 4118) |
| `hamulce.cpp` | 30% | 76.3% (649 of 851) |
| `Oerlikon_ESt.cpp` | 58% | 87.4% (118 of 135) |
| `friction.cpp` | 9% | 90.9% (10 of 11) |

The aim is branch coverage with the rest explained below, not 100%.

The numbers above are from 2026-11-01. Since then the **Coverage** workflow (`.github/workflows/coverage.yml`) measures every pull request and every push to master: a GCC build of the unit tests with gcov, lines, functions and branches per source file of the core library. The totals and the per-file table are in the summary of the workflow run, and the HTML report with the lines marked is in its `unit-test-coverage` artifact. Its counts are source lines and gcov's branches, so they differ somewhat from the blocks in the table. The report isn't sent to SonarQube Cloud: the project's quality gate would then require 80% coverage of the new code of every pull request.

## Code nothing in the game runs

Nothing creates these objects or calls these functions anywhere in the game, which was checked against the whole source tree. A refactor can delete them, or leave them untested.

- **Brake valves that never get created:**
  - `TESt`, `TESt3` and `TESt3AL2`, including `Init`, `PLC` and `SetLP`. The valve names ESt3, ESt3AL2 and ESt4 all create `TNESt3`.
  - `TESt4R::GetPF`, overridden by `TLSt` and never called by it.
  - `TESt::CheckState`, `CVs`, `BVs`, `GetPF` and `EStParams`, and `TNESt3::EStParams`.
- **`TFV4a`** (`GetPF`, `Init`): the FV4a handle of the vehicle definitions creates `TFV4aM`.
- **Base implementations always overridden:** `TFricMat::GetFC` (every material, the default P10 too, has its own) and `TPrzekladnik::Update` (every relay has its own).
- **Functions without callers:**
  - In `hamulce.cpp`: `PF_old`, `TBrake::GetBCF`, `TDriverHandle::OvrldButton`, `TReservoir::pa` and `TBrakeCyl::pa`.
  - In `Mover.cpp`: `BrakeVP`, `PipeRatio`, `RealPipeRatio`, `IncBrakeLevel`, `DecBrakeLevel`, `IncBrakePress`, `DecBrakePress`, `MainCtrlActualPowerPos` and `GetExternalCommand`.
- **Inverter failures** of the induction motor drives (`inverter::Failure_Drive`, `Failure_Const`, `Activate`): nothing ever sets them. The cab only shows them. The branches reading them in `TractionForce` can't be reached.

## Code only data no game vehicle has would run

Where it was worth it, a test fits the missing parameter to a real vehicle or builds a definition with it (`definition::set()`). Those cases are marked "tested" below. The rest is reachable only by a vehicle definition the game data doesn't have:

- **Tested, fitted or handmade:**
  - The valves CV1, CV1_R, CV1_L_TR, the W, K and Hik variants, SW, NESt3 and M483 (`brake cycle: the valves the game knows…`).
  - An Oerlikon valve named with -ED or AL2 without ESt3 (its relays `TPrzekED` and `TPrzekCiagly`).
  - The brake materials FR513, FR510, P10yBg and P10yBgu (`brake cycle: every material…`).
  - The handles test, D2, Westinghouse, MHZ_EN57 and St113 fitted to an EU07.
  - A `CompressorList`, a `PmaxList` and a `Headlights` section.
  - The rear cab's compressor limits (`MinCP_B`, `MaxCP_B`).
  - The controller of the electrodynamic brake apart from the local brake (`SplitEDPneumaticBrake`).
- **Not tested:**
  - The retarder's clutch (`R_IsClutch`, `R_ClutchSpeed`) and the retarder worked by the local brake (`R_WithIndividual`).
  - The automatic relays of type 2 (`RelayType=2`).
  - The variant of `CheckLocomotiveParameters` for a definition whose `RList` doesn't match its powered axles.

## Reachable, not tested yet

These are run by the game, by vehicles it has or by the cab, the AI or the scenery, but no test reaches them yet. Most of them are the far sides of conditions inside functions the tests do run.

- **Entry points from outside the core:**
  - `TBrake::ForceEmptiness` and its overrides: a scenario's vehicle set up with empty brakes (`DynObj.cpp`).
  - `TBrake::GetSoundFlag`: the sounds of the brakes.
  - `LoadFIZ` and `reload_FIZ`: the file loading around `LoadFIZ_Text`, which the tests call directly.
  - `TSecuritySystem::set_cabsignal_lock`.
  - `TMHZ_EN57::GetEP`: an MHZ_EN57 handle in the EP mode of the brake.
- **Inside the functions the tests run** (blocks run of the function's total):
  - `TractionForce` (305 of 373): the inverter failures above. The rest are mostly halves of conditions:
    - the `SST` table of the series-excited diesel-electrics;
    - the speed control's brake intervention with the air under `BrakeInterventionVel`;
    - a diesel-electric's generator tripping on overcurrent;
    - the `ffBrakeList` of the 22WEc, whose electrodynamic brake no test applies.
  - `AutoRelayCheck` (87 of 119): the ET22's group switch under some sequences, the relays at reverse with `BackwardsBranchesAllowed`, and the shunting of the EZTs.
  - `update_doors` (134 of 160), `PantographsCheck` (44 of 64) and `PowerCouplersCheck` (70 of 88): door modes and pantograph valve kinds only some game vehicles have, and the 24 V circuit across couplers.
  - `RelayReset` (29 of 43) and `MainSwitchCheck` (16 of 27): relays tripping in states the drive cycles don't reach.
  - `UpdatePipePressure` (108 of 130): the pressure switch lock of an electric unit's pantographs, which the EW58's automatic valve keeps away. Also the EP mode of the EN57's handle and the `LockPipe` thresholds (`LPOn`, `LPOff`), which no vehicle sets.
  - `TCV1L_TR::GetPF` (9 of 20), `TKE::GetPF` (28 of 37) and the MHZ handles: settings (G with the CV1L_TR, the KE's anti-slip pressure `ASBP`) that no real vehicle has together.

## The rest of the core library

The core library (`eu07_core`) and the test binary hold more than the physics: the parser of the game's text files, the helpers of `utilities.cpp`, the PLC of a vehicle, the parts of the vehicles and of the simulation moved out of the big classes to be tested, and the hosts of the script screens. The sections above don't cover them. Their numbers are lines run, from the report of the Coverage workflow.

The tables show the run before the tests of the parser, the helpers and the PLC (`test_parser.cpp`, `test_utilities.cpp`, `test_ladder_logic.cpp`) were written. What those tests add is measured by the Coverage run of the pull request which brings them, and shown in the summary of that run.

**The run measured:** pull request #325 on 2026-11-11 (its commit `33cedc45` merged into master `10c7527a`), GCC 13.3, unoptimised, gcov lines. All 49 files of the game in the report together: lines 84.4% (10305 of 12204), functions 85.0% (927 of 1090), branches 62.3% (10325 of 16578).

| File | Lines run | Functions run |
|---|---|---|
| `McZapkie/couplerphysics.cpp` | 93.6% (147 of 157) | 20 of 21 |
| `scripting/ladderlogic.cpp` | 4.1% (9 of 217) | 2 of 11 |
| `scripting/ladderlogic.h` | 0% (0 of 25) | 0 of 4 |
| `scripting/luaprogram.cpp` | 74.7% (334 of 447) | 50 of 66 |
| `scripting/scriptscreendrawing.cpp` | 93.8% (135 of 144) | 19 of 19 |
| `scripting/scriptscreenkernels.cpp` | 97.7% (129 of 132) | 11 of 11 |
| `scripting/scriptscreenrules.cpp` | 100% (50 of 50) | 7 of 7 |
| `scripting/scriptscreens.cpp` | 82.8% (216 of 261) | 28 of 33 |
| `scripting/wasmprogram.cpp` | 54.0% (157 of 291) | 35 of 49 |
| `simulation/simulationtestrules.cpp` | 97.4% (75 of 77) | 10 of 10 |
| `utilities/parser.cpp` | 40.0% (170 of 425) | 18 of 41 |
| `utilities/parser.h` | 97.8% (44 of 45) | 11 of 15 |
| `utilities/utilities.cpp` | 28.2% (82 of 291) | 15 of 49 |
| `utilities/utilities.h` | 100% (41 of 41) | 12 of 12 |
| `utilities/framepacer.h` | 100% (17 of 17) | 2 of 2 |
| `vehicle/blackbox.cpp` | 100% (331 of 331) | 41 of 41 |
| `vehicle/enginestart.cpp` | 96.2% (50 of 52) | 6 of 6 |
| `vehicle/handover.cpp` | 89.2% (33 of 37) | 5 of 5 |
| `vehicle/horn.cpp` | 100% (5 of 5) | 2 of 2 |
| `vehicle/linebreaker.cpp` | 100% (6 of 6) | 1 of 1 |
| `vehicle/smoke.cpp` | 100% (12 of 12) | 2 of 2 |
| `vehicle/gaugescale.h` | 97.9% (47 of 48) | 5 of 5 |
| `world/TrackUneven.h` | 100% (111 of 111) | 23 of 25 |

Notes to the table:

- The small headers with the logic of a single part are all run in full: `vehicle/doorwings.h`, `gaugeduplicates.h`, `gaugepositions.h`, `holdcoillever.h`, `lightsprogrammer.h`, `mainctrlact.h`, `mirrors.h`, `scripting/scriptscreendrawing.h`, `scriptscreenkernels.h`, `scriptscreenprogram.h`, `scriptscreens.h`, `utilities/dictionary.h` and `vehicle/blackbox.h`.
- `vehicle/brakesetting.cpp` and `vehicle/odometer.cpp` (the brake setting of the wagons, the odometer store) joined the library on master after that run. They have tests (`test_brake_setting.cpp`, `test_odometer.cpp`) and no numbers yet.
- The script hosts (`scriptscreens.cpp`, `luaprogram.cpp`, `wasmprogram.cpp`) aren't in the library. The test binary compiles them itself, with the real runtimes.
- `scene/scenelayers.h`, `scene/scenenode.h` and `scene/scenenodegroups.h` are in the report with a few lines each: the parser reports the files it includes to the scenery, which the tests replace with stubs (`engine_stubs.cpp`).

### The parser, the helpers and the PLC

`test_parser.cpp`, `test_utilities.cpp` and `test_ladder_logic.cpp` record how `utilities/parser.cpp`, `utilities/utilities.cpp` and `scripting/ladderlogic.cpp` behave now, oddities included, so that a later change of them can be checked. The lines the measured run left out, by what it takes to run them:

| File | Not run | Through the public interface | With files of the test | Left: the system or the game state | Nothing in the game calls it |
|---|---|---|---|---|---|
| `utilities/parser.cpp` | 255 lines | 131 | 105 | 2 | 17 |
| `utilities/utilities.cpp` | 209 lines | 154 | 19 | 19 | 17 |
| `scripting/ladderlogic.cpp` | 208 lines | 169 | 37 | 0 | 2 |

The tests are written for the first two columns, and for what the game doesn't call as far as it can be called from outside. The files of a test are made in the build folder of the tests (`scratch.h`) and removed with it.

**What stays untested, and why:**

- **`parser.cpp`:**
  - The scenery layers of a scenery opened for editing are stubs in the tests. The parser's calls of them are run (`sceneryLayers`), but only the templates used can be checked. The include of a terrain file left out for the binary terrain marks its layer as binary (2 lines): the stub has no layer to mark, so that isn't run.
  - The messages of the log (`Failed to open file`, `Bad include`) are run, and not checked: the log of the tests is a stub.
  - A buffer type which is neither a file nor a text (the `default:` of the constructor) can't be given without an invalid value of the type.
- **`utilities.cpp`:**
  - `Now()`, `user_config_path()`, `user_cache_path()` and `true_random_seed()` depend on the clock, the environment and the entropy source of the system. The tests call them and check the shape of the result. The branches for a system without the variables or without an entropy source aren't run, and those of the other systems aren't compiled.
  - `FileCased()` looks through the working folder for a name given without a folder, and through the folder of the file elsewhere; on Windows it only asks whether the file is there. The search is run on Linux only.
  - `Bezogonkow()` gives other letters with MSVC than with GCC and Clang, see the findings. The test takes either, told by the entry of the table the difference starts at, so each build checks the half it is.
- **`ladderlogic.cpp`:** the two `return`s marked "not reachable" after the switches over the kind of an element.

**Code nothing in the game runs** (checked against the whole source tree):

- **The PLC as a whole.** A vehicle has one (`TMoverParameters::m_plc`) and runs it in every step (`m_plc.update()`), but nothing calls `load()`, so its program is always empty; nothing calls `find_or_insert()`, `input()` or `output()` either, so no file of the game data can be a program for it. Of `ladderlogic.cpp` the game runs the 9 lines of an update without timers and without operations. The tests run the rest with programs made up for them.
- **In `parser.cpp`:** `countTokens()` with its `count()`, and `addCommentStyle()`. No caller gives the separators as a null pointer either, which `updateCharClasses()` allows for.
- **In `utilities.cpp`:** `generate_uuid_v4()`, `FuzzyLogicAI()` and `last_modified()`. In `utilities.h`: `BorlandTime()`.

**Cheap to add next:** the Lua host (`luaprogram.cpp`, 113 lines not run) is mostly functions of the scripts' API no test script calls yet: `fill`, `fill_rect`, the aligned texts, `draw_image`, `copy`, sprites, `time`, `log`, a command from a screen outside of the cab. A script of a few lines each runs them, with the folders `test_script_screen_host.cpp` already makes. The WebAssembly host (`wasmprogram.cpp`, 134 lines) has the same functions behind imports, and needs a module encoded by hand for each group of them.

## What the tests found

The tests record the behaviour as it is. The following points may deserve a look by someone who knows how the real vehicles behave.

- **`Random(10) - Random(10)` (`Mover.cpp`, the noise of the control pipe pressure `CntrlPipePress`):** the order the two calls were evaluated in isn't specified. MSVC Release, MSVC Debug and GCC gave different traces, a control pipe pressure mirrored around 5 bar, which failed 32 tests on the Linux CI. Fixed: the draws are sequenced in the order of MSVC Release, which the golden traces were recorded with.
- **`TCommand::Location` isn't initialised** in a new vehicle. The game always sets it through `PutCommand()`, but a test calling `RunCommand()` directly got different results run to run.
- **`CompressorList` is `int[4][9]`** (`MOVER.h`), though its factors are fractions: 0.9 is read as 0. No game vehicle has the list yet.
- **A train whose pipe leaks under 2.75 bar before its start-up can't be released:**
  - Where this happens: an FV4a handle and an LSt valve, for example 4 minutes standing with wagons, waiting for a hot engine.
  - The cause: the pipe gets cut off the main reservoir (`lock_old` in `UpdatePipePressure`). The AI presses its unlock button only for the lock set by `LPOn` and `LPOff` (`control_main_pipe`).
  - The same after every emergency stop of such a train, by the alarm chain or the radio-stop: reset, it stays with its pipe empty until the releaser is held (`b_rls`), which the test of the emergency stops does.
- **EW58:**
  - Its electrodynamic brake comes in only by the stages of the ep brake (TUHEX), never by the brake pipe alone.
  - It needs 110 V, from the converter of the middle car only.
- **ET42:**
  - At the high current setting the controller stops at the last series position, and the train crawls.
  - From the low setting's top speed (over 100 km/h) the dynamic brake's current trips the overload relay at once.
- **EU47:** its local brake sets the flag of the electrodynamic brake, which keeps the train brake's air out, but the motors don't brake. The local brake's air does it.
- **EP09:** the flag of the automatic electrodynamic brake stays on at a standstill while the cylinders hold less than 2.1 bar, and it flicks on while a high application is released.
- **Diesel engines with a second water circuit (SU45, ST44) started at 88 degrees:** they don't start until that circuit, over its limit (74 or 75 degrees), cools. Off, it cools slowly.
- **Brake cylinders over the vehicle's highest pressure** (found by the invariants, see the README):
  - The K valve of the SM03 and the EU04 fills the cylinders to the pressure of the auxiliary reservoir in a full service or emergency application: 4.27 bar of the SM03's MaxBP 3.5, 3.19 bar of the EU04's 2.8.
  - The ESt3d_PZZ valve of the SN61 does so in an emergency: 4.6 bar of its MaxBP 3.8.
  - The EU07's LSt overshoots a little, to 4.0 bar of its 3.8.
- **Diesel engines driven over their top speed:** a lower gear taken at speed drives the engine through the wheels over its `nmax`, the SM03's by 10%, the WMB10's by 25%. The cutoff (`nmax_cutoff`) only takes the fuel off.
- **The ELF's C car's main reservoir over its governor:** the cars share the supply pipe, the A car's governor stops at 9 bar, the C car's at 8.5, so the C car's reservoir reaches 8.64 bar.
- **Wagons thrown about as a train starts with full physics of the couplers:** the slack taken up throws them at up to 5.7 g, and back within a step.
- **Nothing limits the speed:** a light train at full power runs well past its Vmax (the 754 past 160 km/h, against a Vmax of 100).
- **Vehicles that don't do what they should, as driven by the tests:**
  - An EN57 doesn't get ready (main reservoir).
  - An EN57AKL alone doesn't charge its pipe.
  - The TEM2 barely moves, and the SM31 crawls at 0.3 km/h at full power, its motors taking 444 A.
  - The draisine can't start from rest (the pump's force is under the resistance of a standing draisine).
  - The Elf and the Impuls, their doors opened with the steps permitted, close the doors as they start off but keep the steps out, which keeps them from moving.
  - The ED72 holds an ep application with its handle at running, and with the ep brake switched off too; only the ep release position lets it off. The EW58 releases at running.
  - The road vehicle (MAN) is only slowed down by a minute of full service.
- **A train run with five physics steps in a frame:** an EU07 with wagons takes no current, its controller at full power, and rolls at a few km/h. The test steps the vehicles the way `TDynamicObject` does (`FastComputeMovement` for the steps before the last one), so the game may do the same.
- **Game data:**
  - The 214M's file has `TurboPos` and `MotorParamTable` twice, and the EN57AL's `Engine` twice. The loader reads each in turn, so the later one overrides what both set.
  - A wagon set up braked keeps its hand brake on until someone releases it. The AI does that for the whole train (`AutoRewident`), so a test has to as well.
- **The parser (`parser.cpp`):**
  - Read line by line (`"\n\r"` as the separators), a traction include left out because the traction isn't loaded takes the following lines with it, up to the next `end`: the directive is cut out of its line, but the skipping reads on in the file.
  - A traction file is told by `tr/` or `tra/` anywhere in its name, so with the traction off an include from any folder ending in "tr" (`centr/…`) is left out too.
  - Read line by line, every line starting with the letters `include` is taken for the directive (`included in price` includes the file `d`), also when the parser is told not to expand includes (`expandIncludes = false`).
  - Empty quotes (`""`) give no token: the next token is read in their place, so everything after a value meant to be empty moves by one.
  - The text in quotes keeps its case when the caller asks for lower case. That may be meant.
  - An include of an empty file is logged as a file which can't be opened (`Bad include: can't open file`, seen in the log of the test, which the test itself doesn't check).
  - A text which ends inside a comment or inside quotes leaves the parser failed (`ok()` false), not just at its end; an empty text fails with its first read, through the check for a byte order mark.
  - `deserialize_map()` (`utilities.h`, the sound overrides) therefore reads an empty key from an empty file, and one from a file ending with a line break; in a file with Windows line ends its values keep the carriage return. The game's `sound_overrides.txt` is a single line without a line break.
  - The separators given as a null pointer are taken as none by the tokenizer, but passed on to `strcmp` for a token of 7 characters or more starting with `i`, which crashes (tried by hand, not kept as a test). No caller gives a null pointer.
- **The helpers (`utilities.cpp`, `utilities.h`):**
  - `Bezogonkow()`, which takes the marks off the letters for the timetable and the on-screen texts: its table has a `±` in the source, a UTF-8 file. MSVC makes one byte of it, GCC and Clang two, which moves every entry from 0xB2 on by one. With MSVC the 18 polish letters come out as `ACELNOSZZacelnoszz`; with GCC as `ALELDNSZZ,le,dnszl`: `Ć` as L, `Ń` as D, `Ó` as N, `ą` and `ł` as a comma, `ć` as l, `ń` as d, `ó` as n, `ż` as l, only `Ę` and `ę` by luck as themselves. The Linux and macOS builds of the game show the names of the stations that way. The test checks whichever table the build has (the GCC one was tried with the table alone, compiled by GCC 6.3).
  - `to_minutes_str()` reads past its table of raised digits for minutes with a fraction of 0.99 or more, and before it for negative minutes, which `CompareTime()` can give it for a timetable whose times go back. Tried by hand, both ended in `std::bad_alloc`, the text read from outside of the table being no text; not kept as a test. A width under 2 throws `std::length_error`.
  - `stol_def()` gives its default only for an empty text. A text which isn't a number gives 0 (the flags of an event, the log setting).
  - `split_string_and_number()` throws for a name with a hyphen which no digit follows (`a-b`), and for a number too big. The gamepad bindings and the PLC use it on names from files.
  - `extract_value()` throws `std::out_of_range` when the key is the last thing in the text with nothing after its `=`. With a space after the `=`, the next `key=value` is taken for the value.
  - `seed_of()` takes a word starting with digits for the number in front (`12abc` is 12), so different words can stand for the same seed.
  - `erase_extension()` cuts at the dot of a folder when the file has none (`dir.d/file` becomes `dir`).
  - `Round()` adds a half and cuts the fraction, which rounds the negative numbers up: -1.6 becomes -1.
- **The PLC (`ladderlogic.cpp`):**
  - Nothing loads a program into it, see above.
  - `and` and `or` combine the states bit by bit, and `out` passes a number on as it is: two elements which are both on, with the states 2 and 1, give 0.
  - A counter is done until the first cycle of the program gives it its limit.
  - `end` is read as "no operation": the lines after it are loaded and run.
  - A name with a hyphen (`door-open`) throws out of `load()`, through `split_string_and_number()`.
  - The kind of an element is told by its name: `t`, `ton` or `timer.` and `c` or `counter.` before the number, so a variable can't be named `t1` or `c2`, nor `t` or `c` alone: an input named `c` is a counter, on from the start.
  - The line of an operation is split with `getTokens(3, "\t ")`, which gives the separators in the place of the lower case flag. The result is the same, the default separators being those and the semicolon.
  - An update longer than 32.7 s doesn't fit the milliseconds a timer counts in (`short`). Tried by hand: a timer of 1 s wasn't on after an update of 40 s, nor a second later. Not kept as a test, the conversion being undefined.
