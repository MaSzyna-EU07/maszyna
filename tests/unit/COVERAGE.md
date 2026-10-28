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

## What the tests found

The tests record the behaviour as it is. The following points may deserve a look by someone who knows how the real vehicles behave.

- **`Random(10) - Random(10)` (`Mover.cpp`, the wheel flat noise):** the order the two calls are evaluated in isn't specified. MSVC Release, MSVC Debug and GCC give different traces, so the golden traces are recorded on MSVC Release.
- **`TCommand::Location` isn't initialised** in a new vehicle. The game always sets it through `PutCommand()`, but a test calling `RunCommand()` directly got different results run to run.
- **`CompressorList` is `int[4][9]`** (`MOVER.h`), though its factors are fractions: 0.9 is read as 0. No game vehicle has the list yet.
- **A train whose pipe leaks under 2.75 bar before its start-up can't be released:**
  - Where this happens: an FV4a handle and an LSt valve, for example 4 minutes standing with wagons, waiting for a hot engine.
  - The cause: the pipe gets cut off the main reservoir (`lock_old` in `UpdatePipePressure`). The AI presses its unlock button only for the lock set by `LPOn` and `LPOff` (`control_main_pipe`).
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
  - The TEM2 barely moves.
  - The draisine can't start from rest (the pump's force is under the resistance of a standing draisine).
- **Game data:**
  - The 214M's file has `TurboPos` and `MotorParamTable` twice, and the EN57AL's `Engine` twice. The loader reads each in turn, so the later one overrides what both set.
  - A wagon set up braked keeps its hand brake on until someone releases it. The AI does that for the whole train (`AutoRewident`), so a test has to as well.
