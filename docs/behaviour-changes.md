# Behaviour changes

Changes accepted in review that players, scenery authors or data makers can notice, for release notes and for
triaging new bug reports ("it worked differently before"). Each entry says what changed, who notices, what
data or eu07.ini it needs, and the pull request (#NNN, MaSzyna-EU07/maszyna). Written against `staging` as of
2026-11-24; the list starts at #266 (merged 2026-11-08). Keys marked "optional" change nothing until the data uses them.
#443 (2026-11-22) reverted #408 and merged `master` into `staging`; of the changes it brought from `master` only the
physics log ones, which replace those of #408, are listed here.

The last section lists the new optional data keys, so that data makers can find them; they change nothing for
existing data.

## Logs and files

- **physicslog has a new last column `VehicleVelocity[km/h]`** (#266). The header line of every physics log
  (`physicslog yes`, the black box tapes) changed: `WaterAuxTemp[Deg]` is now followed by a space and the new column,
  the vehicle's speed (the second column is the speed of the wheels, which stays 0 while they are locked).
  Notices: anyone parsing the logs by column count or by the exact header. The old columns keep their positions.
  Needs: nothing.
- **Passing a signal at stop is logged in every session** (#266). The line `<vehicle> passed signal <name> at danger`
  goes to the log for AI and player drivers alike, rail vehicles only, for a signal showing `SetVelocity 0 0`
  (not an automatic block signal with `SetVelocity 0 x`). Before it was only counted in test runs. Notices: log.txt
  of any scenery with trains running red signals. Needs: nothing.
- **The settings dump at the start of log.txt lists the `test.*` keys** (#266): `test.timeout`, `test.standstill`,
  `test.pass_on_standstill`, `test.fail_if_stop_overran`, `test.no_movement_before_pass` (and `test.wall_timeout`,
  `test.vehicle` when set), also in normal sessions, with their defaults. They are ignored outside the test mode.
- **The physics logs are written every 2 s by a thread of their own** (#385 on `master`, on `staging` with #443).
  The files are opened only for the moment of writing; a crash loses the last 2 s. A log whose file can't be written
  keeps its lines and is retried (up to 64 MB in all). `physicslog yes` still logs every driver, road vehicles
  included. The closing writes out everything left, with no time limit; only the test mode stops waiting after 2 s.
- **Characters illegal in Windows file names are replaced by `_` in log file names, on every system** (#385, with
  #443): a spawned vehicle `spawn_ezt:12` logs to `spawn_ezt_12.dat`.
- **`physicslog.trains yes`** (#385, with #443, new): `.dat` logs of the drivers of rail vehicles only (no passengers,
  no road vehicles). **`blackbox yes|no`** (#385, with #443, new): the black box on its own; with no entry it goes with
  `physicslog` (or `physicslog.trains`) as before.
- **A failed start ends with exit code 1 and the reason in the log** (#425); a closing from the player's side still
  ends with 0. The crash dump written by the Windows crash handler is small by default (stacks and data segments,
  tens of MB instead of the whole memory); `crashdump.full yes` brings back the full one.
- **The region cache `<scenery>.sbt` is written through a temporary file and checked whole when read** (#351): a
  damaged file is turned down ("Bad file: ... ignored") and the scenery loads from the text and writes the cache again.
  Format unchanged.
- **Test mode** (#266, `--test`; see `tests/scenarios/README.md`): takes no input from outside, does not read the
  player's own eu07.ini, writes only its verdict to the standard output, and has exit codes of its own (kept clear of 1
  and 3).

## AI

- **AI on a diesel with a shunting gear drives in the shunting mode** (#275). For a `DieselEngine` with
  `ShuntModeAllow` (the SM03's 2Ls150 gear) the AI sets the shunting mode when its order is Shunt/Loose_shunt or the
  consist is over 224 t. Before, `CurrentSwitch()` took the call for the motor overload relay threshold and the mode
  was never set, so the AI always ran in the running mode. Notices: trains of SM03 on a shunting order or over 224 t
  get another speed/power curve, so running times in timetables can change. The same PR makes the first move of the
  SM42/SP42 switch set the shunt mode at once. Needs: nothing.
- **A Connect first order releases the hand brake** (#339).
- **The AI's wagon inspector sets G/P/R by Cw-1 par. 18-19 and cuts out the brake of wagons without G in a G train**
  (#282 and its review commits). A cut-out skipped on the run is done once the train stands. `ai.brakesetting.cw1 no`
  in eu07.ini restores the earlier behaviour (default yes). Notices: another brake setting of formed freight and
  passenger trains, brakes cut out on some wagons. Needs: nothing.
- **Braking for a stop**: making for a passenger stop leaves room for the brake's fill time only, no look-ahead floor
  (#297); the passenger-stop flag no longer flips on every scan (#295); the W4 stop point is shifted by the margin the
  stop is judged by (#321); the shunting AI brakes for a stop point again (#409); gradients of 2.5 per mille and more
  count as up gradients for the hold and the start (#265). Notices: stops at platforms and signals look different,
  running times can move a little.
- **A reduced signal speed ends at a marked end of the interlocking area** (#305): only where the scenery puts the
  `SignalAreaEnd` putvalues event. Without it the old rule holds.
- **No multiple-unit control between electric and diesel traction vehicles** (#279): `Attach()` leaves the control
  connection out, also for the scenery and the AI (a Vectron and an SM42 no longer run in multiple); logged.
  `controlcoupling.enginecheck no` in eu07.ini brings the old behaviour back (default yes).

## Cab controls

- **A lone two-state `main_sw` with a latch needs two presses to switch on after a trip** (#277). The single toggle
  lever of the line breaker (no `main_on_bt`/`main_off_bt` in the cab) stays where it was put; after the breaker was
  knocked out (the converter overload relay of the EP05) the first press puts the lever off, the second closes the
  breaker. Applies to every cab with a lone two-state `main_sw`: e6act, ep03, ep05, et21, et22-2000, eu05, wmb10 (list
  from the review; the cab files are in the SVN and were not checked here). Notices: whoever expects one press.
- **A `main_off_bt` declared `type: toggle` is the bistable hold coil lever of the ET22 and EP09** (#268): off opens the
  breaker and keeps it from closing, back on only feeds the coil. Cabs keep a push button unless their `.mmd` says
  `toggle`.
- **The cruise control has a switch of its own** where the secondary controller shunts the field (6Dg, ST45, BR285)
  (#391): `tempomat_sw`/`tempomatoff_sw` follow the cruise control, not `ScndCtrl`. Vehicles with only one of the two
  are unchanged. No new keys.
- **The compartment lights keyboard toggle** latches also in cabs without `compartmentlights_sw:` or with the on/off
  push buttons (#353); the light preset selector with `Wrap=Yes` turns past its end with animation and sound (#345).
- **The HUD shows the radio channel only with the radio fuse closed**; the radio has a fuse and a hint "Close radio
  fuse" (#280).
- **Door closure lamp (SOD) of the whole consist** (#291): the three-state lamp (lit locked, blinking released, dark
  open) works only on vehicles with `Doors: DoorClosureSignal=Yes` in the `.fiz`. Old EN57/ED72 consists without
  `DoorNeedPermit` do not get the key (they would blink at every standstill) and keep the old lamp that shows the lock
  of the vehicle's own doors. The eu07.ini switch `doorclosuresignal` was removed: the data decides. Needs: the key
  in the `.fiz` of the vehicle whose cab shows the lamp.
- **Separate engine start buttons work only with new data** (#294): the cab controls `mainb_*` and `mainc_*`
  (`_sw`, `_on_bt`, `_off_bt`) start the second and third vehicle of a train in multiple, and `Engine: JointEngineStart=No`
  stops the cab's `main_*` from touching the other vehicles. According to the data owners no cab in the SVN has
  `mainb_*`/`mainc_*` yet (not checked here); without them and without `JointEngineStart=No` nothing changes
  (default `Yes`). New commands `linebreakertoggleb/c`, `linebreakeropenb/c`, `linebreakercloseb/c`, no default keys.

## Physics and scenery

- **`physics.collisions` defaults to `new`** (#379, with #377 and #378): rail vehicle collisions past the buffers
  crush the ends, a vehicle may ride up or be pushed off the track; road traffic keeps the old rule. `old` or
  `monitor` in eu07.ini bring back the earlier behaviour. Notices: harder collisions, derailments of struck vehicles,
  crushed ends that stay until repaired (resetconsist or repair). `physics.derailment` defaults to `monitor` (the
  legacy draw decides, the log tells what the criteria would do); `new` is opt-in. `physics.trackcheck` and
  `couplerphysics` (slack, hysteresis and breaking of the coupler; commits c85aab4f to 226155d4, no PR number) are
  off by default.
- **Welded track buckles in the heat** (#378) when the scenery or an event sets it; no effect otherwise.
- **The cab shake is taken at the frame's moment** (#403, #414) and no longer moves in steps.
- **Station names keep their Polish letters** in the interface and timetables, a W4 finds its station by one key
  (#418). A station of departure written in capitals is still a stop (#343).
- **Timetable service flags** (#398): the header row `[ Flagi AI | ... ]` and station entries such as
  `kierownik=nie` change how the AI serves the train; a timetable without them drives as before. Old builds skip the
  header row.
- **New putvalues commands** for sceneries: `Crew:<vehicle> <cab> <flags>` puts an AI driver in a vehicle or takes
  it off (#422); `SignalAreaEnd` (#305, above); the parameter `H` after the coupling of a vehicle prepares it for
  hauling dead in a train, the "pullee mode" (#419).

## Data keys that need new data

All optional; without the key everything is as before. The data files are in the SVN, not here.

| Key / file | What it does | Data to change | PR |
|---|---|---|---|
| `.fiz` Cntrl `SBDReleaseDelay` (s, default 0) | The holding brake of a unit with the MED (SBD) lets go only after power was demanded for that long. | Elf files with SBD would get `1.0` (its `IniCDelay`). | #424 |
| `.fiz` Engine `PressureSwitchBrakeCylinder=No`, Cntrl `EmergencyBrakeMainOff=<bar>` | Pressure switch of the EP05 with the brake pipe alone; an emergency application on a power position opens the line breaker (pantographs drop with `PantRequiresMain`). | `ep05_v2/ep05.fiz`, `ep05v1.fiz`: `No` and `2.5`. | #421 |
| `.fiz` Cntrl `PantRequiresMain=Yes` | Line breaker closes before the pantographs, on the low voltage; pantograph valve stays open only with the breaker closed; the AI closes the breaker first. | `ep05_v2`, `ep05v1`, `181_cz1.fiz`. | #272 |
| `.fiz` Cntrl `ScndCtrlResistiveLock=Yes` | The field shunt can't be stepped up or down at zero or on a resistive position; it drops to 0 there. | ET21, ET22, EP09 files. | #278 |
| `.fiz` AI `ShuntPantstate=1/2/3` | Pantographs the AI keeps up while shunting. | ET41 files (`203e-a.fiz`, `203e-b.fiz`, `_dsa150`). | #281 |
| `.fiz` Engine `EDBrake`, `EDBrakeFmax/Pmax/V0/V1/Rate/Blending` | Dynamic brake of diesel-electric vehicles. | BR285 and the like. | #394 |
| `.fiz` Doors `DoorClosureSignal=Yes` | SOD lamp of the whole consist, see above. | Vehicles with `DoorNeedPermit`. | #291 |
| `.fiz` Engine `JointEngineStart=No`, `.mmd` `mainb_*`, `mainc_*` | Separate engine start per vehicle (ST43, SP32, SN61), see above. | Cabs and `.fiz` of those vehicles. | #294 |
| `.fiz` security system `AwareOffInShuntMode=Yes` | The SM42's vigilance device (CA) is off in the shunt mode. | SM42/SP42 files. | #275 |
| `.mmd` `animdoor` per door wing | Door wings of different kinds on one side. | Vehicles with mixed doors. | #397 |
| `.mmd` `i-compartmentlights`, `radioon_sw:` etc. | New lamps and radio fuse switches. | Cabs that have them. | #352, #280 |
| eu07.ini `sound.*` | The OpenAL Soft options, each defaulting to what the simulator did before. | Optional. | #404 |
