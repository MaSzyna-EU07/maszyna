# Vehicle graph (.vhg)

A vehicle graph describes a vehicle as connected blocks, in place of the `.fiz` file: the cab controls and
indicators, the logic, the sounds, and the electrical, mechanical and pneumatic systems of the vehicle. The simulator
code is in `vehicle/graph/`; the external editor (`maszyna-graph-editor`) reads the block catalog exported by the
simulator and runs the same blocks in its preview.

A vehicle is a graph vehicle when `<name>.vhg` lies next to its `<name>.mmd`. Then the `.fiz` isn't read, and the
`.mmd` gives only the models: the sounds of the vehicle and the controls and indicators of its cabs come from the graph.
The only exception is the body: entries of the body the graph doesn't give (e.g. a graph made only of cab logic, without
the `body` section) are taken from the body sections (`Param.`, `Load:`, `Dimensions:`, `Wheels:`, `BuffCoupl.`,
`Headlights:`) of `<name>.fiz`, if there is one. A graph vehicle without length or mass isn't loaded, with a message in the
log.

## File format, version 2

A `.vhg` file is JSON. Comments (`//`, `/* */`) are allowed.

```json
{
	"format": "maszyna-vehicle-graph",
	"version": 2,
	"vehicle": { "name": "ET22" },
	"settings": { "low_voltage_simulation": false },
	"body": { "mass": 83500, "length": 16.315, "wheel_diameter": 1.25, "axle_arrangement": "Bo'Bo'" },
	"profiles": { "brake_pipe": 1 },
	"nodes": [
		{ "id": "bat", "type": "cab.switch", "title": "Battery", "params": { "behaviour": "bistable", "submodel": "battery_sw", "command": "batterytoggle" } },
		{ "id": "lamp", "type": "fb.lamp", "params": { "color": "red", "submodel": "i-battery" } }
	],
	"connections": [
		{ "from": "bat.pos", "to": "lamp.on" }
	],
	"layout": { }
}
```

- `nodes[].id`: letters, digits, `_` and `-`; unique in the file.
- `nodes[].type`: block type from the catalog (see `-graph-catalog` and the list below).
- `nodes[].params`: typed values. Numbers are JSON numbers, switches are `true`/`false`, choices are ASCII ids
  from the catalog (e.g. `"bistable"`, `"rotate"`, `"ne"`), tables are arrays of rows of numbers
  (`"rlist": [[0, 0, 0, 0, 0], [1, 24.891, 1, 4, 0]]`; the catalog names the columns). Parameters left out take their
  default values.
- `connections`: `"node.port"` of an output to `"node.port"` of an input. An output can feed many inputs; an input
  has at most one source. Unconnected inputs see their default value.
- `settings.low_voltage_simulation`: when enabled, blocks with low voltage ports simulate the voltages and currents
  of the control circuits (e.g. a lamp needs enough voltage and draws current). When disabled, low voltage
  connections have no effect and the apparatus behaves as if always powered.
- `body`: the vehicle body, see below.
- `profiles`: standard interfaces the vehicle uses, with their versions (`brake_pipe`, `feed_pipe`, `heating`, `ai`,
  all version 1). Unknown profiles are reported; newer versions than the simulator knows are rejected.
- `layout`: positions of the blocks in the editor; ignored by the simulator.

### Body

The body stays native in the simulator: mass, dimensions, axles, couplers, motion along the track, collisions. Its
entries replace the `.fiz` entries named in the catalog (`"fiz"` of each body entry); the simulator writes them as
`.fiz` lines and reads them with the same code as the `.fiz` files, so defaults and derived values are the same.
Entries left out keep the defaults of the `.fiz` reader.

| key | unit | .fiz | key | unit | .fiz |
|---|---|---|---|---|---|
| category (train, road, unimog, ship) | | Param. Category | coupler_type | | BuffCoupl. CType |
| mass | kg | Param. M | coupler_kb, coupler_kc | kN/m | BuffCoupl. kB, kC |
| mass_rotating | kg | Param. Mred | coupler_dmax_b, coupler_dmax_c | m | BuffCoupl. DmaxB, DmaxC |
| vmax | km/h | Param. Vmax | coupler_fmax_b, coupler_fmax_c | kN | BuffCoupl. FmaxB, FmaxC |
| power | kW | Param. PWR | coupler_beta | | BuffCoupl. beta |
| sand_capacity | kg | Param. SandCap | coupler_allowed_flag | | BuffCoupl. AllowedFlag |
| heating_power | kW | Param. HeatingP | control_type | | BuffCoupl. ControlType |
| length, height, width | m | Dimensions: L H W | max_load | | Load: MaxLoad |
| cx | | Dimensions: Cx | load_unit | | Load: LoadQ |
| wheel_diameter | m | Wheels: D | load_accepted | | Load: LoadAccepted |
| wheel_inertia | kg m2 | Wheels: AIM | load_speed, unload_speed | 1/s | Load: LoadSpeed UnLoadSpeed |
| track_gauge | m | Wheels: Tw | light_color ("r g b") | | Headlights: LampRed/Green/Blue |
| axle_arrangement | | Wheels: Axle | collector_type | | Power: PantType |
| axle_distance, bogie_distance | m | Wheels: Ad Bd | collector_min_h, collector_max_h | m | Power: MinH MaxH |
| min_radius | m | Wheels: Rmin | collector_csw | m | Power: CSW |
| bearing (roll, slide) | | Wheels: BearingType | collector_max_voltage, collector_min_voltage | V | Power: MaxVoltage MinV |
| | | | collector_max_current | A | Power: MaxCurrent |
| | | | collector_layout | | Power: PhysicalLayout |

The equipment blocks which stand for parts of the `.fiz` vehicle (brakes, compressor, security system, doors...)
mark their parameters with the `.fiz` entries they replace (`"fiz"` of the parameter in the catalog), and are
written to the body the same way. The pantographs need `power` above zero.

### Ports and domains

Signal ports carry one value (bool, int or float; booleans are 0 and 1). Physical ports carry a pair of values
in opposite directions:

| domain | forward | back |
|---|---|---|
| `high_voltage`, `low_voltage` | voltage [V] | current [A] |
| `aux_ac` | voltage [V] | power [W] |
| `drive` | demand [0-1] | power [W] |
| `mechanical` | torque [Nm] | rotational speed [rad/s] |
| `traction` | force at the wheel rim [N] | velocity [m/s] |
| `air`, `brake_pipe`, `feed_pipe` | pressure [bar] | flow |

Mechanical values are in the frame of the vehicle: positive torque pushes the vehicle towards its front (cab 1),
positive speed is the rotation when moving that way. The `dir` input of the motors is the position of the reverser
in the occupied cab; the motors turn it to the frame of the vehicle.

The air domains connect blocks of the vehicle's own air system: the brake pipe and the feed pipe of the body,
its main reservoir, the brake valves created for the body. The link values show the pressures; the flows go
between the reservoirs of the body, the same way as for the `.fiz` vehicles.

### Order of evaluation

Blocks are evaluated once per physics step, in the order given by the connections. A loop of connections has to pass
through a deferred block (`logic.sr`, `logic.ton`, `logic.tof`, `logic.delay`, `pn.governor`, `el.hsb`,
`el.contactor`), whose outputs change in the next step; other loops are rejected when the file is loaded. Back values
of physical connections are always taken from the previous step. Some blocks (`br.pipe`, `pn.reservoir`) also act
after all blocks were evaluated, e.g. to exchange air with the neighbours. Blocks marked `single` in the catalog stand
for parts the vehicle has one of (brake pipe, main reservoir, distributor, line breaker...): a graph can have one each.
The graph has no other source of time or randomness, so the same inputs give the same results.

## Blocks

`-graph-catalog` gives the full list with all ports, parameters, units and defaults. Origin: *simulator* blocks drive
the existing classes unchanged; *extracted* blocks use code moved out of `TMoverParameters` into functions shared with
the `.fiz` vehicles; *native* blocks are parts of the vehicle body.

**Logic** (`logic.*`): `compare`, `and`, `or`, `xor`, `nand`, `nor`, `not`, `sr`, `ton`, `tof`, `delay`, `hyst`, `const`,
`math`, `select`. **Script**: `lua.block` (below).

**Cab** (all with `submodel` and `cab`: `any`, `1`, `2` - the cab whose model holds the submodel, `0` - the engine room,
as `cab0definition:` of the `.mmd`):
- `cab.switch` - switch, button or lever: `behaviour` (bistable, monostable, multi), positions, min/max values, default,
  `spring`, animation (rotate in degrees, move in millimetres per position, `offset`, `smoothing`), `sound_inc` and
  `sound_dec` (played at the submodel when the position goes up or down; the same format as `soundinc:` and
  `sounddec:` of the `.mmd` controls: a file, a random set `[a.wav b.wav]` or a `{ }` block), `sound_positions`
  (sounds of single positions as the `soundN:` entries of the `.mmd` controls, with N the number of the position from
  0, e.g. `sound0: a.wav sound1: b.wav`; a position with its own sound plays it in place of `sound_inc`/`sound_dec`). Outputs `pos`
  (value) and `index` (number of the position). `command`: a simulator command which presses it (e.g. `batterytoggle`;
  pressed and released with the key); `key_inc`/`key_dec`: commands which move it up and down (e.g.
  `mastercontrollerincrease`), so the keyboard bindings of the simulator operate the graph. `hw_name`: name for the
  hardware desk (not used yet).
- `fb.lamp` - indicator lamp (`<submodel>_on`/`_off` as for the `.mmd` lamps; `sound_on`/`sound_off` played when it
  lights and goes out, as `soundinc:`/`sounddec:` of the `.mmd` lamps), `fb.gauge`, `fb.manometer` (two needles),
  `fb.sound` (file, volume, pitch, loop, placement as `sound_placement`, range, offset), `fb.export` (value for the
  python screens of the cab, under `name`).

**Vehicle** (native): `veh.state` (speed, acceleration, wheel revolutions, slip, occupied cab, time), `veh.doors`
(open left/right, lock; the doors of the body move them), `veh.lights` (light bit masks of the front and rear ends,
dimming), `veh.sander`, `veh.anim` and `veh.visible` (submodels of the external model of the vehicle, `model`:
`vehicle` or `lowpoly` for the low poly interior: `veh.anim` rotates (degrees about y) or moves (millimetres along z)
`submodel` by `offset + scale * value` with the smoothing of TGauge; `veh.visible` shows `submodel` when on and
`submodel_off` when off, optionally blinking. They're applied per vehicle right before it's drawn, so vehicles sharing
a model show their own state; `-graph-run` computes only the state of the blocks).

**Electrical**: `el.pantograph` (index 0 front, 1 rear pantograph of the vehicle model; raising needs air when
connected; without the simulator the catenary has `preview_voltage`), `el.junction`, `el.hsb` (line breaker: Mains of
the body; closing time, init time, undervoltage, overcurrent trip), `el.contactor`, `el.converter` (ConverterFlag),
`el.battery`, `el.voltage_relay`, `el.transformer` (below).

Several supply systems: `el.hsb` and `el.converter` may appear several times. The body's Mains is closed when any line
breaker is closed, ConverterFlag/ConverterAllow are set when any converter runs / is switched on.
- `el.voltage_relay` (in: high_voltage, draws nothing; out: `ok`, `u` [V]; params `min_voltage`, `max_voltage` (0 = no
  upper limit), `delay` [s]): `ok` when the voltage stayed within the range for the delay. Use it to recognise the
  supply system and to close or hold the breaker of that system.
- `el.transformer` (in/out: high_voltage; params `ratio` = out/in, `eff`): out voltage = in × ratio, input current =
  output current × ratio / eff. Main transformer of AC vehicles or AC branches (rectifier implied).
- The simulator gives the vehicles only the catenary voltage (no AC/DC or frequency information: the frequency of the
  power source in the scenery isn't passed to the vehicles), so the systems are told apart by voltage.
- `el.converter`: output `aux` (aux_ac, `out_voltage`) and optional `nn` (low_voltage, `nn_voltage`; 0 = off), whose
  power (current × voltage) is drawn from the high voltage as well. `aux_kind` (ac3 | ac1 | dc) only labels the
  auxiliary circuit for the editor.

**Drive**: `drive.dc_series` (series DC motors with resistor starting, the `.fiz` RList and MotorParamTable0 as tables;
the motor equations of `TMoverParameters::Current` and `Momentum`), `drive.async` (induction motors with inverters, the
`Engine:` parameters of `ElectricInductionMotor`; the motor and inverter computation of `TractionForce`; demand -1..1,
negative for the electrodynamic brake; `ed` output for the blending with the distributor), `drive.simple` (motor with
torque and power limits, for prototypes), `drive.gear`, `drive.axle` (the sum of all axle forces is the tractive force
of the vehicle).

Combustion engines: `drive.diesel` (starter and start time, all-speed or min/max governor, torque from a table rpm → Nm
or kW, or the Mover parabola Mmax/nMmax/Mnmax, friction Mstand, fuel consumption; `dizel_Momentum`), `drive.gearbox`
(friction clutch and gearbox with a table of gears, shifting with the clutch released), `drive.hydro` (torque converter
with filling, fluid coupling range and lockup clutch, the `hydro_TC` entries of the `.fiz`), `drive.generator` (main
generator of diesel-electric vehicles: excitation, voltage ellipse, power limit scaled with the engine speed; feeds
`drive.dc_series`). The engine shaft goes to exactly one of the last three: that block holds the engine speed and its
inertia (`.fiz` AIM / 2π) and returns it on the back channel; a locked clutch makes the engine follow the wheels.
`drive.diesel` keeps `Mains` closed while the engine runs and, with `main_engine`, sets `enrot`, `dizel_fill`, `Mm`,
`EnginePower`; the gearbox and the hydraulic transmission set `dizel_engage` and `hydro_TC_*`. Graph vehicles have no
`EngineType`, so the engine sounds use the generic formulas of the vehicle sounds.

**Pneumatics and brakes**: `pn.compressor`, `pn.governor`, `pn.reservoir` (main reservoir and feed pipe), `br.handle`
(driver's brake valve; `type` as `BrakeHandle` of the `.fiz`; `pos` in the valve's own scale, e.g. -2..6 for FV4a,
-1..10 for MHZ_K8P), `br.local_handle` (FD1, Knorr), `br.pipe` (brake pipe; `vent` is the emergency valve),
`br.distributor` (`type` as `BrakeValve`, `mode` as `BrakeDelays` of the `.fiz`), `br.rigging` (brake force from the
cylinder pressure; the sum is the brake force of the vehicle), `pn.spring_brake`.

**Security**: `sec.ca` (TSecuritySystem of the body: vigilance, cab signalling with the magnets of the track, radio stop;
`brake` for the `vent` of the brake pipe).

**Interfaces**: `if.pg`, `if.pz` (end cocks of the brake pipe and the feed pipe at one end; the exchange with the
neighbour is open only when both cocks of the coupling are open; the `.fiz` vehicles have them always open),
`if.heat` (train heating through the couplers, from the main circuit), `if.ai` (intents of the AI driver; not used yet),
`if.line` (below).

Multiple unit control, `if.line`: a named control line shared by the coupled graph vehicles, like a train wire (params
`name`, `combine` max|min|sum|maxabs|or, `reverse`, `pair`, `coupling`
control|permanent|coupler|highvoltage|heating|power110v|power24v|power3x400v). A vehicle drives the line when `in` is
connected and `active` (default 1) is set; `out` combines the values of all driving vehicles in this vehicle's
orientation (`reverse`: the value of a vehicle facing the other way changes sign; `pair`: such a vehicle gives its pair
line, e.g. drzwi_lewe ↔ drzwi_prawe), `vehicles` counts the vehicles on the line, `drivers` the driving ones. The line
ends at a coupling without the flag, a vehicle without a graph, or a vehicle without the line. Values of the other
vehicles arrive with up to one step of delay. Multiple unit control between graph and `.fiz` vehicles isn't supported.

## Graph vehicles in the simulator

- The body (`TMoverParameters`) is filled from the `body` section and the equipment parameters, then
  `CheckLocomotiveParameters` runs as for a `.fiz` vehicle: the brake valves (`Hamulec`, `Handle`, `LocHandle`) are
  created by the same factories, from the parameters of `br.distributor`, `br.handle` and `br.local_handle`, and the
  blocks drive them. The initial state follows the scenario (standing vehicles braked, moving ones ready).
- In every physics step the graph runs in place of the subsystems of `compute_movement_`. The tractive force is the sum
  of the `drive.axle` blocks, the brake force the sum of the `br.rigging` blocks; slip, couplers, motion resistance and
  gravity stay native. The brake pipe and the feed pipe exchange air with the neighbours as before, so graph vehicles
  run in trains with `.fiz` wagons; without `br.pipe` or `pn.reservoir` the pipes of the body just pass the air on.
- The fields other parts of the simulator read are kept by the blocks: `Mains` (`el.hsb`, `drive.diesel`), `ConverterFlag`,
  `CompressorFlag`, `Pantographs[].is_active`, `PantPress`, `Im`, `EngineVoltage`, `enrot` (drives), `Battery`,
  `PipePress`, `ScndPipePress`, `BrakePress`, `Compressor`, `iLights`, the doors.
- The cab: the controls and indicators of the `.mmd` aren't used; the cab blocks of the graph are created in the
  cab model when the cab is entered, with labels `graph:<id>` (both mouse buttons operate them through the
  `vehiclegraphcontrol` command, whose payload is the id). Commands bound with `command`, `key_inc`, `key_dec` reach
  the graph before the simulator's own handlers.
- Sounds: the `sounds:` section of the `.mmd` and the cab sounds aren't read; `fb.sound` blocks and the sounds of the
  cab switches and lamps play instead.
- The AI doesn't drive graph vehicles yet: a graph vehicle left to the AI stays as its graph keeps it, with a warning in
  the log. Graph vehicles control each other through `if.line`; multiple unit control between graph and `.fiz`
  vehicles isn't supported.
- Messages of the graph (lua `print`, script errors) and the problems of the file go to the log.
- The inspector: window "Vehicle graph" (menu Mode windows) shows the blocks of the occupied vehicle with the values of
  their ports and their states, and forces the signal outputs.
- `fb.export` values appear in the state passed to the python screens of the cab.

## Live preview server

Off unless enabled with `-graph-preview [port]` on the command line, or `vehiclegraph_preview yes` (and optionally
`vehiclegraph_preview_port 7420`) in `eu07.ini`. The simulator listens on `127.0.0.1`, port 7420 by default, for
connections of the editor. One JSON object per line (UTF-8, `\n`), protocol version 1:

| client | simulator |
|---|---|
| `{"op":"hello","version":1}` | `{"op":"hello","version":1,"simulator":"MaSzyna <version>"}` |
| `{"op":"list"}` | `{"op":"vehicles","items":[{"name":"ET22-1001","file":"dynamic/pkp/et22/303e.vhg","graph":"303E"}]}` |
| `{"op":"attach","vehicle":"ET22-1001"}` | `{"op":"attached","vehicle":"...","file":"..."}`, then 10 times a second `{"op":"values","time":12.34,"signals":{"node.port":v},"states":{"node:state":v},"links":{"node.port":[fwd,back]}}` |
| `{"op":"detach"}` | |
| `{"op":"switch","node":"id","action":"press\|release\|toggle\|inc\|dec\|set\|value","value":0}` | |
| `{"op":"force","port":"node.port","value":1.0}`, `{"op":"unforce","port":"node.port"}` | |
| `{"op":"reload"}` | `{"op":"reloaded"}` or `{"op":"error","message":"..."}` with the problems of the file |
| `{"op":"submodels"}` | `{"op":"submodels","vehicle":{"model":"...","submodels":[...]},"lowpoly":{...} or null,"cabs":[{"cab":0,"model":"...","submodels":[...]}]}` |

`submodels` (attached only) gives the sorted names of the submodels of the vehicle model, the low poly interior and the
cab models listed in the `.mmd` (`cabNmodel:`; `cab1model: none` gives the vehicle model), for the pickers of the
editor. Cab models not loaded yet are loaded through the models manager, as when switching cabs; in a network game
only the player's own train lists its cabs. Older simulators answer with an `unknown op` error.

Errors are answered with `{"op":"error","message":"..."}`; the connection stays open. `reload` reads the attached
vehicle's `.vhg` again: the body keeps its state, the blocks start anew. The server runs on the simulation thread,
polled once per frame.

## Lua blocks

A `lua.block` node holds a lua 5.1 script in `"lua"`. The script defines the block's ports, parameters and behaviour:

```lua
block "power_brake" {
  inputs  = { lever = "float", ready = "bool" },    -- bool, int or float
  outputs = { traction = "float", ed_brake = "float" },
  params  = { ramp_up = 0.5 },                      -- numeric ones can be set per node as "lua.ramp_up"
  state   = { t = 0.0 },                            -- copied into self, with the parameters
  rate    = 0,                                      -- updates per second; 0 or none: every step
  update = function(self, i, o, dt)                 -- dt: time since the previous update
    o.traction = ...
  end,
}
```

```json
{ "id": "demand", "type": "lua.block", "params": { "lua.ramp_up": 0.8 }, "lua": "block \"power_brake\" { ... }" }
```

- Every vehicle has a lua state of its own, separate from the scenery's one, and every block an environment of its own:
  globals and changes to the libraries don't leak between blocks.
- Available: the basic functions, `string`, `table`, `math`, `bit` (luajit), `coroutine`. Not available: `io`, `os`,
  `require`, `load`/`loadstring`/`dofile`, `debug`, `ffi`, `jit`. `print` writes to the log (at most 50 messages per block).
- `math.random` uses a generator seeded from the node id, so runs are repeatable; the editor uses the same generator.
- The jit compiler is off for these states and every call has an instruction budget (2 million), so an endless loop
  stops its block instead of the simulator.
- A block whose script fails stops with a message in the log; its outputs keep their last values and the rest of the
  vehicle keeps working. A script which fails while loading, or doesn't define the block, makes the file invalid.
- Outputs keep their values between updates until the script changes them. `int` values are rounded.
- Lua blocks need a simulator built with `WITH_LUA` (the default); other builds reject graphs with lua blocks.

## Modules

A module is a graph of logic kept in its own file (by convention `.vhgm`, the same format as `.vhg`) and used by
vehicle graphs as one block:

```json
{ "id": "k1", "type": "module", "params": { "file": "modules/latch.vhgm" } }
```

- `file` is relative to the file of the graph using the module.
- The ports of the module are its `module.input` and `module.output` blocks; the id of the block is the name of the
  port, `type` gives its type (`bool`, `int`, `float` or a physical domain, e.g. `high_voltage`), `label` a description.
  A `module.input` has a `default` value, which the module sees when the input isn't connected.
- When the graph is loaded, the module node is replaced with the blocks of the module, with ids `<module node>/<block>`
  (e.g. `k1/sr`), and its connections are passed to the blocks connected to the interface blocks. The values in the
  inspector, the live preview and the results of `-graph-run` use these ids. Modules can use other modules; a module
  using itself, a missing file and a connection to an unknown port of a module are errors. Loops through modules need
  a deferred block, as in the graph itself.
- The body and the profiles of a module are ignored.
- A module file can be run on its own with `-graph-run`: its inputs give their default values (or the forced ones from
  the script) and its outputs have the state `value`.

`tests/module.vhg` (with `tests/modules/`) uses two instances of a module with a nested module.

## Command line tools

```
eu07 -graph-catalog [-o <catalog.json>]
eu07 -graph-run <file.vhg> [-script <events>] [-o <results.csv>] [-dt <step, s>] [-t <duration, s>] [-every <steps>] [-state braked|ready] [-consist <file.vhg>[:r]]...
```

`-graph-catalog` writes the description of all block types, ports and parameters, the body entries, the interface
profiles, the commands of the simulator (`"commands": [{"name","target","mode"}]`, the names cab switches can be bound
to) and the preview protocol, for the external editor.

`-graph-run` loads and checks the graph, reports problems, and runs it without the simulator. As in the simulator, body
entries the graph doesn't give come from the `.fiz` file of the same name, if there is one. It builds the vehicle
body from the graph as the simulator does (`-state` chooses the initial state: `braked`, the default, as a vehicle
standing in a scenario, or `ready`), runs the blocks and moves the body along a straight level track without
neighbours. The results are the values of all signal outputs and the states of the blocks (e.g. `lamp:lit`), and for
graphs with a body section the speed (`body:v`, km/h), tractive and brake force (`body:ft`, `body:fb`, kN), brake pipe,
main reservoir and brake cylinder pressures (`body:pg`, `body:zg`, `body:bc`), separated with `;`.
The script lists events, one per line:

```
# <time, s> <operation> <target> [value]
0.0  toggle  bat
6.0  press   ws_on
6.3  release ws_on
11.0 inc     reverser
21.0 set     lever 10
1.1  force   x.q 3
1.7  unforce x.q
```

Operations on cab switches: `press`, `release`, `toggle`, `inc`, `dec`, `set <position>`, `value <value>`.
Operations on signal ports: `force <node.port> <value>`, `unforce <node.port>`.

Consists: `-consist <file.vhg>` adds a vehicle behind the previous one, coupled by its front coupler; `-consist
<file.vhg>:r` adds it reversed (coupled by its rear coupler). Repeatable. Relative paths are tried as given, then
relative to the first vehicle's file. All coupling flags are set (brake pipes connect); vehicle 1 has the occupied
cab; each vehicle moves on its own (no coupler forces). Script targets of vehicle k ≥ 2 are prefixed `k:` (e.g.
`1.0 force 2:dir.q 1`, `2.0 press 3:button`); its result columns are prefixed `k:` and follow the columns of vehicle 1.
Graphs with more than one `el.hsb` get a `body:mains` column (Mains of the body).

On Windows the simulator runs without a console, so use `-o` for the results, and redirect the output to a file
to see the messages, e.g. `eu07.exe -graph-run test.vhg > log.txt`.

## Tests

```
python3 tools/vehicle_graph/run_tests.py <path to the simulator executable> [--update]
```

- `tests/*.vhg` start with `// expect: ok`, `// expect: error <text>` or `// expect: warning <text>`;
- every `.vhg` with a `.script` is run twice; both runs have to give identical results, equal to `.expected.csv`
  within a relative tolerance of 1e-6 (the body computations may differ in the last digits between compilers).
  A script line `# options: ...` adds options of `-graph-run`, e.g. `# options: -state ready -every 100`.
  `--update` replaces the expected results with the current ones, after checking the change.

`examples/` holds two whole vehicles built from their `.fiz` files: `303e.vhg` (ET22: series DC motors with resistor
starting, LSt, FV4a, FD1) and `vectron.vhg` (induction motors, EStED with electrodynamic brake blending, MHZ_K8P,
spring brake); their scripts start the locomotive from the cab, drive off and stop with the train brake. The
`*-logic.vhg` files are the cab logic of the same vehicles from the first stage. `tests/brakes.vhg` charges the
brake pipe of a locomotive with a train of wagons and brakes it. `tests/diesel-mech.vhg`, `diesel-hydro.vhg` and
`diesel-electric.vhg` start a diesel engine and drive off with each transmission; `multisystem.vhg` switches between two
supply systems with two breakers; `consist.vhg` (with `consist-short.vhg`) runs a consist of four vehicles, one reversed,
connected with `if.line`; `anim.vhg` computes the `veh.anim` and `veh.visible` states.

The tests need a simulator built with lua support. The external editor runs the same files with its `Checks` project
and has to give the same results.

## Checking the code shared with the .fiz vehicles

The code moved out of `TMoverParameters` for the blocks (series motor equations, the induction motor and inverter
computation, the brake rigging force, the feed pipe exchange, the brake valve factories) is called by the `.fiz`
vehicles as before. Configure with `-DVGRAPH_SHADOW_CHECK=ON` to run each moved function side by side with its
original form (`McZapkie/Mover_shadow.cpp`) on copies of the state; every difference goes to the log as
`VGRAPH_SHADOW_CHECK: <vehicle>: <function> differs`. Drive the reference vehicles (series motor locomotives and EMUs,
induction motor locomotives and EMUs with and without speed control and blending, wagons with ESt3, LSt, EP2, KE)
and check the log; then the original forms can be removed.
