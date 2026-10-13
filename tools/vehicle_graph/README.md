# Vehicle graph (.vhg)

A vehicle graph describes a vehicle as connected blocks, in place of the `.fiz` file. The graph holds the cab
controls and indicators, the logic, the sounds and, in the following stages, the electrical, mechanical and
pneumatic systems of the vehicle. The simulator code is in `vehicle/graph/`.

Current stage: the core of the graph with logic and cab blocks, available through the command line tools below.
Vehicles aren't loaded from `.vhg` files yet.

## File format, version 2

A `.vhg` file is JSON. Comments (`//`, `/* */`) are allowed.

```json
{
	"format": "maszyna-vehicle-graph",
	"version": 2,
	"vehicle": { "name": "ET22" },
	"settings": { "low_voltage_simulation": false },
	"body": { },
	"profiles": { "brake_pipe": 1 },
	"nodes": [
		{ "id": "bat", "type": "cab.switch", "title": "Battery", "params": { "behaviour": "bistable", "submodel": "battery_sw:" } },
		{ "id": "lamp", "type": "fb.lamp", "params": { "color": "red" } }
	],
	"connections": [
		{ "from": "bat.pos", "to": "lamp.on" }
	],
	"layout": { }
}
```

- `nodes[].id`: letters, digits, `_` and `-`; unique in the file.
- `nodes[].type`: block type from the catalog (see `-graph-catalog`).
- `nodes[].params`: typed values. Numbers are JSON numbers, switches are `true`/`false`, choices are ASCII ids
  from the catalog (e.g. `"bistable"`, `"rotate"`, `"ne"`). Parameters left out take their default values.
- `connections`: `"node.port"` of an output to `"node.port"` of an input. An output can feed many inputs; an input
  has at most one source. Unconnected inputs see their default value.
- `settings.low_voltage_simulation`: when enabled, blocks with low voltage ports simulate the voltages and currents
  of the control circuits (e.g. a lamp needs enough voltage and draws current). When disabled, low voltage
  connections have no effect and the apparatus behaves as if always powered.
- `body`, `profiles`: parameters of the vehicle body and standard interfaces between vehicles; used by the next stages.
- `layout`: positions of the blocks in the editor; ignored by the simulator.

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

### Order of evaluation

Blocks are evaluated once per step in the order given by the connections. A loop of connections has to pass through
a deferred block (`logic.sr`, `logic.ton`, `logic.tof`, `logic.delay`), whose outputs change in the next step; other
loops are rejected when the file is loaded. Back values of physical connections are always taken from the previous
step. The graph has no other source of time or randomness, so the same inputs give the same results.

## Command line tools

```
eu07 -graph-catalog [-o <catalog.json>]
eu07 -graph-run <file.vhg> [-script <events>] [-o <results.csv>] [-dt <step, s>] [-t <duration, s>] [-every <steps>]
```

`-graph-catalog` writes the description of all block types, ports and parameters, for the external editor.

`-graph-run` loads and checks the graph, reports problems, and runs it without the simulator. The results are
the values of all signal outputs and the states of the blocks (e.g. `lamp:lit`), separated with `;`.
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

On Windows the simulator runs without a console, so use `-o` for the results, and redirect the output to a file
to see the messages, e.g. `eu07.exe -graph-run test.vhg > log.txt`.

## Tests

```
python3 tools/vehicle_graph/run_tests.py <path to the simulator executable> [--update]
```

- `tests/*.vhg` start with `// expect: ok`, `// expect: error <text>` or `// expect: warning <text>`;
- every `.vhg` with a `.script` is run twice; both runs have to give identical results, equal to `.expected.csv`.
  `--update` replaces the expected results with the current ones, after checking the change.

`examples/` holds the cab logic of two vehicles, modelled after the 303E (ET22, resistor starting) and Vectron
(induction motors) `.fiz` files.
