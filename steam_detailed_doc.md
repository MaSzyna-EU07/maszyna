# Detailed steam locomotive model (`McZapkie/steam/`, namespace `steam::detailed`)

A crank-angle-resolved model of a two-cylinder simple-expansion steam locomotive, as a **standalone library**. Nothing in it
is wired into `Mover.cpp` or `MOVER.h`: the integration waits for the `Mover.cpp` refactor. It is built into `eu07_core`
and tested, but the game doesn't call it, so it changes nothing in the simulator, in the scenery files or in the vehicle files.

## Relation to #504

[#504](https://github.com/MaSzyna-EU07/maszyna/pull/504) (`McZapkie/steam.cpp`, namespace `steam`) is a lumped model: one
control volume for the boiler, mean cylinder cycle, parameters of a Ty2. This library is the other end of the scale: the cycle
is integrated over the crank angle, with the valve gear solved exactly, wall heat exchange, the fire bed, the draught
balance and the superheater. The two don't share a name (`steam/` directory and `steam::detailed` here, `steam.h/.cpp`
and `steam` there), so they coexist. A lumped model may be fitted to the detailed one, or the detailed one may serve as a
reference for it.

## Layout

| File | Contents |
| --- | --- |
| `properties` | IAPWS-IF97 regions 1, 2 and 4 as tables (ln p × superheat grid, built once); equilibrium state from energy and volume; orifice flow |
| `valve_gear` | Heusinger (Walschaerts) gear: piston, valve, die block solved by Newton on the exact geometry; port openings, events, cutoff ↔ reverser position |
| `engine` | steam chest, four cylinder chambers, exhaust, blast nozzle, cylinder walls, friction; `engine::step(dt, omega, input)` |
| `boiler` | fire bed, firebox, tubes and flues with the superheater elements, draught with the ejector, water and steam in equilibrium, safety valves, carry-over, fusible plug |
| `locomotive` | engine + boiler + injectors, air pump, generator, heating and the stores of the tender; outputs in the units of the simulation |
| `parameters.h` | all parameters, with the Pt31 as the default and the source of each value in a comment |

## What the game gives and takes

`locomotive::step(dt, omega, controls, environment)` takes the angular velocity of the driving wheels (rad/s) and the controls,
and returns the force at the rims, the torque, the boiler state, the stores and the masses. Adhesion, slip, resistances and
brakes stay in the game. Units: bar gauge, °C, kW, N, N m, kg, kg/s, m; inside the library SI.

## Verification

The code is a port of the Python reference model of the Pt31 (`fizyka/`). `tests/unit/test_steam_detailed.cpp` holds
the checks against it:

* steam tables against IAPWS-IF97 values (the full tables agreed with the reference CSV to 7·10⁻⁷),
* valve gear: cutoff reached, events in order, marching equal to following,
* engine: torque, steam flow and indicated power at 60 km/h, at a crawl, in reverse gear, with the regulator shut (≤ 1%),
* boiler: the steady state at 20 km/h / 50% cutoff (≤ 0.5%, the reference file is the state after a step).

## Not in this change

* no change in `Mover.cpp`, `MOVER.h`, the `.fiz` format or `eu07.ini`; no data is needed from the SVN,
* the `.fiz` section that would fill `parameters` is not defined yet; that belongs with the integration,
* the model keeps the assumptions of the reference model one to one (they are marked "assumed" in `parameters.h`).
