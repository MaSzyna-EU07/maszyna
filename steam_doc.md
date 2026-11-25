# The steam locomotive

`EngineType=SteamEngine` in the `Engine:` section of a vehicle definition makes the vehicle a steam locomotive.
Until now the type was read and then did nothing: no force, no controls, no vehicle of the game data uses it. Its
model is the class `SteamEngine` (`McZapkie/steam.h` / `steam.cpp`): its parameters, its state and the controls the crew
works it with, which stands apart from the vehicle and is driven by `TMoverParameters::steam_Update()`. The defaults are those of the PKP Ty2 (DRB class 52), and the optional `Steam:`
section of the definition sets a locomotive's own. Nothing changes for a definition without `EngineType=SteamEngine`.

## What is modelled

Every step of the physics the model updates a few lumped states: the temperature and the mass of the boiler water,
the mass of the fire, the heat of the fire reaching the water, the superheat, the exhaust flow, and the water and
coal of the tender.

- **Boiler.** The boiler pressure is the saturation pressure of its water (the steam tables, IAPWS-IF97, in
  `steam::saturation`). The water and the steel take up the heat of the fire; the steam taken off takes its latent
  heat, the feed water is warmed up to the water's temperature, and the boiler loses some heat to its surroundings.
  The level in the glass follows the water mass, and shows higher with steam taken (swell). Over the top of the
  glass the boiler primes: water is carried over into the cylinders, does no work and lowers the superheat. Under
  the bottom of the glass, down to the firebox crown, the fusible plug melts and its steam puts the fire out.
- **Safety valves.** They open at the working pressure, are wide open 3% over it, and close 0.3 bar under it.
- **Fire.** The coal burns as fast as the draught lets it: the exhaust of the cylinders through the blast pipe, the
  blower, and a little natural draught. A thin fire burns as much less as it is thinner; a thick one little more
  than the right one. The part of the heat that reaches the water falls with the firing rate. The heat reaches the
  water with a delay.
- **Regulator and steam chest.** The regulator is a throttle: choked under a critical pressure ratio, passing less
  as the chest pressure nears the boiler's. The chest pressure is where the regulator passes as much steam as the
  cylinders take, so it falls at speed with the regulator part open, as on the real engine.
- **Cylinders.** The mean effective pressure of an indicator diagram: admission up to the cutoff, hyperbolic
  expansion, exhaust against the back pressure of the blast pipe, with the clearance volume. The admission pressure
  loses a part with speed (wire drawing). The diagram factor stands in for release, compression and throttling.
  The steam taken per revolution is the steam admitted (an ideal gas at the superheated temperature) less what is
  kept in the clearance, plus the initial condensation. In mid gear (the reverser at 0) no steam is taken. With the
  chest pressure low the pressure equalisers open and the pistons drift without pumping.
- **Force.** The force at the rims is the mean effective pressure on the swept volume per revolution, divided by the
  circumference of the driving wheels and the mechanical efficiency. At low speed it swings with the angle of the
  cranks (two cylinders at 90°: 0.79 to 1.11 of the mean), which makes the wheels slip more easily starting.
- **Fireman.** The automatic fireman (the default) keeps the fire up with the steam taken and the pressure a little
  under the working one, keeps the water a little over the middle of the glass with the injectors (the exhaust steam
  one running, the live steam one standing), and works the blower. By hand the crew sets the firing rate, the
  injectors and the blower (`TMoverParameters::Steam.Controls`, a `SteamEngine`).
- **Air pump.** `CompressorPower=Engine` (and the legacy `Main`) of a steam locomotive is its steam air pump: it
  runs with the boiler over 3 bar and takes `AirPumpFlow` of steam.

The smoke of the chimney (a `smokesource_*` submodel of the model) follows the exhaust.

## The controls

For now the model sits on the controls every locomotive has:

| Control | Steam locomotive |
| --- | --- |
| master controller (`MCPN` positions) | the regulator, closed at 0, wide open at the last position |
| reverser | forward / back; at 0 mid gear |
| secondary controller (`SCPN` positions) | the cutoff: 0 is full gear (`CutoffMax`), the last position the shortest cutoff (`CutoffMin`) |

## The `Steam:` section

Every key is optional; the defaults are the Ty2's.

| Key | Default | Unit | Meaning |
| --- | --- | --- | --- |
| `Cylinders` | 2 | | number of cylinders, simple expansion, double acting |
| `CylD` | 0.6 | m | bore |
| `CylS` | 0.66 | m | stroke |
| `CylClearance` | 0.09 | of the swept volume | clearance volume |
| `CutoffMax` | 0.8 | of the stroke | full gear |
| `CutoffMin` | 0.1 | of the stroke | the shortest cutoff |
| `CutoffMaxReverse` | as `CutoffMax` | of the stroke | full gear running backwards, where the valve gear gives less (the Ty23: 0.84 forwards, 0.70 backwards) |
| `DiagramFactor` | 0.95 | | the indicator diagram against the ideal one |
| `MechEff` | 0.85 | | from the cylinders to the rims |
| `WireDrawing` | 1/55 | per √rpm | the admission pressure lost with speed |
| `BackPressure` | 0.04 | bar per (kg/s)² | of the blast pipe |
| `Condensation` | 0.08 | of the steam admitted | initial condensation at short cutoffs |
| `RegulatorFlow` | 7 | kg/s | through the wide open regulator, choked |
| `CocksFlow` | 0.05 | kg/s per bar | through the open cylinder cocks |
| `TorquePulsation` | 1 | 0..1 | how much of the swing of the force over a revolution is felt |
| `EqualiserPressure` | 0.5 | bar | chest pressure under which the pressure equalisers open |
| `BoilerPressure` | 16 | bar | working pressure, where the safety valves open |
| `SafetyValveFlow` | 5 | kg/s | the safety valves wide open |
| `BoilerWater` | 9000 | kg | water with the level in the middle of the glass |
| `GlassWater` | 1200 | kg | water from the bottom to the top of the glass |
| `CrownWater` | 700 | kg | water from the bottom of the glass down to the firebox crown |
| `Swell` | 0.03 | of the glass per kg/s | how much higher the level shows with steam taken |
| `BoilerSteelHeat` | 12 | MJ/K | heat capacity of the boiler's steel |
| `BoilerHeatLoss` | 500 | W/K | heat lost to the surroundings |
| `Superheat` | 160 | K | superheat at the nominal steam rate |
| `SteamRate` | 2.9 | kg/s | nominal steam rate of the boiler |
| `Grate` | 3.89 | m² | grate area |
| `FireMass` | 600 | kg | the right fire on the grate |
| `FiringRateMax` | 550 | kg/(m² h) | coal burned with the strongest draught |
| `CoalHeat` | 29 | MJ/kg | |
| `BoilerEfficiency` | 0.8 | | heat reaching the water, firing gently |
| `BoilerEfficiencyMin` | 0.5 | | and at the highest firing rate |
| `HeatingSurface` | none | m² | the evaporative heating surface (firebox, tubes and flues, on the side of the gases). Given, the efficiency falls with the coal burned an hour per m² of it, down to `BoilerEfficiencyMin` at the 12 kg/(m² h) of the Ty2 fired hardest, and on past it to 1.5 times that rate: a boiler with more surface to its grate keeps more of the heat. Without it, with the firing rate of the grate alone |
| `FireDelay` | 25 | s | from burning to heating the water |
| `ExhaustDraught` | 1 | | draught of the nominal exhaust |
| `BlowerDraught` | 0.35 | | draught of the blower wide open |
| `BlowerFlow` | 0.25 | kg/s | steam of the blower wide open |
| `InjectorFlow1` | 250 | l/min | the live steam injector |
| `InjectorFlow2` | 175 | l/min | the exhaust steam injector |
| `FeedTemperature` | 10 | °C | water in the tender |
| `TenderWater` | 30 | m³ | |
| `TenderCoal` | 10 | t | |
| `AirPumpFlow` | 0.1 | kg/s | steam of the air pump |

## Steam and smoke

A smoke source of the model (a submodel `smokesource_<template>`, its particles from `data/smokesource_<template>.txt`)
shows a part of the steam engine when its template says which, with the optional key `steam:`:

| `steam:` | what it shows | driven by |
|---|---|---|
| `exhaust` | the blast up the chimney, white steam in four puffs a turn of the driving wheels, merging into a steady blast from about 3 turns a second | the exhaust and the blower against the boiler's steam rate (`State.exhaust_flow`, `State.blower`) and the crank angle |
| `smoke` | the smoke of the fire, carried up with the exhaust | the coal burned against the most the grate burns (`State.burn_rate`) |
| `safety_valves` | the safety valves blowing off | `State.safety_valve_flow` |
| `cocks` | the open cylinder cocks, with each stroke | `Controls.cylinder_cocks` and the steam chest pressure |
| `air_pump` | the exhaust of the air pump while it runs | `CompressorFlag`, with steam enough to drive the pump |
| `injectors` | the overflow of the injectors as they feed | `State.injectors` |
| `whistle` | the whistle blowing | `WarningSignal`, with steam in the boiler |
| `generator` | the exhaust of the turbo-generator, steady | the boiler over the air pump's 3 bar |
| `leaks` | steam past the glands of the cylinders, more with the engine working | the steam chest pressure and the revolutions |

Each gives the share of the template's spawn rate and of the opacity and launch velocity of the particles
(`smoke::steam_emission()`, `vehicle/smoke.h`). A template without `steam:`, or a vehicle with no steam engine, works as
before. The AI opens the cylinder cocks while starting, under 10 km/h with the regulator open.

## What is to come

The model is the first step. In rough order:

1. **Calibration against the Ty2's figures.** About 214 kN starting (checked), about 1192 kW indicated, 1200 t at
   65 km/h on the level, and the PKP load tables (1700 t at 50 km/h and 620 t at 80 km/h on the level, 190 t at
   40 km/h on 20 ‰). Several defaults are estimates the sources didn't give: the boiler water, the glass, the
   cutoff range, the valve events, the steam rate and the superheat. The books to get them from are Hütter (DGEG
   2012), Fijałkowski & Kowalewski (WKŁ 1959) and the DR's Merkbuch DV 939. The drawings of the Polish Ty23, a 1'E
   freight locomotive of the same kind, are scanned by the Stacja Muzeum
   (https://cyfrowa.stacjamuzeum.pl/zasoby-muzealne/dokumentacja-techniczna/, six volumes; higher resolution scans
   on request), as are those of the Pt31: a source for the boiler, the valve gear and the cab where the Ty2's own
   aren't at hand; a unit test builds the Ty23 from them (5.8 % over the starting force of its drawing, as the Ty2 is
   over the same formula). Measured on 2026-11-25 with a simple driver keeping the boiler over 13 bar: 1200 t on the
   level 64 km/h (table 65), 1700 t 57 km/h (table 50: the model is about 10 % strong there), 620 t up to the 80 km/h
   of Vmax, and 190 t on 20 ‰ running 40 to 50 km/h at 1200 to 1440 kW while the cutoff is kept at 45 to 60 %
   (table 40), though that driver notches the cutoff too roughly to hold a steady speed.
2. **Cab controls of their own.** A continuous reverser and cutoff screw instead of the secondary controller, the
   regulator as a lever, the cylinder cocks, the blower, the two injectors (steam and water valves), the firehole
   door and the firing, the dampers, the gauges (boiler and chest pressure, the water glasses, the tender), as new
   `user_command`s, `Train.cpp` handlers and cab elements, and in the Python screens.
3. **The tender as a vehicle.** Today the water and coal sit in the locomotive's model; they belong in the tender
   coupled behind it, with taking water and coal at the depot.
4. **Sound.** Chuffs in time with the cranks: four exhausts a revolution for two cylinders, their strength from the
   exhaust flow, the cocks, the safety valves, the injectors, the blower, the air pump. This needs new `.mmd` sound
   entries and samples.
5. **The AI driver** is in: ready with 5 bar in the boiler, the air pump started, the regulator a position at a time,
   the cutoff notched up with the speed and further when the boiler falls behind (`ty2_td_timetable` in
   `tests/scenarios`), and the cylinder cocks opened while starting. Still to come: the blower and the fire on a stop.
6. **Raising steam.** A cold start (`SteamEngine::cool_down()`) from the scenario, with the fire lit and the blower on an
   outside supply until the boiler has pressure.
7. **Faults.** Wheel slip with sanding, priming with the regulator opened too fast, an injector failing to pick up,
   clinker with the fire forced, the fusible plug melted needing a repair.
8. **The valve gear.** The rods are animated by the bones of the model (`bones_doc.md`, the placeholder Ty2 has them);
   the expansion link and the radius rod need two more kinds of points.
9. **A real Ty2** with its model, cab and sounds in the game data, in place of the placeholder `pkp/ty2_v1`.
10. **More classes.** The Ty23 (1'E) and the Pt31 (1'D1') from the drawings of the collection are defined for the
    unit tests (`tests/unit/data/steam_ty23.json`, `steam_pt31.json`) and as placeholder vehicles with their tenders
    (`pkp/ty23_v1`, `pkp/pt31_v1`, not in the SVN yet), each `.fiz` marking with ? what the drawings don't give.
    Both start a few percent over the PKP's formula, as the Ty2 does; the Ty23 takes 1200 t to about 67 km/h and the
    Pt31 500 t to about 90 km/h on the level. With `HeatingSurface` (Ty2 177.7, Ty23 191.1, Pt31 239 m²) the Pt31's
    boiler gives 1.25 times the heat of the Ty2's fired hardest, about the 2000 KM to 1633 KM of their ratings, and
    with the same 650 t of coaches it keeps up 1.15 times the Ty2's indicated power, a few km/h faster on the level
    (87 against 84) and up 5 ‰ (56 against 53). Before, the Pt31's `SteamRate` of 3.9 kept its fireman from firing
    to the full and the two gave the same power. The Pt31's design figure, 650 t up 5 ‰ at 80 km/h, is not reached:
    the coaches of the game, on plain bearings, take about 6 daN/t at 85 km/h, about one and a half times what the
    German formulas give, which is a matter of the resistance of all the game's wagons, not of the steam engine. The Oc-1 boards of the collection give too little for a definition.
11. **What the Ty23's drawings show the model leaves out**, all second-order: the release and compression of the
    valve gear moving with the cutoff (a constant `DiagramFactor` now; drawing 21.03 tabulates them notch by notch),
    the steam a Heusinger gear still admits in mid gear (7 % on the Ty23), and the exhaust-steam injector, which
    heats the feed with the exhaust.
