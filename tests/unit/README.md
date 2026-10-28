# Unit tests

C++ tests of the simulation logic: set up the conditions, run the code, check the outcome. They run
as part of every build of the game, in CI and locally alike, and from the IDE. They need no game
data and no window: the test vehicles are JSON definitions in `data/`, loaded from memory by the
game's own vehicle loader.

The code under test is `eu07_core`, the library of the simulation parts the game links as well
(physics, vehicle definition loader, ...): built once, with the game's settings, and tested as the
game uses it. The python library gets linked along with the game's settings, but the tests don't
use it and run without it.

[COVERAGE.md](COVERAGE.md) says how much of the simulation core the tests reach, what's left and why, and
what they found.

## Running

- **With the build**: building the game (`WITH_TESTS`, on by default) runs them at the end, once the
  game is linked, and prints `Unit tests: 11/11 passed (80 checks)`. A failing test prints the full
  report and fails the build; the game binary gets removed, so the next build links it again and
  reruns them. They also run when only the tests, a definition in `data/` or `run_tests.cmake`
  changed (Ninja and Makefile generators). `-DWITH_TESTS=OFF` builds the game without them.
- **From the IDE**: run or debug the `eu07_unit_tests` target. CLion recognises doctest and lists the
  test cases, so a single one can be run or debugged from the gutter.
- **By hand**: `ctest --test-dir <build dir> --build-config Release`, or the binary itself, which takes
  [doctest options](https://github.com/doctest/doctest/blob/master/doc/markdown/commandline.md), e.g.
  `eu07_unit_tests -tc="brakes*"`. `EU07_UNIT_TESTS_LOG=1` shows the engine log, quiet otherwise.

## Writing a test

Add a `test_<topic>.cpp` here, it's picked up on its own.

```cpp
#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

TEST_CASE("brakes: service braking stops a heavier wagon too") {
	// conditions: the wagon from data/wagon.json, heavier, running at 40 km/h
	test::vehicle wagon{test::definition::load("wagon").set("Param", "M", 60000), 40.0 /* km/h */};
	// action: full service application, as a coupled locomotive would make it
	wagon.hold_brake_pipe(3.5); // bar
	wagon.run(40.0 /* s */, test::track{0.15 /* friction */, 0.0 /* gradient */});
	// outcome
	CHECK(std::abs(wagon.speed()) < 0.1);
}
```

- `definition::load("<name>")` reads `data/<name>.json`, with the sections and keys of the game's
  vehicle definitions; `set()` and `remove()` change it for the test. Table sections give their
  rows as `"rows": [[...], ...]`, keys starting with `_` are comments. A section the loader doesn't
  know is an error (the table of them is in `fiz.cpp`); a misspelt key isn't caught, so check
  in the test that the value the test is about made it to the vehicle. Keep the definitions
  physically sensible: some parts need keys others don't (the ESt brake valves need `Brake.Size`),
  and a test on a half-defined vehicle tests the definition rather than the simulation.
- `definition::fiz("<dir>/<name>")` is one of the game's own vehicles, `data/fiz/<dir>/<name>.json`,
  in the same form, converted from the game's `.fiz` with the files it includes (the list and how to
  add one: `data/fiz/README.md`). The conversion keeps what the game's loader reads of the file and
  checks it reads the same of the converted one (`fiz.h`), so the vehicle is the game's.
- The random numbers of the simulation are reseeded before every test case, so a test gives the
  same result alone or among the others. Loading a vehicle draws some too (the scatter of its brake
  pipe pressure), so a test making vehicles in a loop calls `test::reseed_random()` before each.
- Logic which lives in classes too big to set up in a test (the AI driver, the cab) goes into free
  functions on the vehicle state the class calls, and the test calls them the same way: see
  `vehicle/handover.h`, which `TController::TakeControl()` uses, and `test_handover.cpp`.
- `vehicle` creates the vehicle the way `TDynamicObject::Init()` does and steps it the way the
  simulation loop does, on a straight track of the given friction, gradient, catenary voltage and
  physics iterations a frame. With a cab (`vehicle{definition, 0.0, "name", 1}`) it has a driver.
- `consist` (`consist.h`) couples vehicles into a train and steps them together, coupler forces,
  air, control and power passing between them. The physics reaches a neighbour through its
  `TDynamicObject`, for its `MoverParameters` only; the tests have a stand-in of it with just that.
- `drive.h` has the driver's part, made of the AI's steps: `start_up()` (battery, pumps,
  pantographs, line breaker, converter, compressor), `drive()` (running, acknowledging the
  vigilance), `release_brakes()`, `drive_cycle()` (a whole run from start-up to a stop).
- `scenario` (`scenario.h`) puts a train, its driver and a golden trace for each vehicle together,
  for a test of a feature: `act()` marks an event in the traces, `wait()` runs the train.
- Check behaviour (speeds, pressures, states the player sees) rather than internal flags, and make
  the test check its own preconditions (`REQUIRE` that the braking skidded, before checking what a
  skid leads to).

## Golden traces

A golden trace pins down how the simulation behaves now, for changes which mustn't alter it, like
restructuring a function: the values of some channels of a vehicle over a run, compared with the
ones recorded from the code before, in `data/golden/<name>.csv`. A difference fails the test, naming
the channel, the time and both rows.

```cpp
test::trace trace{"eu07/notch up", test::channels::motion() + test::channels::traction()};
eu07.record(trace); // samples every 10th step
eu07->MainSwitch(true);
trace.mark("main switch"); // shows in the event column of the row
eu07.run(30.0);
trace.check();
```

- `EU07_UNIT_TESTS_UPDATE_GOLDEN=1` records the traces anew, for a new test or a change which alters
  the behaviour on purpose; the diff of the files shows what it altered, review it like code.
- Values are compared within a relative 1e-9, as the CI builds with GCC and glibc, whose maths
  functions can differ from MSVC's in the last bits. `EU07_UNIT_TESTS_GOLDEN_EXACT=1` compares them
  exactly: the check for a change which shouldn't alter anything, on the machine which recorded them.
- Keep the runs short (tens of seconds): the simulation feeds back on itself, and a last bit grows.
- A trace with no assertion about what the behaviour should be is a characterisation, bugs included;
  put the behaviour the test is about into `CHECK`s besides.

### Invariants

Every trace also checks, on every step of its vehicle and not only on the recorded ones, the rules
which hold for any vehicle whatever a test does (`invariants.h`): no NaN, the brake cylinders under
the vehicle's highest pressure, the pipes under the highest governor setting, no motor current with
the line breaker open, no tractive force without power, the distance counted as the speed runs, no
acceleration over 5 g, and a few more. They compare nothing with a recorded file, so they hold on
any compiler, and they still check a behaviour recorded anew on purpose makes sense. A broken rule
fails the test, naming the time, the last event and the values:

```
invariant "no motor current with the line breaker open" broken in traction/et42 fuse/0 at 63.200 s,
after "dynamic brake": Mains=0 Im=-412.5 Itot=-412.5
```

Where a test makes a vehicle break a rule (a collision), or a game vehicle breaks one as it is, the
list of excuses in `invariants.cpp` names the trace, the rule and the reason; the second kind goes
into the findings of [COVERAGE.md](COVERAGE.md) too.

## Code to test, and when the tests fail to link

A source joins the tested code by moving from `SOURCES` to `CORE_SOURCES` in the main
`CMakeLists.txt`; the game links it from the library from then on. The library reaches for a little
of the rest of the engine, which `engine_stubs.cpp` stands in for in the tests (logs, settings,
scenery layers). If a change makes it use something new, either move the real source into the
library too, when it stands on its own, or add a stub to `engine_stubs.cpp`.
