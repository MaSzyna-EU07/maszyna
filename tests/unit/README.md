# Unit tests

C++ tests of the simulation logic: set up the conditions, run the code, check the outcome. They run
as part of every build of the game, in CI and locally alike, and from the IDE. They need no game
data and no window: the test vehicles are JSON definitions in `data/`, loaded from memory by the
game's own vehicle loader.

The code under test is `eu07_core`, the library of the simulation parts the game links as well
(physics, vehicle definition loader, ...): built once, with the game's settings, and tested as the
game uses it. The python library gets linked along with the game's settings, but the tests don't
use it and run without it.

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
  know is an error (the table of them is in `vehicle.cpp`); a misspelt key isn't caught, so check
  in the test that the value the test is about made it to the vehicle. Keep the definitions
  physically sensible: some parts need keys others don't (the ESt brake valves need `Brake.Size`),
  and a test on a half-defined vehicle tests the definition rather than the simulation.
- The random numbers of the simulation are reseeded before every test case, so a test gives the
  same result alone or among the others.
- Logic which lives in classes too big to set up in a test (the AI driver, the cab) goes into free
  functions on the vehicle state the class calls, and the test calls them the same way: see
  `vehicle/handover.h`, which `TController::TakeControl()` uses, and `test_handover.cpp`.
- `vehicle` creates the vehicle the way `TDynamicObject::Init()` does and steps it the way the
  simulation loop does, on a straight track of the given friction and gradient. `pair` puts two of
  them on that track facing each other with the given ends and runs them together, as the
  simulation runs the vehicles of a scene: the coupler and buffer forces, collisions and automatic
  coupling come from the physics (`test_couplers.cpp`). Longer consists aren't supported yet.
- Check behaviour (speeds, pressures, states the player sees) rather than internal flags, and make
  the test check its own preconditions (`REQUIRE` that the braking skidded, before checking what a
  skid leads to).

## Code to test, and when the tests fail to link

A source joins the tested code by moving from `SOURCES` to `CORE_SOURCES` in the main
`CMakeLists.txt`; the game links it from the library from then on. The library reaches for a little
of the rest of the engine, which `engine_stubs.cpp` stands in for in the tests (logs, settings,
scenery layers). If a change makes it use something new, either move the real source into the
library too, when it stands on its own, or add a stub to `engine_stubs.cpp`.
