#pragma once

#include <functional>
#include <string>
#include <vector>

#include "McZapkie/MOVER.h"

namespace test {

// what the rules over time need of the step before
struct snapshot {
	double time;
	double V;
	double DistCounter;
};

// a rule of physics or of the vehicle's circuits, which holds for every vehicle after every step, whatever
// the test does with it. every golden trace checks them as it samples (see golden.h), on every step, not only
// on the recorded ones, and with no comparison to a recorded file, so they hold on any compiler. a change which
// alters the behaviour on purpose records the traces anew, and the rules still check the new behaviour makes sense.
struct invariant {
	std::string name;
	// Before is null at the first step of a trace
	std::function<bool(TMoverParameters const &Now, double Time, snapshot const *Before)> holds;
	// the values the rule looks at, shown when it's broken
	std::function<std::string(TMoverParameters const &Now)> explain;
};

std::vector<invariant> const &invariants();

// the traces where the game breaks a rule, as it is, with the reason (see COVERAGE.md, what the tests found)
bool excused(std::string const &Trace, std::string const &Invariant);

} // namespace test
