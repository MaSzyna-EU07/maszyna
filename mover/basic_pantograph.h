#pragma once

#include "mover/core.h"
#include "mover/basic_valve.h"

namespace mover
{
struct basic_pantograph
{
	// ld inputs
	basic_valve valve; // associated pneumatic valve
	// ld outputs
	bool is_active{false}; // device is working
	bool sound_event{false}; // indicates last state which generated sound event
	double voltage{0.0};
};

} // namespace mover
