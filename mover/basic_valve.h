#pragma once

#include "mover/core.h"
#include "mover/basic_device.h"

namespace mover
{
struct basic_valve : basic_device
{
	// config
	bool solenoid{true}; // requires electric power to operate
	bool spring{true}; // spring return or double acting actuator
};

} // namespace mover
