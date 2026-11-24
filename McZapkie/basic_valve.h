#pragma once

#include "core.h"
#include "basic_device.h"

namespace mover
{
struct basic_valve : basic_device
{
	// config
	bool solenoid{true}; // requires electric power to operate
	bool spring{true}; // spring return or double acting actuator
};

} // namespace mover
