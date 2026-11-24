#pragma once

#include "mover/core.h"
#include "mover/basic_device.h"

namespace mover
{
struct water_pump : public basic_device
{
	// ld inputs
	// TODO: move to breaker list in the basic device once implemented
	bool breaker{false}; // device is allowed to operate
};

} // namespace mover
