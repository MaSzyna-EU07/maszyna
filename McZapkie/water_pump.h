#pragma once

#include "core.h"
#include "basic_device.h"

namespace mover
{
struct water_pump : public basic_device
{
	// ld inputs
	// TODO: move to breaker list in the basic device once implemented
	bool breaker{false}; // device is allowed to operate
};

} // namespace mover
