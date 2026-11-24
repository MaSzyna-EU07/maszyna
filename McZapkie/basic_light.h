#pragma once

#include "core.h"
#include "basic_device.h"

namespace mover
{
struct basic_light : public basic_device
{
	// config
	float dimming{1.0f}; // light strength multiplier
	// ld outputs
	float intensity{0.0f}; // current light strength
};

} // namespace mover
