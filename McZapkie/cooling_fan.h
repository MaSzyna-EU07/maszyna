#pragma once

#include "core.h"
#include "basic_device.h"

namespace mover
{
struct cooling_fan : public basic_device
{
	// config
	float speed{0.f}; // cooling fan rpm; either fraction of parent rpm, or absolute value if negative
	float sustain_time{0.f}; // time of sustaining work of cooling fans after stop
	float min_start_velocity{-1.f}; // minimal velocity of vehicle, when cooling fans activate
	// ld outputs
	float revolutions{0.f}; // current fan rpm
	float stop_timer{0.f}; // current time, when shut off condition is active
};

} // namespace mover
