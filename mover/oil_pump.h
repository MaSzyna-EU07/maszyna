#pragma once

#include "mover/core.h"
#include "mover/basic_device.h"

namespace mover
{
struct oil_pump : public basic_device
{
	// config
	float pressure_minimum{0.f}; // lowest acceptable working pressure
	float pressure_maximum{0.65f}; // oil pressure at maximum engine revolutions
	// ld inputs
	float resource_amount{1.f}; // amount of affected resource, compared to nominal value
	// internal data
	float pressure_target{0.f};
	// ld outputs
	float pressure{0.f}; // current pressure
};

} // namespace mover
