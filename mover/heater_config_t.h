#pragma once

#include "mover/core.h"

namespace mover
{
struct heater_config_t
{
	float temp_min{-1}; // lowest accepted temperature
	float temp_max{-1}; // highest accepted temperature
};

} // namespace mover
