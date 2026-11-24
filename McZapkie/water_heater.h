#pragma once

#include "core.h"
#include "heater_config_t.h"

namespace mover
{
struct water_heater
{
	// config

	heater_config_t config;
	// ld inputs
	bool breaker{false}; // device is allowed to operate
	bool is_enabled{false}; // device is requested to operate
	// ld outputs
	bool is_active{false}; // device is working
	bool is_damaged{false}; // device is damaged
};

} // namespace mover
