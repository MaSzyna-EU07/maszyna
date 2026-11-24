#pragma once

#include "core.h"

namespace mover
{
struct circuit_config_t
{
	float temp_min{-1}; // lowest accepted temperature
	float temp_max{-1}; // highest accepted temperature
	float temp_cooling{-1}; // active cooling activation point
	float temp_flow{-1}; // fluid flow activation point
	bool shutters{false}; // the radiator has shutters to assist the cooling
};

} // namespace mover
