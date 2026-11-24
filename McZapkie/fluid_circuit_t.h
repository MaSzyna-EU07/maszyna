#pragma once

#include "core.h"
#include "circuit_config_t.h"

namespace mover
{
struct fluid_circuit_t
{

	circuit_config_t config;
	bool is_cold{false}; // fluid is too cold
	bool is_warm{false}; // fluid is too hot
	bool is_hot{false}; // fluid temperature crossed cooling threshold
	bool is_flowing{false}; // fluid is being pushed through the circuit
};

} // namespace mover
