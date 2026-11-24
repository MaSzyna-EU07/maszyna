#pragma once

#include "mover/core.h"

namespace mover
{
struct basic_device
{
	// config
	start_t start_type{start_t::manual};
	// ld inputs
	bool is_enabled{false}; // device is allowed/requested to operate
	bool is_disabled{false}; // device is requested to stop
	// TODO: add remaining inputs; start conditions and potential breakers
	// ld outputs
	bool is_active{false}; // device is working
};

} // namespace mover
