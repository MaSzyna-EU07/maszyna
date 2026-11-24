#pragma once

#include "core.h"

namespace mover
{
struct neighbour_data
{
	TDynamicObject *vehicle{nullptr}; // detected obstacle
	int vehicle_end{-1}; // facing end of the obstacle
	float distance{10000.f}; // distance to the obstacle // NOTE: legacy value. TBD, TODO: use standard -1 instead?
};

} // namespace mover
