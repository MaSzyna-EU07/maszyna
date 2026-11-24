#pragma once

#include "core.h"

namespace mover
{
struct power_coupling
{
	double current{0.0};
	double voltage{0.0};
	bool is_local{false}; // whether the power comes from external or onboard source
	bool is_live{false}; // whether the coupling with next vehicle is live
};

} // namespace mover
