#pragma once

#include "core.h"

namespace mover
{
struct TShuntScheme
{
	double Umin = 0.0;
	double Umax = 0.0;
	double Pmin = 0.0;
	double Pmax = 0.0;
};

using TShuntSchemeTable = TShuntScheme[33];

} // namespace mover
