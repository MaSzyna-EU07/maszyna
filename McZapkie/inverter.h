#pragma once

#include "core.h"

namespace mover
{
struct inverter
{
	double Freal = 0.0;
	double Request = 0.0;
	bool IsActive = true;
	bool Activate = true;
	bool Error = false;
	bool Failure_Drive = false;
	bool Failure_Const = false;
};

} // namespace mover
