#pragma once

#include "mover/core.h"
#include "mover/TGrateType.h"
#include "mover/TBoilerType.h"

namespace mover
{
struct THeater
{
	/// <summary>Grate parameters.</summary>
	TGrateType Grate;
	/// <summary>Boiler parameters.</summary>
	TBoilerType Boiler;
};

} // namespace mover
