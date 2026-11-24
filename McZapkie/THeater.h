#pragma once

#include "core.h"
#include "TGrateType.h"
#include "TBoilerType.h"

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
