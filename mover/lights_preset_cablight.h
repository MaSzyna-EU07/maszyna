#pragma once

#include "mover/core.h"

namespace mover
{
struct lights_preset_cablight
{
	/// <summary>Bits of the optional third column of a LightsList row.</summary>
	enum flag
	{
		/// <summary>The cab light is on.</summary>
		light_on = 1 << 0,
		/// <summary>The cab light is dimmed.</summary>
		light_dimmed = 1 << 1
	};
	bool light{false};
	bool dim{false};
};

} // namespace mover
