#pragma once

#include "core.h"

namespace mover
{
struct engine_generator
{
	// ld inputs
	/// <summary>Pointer to the prime-mover revolutions [rev/s].</summary>
	double *engine_revolutions; // revs per second of the prime mover
	// config
	/// <summary>Minimum working rev rate [rev/s].</summary>
	double revolutions_min; // min working revolutions rate, in revs per second
	/// <summary>Maximum working rev rate [rev/s].</summary>
	double revolutions_max; // max working revolutions rate, in revs per second
	/// <summary>Voltage generated at <c>revolutions_min</c> [V].</summary>
	double voltage_min; // voltage generated at min working revolutions
	/// <summary>Voltage generated at <c>revolutions_max</c> [V].</summary>
	double voltage_max; // voltage generated at max working revolutions
	// ld outputs
	/// <summary>Current rev rate [rev/s].</summary>
	double revolutions;
	/// <summary>Current generated voltage [V].</summary>
	double voltage;
};

} // namespace mover
