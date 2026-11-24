#pragma once

#include "core.h"

namespace mover
{
struct TAccumulator
{
	/// <summary>Maximum capacity [Ah].</summary>
	double MaxCapacity;
	/// <summary>Source used to recharge the battery.</summary>
	TPowerSource RechargeSource;
	// inline _mover__1() {
	//     MaxCapacity = 0.0;
	//     RechargeSource = NotDefined;
	// }
};

} // namespace mover
