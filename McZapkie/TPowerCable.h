#pragma once

#include "core.h"

namespace mover
{
struct TPowerCable
{
	/// <summary>Type of power transmitted.</summary>
	TPowerType PowerTrans;
	/// <summary>Steam pressure (when SteamPower is transmitted) [bar].</summary>
	double SteamPressure;
	// inline _mover__2() {
	//     SteamPressure = 0.0;
	//     PowerTrans = NoPower;
	// }
};

} // namespace mover
