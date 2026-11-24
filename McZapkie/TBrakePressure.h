#pragma once

#include "core.h"

namespace mover
{
struct TBrakePressure
{
	/// <summary>Brake pipe pressure target [bar].</summary>
	double PipePressureVal = 0.0;
	/// <summary>Brake cylinder pressure target [bar].</summary>
	double BrakePressureVal = 0.0;
	/// <summary>Flow speed (orifice scaling).</summary>
	double FlowSpeedVal = 0.0;
	/// <summary>Brake system this entry applies to.</summary>
	TBrakeSystem BrakeType = TBrakeSystem::Pneumatic;
};

using TBrakePressureTable = std::map<int, TBrakePressure>;

} // namespace mover
