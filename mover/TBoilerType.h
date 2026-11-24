#pragma once

#include "mover/core.h"

namespace mover
{
struct TBoilerType
{
	/// <summary>Boiler internal volume [m³].</summary>
	double BoilerVolume;
	/// <summary>Heat exchange surface of the boiler [m²].</summary>
	double BoilerHeatSurface;
	/// <summary>Superheater surface [m²].</summary>
	double SuperHeaterSurface;
	/// <summary>Maximum water volume [m³].</summary>
	double MaxWaterVolume;
	/// <summary>Minimum (safety) water volume [m³].</summary>
	double MinWaterVolume;
	/// <summary>Maximum boiler pressure [bar].</summary>
	double MaxPressure;
	// inline TBoilerType() {
	//     BoilerVolume = 0.0;
	//     BoilerHeatSurface = 0.0;
	//     SuperHeaterSurface = 0.0;
	//     MaxWaterVolume = 0.0;
	//     MinWaterVolume = 0.0;
	//     MaxPressure = 0.0;
	// }
};

} // namespace mover
