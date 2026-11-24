#pragma once

#include "mover/core.h"

namespace mover
{
struct TGrateType
{
	/// <summary>Fuel type burned on this grate.</summary>
	TFuelType FuelType;
	/// <summary>Grate surface area [m²].</summary>
	double GrateSurface;
	/// <summary>Fuel transport speed (stoker rate).</summary>
	double FuelTransportSpeed;
	/// <summary>Fuel ignition temperature [°C].</summary>
	double IgnitionTemperature;
	/// <summary>Maximum allowable grate temperature [°C].</summary>
	double MaxTemperature;
	// inline TGrateType() {
	//     FuelType = Undefined;
	//     GrateSurface = 0.0;
	//     FuelTransportSpeed = 0.0;
	//     IgnitionTemperature = 0.0;
	//     MaxTemperature = 0.0;
	// }
};

} // namespace mover
