#pragma once

#include "mover/core.h"

namespace mover
{
struct TTractionParam
{
	/// <summary>Catenary voltage [V].</summary>
	double TractionVoltage = 0.0; /*napiecie*/
	/// <summary>Catenary frequency [Hz] (0 for DC).</summary>
	double TractionFreq = 0.0; /*czestotliwosc*/
	/// <summary>Maximum current capacity [A].</summary>
	double TractionMaxCurrent = 0.0; /*obciazalnosc*/
	/// <summary>Pantograph-catenary contact resistance [Ω].</summary>
	double TractionResistivity = 0.0; /*rezystancja styku*/
};

} // namespace mover
