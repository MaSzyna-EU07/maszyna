#pragma once

#include <string>

#include "core.h"

namespace mover
{
struct TTrackParam
{ /*parametry odcinka - szerokosc, tarcie statyczne, kategoria,  obciazalnosc w t/os, uszkodzenia*/
	/// <summary>Track gauge / width [m].</summary>
	double Width = 0.0;
	/// <summary>Static friction coefficient on this segment.</summary>
	double friction = 0.0;
	/// <summary>Track category bit flags.</summary>
	int CategoryFlag = 0;
	/// <summary>Track-quality bit flags.</summary>
	int QualityFlag = 0;
	/// <summary>Track-damage bit flags (dtrack_*).</summary>
	int DamageFlag = 0;
	/// <summary>Maximum allowed velocity on this segment (used by AI driver).</summary>
	double Velmax; /*dla uzytku maszynisty w ai_driver*/
	std::string const *Name = nullptr;
};

} // namespace mover
