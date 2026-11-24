#pragma once

#include "mover/core.h"

namespace mover
{
struct TTrackShape
{ /*ksztalt odcinka*/
	/// <summary>Curvature radius [m]; 0 means straight track.</summary>
	double R = 0.0; // promien
	/// <summary>Segment length [m].</summary>
	double Len = 0.0; // dlugosc
	/// <summary>Track gradient (rise/run).</summary>
	double dHtrack = 0.0; // nachylenie
	/// <summary>Track cant / superelevation.</summary>
	double dHrail = 0.0; // przechylka
};

} // namespace mover
