#pragma once

#include "mover/core.h"

namespace mover
{
struct TDEScheme
{
	double RPM = 0.0; /*obroty diesla*/
	double GenPower = 0.0; /*moc maksymalna*/
	double Umax = 0.0; /*napiecie maksymalne*/
	double Imax = 0.0; /*prad maksymalny*/
};

using TDESchemeTable = TDEScheme[33]; /*tablica WWList dla silnikow spalinowych*/

} // namespace mover
