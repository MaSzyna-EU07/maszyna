#pragma once

#include "mover/core.h"

namespace mover
{
struct TTransmision
{ // liczba zebow przekladni
	int NToothM = 0;
	int NToothW = 0;
	double Ratio = 1.0;
	double Efficiency = 1.0;
};

} // namespace mover
