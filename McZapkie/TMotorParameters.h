#pragma once

#include "core.h"

namespace mover
{
struct TMotorParameters
{
	double mfi{0.0};
	double mIsat{0.0};
	double mfi0{0.0}; // aproksymacja M(I) silnika} {dla dizla mIsat=przekladnia biegu
	double fi{0.0};
	double Isat{0.0};
	double fi0{0.0}; // aproksymacja E(n)=fi*n}    {dla dizla fi, mfi: predkosci przelozenia biegu <->
	bool AutoSwitch{false};
};

} // namespace mover
