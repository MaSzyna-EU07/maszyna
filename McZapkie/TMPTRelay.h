#pragma once

#include "core.h"

namespace mover
{
struct TMPTRelay
{ /*lista przekaznikow bocznikowania*/
	double Iup = 0.0;
	double Idown = 0.0;
};

using TMPTRelayTable = TMPTRelay[MotorParametersArraySize + 1]; // indexed like MotorParam

} // namespace mover
