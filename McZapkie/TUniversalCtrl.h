#pragma once

#include "core.h"

namespace mover
{
struct TUniversalCtrl
{
	int mode = 0; /*tryb pracy zadajnika - pomocnicze*/
	double MinCtrlVal = 0.0; /*minimalna wartosc nastawy*/
	double MaxCtrlVal = 0.0; /*maksymalna wartosc nastawy*/
	double SetCtrlVal = 0.0; /*docelowa wartosc nastawy*/
	double SpeedUp = 0.0; /*szybkosc zwiekszania nastawy*/
	double SpeedDown = 0.0; /*szybkosc zmniejszania nastawy*/
	int ReturnPosition = 0; /*pozycja na ktora odskakuje zadajnik*/
	int NextPosFastInc = 0; /*nastepna duza pozycja przy przechodzeniu szybkim*/
	int PrevPosFastDec = 0; /*poprzednia duza pozycja przy przechodzeniu szybkim*/
};

using TUniversalCtrlTable = std::array<TUniversalCtrl, UniversalCtrlArraySize + 1>;

} // namespace mover
