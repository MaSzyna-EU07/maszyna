#pragma once

#include "core.h"

namespace mover
{
struct TScheme
{
	int Relay = 0; /*numer pozycji rozruchu samoczynnego*/
	double R = 0.0; /*opornik rozruchowy*/ /*dla dizla napelnienie*/
	int Bn = 0;
	int Mn = 0; /*ilosc galezi i silnikow w galezi*/ /*dla dizla Mn: czy luz czy nie*/
	bool AutoSwitch = false; /*czy dana pozycja nastawniana jest recznie czy autom.*/
	int ScndAct = 0; /*jesli ma bocznik w nastawniku, to ktory bocznik na ktorej pozycji*/
};

using TSchemeTable = TScheme[ResArraySize + 1]; /*tablica rezystorow rozr.*/

} // namespace mover
