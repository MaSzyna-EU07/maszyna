#pragma once

#include "core.h"

namespace mover
{
struct TWiperScheme
{
	uint8_t byteSum = 0; // suma bitowa pracujacych wycieraczek
	double WiperSpeed = 0.0; // predkosc wycieraczki
	double interval = 0.0; // interwal pracy wycieraczki
	double outBackDelay = 0.0; // czas po jakim wycieraczka zacznie wracac z konca do poczatku
};

using TWiperSchemeTable = TWiperScheme[16];

} // namespace mover
