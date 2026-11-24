#pragma once

#include "mover/core.h"

namespace mover
{
struct TFFScheme
{
	double v = 0.0; // parametr wejsciowy
	double freq = 0.0; // wyjscie: czestotliwosc falownika
};

using TFFSchemeTable = TFFScheme[33];

} // namespace mover
