/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the helpers the files of physics/mover/ share: only they include this header

#pragma once

#include "physics/mover.h"

const double CouplerTune = 0.1; // skalowanie tlumiennosci


inline double square(double val) // SQR() zle liczylo w current() ...
{
	return val * val;
}

int DirPatch(int Coupler1, int Coupler2);

int DirF(int CouplerN);

double TableInterpolation(std::map<double, double> &Map, double Parameter);

double compression_at(TCoupling const &Coupler, int const End);

bool issection(std::string_view Name, std::string_view Input);

