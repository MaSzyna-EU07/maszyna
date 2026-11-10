/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"

#include "rendering/opengl46/opengl46light.h"

void opengl46_light::apply_intensity(float const Factor) {

	factor = Factor;
}

void opengl46_light::apply_angle() const { /* nothing to do */ }


//---------------------------------------------------------------------------
