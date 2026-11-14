/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/consistbraketable.h"

#include <algorithm>

double consist_brake_table::full_braking(double const Kmh) const {
	auto const index{std::clamp(static_cast<int>(size * Kmh / std::max(1.0, vmax)), 1, size)};
	return std::max(0.01, a0[index] + 12.0 * a1[index]);
}

bool consist_brake_table::known() const {
	for (auto index{1}; index <= size; ++index) {
		if (a0[index] > 0.0 || a1[index] > 0.0) {
			return true;
		}
	}
	return false;
}
