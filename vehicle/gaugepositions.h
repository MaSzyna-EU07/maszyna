/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <map>

// positions of a cab control which have sounds of their own
namespace gauge_positions {

// the entry of the last position of Positions the control reached on its way from the Previous to the Desired value,
// both in hundredths of a position, or Positions.end() if it reached none: a lever moved smoothly passes the
// positions rather than stops at them, and should sound each one it gets to
template <typename Value_>
typename std::map<int, Value_>::iterator last_reached(std::map<int, Value_> &Positions, int const Previous, int const Desired) {
	auto reached{Positions.end()};
	for (auto position{Positions.begin()}; position != Positions.end(); ++position) {
		auto const at{position->first * 100};
		if (Desired > Previous && at > Previous && at <= Desired) {
			reached = position; // moving up, the last one in the ascending order is the one reached last
		}
		else if (Desired < Previous && at < Previous && at >= Desired) {
			return position; // moving down, the first one is
		}
	}
	return reached;
}

} // namespace gauge_positions
