/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the mirrors of a vehicle, unfolded on the side of the open doors and folded at speed
namespace mirror_moves {

enum class event {
	none,
	unfolding,
	folding
};

// the move of a mirror from one update to the next. a mirror sounds as it sets off, unfolding or folding, not on
// every update it keeps moving the same way
struct motion {
	int direction{0}; // of the last update: +1 unfolding, -1 folding, 0 standing

	// the event the Change of the mirror position (0: folded, 1: unfolded) since the last update starts, if any
	event update(double const Change) {
		auto now{0};
		if (Change > 0.0) {
			now = 1;
		}
		else if (Change < 0.0) {
			now = -1;
		}
		auto const setsoff{now != 0 && now != direction};
		direction = now;
		if (false == setsoff) {
			return event::none;
		}
		return now > 0 ? event::unfolding : event::folding;
	}
};

} // namespace mirror_moves
