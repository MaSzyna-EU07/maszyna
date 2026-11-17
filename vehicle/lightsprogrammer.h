/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>

#include "McZapkie/MOVER.h"

// the lights the light preset selector (programmer) of the occupied cab gives one vehicle of the consist
namespace lights_programmer {

// the lights for the vehicle's two ends, in the terms of TDynamicObject::RaLightsSet(): head is the end
// towards the head of the consist (TDynamicObject::Prev()), rear the end towards its tail (Next())
struct consist_lights {
	int head;
	int rear;
};

// Cablights: the selector's lights for the occupied cab's front and back (Lights[end::front / end::rear]).
// Cabfacinghead: the occupied cab faces the head of the consist.
// Direction: the vehicle's iDirection, 1 when its end::front is towards the head of the consist.
// Coupled: whether another vehicle is coupled at the vehicle's end::front / end::rear; an end with a coupled
// vehicle stays dark
inline consist_lights vehicle_lights(std::array<int, 2> const &Cablights, bool const Cabfacinghead, int const Direction, std::array<bool, 2> const &Coupled) {
	auto const headlights{Cablights[Cabfacinghead ? end::front : end::rear]};
	auto const rearlights{Cablights[Cabfacinghead ? end::rear : end::front]};
	// the head lights go to the vehicle's end towards the head of the consist whichever way the cab faces,
	// so the coupling which keeps them dark is the one at that same end (eu07/maszyna#395)
	auto const headend{Direction > 0 ? end::front : end::rear};
	auto const rearend{Direction > 0 ? end::rear : end::front};
	return {Coupled[headend] ? 0 : headlights, Coupled[rearend] ? 0 : rearlights};
}

} // namespace lights_programmer
