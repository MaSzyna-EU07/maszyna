/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/gaugewrap.h"

#include <cmath>

namespace gauge_wrap {

namespace {

// how far the positions in a full turn worked out from the scale may be off a whole number: the scales in the cab
// definitions are given to three decimals or so, 0.167 for a sixth gives 5.988
constexpr double turn_tolerance{0.05};

} // namespace

std::optional<int> full_turn(float const Scale, int const Positions) {
	if (Positions < 2 || Scale == 0.f) {
		return std::nullopt;
	}
	auto const perturn{1.0 / std::abs(static_cast<double>(Scale))};
	auto const turn{std::round(perturn)};
	if (std::abs(perturn - turn) > turn_tolerance) {
		// the positions don't make a full turn: a lever, or a knob that stops at its ends
		return std::nullopt;
	}
	auto const positions{static_cast<int>(turn)};
	// the way on from the last position to the first one, in positions
	auto const wayon{positions - (Positions - 1)};
	if (wayon < 1 || 2 * wayon > positions) {
		// the positions overlap in a turn, or the knob would turn on more than half a turn
		return std::nullopt;
	}
	return positions;
}

float turned_round(float const Value, int const Turn, bool const Forward) {
	return Forward ? Value - static_cast<float>(Turn) : Value + static_cast<float>(Turn);
}

} // namespace gauge_wrap
