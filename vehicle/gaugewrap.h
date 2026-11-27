/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>

// a multi-position rotary switch of the cab turned past its last position to the first one, or back past the first
// one to the last (a selector with Wrap=Yes, such as the light preset selector)
namespace gauge_wrap {

// the number of positions in one full turn of a rotary switch of Positions positions that turns by Scale of a full turn
// per position, if the switch can turn on past its end the short way round: the full turn is a whole number of positions
// (within the rounding of the scale in the cab definition, e.g. 0.167 for a sixth), all the positions fit in it, and the
// way on from the last position to the first one is at most half a turn. None otherwise
std::optional<int> full_turn(float Scale, int Positions);

// the value of the switch at Value seen one full turn of Turn positions earlier (Forward: it turns on past its last
// position to the first one) or later (back past the first position to the last one). The knob stands the same there,
// and turning from that value to the new position turns it on the way it was turned rather than all the way back
float turned_round(float Value, int Turn, bool Forward);

} // namespace gauge_wrap
