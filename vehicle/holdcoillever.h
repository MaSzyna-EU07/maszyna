/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the lever of the line breaker's hold coil (eu07/maszyna#364), as on the ET22 and the EP09: the cab's main_off_bt
// control declared with "type: toggle" stays in the position it's put in. Position 0 keeps the hold coil fed, so the
// line breaker can be closed; position 1 cuts the coil off, which opens the line breaker and keeps it from closing
// until the lever is put back. Without the declared type main_off_bt stays the push button it always was, which
// opens the line breaker and springs back.
namespace hold_coil_lever {

// whether the main_off_bt control is the bistable lever, from the type its definition declares, if it declares one
inline bool is_bistable(bool const Typedeclared, bool const Toggletype) {
	return Typedeclared && Toggletype;
}

// whether the lever in the given position cuts off the hold coil
inline bool cuts_off(float const Position) {
	return Position > 0.5f;
}

// the position the lever goes to when it's operated: the other one
inline float operated(float const Position) {
	return cuts_off(Position) ? 0.f : 1.f;
}

} // namespace hold_coil_lever
