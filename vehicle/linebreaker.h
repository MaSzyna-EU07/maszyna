/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the line breaker (main switch, WS) controls of the cab
namespace linebreaker {

enum class toggle : int {
	none, // the press does nothing
	close, // the control goes to its on position, and the line breaker closes if it can
	open // the control goes to its off position, and the line breaker opens
};

// what a press of the line breaker toggle does. Breakerstate is the state the cab keeps of the line
// breaker (0: open, 1: closed, 2: freshly closed). Latchinglever is a single two-state switch which
// stays where it was put, Leveron that it stands in its on position. such a lever doesn't follow the
// line breaker: knocked out (by the converter overload relay, say) the breaker opens with the lever
// left on, and the lever has to go off before it can be put on again (eu07/maszyna#394)
toggle pressed(int Breakerstate, bool Latchinglever, bool Leveron);

} // namespace linebreaker
