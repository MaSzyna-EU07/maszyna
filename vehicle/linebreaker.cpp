/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/linebreaker.h"

namespace linebreaker {

toggle pressed(state const Breakerstate, bool const Latchinglever, bool const Leveron) {
	if (Breakerstate == state::closed) {
		return toggle::open;
	}
	if (Breakerstate != state::open) {
		return toggle::none;
	}
	// the line breaker is open; a lever left on goes off first
	return Latchinglever && Leveron ? toggle::open : toggle::close;
}

} // namespace linebreaker
