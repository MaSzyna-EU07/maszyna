/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/horn.h"

namespace horn {

int pressed(int const Signal, int const Tone, bool const Singlelever) {
	auto const other{Tone == low ? high : low};
	return (Signal | Tone) & (Singlelever ? ~other : ~0);
}

int released(int const Signal, int const Tone) {
	return Signal & ~Tone;
}

} // namespace horn
