/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/fieldshunting.h"

namespace fieldshunting {

bool has_switch(int const Autorelaytype, bool const Fieldshuntswitch) {
	return Autorelaytype == 2 && Fieldshuntswitch;
}

control diesel_electric(int const Autorelaytype, bool const Fieldshuntswitch, bool const Autorelayflag) {
	using enum control;
	if (Autorelayflag) {
		return automatic;
	}
	return has_switch(Autorelaytype, Fieldshuntswitch) ? off : manual;
}

} // namespace fieldshunting
