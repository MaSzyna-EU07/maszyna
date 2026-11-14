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

control diesel_electric(int const Autorelaytype, bool const Autorelayflag) {
	if (Autorelayflag) {
		return control::automatic;
	}
	return Autorelaytype == 2 ? control::off : control::manual;
}

} // namespace fieldshunting
