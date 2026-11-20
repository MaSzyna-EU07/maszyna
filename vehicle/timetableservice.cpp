/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/timetableservice.h"

#include "McZapkie/MOVER.h"

namespace timetable {

bool opens_doors(TMoverParameters const &Vehicle, service_flags const &Flags) {
	switch (Vehicle.Doors.open_control) {
		case control_t::conductor:
		case control_t::driver:
			return true;
		case control_t::mixed:
			// left to the passengers, unless the timetable has the driver open them all
			return Flags.central_doors == tristate::yes;
		default:
			return false;
	}
}

std::optional<bool> doorstep(service_flags const &Flags) {
	switch (Flags.doorstep) {
		case tristate::yes:
			return true;
		case tristate::no:
			return false;
		default:
			return std::nullopt;
	}
}

bool has_conductor(service_flags const &Flags) {
	return Flags.conductor != tristate::no;
}

bool makes_brake_test(service_flags const &Flags) {
	return Flags.brake_test != tristate::no;
}

} // namespace timetable
