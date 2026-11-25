/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/brakereleaser.h"

namespace brake_releaser {

namespace {

// the rules for a driver that wants to speed up
bool wanted_to_start(conditions const &State) {
	// fill up the brake pipe if needed
	if (State.pipe_pressure < 3.0) {
		return true;
	}
	// wyluzuj lokomotywę, to szybciej ruszymy
	// this rule is for starting off, so only below 1 km/h: at speed, just after an application, it would vent
	// the control reservoir of a locomotive that may have to brake again before the pipe has refilled it
	if (State.cylinder_pressure > 0.4
	    && (State.control_pressure > 4.9
	        // starting uphill the train brake stays applied on purpose, and the locomotive's own share of it has to go
	        // for the power to come on (asynchronous drives and the pressure switch cut the power while it is applied)
	        || State.uphill_start)
	    && State.velocity < 1.0) {
		return true;
	}
	// keep engine brakes released during coupling/uncoupling
	return State.pressing && State.cylinder_pressure > 0.1;
}

} // namespace

decision decide(conditions const &State) {
	decision result;
	if (State.acceleration_desired > no_acceleration) {
		result.fill_pipe = State.pipe_pressure < 3.0;
		result.actuate = wanted_to_start(State);
	}
	// uncoupling, stage 3: the locomotive has to push the slack of the coupler out whatever its speed table says.
	// the consist stays held by its own brakes, applied in stage 2, and the push asks for 2 km/h at most. a signal
	// at stop that the consist stands across after the direction change caps the speed at 0 and so AccDesired
	// below no_acceleration, and a locomotive left braked by the consist's brake application never closes the gap
	if (State.compressing_buffers && State.cylinder_pressure > 0.1) {
		result.actuate = true;
	}
	// don't overcharge train brake pipe
	if (State.pipe_pressure > 5.2) {
		result.actuate = false;
	}
	return result;
}

} // namespace brake_releaser
