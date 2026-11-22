/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/brakecharge.h"

namespace brake_charge {

bool charged(state &State, double const Level, double const Reference, double const Time, bool const Supplied) {

	if (State.going) {
		return true;
	}
	if (Level >= Reference - margin) {
		State = {};
		State.going = true;
		return true;
	}
	if ((State.best < 0.0) || (Level > State.best + stall_gain) || (false == Supplied)) {
		// still charging, or nothing to charge with yet: the time for a stall counts from when there is
		State.best = Level;
		State.best_time = Time;
		State.stalled = false;
		return false;
	}
	State.stalled = (Time - State.best_time >= stall_time);
	State.going = State.stalled;
	return State.stalled;
}

double locomotive_brake_force(double const Blockforce, double const Weight) {

	return std::max(0.0, std::min(Blockforce, Weight * adhesion));
}

bool locomotive_brake_holds(double const Locomotivebrakeforce, double const Gradeforce) {

	return Locomotivebrakeforce >= holding_force * std::abs(Gradeforce);
}

double reference(double const Headpipe, double const Working, bool const Trainbrakeapplied) {

	if (false == Trainbrakeapplied) {
		return Working;
	}
	return std::clamp(Headpipe, Working - service_drop + margin, Working);
}

bool holds_application(bool const Hold, bool const Charged, bool const Standing, double const Position) {

	return Hold && (false == Charged) && Standing && (Position > 0.0) && (Position <= 1.0);
}

bool release_waits(bool const Charged, bool const Standing, bool const Trainbrakeapplied) {

	return (false == Charged) && Standing && (false == Trainbrakeapplied);
}

} // namespace brake_charge
