/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/uphillstart.h"

namespace uphill_start {

bool begins(situation const &Now) {

	return Now.standing
		// the whole train at a stand, not only its head stopped for a moment while the train rolls back
		&& (Now.consist_velocity > rollback_velocity) && Now.speed_up && Now.speed_asked
		// only with the train brake still applied to hold the train
		&& Now.train_brake_applied
		// and charged to what the pipe has, or the brake that holds the train has no air
		&& Now.consist_charged;
}

bool rolls_back(situation const &Now) {

	return Now.consist_velocity < rollback_velocity;
}

bool power_holds(situation const &Now, double const Holdtime) {

	return (Now.velocity > moving_velocity) || (Now.tractive_force > holding_force * Now.grade_force) || (Holdtime > hold_time);
}

bool released(situation const &Now, double const Releasetime) {

	return Now.consist_released || (Releasetime > release_time);
}

bool may_add_power(bool const Releasing, double const Tractiveforce, double const Holdingforce) {

	return (false == Releasing) || (Tractiveforce < holding_force * Holdingforce);
}

top_up_action brake_top_up(
	top_up &State, double const Brakeforce, double const Holdingforce, double const Brakeposition, double const Heldtime,
	double const Applicationtime) {

	if (State.done || (Brakeforce >= holding_force * Holdingforce) || (Brakeposition >= top_up_position)) {
		return top_up_action::none;
	}
	// each step is given the full application time of the train brake, for the cylinders along the train to fill
	if (Heldtime < Applicationtime) {
		return top_up_action::wait;
	}
	if ((State.last_force >= 0.0) && (Brakeforce - State.last_force < top_up_gain * Holdingforce)) {
		// the last step added next to nothing: the brake of this train can't hold it (brakes cut out, a load setting
		// too low), and the start goes on without
		State.done = true;
		return top_up_action::none;
	}
	State.last_force = Brakeforce;
	return top_up_action::step;
}

} // namespace uphill_start
