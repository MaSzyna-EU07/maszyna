/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// starting uphill (TController::control_uphill_start()), the AI's driver lets the locomotive brake off only once
// the brake of the rest of the train holds the train on its own: a train stopped on a light application stands on
// the locomotive brake too, and rolls back as soon as it is off. until then the train brake is applied further, a
// step at a time
namespace uphill_start {

auto const holding_force{1.1}; // a force that holds a train uphill, as a multiple of the grade force
auto const top_up_position{4.0}; // full service: the furthest the train brake is applied to hold the train
// a step that adds less brake force than this, as a multiple of the grade force, is the last one
auto const top_up_gain{0.02};

auto const hold_time{45.0}; // [s] a locomotive that can't hold the train on its own power gets the brake off after it all the same
auto const release_time{60.0}; // [s] the longest the power stays on against the releasing brake (a hand brake doesn't come off)
auto const rollback_velocity{-0.5}; // [km/h] the train as a whole rolling back faster than this gives a start up
auto const moving_velocity{1.0}; // [km/h] the head of the train going forward faster than this is under way

// what a start goes by, at a moment
struct situation {
	bool standing{false}; // the head of the train at a stand
	double velocity{0.0}; // of the head of the train, in the direction of travel [km/h]
	// of the whole train, mass-weighted, in the direction of travel [km/h]: with the locomotive brake off, the
	// locomotive and the first vehicles run back into the slack of a long train still held by its brake
	double consist_velocity{0.0};
	bool speed_up{false}; // the driver is asked to speed up
	bool speed_asked{false}; // there is a speed to go at
	bool train_brake_applied{false}; // the train brake handle is past running
	double tractive_force{0.0}; // of the train, in the direction of travel [N]
	double holding_force{0.0}; // the grade force on the train [N]
	bool consist_released{false}; // the brakes of the train are off, the hold of the start not counted
	// the brakes of the train charged to what the pipe has: else the brake that holds the train has no air
	bool consist_charged{true};
};

// a start begins: the whole train at a stand on the train brake, charged, and asked to go
bool begins(situation const &Now);
// the train rolls back all the same: the rules for a train slipping back take over
bool rolls_back(situation const &Now);
// the power holds the train, so the train brake can be released. Holdtime: since the start began, or since the last
// step of the top-up [s]
bool power_holds(situation const &Now, double Holdtime);
// the release phase is over: the brakes are off, or they don't come off. Releasetime: since it began [s]
bool released(situation const &Now, double Releasetime);
// power may be added. against the releasing brake, the power that holds the train is enough: the brake coming off
// speeds the train up, more power only drags the brake and slips the wheels. nor is a notch past that taken back:
// the power got there in one notch from below, and the notch below doesn't hold the train
bool may_add_power(bool Releasing, double Tractiveforce, double Holdingforce);

// what has been done to apply the train brake further, in one start
struct top_up {
	double last_force{-1.0}; // the brake force when the last step was given, -1 before the first [N]
	bool done{false}; // a step added next to nothing: no more of them in this start
};

enum class top_up_action {
	none, // the brake holds the train, or can't be made to: the start goes on
	wait, // the cylinders are still filling after the last step
	step // apply the train brake one step further
};

// Brakeforce: the brake force of the unpowered vehicles [N]; Holdingforce: the grade force on the train [N];
// Brakeposition: the train brake handle, as BrakeCtrlPosition; Heldtime: how long the handle has been where it
// is [s]; Applicationtime: the time the train brake takes to apply, for the cylinders along the train to fill [s]
top_up_action brake_top_up(
	top_up &State, double Brakeforce, double Holdingforce, double Brakeposition, double Heldtime, double Applicationtime);

} // namespace uphill_start
