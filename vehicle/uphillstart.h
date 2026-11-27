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
