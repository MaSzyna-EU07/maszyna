/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// when the AI driver works the brake releaser of its locomotive (TController::control_releaser): it vents the
// locomotive's own train brake cylinders, to start off sooner, to fill up an empty brake pipe, and to let the
// locomotive press the buffers while it (un)couples
namespace brake_releaser {

// the state of the locomotive and of its driver the decision depends on
struct conditions {
	double acceleration_desired{0.0}; // AccDesired of the driver [m/s2]; above no_acceleration it wants to speed up
	double pipe_pressure{0.0}; // brake pipe [bar]
	double cylinder_pressure{0.0}; // the locomotive's brake cylinders [bar]
	double control_pressure{0.0}; // the control reservoir of the locomotive's distributor [bar]
	double velocity{0.0}; // [km/h]
	bool uphill_start{false}; // starting uphill with the train brake held on purpose
	bool pressing{false}; // (un)coupling: the driver presses the buffers (movePress)
	bool compressing_buffers{false}; // uncoupling, stage 3: pushing towards the cut to slacken the coupler
	// standing, the train charges its brakes before its first start (bBrakeChargeStart), with the brake handle where
	// the pipe can be unlocked: a pipe below the lock is unlocked as the train stands, not only once it is to go
	bool charging_standing{false};
};

struct decision {
	bool actuate{false}; // the releaser goes on (else off)
	bool fill_pipe{false}; // the brake pipe is to be filled up: the brake handle goes to its unlock position
};

// AccDesired at or below this asks for no speeding up (EU07_AI_NOACCELERATION of Driver.h)
inline constexpr double no_acceleration{-0.05};

decision decide(conditions const &State);

} // namespace brake_releaser
