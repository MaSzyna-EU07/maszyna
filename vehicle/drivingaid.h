/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>

class TMoverParameters;

// what the driving aid panel shows of the controls and of the braking: the numbers of its throttle and
// brakes lines, worked out from the vehicle state (eu07/maszyna#558, eu07/maszyna#555)
namespace drivingaid {

// the cab controls the panel needs to know of, which the cab definition (.mmd) has or lacks
struct cab_controls {
	// a lever of the local (auxiliary) brake: localbrake:
	bool localbrake{true};
	// a joint controller, jointctrl:, whose braking range is the local brake, or the ED lever with
	// SplitEDPneumaticBrake
	bool jointctrl{false};
};

struct input {
	// the vehicle works out the integrated drive and brake demand (eimic): an induction motor vehicle,
	// or a diesel one with an EIM controller
	bool eim{false};
	int eimctrltype{0};
	// the ED brake has a lever of its own (SplitEDPneumaticBrake), with dynamicbrakeposno steps
	bool splited{false};
	// the universal controller brakes with the ED brake, so the brake handle number is the ED demand already
	bool integratedbrake{false};
	// the demand in use, -1..1, negative braking
	double eimic_real{0.0};
	double dynamicbrakepos{0.0}; // 0..1
	int dynamicbrakeposno{10};
	int mainctrlpos{0};
	int scndctrlpos{0};
	double brakectrlpos{0.0}; // the train brake handle
	double localbrakepos{0.0}; // 0..1
	cab_controls cab;
};

struct readout {
	// "Throttle: first+second"
	int throttle_first{0};
	int throttle_second{0};
	// "Brakes: first+local"
	double brake_first{0.0};
	double local_brake{0.0};
	bool show_local_brake{true};
	// the ED braking demand in percent, while there is some
	std::optional<int> ed_percent;
};

// the panel's numbers from the vehicle state
readout compute(input const &Input);

// the state the panel reads: Vehicle is the one the driver sits in, Controlling the one whose controllers
// the driver moves (TController::Controlling())
input from_movers(TMoverParameters const &Vehicle, TMoverParameters const &Controlling, cab_controls const &Cab);

} // namespace drivingaid
