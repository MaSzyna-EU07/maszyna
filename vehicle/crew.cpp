/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/crew.h"

#include "McZapkie/TMoverParameters.h"
#include "vehicle/handover.h"

namespace crew {

namespace {

// the scenery parser folds the ASCII letters of the names only, and so does this
char lower_ascii(char const Character) {
	return Character >= 'A' && Character <= 'Z' ? static_cast<char>(Character - 'A' + 'a') : Character;
}

// the controllers of the vehicle to the positions giving no power, and the vehicles it controls told so
void cut_power(mover::TMoverParameters &Vehicle) {
	Vehicle.MainCtrlPos = Vehicle.MainCtrlNoPowerPos();
	Vehicle.ScndCtrlPos = 0;
	// what the integrated handle and the cruise control asked for last stays in force until somebody drives
	Vehicle.eimic = 0.0;
	Vehicle.eimic_real = 0.0;
	Vehicle.SpeedCtrlValue = 0.0;
	Vehicle.SpeedCtrlUnit.IsActive = false;
	Vehicle.SendCtrlToNext("MainCtrl", Vehicle.MainCtrlPosForCoupled(), Vehicle.CabActive);
	Vehicle.SendCtrlToNext("ScndCtrl", Vehicle.ScndCtrlPos, Vehicle.CabActive);
	Vehicle.SendCtrlToNext("EIMIC", 0.0, Vehicle.CabActive);
}

} // namespace

bool is_command(std::string_view const Text) {
	return Text.starts_with(command_prefix);
}

request parse(std::string_view const Text, double const Cab, double const Flags) {
	request result;
	if (false == is_command(Text)) {
		return result;
	}
	result.vehicle = Text.substr(command_prefix.size());
	std::ranges::transform(result.vehicle, result.vehicle.begin(), lower_ascii);
	result.cab_valid = Cab == 1.0 || Cab == -1.0 || Cab == 0.0;
	if (result.cab_valid) {
		result.cab = static_cast<int>(Cab);
	}
	result.flags = static_cast<int>(std::clamp(Flags, -65535.0, 65535.0));
	return result;
}

action decide(request const &Request, vehicle_state const &State) {
	using enum action;
	if (false == Request.cab_valid) {
		return refuse_cab;
	}
	if (State.occupied || (State.has_driver && false == State.driver_is_ai)) {
		return refuse_user;
	}
	if (Request.cab == 0) {
		return State.has_driver ? remove : nothing;
	}
	if (State.has_driver) {
		return refuse_crewed;
	}
	if (false == State.has_cab) {
		return refuse_no_cab;
	}
	if (State.consist_driven) {
		return refuse_consist_driven;
	}
	return Request.cab > 0 ? add_front : add_rear;
}

bool changes_crew(action const Action) {
	return Action == action::add_front || Action == action::add_rear || Action == action::remove;
}

std::string_view describe(action const Action) {
	using enum action;
	switch (Action) {
	case add_front:
		return "an AI driver takes cab A";
	case add_rear:
		return "an AI driver takes cab B";
	case remove:
		return "the AI driver leaves";
	case nothing:
		return "no driver to take off";
	case refuse_cab:
		return "refused, the cab has to be 1, -1 or 0";
	case refuse_user:
		return "refused, a user is in the vehicle";
	case refuse_crewed:
		return "refused, the vehicle has a driver already";
	case refuse_no_cab:
		return "refused, the vehicle has no cab";
	case refuse_consist_driven:
		return "refused, the consist has a driver in charge in another vehicle";
	default:
		return "";
	}
}

void secure(mover::TMoverParameters &Occupied, mover::TMoverParameters &Controlling) {
	handover::release_ai_controls(Occupied, Controlling);
	cut_power(Controlling);
	if (&Occupied != &Controlling) {
		cut_power(Occupied);
	}
	// the emergency position vents the brake pipe whatever the brake mode, and with nobody in the cab too
	Occupied.BrakeLevelSet(Occupied.Handle->GetPos(bh_EB));
	// a vehicle with no train brake (a car, a draisine) gets the brake its driver stops it with, in full
	if (Occupied.BrakeSystem == mover::TBrakeSystem::Individual) {
		if (Occupied.LocalBrake == mover::TLocalBrake::ManualBrake) {
			Occupied.IncManualBrakeLevel(mover::ManualBrakePosNo);
		}
		else {
			Occupied.IncLocalBrakeLevel(LocalBrakePosNo);
		}
	}
	Occupied.CabDeactivisation();
	Occupied.CabOccupied = 0;
}

} // namespace crew
