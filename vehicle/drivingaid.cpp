/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/drivingaid.h"

#include "McZapkie/MOVER.h"
#include "McZapkie/hamulce.h"

#include <algorithm>
#include <cmath>

namespace drivingaid {

namespace {

// the first throttle number: the drive demand in percent with an EIM controller, else the master controller.
// a separate ED lever off its zero shows as the negative step the cab's joint controller is at
int throttle_first(input const &Input) {
	if (Input.splited && Input.dynamicbrakepos > 0.0) {
		return -static_cast<int>(std::lround(Input.dynamicbrakepos * Input.dynamicbrakeposno));
	}
	if (Input.eimctrltype > 0) {
		return std::max(0, static_cast<int>(100.4 * Input.eimic_real));
	}
	return Input.mainctrlpos;
}

// the cab has a control of the local brake: its own lever, or the braking range of the joint controller
// unless that range is the separate ED lever
bool has_local_brake_control(input const &Input) {
	return Input.cab.localbrake || (Input.cab.jointctrl && false == Input.splited);
}

} // namespace

readout compute(input const &Input) {
	readout result;
	result.throttle_first = throttle_first(Input);
	result.throttle_second = Input.eimctrltype > 0 ? Input.mainctrlpos : Input.scndctrlpos;

	auto const edbraking{std::max(0.0, -100.0 * Input.eimic_real)};
	result.brake_first = Input.integratedbrake ? edbraking : Input.brakectrlpos;
	result.local_brake = Input.localbrakepos * LocalBrakePosNo;
	// without a control in the cab the local brake is left out, unless it is applied after all (the AI or a key)
	result.show_local_brake = has_local_brake_control(Input) || Input.localbrakepos > 0.0;
	// the integrated universal controller shows the ED demand as its brake number already
	if (Input.eim && false == Input.integratedbrake) {
		if (auto const percent{static_cast<int>(std::lround(edbraking))}; percent > 0) {
			result.ed_percent = percent;
		}
	}
	return result;
}

input from_movers(TMoverParameters const &Vehicle, TMoverParameters const &Controlling, cab_controls const &Cab) {
	input result;
	result.eim = Vehicle.EngineType == TEngineType::ElectricInductionMotor || Vehicle.EIMCtrlType > 0;
	result.eimctrltype = Vehicle.EIMCtrlType;
	result.splited = Controlling.SplitEDPneumaticBrake;
	result.integratedbrake = Vehicle.UniCtrlIntegratedBrakeCtrl;
	result.eimic_real = Vehicle.eimic_real;
	result.dynamicbrakepos = Controlling.DynamicBrakeCtrlPos;
	result.dynamicbrakeposno = Controlling.DynamicBrakeCtrlPosNo;
	result.mainctrlpos = Controlling.MainCtrlPos;
	result.scndctrlpos = Controlling.ScndCtrlPos;
	result.brakectrlpos = Vehicle.fBrakeCtrlPos;
	result.localbrakepos = Vehicle.LocalBrakePosA;
	result.cab = Cab;
	return result;
}

} // namespace drivingaid
