/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "McZapkie/MOVER.h"

// the position the controller position marker of the cab (mainctrlact:) shows
namespace main_ctrl_act {

// the diesel engines work the controller position straight away, they have no camshaft or relays of their own that
// would follow it, and nothing sets their MainCtrlActualPos; only with a controller of the EIM kind (EIMCtrlType)
// it is kept up to date (CheckEIMIC). so their marker shows the position of the controller (MaSzyna-EU07/maszyna#74)
inline bool follows_controller(TMoverParameters const &Vehicle) {
	return (Vehicle.EngineType == TEngineType::DieselEngine || Vehicle.EngineType == TEngineType::DieselElectric) && Vehicle.EIMCtrlType == 0;
}

// Scaletable: the cab's gauge has a scaletable:. only such a marker follows the controller; the cabs that define
// mainctrlact: without one (the ST43 so far) keep their marker still, as before, instead of moving it on a scale
// nobody has checked
inline float position(TMoverParameters const &Vehicle, bool const Scaletable) {
	auto const controller{Scaletable && follows_controller(Vehicle)};
	auto const main{controller ? Vehicle.MainCtrlPos : Vehicle.MainCtrlActualPos};
	auto const scnd{controller ? Vehicle.ScndCtrlPos : Vehicle.ScndCtrlActualPos};
	return static_cast<float>(Vehicle.CoupledCtrl ? main + scnd : main);
}

} // namespace main_ctrl_act
