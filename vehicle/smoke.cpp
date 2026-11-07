/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/smoke.h"

#include "McZapkie/MOVER.h"

namespace smoke {

namespace {

// how hard the engine works, scaling the spawn rate of the source
double engine_load(TMoverParameters const &Mover) {
	if (Mover.dizel_spinup) {
		// starting up, it puffs harder the faster it turns
		return ((Mover.enrot / 4) * 0.01);
	}
	auto const rpmrise{(Mover.DElist[Mover.MainCtrlPosNo].RPM - Mover.enrot) / 60};
	if (Mover.DirAbsolute == 0 || Mover.Im == 0) {
		// idling: speeding up to the revolutions of the master controller
		return ((rpmrise * 0.02) * (Mover.EnginePower * 0.005));
	}
	// under load: the more current the traction motors draw, the more smoke
	return ((rpmrise * (std::sqrt(Mover.Im) * 0.01) * 0.02) * (Mover.EnginePower * 0.005));
}

} // namespace

float vehicle_spawn_count(TMoverParameters const &Mover, bool const Ontracks, spawn_rate const &Rate, double const Timedelta) {
	if (false == Ontracks) {
		return 0.f;
	}
	// the steady rate of the source, the way a source of the scenery spawns...
	auto const steady{std::min<float>(static_cast<float>(Rate.pending + (Rate.rate * Timedelta * Rate.fidelity)), Rate.limit)};
	// ...and on top of it the share of the engine
	return std::min<float>(static_cast<float>(steady + (Rate.rate * Timedelta * Rate.fidelity * engine_load(Mover))), Rate.limit);
}

} // namespace smoke
