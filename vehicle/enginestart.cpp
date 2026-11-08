/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/enginestart.h"

#include "McZapkie/MOVER.h"
#include "utilities/utilities.h"

namespace engine_start {

namespace {

// whether the vehicle has an engine the cab can start, the same test the tachometers of the vehicles in multiple make
bool has_engine(TMoverParameters const &Vehicle) {
	return Vehicle.Power > 0.01;
}

// walks from Start through its coupler Side along the control cable, and returns the Number-th vehicle with an engine
// other than Skipped it meets; Number is lowered by the ones it met when it doesn't get that far
TMoverParameters *find_along(TMoverParameters const &Start, int const Side, TMoverParameters const &Skipped, int &Number) {
	auto const *vehicle{&Start};
	auto side{Side};
	// a train can't loop back on itself, the limit only guards against a broken chain of couplers
	for (auto steps = 0; steps < 1000; ++steps) {
		auto const &coupler{vehicle->Couplers[side]};
		if (coupler.Connected == nullptr || false == TestFlag(coupler.CouplingFlag, coupling::control)) {
			return nullptr;
		}
		// onwards through the other end of the next vehicle, whichever way round it stands
		side = 1 - coupler.ConnectedNr;
		vehicle = coupler.Connected;
		if (vehicle == &Skipped || false == has_engine(*vehicle)) {
			continue;
		}
		--Number;
		if (Number == 0) {
			return coupler.Connected;
		}
	}
	return nullptr;
}

} // namespace

TMoverParameters *engine(TMoverParameters const &Occupied, TMoverParameters &Controlled, int Number) {
	if (Number < 1) {
		return nullptr;
	}
	if (Number == 1) {
		return &Controlled;
	}
	// the cab's own vehicle is the first one
	auto others{Number - 1};
	// behind the cab is the rear end of a vehicle driven from its front cab, and the other way round
	int const behind{Occupied.CabOccupied < 0 ? end::front : end::rear};
	if (auto *found{find_along(Occupied, behind, Controlled, others)}; found != nullptr) {
		return found;
	}
	return find_along(Occupied, 1 - behind, Controlled, others);
}

void remote_switch::update(TMoverParameters *Engine, bool const Closeheld, bool const Openheld, double const Deltatime, bool const Closesonrelease) {
	if (Engine != m_engine) {
		// another vehicle under the switch, after a change in the train
		m_engine = Engine;
		m_heldtime = 0.0;
		m_ready = false;
	}
	if (Engine == nullptr) {
		return;
	}
	if (Openheld || is_running(*Engine)) {
		// the switch opening the circuit is held down, or there's nothing left to start
		m_heldtime = 0.0;
		m_ready = false;
		return;
	}
	if (Closeheld && Engine->MainSwitchCheck()) {
		m_heldtime += Deltatime;
	}
	if (m_heldtime > Engine->InitialCtrlDelay) {
		m_ready = true;
	}
	if (m_ready && false == (Closesonrelease && Closeheld)) {
		Engine->MainSwitch(true, range_t::local);
		m_ready = false;
		m_heldtime = 0.0;
	}
	if (false == Closeheld) {
		m_heldtime = 0.0;
	}
}

bool remote_switch::open(TMoverParameters &Engine) {
	return Engine.MainSwitch(false, range_t::local);
}

bool remote_switch::is_running(TMoverParameters const &Engine) {
	return Engine.Mains || Engine.dizel_startup;
}

} // namespace engine_start
