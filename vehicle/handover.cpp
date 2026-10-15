/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/handover.h"

#include "McZapkie/MOVER.h"

namespace handover {

void release_ai_controls(TMoverParameters &Occupied, TMoverParameters &Controlling) {
	// the AI's hornoff action touches the horn only while the AI drives
	Occupied.WarningSignal = 0;
	// the AI drives the sandbox directly, bypassing the manual and automatic sanding states
	if (Controlling.SandDose && false == (Controlling.SandDoseManual || Controlling.SandDoseAuto)) {
		Controlling.Sandbox(false);
	}
	if (Occupied.DepartureSignal) {
		Occupied.signal_departure(false);
	}
	if (Occupied.Hamulec->Releaser()) {
		Occupied.BrakeReleaser(0);
	}
}

int light_selector_position(TMoverParameters const &Occupied, int const Lightstable, int const Headlights, int const Inventory) {
	if (Occupied.LightsPosNo <= 0) {
		return Occupied.LightsPos;
	}
	auto const lightsmatch{[&](int const Position) { return (Occupied.Lights[Lightstable][Position - 1] & Inventory) == Headlights; }};
	if (Occupied.LightsPos >= 1 && Occupied.LightsPos <= Occupied.LightsPosNo && lightsmatch(Occupied.LightsPos)) {
		return Occupied.LightsPos;
	}
	for (auto position = 1; position <= Occupied.LightsPosNo; ++position) {
		if (lightsmatch(position)) {
			return position;
		}
	}
	return Occupied.LightsPos;
}

int dimmer_position(TMoverParameters const &Occupied, bool const Headlightson) {
	auto const &positions{Occupied.dimPositions};
	auto const current{Occupied.modernDimmerPosition};
	auto const count{static_cast<int>(positions.size())};
	auto const isoff{[&](int const Position) { return Position < 0 || Position >= count || positions[Position].isOff; }};

	if (false == Occupied.enableModernDimmer || false == Headlightson || false == isoff(current)) {
		return current;
	}
	if (false == isoff(Occupied.modernDimmerDefaultPosition)) {
		return Occupied.modernDimmerDefaultPosition;
	}
	for (auto position = 0; position < count; ++position) {
		if (false == (positions[position].isOff || positions[position].isDimmed || positions[position].isHighBeam)) {
			return position;
		}
	}
	for (auto position = 0; position < count; ++position) {
		if (false == positions[position].isOff) {
			return position;
		}
	}
	return current;
}

} // namespace handover
