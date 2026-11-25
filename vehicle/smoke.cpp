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

#include <algorithm>
#include <cmath>

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

// the beat of the exhaust: four puffs a turn of the driving wheels from two double-acting cylinders, merging into a
// steady blast as the wheels turn faster
float exhaust_beat(double const Crankangle, double const Revolutions) {
	auto const puff{0.5 + 0.5 * std::cos(4.0 * Crankangle)};
	auto const beat{0.15 + 0.85 * puff * puff * puff};
	auto const merging{std::clamp((std::abs(Revolutions) - 2.0) / 3.0, 0.0, 1.0)};
	return static_cast<float>(std::lerp(beat, 1.0, merging));
}

// the steam up the chimney against the nominal steam rate of the boiler: the exhaust of the cylinders and the blower
float blast(SteamEngine const &Engine) {
	auto const flow{Engine.State.exhaust_flow + Engine.State.blower * Engine.Parameters.blower_flow};
	return static_cast<float>(std::clamp(flow / std::max(Engine.Parameters.steam_rate, 0.1), 0.0, 1.5));
}

steam_output exhaust_emission(SteamEngine const &Engine, double const Revolutions) {
	auto const strength{blast(Engine)};
	auto const beat{exhaust_beat(Engine.CrankAngle, Revolutions)};
	return {strength * beat, std::clamp(0.3f + 0.7f * strength, 0.f, 1.f) * (0.4f + 0.6f * beat), 0.4f + 2.0f * strength * beat};
}

steam_output fire_emission(SteamEngine const &Engine) {
	if (Engine.State.fire_mass <= 0.0) {
		return {};
	}
	auto const fullburn{std::max(Engine.Parameters.firing_rate_max * Engine.Parameters.grate_area / 3600.0, 1e-3)};
	auto const burning{static_cast<float>(std::clamp(Engine.State.burn_rate / fullburn, 0.0, 1.0))};
	return {0.3f + 0.7f * burning, 0.15f + 0.85f * burning, 0.5f + 1.5f * blast(Engine)};
}

steam_output safety_valve_emission(SteamEngine const &Engine) {
	if (Engine.State.safety_valve_flow <= 0.0) {
		return {};
	}
	auto const share{static_cast<float>(std::clamp(Engine.State.safety_valve_flow / std::max(Engine.Parameters.safety_valve_flow, 0.1), 0.0, 1.0))};
	return {0.3f + 0.7f * share, std::min(1.f, 0.5f + share), 0.6f + 0.8f * share};
}

steam_output cock_emission(SteamEngine const &Engine, double const Revolutions) {
	if (false == Engine.Controls.cylinder_cocks) {
		return {};
	}
	auto const pressure{(Engine.State.chest_pressure - steam::atmosphere) / std::max(Engine.Parameters.working_pressure, 1.0)};
	auto const share{static_cast<float>(std::clamp(pressure, 0.0, 1.0))};
	if (share <= 0.f) {
		return {};
	}
	auto const beat{exhaust_beat(Engine.CrankAngle, Revolutions)};
	return {share * beat, share, 0.5f + share};
}

} // namespace

steam_part steam_part_of(std::string_view const Name) {
	using enum steam_part;
	if (Name == "exhaust") {
		return exhaust;
	}
	if (Name == "smoke") {
		return smoke;
	}
	if (Name == "safety_valves") {
		return safety_valves;
	}
	if (Name == "cocks") {
		return cocks;
	}
	return none;
}

steam_output steam_emission(TMoverParameters const &Mover, steam_part const Part) {
	if (Mover.EngineType != TEngineType::SteamEngine) {
		return {};
	}
	switch (Part) {
	case steam_part::exhaust:
		return exhaust_emission(Mover.Steam, Mover.nrot);
	case steam_part::smoke:
		return fire_emission(Mover.Steam);
	case steam_part::safety_valves:
		return safety_valve_emission(Mover.Steam);
	case steam_part::cocks:
		return cock_emission(Mover.Steam, Mover.nrot);
	default:
		return {};
	}
}

float vehicle_spawn_count(TMoverParameters const &Mover, bool const Ontracks, spawn_rate const &Rate, double const Timedelta) {
	if (false == Ontracks || Mover.dizel_FiringCountdown > 0.0) {
		return 0.f;
	}
	// the steady rate of the source, the way a source of the scenery spawns...
	auto const steady{std::min<float>(static_cast<float>(Rate.pending + (Rate.rate * Timedelta * Rate.fidelity)), Rate.limit)};
	// ...and on top of it the share of the engine
	return std::min<float>(static_cast<float>(steady + (Rate.rate * Timedelta * Rate.fidelity * engine_load(Mover))), Rate.limit);
}

} // namespace smoke
