/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "McZapkie/weatherphysics.h"

#include <algorithm>
#include <cmath>

namespace weather_physics
{

namespace
{

double smoothstep(double const Edge0, double const Edge1, double const Value)
{
	auto const t{std::clamp((Value - Edge0) / (Edge1 - Edge0), 0.0, 1.0)};
	return t * t * (3.0 - 2.0 * t);
}

double mix(double const A, double const B, double const T)
{
	return A + (B - A) * T;
}

// a share moved towards Target over a time constant of Time seconds in a step of Deltatime
double approach(double const Value, double const Target, double const Deltatime, double const Time)
{
	return Value + (Target - Value) * (1.0 - std::exp(-Deltatime / std::max(Time, 1e-3)));
}

// the heat the brakes of a vehicle take to dry, and to thaw, J a kg of the vehicle
constexpr double DRYING_HEAT{40.0};
constexpr double THAWING_HEAT{120.0};

} // namespace

double adhesion_factor(settings const &Settings, railhead const &Rail, double const Snow, double const Since)
{
	// the wheels take the leaves, the ice and the dirt off; the weather lays them back
	auto const cleaned{Since >= 0.0 ? std::exp(-Since / std::max(static_cast<double>(Settings.pass_recovery), 1.0)) : 0.0};
	auto const leaves{std::clamp(static_cast<double>(Rail.leaves), 0.0, 1.0) * (1.0 - 0.8 * cleaned)};
	auto const ice{std::clamp(static_cast<double>(Rail.ice), 0.0, 1.0) * (1.0 - 0.7 * cleaned)};
	auto const dirt{std::clamp(static_cast<double>(Rail.dirt), 0.0, 1.0) * (1.0 - 0.5 * cleaned)};
	auto const water{std::clamp(static_cast<double>(Rail.water), 0.0, 1.0)};
	// a little water is enough to make the dirt a paste, more washes it off (railsurface.cpp)
	auto const watered{smoothstep(0.02, 0.15, water)};

	auto factor{mix(1.0, Settings.wet, smoothstep(0.0, 0.3, water))};
	factor *= mix(1.0, Settings.emulsion, dirt * watered);
	factor *= mix(1.0, mix(Settings.leaves_dry, Settings.leaves_wet, watered), leaves);
	factor *= mix(1.0, Settings.ice, ice);
	factor *= mix(1.0, Settings.snow, smoothstep(0.0, 0.02, Snow));
	if (factor >= 1.0)
		return 1.0;
	return std::max(factor, static_cast<double>(Settings.minimum));
}

double sanded(settings const &Settings, double const Friction, double const Dry, double const Sandfactor)
{
	if (Friction <= 0.0)
		return Sandfactor;
	return std::max(Sandfactor, Dry * Settings.sand / Friction);
}

void update_brakes(brake_state &State, brake_weather const &Weather, double const Speed, double const Power, double const Mass, double const Deltatime)
{
	if (Deltatime <= 0.0)
		return;
	auto const speed{std::abs(Speed)};
	auto const rain{std::clamp(Weather.rain, 0.0, 1.0)};
	auto const snow{std::clamp(Weather.snow, 0.0, 1.0)};
	auto const temperature{Weather.temperature};

	// the rain and the spray of the wheels wet the blocks in a few minutes, the air dries them in half an hour, slower in the cold
	if (rain > 0.0)
		State.water = approach(State.water, 1.0, Deltatime, 300.0 / (rain * (0.3 + std::min(speed / 20.0, 1.0))));
	else
		State.water = approach(State.water, 0.0, Deltatime, 1800.0 * (1.0 + std::max(0.0, -temperature) / 10.0));

	// in a frost the water freezes on them, and the snow the running gear goes through packs into them
	if (temperature < 0.0)
	{
		auto const frozen{State.water * (1.0 - std::exp(-Deltatime / (900.0 / (1.0 + -temperature / 3.0))))};
		State.water -= frozen;
		State.ice += frozen;
	}
	if ((temperature < 0.5) && (snow > 0.0) && (speed > 3.0))
		State.ice = approach(State.ice, 1.0, Deltatime, 1200.0 / (snow * std::min(speed / 25.0, 1.5)));
	else if (temperature > 1.0)
		State.ice = approach(State.ice, 0.0, Deltatime, 1200.0 / temperature);

	// the heat of the braking dries them first, then thaws them
	if ((Power > 0.0) && (Mass > 0.0))
	{
		auto heat{Power * Deltatime / Mass};
		auto const drying{std::min(State.water, heat / DRYING_HEAT)};
		State.water -= drying;
		heat -= drying * DRYING_HEAT;
		State.ice -= std::min(State.ice, heat / THAWING_HEAT);
	}
	State.water = std::clamp(State.water, 0.0, 1.0);
	State.ice = std::clamp(State.ice, 0.0, 1.0);
}

double brake_factor(settings const &Settings, brake_material const Material, brake_state const &State)
{
	double wet;
	double iced;
	switch (Material)
	{
	case brake_material::composite:
		wet = Settings.brake_wet_composite;
		iced = Settings.brake_ice_composite;
		break;
	case brake_material::disc:
		wet = Settings.brake_wet_disc;
		iced = Settings.brake_ice_disc;
		break;
	default:
		wet = Settings.brake_wet_cast_iron;
		iced = Settings.brake_ice_cast_iron;
		break;
	}
	return mix(1.0, wet, std::clamp(State.water, 0.0, 1.0)) * mix(1.0, iced, std::clamp(State.ice, 0.0, 1.0));
}

double air_density(double const Temperature)
{
	// at the same pressure the density goes with the inverse of the absolute temperature
	return std::clamp(288.15 / (273.15 + Temperature), 0.8, 1.25);
}

double aerodynamic_force(double const Drag, double const V, double const Along, double const Across, double const Density, double const Yaw)
{
	// the air against the vehicle
	auto const along{V - Along};
	auto const speed2{along * along + Across * Across};
	if (speed2 < 1e-12)
		return 0.0;
	auto const yawed{1.0 + Yaw * Across * Across / speed2};
	return -Drag * Density * std::sqrt(speed2) * along * yawed;
}

double wire_sway(settings const &Settings, double const Length, double const Across, double const Wireice, double const Density)
{
	// the drag of the wind on a metre of the wire (a cylinder, a drag coefficient of 1.2, air of 1.225 kg/m3 at 15 degrees), and the
	// sideways deflection of the middle of a taut string under it: q L^2 / 8 T
	constexpr double drag_coefficient{1.2};
	constexpr double air{1.225};
	auto const diameter{std::max(static_cast<double>(Settings.wire_diameter), 0.0) + 2.0 * std::max(Wireice, 0.0)};
	auto const load{0.5 * air * Density * drag_coefficient * diameter * Across * std::abs(Across)};
	auto const deflection{load * Length * Length / (8.0 * std::max(static_cast<double>(Settings.wire_tension), 1000.0))};
	return std::clamp(deflection, -1.5, 1.5);
}

double contact_ice(settings const &Settings, double const Glaze, double const Rimehard, double const Rimesoft, double const Wetsnow)
{
	return std::max(0.0, Glaze) * Settings.break_glaze + std::max(0.0, Rimehard) * Settings.break_rime_hard + std::max(0.0, Rimesoft) * Settings.break_rime_soft +
	       std::max(0.0, Wetsnow) * Settings.break_wet_snow;
}

double arc_strength(double const Current)
{
	return smoothstep(20.0, 600.0, std::abs(Current));
}

double break_rate(settings const &Settings, double const Wireice, double const Plaster, double const Speed, bool const Leading)
{
	auto const ice{Wireice * 1000.0}; // mm
	if ((ice < 0.5) || (Settings.wire_ice_breaks <= 0.f))
		return 0.0;
	auto const speed{std::min(std::abs(Speed), 60.0)};
	// at 2 mm of ice and 90 km/h about once a second; more as the ice gets thicker, and much more with the speed
	auto rate{Settings.wire_ice_breaks * (ice / 2.0) * std::pow(speed / 25.0, 1.5)};
	rate *= 1.0 + std::clamp(Plaster, 0.0, 1.0);
	if (false == Leading)
		rate *= Settings.trailing_share;
	return rate;
}

double break_length(settings const &Settings, double const Wireice, double const Random)
{
	auto const ice{std::max(Wireice * 1000.0, 0.0)};
	return std::min(Settings.break_time * std::sqrt(ice) * (0.5 + std::clamp(Random, 0.0, 1.0)), 0.6);
}

} // namespace weather_physics
