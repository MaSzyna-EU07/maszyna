/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "environment/snowcover.h"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double HOUR{3600.0};
// water of the heaviest snowfall and rain, m an hour (5 mm an hour)
constexpr double HEAVIEST_PRECIPITATION{5.0e-3};
constexpr float WATER_DENSITY{1000.f};
// the densest the lying snow gets here: old, wet and settled, kg/m3
constexpr float DENSEST_SNOW{500.f};
// the deepest snow kept, m
constexpr float DEEPEST_SNOW{3.f};
// the thickest ice kept on a wire, m: heavier sleeves break off
constexpr float THICKEST_WIRE_ICE{0.04f};
// time over which the wind carrying the snow works it into drifts, s
constexpr double DRIFT_TIME{1.5 * HOUR};
// m an hour the wind fills a place cleared of the snow (a track the trains keep clear, sunk in the snow around) at the most it
// carries
constexpr double CLEARED_FILL{0.3};

float smoothstep(float const Edge0, float const Edge1, float const Value)
{
	auto const t{std::clamp((Value - Edge0) / (Edge1 - Edge0), 0.f, 1.f)};
	return t * t * (3.f - 2.f * t);
}

// the density of new snow at a temperature, kg/m3 (Hedstrom and Pomeroy 1998): about 70 in a hard frost, over 100 at zero
float new_snow_density(float const Temperature)
{
	return std::clamp(67.9f + 51.3f * std::exp(std::min(Temperature, 2.f) / 2.6f), 50.f, 250.f);
}

// a share moved towards Target over a time constant of Time seconds in a step of Deltatime
float approach(float const Value, float const Target, double const Deltatime, double const Time)
{
	auto const share{static_cast<float>(1.0 - std::exp(-Deltatime / std::max(Time, 1e-3)))};
	return Value + (Target - Value) * share;
}

// the wind over which the snow starts to move along the ground, m/s: about 4 for new dry snow, more for old, crusted or wet snow
float carrying_threshold(float const Fresh, float const Wetness)
{
	return 4.f + 3.f * (1.f - Fresh) + 8.f * Wetness;
}

} // namespace

void snow_cover::reset()
{
	auto const scale{time_scale};
	*this = snow_cover{};
	time_scale = scale;
}

void snow_cover::set(float const Depth, float const Drift, glm::vec2 const &Wind, float const Fresh, float const Tracks)
{
	m_depth = std::clamp(Depth, 0.f, DEEPEST_SNOW);
	// the tracks hold what was brought since they were cleared (TTrack::snow_at()), which none of them was yet at the start
	if (Tracks >= 0.f)
		m_brought = static_cast<double>(m_depth * std::clamp(Tracks, 0.f, 1.f));
	m_drift = std::clamp(Drift, 0.f, 1.f);
	if (glm::length(Wind) > 1e-3f)
		m_driftwind = Wind;
	m_fresh = std::clamp(Fresh, 0.f, 1.f);
	// snow which lies for a while has settled
	m_density = (m_fresh > 0.5f ? 150.f : 280.f);
	m_wetness = 0.f;
}

void snow_cover::update(double const Deltatime, snow_weather const &Weather)
{
	if (Deltatime <= 0.0)
		return;
	m_wind = Weather.wind;

	auto const step{Deltatime * static_cast<double>(std::max(time_scale, 0.f))};
	if (step <= 0.0)
		return;
	fall(step, Weather);
	melt(step, Weather);
	carry(step, Weather);
	stick(step, Weather);
	ice(step, Weather);
}

void snow_cover::fall(double const Deltatime, snow_weather const &Weather)
{
	m_falling = approach(m_falling, std::clamp(Weather.snowfall, 0.f, 1.f), Deltatime, 60.0);
	if (Weather.snowfall <= 0.f)
		return;
	// the water of the snow fallen in the step, m, and the depth it makes
	auto const water{static_cast<float>(HEAVIEST_PRECIPITATION / HOUR * Deltatime) * Weather.snowfall};
	// the flakes are as dense as the air they fell through made them: by its wet-bulb temperature
	auto const density{new_snow_density(Weather.wet_bulb)};
	auto const added{water * WATER_DENSITY / density};
	auto const before{m_depth};
	auto const depth{std::min(before + added, DEEPEST_SNOW)};
	if (depth <= 0.f)
		return;
	auto const mass{before * m_density + added * density};
	m_density = std::clamp(mass / (before + added), 50.f, DENSEST_SNOW);
	m_fresh = (m_fresh * before + added) / (before + added);
	m_depth = depth;
	m_brought += added;
	// new snow falling in a calm evens out the drifts below it; in a wind it is carried into them at once
	auto const calm{1.f - smoothstep(4.f, 10.f, glm::length(m_wind))};
	auto const evened{added / (before + added + 0.01f)};
	m_drift *= 1.f - std::clamp(evened * calm, 0.f, 1.f);
}

void snow_cover::melt(double const Deltatime, snow_weather const &Weather)
{
	auto const temperature{Weather.temperature};
	auto const warm{smoothstep(-4.f, 1.f, temperature)};
	// the wetness follows the warmth and the rain, and freezes into a crust in a frost
	// (wet snow falling, above a wet-bulb temperature of zero, wets it too)
	auto const wetfall{(Weather.snowfall > 0.f) ? std::clamp(Weather.wet_bulb, 0.f, 1.f) : 0.f};
	auto const wet{std::max({std::clamp((temperature + 0.5f) / 2.f, 0.f, 1.f), (temperature > -1.f ? Weather.rainfall : 0.f), wetfall})};
	if (wet > m_wetness)
		m_wetness = approach(m_wetness, wet, Deltatime, 0.5 * HOUR);
	else
		m_wetness = approach(m_wetness, wet, Deltatime, (temperature < -2.f ? 2.0 : 6.0) * HOUR);
	if (m_depth <= 0.f)
	{
		m_depth = 0.f;
		m_fresh = 0.f;
		m_drift = 0.f;
		return;
	}
	// the loose snow settles, faster in the warmth; the mass stays, the density grows
	auto const settling{(0.02 + 0.08 * warm) / HOUR * m_fresh};
	auto const kept{static_cast<float>(std::exp(-settling * Deltatime))};
	m_depth *= kept;
	m_density = std::min(m_density / kept, DENSEST_SNOW);
	// and it stops being loose: in a day in a hard frost, in a few hours near zero, at once when wet
	auto const looseness{(3.0 + 21.0 * std::clamp(-temperature / 15.f, 0.f, 1.f)) * HOUR};
	m_fresh = approach(m_fresh, 0.f, Deltatime, looseness);
	m_fresh = approach(m_fresh, 0.f, Deltatime, 0.5 * HOUR / std::max(m_wetness, 1e-3f));
	// dry air and wind take a little of it straight off (a few tenths of a millimetre of water a day)
	auto const sublimated{0.4e-3 * (1.0 - Weather.humidity) * (0.3 + std::min(glm::length(m_wind), 10.f) / 10.0) / 86400.0 * Deltatime};
	m_depth = std::max(0.f, m_depth - static_cast<float>(sublimated) * WATER_DENSITY / m_density);
	// melting by the degree-day: 3 mm of water a degree and a day, and the rain falling on it
	if (temperature <= 0.f)
		return;
	auto const water{(3.0e-3 * temperature / 86400.0 + 0.5e-3 * Weather.rainfall * temperature / HOUR) * Deltatime};
	m_density = approach(m_density, DENSEST_SNOW, Deltatime, 6.0 * HOUR);
	m_depth = std::max(0.f, m_depth - static_cast<float>(water) * WATER_DENSITY / m_density);
}

void snow_cover::carry(double const Deltatime, snow_weather const &Weather)
{
	auto const speed{glm::length(m_wind)};
	auto const threshold{carrying_threshold(m_fresh, m_wetness)};
	auto const lying{smoothstep(0.002f, 0.02f, m_depth)};
	// the snow carried along the ground grows with the cube of the wind over the threshold
	auto const over{std::max(0.f, speed - threshold)};
	auto const carriage{over * over * over / (over * over * over + 125.f) * m_fresh * lying};
	// what is seen moving: the loose snow lifted from the ground, and the falling snow driven along it in a strong wind
	auto const blowing{std::max(m_fresh * lying * smoothstep(threshold - 1.f, threshold + 5.f, speed), m_falling * smoothstep(6.f, 15.f, speed) * 0.6f)};
	m_blowing = approach(m_blowing, blowing, Deltatime, 5.0);
	if (carriage <= 0.f)
		return;
	// what it carries fills the places cleared of the snow too
	m_brought += static_cast<double>(carriage) * CLEARED_FILL / HOUR * Deltatime;
	auto const share{static_cast<float>(1.0 - std::exp(-carriage * Deltatime / DRIFT_TIME))};
	m_drift += (1.f - m_drift) * share;
	// the drifts turn to the wind which works them now, the faster the less they are made yet
	auto const turn{std::clamp(share / (m_drift + 0.1f) * 2.f, 0.f, 1.f)};
	m_driftwind = m_driftwind + (Weather.wind - m_driftwind) * turn;
	// the snow carried packs into wind slabs
	m_fresh = approach(m_fresh, 0.f, Deltatime, 4.0 * HOUR / std::max(carriage, 1e-3f));
}

void snow_cover::stick(double const Deltatime, snow_weather const &Weather)
{
	auto const temperature{Weather.temperature};
	auto const speed{glm::length(m_wind)};
	// wet snow, at a wet-bulb temperature about zero, sticks best; the dry snow of a frost sticks a little
	auto const stickiness{std::clamp(1.f - std::abs(Weather.wet_bulb - 0.3f) / 2.5f, 0.15f, 1.f)};
	auto const rate{m_falling * smoothstep(2.f, 10.f, speed) * stickiness};
	if (rate > 0.f)
	{
		auto const share{static_cast<float>(1.0 - std::exp(-rate * Deltatime / HOUR))};
		m_plaster += (1.f - m_plaster) * share;
		m_plasterwind = m_plasterwind + (Weather.wind - m_plasterwind) * std::clamp(share * 3.f, 0.f, 1.f);
	}
	// it falls off in the warmth and the rain, and slowly goes in dry air and wind
	auto loss{(0.5 + 3.0 * (1.0 - Weather.humidity)) * (0.5 + std::min(speed, 10.f) / 10.0) / (24.0 * HOUR)};
	if (temperature > 0.5f)
		loss += temperature / HOUR;
	loss += Weather.rainfall * 2.0 / HOUR;
	m_plaster *= static_cast<float>(std::exp(-loss * Deltatime));
}

float snow_cover::wire_ice() const
{
	auto total{0.f};
	for (auto const ice : m_wireice)
		total += ice;
	return total;
}

float snow_cover::wire_ice_density() const
{
	// the densities of the kinds, in the order of ice_kind
	constexpr float densities[]{900.f, 450.f, 120.f, 550.f};
	auto mass{0.f};
	auto total{0.f};
	for (int kind = 0; kind < static_cast<int>(ice_kind::count); ++kind)
	{
		mass += m_wireice[kind] * densities[kind];
		total += m_wireice[kind];
	}
	return total > 0.f ? mass / total : 900.f;
}

void snow_cover::ice(double const Deltatime, snow_weather const &Weather)
{
	auto const temperature{Weather.temperature};
	auto const speed{glm::length(m_wind)};
	auto const step{static_cast<float>(Deltatime)};
	auto &glaze{m_wireice[static_cast<int>(ice_kind::glaze)]};
	auto &hard{m_wireice[static_cast<int>(ice_kind::rime_hard)]};
	auto &soft{m_wireice[static_cast<int>(ice_kind::rime_soft)]};
	auto &sleeve{m_wireice[static_cast<int>(ice_kind::wet_snow)]};
	auto const allowed = [this](ice_kind const Kind) { return wire_ice_kinds[static_cast<int>(Kind)]; };

	// the growth of each kind, the radial thickness, m/s
	// glaze: rain or drizzle freezing on the wires, about a third of the precipitation for a cylinder
	if (allowed(ice_kind::glaze) && (temperature <= 0.f) && (temperature > -8.f))
		glaze += Weather.rainfall * 1.3e-3f / 3600.f * step;
	// rime: the droplets of a freezing fog caught by the wire. in a wind and a moderate frost they freeze on as they hit, into a
	// dense hard rime; in a calm or a hard frost they make the feathery soft rime, and the saturated air without a fog the
	// hoarfrost
	auto const fog{std::clamp(Weather.fog, 0.f, 1.f)};
	auto const saturated{std::clamp((Weather.humidity - 0.95f) / 0.05f, 0.f, 1.f)};
	if (temperature < -1.f)
	{
		// the share of the hard rime: 1 in a wind of 4 m/s and more and a frost down to -10, none in a calm or past -14
		auto const hardshare{smoothstep(2.f, 4.f, speed) * smoothstep(-14.f, -10.f, temperature)};
		if (allowed(ice_kind::rime_hard))
			hard += fog * hardshare * std::min(speed, 10.f) / 10.f * 0.3e-3f / 3600.f * step;
		if (allowed(ice_kind::rime_soft))
			soft += (fog * (1.f - hardshare) * 0.25e-3f + saturated * 0.4f * 0.1e-3f) / 3600.f * step;
	}
	// a sleeve of wet snow turning round the wire
	if (allowed(ice_kind::wet_snow) && (Weather.wet_bulb > -1.f) && (Weather.wet_bulb < 1.2f) && (Weather.snowfall > 0.f))
		sleeve += m_falling * 2.0e-3f / 3600.f * step;

	// what goes: a kind left out melts away at once
	for (int kind = 0; kind < static_cast<int>(ice_kind::count); ++kind)
		if (false == wire_ice_kinds[kind])
			m_wireice[kind] = 0.f;
	auto const dry{0.15e-3f * (1.f - Weather.humidity) / 3600.f};
	auto const thaw{(temperature > 0.f) ? 1.0e-3f * temperature / 3600.f : 0.f};
	// the glaze melts in the warmth and sublimates slowly in dry air
	glaze = std::max(0.f, glaze - (dry + 0.02e-3f / 3600.f + thaw) * step);
	// the hard rime the same, and a gale breaks it off
	hard = std::max(0.f, hard - (dry + 0.02e-3f / 3600.f + thaw) * step);
	if (speed > 15.f)
		hard *= static_cast<float>(std::exp(-Deltatime * (speed - 15.f) / 3600.f));
	// the soft rime sublimates three times as fast, melts at once and a wind of 8 m/s shakes it off
	soft = std::max(0.f, soft - (3.f * dry + 0.05e-3f / 3600.f + 3.f * thaw) * step);
	if (speed > 8.f)
		soft *= static_cast<float>(std::exp(-Deltatime * (speed - 8.f) / 600.f));
	// the wet snow slides off in a thaw and in the rain, and the wind tears it off
	sleeve = std::max(0.f, sleeve - (dry + thaw) * step);
	if ((temperature > 0.5f) || (Weather.rainfall > 0.f))
		sleeve *= static_cast<float>(std::exp(-Deltatime / 1800.0));
	if (speed > 12.f)
		sleeve *= static_cast<float>(std::exp(-Deltatime * (speed - 12.f) / 1800.0));
	// heavier sleeves break off
	auto const total{wire_ice()};
	if (total > THICKEST_WIRE_ICE)
		for (auto &ice : m_wireice)
			ice *= THICKEST_WIRE_ICE / total;
}
