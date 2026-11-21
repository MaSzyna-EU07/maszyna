/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "environment/railsurface.h"

#include <algorithm>
#include <cmath>

namespace
{

constexpr double HOUR{3600.0};
constexpr double MINUTE{60.0};

float smoothstep(float const Edge0, float const Edge1, float const Value)
{
	auto const t{std::clamp((Value - Edge0) / (Edge1 - Edge0), 0.f, 1.f)};
	return t * t * (3.f - 2.f * t);
}

// a share moved towards Target over a time constant of Time seconds in a step of Deltatime
float approach(float const Value, float const Target, double const Deltatime, double const Time)
{
	auto const share{static_cast<float>(1.0 - std::exp(-Deltatime / std::max(Time, 1e-3)))};
	return Value + (Target - Value) * share;
}

} // namespace

void rail_surface::reset()
{
	auto const scale{time_scale};
	*this = rail_surface{};
	time_scale = scale;
}

float rail_surface::leaf_season(int const Yearday)
{
	// days of the year (not a leap one): 20 September, 15 October, 20 November, 10 December
	auto const day{static_cast<float>(Yearday)};
	return smoothstep(263.f, 288.f, day) * (1.f - smoothstep(324.f, 344.f, day));
}

void rail_surface::update(double const Deltatime, snow_weather const &Weather, float const Daylight, float const Leaves)
{
	if (Deltatime <= 0.0)
		return;
	auto const step{Deltatime * static_cast<double>(std::max(time_scale, 0.f))};
	if (step <= 0.0)
		return;
	this->step(step, Weather, Daylight, Leaves);
}

void rail_surface::settle(snow_weather const &Weather, float const Daylight, float const Leaves)
{
	// a day of the same weather, in steps of ten minutes: long enough for each part to come to what the weather keeps it at
	for (int i = 0; i < 144; ++i)
		step(10.0 * MINUTE, Weather, Daylight, Leaves);
}

void rail_surface::step(double const Deltatime, snow_weather const &Weather, float const Daylight, float const Leaves)
{
	auto const temperature{Weather.temperature};
	auto const rain{std::clamp(Weather.rainfall, 0.f, 1.f)};
	auto const snowfall{std::clamp(Weather.snowfall, 0.f, 1.f)};
	auto const humidity{std::clamp(Weather.humidity, 0.f, 1.f)};
	auto const wind{glm::length(Weather.wind)};
	auto const daylight{std::clamp(Daylight, 0.f, 1.f)};
	auto &rail{m_state};

	// the water. the rain wets the rails in a minute or two, the wet snow melting on them a little; the dew comes on a calm humid
	// night, when the steel gets colder than the air
	auto wetting{0.f};
	if ((rain > 0.f) && (temperature > -0.5f))
		wetting = std::min(1.f, 0.3f + rain);
	if ((snowfall > 0.f) && (temperature > 0.5f))
		wetting = std::max(wetting, 0.5f * snowfall);
	auto const dew{(temperature > 0.f) ? 0.25f * smoothstep(0.93f, 1.f, humidity) * (1.f - smoothstep(0.1f, 0.4f, daylight)) * (1.f - smoothstep(1.f, 4.f, wind)) : 0.f};
	auto const target{std::max(wetting, dew)};
	if (rail.water < target)
	{
		rail.water = approach(rail.water, target, Deltatime, (wetting > dew) ? 2.0 * MINUTE / (0.2 + rain) : 0.5 * HOUR);
	}
	else
	{
		// the steel dries in half an hour: faster in dry air, in the wind, in the sun and the warmth
		auto const evaporation{(0.2f + 2.f * (1.f - humidity)) * (1.f + wind / 4.f) * (0.5f + daylight) * std::clamp(std::exp(temperature / 15.f), 0.3f, 3.f)};
		rail.water = approach(rail.water, target, Deltatime, 0.5 * HOUR / std::max(evaporation, 0.05f));
	}

	// the ice: the water freezing on the rail, the freezing rain, the frost of a humid or foggy air in a frost
	if (temperature < -0.5f)
	{
		auto const frozen{rail.water * static_cast<float>(1.0 - std::exp(-Deltatime / (10.0 * MINUTE / (1.0 + -temperature))))};
		rail.water -= frozen;
		rail.ice = std::min(1.f, rail.ice + frozen);
	}
	if ((rain > 0.f) && (temperature <= 0.f))
		rail.ice = approach(rail.ice, 1.f, Deltatime, 5.0 * MINUTE / (0.2 + rain));
	auto const moist{std::max(Weather.fog, smoothstep(0.95f, 1.f, humidity))};
	if ((temperature < 0.f) && (moist > 0.f))
		rail.ice = std::max(rail.ice, approach(rail.ice, 0.3f * moist, Deltatime, 3.0 * HOUR));
	if (temperature > 0.5f)
		rail.ice = approach(rail.ice, 0.f, Deltatime, 15.0 * MINUTE / (temperature * (1.0 + 3.0 * rain)));

	// the dirt of a dry spell builds up over a day; the rain washes it off, a drizzle slowly
	if (rail.water < 0.05f)
		rail.dirt = approach(rail.dirt, 1.f, Deltatime, 24.0 * HOUR);
	if (rain > 0.f)
		rail.dirt = approach(rail.dirt, 0.f, Deltatime, 15.0 * MINUTE / (rain * rain + 0.02));

	// the leaves fall in the autumn, sooner in a wind, and rot away out of it
	auto const leaves{std::clamp(Leaves, 0.f, 2.f)};
	if (leaves > 0.f)
		rail.leaves = approach(rail.leaves, std::min(leaves, 1.f), Deltatime, 6.0 * HOUR / (1.0 + wind / 8.0) / std::max(leaves, 0.25f));
	else
		rail.leaves = approach(rail.leaves, 0.f, Deltatime, 48.0 * HOUR);

	rail.water = std::clamp(rail.water, 0.f, 1.f);
	rail.ice = std::clamp(rail.ice, 0.f, 1.f);
	rail.dirt = std::clamp(rail.dirt, 0.f, 1.f);
	rail.leaves = std::clamp(rail.leaves, 0.f, 1.f);
}
