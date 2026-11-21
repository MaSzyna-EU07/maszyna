/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <cmath>
#include <optional>

// the moisture of the air and what follows from it: the humidity from the temperature and the dew point and back, the wet-bulb
// temperature, which tells what falls (snow reaches the ground in dry air a few degrees above zero, since it cools as it evaporates,
// and turns to rain just above zero in saturated air), and the temperature through the day between its minimum and maximum
namespace psychrometry
{

// pressure of the saturated vapour over water at a temperature, hPa (Magnus, Alduchov and Eskridge 1996)
inline float saturation_pressure(float const Temperature)
{
	return 6.1094f * std::exp(17.625f * Temperature / (Temperature + 243.04f));
}

// relative humidity, 0..1, of air at a temperature with a dew point, degrees C
inline float relative_humidity(float const Temperature, float const Dewpoint)
{
	return std::clamp(saturation_pressure(std::min(Dewpoint, Temperature)) / saturation_pressure(Temperature), 0.f, 1.f);
}

// the dew point of air at a temperature with a relative humidity, 0..1
inline float dew_point(float const Temperature, float const Humidity)
{
	auto const gamma{std::log(std::clamp(Humidity, 0.01f, 1.f)) + 17.625f * Temperature / (Temperature + 243.04f)};
	return 243.04f * gamma / (17.625f - gamma);
}

// the wet-bulb temperature of air at a temperature with a relative humidity, 0..1 (Stull 2011, good to about 0.3 degrees between 5%
// and 99% and from -20 to 50 degrees); the temperature itself in saturated air
inline float wet_bulb(float const Temperature, float const Humidity)
{
	auto const percent{std::clamp(Humidity, 0.05f, 1.f) * 100.f};
	if (percent >= 99.f)
		return Temperature;
	auto const result{Temperature * std::atan(0.151977f * std::sqrt(percent + 8.313659f)) + std::atan(Temperature + percent) - std::atan(percent - 1.676331f) +
	                  0.00391838f * std::pow(percent, 1.5f) * std::atan(0.023101f * percent) - 4.686035f};
	return std::min(result, Temperature);
}

// the air temperature at an hour of the local day (0..24) between the minimum of the day, about sunrise, and its maximum in the
// afternoon: half a cosine up over the ten hours from 5 to 15, half a cosine down over the fourteen of the evening and the night
inline float diurnal_temperature(float const Minimum, float const Maximum, float const Hour)
{
	constexpr float coldest{5.f};
	constexpr float warmest{15.f};
	constexpr float pi{3.14159265f};
	auto hour{std::fmod(Hour, 24.f)};
	if (hour < 0.f)
		hour += 24.f;
	float warmth; // 0 at the minimum, 1 at the maximum
	if ((hour >= coldest) && (hour <= warmest))
		warmth = 0.5f - 0.5f * std::cos(pi * (hour - coldest) / (warmest - coldest));
	else
	{
		auto const since{hour > warmest ? hour - warmest : hour + 24.f - warmest};
		warmth = 0.5f + 0.5f * std::cos(pi * since / (24.f - warmest + coldest));
	}
	return Minimum + (Maximum - Minimum) * warmth;
}

// what falls
enum class falling
{
	nothing,
	rain,
	freezing_rain, // rain on a frozen ground and frozen wires: glaze
	wet_snow, // snow melting as it falls, sticky and heavy
	snow
};

// what falls when something does, by the temperature of the air and, where the moisture of the air is known, its wet-bulb
// temperature: snow up to a wet-bulb temperature of 1 degree (without the moisture: an air temperature of 1 degree, as the weather
// always took it), wet above zero. Rain: whether a description says rain (true) or snow (false) whatever the temperature
inline falling falling_kind(float const Temperature, std::optional<float> const Wetbulb, std::optional<bool> const Rain)
{
	auto const reference{Wetbulb.value_or(Temperature)};
	if (Rain)
	{
		if (*Rain)
			return (Temperature <= 0.f) ? falling::freezing_rain : falling::rain;
		return (reference > 0.f) ? falling::wet_snow : falling::snow;
	}
	if (reference > 1.f)
		return falling::rain;
	return (reference > 0.f) ? falling::wet_snow : falling::snow;
}

inline bool is_snow(falling const Kind)
{
	return (Kind == falling::snow) || (Kind == falling::wet_snow);
}

} // namespace psychrometry
