/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the answer of Open-Meteo read into a weather report (environment/liveweather.h), apart from the fetching, for the tests

#include "stdafx.h"
#include "environment/liveweather.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <vector>

#include <nlohmann/json.hpp>

namespace
{

// the variables of the "current" block asked for
char const *const CURRENT{
    "temperature_2m,relative_humidity_2m,dew_point_2m,weather_code,cloud_cover,cloud_cover_low,cloud_cover_mid,cloud_cover_high,"
    "wind_speed_10m,wind_direction_10m,wind_gusts_10m,visibility,snow_depth"};

// a number of the block, nullopt where it's missing or null
std::optional<double> number(nlohmann::json const &Block, char const *Name)
{
	auto const item{Block.find(Name)};
	if ((item == Block.end()) || (false == item->is_number()))
		return std::nullopt;
	return item->get<double>();
}

// a temperature of a METAR: whole degrees, M for below zero
std::string temperature(double const Value)
{
	auto const whole{static_cast<int>(std::lround(Value))};
	std::ostringstream text;
	text << (whole < 0 ? "M" : "") << std::setw(2) << std::setfill('0') << std::abs(whole);
	return text.str();
}

// a layer of clouds of a METAR: the amount by the share of the sky covered (%), the base in hundreds of feet; empty for too few
std::string layer(double const Cover, double const Base)
{
	if (Cover < 6.0)
		return {};
	auto const amount{Cover <= 25.0 ? "FEW" : Cover <= 50.0 ? "SCT" : Cover <= 87.0 ? "BKN" : "OVC"};
	auto const hundreds{std::clamp(static_cast<int>(std::lround(Base / 30.48)), 1, 999)};
	std::ostringstream text;
	text << amount << std::setw(3) << std::setfill('0') << hundreds;
	return text.str();
}

} // namespace

std::string live_weather::url(glm::dvec2 const &Place)
{
	std::ostringstream text;
	text << std::fixed << std::setprecision(4) << "https://api.open-meteo.com/v1/forecast?latitude=" << Place.x << "&longitude=" << Place.y << "&current=" << CURRENT
	     << "&wind_speed_unit=ms";
	return text.str();
}

std::string live_weather::present_weather(int const Code)
{
	switch (Code)
	{
	case 45: return "FG";
	case 48: return "FZFG"; // the fog leaving rime
	case 51: return "-DZ";
	case 53: return "DZ";
	case 55: return "+DZ";
	case 56: return "-FZDZ";
	case 57: return "+FZDZ";
	case 61: return "-RA";
	case 63: return "RA";
	case 65: return "+RA";
	case 66: return "-FZRA";
	case 67: return "+FZRA";
	case 71: return "-SN";
	case 73: return "SN";
	case 75: return "+SN";
	case 77: return "SG";
	case 80: return "-SHRA";
	case 81: return "SHRA";
	case 82: return "+SHRA";
	case 85: return "-SHSN";
	case 86: return "+SHSN";
	case 95: return "TSRA";
	case 96: return "TSRAGR";
	case 99: return "+TSRAGR";
	default: return {};
	}
}

live_weather::report live_weather::read(std::string const &Answer, glm::dvec2 const &Place)
{
	report result;
	result.place = Place;
	auto const answer{nlohmann::json::parse(Answer, nullptr, false)};
	if (answer.is_discarded() || (false == answer.is_object()))
	{
		result.error = "the answer isn't json";
		return result;
	}
	if (auto const reason{answer.find("reason")}; (reason != answer.end()) && reason->is_string())
	{
		result.error = reason->get<std::string>();
		return result;
	}
	auto const current{answer.find("current")};
	if ((current == answer.end()) || (false == current->is_object()))
	{
		result.error = "no current conditions in the answer";
		return result;
	}
	auto const &block{*current};
	auto const air{number(block, "temperature_2m")};
	if (false == air.has_value())
	{
		result.error = "no temperature in the answer";
		return result;
	}
	auto const dew{number(block, "dew_point_2m").value_or(*air - 3.0)};
	std::vector<std::string> groups;
	// the wind at 10 m, m/s, to the nearest ten degrees it blows from; the gusts where they're well over it
	auto const speed{number(block, "wind_speed_10m").value_or(0.0)};
	auto const gust{number(block, "wind_gusts_10m").value_or(0.0)};
	auto const direction{static_cast<int>(std::lround(number(block, "wind_direction_10m").value_or(0.0) / 10.0) * 10) % 360};
	{
		std::ostringstream wind;
		auto const metres{static_cast<int>(std::lround(speed))};
		wind << std::setw(3) << std::setfill('0') << (metres > 0 ? (direction == 0 ? 360 : direction) : 0) << std::setw(2) << std::setfill('0') << metres;
		if (gust >= speed + 5.0)
			wind << 'G' << std::setw(2) << std::setfill('0') << static_cast<int>(std::lround(gust));
		wind << "MPS";
		groups.emplace_back(wind.str());
	}
	// the visibility, m
	auto const visibility{std::clamp(number(block, "visibility").value_or(10000.0), 0.0, 9999.0)};
	{
		std::ostringstream text;
		text << std::setw(4) << std::setfill('0') << static_cast<int>(std::lround(visibility));
		groups.emplace_back(text.str());
	}
	// what falls, and a mist where the air is wet and the view short without anything else
	auto const code{static_cast<int>(number(block, "weather_code").value_or(0.0))};
	auto weather{present_weather(code)};
	if (weather.empty() && (visibility < 5000.0) && (number(block, "relative_humidity_2m").value_or(0.0) > 85.0))
		weather = "BR";
	if (false == weather.empty())
		groups.emplace_back(weather);
	// the clouds: the low ones with the base the spread of the temperature and the dew point gives (125 m a degree), the middle and
	// the high ones at the heights of their levels
	auto const spread{std::max(*air - dew, 0.0)};
	auto const lowbase{std::clamp(125.0 * spread, 120.0, 1900.0)};
	auto const shower{(code >= 80) && (code != 77)};
	auto clouds{0};
	for (auto const &[name, base] : {std::pair{"cloud_cover_low", lowbase}, std::pair{"cloud_cover_mid", 3500.0}, std::pair{"cloud_cover_high", 7500.0}})
	{
		auto text{layer(number(block, name).value_or(0.0), base)};
		if (text.empty())
			continue;
		// the heaped clouds of the showers and the storms
		if (shower && (clouds == 0))
			text += (code >= 95 ? "CB" : "TCU");
		groups.emplace_back(text);
		++clouds;
	}
	if (clouds == 0)
	{
		// the total only, where the layers are missing
		if (auto const total{layer(number(block, "cloud_cover").value_or(0.0), lowbase)}; false == total.empty())
			groups.emplace_back(total);
		else
			groups.emplace_back("NSC");
	}
	groups.emplace_back(temperature(*air) + "/" + temperature(dew));
	std::ostringstream metar;
	for (std::size_t index = 0; index < groups.size(); ++index)
		metar << (index > 0 ? " " : "") << groups[index];
	result.metar = metar.str();
	if (auto const depth{number(block, "snow_depth")})
		result.snow_depth = static_cast<float>(std::max(*depth, 0.0));
	return result;
}

