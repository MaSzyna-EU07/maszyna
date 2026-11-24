/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorProjectSetup.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <format>
#include <sstream>

namespace project_setup
{

namespace
{

std::array<std::pair<norms::profile_id, std::string_view>, 4> constexpr profile_keys{{
    {norms::profile_id::plk, "plk"},
    {norms::profile_id::tsi, "tsi"},
    {norms::profile_id::en13803, "en13803"},
    {norms::profile_id::ch, "ch"},
}};

std::array<std::pair<norms::traffic, std::string_view>, 3> constexpr traffic_keys{{
    {norms::traffic::passenger, "passenger"},
    {norms::traffic::mixed, "mixed"},
    {norms::traffic::freight, "freight"},
}};

// a name with no blanks: %20 for a blank, %25 for a percent sign, - for none
std::string word(std::string_view const Text)
{
	if (Text.empty())
		return "-";
	std::string result;
	for (auto const character : Text)
	{
		if (character == ' ')
			result += "%20";
		else if (character == '%')
			result += "%25";
		else
			result += character;
	}
	return result;
}

std::string unword(std::string_view const Word)
{
	if (Word == "-")
		return {};
	std::string result;
	for (std::size_t i = 0; i < Word.size(); ++i)
	{
		if (Word.substr(i, 3) == "%20")
		{
			result += ' ';
			i += 2;
		}
		else if (Word.substr(i, 3) == "%25")
		{
			result += '%';
			i += 2;
		}
		else
			result += Word[i];
	}
	return result;
}

std::optional<double> number_of(std::string_view const Text)
{
	double value{0.0};
	auto const [end, error]{std::from_chars(Text.data(), Text.data() + Text.size(), value)};
	if (error != std::errc{} || end != Text.data() + Text.size() || false == std::isfinite(value))
		return std::nullopt;
	return value;
}

// the value of a key of the record into the setup; a value it can't read leaves the default
void take(setup &Setup, std::string_view const Key, std::string_view const Value)
{
	if (Key == "norms")
	{
		if (auto const profile{profile_of_key(Value)})
			Setup.profile = *profile;
	}
	else if (Key == "speed")
	{
		if (auto const speed{number_of(Value)}; speed.has_value() && *speed > 0.0)
			Setup.speed = std::clamp(*speed, 5.0, 400.0);
	}
	else if (Key == "traffic")
	{
		for (auto const &[kind, name] : traffic_keys)
			if (name == Value)
				Setup.traffic = kind;
	}
	else if (Key == "track")
	{
		if (Value == "main" || Value == "side")
			Setup.track = Value == "side" ? norms::track_kind::side : norms::track_kind::main;
	}
	else if (Key == "gauge")
	{
		if (auto const gauge{number_of(Value)}; gauge.has_value() && *gauge >= 300.0 && *gauge <= 2000.0)
			Setup.gauge = static_cast<int>(std::lround(*gauge));
	}
	else if (Key == "rail")
		Setup.rail = unword(Value);
	else if (Key == "rails")
		Setup.rails = unword(Value);
	else if (Key == "ballast")
		Setup.ballast = unword(Value);
}

} // namespace

std::string_view profile_key(norms::profile_id const Profile)
{
	for (auto const &[id, key] : profile_keys)
		if (id == Profile)
			return key;
	return "plk";
}

std::optional<norms::profile_id> profile_of_key(std::string_view const Key)
{
	for (auto const &[id, key] : profile_keys)
		if (key == Key)
			return id;
	return std::nullopt;
}

norms::line_class line_class(setup const &Setup, double const Speed)
{
	norms::line_class result;
	result.vmax = Speed > 0.0 ? Speed : Setup.speed;
	result.kind = Setup.traffic;
	result.track = Setup.track;
	return result;
}

double rail_axes(setup const &Setup)
{
	return static_cast<double>(Setup.gauge) + 65.0;
}

double diverging_speed(setup const &Setup)
{
	if (Setup.track == norms::track_kind::side || Setup.speed < 100.0)
		return 40.0;
	if (Setup.speed < 160.0)
		return 60.0;
	return Setup.speed < 200.0 ? 80.0 : 100.0;
}

std::string record(setup const &Setup)
{
	std::string_view traffic{"mixed"};
	for (auto const &[kind, name] : traffic_keys)
		if (kind == Setup.traffic)
			traffic = name;
	auto text{std::format("setup norms {} speed {} traffic {} track {} gauge {} rail {}", profile_key(Setup.profile), Setup.speed, traffic, Setup.track == norms::track_kind::side ? "side" : "main",
	                      Setup.gauge, word(Setup.rail))};
	if (false == Setup.rails.empty())
		text += " rails " + word(Setup.rails);
	if (false == Setup.ballast.empty())
		text += " ballast " + word(Setup.ballast);
	return text;
}

std::optional<setup> read(std::string_view const Line)
{
	std::istringstream words{std::string{Line}};
	std::string keyword;
	if (false == static_cast<bool>(words >> keyword) || keyword != "setup")
		return std::nullopt;
	setup result;
	for (std::string key, value; words >> key >> value;)
		take(result, key, value);
	return result;
}

std::optional<setup> read(std::vector<std::string> const &Lines)
{
	for (auto const &line : Lines)
		if (auto const found{read(std::string_view{line})})
			return found;
	return std::nullopt;
}

} // namespace project_setup
