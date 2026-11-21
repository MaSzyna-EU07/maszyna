/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "environment/weather.h"

#include "editor/editorGeoMap.hpp"
#include "editor/editorSettings.hpp"
#include "environment/climate.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "simulation/simulationenvironment.h"
#include "simulation/simulationtime.h"
#include "simulation/simulation.h"
#include "environment/georeference.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

#include <algorithm>

namespace
{

// name, abbreviation, base, thickness, extinction, cell, spread, bottom, top, top variation, anvil, erosion, stretch, opacity, flat.
// the extinction of the volumetric genera is about what makes the visibility inside a cloud a few tens of metres (cumulus) to
// a few hundred (altostratus); of the flat ones it is the optical depth of the sheet
cloud_genus const GENERA[static_cast<int>(cloud_type::count)]{
    {"cumulus", "Cu", 1100.f, 1200.f, 0.040f, 1400.f, 5000.f, 0.06f, 0.45f, 0.65f, 0.0f, 0.35f, 1.0f, 0.80f, false},
    {"towering_cumulus", "TCu", 900.f, 4000.f, 0.050f, 2500.f, 9000.f, 0.05f, 0.35f, 0.60f, 0.0f, 0.30f, 1.0f, 0.90f, false},
    {"cumulonimbus", "Cb", 800.f, 9000.f, 0.060f, 6000.f, 25000.f, 0.04f, 0.15f, 0.30f, 1.0f, 0.20f, 1.0f, 1.00f, false},
    {"stratocumulus", "Sc", 900.f, 600.f, 0.030f, 900.f, 3500.f, 0.15f, 0.50f, 0.30f, 0.0f, 0.30f, 1.0f, 0.85f, false},
    {"stratus", "St", 300.f, 450.f, 0.020f, 3000.f, 9000.f, 0.25f, 0.30f, 0.10f, 0.0f, 0.10f, 1.5f, 1.00f, false},
    {"nimbostratus", "Ns", 500.f, 3000.f, 0.030f, 4000.f, 15000.f, 0.10f, 0.30f, 0.15f, 0.0f, 0.10f, 1.0f, 1.00f, false},
    {"altocumulus", "Ac", 3500.f, 400.f, 0.020f, 450.f, 3000.f, 0.20f, 0.50f, 0.30f, 0.0f, 0.45f, 1.0f, 0.50f, false},
    {"altostratus", "As", 4000.f, 1500.f, 0.006f, 5000.f, 15000.f, 0.20f, 0.40f, 0.10f, 0.0f, 0.10f, 1.5f, 0.60f, false},
    {"cirrus", "Ci", 8000.f, 800.f, 0.50f, 3000.f, 12000.f, 0.0f, 0.0f, 0.0f, 0.0f, 0.50f, 5.0f, 0.15f, true},
    {"cirrostratus", "Cs", 8500.f, 700.f, 0.30f, 8000.f, 20000.f, 0.0f, 0.0f, 0.0f, 0.0f, 0.20f, 2.0f, 0.25f, true},
    {"cirrocumulus", "Cc", 7500.f, 300.f, 0.25f, 300.f, 3000.f, 0.0f, 0.0f, 0.0f, 0.0f, 0.60f, 1.0f, 0.20f, true},
};

// the old fog (exp2 of the squared distance over the fog range) thinned out a dark object to 5% at 1.7 ranges and to a half at
// 0.83. the visibility is put where an exponential fog crosses the old one around a third
float const LEGACY_VISIBILITY_SCALE{2.5f};
float const FEET{0.3048f};
float const KNOTS{0.514444f};

std::string upper(std::string_view Text)
{
	std::string result{Text};
	for (auto &c : result)
		c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
	return result;
}

std::string lower(std::string_view Text)
{
	std::string result{Text};
	for (auto &c : result)
		c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	return result;
}

bool all_digits(std::string_view Text)
{
	return !Text.empty() && std::all_of(Text.begin(), Text.end(), [](char c) { return std::isdigit(static_cast<unsigned char>(c)) != 0; });
}

// a number in the C locale, the whole token
std::optional<float> number(std::string_view Text)
{
	if (Text.empty())
		return std::nullopt;
	std::istringstream stream{std::string{Text}};
	stream.imbue(std::locale::classic());
	float value;
	stream >> value;
	if (stream.fail())
		return std::nullopt;
	char rest;
	if (stream >> rest)
		return std::nullopt;
	return value;
}

std::vector<std::string> split(std::string_view Text)
{
	std::vector<std::string> tokens;
	std::string token;
	for (auto const c : Text)
	{
		// commas and semicolons separate as the spaces do: the text of a memory cell set by an event can't hold spaces
		if (std::isspace(static_cast<unsigned char>(c)) || c == ';' || c == '=' || c == ',')
		{
			if (!token.empty())
				tokens.push_back(token);
			token.clear();
		}
		else
			token += c;
	}
	if (!token.empty())
		tokens.push_back(token);
	return tokens;
}

std::string strip_quotes(std::string Text)
{
	if (!Text.empty() && Text.front() == '\"')
		Text.erase(0, 1);
	if (!Text.empty() && Text.back() == '\"')
		Text.pop_back();
	return Text;
}

// the genus of a layer of a report, which names only the convective ones: by the height, the amount and the precipitation
cloud_type report_genus(float const Base, float const Coverage, bool const Precipitation, bool const Showers)
{
	if (Base < 2000.f)
	{
		if (Coverage >= 0.99f)
			return Precipitation ? cloud_type::nimbostratus : (Base < 600.f ? cloud_type::stratus : cloud_type::stratocumulus);
		if (Coverage > 0.5f)
			return (Showers ? cloud_type::cumulus : (Base < 400.f ? cloud_type::stratus : cloud_type::stratocumulus));
		return cloud_type::cumulus;
	}
	if (Base < 6000.f)
		return (Coverage > 0.5f ? cloud_type::altostratus : cloud_type::altocumulus);
	return (Coverage > 0.5f ? cloud_type::cirrostratus : cloud_type::cirrus);
}

float coverage_from_amount(std::string_view Amount)
{
	if (Amount == "FEW")
		return 1.5f / 8.f;
	if (Amount == "SCT")
		return 3.5f / 8.f;
	if (Amount == "BKN")
		return 6.f / 8.f;
	if (Amount == "OVC")
		return 1.f;
	return -1.f;
}

// one layer of a typical depth of its genus, below the next height it may not reach
cloud_layer typical_layer(cloud_type const Type, float const Coverage, float const Base)
{
	auto const &genus{cloud_genus_info(Type)};
	cloud_layer layer;
	layer.type = Type;
	layer.coverage = Coverage;
	layer.base = Base;
	layer.top = Base + genus.thickness;
	return layer;
}

bool is_section_keyword(std::string const &Token);

// the wind at the ground of a description, with the wind at the clouds going the same way half as fast again
void describe_wind(weather_conditions &Conditions, float const Direction, float const Speed, float const Gust)
{
	Conditions.wind_described = true;
	Conditions.wind_direction = Direction;
	Conditions.wind_speed = std::max(Speed, 0.f);
	Conditions.wind_gust = std::max(Gust, 0.f);
	Conditions.cloud_wind_direction = Direction;
	Conditions.cloud_wind_speed = std::max(Conditions.wind_speed * 1.5f, 2.f);
}

// where the scenery lies: the place the editor keeps for it (the origin of its aerial photos, PUWG 1992), degrees of latitude and
// longitude
std::optional<glm::vec2> editor_location()
{
	auto scenery{Global.SceneryFile};
	if (auto const slash = scenery.find_last_of("/\\"); slash != std::string::npos)
		scenery = scenery.substr(slash + 1);
	if (auto const dot = scenery.find_last_of('.'); dot != std::string::npos)
		scenery = scenery.substr(0, dot);
	double north{0.0};
	double east{0.0};
	if (scenery.empty() || (false == std::as_const(EditorSettings).orthophoto_origin(scenery, north, east)))
		return std::nullopt;
	auto const degrees{editor_geo::to_degrees(glm::dvec2{east, north})};
	return glm::vec2{static_cast<float>(degrees.x), static_cast<float>(degrees.y)};
}

// the weather of a kind (a preset of environment/climate.h, random: one drawn as often as each comes) the climate of the place has
// in the month of the simulation, in place of what Conditions held; the place and the seed are kept. false for an unknown kind
bool describe_climate(weather_conditions &Conditions, std::string const &Kind)
{
	auto const location{Conditions.location ? Conditions.location : editor_location()};
	auto const &station{location ? climate::stations().nearest(location->x, location->y) : climate::default_station()};
	auto const month{std::clamp(static_cast<int>(simulation::Time.data().wMonth), 1, 12) - 1};
	// a seed of the description makes the same day each time, without one it's another day each time
	auto const seed{Conditions.seed != 0 ? Conditions.seed : static_cast<std::uint32_t>(Random(1, 1000000))};
	auto preset{climate::preset::typical};
	if (Kind == "random")
		preset = climate::pick(station, month, static_cast<float>(seed % 9973u) / 9973.f);
	else if (auto const named = climate::preset_from_name(Kind))
		preset = *named;
	else
		return false;
	auto const kept{Conditions.location};
	Conditions = climate::make(station, month, preset, seed);
	Conditions.location = kept;
	WriteLog(std::string{"Weather: "} + climate::preset_name(preset) + " of month " + std::to_string(month + 1) + " at " + station.name);
	return true;
}

// reads the body of a weather section (or the text the scripting interface gives) into Conditions
void parse_section(std::vector<std::string> const &Tokens, weather_conditions &Conditions, std::optional<float> *Elevation, float *Transition)
{
	std::size_t i{0};
	auto const next_number = [&](float &Value) {
		if (i < Tokens.size())
		{
			if (auto const value = number(Tokens[i]))
			{
				Value = *value;
				++i;
				return true;
			}
		}
		return false;
	};
	while (i < Tokens.size())
	{
		auto const keyword{lower(Tokens[i++])};
		if (keyword == "metar")
		{
			// the report is a single quoted token, or the rest of the tokens up to the next keyword of the section
			if (i < Tokens.size())
				parse_metar(strip_quotes(Tokens[i++]), Conditions);
		}
		else if (keyword == "visibility")
			next_number(Conditions.visibility);
		else if (keyword == "haze")
			next_number(Conditions.haze_height);
		else if (keyword == "groundfog")
		{
			next_number(Conditions.groundfog_thickness);
			next_number(Conditions.groundfog_visibility);
		}
		else if (keyword == "fognoise")
			next_number(Conditions.fog_noise);
		else if (keyword == "noclouds" || keyword == "clear")
			Conditions.clouds.clear();
		else if (keyword == "clouds")
		{
			if (i >= Tokens.size())
				break;
			auto const type{cloud_type_from_name(Tokens[i++])};
			if (!type)
			{
				WriteLog("Weather: unknown cloud genus \"" + Tokens[i - 1] + "\"", logtype::file);
				continue;
			}
			auto const &genus{cloud_genus_info(*type)};
			float coverage{0.f};
			if (i < Tokens.size())
			{
				auto const amount{coverage_from_amount(upper(Tokens[i]))};
				if (amount >= 0.f)
				{
					coverage = amount;
					++i;
				}
				else if (next_number(coverage))
					coverage /= 8.f; // oktas
			}
			auto layer{typical_layer(*type, coverage, genus.base)};
			if (next_number(layer.base))
			{
				layer.top = layer.base + genus.thickness;
				next_number(layer.top);
				next_number(layer.density);
			}
			Conditions.clouds.push_back(layer);
		}
		else if (keyword == "precipitation")
		{
			if (i < Tokens.size())
			{
				auto const kind{lower(Tokens[i])};
				auto const known{kind == "none" || kind == "rain" || kind == "snow" || kind == "auto" || kind == "automatic"};
				if (known)
				{
					++i;
					Conditions.precipitation = (kind == "none") ? precipitation_type::none : (kind == "rain") ? precipitation_type::rain : (kind == "snow") ? precipitation_type::snow : precipitation_type::automatic;
				}
				Conditions.precipitation_intensity = (Conditions.precipitation == precipitation_type::none) ? 0.f : 0.5f;
				next_number(Conditions.precipitation_intensity);
			}
		}
		else if (keyword == "cloudwind")
		{
			next_number(Conditions.cloud_wind_direction);
			next_number(Conditions.cloud_wind_speed);
		}
		else if (keyword == "wind")
		{
			// wind direction speed [gust]: at the ground
			auto direction{Conditions.wind_direction};
			auto speed{Conditions.wind_speed};
			auto gust{0.f};
			if (next_number(direction) && next_number(speed))
			{
				next_number(gust);
				describe_wind(Conditions, direction, speed, gust);
			}
		}
		else if (keyword == "temperature")
		{
			float temperature{0.f};
			if (next_number(temperature))
			{
				Conditions.temperature = temperature;
				// a single temperature holds all day
				Conditions.temperature_min.reset();
				Conditions.temperature_max.reset();
			}
		}
		else if (keyword == "temperaturerange")
		{
			// temperaturerange lowest highest: of the day
			float lowest{0.f};
			float highest{0.f};
			if (next_number(lowest) && next_number(highest))
			{
				Conditions.temperature_min = std::min(lowest, highest);
				Conditions.temperature_max = std::max(lowest, highest);
				Conditions.temperature = 0.5f * (lowest + highest);
			}
		}
		else if (keyword == "dewpoint")
		{
			float dewpoint{0.f};
			if (next_number(dewpoint))
				Conditions.dew_point = dewpoint;
		}
		else if (keyword == "humidity")
		{
			// relative humidity in percent, at the temperature given before it (or the one of the simulation)
			float humidity{0.f};
			if (next_number(humidity))
				Conditions.dew_point = psychrometry::dew_point(Conditions.temperature.value_or(Global.AirTemperature), std::clamp(humidity / 100.f, 0.01f, 1.f));
		}
		else if (keyword == "location")
		{
			float latitude{0.f};
			float longitude{0.f};
			if (next_number(latitude) && next_number(longitude))
				Conditions.location = glm::vec2{latitude, longitude};
		}
		else if (keyword == "climate")
		{
			// climate [kind]: the weather the place has in this month; without a kind an average day
			std::string kind{"typical"};
			if ((i < Tokens.size()) && (false == is_section_keyword(Tokens[i])) && (false == number(Tokens[i]).has_value()))
				kind = lower(Tokens[i++]);
			if (false == describe_climate(Conditions, kind))
				WriteLog("Weather: unknown kind of weather \"" + kind + "\"", logtype::file);
		}
		else if (keyword == "elevation")
		{
			float elevation{0.f};
			if (next_number(elevation) && (Elevation != nullptr))
				*Elevation = elevation;
		}
		else if (keyword == "seed")
		{
			float seed{0.f};
			if (next_number(seed))
				Conditions.seed = static_cast<std::uint32_t>(std::max(seed, 0.f));
		}
		else if (keyword == "snowcover")
		{
			// snowcover depth [drift [fresh [tracks]]]: the snow lying, m, how far it is worked into drifts and how loose it is, 0..1,
			// and the share of it lying on the tracks too, as before the first train went (without it the tracks are cleared)
			float depth{0.f};
			if (next_number(depth))
			{
				Conditions.snow_depth = std::max(depth, 0.f);
				next_number(Conditions.snow_drift);
				next_number(Conditions.snow_fresh);
				next_number(Conditions.snow_tracks);
			}
		}
		else if (keyword == "transition")
		{
			float transition{0.f};
			if (next_number(transition) && (Transition != nullptr))
				*Transition = std::max(transition, 0.f);
		}
		else
			WriteLog("Weather: unknown entry \"" + Tokens[i - 1] + "\"", logtype::file);
	}
	Conditions.normalize();
}

bool is_section_keyword(std::string const &Token)
{
	static std::array<char const *, 21> const keywords{"metar",     "visibility", "haze", "groundfog",  "fognoise",  "noclouds", "clear",            "clouds",   "precipitation", "cloudwind", "temperature",
	                                                   "elevation", "seed",       "transition", "snowcover", "wind",     "temperaturerange", "dewpoint", "humidity",      "location",  "climate"};
	auto const token{lower(Token)};
	return std::any_of(keywords.begin(), keywords.end(), [&](char const *Keyword) { return token == Keyword; });
}

} // namespace

cloud_genus const &cloud_genus_info(cloud_type const Type)
{
	auto const index{std::clamp(static_cast<int>(Type), 0, static_cast<int>(cloud_type::count) - 1)};
	return GENERA[index];
}

std::optional<cloud_type> cloud_type_from_name(std::string_view const Name)
{
	auto const name{lower(Name)};
	for (int i = 0; i < static_cast<int>(cloud_type::count); ++i)
	{
		if (name == GENERA[i].name || name == lower(GENERA[i].abbreviation))
			return static_cast<cloud_type>(i);
	}
	if (name == "toweringcumulus")
		return cloud_type::towering_cumulus;
	return std::nullopt;
}

float weather_conditions::overcast() const
{
	float clear{1.f};
	for (auto const &layer : clouds)
	{
		auto const &genus{cloud_genus_info(layer.type)};
		clear *= 1.f - std::clamp(layer.coverage * genus.opacity * std::min(layer.density, 1.5f), 0.f, 1.f);
	}
	if ((precipitation != precipitation_type::none) && (precipitation_intensity > 0.f))
		return 1.f + std::clamp(precipitation_intensity, 0.f, 1.f);
	return std::clamp(1.f - clear, 0.f, 1.f);
}

void weather_conditions::normalize()
{
	for (auto &layer : clouds)
	{
		layer.coverage = std::clamp(layer.coverage, 0.f, 1.f);
		layer.base = std::clamp(layer.base, 0.f, 15000.f);
		layer.top = std::clamp(layer.top, layer.base + 50.f, 20000.f);
		layer.density = std::clamp(layer.density, 0.f, 4.f);
	}
	clouds.erase(std::remove_if(clouds.begin(), clouds.end(), [](cloud_layer const &Layer) { return Layer.coverage <= 0.f; }), clouds.end());
	if (clouds.size() > max_cloud_layers)
	{
		// the ones which cover the most are kept
		std::stable_sort(clouds.begin(), clouds.end(), [](cloud_layer const &Left, cloud_layer const &Right) { return Left.coverage > Right.coverage; });
		clouds.resize(max_cloud_layers);
	}
	std::stable_sort(clouds.begin(), clouds.end(), [](cloud_layer const &Left, cloud_layer const &Right) { return Left.base < Right.base; });
	visibility = std::clamp(visibility, 10.f, 200000.f);
	haze_height = std::clamp(haze_height, 10.f, 10000.f);
	groundfog_thickness = std::clamp(groundfog_thickness, 0.f, 2000.f);
	groundfog_visibility = std::clamp(groundfog_visibility, 10.f, 100000.f);
	fog_noise = std::clamp(fog_noise, 0.f, 1.f);
	precipitation_intensity = std::clamp(precipitation_intensity, 0.f, 1.f);
	cloud_wind_direction = std::fmod(std::fmod(cloud_wind_direction, 360.f) + 360.f, 360.f);
	cloud_wind_speed = std::clamp(cloud_wind_speed, 0.f, 100.f);
}

bool parse_metar(std::string_view const Report, weather_conditions &Conditions)
{
	// underscores may stand for the spaces too, as in the text of a memory cell
	auto report{upper(Report)};
	std::replace(report.begin(), report.end(), '_', ' ');
	auto const tokens{split(report)};
	bool understood{false};
	bool cloudsgiven{false};
	std::vector<cloud_layer> clouds;
	struct report_layer
	{
		float coverage;
		float base;
		std::string kind;
	};
	std::vector<report_layer> layers;
	std::optional<float> visibility;
	bool fog{false}, shallowfog{false}, patchyfog{false}, mist{false}, haze{false}, showers{false}, thunder{false};
	std::optional<float> verticalvisibility;
	precipitation_type precipitation{precipitation_type::none};
	float intensity{0.f};

	for (std::size_t i = 0; i < tokens.size(); ++i)
	{
		auto token{tokens[i]};
		if (token == "TEMPO" || token == "BECMG" || token == "RMK" || token == "NOSIG" || token == "TREND")
			break; // forecasts and remarks
		if (token == "METAR" || token == "SPECI" || token == "AUTO" || token == "COR" || token == "NIL")
			continue;
		// wind: dddssKT, dddssGggKT, VRBssKT, MPS or KMH
		{
			auto const unit{token.ends_with("KT") ? 2 : (token.ends_with("MPS") || token.ends_with("KMH")) ? 3 : 0};
			if (unit > 0 && token.size() >= 5 + static_cast<std::size_t>(unit))
			{
				auto const body{token.substr(0, token.size() - unit)};
				auto const direction{body.substr(0, 3)};
				auto speedtext{body.substr(3)};
				std::string gusttext;
				if (auto const gust = speedtext.find('G'); gust != std::string::npos)
				{
					gusttext = speedtext.substr(gust + 1);
					speedtext = speedtext.substr(0, gust);
				}
				if ((all_digits(direction) || direction == "VRB") && all_digits(speedtext))
				{
					auto const scale{token.ends_with("KT") ? KNOTS : token.ends_with("KMH") ? (1.f / 3.6f) : 1.f};
					auto const speed{static_cast<float>(std::stoi(speedtext)) * scale};
					auto const gust{all_digits(gusttext) ? static_cast<float>(std::stoi(gusttext)) * scale : 0.f};
					// the wind at the ground, a variable one keeping its direction; the wind at the clouds is stronger
					describe_wind(Conditions, direction != "VRB" ? static_cast<float>(std::stoi(direction)) : Conditions.wind_direction, speed, gust);
					understood = true;
					continue;
				}
			}
		}
		// variable wind direction range, runway visual range, pressure
		if ((token.size() == 7 && token[3] == 'V' && all_digits(token.substr(0, 3)) && all_digits(token.substr(4))) || (token.size() > 1 && token[0] == 'R' && token.find('/') != std::string::npos) ||
		    (token.size() == 5 && (token[0] == 'Q' || token[0] == 'A') && all_digits(token.substr(1))))
			continue;
		// station and time
		if ((token.size() == 4 && std::all_of(token.begin(), token.end(), [](char c) { return std::isalpha(static_cast<unsigned char>(c)) != 0; }) && i == 0) ||
		    (token.size() == 7 && token.back() == 'Z' && all_digits(token.substr(0, 6))))
			continue;
		if (token == "CAVOK")
		{
			visibility = 15000.f;
			cloudsgiven = true;
			understood = true;
			continue;
		}
		// visibility in metres, 9999 being 10 km or more
		if (token.size() == 4 && all_digits(token))
		{
			auto const value{static_cast<float>(std::stoi(token))};
			visibility = (value >= 9999.f) ? 15000.f : std::max(value, 50.f);
			understood = true;
			continue;
		}
		if (token.size() > 4 && all_digits(token.substr(0, 4)) && !all_digits(token.substr(4)) && token.substr(4).find_first_not_of("NESW") == std::string::npos)
			continue; // lowest visibility in a direction
		// visibility in statute miles: P6SM, 10SM, 1/2SM, 1 1/2SM
		if (token.ends_with("SM"))
		{
			auto body{token.substr(0, token.size() - 2)};
			float miles{-1.f};
			if (!body.empty() && (body[0] == 'P' || body[0] == 'M'))
				body.erase(0, 1);
			if (auto const slash = body.find('/'); slash != std::string::npos)
			{
				auto const numerator{body.substr(0, slash)}, denominator{body.substr(slash + 1)};
				if (all_digits(numerator) && all_digits(denominator) && std::stoi(denominator) > 0)
					miles = static_cast<float>(std::stoi(numerator)) / static_cast<float>(std::stoi(denominator));
				// a whole number before the fraction
				if (i > 0 && all_digits(tokens[i - 1]) && tokens[i - 1].size() <= 2 && visibility)
					miles += *visibility / 1609.34f;
			}
			else if (all_digits(body))
				miles = static_cast<float>(std::stoi(body));
			if (miles >= 0.f)
			{
				visibility = std::max(miles * 1609.34f, 50.f);
				understood = true;
			}
			continue;
		}
		if (all_digits(token) && token.size() <= 2 && i + 1 < tokens.size() && tokens[i + 1].ends_with("SM"))
		{
			// whole miles before a fraction, added by the fraction
			visibility = static_cast<float>(std::stoi(token)) * 1609.34f;
			continue;
		}
		// clouds
		if (token == "SKC" || token == "CLR" || token == "NSC" || token == "NCD")
		{
			cloudsgiven = true;
			understood = true;
			continue;
		}
		if (token.size() >= 6 && coverage_from_amount(token.substr(0, 3)) >= 0.f)
		{
			auto const height{token.substr(3, 3)};
			auto const kind{token.substr(6)};
			cloudsgiven = true;
			understood = true;
			if (all_digits(height))
				layers.push_back({coverage_from_amount(token.substr(0, 3)), static_cast<float>(std::stoi(height)) * 100.f * FEET, kind});
			continue;
		}
		if (token.size() == 5 && token.starts_with("VV"))
		{
			if (all_digits(token.substr(2)))
				verticalvisibility = static_cast<float>(std::stoi(token.substr(2))) * 100.f * FEET;
			understood = true;
			continue;
		}
		// temperature and dew point
		if (auto const slash = token.find('/'); slash != std::string::npos && slash > 0)
		{
			auto temperature{token.substr(0, slash)};
			auto const negative{temperature[0] == 'M'};
			if (negative)
				temperature.erase(0, 1);
			if (all_digits(temperature) && temperature.size() <= 2)
			{
				Conditions.temperature = static_cast<float>(std::stoi(temperature)) * (negative ? -1.f : 1.f);
				Conditions.temperature_min.reset();
				Conditions.temperature_max.reset();
				auto dewpoint{token.substr(slash + 1)};
				auto const below{(false == dewpoint.empty()) && (dewpoint[0] == 'M')};
				if (below)
					dewpoint.erase(0, 1);
				if (all_digits(dewpoint) && (dewpoint.size() <= 2))
					Conditions.dew_point = static_cast<float>(std::stoi(dewpoint)) * (below ? -1.f : 1.f);
				understood = true;
				continue;
			}
		}
		// present weather: intensity, descriptor, phenomena
		{
			std::string_view body{token};
			float strength{0.6f};
			if (body.starts_with('-'))
			{
				strength = 0.3f;
				body.remove_prefix(1);
			}
			else if (body.starts_with('+'))
			{
				strength = 1.f;
				body.remove_prefix(1);
			}
			else if (body.starts_with("VC"))
			{
				// in the vicinity only
				continue;
			}
			bool recognized{false};
			bool shallow{false}, patches{false}, freezing{false};
			while (body.size() >= 2)
			{
				auto const code{body.substr(0, 2)};
				body.remove_prefix(2);
				if (code == "MI") { shallow = true; recognized = true; }
				else if (code == "BC" || code == "PR") { patches = true; recognized = true; }
				else if (code == "SH") { showers = true; recognized = true; }
				else if (code == "TS") { thunder = true; showers = true; recognized = true; }
				else if (code == "FZ") { freezing = true; recognized = true; }
				else if (code == "DR" || code == "BL") { recognized = true; }
				else if (code == "RA" || code == "DZ" || code == "UP")
				{
					if (precipitation != precipitation_type::snow)
						precipitation = precipitation_type::rain;
					intensity = std::max(intensity, strength * (code == "DZ" ? 0.4f : 1.f));
					recognized = true;
				}
				else if (code == "SN" || code == "SG" || code == "PL" || code == "IC" || code == "GS" || code == "GR")
				{
					precipitation = precipitation_type::snow;
					intensity = std::max(intensity, strength);
					recognized = true;
				}
				else if (code == "FG") { fog = true; shallowfog = shallowfog || shallow; patchyfog = patchyfog || patches; recognized = true; }
				else if (code == "BR") { mist = true; recognized = true; }
				else if (code == "HZ" || code == "FU" || code == "DU" || code == "SA" || code == "VA") { haze = true; recognized = true; }
				else
				{
					recognized = false;
					break;
				}
			}
			(void)freezing;
			if (recognized)
			{
				understood = true;
				continue;
			}
		}
	}
	if (!understood)
		return false;

	if (visibility)
		Conditions.visibility = *visibility;
	Conditions.groundfog_thickness = 0.f;
	if (fog || verticalvisibility)
	{
		// the reported visibility is the one in the fog; above it the air is clearer
		auto const infog{visibility.value_or(500.f)};
		Conditions.groundfog_visibility = infog;
		Conditions.groundfog_thickness = verticalvisibility ? std::max(*verticalvisibility, 30.f) : shallowfog ? 8.f : 120.f;
		Conditions.visibility = std::max(infog * 6.f, 4000.f);
		Conditions.fog_noise = patchyfog ? 0.8f : shallowfog ? 0.5f : 0.3f;
	}
	else if (mist || haze)
	{
		Conditions.haze_height = mist ? 300.f : 1200.f;
		Conditions.fog_noise = mist ? 0.4f : 0.2f;
	}
	else
		Conditions.haze_height = 800.f;
	if (precipitation != precipitation_type::none)
	{
		Conditions.precipitation = precipitation;
		Conditions.precipitation_intensity = std::clamp(intensity, 0.1f, 1.f);
	}
	else
	{
		Conditions.precipitation = precipitation_type::none;
		Conditions.precipitation_intensity = 0.f;
	}
	if (cloudsgiven || verticalvisibility)
	{
		auto const precipitating{precipitation != precipitation_type::none};
		for (std::size_t l = 0; l < layers.size(); ++l)
		{
			auto const &layer{layers[l]};
			cloud_type type;
			if (layer.kind == "CB")
				type = cloud_type::cumulonimbus;
			else if (layer.kind == "TCU")
				type = cloud_type::towering_cumulus;
			else
				type = report_genus(layer.base, layer.coverage, precipitating && l == 0, showers);
			auto cloud{typical_layer(type, layer.coverage, layer.base)};
			// a layer doesn't reach into the next one reported above it, apart from the towers
			if ((l + 1 < layers.size()) && (type != cloud_type::cumulonimbus) && (type != cloud_type::towering_cumulus))
				cloud.top = std::max(cloud.base + 100.f, std::min(cloud.top, layers[l + 1].base - 100.f));
			clouds.push_back(cloud);
		}
		if (thunder && std::none_of(clouds.begin(), clouds.end(), [](cloud_layer const &Layer) { return Layer.type == cloud_type::cumulonimbus; }))
			clouds.push_back(typical_layer(cloud_type::cumulonimbus, 0.3f, 900.f));
		Conditions.clouds = clouds;
	}
	Conditions.normalize();
	return true;
}

std::string to_metar(weather_conditions const &Conditions)
{
	std::ostringstream report;
	report.imbue(std::locale::classic());
	// the wind at the ground: the described one, or the one the clouds go with, slowed down
	auto const direction{Conditions.wind_described ? Conditions.wind_direction : Conditions.cloud_wind_direction};
	auto const knots{static_cast<int>(std::round((Conditions.wind_described ? Conditions.wind_speed : Conditions.cloud_wind_speed / 1.5f) / KNOTS))};
	auto const gust{static_cast<int>(std::round(Conditions.wind_gust / KNOTS))};
	report << std::setfill('0') << std::setw(3) << static_cast<int>(std::round(direction / 10.f) * 10.f) % 360 << std::setw(2) << std::min(knots, 99);
	if (Conditions.wind_described && (gust >= knots + 5))
		report << 'G' << std::setw(2) << std::min(gust, 99);
	report << "KT ";
	auto const fogged{Conditions.groundfog_thickness > 0.f};
	auto const visibility{fogged ? Conditions.groundfog_visibility : Conditions.visibility};
	if (visibility >= 10000.f)
		report << "9999";
	else
		report << std::setw(4) << static_cast<int>(std::round(visibility / 50.f) * 50.f);
	if (Conditions.precipitation_intensity > 0.f && Conditions.precipitation != precipitation_type::none)
	{
		report << ' ' << (Conditions.precipitation_intensity < 0.4f ? "-" : Conditions.precipitation_intensity > 0.8f ? "+" : "");
		std::optional<bool> rain;
		if (Conditions.precipitation != precipitation_type::automatic)
			rain = (Conditions.precipitation == precipitation_type::rain);
		std::optional<float> wetbulb;
		if (Conditions.dew_point)
			wetbulb = psychrometry::wet_bulb(Global.AirTemperature, psychrometry::relative_humidity(Global.AirTemperature, *Conditions.dew_point));
		auto const kind{psychrometry::falling_kind(Global.AirTemperature, wetbulb, rain)};
		report << (kind == psychrometry::falling::freezing_rain ? "FZRA" : kind == psychrometry::falling::wet_snow ? "RASN" : psychrometry::is_snow(kind) ? "SN" : "RA");
	}
	if (fogged)
		report << ' ' << (Conditions.groundfog_thickness < 30.f ? "MIFG" : Conditions.groundfog_visibility < 1000.f ? "FG" : "BR");
	else if (Conditions.visibility < 5000.f)
		report << " BR";
	bool any{false};
	for (auto const &layer : Conditions.clouds)
	{
		if (layer.coverage < 0.03f)
			continue;
		auto const amount{layer.coverage <= 0.25f ? "FEW" : layer.coverage <= 0.5f ? "SCT" : layer.coverage < 0.95f ? "BKN" : "OVC"};
		report << ' ' << amount << std::setw(3) << std::clamp(static_cast<int>(std::round(layer.base / FEET / 100.f)), 0, 999);
		if (layer.type == cloud_type::cumulonimbus)
			report << "CB";
		else if (layer.type == cloud_type::towering_cumulus)
			report << "TCU";
		any = true;
	}
	if (!any)
		report << " NSC";
	if (Conditions.temperature)
	{
		auto const temperature{static_cast<int>(std::round(*Conditions.temperature))};
		report << ' ' << (temperature < 0 ? "M" : "") << std::setw(2) << std::abs(temperature) << '/';
		if (Conditions.dew_point)
		{
			auto const dewpoint{static_cast<int>(std::round(std::min(*Conditions.dew_point, *Conditions.temperature)))};
			report << (dewpoint < 0 ? "M" : "") << std::setw(2) << std::abs(dewpoint);
		}
	}
	return report.str();
}

std::vector<cloud_layer> clouds_from_overcast(float const Overcast)
{
	auto const overcast{std::clamp(Overcast, 0.f, 2.f)};
	auto const sky{std::min(overcast, 1.f)};
	std::vector<cloud_layer> clouds;
	auto const add = [&](cloud_type const Type, float const Coverage, float const Base, float const Top, float const Density = 1.f) {
		clouds.push_back(cloud_layer{Type, Coverage, Base, Top, Density});
	};
	// the high veil is there on most days, hidden once the low clouds close up
	if (sky < 0.95f)
		add(cloud_type::cirrus, 0.15f + 0.3f * sky, 8200.f, 9000.f);
	if (sky <= 0.5f)
		// fair weather cumulus, growing taller with the cover
		add(cloud_type::cumulus, std::max(0.05f, sky * 0.9f), 1200.f, 2000.f + 1600.f * sky);
	else if (sky < 0.9f)
	{
		add(cloud_type::stratocumulus, 0.45f + (sky - 0.5f) * 1.2f, 1000.f, 1700.f);
		add(cloud_type::altocumulus, 0.3f, 3600.f, 4000.f);
	}
	else if (overcast <= 1.f)
	{
		add(cloud_type::stratocumulus, 0.95f + (sky - 0.9f) * 0.5f, 700.f, 1500.f);
		add(cloud_type::altostratus, 0.6f, 3500.f, 5000.f);
	}
	else
	{
		// rain or snow: a thick, low sheet, lower and denser the heavier it falls, with storms in the heaviest
		auto const precipitation{overcast - 1.f};
		add(cloud_type::nimbostratus, 1.f, 600.f - 300.f * precipitation, 3500.f, 1.f + 0.5f * precipitation);
		if (precipitation > 0.6f)
			add(cloud_type::cumulonimbus, 0.3f * (precipitation - 0.6f) / 0.4f, 600.f, 9000.f);
	}
	return clouds;
}

float visibility_from_fog_range(float const Fogend, float const Overcast)
{
	auto const range{Fogend / std::max(1.f, Overcast * 2.f)};
	return std::clamp(range * LEGACY_VISIBILITY_SCALE, 10.f, 200000.f);
}

float fog_range_from_visibility(float const Visibility, float const Overcast)
{
	return Visibility / LEGACY_VISIBILITY_SCALE * std::max(1.f, Overcast * 2.f);
}

void world_weather::reset()
{
	m_scenery.reset();
	m_scenerytokens.clear();
	m_surfacewind = glm::vec2{0.f};
	m_snowing = false;
	m_elevation.reset();
	m_described_clouds = false;
	m_described_fog = false;
	m_known_overcast = -1.f;
	m_known_fogend = -1.f;
	m_transition = 0.0;
	m_transition_time = 0.0;
	m_ground_set = false;
	m_ground = 0.f;
	m_cloud_offset = glm::dvec2{0.0};
	m_snow.reset();
	m_rails.reset();
	m_live.reset();
	m_livenext = 0.0;
	m_liveapplied = false;
	m_livestatus.clear();
	m_time = 0.0;
}

void world_weather::deserialize(cParser &Input)
{
	std::vector<std::string> tokens;
	auto token{Input.getToken<std::string>(false)};
	while (!token.empty() && lower(token) != "endweather")
	{
		tokens.push_back(token);
		token = Input.getToken<std::string>(false);
	}
	// a later section adds to an earlier one. they're read in init(), once the date of the simulation is known: the weather a
	// section takes from the climate depends on the month
	m_scenerytokens.insert(m_scenerytokens.end(), tokens.begin(), tokens.end());
}

void world_weather::init()
{
	m_transition = 0.0;
	m_transition_time = 0.0;
	// the weather of the settings over the one of the scenery (weather.mode): what they say is read after the section of the
	// scenery, so it wins where both say something, and a preset of the climate replaces it all but the place and the ground
	auto tokens{m_scenerytokens};
	if (Global.WeatherMode == global_settings::weather_mode::manual)
		tokens.insert(tokens.end(), Global.WeatherManual.begin(), Global.WeatherManual.end());
	else if (Global.WeatherMode == global_settings::weather_mode::automatic)
	{
		// the climate only where the place of the scenery is known (its location entry, or the place the editor keeps for it);
		// elsewhere the weather the starter set (atmo, the temperature and the date of the config of the scenery) stands, as
		// without the setting, rather than the climate of a stand-in station
		auto const located{std::ranges::any_of(m_scenerytokens, [](std::string const &Token) { return lower(Token) == "location"; })};
		if (located || editor_location().has_value())
		{
			tokens.emplace_back("climate");
			tokens.emplace_back("random");
		}
		else
			WriteLog("Weather: the place of the scenery isn't known, the weather of the starter stands (weather.mode auto)");
	}
	else if (Global.WeatherMode == global_settings::weather_mode::preset)
	{
		// a preset of the climate by its name, or one drawn as often as each comes in the month; the climate of the place where it's
		// known, of the stand-in station elsewhere
		auto preset{lower(Global.WeatherPreset)};
		if ((preset != "random") && (false == climate::preset_from_name(preset).has_value()))
		{
			WriteLog("Weather: unknown preset \"" + Global.WeatherPreset + "\" of weather.preset, one drawn instead");
			preset = "random";
		}
		tokens.emplace_back("climate");
		tokens.emplace_back(preset);
	}
	if (false == tokens.empty())
	{
		weather_conditions conditions;
		parse_section(tokens, conditions, &m_elevation, nullptr);
		m_scenery = conditions;
	}
	if (m_scenery)
	{
		// the description wins over the atmo section, wherever it was in the scenery
		m_target = *m_scenery;
		if (m_target.temperature)
			Global.AirTemperature = *m_target.temperature;
		m_described_clouds = true;
		m_described_fog = true;
		m_current = m_target;
		write_globals();
	}
	else
	{
		m_target = weather_conditions{};
		follow_overcast();
		follow_fog_range();
		m_current = m_target;
	}
	m_from = m_current;
	apply_snow();
	if (m_elevation)
	{
		m_ground = *m_elevation;
		m_ground_set = true;
	}
	// the live weather: the weather of the scenery stands until the first report comes, fetched at once
	m_live.reset();
	m_livenext = 0.0;
	m_liveapplied = false;
	m_livestatus.clear();
	if (Global.WeatherMode == global_settings::weather_mode::live)
	{
		if (false == georeference::scenery_origin().has_value())
		{
			m_livestatus = "the scenery isn't laid on the map: the weather of the scenery";
			WriteLog("Weather: the scenery isn't laid on the map (no //$g line, no place in the editor), the weather of the scenery stands (weather.mode live)");
		}
		else if (false == live_weather::available())
		{
			m_livestatus = "this build can't fetch anything: the weather of the scenery";
			WriteLog("Weather: no https in this build, the weather of the scenery stands (weather.mode live)");
		}
		else
			m_livestatus = "waiting for the first report";
	}
	// the rails as the weather of the start left them: wet in the rain, iced in a freezing rain, with the leaves of the autumn
	m_rails.reset();
	m_rails.settle(surface_weather(), rails_daylight(), rails_leaves());
	WriteLog("Weather: " + to_metar(m_current) + (m_scenery ? " (described)" : " (from the overcast and the fog range)"));
}

void world_weather::follow_overcast()
{
	m_target.clouds = clouds_from_overcast(Global.Overcast);
	m_target.precipitation = precipitation_type::automatic;
	m_target.precipitation_intensity = std::clamp(Global.Overcast - 1.f, 0.f, 1.f);
	m_target.normalize();
	m_known_overcast = Global.Overcast;
	m_described_clouds = false;
}

void world_weather::follow_fog_range()
{
	m_target.visibility = visibility_from_fog_range(Global.fFogEnd, Global.Overcast);
	m_target.groundfog_thickness = 0.f;
	m_target.haze_height = 800.f;
	m_target.normalize();
	m_known_fogend = Global.fFogEnd;
	m_described_fog = false;
}

void world_weather::write_globals()
{
	auto changed{false};
	if (m_described_clouds)
	{
		auto const overcast{m_current.overcast()};
		if (overcast != Global.Overcast)
		{
			Global.Overcast = overcast;
			changed = true;
		}
		m_known_overcast = Global.Overcast;
	}
	if (m_described_fog)
	{
		// the fog the old renderers draw, and the scripting interface reads
		auto const fogend{std::clamp(fog_range_from_visibility(std::min(m_current.visibility, m_current.groundfog_thickness > 0.f ? m_current.groundfog_visibility : m_current.visibility), Global.Overcast), 10.f, 50000.f)};
		Global.fFogEnd = fogend;
		m_known_fogend = Global.fFogEnd;
	}
	if (changed)
	{
		// the weather name, the turbidity and the precipitation follow the overcast. the name drives the precipitation renderer,
		// which picks it up by the change of the overcast
		simulation::Environment.compute_weather();
	}
}

void world_weather::update(double const Deltatime, glm::dvec3 const &Camera)
{
	update_live(Camera);
	if (!m_ground_set)
	{
		// the ground below the place the simulation starts at, for want of anything better: the camera is in the cab
		m_ground = static_cast<float>(Camera.y) - 3.f;
		m_ground_set = true;
	}
	// something set the old overcast or fog range: the scenery atmo, the sliders, a script, the network. tiny changes are the way
	// the precipitation is made to update after a change of the season, which keeps the description
	auto const overcastchanged{std::abs(Global.Overcast - m_known_overcast) > 1e-3f};
	if (overcastchanged)
	{
		follow_overcast();
		m_current.clouds = m_target.clouds;
		m_current.precipitation = m_target.precipitation;
		m_current.precipitation_intensity = m_target.precipitation_intensity;
		m_from = m_current;
	}
	else
		m_known_overcast = Global.Overcast;
	// the old fog thickened with the overcast, so it is followed with every change of either
	if ((std::abs(Global.fFogEnd - m_known_fogend) > 0.5f) || (false == m_described_fog && overcastchanged))
	{
		follow_fog_range();
		m_current.visibility = m_target.visibility;
		m_current.groundfog_thickness = m_target.groundfog_thickness;
		m_current.haze_height = m_target.haze_height;
		m_from = m_current;
	}

	// a change in progress
	if (m_transition > 0.0)
	{
		m_transition_time += Deltatime;
		auto const t{static_cast<float>(std::clamp(m_transition_time / m_transition, 0.0, 1.0))};
		auto mixed{m_target};
		auto const mix = [t](float From, float To) { return From + (To - From) * t; };
		mixed.visibility = std::exp(mix(std::log(m_from.visibility), std::log(m_target.visibility)));
		mixed.haze_height = mix(m_from.haze_height, m_target.haze_height);
		mixed.groundfog_thickness = mix(m_from.groundfog_thickness, m_target.groundfog_thickness);
		mixed.groundfog_visibility = std::exp(mix(std::log(m_from.groundfog_visibility), std::log(m_target.groundfog_visibility)));
		mixed.fog_noise = mix(m_from.fog_noise, m_target.fog_noise);
		auto const falling = [](weather_conditions const &Conditions) { return Conditions.precipitation == precipitation_type::none ? 0.f : Conditions.precipitation_intensity; };
		mixed.precipitation_intensity = mix(falling(m_from), falling(m_target));
		if (mixed.precipitation == precipitation_type::none && mixed.precipitation_intensity > 0.f)
			mixed.precipitation = m_from.precipitation;
		mixed.cloud_wind_speed = mix(m_from.cloud_wind_speed, m_target.cloud_wind_speed);
		auto turn{m_target.cloud_wind_direction - m_from.cloud_wind_direction};
		turn -= 360.f * std::round(turn / 360.f);
		mixed.cloud_wind_direction = m_from.cloud_wind_direction + turn * t;
		if (m_from.wind_described && m_target.wind_described)
		{
			mixed.wind_speed = mix(m_from.wind_speed, m_target.wind_speed);
			mixed.wind_gust = mix(m_from.wind_gust, m_target.wind_gust);
			auto veer{m_target.wind_direction - m_from.wind_direction};
			veer -= 360.f * std::round(veer / 360.f);
			mixed.wind_direction = m_from.wind_direction + veer * t;
		}
		// the air: from what it was when the change began
		if (m_target.dew_point)
			mixed.dew_point = mix(m_from.dew_point.value_or(*m_target.dew_point), *m_target.dew_point);
		if (m_target.temperature_min && m_target.temperature_max)
		{
			mixed.temperature_min = mix(m_from.temperature_min.value_or(m_from.temperature.value_or(*m_target.temperature_min)), *m_target.temperature_min);
			mixed.temperature_max = mix(m_from.temperature_max.value_or(m_from.temperature.value_or(*m_target.temperature_max)), *m_target.temperature_max);
		}
		// the same layers change their sizes; different ones fade out and in
		auto const same{(m_from.clouds.size() == m_target.clouds.size()) &&
		                std::equal(m_from.clouds.begin(), m_from.clouds.end(), m_target.clouds.begin(), [](cloud_layer const &Left, cloud_layer const &Right) { return Left.type == Right.type; })};
		if (same)
		{
			for (std::size_t i = 0; i < mixed.clouds.size(); ++i)
			{
				auto &layer{mixed.clouds[i]};
				auto const &from{m_from.clouds[i]};
				layer.coverage = mix(from.coverage, layer.coverage);
				layer.base = mix(from.base, layer.base);
				layer.top = mix(from.top, layer.top);
				layer.density = mix(from.density, layer.density);
			}
		}
		else if (t < 0.5f)
		{
			mixed.clouds = m_from.clouds;
			for (auto &layer : mixed.clouds)
				layer.coverage *= 1.f - 2.f * t;
		}
		else
		{
			for (auto &layer : mixed.clouds)
				layer.coverage *= 2.f * t - 1.f;
		}
		mixed.clouds.erase(std::remove_if(mixed.clouds.begin(), mixed.clouds.end(), [](cloud_layer const &Layer) { return Layer.coverage <= 0.f; }), mixed.clouds.end());
		m_current = mixed;
		if (t >= 1.f)
		{
			m_transition = 0.0;
			m_current = m_target;
		}
	}
	else
		m_current = m_target;

	write_globals();

	// the clouds go with the wind
	m_time += Deltatime;
	m_cloud_offset += glm::dvec2{cloud_velocity()} * Deltatime;
	// kept small, the shaders take it in single precision; the noise of the clouds repeats long before this
	auto const period{1024.0 * 1024.0};
	m_cloud_offset = glm::mod(m_cloud_offset, glm::dvec2{period});

	update_air();
	update_snow(Deltatime);
}

glm::vec2 world_weather::ground_wind() const
{
	if (false == m_current.wind_described)
		return cloud_velocity() / 1.5f;
	// where the wind goes to, as cloud_velocity() has it
	auto const to{glm::radians(m_current.wind_direction + 180.f)};
	return glm::vec2{-std::sin(to), std::cos(to)} * m_current.wind_speed;
}

float world_weather::dew_point() const
{
	if (m_current.dew_point)
		return std::min(*m_current.dew_point, Global.AirTemperature);
	return psychrometry::dew_point(Global.AirTemperature, humidity());
}

float world_weather::humidity() const
{
	if (m_current.dew_point)
		return psychrometry::relative_humidity(Global.AirTemperature, *m_current.dew_point);
	// what the weather is likely to have: saturated in a fog, nearly so while something falls, drier the clearer the sky
	if ((m_current.groundfog_thickness > 0.f) || (m_current.visibility < 1000.f))
		return 0.98f;
	if (Global.Overcast > 1.f)
		return 0.92f;
	return 0.55f + 0.3f * std::clamp(Global.Overcast, 0.f, 1.f);
}

float world_weather::wet_bulb() const
{
	return psychrometry::wet_bulb(Global.AirTemperature, humidity());
}

psychrometry::falling world_weather::falling() const
{
	if (Global.Overcast <= 1.f)
		return psychrometry::falling::nothing;
	std::optional<bool> rain;
	if (auto const kind{precipitation_override()})
		rain = (*kind == precipitation_type::rain);
	// without a described moisture the air temperature decides, as it always did
	std::optional<float> wetbulb;
	if (m_current.dew_point)
		wetbulb = wet_bulb();
	return psychrometry::falling_kind(Global.AirTemperature, wetbulb, rain);
}

std::optional<glm::vec2> world_weather::location() const
{
	return m_current.location ? m_current.location : editor_location();
}

void world_weather::update_air()
{
	if (m_current.temperature_min && m_current.temperature_max)
	{
		auto const &time{simulation::Time.data()};
		auto const hour{static_cast<float>(time.wHour) + static_cast<float>(time.wMinute) / 60.f + static_cast<float>(time.wSecond) / 3600.f};
		Global.AirTemperature = psychrometry::diurnal_temperature(*m_current.temperature_min, *m_current.temperature_max, hour);
	}
	// the weather name tells the snow from the rain; it follows the air as the day goes
	if (auto const snowing{psychrometry::is_snow(falling())}; snowing != m_snowing)
	{
		m_snowing = snowing;
		simulation::Environment.compute_weather();
	}
}

void world_weather::apply_snow()
{
	if (!m_target.snow_depth)
		return;
	m_snow.set(*m_target.snow_depth, m_target.snow_drift, ground_wind(), m_target.snow_fresh, m_target.snow_tracks);
	WriteLog("Weather: snow cover " + std::to_string(*m_target.snow_depth) + " m");
	// applied once; a later change of the weather keeps the snow as it lies then
	m_target.snow_depth.reset();
	m_current.snow_depth.reset();
}

std::optional<glm::dvec2> world_weather::live_place(glm::dvec3 const &Camera)
{
	auto const origin{georeference::scenery_origin()};
	if (false == origin.has_value())
		return std::nullopt;
	// the vehicle driven, else the camera
	auto const *vehicle{(simulation::Train != nullptr) ? simulation::Train->Dynamic() : nullptr};
	return georeference::to_degrees(*origin, vehicle != nullptr ? vehicle->GetPosition() : Camera);
}

void world_weather::update_live(glm::dvec3 const &Camera)
{
	if ((Global.WeatherMode != global_settings::weather_mode::live) || EditorModeFlag || (false == live_weather::available()))
		return;
	if (auto const report{m_live.take()})
	{
		if (false == report->error.empty())
		{
			m_livestatus = "the fetch failed: " + report->error;
			WriteLog("Weather: live weather of " + std::to_string(report->place.x) + ", " + std::to_string(report->place.y) + " not fetched: " + report->error);
			// tried again in a minute
			m_livenext = std::min(m_livenext, m_time + 60.0);
		}
		else
		{
			// the report read as any other; the place kept for the climate and the sun
			auto conditions{m_target};
			auto const place{conditions.location};
			conditions = weather_conditions{};
			conditions.location = place;
			if (parse_metar(report->metar, conditions))
			{
				// the first report comes with the snow lying there and goes at once; the next ones change the weather over two minutes
				// and leave the snow as the simulation keeps it
				if (false == m_liveapplied)
					conditions.snow_depth = report->snow_depth;
				apply(conditions, m_liveapplied ? 120.f : 0.f, m_liveapplied);
				m_liveapplied = true;
				char place_text[64];
				std::snprintf(place_text, sizeof(place_text), "%.4f, %.4f", report->place.x, report->place.y);
				m_livestatus = std::string{"Open-Meteo at "} + place_text + ": " + report->metar;
				WriteLog("Weather: live weather of " + std::string{place_text} + " (Open-Meteo): " + report->metar);
			}
			else
				m_livestatus = "the report wasn't understood: " + report->metar;
		}
	}
	if ((m_time < m_livenext) || m_live.busy())
		return;
	auto const place{live_place(Camera)};
	if (false == place.has_value())
		return;
	m_live.request(*place);
	m_livenext = m_time + std::clamp(static_cast<double>(Global.WeatherLiveInterval), 60.0, 300.0);
}

snow_weather world_weather::surface_weather() const
{
	snow_weather weather;
	weather.temperature = Global.AirTemperature;
	weather.humidity = humidity();
	weather.wet_bulb = psychrometry::wet_bulb(weather.temperature, weather.humidity);
	weather.wind = m_surfacewind;
	// what falls: the overcast past 1 is the precipitation (weather_conditions::overcast()), as the renderers draw it
	auto const intensity{std::clamp(Global.Overcast - 1.f, 0.f, 1.f)};
	auto const snow{psychrometry::is_snow(falling())};
	weather.snowfall = (snow ? intensity : 0.f);
	weather.rainfall = (snow ? 0.f : intensity);
	// a fog thick enough to leave rime on the wires
	weather.fog = std::clamp(1.f - m_current.visibility / 1000.f, 0.f, 1.f);
	return weather;
}

float world_weather::rails_daylight()
{
	// the height of the sun, degrees: dark at the end of the civil twilight, full day a little over the horizon
	auto const angle{simulation::Environment.sun().getAngle()};
	return std::clamp((angle + 6.f) / 16.f, 0.f, 1.f);
}

float world_weather::rails_leaves()
{
	return rail_surface::leaf_season(simulation::Time.year_day()) * std::max(Global.physics_weather.leaves, 0.f);
}

void world_weather::update_snow(double const Deltatime)
{
	auto const weather{surface_weather()};
	// the kinds of the ice on the wires the settings let the weather lay (physics.weather.glaze, .rimehard, .rimesoft, .wetsnow)
	auto const &settings{Global.physics_weather};
	m_snow.wire_ice_kinds[static_cast<int>(snow_cover::ice_kind::glaze)] = settings.glaze;
	m_snow.wire_ice_kinds[static_cast<int>(snow_cover::ice_kind::rime_hard)] = settings.rime_hard;
	m_snow.wire_ice_kinds[static_cast<int>(snow_cover::ice_kind::rime_soft)] = settings.rime_soft;
	m_snow.wire_ice_kinds[static_cast<int>(snow_cover::ice_kind::wet_snow)] = settings.wet_snow;
	m_snow.update(Deltatime, weather);
	m_rails.time_scale = m_snow.time_scale;
	m_rails.update(Deltatime, weather, rails_daylight(), rails_leaves());
}

glm::vec2 world_weather::cloud_velocity() const
{
	// where the wind goes to, from the north clockwise. the world has the north at +z and the west at +x
	auto const to{glm::radians(m_current.cloud_wind_direction + 180.f)};
	return glm::vec2{-std::sin(to), std::cos(to)} * m_current.cloud_wind_speed;
}

std::optional<precipitation_type> world_weather::precipitation_override() const
{
	if (!m_described_clouds)
		return std::nullopt;
	if (m_current.precipitation == precipitation_type::rain || m_current.precipitation == precipitation_type::snow)
		return m_current.precipitation;
	return std::nullopt;
}

void world_weather::apply(weather_conditions const &Conditions, float const Transition, bool const Quiet)
{
	m_from = m_current;
	m_target = Conditions;
	m_target.normalize();
	if (m_target.temperature)
		Global.AirTemperature = *m_target.temperature;
	m_described_clouds = true;
	m_described_fog = true;
	m_transition = std::max(Transition, 0.f);
	m_transition_time = 0.0;
	if (m_transition <= 0.0)
		m_current = m_target;
	apply_snow();
	write_globals();
	if (false == Quiet)
		WriteLog("Weather: changed to " + to_metar(m_target) + (m_transition > 0.0 ? " over " + std::to_string(static_cast<int>(m_transition)) + " s" : ""));
}

bool world_weather::apply_text(std::string_view const Text, float const Transition)
{
	auto tokens{split(Text)};
	if (tokens.empty())
		return false;
	auto conditions{m_target};
	auto transition{Transition};
	if (is_section_keyword(tokens.front()))
	{
		// the quoted report of a section given as a text is split here, glue it back up to the next keyword
		std::vector<std::string> glued;
		for (std::size_t i = 0; i < tokens.size(); ++i)
		{
			glued.push_back(tokens[i]);
			if (lower(tokens[i]) == "metar")
			{
				std::string report;
				while (i + 1 < tokens.size() && !is_section_keyword(tokens[i + 1]))
					report += (report.empty() ? "" : " ") + strip_quotes(tokens[++i]);
				glued.push_back(report);
			}
		}
		parse_section(glued, conditions, nullptr, &transition);
	}
	else if (!parse_metar(Text, conditions))
		return false;
	apply(conditions, transition);
	return true;
}
