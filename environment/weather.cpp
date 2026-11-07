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

#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/parser.h"
#include "simulation/simulationenvironment.h"

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
		else if (keyword == "temperature")
		{
			float temperature{0.f};
			if (next_number(temperature))
				Conditions.temperature = temperature;
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
	static std::array<char const *, 14> const keywords{"metar", "visibility", "haze", "groundfog", "fognoise", "noclouds", "clear", "clouds", "precipitation", "cloudwind", "temperature", "elevation", "seed", "transition"};
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
				if (auto const gust = speedtext.find('G'); gust != std::string::npos)
					speedtext = speedtext.substr(0, gust);
				if ((all_digits(direction) || direction == "VRB") && all_digits(speedtext))
				{
					auto speed{static_cast<float>(std::stoi(speedtext))};
					speed *= token.ends_with("KT") ? KNOTS : token.ends_with("KMH") ? (1.f / 3.6f) : 1.f;
					if (direction != "VRB")
						Conditions.cloud_wind_direction = static_cast<float>(std::stoi(direction));
					// the wind at the clouds is stronger than at the ground
					Conditions.cloud_wind_speed = std::max(speed * 1.5f, 2.f);
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
	auto const knots{static_cast<int>(std::round(Conditions.cloud_wind_speed / 1.5f / KNOTS))};
	report << std::setfill('0') << std::setw(3) << static_cast<int>(std::round(Conditions.cloud_wind_direction / 10.f) * 10.f) % 360 << std::setw(2) << std::min(knots, 99) << "KT ";
	auto const fogged{Conditions.groundfog_thickness > 0.f};
	auto const visibility{fogged ? Conditions.groundfog_visibility : Conditions.visibility};
	if (visibility >= 10000.f)
		report << "9999";
	else
		report << std::setw(4) << static_cast<int>(std::round(visibility / 50.f) * 50.f);
	if (Conditions.precipitation_intensity > 0.f && Conditions.precipitation != precipitation_type::none)
	{
		report << ' ' << (Conditions.precipitation_intensity < 0.4f ? "-" : Conditions.precipitation_intensity > 0.8f ? "+" : "");
		auto const snow{(Conditions.precipitation == precipitation_type::snow) || ((Conditions.precipitation == precipitation_type::automatic) && (Global.AirTemperature <= 1.f))};
		report << (snow ? "SN" : "RA");
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
	// a later section adds to an earlier one
	auto conditions{m_scenery.value_or(weather_conditions{})};
	parse_section(tokens, conditions, &m_elevation, nullptr);
	m_scenery = conditions;
}

void world_weather::init()
{
	m_transition = 0.0;
	m_transition_time = 0.0;
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
	if (m_elevation)
	{
		m_ground = *m_elevation;
		m_ground_set = true;
	}
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
