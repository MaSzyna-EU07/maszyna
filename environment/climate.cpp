/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "environment/climate.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "environment/psychrometry.h"
#include "environment/weather.h"

namespace climate
{

namespace
{

constexpr std::array<char const *, static_cast<std::size_t>(preset::count)> PRESET_NAMES{"typical", "fair", "overcast", "precipitation", "showers", "storm",
                                                                                         "fog",     "frost", "thaw",     "blizzard",      "gale",    "heat"};
constexpr std::array<float, 12> MONTH_DAYS{31.f, 28.25f, 31.f, 30.f, 31.f, 30.f, 31.f, 31.f, 30.f, 31.f, 30.f, 31.f};
constexpr std::array<char const *, 4> SEASON_NAMES{"winter", "spring", "summer", "autumn"};

// Lodz-Lublinek, the lowlands in the middle of Poland, away from the sea and the mountains. the lowest and highest temperatures
// of the day are the 1991-2020 normals; the sums and the days of the precipitation are rounded ones, the humidity, the wind, the
// snow, the fog, the storms, the cloudiness and the wind roses are typical values of the lowlands of central Poland, to be replaced
// by the table tools/weather/imgw_normals.py makes of the measurements
station make_default()
{
	station result;
	result.name = "Lodz-Lublinek";
	result.latitude = 51.72f;
	result.longitude = 19.40f;
	result.elevation = 187.f;
	constexpr std::array<std::array<float, 11>, 12> months{{
	    // tmin, tmax, humidity, wind, precipitation, its days, snow days, snow depth, fog days, storm days, cloudiness
	    {-4.0f, 1.2f, 87.f, 4.2f, 30.f, 8.f, 16.f, 10.f, 5.f, 0.f, 6.3f},
	    {-3.3f, 2.9f, 84.f, 4.1f, 30.f, 7.f, 14.f, 10.f, 4.f, 0.f, 6.0f},
	    {-0.7f, 7.4f, 77.f, 4.0f, 35.f, 8.f, 6.f, 6.f, 3.f, 0.3f, 5.5f},
	    {3.6f, 14.4f, 68.f, 3.5f, 40.f, 7.f, 1.f, 1.f, 2.f, 1.f, 4.9f},
	    {8.2f, 19.4f, 69.f, 3.2f, 65.f, 10.f, 0.f, 0.f, 2.f, 4.f, 4.9f},
	    {11.6f, 22.7f, 70.f, 3.0f, 75.f, 9.f, 0.f, 0.f, 1.f, 5.f, 4.9f},
	    {13.6f, 24.9f, 70.f, 3.0f, 90.f, 11.f, 0.f, 0.f, 1.f, 6.f, 4.8f},
	    {13.3f, 24.6f, 71.f, 2.8f, 60.f, 8.f, 0.f, 0.f, 2.f, 4.f, 4.5f},
	    {9.0f, 19.1f, 78.f, 3.1f, 50.f, 8.f, 0.f, 0.f, 4.f, 1.5f, 4.9f},
	    {5.0f, 13.0f, 83.f, 3.5f, 40.f, 8.f, 0.f, 0.f, 6.f, 0.2f, 5.4f},
	    {1.3f, 6.8f, 88.f, 3.9f, 35.f, 8.f, 3.f, 3.f, 6.f, 0.f, 6.4f},
	    {-2.4f, 2.4f, 89.f, 4.1f, 30.f, 8.f, 10.f, 7.f, 5.f, 0.f, 6.5f},
	}};
	for (std::size_t i = 0; i < months.size(); ++i)
	{
		auto const &source{months[i]};
		auto &target{result.months[i]};
		target.temperature_min = source[0];
		target.temperature_max = source[1];
		target.humidity = source[2];
		target.wind_speed = source[3];
		target.precipitation = source[4];
		target.precipitation_days = source[5];
		target.snow_days = source[6];
		target.snow_depth = source[7];
		target.fog_days = source[8];
		target.storm_days = source[9];
		target.cloudiness = source[10];
	}
	// from N, NE, E, SE, S, SW, W, NW
	result.roses = {{
	    {0.06f, 0.06f, 0.11f, 0.12f, 0.12f, 0.19f, 0.24f, 0.10f},
	    {0.10f, 0.10f, 0.13f, 0.11f, 0.09f, 0.13f, 0.20f, 0.14f},
	    {0.10f, 0.07f, 0.08f, 0.08f, 0.08f, 0.15f, 0.27f, 0.17f},
	    {0.05f, 0.05f, 0.11f, 0.14f, 0.14f, 0.20f, 0.22f, 0.09f},
	}};
	return result;
}

int month_index(int const Month)
{
	return ((Month % 12) + 12) % 12;
}

// winter 0 (December to February), spring 1, summer 2, autumn 3
int season_of(int const Month)
{
	return ((month_index(Month) + 1) % 12) / 3;
}

// a number 0..1 made of a seed and the place of the number in the weather
float hashed(std::uint32_t const Seed, std::uint32_t const Salt)
{
	auto value{Seed * 0x9E3779B9u + Salt * 0x85EBCA6Bu + 0x27D4EB2Fu};
	value ^= value >> 15;
	value *= 0x2C1B3C6Du;
	value ^= value >> 12;
	value *= 0x297A2D39u;
	value ^= value >> 15;
	return static_cast<float>(value & 0xFFFFFFu) / static_cast<float>(0x1000000u);
}

// where the wind blows from, degrees, drawn from a rose
float rose_direction(wind_rose const &Rose, float const Random, float const Jitter)
{
	auto total{0.f};
	for (auto const share : Rose)
		total += std::max(share, 0.f);
	auto sector{6}; // from the west, when the rose is empty
	if (total > 0.f)
	{
		auto left{Random * total};
		for (int i = 0; i < 8; ++i)
		{
			left -= std::max(Rose[static_cast<std::size_t>(i)], 0.f);
			if (left <= 0.f)
			{
				sector = i;
				break;
			}
		}
	}
	auto const direction{static_cast<float>(sector) * 45.f + (Jitter - 0.5f) * 40.f};
	return std::fmod(direction + 360.f, 360.f);
}

cloud_layer layer(cloud_type const Type, float const Coverage, float const Base, float const Top, float const Density = 1.f)
{
	cloud_layer result;
	result.type = Type;
	result.coverage = std::clamp(Coverage, 0.f, 1.f);
	result.base = Base;
	result.top = Top;
	result.density = Density;
	return result;
}

// the weather being put together
struct draft
{
	float temperature_min{0.f};
	float temperature_max{0.f};
	float humidity{0.8f}; // 0..1 at the mean temperature of the day
	float wind_speed{3.f};
	float gusts{1.5f}; // of the mean speed
	std::optional<float> wind_direction; // where from, when the kind of weather tells it
	bool snow_likely{false}; // the snow lies whenever the month has any to speak of
	float snow_share{0.6f}; // of the deepest snow of the month
	float snow_drift{0.3f};
	float snow_fresh{0.3f};
};

// the temperatures of the day squeezed towards their mean under a thick sky, Share of the range left
void squeeze(draft &Draft, float const Share)
{
	auto const mean{0.5f * (Draft.temperature_min + Draft.temperature_max)};
	Draft.temperature_min = mean + (Draft.temperature_min - mean) * Share;
	Draft.temperature_max = mean + (Draft.temperature_max - mean) * Share;
}

void shift(draft &Draft, float const Minimum, float const Maximum)
{
	Draft.temperature_min += Minimum;
	Draft.temperature_max += Maximum;
}

void make_fair(month const &Month, bool const Warm, draft &Draft, weather_conditions &Result)
{
	shift(Draft, Warm ? -1.f : -3.f, Warm ? 2.5f : 0.5f);
	Draft.humidity -= 0.10f;
	Draft.wind_speed *= 0.7f;
	Result.visibility = 30000.f;
	if (Warm)
		Result.clouds.push_back(layer(cloud_type::cumulus, 0.15f, 1400.f, 2400.f));
	Result.clouds.push_back(layer(cloud_type::cirrus, 0.2f, 8500.f, 9200.f, 0.7f));
	static_cast<void>(Month);
}

void make_overcast(draft &Draft, weather_conditions &Result)
{
	squeeze(Draft, 0.45f);
	Draft.humidity += 0.08f;
	Result.visibility = 9000.f;
	Result.clouds.push_back(layer(cloud_type::stratocumulus, 1.f, 500.f, 1100.f));
}

void make_precipitation(bool const Cold, draft &Draft, weather_conditions &Result)
{
	squeeze(Draft, 0.35f);
	Draft.humidity = 0.93f;
	Draft.wind_speed *= 1.2f;
	Result.visibility = Cold ? 3500.f : 6000.f;
	Result.precipitation = precipitation_type::automatic;
	Result.precipitation_intensity = 0.45f;
	Result.clouds.push_back(layer(cloud_type::nimbostratus, 1.f, 350.f, 2800.f));
	Draft.snow_fresh = 0.9f;
	Draft.snow_likely = Cold;
}

void make_showers(draft &Draft, weather_conditions &Result)
{
	Draft.humidity += 0.05f;
	Draft.wind_speed *= 1.35f;
	Draft.gusts = 1.9f;
	Result.visibility = 15000.f;
	Result.precipitation = precipitation_type::automatic;
	Result.precipitation_intensity = 0.5f;
	Result.clouds.push_back(layer(cloud_type::cumulus, 0.5f, 900.f, 2200.f));
	Result.clouds.push_back(layer(cloud_type::towering_cumulus, 0.3f, 900.f, 4500.f));
}

void make_storm(draft &Draft, weather_conditions &Result)
{
	shift(Draft, 1.f, 2.f);
	Draft.humidity = 0.85f;
	Draft.wind_speed *= 1.5f;
	Draft.gusts = 2.6f;
	Result.visibility = 4000.f;
	Result.precipitation = precipitation_type::rain;
	Result.precipitation_intensity = 0.9f;
	Result.clouds.push_back(layer(cloud_type::cumulonimbus, 0.75f, 800.f, 10500.f));
	Result.clouds.push_back(layer(cloud_type::stratocumulus, 0.6f, 600.f, 1200.f));
}

void make_fog(draft &Draft, weather_conditions &Result)
{
	Draft.temperature_max = Draft.temperature_min + 3.f;
	Draft.humidity = 1.f;
	Draft.wind_speed = 1.f;
	Draft.gusts = 1.2f;
	Result.visibility = 1500.f;
	Result.groundfog_thickness = 90.f;
	Result.groundfog_visibility = 200.f;
	Result.fog_noise = 0.35f;
	Result.clouds.push_back(layer(cloud_type::stratus, 0.9f, 150.f, 400.f));
}

void make_frost(draft &Draft, weather_conditions &Result)
{
	shift(Draft, -8.f, -5.f);
	Draft.humidity = 0.75f;
	Draft.wind_speed = 2.f;
	Draft.wind_direction = 80.f; // the continental air comes from the east
	Draft.snow_likely = true;
	Draft.snow_share = 1.f;
	Draft.snow_fresh = 0.5f;
	Result.visibility = 25000.f;
	Result.clouds.push_back(layer(cloud_type::cirrus, 0.1f, 8500.f, 9200.f, 0.6f));
}

void make_thaw(draft &Draft, weather_conditions &Result)
{
	Draft.temperature_min = std::max(Draft.temperature_min + 4.f, 1.f);
	Draft.temperature_max = std::max(Draft.temperature_max + 4.f, 5.f);
	Draft.humidity = 0.95f;
	Draft.wind_speed = std::max(Draft.wind_speed * 1.4f, 6.f);
	Draft.wind_direction = 230.f; // mild air from the Atlantic
	Draft.snow_likely = true;
	Draft.snow_share = 0.6f;
	Draft.snow_fresh = 0.f;
	Result.visibility = 5000.f;
	Result.precipitation = precipitation_type::rain;
	Result.precipitation_intensity = 0.2f;
	Result.clouds.push_back(layer(cloud_type::nimbostratus, 1.f, 250.f, 2000.f));
}

void make_blizzard(draft &Draft, weather_conditions &Result)
{
	Draft.temperature_min = std::min(Draft.temperature_min, -6.f);
	Draft.temperature_max = std::min(Draft.temperature_max, -2.f);
	Draft.humidity = 0.9f;
	Draft.wind_speed = 12.f;
	Draft.gusts = 1.7f;
	Draft.snow_likely = true;
	Draft.snow_share = 1.5f;
	Draft.snow_drift = 0.8f;
	Draft.snow_fresh = 1.f;
	Result.visibility = 600.f;
	Result.precipitation = precipitation_type::snow;
	Result.precipitation_intensity = 0.85f;
	Result.clouds.push_back(layer(cloud_type::nimbostratus, 1.f, 200.f, 3000.f));
}

void make_gale(draft &Draft, weather_conditions &Result)
{
	squeeze(Draft, 0.6f);
	Draft.wind_speed = 14.f;
	Draft.gusts = 1.75f;
	Draft.wind_direction = 280.f;
	Draft.snow_drift = 0.7f;
	Result.visibility = 12000.f;
	Result.precipitation = precipitation_type::automatic;
	Result.precipitation_intensity = 0.25f;
	Result.clouds.push_back(layer(cloud_type::stratocumulus, 0.8f, 600.f, 1500.f));
	Result.clouds.push_back(layer(cloud_type::altostratus, 0.6f, 3000.f, 4200.f));
}

void make_heat(draft &Draft, weather_conditions &Result)
{
	shift(Draft, 4.f, 7.f);
	Draft.humidity = 0.45f;
	Draft.wind_speed = 2.f;
	Draft.wind_direction = 150.f;
	Result.visibility = 20000.f;
	Result.clouds.push_back(layer(cloud_type::cumulus, 0.1f, 1800.f, 2600.f));
}

void make_typical(month const &Month, bool const Warm, weather_conditions &Result)
{
	auto const cover{std::clamp(Month.cloudiness / 8.f, 0.f, 1.f)};
	Result.visibility = Warm ? 20000.f : 10000.f;
	if (Warm)
		Result.clouds.push_back(layer(cloud_type::cumulus, cover * 0.8f, 1200.f, 2400.f));
	else
		Result.clouds.push_back(layer(cloud_type::stratocumulus, cover, 600.f, 1300.f));
	Result.clouds.push_back(layer(cloud_type::cirrus, 0.25f, 8500.f, 9200.f, 0.7f));
}

// the clouds, the visibility, what falls and how the kind of weather changes the temperatures, the moisture and the wind
void shape(month const &Month, preset const Preset, draft &Draft, weather_conditions &Result)
{
	auto const warm{Month.temperature() > 8.f};
	auto const cold{Month.temperature() < 1.5f};
	switch (Preset)
	{
	case preset::fair:
		make_fair(Month, warm, Draft, Result);
		break;
	case preset::overcast:
		make_overcast(Draft, Result);
		break;
	case preset::precipitation:
		make_precipitation(cold, Draft, Result);
		break;
	case preset::showers:
		make_showers(Draft, Result);
		break;
	case preset::storm:
		make_storm(Draft, Result);
		break;
	case preset::fog:
		make_fog(Draft, Result);
		break;
	case preset::frost:
		make_frost(Draft, Result);
		break;
	case preset::thaw:
		make_thaw(Draft, Result);
		break;
	case preset::blizzard:
		make_blizzard(Draft, Result);
		break;
	case preset::gale:
		make_gale(Draft, Result);
		break;
	case preset::heat:
		make_heat(Draft, Result);
		break;
	default:
		make_typical(Month, warm, Result);
		break;
	}
}

bool read_month(std::istringstream &Line, station &Station)
{
	int number{0};
	month value;
	Line >> number >> value.temperature_min >> value.temperature_max >> value.humidity >> value.wind_speed >> value.precipitation >> value.precipitation_days >> value.snow_days >> value.snow_depth >>
	    value.fog_days >> value.storm_days >> value.cloudiness;
	if (Line.fail() || (number < 1) || (number > 12))
		return false;
	Station.months[static_cast<std::size_t>(number - 1)] = value;
	return true;
}

bool read_rose(std::istringstream &Line, station &Station)
{
	std::string season;
	wind_rose rose{};
	Line >> season;
	for (auto &share : rose)
		Line >> share;
	auto const found{std::ranges::find(SEASON_NAMES, season)};
	if (Line.fail() || (found == SEASON_NAMES.end()))
		return false;
	Station.roses[static_cast<std::size_t>(found - SEASON_NAMES.begin())] = rose;
	return true;
}

} // namespace

char const *preset_name(preset const Preset)
{
	auto const index{static_cast<std::size_t>(Preset)};
	return (index < PRESET_NAMES.size()) ? PRESET_NAMES[index] : PRESET_NAMES[0];
}

std::optional<preset> preset_from_name(std::string_view const Name)
{
	std::string name{Name};
	std::ranges::transform(name, name.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	for (std::size_t i = 0; i < PRESET_NAMES.size(); ++i)
	{
		if (name == PRESET_NAMES[i])
			return static_cast<preset>(i);
	}
	return std::nullopt;
}

station const &default_station()
{
	static station const result{make_default()};
	return result;
}

bool table::load(std::string const &Path)
{
	std::ifstream file{Path};
	if (!file)
		return false;
	std::ostringstream text;
	text << file.rdbuf();
	return parse(text.str());
}

bool table::parse(std::string_view const Text)
{
	m_stations.clear();
	std::istringstream text{std::string{Text}};
	text.imbue(std::locale::classic());
	std::string row;
	while (std::getline(text, row))
	{
		std::istringstream line{row};
		line.imbue(std::locale::classic());
		std::string keyword;
		line >> keyword;
		if (keyword.empty() || (keyword.front() == '#'))
			continue;
		if (keyword == "station")
		{
			// a station starts with the months and the roses of the default one, for what its rows leave out
			station next{default_station()};
			line >> next.name >> next.latitude >> next.longitude >> next.elevation;
			if (false == line.fail())
				m_stations.push_back(next);
			continue;
		}
		if (m_stations.empty())
			continue;
		if (keyword == "month")
			read_month(line, m_stations.back());
		else if (keyword == "rose")
			read_rose(line, m_stations.back());
	}
	return false == m_stations.empty();
}

station const &table::nearest(float const Latitude, float const Longitude, float const Reach) const
{
	station const *best{nullptr};
	auto bestdistance{Reach};
	for (auto const &candidate : m_stations)
	{
		// km, flat enough over a country
		auto const north{(candidate.latitude - Latitude) * 111.2f};
		auto const east{(candidate.longitude - Longitude) * 111.2f * std::cos(glm::radians(Latitude))};
		auto const distance{std::sqrt(north * north + east * east)};
		if (distance <= bestdistance)
		{
			bestdistance = distance;
			best = &candidate;
		}
	}
	return (best != nullptr) ? *best : default_station();
}

table const &stations()
{
	static table const result{[]() {
		table loaded;
		loaded.load("climate/stations.txt");
		return loaded;
	}()};
	return result;
}

float weight(station const &Station, int const Month, preset const Preset)
{
	auto const index{static_cast<std::size_t>(month_index(Month))};
	auto const &month{Station.months[index]};
	auto const days{MONTH_DAYS[index]};
	auto const mean{month.temperature()};
	auto const wet{std::clamp(month.precipitation_days / days, 0.f, 1.f)};
	auto const cover{std::clamp(month.cloudiness / 8.f, 0.f, 1.f)};
	auto const snowy{mean < 1.5f};
	switch (Preset)
	{
	case preset::typical:
		return 0.2f;
	case preset::fair:
		return (1.f - cover) * 0.6f;
	case preset::overcast:
		return cover * 0.5f;
	case preset::precipitation:
		return wet * (mean < 8.f ? 0.8f : 0.5f);
	case preset::showers:
		return wet * (mean > 8.f ? 0.4f : 0.1f);
	case preset::storm:
		return std::clamp(month.storm_days / days, 0.f, 1.f);
	case preset::fog:
		return std::clamp(month.fog_days / days, 0.f, 1.f);
	case preset::frost:
		return snowy ? (1.f - cover) * 0.5f : 0.f;
	case preset::thaw:
		return (snowy && (month.snow_days > 3.f)) ? 0.12f : 0.f;
	case preset::blizzard:
		return (snowy && (month.snow_days > 3.f)) ? wet * 0.12f : 0.f;
	case preset::gale:
		return 0.03f * month.wind_speed / 3.5f;
	case preset::heat:
		return (month.temperature_max > 22.f) ? 0.1f : 0.f;
	default:
		return 0.f;
	}
}

preset pick(station const &Station, int const Month, float const Random)
{
	auto total{0.f};
	for (int i = 0; i < static_cast<int>(preset::count); ++i)
		total += weight(Station, Month, static_cast<preset>(i));
	auto left{std::clamp(Random, 0.f, 1.f) * total};
	for (int i = 0; i < static_cast<int>(preset::count); ++i)
	{
		left -= weight(Station, Month, static_cast<preset>(i));
		if (left <= 0.f)
			return static_cast<preset>(i);
	}
	return preset::typical;
}

weather_conditions make(station const &Station, int const Month, preset const Preset, std::uint32_t const Seed)
{
	auto const index{static_cast<std::size_t>(month_index(Month))};
	auto const &month{Station.months[index]};
	weather_conditions result;
	result.seed = Seed;
	// the numbers of the day, other ones in another month
	auto const day{Seed + static_cast<std::uint32_t>(index) * 7919u};
	result.precipitation = precipitation_type::none;
	result.precipitation_intensity = 0.f;
	result.haze_height = 800.f;

	draft draft;
	// a day a little colder or warmer than the average of the month
	auto const anomaly{(hashed(day, 1) - 0.5f) * 3.f};
	draft.temperature_min = month.temperature_min + anomaly;
	draft.temperature_max = month.temperature_max + anomaly;
	draft.humidity = month.humidity / 100.f;
	draft.wind_speed = month.wind_speed * (0.8f + 0.4f * hashed(day, 2));
	shape(month, Preset, draft, result);

	result.temperature_min = draft.temperature_min;
	result.temperature_max = std::max(draft.temperature_max, draft.temperature_min + 0.5f);
	auto const mean{0.5f * (*result.temperature_min + *result.temperature_max)};
	result.temperature = mean;
	result.dew_point = psychrometry::dew_point(mean, std::clamp(draft.humidity, 0.2f, 1.f));

	result.wind_described = true;
	result.wind_direction = draft.wind_direction ? std::fmod(*draft.wind_direction + (hashed(day, 3) - 0.5f) * 40.f + 360.f, 360.f) :
	                                               rose_direction(Station.roses[static_cast<std::size_t>(season_of(Month))], hashed(day, 4), hashed(day, 5));
	result.wind_speed = draft.wind_speed;
	result.wind_gust = draft.wind_speed * draft.gusts;
	// the clouds go with the same wind, faster
	result.cloud_wind_direction = result.wind_direction;
	result.cloud_wind_speed = std::max(result.wind_speed * 1.5f, 2.f);

	// the snow lying: as often as the month has it, always in the weather which comes with it, never in a warm one
	auto const days{MONTH_DAYS[index]};
	auto const often{std::clamp(month.snow_days / days, 0.f, 1.f)};
	auto const lying{(mean < 4.f) && (month.snow_days >= 1.f) && ((draft.snow_likely && (often > 0.08f)) || (hashed(day, 6) < often))};
	if (lying || (Preset == preset::blizzard))
	{
		auto const deepest{std::max(month.snow_depth, Preset == preset::blizzard ? 15.f : 2.f) / 100.f};
		result.snow_depth = deepest * draft.snow_share * (0.6f + 0.6f * hashed(day, 7));
		result.snow_drift = draft.snow_drift;
		result.snow_fresh = draft.snow_fresh;
	}
	else
		result.snow_depth = 0.f;
	return result;
}

} // namespace climate
