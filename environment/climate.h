/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <glm/glm.hpp>

struct weather_conditions;

// the climate of a place, month by month, and the weather made of it: kinds of weather a month there typically has (a preset), each
// with the temperatures of the day, the moisture of the air, the wind at the ground with its gusts, the clouds, the visibility, what
// falls and the snow lying, and how often each comes, so the weather of a scenery can be drawn as the place and the month have it.
// the months are the climate normals of a weather station: 1991-2020 of the synoptic stations of IMGW-PIB, built into the
// simulator (environment/climatestations.cpp, made by tools/weather/imgw_normals.py of the public data). for a place with no
// station near a station of the lowlands in the middle of Poland stands in (Lodz-Lublinek)
namespace climate
{

struct month
{
	float temperature_min{0.f}; // mean lowest temperature of a day, degrees C
	float temperature_max{0.f}; // mean highest temperature of a day
	float humidity{80.f}; // mean relative humidity, %
	float wind_speed{3.5f}; // mean speed of the wind, m/s
	float precipitation{40.f}; // mean sum of the month, mm
	float precipitation_days{12.f}; // days with any precipitation
	float snow_days{0.f}; // days with snow lying
	float snow_depth{0.f}; // the deepest snow of the month on average, cm
	float fog_days{2.f}; // days with fog
	float storm_days{0.f}; // days with a thunderstorm
	float cloudiness{5.5f}; // mean cover of the sky, oktas

	float temperature() const
	{
		return 0.5f * (temperature_min + temperature_max);
	}
};

// shares of the wind from N, NE, E, SE, S, SW, W, NW
using wind_rose = std::array<float, 8>;

struct station
{
	std::string name;
	float latitude{52.f}; // degrees
	float longitude{19.f};
	float elevation{150.f}; // m above the sea
	std::array<month, 12> months;
	// winter (December to February), spring, summer, autumn
	std::array<wind_rose, 4> roses;
};

// the kinds of weather; the order is that of the names of preset_name()
enum class preset : int
{
	typical = 0, // an average day of the month
	fair, // a high: little cloud, a wide range of the temperature
	overcast, // a low grey sky, dry
	precipitation, // steady rain or snow from a thick layer
	showers, // passing showers from heaped clouds, gusty
	storm, // a thunderstorm
	fog, // a still foggy day
	frost, // a winter high: clear, still, well below the average
	thaw, // mild wet air over lying snow
	blizzard, // heavy snow in a strong wind
	gale, // a deep low passing: a very strong gusty wind
	heat, // a hot spell
	count
};

char const *preset_name(preset Preset);
// a preset by its name, any case; nullopt for an unknown one
std::optional<preset> preset_from_name(std::string_view Name);

// the station standing in where there's no table or no station near
station const &default_station();

// the stations of the table
class table
{
  public:
	// the stations built into the simulator
	void builtin();
	// reads the stations of a file made by tools/weather/imgw_normals.py; false when it isn't there or holds none
	bool load(std::string const &Path);
	// reads them from a text
	bool parse(std::string_view Text);
	std::vector<station> const &stations() const
	{
		return m_stations;
	}
	// the station nearest to a place (degrees) within Reach km, the summits left out, or the default one
	station const &nearest(float const Latitude, float const Longitude, float const Reach = 150.f) const;

  private:
	std::vector<station> m_stations;
};

// the table of the simulator: the one built in, or climate/stations.txt in its folder when there is one, read at the first use
table const &stations();

// how often a kind of weather comes in a month (0..11) at a station; not normalised, 0 for one which doesn't come then
float weight(station const &Station, int const Month, preset const Preset);
// the kind drawn for a month by a number 0..1, each as often as it comes
preset pick(station const &Station, int const Month, float const Random);
// the weather of a kind in a month at a station; Seed makes one day of that kind out of many (the direction of the wind, how much
// colder or warmer than the average, whether the snow lies)
weather_conditions make(station const &Station, int const Month, preset const Preset, std::uint32_t const Seed);

} // namespace climate
