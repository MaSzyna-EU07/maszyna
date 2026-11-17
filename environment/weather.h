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

#include "environment/psychrometry.h"
#include "environment/snowcover.h"

class cParser;

// cloud genera as aviation reports them (WMO), each drawn in its own way. the order is that of the names in cloud_genus_info()
enum class cloud_type : int
{
	cumulus = 0,
	towering_cumulus,
	cumulonimbus,
	stratocumulus,
	stratus,
	nimbostratus,
	altocumulus,
	altostratus,
	cirrus,
	cirrostratus,
	cirrocumulus,
	count
};

// what a genus typically looks like, and how it is drawn
struct cloud_genus
{
	char const *name; // full name, as in the weather section of a scenery
	char const *abbreviation; // as in reports (Cu, TCu, Cb...)
	float base; // typical height of the base above the ground, metres
	float thickness; // typical depth of the layer, metres
	float extinction; // light taken out per metre inside the cloud at density 1 (1/m); for a flat layer its optical depth
	float cell; // horizontal size of the single clouds or of the lumps of a sheet, metres
	float spread; // horizontal size of the areas the coverage is spread over, metres
	float bottom; // part of the depth over which the density rises from the base: small for the flat base of a cumulus
	float top; // part of the depth from which the density falls towards the top
	float top_variation; // how much the top of single clouds varies (towers of different heights), 0-1
	float anvil; // how far the top spreads out (the anvil of a cumulonimbus), 0-1
	float erosion; // how much the small scale noise eats into the edges, 0-1
	float stretch; // how much the noise is drawn out along the wind (streaks of a cirrus), 1 = not at all
	float opacity; // share of the sky light a full layer holds back, for the overcast the rest of the simulation goes by
	bool flat; // drawn as a thin sheet (high clouds), not ray marched through
};

cloud_genus const &cloud_genus_info(cloud_type Type);
// genus named by its full name or abbreviation, any case
std::optional<cloud_type> cloud_type_from_name(std::string_view Name);

struct cloud_layer
{
	cloud_type type{cloud_type::cumulus};
	float coverage{0.f}; // share of the sky the layer covers, 0-1 (oktas / 8)
	float base{1000.f}; // height of the base above the ground reference, metres
	float top{2000.f}; // height of the top above the ground reference, metres
	float density{1.f}; // multiplier of the typical density of the genus
};

enum class precipitation_type : int
{
	automatic = 0, // rain or snow by the air temperature, as the overcast above 1 always did
	none,
	rain,
	snow
};

// the weather of the scenery: what the simulation keeps and the renderer draws. all heights are above the ground reference
struct weather_conditions
{
	static constexpr std::size_t max_cloud_layers{4};

	std::vector<cloud_layer> clouds; // ordered by the base, at most max_cloud_layers
	// meteorological optical range at the ground, metres: how far a dark object is still seen against the horizon (5% contrast)
	float visibility{10000.f};
	float haze_height{800.f}; // scale height of the haze: its density falls e times every this many metres up
	float groundfog_thickness{0.f}; // fog lying on the ground (in the valleys below the reference too), metres; 0 = none
	float groundfog_visibility{300.f}; // visibility inside it
	float fog_noise{0.3f}; // how patchy the fog is, 0-1
	precipitation_type precipitation{precipitation_type::automatic};
	float precipitation_intensity{0.f}; // 0-1
	float cloud_wind_direction{270.f}; // where the wind at the height of the clouds blows from, degrees from the north, clockwise
	float cloud_wind_speed{8.f}; // m/s
	// the wind at the ground, when the description gives it (a report, the wind entry): where it blows from, degrees from the north,
	// clockwise, its mean speed and the speed of its gusts, m/s (0: none given, they're taken as half as much again). without it
	// the simulation keeps its light changing wind
	bool wind_described{false};
	float wind_direction{270.f};
	float wind_speed{0.f};
	float wind_gust{0.f};
	std::uint32_t seed{0}; // changes the shapes of the clouds
	std::optional<float> temperature; // air temperature a report gave, degrees C; applied with the weather
	// the lowest and the highest temperature of the day, degrees C: with both the air temperature follows the time of the day
	// between them (psychrometry::diurnal_temperature())
	std::optional<float> temperature_min;
	std::optional<float> temperature_max;
	// the dew point, degrees C: the moisture of the air, which the humidity and the wet-bulb temperature are worked out from
	std::optional<float> dew_point;
	// where the scenery lies, degrees of latitude and longitude, for the climate its weather is taken from (environment/climate.h)
	std::optional<glm::vec2> location;
	// snow lying on the ground (the snowcover entry), applied with the weather: the depth over open flat ground, m, and how far
	// the wind has worked it into drifts, 0..1, and how loose it still is, 0..1
	std::optional<float> snow_depth;
	float snow_drift{0.5f};
	float snow_fresh{0.5f};

	// share of the sky the clouds hold back, 0-1, and the precipitation past it: the overcast the rest of the simulation goes by
	float overcast() const;
	// sorts the layers by their base and keeps them within their limits
	void normalize();
};

// reads a weather report (METAR, the body of it: the station and time may be there or not) into Conditions. what the report does
// not mention is left as it was, apart from the clouds, which are replaced when it has any cloud group or says there are none.
// returns false when nothing in it was understood
bool parse_metar(std::string_view Report, weather_conditions &Conditions);
// a report of the conditions in the METAR form, for the user interface and the scripting interface
std::string to_metar(weather_conditions const &Conditions);
// clouds and precipitation of the old single number of the weather (atmo, the overcast slider, scripts): up to 1 the sky clouds
// over, past it rain or snow falls
std::vector<cloud_layer> clouds_from_overcast(float Overcast);
// visibility of the old fog range (atmo, Global.fFogEnd), as the old fog thickened with the overcast, and back
float visibility_from_fog_range(float Fogend, float Overcast);
float fog_range_from_visibility(float Visibility, float Overcast);

// the weather of the simulation. the scenery may describe it (the weather section); otherwise, and whenever something sets the
// old overcast and fog range (atmo, the sliders, the scripting interface, the setweather command of the network), the clouds and
// the visibility are made from those. while the weather is described, the overcast and fog range are worked out from it, so the
// rest of the simulation (lights, sounds, precipitation, friction) goes on as before
class world_weather
{
  public:
	// forgets the weather of the previous scenery
	void reset();
	// the weather section of a scenery, read before the simulation starts
	void deserialize(cParser &Input);
	// starts the weather of the loaded scenery
	void init();
	// moves the clouds with the wind, goes on with a change of the weather and keeps the old global state in step
	void update(double const Deltatime, glm::dvec3 const &Camera);
	// changes the weather while the simulation runs, over Transition seconds. Quiet: not logged (changes made by a slider)
	void apply(weather_conditions const &Conditions, float const Transition, bool const Quiet = false);
	// changes the weather after a report or a weather section text, as the scripting interface gives it. false when it says nothing
	bool apply_text(std::string_view Text, float const Transition);

	// the weather as it is now (in the middle of a change, a mix of the old and the new one)
	weather_conditions const &conditions() const
	{
		return m_current;
	}
	// the weather a change goes to, or the current one
	weather_conditions const &target() const
	{
		return m_target;
	}
	// true when the clouds come from a description, false when from the old overcast
	bool described() const
	{
		return m_described_clouds;
	}
	// world height (y) of the ground the heights of the weather are counted from
	float ground() const
	{
		return m_ground;
	}
	// how far the wind carried the clouds, world x and z, metres
	glm::dvec2 const &cloud_offset() const
	{
		return m_cloud_offset;
	}
	// seconds of simulation since the start, for the clouds and fog to change their shapes slowly
	double time() const
	{
		return m_time;
	}
	// world space velocity of the clouds (x, z), m/s
	glm::vec2 cloud_velocity() const;
	// precipitation the description sets, or nullopt when the air temperature decides
	std::optional<precipitation_type> precipitation_override() const;
	// the snow lying on the ground and stuck to the scenery, and the ice on the wires
	snow_cover const &snow() const
	{
		return m_snow;
	}
	snow_cover &snow()
	{
		return m_snow;
	}
	// the mean wind at the ground the weather describes, world x and z of where it blows to, m/s; without a described wind the one
	// the clouds go with, slowed down as it is near the ground
	glm::vec2 ground_wind() const;
	// the wind at the ground as it blows now, with its gusts, world x and z of where it blows to, m/s: the wind of the simulation
	// (world_environment::wind()), which follows the described one, given here by the environment every frame
	glm::vec2 const &surface_wind() const
	{
		return m_surfacewind;
	}
	void surface_wind(glm::vec2 const &Wind)
	{
		m_surfacewind = Wind;
	}
	// the moisture of the air now: the dew point (the described one, or a likely one for the weather), the relative humidity 0..1
	// and the wet-bulb temperature, degrees C
	float dew_point() const;
	float humidity() const;
	float wet_bulb() const;
	// what falls now
	psychrometry::falling falling() const;
	// where the scenery lies, degrees of latitude and longitude: the location entry of the weather section, or the place the editor
	// keeps for the scenery; nullopt when neither is there
	std::optional<glm::vec2> location() const;

  private:
	// the clouds and the visibility of the old global state
	void follow_overcast();
	void follow_fog_range();
	// the old global state of the weather described
	void write_globals();
	// the snow of a description applied, and the snow cover moved on by a step
	void apply_snow();
	void update_snow(double const Deltatime);
	// the air temperature of the time of the day, where the weather gives its range, and the weather name after what falls now
	void update_air();

	std::optional<weather_conditions> m_scenery; // read from the scenery
	std::optional<float> m_elevation; // ground reference the scenery gives
	weather_conditions m_from; // weather a change started at
	weather_conditions m_target;
	weather_conditions m_current;
	snow_cover m_snow;
	glm::vec2 m_surfacewind{0.f};
	std::vector<std::string> m_scenerytokens; // the weather sections of the scenery, read once its date is known (init())
	bool m_snowing{false}; // what the weather name was made for
	double m_transition{0.0}; // length of the change in progress, seconds
	double m_transition_time{0.0}; // how far it got
	bool m_described_clouds{false};
	bool m_described_fog{false};
	// the overcast and fog range last written or followed, to tell when something else changed them
	float m_known_overcast{-1.f};
	float m_known_fogend{-1.f};
	bool m_ground_set{false};
	float m_ground{0.f};
	glm::dvec2 m_cloud_offset{0.0};
	double m_time{0.0};
};
