#include "stdafx.h"
#include "doctest.h"
#include "environment/railsurface.h"
#include "environment/psychrometry.h"
#include "environment/liveweather.h"
#include "McZapkie/MOVER.h"
#include "McZapkie/weatherphysics.h"
#include "rendering/common/arcflicker.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "vehicle.h"

#include <algorithm>
#include <cstdint>
#include <memory>

// the weather in the physics of the vehicles (McZapkie/weatherphysics.h, environment/railsurface.h, physics.weather)

namespace {

weather_physics::settings const defaults{};

// physics.weather as a test sets it (off by default), given back as it was afterwards
class weather_physics_setting {
public:
	explicit weather_physics_setting(bool const Enabled) : m_previous{Global.physics_weather} { Global.physics_weather.enabled = Enabled; }
	~weather_physics_setting() { Global.physics_weather = m_previous; }
	weather_physics_setting(weather_physics_setting const &) = delete;
	weather_physics_setting &operator=(weather_physics_setting const &) = delete;

private:
	weather_physics::settings m_previous;
};

// Hours of the weather given, in steps of a minute
void run(rail_surface &Rails, snow_weather const &Weather, double const Hours, float const Daylight = 1.f, float const Leaves = 0.f) {
	auto const steps{static_cast<int>(Hours * 60.0)};
	for (int step = 0; step < steps; ++step)
		Rails.update(60.0, Weather, Daylight, Leaves);
}

snow_weather dry(float const Temperature = 15.f) {
	snow_weather weather;
	weather.temperature = Temperature;
	weather.humidity = 0.6f;
	return weather;
}

snow_weather rain(float const Intensity, float const Temperature = 10.f) {
	auto weather{dry(Temperature)};
	weather.rainfall = Intensity;
	weather.humidity = 0.95f;
	return weather;
}

} // namespace

TEST_CASE("weather physics: a dry clean rail keeps all of its adhesion") {
	CHECK(weather_physics::adhesion_factor(defaults, weather_physics::railhead{}, 0.0, -1.0) == 1.0);
}

TEST_CASE("weather physics: the first drizzle on the dirt of a dry spell is worse than the rail washed by a downpour") {
	rail_surface rails;
	run(rails, dry(), 48.0);
	CHECK(rails.state().dirt > 0.8f);
	CHECK(rails.state().water < 0.01f);
	// ten minutes of a drizzle wet the dirt without washing it off
	run(rails, rain(0.15f), 10.0 / 60.0);
	auto const drizzle{weather_physics::adhesion_factor(defaults, rails.state(), 0.0, -1.0)};
	// an hour of a heavy rain washes it off
	run(rails, rain(1.f), 1.0);
	auto const washed{weather_physics::adhesion_factor(defaults, rails.state(), 0.0, -1.0)};
	CAPTURE(drizzle);
	CAPTURE(washed);
	CHECK(rails.state().dirt < 0.1f);
	CHECK(drizzle < washed);
	CHECK(washed == doctest::Approx(defaults.wet).epsilon(0.05));
	CHECK(drizzle < 0.55);
}

TEST_CASE("weather physics: a wet rail dries in an hour or so after the rain, and freezes in a frost") {
	rail_surface rails;
	run(rails, rain(1.f), 1.0);
	CHECK(rails.state().water > 0.9f);
	run(rails, dry(), 2.0);
	CHECK(rails.state().water < 0.05f);

	rail_surface frozen;
	run(frozen, rain(1.f, 2.f), 1.0);
	run(frozen, dry(-6.f), 2.0);
	CHECK(frozen.state().water < 0.05f);
	CHECK(frozen.state().ice > 0.8f);
	CHECK(weather_physics::adhesion_factor(defaults, frozen.state(), 0.0, -1.0) < 0.35);
}

TEST_CASE("weather physics: a freezing rain glazes the rail, the thaw takes the ice away") {
	rail_surface rails;
	run(rails, rain(0.5f, -2.f), 0.5);
	CHECK(rails.state().ice > 0.9f);
	run(rails, rain(0.5f, 6.f), 1.0);
	CHECK(rails.state().ice < 0.05f);
}

TEST_CASE("weather physics: the wheels going over a rail clean it for the ones behind them") {
	weather_physics::railhead leaves;
	leaves.leaves = 1.f;
	leaves.water = 1.f;
	auto const untouched{weather_physics::adhesion_factor(defaults, leaves, 0.0, -1.0)};
	auto const just{weather_physics::adhesion_factor(defaults, leaves, 0.0, 1.0)};
	auto const hour{weather_physics::adhesion_factor(defaults, leaves, 0.0, 3600.0)};
	CAPTURE(untouched);
	CAPTURE(just);
	CHECK(untouched < 0.25);
	CHECK(just > untouched * 2.0);
	CHECK(hour < just);
	CHECK(hour == doctest::Approx(untouched).epsilon(0.05));
}

TEST_CASE("weather physics: snow on the head of the rail takes the adhesion down, the least is kept") {
	CHECK(weather_physics::adhesion_factor(defaults, weather_physics::railhead{}, 0.05, -1.0) == doctest::Approx(defaults.snow));
	weather_physics::railhead worst{1.f, 1.f, 1.f, 1.f};
	CHECK(weather_physics::adhesion_factor(defaults, worst, 0.05, -1.0) == doctest::Approx(defaults.minimum));
}

TEST_CASE("weather physics: the leaves fall in the autumn only") {
	CHECK(rail_surface::leaf_season(200) == 0.f);
	CHECK(rail_surface::leaf_season(300) == 1.f);
	CHECK(rail_surface::leaf_season(360) == 0.f);
	rail_surface rails;
	run(rails, dry(), 24.0, 1.f, rail_surface::leaf_season(300));
	CHECK(rails.state().leaves > 0.9f);
}

TEST_CASE("weather physics: the sand gives back a share of the dry rail on a slippery one, on a dry one as before") {
	CHECK(weather_physics::sanded(defaults, 0.15, 0.15, 1.25) == 1.25);
	auto const leaves{weather_physics::sanded(defaults, 0.03, 0.15, 1.25)};
	CHECK(0.03 * leaves == doctest::Approx(0.15 * defaults.sand));
}

TEST_CASE("weather physics: the rain wets the brakes, the braking dries them; the snow ices them in a frost") {
	weather_physics::brake_state brakes;
	weather_physics::brake_weather rainy{1.0, 0.0, 10.0};
	for (int i = 0; i < 600; ++i)
		weather_physics::update_brakes(brakes, rainy, 25.0, 0.0, 40000.0, 1.0);
	CHECK(brakes.water > 0.9);
	auto const wet{weather_physics::brake_factor(defaults, weather_physics::brake_material::cast_iron, brakes)};
	CHECK(wet == doctest::Approx(defaults.brake_wet_cast_iron).epsilon(0.05));
	// a few seconds of a full braking of a 40 t wagon at 70 km/h dry them
	for (int i = 0; i < 3; ++i)
		weather_physics::update_brakes(brakes, rainy, 20.0, 40000.0 * 20.0, 40000.0, 1.0);
	CHECK(brakes.water < 0.1);

	weather_physics::brake_state frozen;
	weather_physics::brake_weather snowy{0.0, 1.0, -8.0};
	for (int i = 0; i < 1800; ++i)
		weather_physics::update_brakes(frozen, snowy, 30.0, 0.0, 40000.0, 1.0);
	CHECK(frozen.ice > 0.7);
	CHECK(weather_physics::brake_factor(defaults, weather_physics::brake_material::composite, frozen) < 0.6);
}

TEST_CASE("weather physics: the drag of the calm air at 15 degrees is the one of FrictionForce(), the wind changes it") {
	auto const drag{3.0};
	auto const calm{weather_physics::aerodynamic_force(drag, 20.0, 0.0, 0.0, 1.0, defaults.wind_yaw)};
	CHECK(calm == doctest::Approx(-drag * 400.0));
	// a head wind holds back, a following one pushes a vehicle at a stand
	CHECK(weather_physics::aerodynamic_force(drag, 20.0, -10.0, 0.0, 1.0, defaults.wind_yaw) < calm);
	CHECK(weather_physics::aerodynamic_force(drag, 0.0, 15.0, 0.0, 1.0, defaults.wind_yaw) > 0.0);
	// the wind across adds to it
	CHECK(weather_physics::aerodynamic_force(drag, 20.0, 0.0, 10.0, 1.0, defaults.wind_yaw) < calm);
	// the cold air is denser
	CHECK(weather_physics::air_density(-15.0) > 1.1);
	CHECK(weather_physics::air_density(15.0) == doctest::Approx(1.0));
}

TEST_CASE("weather physics: the vehicle gets no force of the weather in calm air at 15 degrees") {
	weather_physics_setting const on{true};
	auto vehicle{std::make_unique<TMoverParameters>(0.0, "test", "test", 1)};
	vehicle->DragConst = 3.0;
	vehicle->V = 20.0;
	CHECK(vehicle->WeatherDragForce() == 0.0);
	vehicle->RunningWeather.wind_along = -10.0;
	CHECK(vehicle->WeatherDragForce() < 0.0);
	vehicle->RunningWeather.wind_along = 0.0;
	vehicle->RunningWeather.air_density = weather_physics::air_density(-15.0);
	CHECK(vehicle->WeatherDragForce() < 0.0);
}

TEST_CASE("weather physics: the ice on the wires breaks the contact more at speed, less behind another pantograph") {
	CHECK(weather_physics::break_rate(defaults, 0.0, 0.0, 30.0, true) == 0.0);
	auto const slow{weather_physics::break_rate(defaults, 0.003, 0.0, 10.0, true)};
	auto const fast{weather_physics::break_rate(defaults, 0.003, 0.0, 30.0, true)};
	auto const behind{weather_physics::break_rate(defaults, 0.003, 0.0, 30.0, false)};
	CHECK(fast > slow * 3.0);
	CHECK(behind == doctest::Approx(fast * defaults.trailing_share));
	CHECK(weather_physics::break_length(defaults, 0.004, 0.5) == doctest::Approx(defaults.break_time * 2.0));
}

TEST_CASE("weather physics: a gale blows a span of the catenary off the strip of a pantograph, a breeze doesn't") {
	// a span of 65 m square across the wind, a wire of 12 mm at 10 kN
	auto const breeze{weather_physics::wire_sway(defaults, 65.0, 10.0, 0.0, 1.0)};
	auto const strong{weather_physics::wire_sway(defaults, 65.0, 20.0, 0.0, 1.0)};
	auto const gale{weather_physics::wire_sway(defaults, 65.0, 30.0, 0.0, 1.0)};
	CAPTURE(strong);
	CAPTURE(gale);
	CHECK(breeze < 0.05);
	CHECK(strong == doctest::Approx(0.186).epsilon(0.05));
	CHECK(gale == doctest::Approx(0.419).epsilon(0.05));
	// with the stagger of 0.3 m the gale takes the wire past half the strip of an AKP-4E (0.635 m)
	CHECK(gale + 0.3 > 0.635);
	// towards where the wind blows, more with the ice on the wire
	CHECK(weather_physics::wire_sway(defaults, 65.0, -30.0, 0.0, 1.0) == doctest::Approx(-gale));
	CHECK(weather_physics::wire_sway(defaults, 65.0, 30.0, 0.005, 1.0) > gale * 1.5);
	// none at the supports, all of it in the middle
	CHECK(weather_physics::sway_profile(0.0) == 0.0);
	CHECK(weather_physics::sway_profile(0.5) == 1.0);
	CHECK(weather_physics::sway_profile(1.0) == 0.0);
}

namespace {

// a freezing fog of the given frost and wind, Hours of it in steps of a minute
snow_cover fog(float const Temperature, float const Wind, double const Hours) {
	snow_cover snow;
	snow_weather weather;
	weather.temperature = Temperature;
	weather.humidity = 1.f;
	weather.wet_bulb = Temperature;
	weather.fog = 1.f;
	weather.wind = {Wind, 0.f};
	for (int step = 0; step < static_cast<int>(Hours * 60.0); ++step)
		snow.update(60.0, weather);
	return snow;
}

} // namespace

TEST_CASE("weather physics: a freezing fog in a wind lays hard rime on the wires, in a calm the soft one") {
	using kind = snow_cover::ice_kind;
	auto const windy{fog(-5.f, 6.f, 8.0)};
	auto const calm{fog(-5.f, 0.5f, 6.0)};
	CAPTURE(windy.wire_ice(kind::rime_hard));
	CAPTURE(calm.wire_ice(kind::rime_soft));
	CHECK(windy.wire_ice(kind::rime_hard) > 0.001f);
	CHECK(windy.wire_ice(kind::rime_hard) > windy.wire_ice(kind::rime_soft) * 3.f);
	CHECK(calm.wire_ice(kind::rime_soft) > 0.0005f);
	CHECK(calm.wire_ice(kind::rime_hard) == 0.f);
	CHECK(calm.wire_ice(kind::glaze) == 0.f);
	// the soft rime is light, the hard rime denser
	CHECK(calm.wire_ice_density() < 200.f);
	CHECK(windy.wire_ice_density() > 300.f);
}

TEST_CASE("weather physics: a kind of the ice left out by the settings doesn't grow") {
	using kind = snow_cover::ice_kind;
	snow_cover snow;
	snow.wire_ice_kinds[static_cast<int>(kind::glaze)] = false;
	snow_weather rain;
	rain.temperature = -2.f;
	rain.wet_bulb = psychrometry::wet_bulb(-2.f, rain.humidity);
	rain.rainfall = 1.f;
	for (int step = 0; step < 180; ++step)
		snow.update(60.0, rain);
	CHECK(snow.wire_ice() == 0.f);
}

TEST_CASE("weather physics: the glaze breaks the contact most, the soft rime least; an arc needs a current") {
	auto const glaze{weather_physics::contact_ice(defaults, 0.002, 0.0, 0.0, 0.0)};
	auto const soft{weather_physics::contact_ice(defaults, 0.0, 0.0, 0.002, 0.0)};
	CHECK(glaze == doctest::Approx(0.002));
	CHECK(soft == doctest::Approx(0.002 * defaults.break_rime_soft));
	CHECK(weather_physics::break_rate(defaults, soft, 0.0, 25.0, true) < weather_physics::break_rate(defaults, glaze, 0.0, 25.0, true));
	CHECK(weather_physics::arc_strength(0.0) == 0.0);
	CHECK(weather_physics::arc_strength(-800.0) == 1.0);
	CHECK(weather_physics::arc_strength(300.0) > 0.3);
	CHECK(weather_physics::arc_strength(300.0) < 0.7);
}

TEST_CASE("live weather: the answer of Open-Meteo read into a weather report") {
	auto const answer{R"({"latitude":52.2,"longitude":21.0,"current":{"time":"2026-11-19T22:00","interval":900,"temperature_2m":-2.4,
		"relative_humidity_2m":93,"dew_point_2m":-3.3,"weather_code":66,"cloud_cover":100,"cloud_cover_low":100,"cloud_cover_mid":40,
		"cloud_cover_high":0,"wind_speed_10m":6.2,"wind_direction_10m":274,"wind_gusts_10m":13.0,"visibility":2400,"snow_depth":0.05}})"};
	auto const report{live_weather::read(answer, {52.2, 21.0})};
	CAPTURE(report.metar);
	CHECK(report.error.empty());
	// the wind from 270 at 6 m/s gusting 13, the view 2.4 km, a light freezing rain, a low overcast with a base the spread of the
	// temperature and the dew point gives, the middle clouds scattered, the temperature and the dew point below zero
	CHECK(report.metar == "27006G13MPS 2400 -FZRA OVC004 SCT115 M02/M03");
	REQUIRE(report.snow_depth.has_value());
	CHECK(*report.snow_depth == doctest::Approx(0.05f));
}

TEST_CASE("live weather: a calm clear night, a storm, and an answer with an error") {
	auto const clear{live_weather::read(R"({"current":{"temperature_2m":4.0,"dew_point_2m":1.0,"weather_code":0,"cloud_cover":0,
		"cloud_cover_low":0,"cloud_cover_mid":0,"cloud_cover_high":0,"wind_speed_10m":0.2,"wind_direction_10m":0,"wind_gusts_10m":1.0,
		"visibility":24000}})",
	                                    {50.0, 20.0})};
	CHECK(clear.metar == "00000MPS 9999 NSC 04/01");
	auto const storm{live_weather::read(R"({"current":{"temperature_2m":24.0,"dew_point_2m":18.0,"weather_code":95,"cloud_cover_low":70,
		"wind_speed_10m":9.0,"wind_direction_10m":225,"wind_gusts_10m":21.0,"visibility":6000}})",
	                                    {50.0, 20.0})};
	CAPTURE(storm.metar);
	CHECK(storm.metar.find("TSRA") != std::string::npos);
	CHECK(storm.metar.find("CB") != std::string::npos);
	auto const failed{live_weather::read(R"({"error":true,"reason":"Latitude must be in range of -90 to 90°."})", {200.0, 20.0})};
	CHECK(failed.metar.empty());
	CHECK(failed.error.find("Latitude") != std::string::npos);
	CHECK(live_weather::url({52.25, 21.0}).find("latitude=52.2500&longitude=21.0000") != std::string::npos);
}

namespace {

// a cold wet day with a strong head wind blowing across too: all the weather a vehicle can run in
TWeatherParam foul_weather() {
	TWeatherParam weather;
	weather.rain = 1.0;
	weather.snow = 1.0;
	weather.temperature = -2.0;
	weather.wind_along = -15.0;
	weather.wind_across = 12.0;
	weather.air_density = weather_physics::air_density(-2.0);
	return weather;
}

struct braking {
	double speed{0.0}; // m/s, after the braking
	// the weather factor of the friction of its brakes and the water and the ice on them, as the braking begins: the braking itself
	// dries them, after it they may be dry again
	double brake_factor{1.0};
	weather_physics::brake_state brakes;
};

// a wagon coasting at 60 km/h with the brake pipe charged, then braking with a service application, in the Weather
braking braked_in(TWeatherParam const &Weather) {
	test::reseed_random();
	test::vehicle wagon{test::definition::load("wagon"), 60.0};
	wagon->DragConst = 3.0;
	wagon->RunningWeather = Weather;
	wagon.hold_brake_pipe(5.0);
	wagon.run(30.0);
	braking result;
	result.brake_factor = wagon->Hamulec->weather_factor();
	result.brakes = wagon->BrakeWeather;
	wagon.hold_brake_pipe(3.5);
	wagon.run(15.0);
	result.speed = wagon->V;
	return result;
}

} // namespace

TEST_CASE("weather physics: off (physics.weather 0), a vehicle brakes and coasts in any weather as it does in the calm") {
	braking calm;
	braking foul;
	{
		weather_physics_setting const off{false};
		calm = braked_in(TWeatherParam{});
		foul = braked_in(foul_weather());
	}
	CAPTURE(calm.speed);
	CAPTURE(foul.speed);
	// the brakes neither get wet nor iced, the wind and the cold air add no drag: the very same run
	CHECK(foul.speed == calm.speed);
	CHECK(foul.brake_factor == 1.0);
	CHECK(foul.brakes.water == 0.0);
	CHECK(foul.brakes.ice == 0.0);
	// the run still goes: the wagon slows down, but isn't stopped yet
	CHECK(calm.speed > 0.0);
	CHECK(calm.speed < 60.0 / 3.6);

	// on, the same weather does change the run: the comparison above isn't of two runs the weather can't touch
	weather_physics_setting const on{true};
	auto const weathered{braked_in(foul_weather())};
	CAPTURE(weathered.speed);
	CHECK(weathered.brakes.water + weathered.brakes.ice > 0.0);
	CHECK(weathered.brake_factor < 1.0);
	CHECK(weathered.speed != calm.speed);
}

TEST_CASE("weather physics: off (physics.weather 0), the sand on a slippery rail adds the share it always did") {
	auto vehicle{std::make_unique<TMoverParameters>(0.0, "test", "test", 1)};
	vehicle->SandDose = true;
	// a rail of leaves, of 0.15 when dry and clean
	vehicle->RunningTrack.friction_dry = 0.15;
	auto const slippery{0.03};
	// at a stand, no slip: the adhesion is twice the friction, the sand adds a quarter to it
	{
		weather_physics_setting const off{false};
		CHECK(vehicle->Adhesive(slippery) == doctest::Approx(slippery * 2.0 * 1.25));
	}
	// on, the sand gives back a share of the dry rail
	weather_physics_setting const on{true};
	CHECK(vehicle->Adhesive(slippery) == doctest::Approx(0.15 * defaults.sand * 2.0));
}

TEST_CASE("weather physics: off by default, the existing sceneries go as they did until the player turns it on") {
	CHECK(false == weather_physics::settings{}.enabled);
}

TEST_CASE("arc flicker: the same for the billboard and the light, not of the engine of Random(), 0.55..1") {
	std::uintptr_t const vehicle{0x12345678};
	// worked out, not drawn: the engine the network sessions keep in step isn't touched
	auto const before{Random()};
	test::reseed_random();
	auto const once{gfx::arc_flicker(vehicle, 0, 12.34)};
	auto const again{gfx::arc_flicker(vehicle, 0, 12.34)};
	CHECK(once == again);
	CHECK(Random() == before);
	// steady within a tick, another one in the next, at any frame rate
	CHECK(gfx::arc_flicker(vehicle, 0, 12.0) == gfx::arc_flicker(vehicle, 0, 12.0 + 0.5 / gfx::ARC_FLICKER_RATE));
	auto lowest{1.f};
	auto highest{0.f};
	auto changes{0};
	auto previous{gfx::arc_flicker(vehicle, 0, 0.0)};
	for (int tick = 1; tick < 3000; ++tick)
	{
		auto const value{gfx::arc_flicker(vehicle, 0, tick / gfx::ARC_FLICKER_RATE)};
		lowest = std::min(lowest, value);
		highest = std::max(highest, value);
		changes += (value != previous ? 1 : 0);
		previous = value;
	}
	CHECK(lowest >= 0.55f);
	CHECK(highest < 1.f);
	// it does flicker, over the whole range
	CHECK(lowest < 0.6f);
	CHECK(highest > 0.95f);
	CHECK(changes > 2900);
	// the pantographs of a vehicle, and the vehicles, flicker apart
	CHECK(gfx::arc_flicker(vehicle, 0, 12.34) != gfx::arc_flicker(vehicle, 1, 12.34));
	CHECK(gfx::arc_flicker(vehicle, 0, 12.34) != gfx::arc_flicker(vehicle + 64, 0, 12.34));
}
