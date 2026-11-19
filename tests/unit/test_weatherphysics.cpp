#include "stdafx.h"
#include "doctest.h"
#include "environment/railsurface.h"
#include "McZapkie/MOVER.h"
#include "McZapkie/weatherphysics.h"
#include "utilities/Globals.h"

#include <memory>

// the weather in the physics of the vehicles (McZapkie/weatherphysics.h, environment/railsurface.h, physics.weather)

namespace {

weather_physics::settings const defaults{};

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
