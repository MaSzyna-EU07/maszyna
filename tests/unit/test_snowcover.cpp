#include "stdafx.h"
#include "doctest.h"
#include "environment/psychrometry.h"
#include "environment/snowcover.h"

namespace {

// Hours of the weather given, in steps of a minute
void run(snow_cover &Snow, snow_weather const &Weather, double const Hours) {
	auto const steps{static_cast<int>(Hours * 60.0)};
	for (int step = 0; step < steps; ++step)
		Snow.update(60.0, Weather);
}

snow_weather frost(float const Snowfall, glm::vec2 const &Wind) {
	snow_weather weather;
	weather.temperature = -5.f;
	weather.wet_bulb = psychrometry::wet_bulb(-5.f, weather.humidity);
	weather.snowfall = Snowfall;
	weather.wind = Wind;
	return weather;
}

} // namespace

TEST_CASE("snow cover: an hour of the heaviest snowfall of a frost lays some 7 cm of loose snow") {
	snow_cover snow;
	run(snow, frost(1.f, {0.f, 0.f}), 1.0);
	CAPTURE(snow.depth());
	CHECK(snow.depth() > 0.05f);
	CHECK(snow.depth() < 0.08f);
	CHECK(snow.fresh() > 0.9f);
	CHECK(snow.falling() > 0.9f);
	// in a calm nothing is worked into drifts, nothing is stuck to the faces turned to the wind
	CHECK(snow.drift() < 0.01f);
	CHECK(snow.plaster() < 0.01f);
}

TEST_CASE("snow cover: a strong wind works the loose snow into drifts turned to it, a light one doesn't") {
	snow_cover light;
	light.set(0.3f, 0.f, {0.f, 3.f}, 1.f);
	run(light, frost(0.f, {3.f, 0.f}), 3.0);
	CHECK(light.drift() < 0.01f);
	CHECK(light.blowing() < 0.05f);

	snow_cover strong;
	strong.set(0.3f, 0.f, {0.f, 3.f}, 1.f);
	run(strong, frost(0.f, {12.f, 0.f}), 3.0);
	CAPTURE(strong.drift());
	CHECK(strong.drift() > 0.3f);
	CHECK(strong.blowing() > 0.2f);
	// the drifts turned to the wind blowing along x
	auto const direction{glm::normalize(strong.drift_wind())};
	CHECK(direction.x > 0.9f);
	// the snow carried packs, less of it is loose
	CHECK(strong.fresh() < 1.f);
}

TEST_CASE("snow cover: new snow falling in a calm evens the drifts out") {
	snow_cover snow;
	snow.set(0.1f, 0.8f, {10.f, 0.f}, 0.2f);
	run(snow, frost(1.f, {0.f, 0.f}), 2.0);
	CAPTURE(snow.drift());
	CHECK(snow.drift() < 0.4f);
}

TEST_CASE("snow cover: a thaw melts the snow and wets it, a frost lets it be") {
	snow_cover thaw;
	thaw.set(0.2f, 0.f, {0.f, 1.f}, 0.f);
	snow_weather warm;
	warm.temperature = 5.f;
	warm.wet_bulb = psychrometry::wet_bulb(5.f, warm.humidity);
	run(thaw, warm, 24.0);
	CAPTURE(thaw.depth());
	CHECK(thaw.depth() < 0.2f);
	CHECK(thaw.depth() > 0.05f);
	CHECK(thaw.wetness() > 0.9f);

	snow_cover cold;
	cold.set(0.2f, 0.f, {0.f, 1.f}, 0.f);
	run(cold, frost(0.f, {0.f, 0.f}), 24.0);
	CHECK(cold.depth() == doctest::Approx(0.2f).epsilon(0.01));
	CHECK(cold.wetness() < 0.01f);
}

TEST_CASE("snow cover: wet snow in a wind sticks to the faces turned to it") {
	snow_cover snow;
	snow_weather weather;
	weather.temperature = 0.3f;
	weather.humidity = 0.98f;
	weather.wet_bulb = psychrometry::wet_bulb(0.3f, weather.humidity);
	weather.snowfall = 0.8f;
	weather.wind = {0.f, 10.f};
	run(snow, weather, 2.0);
	CAPTURE(snow.plaster());
	CHECK(snow.plaster() > 0.5f);
	CHECK(glm::normalize(snow.plaster_wind()).y > 0.9f);
}

TEST_CASE("snow cover: freezing rain glazes the wires, a thaw takes the ice off") {
	snow_cover snow;
	snow_weather rain;
	rain.temperature = -2.f;
	rain.wet_bulb = psychrometry::wet_bulb(-2.f, rain.humidity);
	rain.rainfall = 1.f;
	run(snow, rain, 3.0);
	CAPTURE(snow.wire_ice());
	// about 1.3 mm an hour, less what sublimates
	CHECK(snow.wire_ice() > 0.003f);
	CHECK(snow.wire_ice() < 0.005f);
	CHECK(snow.wire_ice_density() == doctest::Approx(900.f));
	snow_weather warm;
	warm.temperature = 4.f;
	warm.wet_bulb = psychrometry::wet_bulb(4.f, warm.humidity);
	run(snow, warm, 2.0);
	CHECK(snow.wire_ice() == 0.f);
}

TEST_CASE("snow cover: the time of the snow can go faster than the simulation") {
	snow_cover slow;
	snow_cover fast;
	fast.time_scale = 60.f;
	run(slow, frost(1.f, {0.f, 0.f}), 1.0);
	run(fast, frost(1.f, {0.f, 0.f}), 1.0 / 60.0);
	CHECK(fast.depth() == doctest::Approx(slow.depth()).epsilon(0.02));
	// set by hand and reset
	fast.set(0.5f, 0.7f, {5.f, 0.f}, 0.4f);
	CHECK(fast.depth() == doctest::Approx(0.5f));
	CHECK(fast.drift() == doctest::Approx(0.7f));
	fast.reset();
	CHECK(fast.depth() == 0.f);
	CHECK(fast.time_scale == doctest::Approx(60.f));
}

TEST_CASE("snow cover: dry air takes the snow stuck to the windward faces sooner than moist air") {
	snow_cover dry;
	snow_cover moist;
	snow_weather sticking;
	sticking.temperature = -0.5f;
	sticking.wet_bulb = -0.5f;
	sticking.snowfall = 0.8f;
	sticking.wind = {0.f, 10.f};
	run(dry, sticking, 2.0);
	run(moist, sticking, 2.0);
	auto after{frost(0.f, {0.f, 6.f})};
	after.humidity = 0.4f;
	run(dry, after, 12.0);
	after.humidity = 0.98f;
	run(moist, after, 12.0);
	CHECK(dry.plaster() < moist.plaster() * 0.8f);
}
