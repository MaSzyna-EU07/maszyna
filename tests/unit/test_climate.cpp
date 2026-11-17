#include "stdafx.h"
#include "doctest.h"
#include "environment/climate.h"
#include "environment/psychrometry.h"
#include "environment/weather.h"

#include <algorithm>
#include <cmath>

namespace {

bool has_cloud(weather_conditions const &Weather, cloud_type const Type) {
	return std::ranges::any_of(Weather.clouds, [Type](cloud_layer const &Layer) { return Layer.type == Type; });
}

// what falls at the mean temperature of the day of a weather
psychrometry::falling falls(weather_conditions const &Weather) {
	auto const temperature{*Weather.temperature};
	std::optional<bool> rain;
	if (Weather.precipitation != precipitation_type::automatic)
		rain = (Weather.precipitation == precipitation_type::rain);
	return psychrometry::falling_kind(temperature, psychrometry::wet_bulb(temperature, psychrometry::relative_humidity(temperature, *Weather.dew_point)), rain);
}

} // namespace

TEST_CASE("psychrometry: humidity, dew point and wet bulb agree with the tables") {
	CHECK(psychrometry::relative_humidity(20.f, 20.f) == doctest::Approx(1.f));
	CHECK(psychrometry::dew_point(20.f, 0.5f) == doctest::Approx(9.3f).epsilon(0.03));
	CHECK(psychrometry::relative_humidity(20.f, psychrometry::dew_point(20.f, 0.5f)) == doctest::Approx(0.5f).epsilon(0.01));
	CHECK(psychrometry::wet_bulb(20.f, 0.5f) == doctest::Approx(13.7f).epsilon(0.03));
	CHECK(psychrometry::wet_bulb(-5.f, 1.f) == doctest::Approx(-5.f));
	// never above the air temperature
	CHECK(psychrometry::wet_bulb(-10.f, 0.95f) <= -10.f);
}

TEST_CASE("psychrometry: snow falls in dry air above zero, rain in saturated air just above it") {
	using psychrometry::falling;
	// 2 degrees: snow at 40% (wet bulb about -2), rain at 100%
	CHECK(psychrometry::falling_kind(2.f, psychrometry::wet_bulb(2.f, 0.4f), std::nullopt) == falling::snow);
	CHECK(psychrometry::falling_kind(2.f, psychrometry::wet_bulb(2.f, 1.f), std::nullopt) == falling::rain);
	// just above zero in moist air: wet snow
	CHECK(psychrometry::falling_kind(0.6f, psychrometry::wet_bulb(0.6f, 0.98f), std::nullopt) == falling::wet_snow);
	// without the moisture the air temperature decides at 1 degree, as before
	CHECK(psychrometry::is_snow(psychrometry::falling_kind(1.f, std::nullopt, std::nullopt)));
	CHECK(psychrometry::falling_kind(1.5f, std::nullopt, std::nullopt) == falling::rain);
	// a description saying rain in a frost: freezing rain; saying snow in the warmth: wet snow
	CHECK(psychrometry::falling_kind(-2.f, std::nullopt, true) == falling::freezing_rain);
	CHECK(psychrometry::falling_kind(3.f, std::nullopt, false) == falling::wet_snow);
}

TEST_CASE("psychrometry: the temperature of the day is lowest about sunrise and highest in the afternoon") {
	CHECK(psychrometry::diurnal_temperature(-4.f, 6.f, 5.f) == doctest::Approx(-4.f));
	CHECK(psychrometry::diurnal_temperature(-4.f, 6.f, 15.f) == doctest::Approx(6.f));
	CHECK(psychrometry::diurnal_temperature(-4.f, 6.f, 10.f) == doctest::Approx(1.f));
	// continuous over midnight, falling through the night
	auto const evening{psychrometry::diurnal_temperature(-4.f, 6.f, 23.9f)};
	auto const night{psychrometry::diurnal_temperature(-4.f, 6.f, 0.1f)};
	CHECK(std::abs(evening - night) < 0.3f);
	CHECK(night > psychrometry::diurnal_temperature(-4.f, 6.f, 3.f));
	for (float hour = 0.f; hour < 24.f; hour += 0.5f) {
		auto const value{psychrometry::diurnal_temperature(-4.f, 6.f, hour)};
		CHECK(value >= -4.f);
		CHECK(value <= 6.f);
	}
}

TEST_CASE("climate: the months of the default station are the ones of the middle of Poland") {
	auto const &station{climate::default_station()};
	CHECK(station.months[0].temperature() == doctest::Approx(-1.4f));
	CHECK(station.months[6].temperature() == doctest::Approx(19.25f));
	for (auto const &rose : station.roses) {
		auto total{0.f};
		for (auto const share : rose)
			total += share;
		CHECK(total == doctest::Approx(1.f));
	}
}

TEST_CASE("climate: every month has its kinds of weather, the winter ones in winter and the summer ones in summer") {
	auto const &station{climate::default_station()};
	for (int month = 0; month < 12; ++month) {
		CAPTURE(month);
		CHECK(climate::weight(station, month, climate::preset::typical) > 0.f);
		CHECK(climate::weight(station, month, climate::preset::precipitation) > 0.f);
		// every number draws a kind which comes in the month
		for (float random = 0.f; random <= 1.f; random += 0.05f)
			CHECK(climate::weight(station, month, climate::pick(station, month, random)) > 0.f);
	}
	CHECK(climate::weight(station, 0, climate::preset::blizzard) > 0.f);
	CHECK(climate::weight(station, 0, climate::preset::frost) > 0.f);
	CHECK(climate::weight(station, 0, climate::preset::heat) == 0.f);
	CHECK(climate::weight(station, 0, climate::preset::storm) == 0.f);
	CHECK(climate::weight(station, 6, climate::preset::blizzard) == 0.f);
	CHECK(climate::weight(station, 6, climate::preset::frost) == 0.f);
	CHECK(climate::weight(station, 6, climate::preset::storm) > climate::weight(station, 3, climate::preset::storm));
	CHECK(climate::weight(station, 9, climate::preset::fog) > climate::weight(station, 5, climate::preset::fog));
}

TEST_CASE("climate: a January blizzard is snow in a gale over deep drifting snow") {
	auto const weather{climate::make(climate::default_station(), 0, climate::preset::blizzard, 7)};
	CHECK(*weather.temperature_max <= -2.f);
	CHECK(weather.wind_described);
	CHECK(weather.wind_speed >= 10.f);
	CHECK(weather.wind_gust > weather.wind_speed);
	CHECK(weather.precipitation_intensity > 0.5f);
	CHECK(falls(weather) == psychrometry::falling::snow);
	CHECK(*weather.snow_depth > 0.1f);
	CHECK(weather.snow_drift > 0.5f);
	CHECK(weather.visibility < 1500.f);
}

TEST_CASE("climate: a July storm is a hot day with a cumulonimbus, heavy rain and gusts, and no snow") {
	auto const weather{climate::make(climate::default_station(), 6, climate::preset::storm, 3)};
	CHECK(*weather.temperature_max > 22.f);
	CHECK(has_cloud(weather, cloud_type::cumulonimbus));
	CHECK(falls(weather) == psychrometry::falling::rain);
	CHECK(weather.wind_gust > 2.f * weather.wind_speed);
	CHECK(*weather.snow_depth == 0.f);
}

TEST_CASE("climate: the precipitation of a month falls as its temperatures have it") {
	auto const &station{climate::default_station()};
	CHECK(psychrometry::is_snow(falls(climate::make(station, 0, climate::preset::precipitation, 1))));
	CHECK(falls(climate::make(station, 4, climate::preset::precipitation, 1)) == psychrometry::falling::rain);
}

TEST_CASE("climate: a frost is far below the average of the month, a thaw above zero over old snow") {
	auto const &station{climate::default_station()};
	auto const frost{climate::make(station, 0, climate::preset::frost, 11)};
	CHECK(*frost.temperature_min < station.months[0].temperature_min - 5.f);
	CHECK(*frost.snow_depth > 0.f);
	// from the east, give or take
	CHECK(frost.wind_direction > 40.f);
	CHECK(frost.wind_direction < 120.f);
	auto const thaw{climate::make(station, 0, climate::preset::thaw, 11)};
	CHECK(*thaw.temperature_min >= 1.f);
	CHECK(thaw.snow_fresh == 0.f);
	CHECK(falls(thaw) == psychrometry::falling::rain);
}

TEST_CASE("climate: the same seed makes the same day, another seed another one") {
	auto const &station{climate::default_station()};
	auto const first{climate::make(station, 9, climate::preset::typical, 42)};
	auto const again{climate::make(station, 9, climate::preset::typical, 42)};
	auto const other{climate::make(station, 9, climate::preset::typical, 43)};
	CHECK(*first.temperature_min == *again.temperature_min);
	CHECK(first.wind_direction == again.wind_direction);
	CHECK(((*first.temperature_min != *other.temperature_min) || (first.wind_direction != other.wind_direction)));
	// the wind of the typical days comes mostly from the west and the south-west, as the rose has it
	int westerly{0};
	for (std::uint32_t seed = 1; seed <= 400; ++seed) {
		auto const direction{climate::make(station, 9, climate::preset::typical, seed).wind_direction};
		if ((direction > 200.f) && (direction < 295.f))
			++westerly;
	}
	CHECK(westerly > 120);
	CHECK(westerly < 280);
}

TEST_CASE("climate: the table of the stations is read and the nearest station found") {
	climate::table table;
	CHECK(table.parse("# stations\n"
	                  "station Suwalki 54.13 22.95 184\n"
	                  "month 1 -6.5 -1.2 88 3.9 34 10 24 18 4 0 6.4\n"
	                  "rose winter 0.05 0.05 0.1 0.15 0.15 0.2 0.2 0.1\n"
	                  "station Wroclaw 51.10 16.88 120\n"
	                  "month 1 -2.6 3.0 84 3.6 30 8 9 6 4 0 6.2\n"
	                  "month 13 0 0 0 0 0 0 0 0 0 0 0\n"));
	REQUIRE(table.stations().size() == 2);
	auto const &suwalki{table.nearest(53.8f, 22.5f)};
	CHECK(suwalki.name == "Suwalki");
	CHECK(suwalki.months[0].temperature_min == doctest::Approx(-6.5f));
	CHECK(suwalki.months[0].snow_days == doctest::Approx(24.f));
	CHECK(suwalki.roses[0][5] == doctest::Approx(0.2f));
	// the months a station leaves out are those of the default one
	CHECK(suwalki.months[6].temperature_max == doctest::Approx(climate::default_station().months[6].temperature_max));
	CHECK(table.nearest(51.2f, 17.0f).name == "Wroclaw");
	// far from every station: the default one
	CHECK(table.nearest(49.3f, 22.5f, 100.f).name == climate::default_station().name);
	climate::table empty;
	CHECK_FALSE(empty.parse("# nothing\n"));
	CHECK(empty.nearest(52.f, 19.f).name == climate::default_station().name);
}
