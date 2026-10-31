#include "stdafx.h"
#include "doctest.h"
#include "world/TrackUneven.h"

#include <cmath>
#include <vector>

namespace {

double deviation(std::vector<double> const &Values)
{
	auto mean{0.0};
	for (auto const value : Values)
		mean += value;
	mean /= Values.size();
	auto sum{0.0};
	for (auto const value : Values)
		sum += (value - mean) * (value - mean);
	return std::sqrt(sum / Values.size());
}

double peak(std::vector<double> Values)
{
	for (auto &value : Values)
		value = std::abs(value);
	std::sort(Values.begin(), Values.end());
	return Values[static_cast<std::size_t>(0.999 * Values.size())];
}

void add_chord(std::vector<double> &Output, std::vector<double> const &Line, int const Half)
{
	for (int i = Half; i + Half < static_cast<int>(Line.size()); ++i)
		Output.push_back(Line[i] - 0.5 * (Line[i - Half] + Line[i + Half]));
}

} // namespace

TEST_CASE("uneven track: measures as the Id-14 limits for its value")
{
	// appendix 1, class C at 80 km/h: 18 mm vertically, 17 mm sideways, 16 mm of twist on 5 m, gauge +10/-8 mm, 20 mm of cross level, J = 5.3
	auto const amplitude{track_uneven::stray_per_mm * 18.0 * 1000.0}; // mm
	double const gauge{1.435};
	double const step{0.5};
	int const steps{4000};
	int const base{10}; // 5 m
	int const lines{36};
	double vertical{0.0}, horizontal{0.0}, twist{0.0}, wander{0.0}, level{0.0}, j{0.0}, away{0.0};
	for (int line = 0; line < lines; ++line)
	{
		auto const heading{glm::pi<double>() * (line + 0.37) / lines};
		glm::dvec3 const start{7919.0 * std::sin(line * 1.7), 0.0, 6577.0 * std::cos(line * 2.3)};
		glm::dvec3 const along{std::cos(heading), 0.0, std::sin(heading)};
		glm::dvec3 const across{-along.z, 0.0, along.x};
		std::vector<double> up[2], side[2], middle;
		for (int i = 0; i < steps; ++i)
		{
			auto const point{start + along * (i * step)};
			for (int rail = 0; rail < 2; ++rail)
			{
				auto const sign{rail == 0 ? 1.0 : -1.0};
				auto const stray{track_uneven::field(point + across * (sign * 0.5 * gauge))};
				up[rail].push_back((stray.y - sign * 0.5 * track_uneven::level(point)) * amplitude);
				side[rail].push_back(glm::dot(stray, across) * amplitude);
			}
			middle.push_back(track_uneven::field(point).y * amplitude);
		}
		std::vector<double> verticals, horizontals, twists, gauges, levels;
		for (int rail = 0; rail < 2; ++rail)
		{
			add_chord(verticals, up[rail], base);
			add_chord(horizontals, side[rail], base);
		}
		for (int i = 0; i + base < steps; ++i)
			twists.push_back((up[0][i + base] - up[1][i + base]) - (up[0][i] - up[1][i]));
		for (int i = 0; i < steps; ++i)
		{
			gauges.push_back(side[0][i] - side[1][i]);
			levels.push_back(up[0][i] - up[1][i]);
		}
		vertical += peak(verticals) / lines;
		horizontal += peak(horizontals) / lines;
		twist += peak(twists) / lines;
		wander += peak(gauges) / lines;
		level += peak(levels) / lines;
		away += peak(middle) / lines;
		j += (deviation(verticals) + deviation(horizontals) + deviation(twists) + 0.5 * deviation(gauges)) / 3.5 / lines;
	}
	CHECK(vertical == doctest::Approx(18.0).epsilon(0.1));
	CHECK(horizontal == doctest::Approx(17.0).epsilon(0.2));
	CHECK(twist == doctest::Approx(16.0).epsilon(0.2));
	CHECK(wander == doctest::Approx(10.0).epsilon(0.2));
	CHECK(level == doctest::Approx(20.0).epsilon(0.1));
	CHECK(j == doctest::Approx(5.3).epsilon(0.2));
	CHECK(away < 25.0); // appendix 1a: 15 mm off the survey marks
}

TEST_CASE("uneven track: a place strays the same every time")
{
	// no random numbers: changing these changes how every uneven track in every scenery looks
	glm::dvec3 const place{1234.5, 0.0, -678.9};
	auto const stray{track_uneven::field(place)};
	CHECK(stray.x == doctest::Approx(0.309777548158).epsilon(1e-9));
	CHECK(stray.y == doctest::Approx(-0.295557487607).epsilon(1e-9));
	CHECK(stray.z == doctest::Approx(-0.328429734019).epsilon(1e-9));
	CHECK(track_uneven::level(place) == doctest::Approx(0.6 * -0.178598957983).epsilon(1e-9));
}

TEST_CASE("uneven track: the field stays within its bound")
{
	auto most{0.0};
	for (int i = 0; i < 20000; ++i)
	{
		auto const stray{track_uneven::field({i * 3.7 - 30000.0, 0.0, i * -2.9 + 41000.0})};
		most = std::max({most, std::abs(stray.x), std::abs(stray.y), std::abs(stray.z)});
	}
	CHECK(most <= 1.0);
	CHECK(most > 0.5);
}

TEST_CASE("uneven track: the value along a track meets its neighbours exactly")
{
	double const length{500.0};
	CHECK(track_uneven::along(20.0, 5.0, 7.0, 0.0, length) == doctest::Approx(5.0));
	CHECK(track_uneven::along(20.0, 5.0, 7.0, length, length) == doctest::Approx(7.0));
	CHECK(track_uneven::along(20.0, 5.0, 7.0, 0.5 * length, length) == doctest::Approx(20.0));
	// equal values stay constant
	CHECK(track_uneven::along(20.0, 20.0, 20.0, 3.0, length) == doctest::Approx(20.0));
	// a short track between ends of no value hardly strays
	CHECK(track_uneven::along(20.0, 0.0, 0.0, 2.5, 5.0) < 1.0);
	// a switch of more than the track: the track starts at the value of the switch
	CHECK(track_uneven::along(18.0, 35.0, 18.0, 0.0, length) == doctest::Approx(35.0));
	CHECK(track_uneven::along(18.0, 35.0, 18.0, track_uneven::fade, length) == doctest::Approx(18.0));
	// the short piece inside a slip goes with its switches
	CHECK(track_uneven::along(18.0, 35.0, 35.0, 0.25, 0.5) == doctest::Approx(35.0).epsilon(0.01));
	// no jump anywhere
	auto last{track_uneven::along(18.0, 35.0, 0.0, 0.0, length)};
	for (auto distance{0.25}; distance <= length; distance += 0.25)
	{
		auto const value{track_uneven::along(18.0, 35.0, 0.0, distance, length)};
		REQUIRE(std::abs(value - last) < 0.5);
		last = value;
	}
}

TEST_CASE("uneven track: the rails dip around a joint")
{
	CHECK(track_uneven::dip(0.0) == doctest::Approx(1.0));
	CHECK(track_uneven::dip(1.0) == doctest::Approx(0.75).epsilon(0.01)); // the worst case for rings 2 m apart
	CHECK(track_uneven::dip(track_uneven::dip_reach) == doctest::Approx(0.0));
	CHECK(track_uneven::rail_phase(26.0, 25.0) == doctest::Approx(1.0));
	CHECK(track_uneven::rail_phase(-1.0, 25.0) == doctest::Approx(24.0));
}
