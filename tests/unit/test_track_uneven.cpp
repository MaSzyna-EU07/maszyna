#include "stdafx.h"
#include "doctest.h"
#include "world/TrackUneven.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace {

double deviation(std::vector<double> const &Values)
{
	auto mean{0.0};
	for (auto const value : Values)
		mean += value;
	auto const count{static_cast<double>(Values.size())};
	mean /= count;
	auto sum{0.0};
	for (auto const value : Values)
		sum += (value - mean) * (value - mean);
	return std::sqrt(sum / count);
}

double peak(std::vector<double> Values)
{
	for (auto &value : Values)
		value = std::abs(value);
	std::ranges::sort(Values);
	return Values[Values.size() * 999 / 1000];
}

void add_chord(std::vector<double> &Output, std::vector<double> const &Line, int const Half)
{
	for (int i = Half; i + Half < static_cast<int>(Line.size()); ++i)
		Output.push_back(Line[i] - 0.5 * (Line[i - Half] + Line[i + Half]));
}

} // namespace

TEST_CASE("uneven track: the cab keeps what the ride on the track doesn't stand in for")
{
	auto const replaced = [](double const Up, double const Side, double const Cross) {
		return track_uneven::replaced_shake({Up, Side, Cross});
	};
	// all of it, as "uneven 18"
	CHECK(replaced(18.0, 18.0, 18.0).sway);
	CHECK(replaced(18.0, 18.0, 18.0).jolts);
	// nothing at all, as on a track with dips at its joints alone
	CHECK_FALSE(replaced(0.0, 0.0, 0.0).sway);
	CHECK_FALSE(replaced(0.0, 0.0, 0.0).jolts);
	// up and down only: there is nothing to sway with
	CHECK_FALSE(replaced(18.0, 0.0, 0.0).sway);
	CHECK(replaced(18.0, 0.0, 0.0).jolts);
	// sideways only
	CHECK(replaced(0.0, 18.0, 0.0).sway);
	CHECK(replaced(0.0, 18.0, 0.0).jolts);
	// cross level only turns the body
	CHECK_FALSE(replaced(0.0, 0.0, 18.0).sway);
	CHECK_FALSE(replaced(0.0, 0.0, 18.0).jolts);
}

TEST_CASE("uneven track: measures as the Id-14 limits for its value")
{
	// appendix 1, class C at 80 km/h: 18 mm vertically, 17 mm sideways, 16 mm of twist on 5 m, gauge +10/-8 mm, 20 mm of cross level, J = 5.3
	auto const amplitude{track_uneven::stray_per_mm * 18.0 * 1000.0}; // mm
	double const gauge{1.435};
	double const step{0.5};
	int const steps{4000};
	int const base{10}; // 5 m
	int const lines{36};
	double vertical{0.0};
	double horizontal{0.0};
	double twist{0.0};
	double wander{0.0};
	double level{0.0};
	double j{0.0};
	double away{0.0};
	for (int line = 0; line < lines; ++line)
	{
		auto const heading{glm::pi<double>() * (line + 0.37) / lines};
		glm::dvec3 const start{7919.0 * std::sin(line * 1.7), 0.0, 6577.0 * std::cos(line * 2.3)};
		glm::dvec3 const along{std::cos(heading), 0.0, std::sin(heading)};
		glm::dvec3 const across{-along.z, 0.0, along.x};
		std::array<std::vector<double>, 2> up;
		std::array<std::vector<double>, 2> side;
		std::vector<double> middle;
		for (int i = 0; i < steps; ++i)
		{
			auto const point{start + along * (i * step)};
			for (int rail = 0; rail < 2; ++rail)
			{
				auto const sign{rail == 0 ? 1.0 : -1.0};
				auto const stray{track_uneven::field(point + across * (sign * 0.5 * gauge))};
				up[rail].push_back((stray.y - sign * 0.5 * track_uneven::level(point, across)) * amplitude);
				side[rail].push_back(glm::dot(stray, across) * amplitude);
			}
			middle.push_back(track_uneven::field(point).y * amplitude);
		}
		std::vector<double> verticals;
		std::vector<double> horizontals;
		std::vector<double> twists;
		std::vector<double> gauges;
		std::vector<double> levels;
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
	CHECK(away < 40.0); // the long waves go past the 15 mm off the survey marks of appendix 1a
}

TEST_CASE("uneven track: a place strays the same every time")
{
	// no random numbers: changing these changes how every uneven track in every scenery looks
	glm::dvec3 const place{1234.5, 0.0, -678.9};
	auto const stray{track_uneven::field(place)};
	CHECK(stray.x == doctest::Approx(0.309777548158).epsilon(1e-9));
	CHECK(stray.y == doctest::Approx(0.311350299745).epsilon(1e-9));
	CHECK(stray.z == doctest::Approx(-0.328429734019).epsilon(1e-9));
	CHECK(track_uneven::level(place, {1.0, 0.0, 0.0}) == doctest::Approx(0.53 * -0.178598957983).epsilon(1e-9));
	CHECK(track_uneven::level(place, {0.0, 0.0, 1.0}) == doctest::Approx(0.53 * -0.868291889902).epsilon(1e-9));
}

TEST_CASE("uneven track: tracks laid towards each other meet at one cant")
{
	// the same rail stays lower seen from either of them, and a path taken the other way sees the roll turned over
	glm::dvec3 const place{812.25, 0.0, 3071.5};
	glm::dvec3 const across{0.6, 0.0, -0.8};
	CHECK(track_uneven::level(place, -across) == doctest::Approx(-track_uneven::level(place, across)));
	CHECK(track_uneven::roll(place, -across, 1.435) == doctest::Approx(-track_uneven::roll(place, across, 1.435)));
	// and there's no step along a curve turning through the headings
	auto last{track_uneven::level(place, {1.0, 0.0, 0.0})};
	for (int step = 1; step <= 3600; ++step)
	{
		auto const heading{glm::two_pi<double>() * step / 3600.0};
		auto const value{track_uneven::level(place, {std::cos(heading), 0.0, std::sin(heading)})};
		REQUIRE(std::abs(value - last) < 0.002);
		last = value;
	}
}

TEST_CASE("uneven track: the field stays within its bound")
{
	auto most{0.0};
	auto highest{0.0};
	for (int i = 0; i < 20000; ++i)
	{
		auto const stray{track_uneven::field({i * 3.7 - 30000.0, 0.0, i * -2.9 + 41000.0})};
		most = std::max({most, std::abs(stray.x), std::abs(stray.z)});
		highest = std::max(highest, std::abs(stray.y));
	}
	CHECK(most <= 1.0);
	CHECK(most > 0.5);
	CHECK(highest <= 1.69);
	CHECK(highest > 0.5);
}

TEST_CASE("uneven track: all the values in one attribute")
{
	auto const one{track_uneven::read_fields<6>("18")};
	CHECK(one[0] == doctest::Approx(18.f));
	CHECK(one[1] < 0.f);
	auto const both{track_uneven::read_fields<6>("14;35")};
	CHECK(both[0] == doctest::Approx(14.f));
	CHECK(both[1] == doctest::Approx(35.f));
	CHECK(both[2] < 0.f);
	auto const gaps{track_uneven::read_fields<6>("18.5;;;25;6")};
	CHECK(gaps[0] == doctest::Approx(18.5f));
	CHECK(gaps[1] < 0.f);
	CHECK(gaps[2] < 0.f);
	CHECK(gaps[3] == doctest::Approx(25.f));
	CHECK(gaps[4] == doctest::Approx(6.f));
	auto const bounds{track_uneven::read_fields<6>("500;-3;x")};
	CHECK(bounds[0] == doctest::Approx(track_uneven::most));
	CHECK(bounds[1] == doctest::Approx(0.f));
	CHECK(bounds[2] < 0.f);
	CHECK(track_uneven::write_fields<6>({14.f, 35.f, -1.f, -1.f, -1.f, -1.f}) == "14;35");
	CHECK(track_uneven::write_fields<6>({18.5f, -1.f, -1.f, 25.f, 6.f, -1.f}) == "18.5;;;25;6");
	CHECK(track_uneven::write_fields<6>({0.f, -1.f, 30.f, -1.f, -1.f, -1.f}) == "0;;30");
	CHECK(track_uneven::write_fields<6>({18.f, -1.f, -1.f, -1.f, -1.f, -1.f}) == "18");
}

TEST_CASE("uneven track: the attribute goes by the entry it's written into")
{
	auto const plain{track_uneven::read_attribute("18;22;19;25;6", false)};
	CHECK(track_uneven::write_attribute(plain, false) == "uneven 18;22;19;25;6 ");
	// a switch made in the style of a plain track: no path p3-p4 of its own, the components in their places
	CHECK(track_uneven::write_attribute(plain, true) == "uneven 18;;22;19 ");
	auto const made{track_uneven::read_attribute("18;;22;19", true)};
	CHECK(track_uneven::declared(made, false, 0).up == doctest::Approx(22.0));
	CHECK(track_uneven::declared(made, false, 0).side == doctest::Approx(19.0));
	CHECK(track_uneven::declared(made, false, 1).up == doctest::Approx(22.0));
	CHECK(track_uneven::declared(made, false, 0).cross == doctest::Approx(0.0));
	// a switch with its own values, and a plain track in its style
	auto const points{track_uneven::read_attribute("14;35;16;17", true)};
	CHECK(track_uneven::write_attribute(points, true) == "uneven 14;35;16;17 ");
	CHECK(track_uneven::declared(points, false, 1).up == doctest::Approx(35.0));
	CHECK(track_uneven::declared(points, false, 1).side == doctest::Approx(35.0));
	CHECK(track_uneven::write_attribute(points, false) == "uneven 14;16;17 ");
	// a plain track: the value stands in for what's left out, only there is the cross level
	auto const track{track_uneven::read_attribute("18;;30", false)};
	CHECK(track_uneven::declared(track, true, 0).up == doctest::Approx(18.0));
	CHECK(track_uneven::declared(track, true, 0).side == doctest::Approx(30.0));
	CHECK(track_uneven::declared(track, true, 0).cross == doctest::Approx(18.0));
	// nothing given, nothing written
	CHECK(track_uneven::write_attribute({}, false).empty());
	CHECK(track_uneven::write_attribute({}, true).empty());
	// a value of 0 stays when there's more after it
	CHECK(track_uneven::write_attribute(track_uneven::read_attribute("0;;;;6", false), false) == "uneven 0;;;;6 ");
}

TEST_CASE("uneven track: each component comes to its neighbours along a track")
{
	double const length{500.0};
	auto const atstart{track_uneven::along(track_uneven::amounts{18.0, 16.0, 14.0}, {5.0, 4.0, 3.0}, {7.0, 8.0, 9.0}, 0.0, length)};
	CHECK(atstart.up == doctest::Approx(5.0));
	CHECK(atstart.side == doctest::Approx(4.0));
	CHECK(atstart.cross == doctest::Approx(3.0));
	auto const inside{track_uneven::along(track_uneven::amounts{18.0, 16.0, 14.0}, {5.0, 4.0, 3.0}, {7.0, 8.0, 9.0}, 0.5 * length, length)};
	CHECK(inside.up == doctest::Approx(18.0));
	CHECK(inside.side == doctest::Approx(16.0));
	CHECK(inside.cross == doctest::Approx(14.0));
}

TEST_CASE("uneven track: axles don't stray along the track")
{
	glm::dvec3 const across{0.6, 0.0, -0.8};
	glm::dvec3 const along{0.8, 0.0, 0.6};
	auto const moved{track_uneven::across_only(across * 0.03 + along * 0.05 + glm::dvec3{0.0, 0.02, 0.0}, across)};
	CHECK(glm::dot(moved, along) == doctest::Approx(0.0));
	CHECK(glm::dot(moved, across) == doctest::Approx(0.03));
	CHECK(moved.y == doctest::Approx(0.02));
}

TEST_CASE("uneven track: a path of no length gives no vector across")
{
	glm::dvec3 const none{0.0};
	CHECK(track_uneven::across_of(none, none) == none);
	CHECK(glm::length(track_uneven::across_of({3.0, 0.0, 4.0}, none)) == doctest::Approx(1.0));
	// it can't pass for a joint: the check of the two vectors being alike fails
	CHECK(glm::dot(track_uneven::across_of(none, none), glm::dvec3{1.0, 0.0, 0.0}) < 0.9);
	// falls back on the chord
	auto const chord{track_uneven::across_of(none, {0.0, 0.0, 10.0})};
	CHECK(chord.x == doctest::Approx(-1.0));
	CHECK(chord.z == doctest::Approx(0.0));
	// straight up has no direction in the plan
	CHECK(glm::length(track_uneven::across_of({0.0, 5.0, 0.0}, {0.0, 0.0, 10.0})) == doctest::Approx(1.0));
}

TEST_CASE("uneven track: the cant at a joint")
{
	CHECK(track_uneven::joint_roll(0.1f, -0.1f, true, true) == doctest::Approx(0.f));
	CHECK(track_uneven::joint_roll(0.2f, 0.4f, false, false) == doctest::Approx(0.3f));
	CHECK(track_uneven::joint_roll(0.1f, 0.0f, true, false) == doctest::Approx(0.f));
	CHECK(track_uneven::joint_roll(0.0f, 0.1f, false, true) == doctest::Approx(0.f));
}

TEST_CASE("uneven track: amounts take the lowest of each component")
{
	auto const least{track_uneven::least({14.0, 9.0, 0.0}, {7.0, 35.0, 3.0})};
	CHECK(least.up == doctest::Approx(7.0));
	CHECK(least.side == doctest::Approx(9.0));
	CHECK(least.cross == doctest::Approx(0.0));
	// the diverging path of a switch: from the common end at the lowest to its own value at the far end
	double const length{30.0};
	CHECK(track_uneven::along(35.0, 7.0, 35.0, 0.0, length) == doctest::Approx(7.0));
	CHECK(track_uneven::along(35.0, 7.0, 35.0, length, length) == doctest::Approx(35.0));
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
	for (int step = 1; step <= 2000; ++step)
	{
		auto const value{track_uneven::along(18.0, 35.0, 0.0, step * 0.25, length)};
		REQUIRE(std::abs(value - last) < 0.5);
		last = value;
	}
}

TEST_CASE("uneven track: the foot of the ballast stays where it was")
{
	double const rail{0.18};
	double const height{0.38};
	CHECK(track_uneven::bed_share(0.0, rail, height) == doctest::Approx(1.0)); // rail top
	CHECK(track_uneven::bed_share(rail, rail, height) == doctest::Approx(1.0)); // top of the ballast
	CHECK(track_uneven::bed_share(rail + 0.5 * height, rail, height) == doctest::Approx(0.5));
	CHECK(track_uneven::bed_share(rail + height, rail, height) == doctest::Approx(0.0)); // its foot
	CHECK(track_uneven::bed_share(rail + 2.0 * height, rail, height) == doctest::Approx(0.0));
	CHECK(track_uneven::bed_share(rail + 1.0, rail, 0.0) == doctest::Approx(1.0)); // no ballast
}

TEST_CASE("uneven track: the rails dip around a joint")
{
	CHECK(track_uneven::dip(0.0) == doctest::Approx(1.0));
	CHECK(track_uneven::dip(1.0) == doctest::Approx(0.75).epsilon(0.01)); // the worst case for rings 2 m apart
	CHECK(track_uneven::dip(track_uneven::dip_reach) == doctest::Approx(0.0));
	CHECK(track_uneven::rail_phase(26.0, 25.0) == doctest::Approx(1.0));
	CHECK(track_uneven::rail_phase(-1.0, 25.0) == doctest::Approx(24.0));
}
