#include "stdafx.h"
#include "doctest.h"
#include "editor/editorSwitchCurve.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace {

// a line every half metre: a straight, a transition curve from it to the arc of the radius, with a straight cant ramp along it, and
// the arc, each of the lengths given
std::vector<switch_curve::track_point> line(double const Straight, double const Transition, double const Arc, double const Radius, double const Cant) {
	std::vector<switch_curve::track_point> points;
	double heading{0.0};
	auto const curvature = [&](double const S) { return S <= Straight ? 0.0 : S <= Straight + Transition ? (S - Straight) / Transition / Radius : 1.0 / Radius; };
	auto const cant = [&](double const S) { return S <= Straight ? 0.0 : S <= Straight + Transition ? Cant * (S - Straight) / Transition : Cant; };
	for (double s{0.0}; s <= Straight + Transition + Arc + 1e-9; s += 0.5) {
		if (s > 0.0)
			heading += 0.5 * (curvature(s - 0.5) + curvature(s)) * 0.5;
		points.push_back({s, heading, cant(s)});
	}
	return points;
}

// the tracks of a switch Length long laid into the line from the station: the main one along it, the diverging one turning off it
// to the side by the curvature of the basic switch
std::pair<std::vector<switch_curve::track_point>, std::vector<switch_curve::track_point>> laid(std::vector<switch_curve::track_point> const &Line, double const From, double const Length, double const Radius, int const Side) {
	std::vector<switch_curve::track_point> main;
	std::vector<switch_curve::track_point> branch;
	auto const first{std::lower_bound(Line.begin(), Line.end(), From, [](auto const &Point, double const Value) { return Point.station < Value; })};
	for (auto point{first}; point != Line.end() && point->station <= From + Length + 1e-9; ++point) {
		auto const s{point->station - From};
		main.push_back({s, point->heading - first->heading, 0.0});
		branch.push_back({s, point->heading - first->heading + Side * s / Radius, 0.0});
	}
	return {main, branch};
}

bool says(std::vector<std::string> const &Lines, char const *Text) {
	return std::any_of(Lines.begin(), Lines.end(), [&](std::string const &Line) { return Line.find(Text) != std::string::npos; });
}

} // namespace

TEST_CASE("switch on a curve: the curvatures of its tracks on a transition curve, read the way the line turns") {
	auto const track{line(100.0, 120.0, 200.0, 1000.0, 60.0)};
	auto const [main, branch] {laid(track, 150.0, 30.0, 500.0, -1)};
	auto const report{switch_curve::check(main, branch, track, 150.0, 180.0, 500.0, 12.0)};
	CHECK(report.curved);
	CHECK(report.transition);
	CHECK(report.main_start == doctest::Approx(51.0 / 120.0 / 1000.0).epsilon(0.02));
	CHECK(report.main_end == doctest::Approx(79.0 / 120.0 / 1000.0).epsilon(0.02));
	// turning off to the outside of the curve: against it all along, the case KZW I
	CHECK(report.branch_start < 0.0);
	CHECK(report.branch_end < 0.0);
	CHECK(false == says(report.warnings, "KZW II"));
	// laid on the ramp, the cant changing under it
	CHECK(report.cant_start == doctest::Approx(25.0).epsilon(0.02));
	CHECK(report.cant_end == doctest::Approx(40.0).epsilon(0.02));
	CHECK(says(report.warnings, "The cant changes within the switch"));
}

TEST_CASE("switch on a curve: the diverging track turning from one side to the other is the case KZW II") {
	// a transition growing past the curvature of the basic switch along the switch
	auto const track{line(100.0, 60.0, 100.0, 300.0, 0.0)};
	auto const [main, branch] {laid(track, 125.0, 30.0, 500.0, -1)};
	auto const report{switch_curve::check(main, branch, track, 125.0, 155.0, 500.0, 12.0)};
	CHECK(report.branch_start < 0.0);
	CHECK(report.branch_end > 0.0);
	CHECK(says(report.warnings, "KZW II"));
}

TEST_CASE("switch on a curve: the ends of a cant ramp at the ends of the switch or 6 m off it") {
	auto const track{line(100.0, 120.0, 200.0, 1000.0, 60.0)};
	auto const ramps{switch_curve::ramp_ends(track)};
	REQUIRE(ramps.size() == 2);
	CHECK(ramps[0] == doctest::Approx(100.0).epsilon(0.01));
	CHECK(ramps[1] == doctest::Approx(220.0).epsilon(0.01));

	auto const at = [&](double const From) {
		auto const [main, branch] {laid(track, From, 30.0, 500.0, 1)};
		return switch_curve::check(main, branch, track, From, From + 30.0, 500.0, 12.0);
	};
	CHECK(says(at(210.0).warnings, "within the switch"));
	CHECK(says(at(222.0).warnings, "6 m at least"));
	CHECK(false == says(at(220.0).warnings, "within the switch"));
	CHECK(says(at(220.0).notes, "at an end of the switch"));
	CHECK(false == says(at(230.0).warnings, "cant ramp"));

	// the start pressed near those places drawn to them
	std::string what;
	CHECK(switch_curve::snapped_start(ramps, 218.5, 30.0, 1, 3.0, &what) == doctest::Approx(220.0));
	CHECK(what.find("starts at an end") != std::string::npos);
	CHECK(switch_curve::snapped_start(ramps, 228.0, 30.0, 1, 3.0, &what) == doctest::Approx(226.0));
	CHECK(what.find("6 m off") != std::string::npos);
	CHECK(switch_curve::snapped_start(ramps, 72.0, 30.0, 1, 3.0, &what) == doctest::Approx(70.0));
	CHECK(what.find("ends at an end") != std::string::npos);
	CHECK(switch_curve::snapped_start(ramps, 160.0, 30.0, 1, 3.0, &what) == doctest::Approx(160.0));
	CHECK(what.empty());
}
