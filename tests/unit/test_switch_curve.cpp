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

TEST_CASE("switch on a curve: one with a straight frog bent up to the frog, both tracks straight over it") {
	auto const track{line(0.0, 1.0, 400.0, 1500.0, 0.0)};
	for (auto const side : {1, -1}) {
		CAPTURE(side);
		auto [main, branch] {laid(track, 150.0, 27.0, 190.0, side)};
		// the last 6 m of both straight, the diverging one with no curve of its own there either: the frog of the switch
		auto const frog{21.0};
		auto const straighten = [&](std::vector<switch_curve::track_point> &Track) {
			auto const at{std::find_if(Track.begin(), Track.end(), [&](auto const &Point) { return Point.station >= frog; })};
			for (auto point{at}; point != Track.end(); ++point)
				point->heading = at->heading;
		};
		straighten(main);
		straighten(branch);
		auto const report{switch_curve::check(main, branch, track, 150.0, 177.0, 190.0, 9.0, 27.0 - frog)};
		CHECK(report.curved);
		// the curvature of the bent part only: no transition curve under it, the diverging track turning one way all along
		CHECK_FALSE(report.transition);
		CHECK(false == says(report.warnings, "KZW II"));
		CHECK(says(report.warnings, "Bent up to the frog"));
	}
}

TEST_CASE("switch on a curve: none of 1:9.403 nor steeper than 1:9 bent in the running and the main tracks, 1:9 itself is") {
	auto const track{line(0.0, 1.0, 400.0, 1500.0, 0.0)};
	auto const [main, branch] {laid(track, 150.0, 33.0, 300.0, 1)};
	auto const banned = [&](double const Ratio) {
		CAPTURE(Ratio);
		auto const report{switch_curve::check(main, branch, track, 150.0, 183.0, 300.0, Ratio)};
		REQUIRE(report.curved);
		return says(report.warnings, "(11.4 pkt 4)");
	};
	// 1:9 of the catalogue and as read off the angle of the tracks
	CHECK_FALSE(banned(9.0));
	CHECK_FALSE(banned(8.998));
	CHECK_FALSE(banned(9.002));
	CHECK(banned(9.403));
	CHECK(banned(9.41));
	CHECK_FALSE(banned(9.51));
	CHECK_FALSE(banned(10.0));
	CHECK_FALSE(banned(12.0));
	CHECK(banned(7.5));
	CHECK(banned(6.6));
	CHECK(banned(4.8));
}

namespace {

// a track of one curvature from its start, heading as given there, a point every half metre
std::vector<switch_curve::track_point> arc(double const Length, double const Heading, double const Curvature) {
	std::vector<switch_curve::track_point> points;
	for (double s{0.0}; s <= Length + 1e-9; s += 0.5)
		points.push_back({s, Heading + Curvature * s, 0.0});
	return points;
}

// a slip of 1:9 with the slip curves of R 190 m bent onto the line from the station on: the tracks through it with the curvature of the
// line, the slip curves with theirs plus the one of the line, one turning as the line does, the other against it
switch_curve::slip_report bent_slip(std::vector<switch_curve::track_point> const &Line, double const From, double const Curvature) {
	auto const angle{std::atan(1.0 / 9.0)};
	auto const half{16.6};
	auto const tracks{std::vector{arc(2.0 * half, 0.0, Curvature), arc(2.0 * half, angle, Curvature * std::cos(angle))}};
	auto const curves{std::vector{arc(190.0 * angle, 0.0, 1.0 / 190.0 + Curvature), arc(190.0 * angle, angle, -1.0 / 190.0 + Curvature)}};
	return switch_curve::check_slip(tracks, curves, Line, From, From + 2.0 * half);
}

} // namespace

TEST_CASE("slip bent into a curve: in a side track only, its slip curve bent the way of the curve no sharper than a side track takes") {
	auto const sharp{bent_slip(line(0.0, 0.0, 400.0, 800.0, 0.0), 150.0, 1.0 / 800.0)};
	CHECK(sharp.curved);
	CHECK(says(sharp.warnings, "11.4 pkt 4b"));
	CHECK(sharp.first == doctest::Approx(1.0 / 800.0).epsilon(0.02));
	CHECK(sharp.second == doctest::Approx(std::cos(std::atan(1.0 / 9.0)) / 800.0).epsilon(0.02));
	REQUIRE(sharp.curves.size() == 2);
	// 1/190 + 1/800: R 153.5 m, 1/190 - 1/800: R 249.2 m
	CHECK(1.0 / sharp.curves[0] == doctest::Approx(153.5).epsilon(0.01));
	CHECK(1.0 / sharp.curves[1] == doctest::Approx(249.2).epsilon(0.01));
	CHECK(says(sharp.warnings, "R 154 m, below the least 180 m"));
	CHECK(false == says(sharp.warnings, "R 249 m"));
	CHECK(false == says(sharp.warnings, "9.7 pkt 12"));

	// R 4000 m: the slip curve of 1/190 + 1/4000, R 181.4 m, still within the side tracks
	auto const gentle{bent_slip(line(0.0, 0.0, 400.0, 4000.0, 0.0), 150.0, 1.0 / 4000.0)};
	CHECK(gentle.curved);
	CHECK(says(gentle.warnings, "11.4 pkt 4b"));
	CHECK(false == says(gentle.warnings, "table 9.2"));
	CHECK(gentle.notes.empty());

	// one on a straight is no bent one
	auto const straight{bent_slip(line(400.0, 0.0, 0.0, 1000.0, 0.0), 150.0, 0.0)};
	CHECK_FALSE(straight.curved);
	CHECK(straight.warnings.empty());
}

TEST_CASE("slip bent into a curve: the cant of the side track, a track through it turning from one side to the other") {
	// the cant the same over it, none recommended in a side track though
	auto const canted{bent_slip(line(0.0, 0.0, 400.0, 1000.0, 30.0), 150.0, 1.0 / 1000.0)};
	CHECK(canted.cant_start == doctest::Approx(30.0));
	CHECK(canted.cant_end == doctest::Approx(30.0));
	CHECK(false == says(canted.warnings, "The cant changes"));
	CHECK(says(canted.notes, "9.3 pkt 5"));

	// a reverse curve under it
	auto const curve{line(0.0, 0.0, 400.0, 1000.0, 0.0)};
	auto first{arc(33.2, 0.0, 1.0 / 1000.0)};
	for (auto &point : first)
		if (point.station > 16.6)
			point.heading = 16.6 / 1000.0 - (point.station - 16.6) / 1000.0;
	auto const angle{std::atan(1.0 / 9.0)};
	auto const report{switch_curve::check_slip({first, arc(33.2, angle, 1.0 / 1000.0)}, {arc(21.0, 0.0, 1.0 / 190.0)}, curve, 150.0, 183.2)};
	CHECK(says(report.warnings, "9.7 pkt 12"));
}

TEST_CASE("switch exit: a curve right behind a straight frog, its sudden changes of the cant deficiency apart or together") {
	double const sw{1.0 / 190.0};
	// R 190 on behind a frog straight for 6.09 m, at 40 km/h: apart, 99 mm at the end, within the 130 mm of table 9.15
	auto const same{switch_curve::check_exit(sw, 6.092, {{40.0, sw, sw}}, 40.0)};
	CHECK(same.apart == doctest::Approx(6.092));
	CHECK(same.element == doctest::Approx(6.0));
	CHECK(same.change == doctest::Approx(99.4).epsilon(0.01));
	CHECK(same.warnings.empty());
	// the other way behind a frog straight of 1.66 m only: together, 199 mm, over the 130 mm
	auto const reverse{switch_curve::check_exit(sw, 1.662, {{40.0, -sw, -sw}}, 40.0)};
	CHECK(reverse.apart == doctest::Approx(1.662));
	CHECK(reverse.change == doctest::Approx(198.8).epsilon(0.01));
	REQUIRE(reverse.warnings.size() == 1);
	CHECK(says(reverse.warnings, "tables 9.15, 9.16"));
	// the same way behind it: together the curvature hardly changes
	CHECK(switch_curve::check_exit(sw, 1.662, {{40.0, sw, sw}}, 40.0).warnings.empty());
	// a straight of 5 m laid on first: 6.66 m apart, the curve the other way alone, 99 mm
	auto const later{switch_curve::check_exit(sw, 1.662, {{5.0, 0.0, 0.0}, {40.0, -sw, -sw}}, 40.0)};
	CHECK(later.apart == doctest::Approx(6.662));
	CHECK(later.warnings.empty());
	// a sharp curve of R 100 alone: 189 mm
	CHECK(says(switch_curve::check_exit(sw, 6.092, {{30.0, 0.01, 0.01}}, 40.0).warnings, "table 9.15"));
	// a transition out of the frog straight: no sudden change behind it
	CHECK(switch_curve::check_exit(sw, 1.662, {{20.0, 0.0, -sw}, {20.0, -sw, -sw}}, 40.0).warnings.empty());
	// the curve of a switch with a curved frog going on: nothing of the switch to judge
	auto const on{switch_curve::check_exit(1.0 / 300.0, 0.0, {{30.0, 1.0 / 300.0, 1.0 / 300.0}}, 50.0)};
	CHECK(on.apart < 0.0);
	CHECK(on.warnings.empty());
}
