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
	for (int k = 0; k * 0.5 <= Straight + Transition + Arc + 1e-9; ++k) {
		auto const s{k * 0.5};
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
	CHECK(false == says(report.refusals, "KZW II"));
	// laid on the ramp, the cant changing under it: not laid so (11.4 pkt 7)
	CHECK(report.cant_start == doctest::Approx(25.0).epsilon(0.02));
	CHECK(report.cant_end == doctest::Approx(40.0).epsilon(0.02));
	CHECK(says(report.refusals, "The cant changes within the switch"));
	CHECK(false == says(report.warnings, "The cant changes within the switch"));
}

TEST_CASE("switch on a curve: the diverging track turning from one side to the other is the case KZW II") {
	// a transition growing past the curvature of the basic switch along the switch
	auto const track{line(100.0, 60.0, 100.0, 300.0, 0.0)};
	auto const [main, branch] {laid(track, 125.0, 30.0, 500.0, -1)};
	auto const report{switch_curve::check(main, branch, track, 125.0, 155.0, 500.0, 12.0)};
	CHECK(report.branch_start < 0.0);
	CHECK(report.branch_end > 0.0);
	// refused, with the radii it turns with either way
	CHECK(says(report.refusals, "KZW II"));
	CHECK(says(report.refusals, "(9.7 pkt 12)"));
	CHECK(false == says(report.warnings, "KZW II"));
	// the main track, on the transition curve alone, turns one way all along
	CHECK(false == says(report.refusals, "main track"));
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
	// within it or nearer than 6 m: not laid so
	CHECK(says(at(210.0).refusals, "within the switch"));
	CHECK(says(at(222.0).refusals, "6 m at least"));
	CHECK(says(at(222.0).refusals, "2.0 m off the switch"));
	CHECK(false == says(at(220.0).refusals, "within the switch"));
	CHECK(says(at(220.0).notes, "at an end of the switch"));
	CHECK(at(230.0).refusals.empty());
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
		CHECK(false == says(report.refusals, "KZW II"));
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
		// the editor tells no main track from a side one: a warning, which says so, not a refusal
		CHECK(report.refusals.empty());
		return says(report.warnings, "(11.4 pkt 4)") && says(report.warnings, "only warns");
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
	for (int k = 0; k * 0.5 <= Length + 1e-9; ++k)
		points.push_back({k * 0.5, Heading + Curvature * (k * 0.5), 0.0});
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
	// none bent, the editor telling no main track from a side one
	CHECK(says(sharp.refusals, "R 800 m: no double switch, slip nor crossing is bent (11.4 pkt 4)"));
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
	CHECK(says(gentle.refusals, "(11.4 pkt 4)"));
	CHECK(false == says(gentle.warnings, "table 9.2"));
	CHECK(gentle.notes.empty());

	// one on a straight is no bent one
	auto const straight{bent_slip(line(400.0, 0.0, 0.0, 1000.0, 0.0), 150.0, 0.0)};
	CHECK_FALSE(straight.curved);
	CHECK(straight.warnings.empty());
	CHECK(straight.refusals.empty());
}

TEST_CASE("slip bent into a curve: the cant of the side track, a track through it turning from one side to the other") {
	// the cant the same over it, none recommended in a side track though
	auto const canted{bent_slip(line(0.0, 0.0, 400.0, 1000.0, 30.0), 150.0, 1.0 / 1000.0)};
	CHECK(canted.cant_start == doctest::Approx(30.0));
	CHECK(canted.cant_end == doctest::Approx(30.0));
	CHECK(false == says(canted.refusals, "The cant changes"));
	CHECK(says(canted.notes, "9.3 pkt 5"));

	// a reverse curve under it
	auto const curve{line(0.0, 0.0, 400.0, 1000.0, 0.0)};
	auto first{arc(33.2, 0.0, 1.0 / 1000.0)};
	for (auto &point : first)
		if (point.station > 16.6)
			point.heading = 16.6 / 1000.0 - (point.station - 16.6) / 1000.0;
	auto const angle{std::atan(1.0 / 9.0)};
	auto const report{switch_curve::check_slip({first, arc(33.2, angle, 1.0 / 1000.0)}, {arc(21.0, 0.0, 1.0 / 190.0)}, curve, 150.0, 183.2)};
	CHECK(says(report.refusals, "9.7 pkt 12"));
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

TEST_CASE("switch on a curve: a double switch isn't bent, nor is a slip or a crossing (11.4 pkt 4)") {
	auto const track{line(0.0, 1.0, 400.0, 1500.0, 0.0)};
	auto const [main, branch] {laid(track, 150.0, 30.0, 190.0, 1)};
	auto const bent{switch_curve::check(main, branch, track, 150.0, 180.0, 190.0, 9.0, 0.0, true)};
	REQUIRE(bent.curved);
	CHECK(says(bent.refusals, "Bent into R 1500 m: no double switch, slip nor crossing is bent (11.4 pkt 4)"));
	// a single one of the same shape is
	CHECK(switch_curve::check(main, branch, track, 150.0, 180.0, 190.0, 9.0).refusals.empty());
	// and a double one on a straight
	auto const straight{line(400.0, 0.0, 0.0, 1000.0, 0.0)};
	auto const [smain, sbranch] {laid(straight, 150.0, 30.0, 190.0, 1)};
	auto const on_straight{switch_curve::check(smain, sbranch, straight, 150.0, 180.0, 190.0, 9.0, 0.0, true)};
	CHECK_FALSE(on_straight.curved);
	CHECK(on_straight.refusals.empty());
}

TEST_CASE("switch on a straight: the cant of the line over it, no ramp within it nor nearer than 6 m (11.4 pkt 7, 9.6 pkt 16)") {
	// a straight of 100 m, a ramp of 60 m up to 60 mm, then 100 m of 60 mm
	std::vector<switch_curve::track_point> straight;
	for (int k = 0; k * 0.5 <= 260.0 + 1e-9; ++k) {
		auto const s{k * 0.5};
		straight.push_back({s, 0.0, s <= 100.0 ? 0.0 : s <= 160.0 ? (s - 100.0) : 60.0});
	}
	auto const at = [&](double const From) { return switch_curve::check_line(straight, From, From + 33.2); };
	CHECK(at(20.0).refusals.empty());
	CHECK(at(60.0).refusals.empty());
	// 1.8 m off the start of the ramp
	CHECK(says(at(65.0).refusals, "1.8 m off the switch, 6 m at least (9.6 pkt 16)"));
	// on it: the cant changing under it, the start of the ramp within it
	auto const on{at(90.0)};
	CHECK(says(on.refusals, "The cant changes within the switch, 0 to 23 mm: it's to stay the same over it (11.4 pkt 7)"));
	CHECK(says(on.refusals, "10.0 m from its start"));
	// ending right where the ramp starts, and of the cant all along
	CHECK(at(100.0 - 33.2).refusals.empty());
	CHECK(says(at(100.0 - 33.2).notes, "at an end of the switch"));
	CHECK(at(180.0).refusals.empty());
	CHECK(at(180.0).cant_start == doctest::Approx(60.0));
}

TEST_CASE("table 11.8: the diverging track of a switch bent one-sided and two-sided, the tangent t correcting the sum of the curvatures") {
	// Rz 60E1-500-1:12: t = 500 tan(atan(1/12) / 2) = 20.797 m, as a of the catalogue
	// a) two-sided, the main track bent to R 1200 against the diverging one: r" = (R r + t^2) / (R - r)
	CHECK(switch_curve::bent_radius(-1200.0, 500.0, 12.0) == doctest::Approx((1200.0 * 500.0 + 20.797 * 20.797) / 700.0).epsilon(1e-5));
	CHECK(switch_curve::bent_radius(-1200.0, 500.0, 12.0) == doctest::Approx(857.76).epsilon(1e-5));
	// b) one-sided, the main track bent to R 1200 with it: r" = (R r - t^2) / (R + r)
	CHECK(switch_curve::bent_radius(1200.0, 500.0, 12.0) == doctest::Approx(352.69).epsilon(1e-5));
	// near the plain sums of the curvatures, 1 / (1/500 -+ 1/1200)
	CHECK(switch_curve::bent_radius(-1200.0, 500.0, 12.0) == doctest::Approx(857.14).epsilon(1e-3));
	CHECK(switch_curve::bent_radius(1200.0, 500.0, 12.0) == doctest::Approx(352.94).epsilon(1e-3));
	// b) bent from the diverging track: the main track sharper than it takes it past straight, r" = (R r + t^2) / (r - R), the other way
	CHECK(switch_curve::bent_radius(-400.0, 500.0, 12.0) == doctest::Approx(-2004.33).epsilon(1e-5));
	// the main track as sharp as the diverging curve against it: the diverging track straight; not bent: the basic switch
	CHECK(switch_curve::bent_radius(-500.0, 500.0, 12.0) == 0.0);
	CHECK(switch_curve::bent_radius(0.0, 500.0, 12.0) == 500.0);
}

TEST_CASE("table 11.9: the least radii of the switches bent one-sided, the diverging track of the least main radius by table 11.8") {
	struct row {
		double radius;
		double ratio;
		double branch; // the limiting curvature, 1/km
		double least_branch; // m
		double least_main; // m
		double branch_exception;
		double least_branch_exception;
		double least_main_exception;
	};
	// the values in the brackets for 300 and 500, allowed by way of exception
	row const rows[]{{300.0, 9.0, 4.000, 250.0, 1505.522, 4.673, 214.0, 749.722},
	                 {500.0, 12.0, 4.000, 250.0, 501.730, 4.673, 214.0, 375.638},
	                 {760.0, 14.0, 3.333, 300.0, 497.250, 3.333, 300.0, 497.250},
	                 {1200.0, 18.5, 2.169, 461.0, 750.001, 2.169, 461.0, 750.001},
	                 {2500.0, 26.5, 1.063, 941.0, 1510.0, 1.063, 941.0, 1510.0}};
	for (auto const &entry : rows) {
		CAPTURE(entry.radius);
		auto const limit{switch_curve::bent_limit_of(entry.radius)};
		REQUIRE(limit.has_value());
		CHECK(limit->branch * 1000.0 == doctest::Approx(entry.branch));
		CHECK(limit->main == doctest::Approx(entry.least_main));
		CHECK(limit->branch_exception * 1000.0 == doctest::Approx(entry.branch_exception));
		CHECK(limit->main_exception == doctest::Approx(entry.least_main_exception));
		// the least main radius bends the diverging track one-sided to the least radius the table gives it
		CHECK(switch_curve::bent_radius(entry.least_main, entry.radius, entry.ratio) == doctest::Approx(entry.least_branch).epsilon(0.001));
		CHECK(switch_curve::bent_radius(entry.least_main_exception, entry.radius, entry.ratio) == doctest::Approx(entry.least_branch_exception).epsilon(0.001));
	}
	CHECK_FALSE(switch_curve::bent_limit_of(190.0).has_value());
}

TEST_CASE("table 9.7: the cant deficiency over a switch, fixed frog and movable nose") {
	auto const fixed{switch_curve::switch_deficiency(false)};
	CHECK(fixed.recommended == 90.0);
	CHECK(fixed.normal == 110.0);
	CHECK(fixed.limit == 110.0);
	auto const movable{switch_curve::switch_deficiency(true)};
	CHECK(movable.recommended == 100.0);
	CHECK(movable.normal == 130.0);
	CHECK(movable.limit == 130.0);
}

TEST_CASE("formula 9.9 and table 9.7: the speeds of table 11.2 on the diverging tracks of the basic switches with a fixed frog") {
	// V = sqrt(I R / 11.8) at the normal 110 mm, rounded down to 10 km/h: 190 40, 300 (1:9 and 1:9.403) 50, 500 60, 760 80, 1200 100
	auto const table = [](double const Radius) { return std::floor(switch_curve::curve_speed(Radius, 0.0, switch_curve::switch_deficiency(false).normal) / 10.0) * 10.0; };
	CHECK(table(190.0) == 40.0);
	CHECK(table(300.0) == 50.0);
	CHECK(table(500.0) == 60.0);
	CHECK(table(760.0) == 80.0);
	CHECK(table(1200.0) == 100.0);
	// the 2500-1:26.5 with a movable nose would take 166 km/h; table 11.2 and Id-4 give it 130, which the editor keeps
	CHECK(switch_curve::curve_speed(2500.0, 0.0, switch_curve::switch_deficiency(true).normal) == doctest::Approx(166.0).epsilon(0.001));
	// a straight track takes any speed, a cant against the turn as large as the deficiency none
	CHECK(std::isinf(switch_curve::curve_speed(0.0, 0.0, 110.0)));
	CHECK(switch_curve::curve_speed(500.0, -110.0, 110.0) == 0.0);
	CHECK(switch_curve::curve_speed(500.0, -150.0, 110.0) == 0.0);
}

TEST_CASE("formula 9.9: the speed on the diverging track of a bent switch, the cant with its sign, the frog by table 9.7") {
	// the example of the review: Rz 60E1-500-1:12 bent outward into R 1200 m with 60 mm of cant, two-sided, r" = 857.8 m. The cant
	// leans against the diverging track: with I up to 100 mm, the 0.65 m/s2 the editor took for the switch branches, about 54 km/h,
	// where the cant taken without its sign gave 107
	auto const radius{switch_curve::bent_radius(-1200.0, 500.0, 12.0)};
	CHECK(switch_curve::curve_speed(radius, -60.0, 100.0) == doctest::Approx(53.9).epsilon(0.002));
	CHECK(switch_curve::curve_speed(radius, 60.0, 100.0) == doctest::Approx(107.8).epsilon(0.002));
	// by its frog, table 9.7: fixed, 90 recommended and 110 normal; movable nose, 100 and 130
	auto const fixed{switch_curve::switch_deficiency(false)};
	auto const movable{switch_curve::switch_deficiency(true)};
	CHECK(switch_curve::curve_speed(radius, -60.0, fixed.recommended) == doctest::Approx(46.7).epsilon(0.002));
	CHECK(switch_curve::curve_speed(radius, -60.0, fixed.normal) == doctest::Approx(60.3).epsilon(0.002));
	CHECK(switch_curve::curve_speed(radius, -60.0, movable.recommended) == doctest::Approx(53.9).epsilon(0.002));
	CHECK(switch_curve::curve_speed(radius, -60.0, movable.normal) == doctest::Approx(71.3).epsilon(0.002));
	// bent inward, one-sided into R 1200: r" = 352.7 m, the cant leaning into its turn, 71 km/h at 110 mm
	CHECK(switch_curve::curve_speed(switch_curve::bent_radius(1200.0, 500.0, 12.0), 60.0, fixed.normal) == doctest::Approx(71.3).epsilon(0.002));
	// Rz 60E1-190-1:9 in the curve R 600 m with 60 mm of the self-test scenery: outward 278.3 m and 34.3 km/h, inward 144.2 m and 45.6 km/h
	CHECK(switch_curve::bent_radius(-600.0, 190.0, 9.0) == doctest::Approx(278.3).epsilon(0.001));
	CHECK(switch_curve::curve_speed(switch_curve::bent_radius(-600.0, 190.0, 9.0), -60.0, fixed.normal) == doctest::Approx(34.3).epsilon(0.002));
	CHECK(switch_curve::bent_radius(600.0, 190.0, 9.0) == doctest::Approx(144.2).epsilon(0.001));
	CHECK(switch_curve::curve_speed(switch_curve::bent_radius(600.0, 190.0, 9.0), 60.0, fixed.normal) == doctest::Approx(45.6).epsilon(0.002));
}
