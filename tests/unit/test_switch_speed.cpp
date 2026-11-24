// the speed on the diverging track of a switch, the rule the editor writes when it lays a switch and the domain model of the track
// geometry (editor/layout) computes for its turnouts: formula 9.9 with the cant with its sign and the deficiency of the frog (table 9.7),
// the main track bent by table 11.8, rounded down to the steps of the speed limits and no more than table 11.2; and the one velocity a
// switch keeps for both its tracks

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorSwitchCurve.hpp"
#include "editor/editorTrack.hpp"

#include <cmath>

TEST_CASE("switch speed: the examples of Q2, the main track bent outward and inward, the cant with its sign") {
	// Rz 60E1-500-1:12 bent outward into R 1200 m with 60 mm, two-sided: the cant leans against the diverging track, 60.3 km/h at the
	// normal 110 mm of a fixed frog, 60 km/h
	auto const outward{switch_curve::branch_speed(500.0, 12.0, -1200.0, -60.0, false)};
	CHECK(outward.radius == doctest::Approx(857.76).epsilon(1e-5));
	CHECK(outward.cant == -60.0);
	CHECK(outward.deficiency == 110.0);
	CHECK(outward.formula == doctest::Approx(60.3).epsilon(0.002));
	CHECK(outward.speed == 60.0);
	// inward, one-sided: r" = 352.7 m, 71.3 km/h, no more than the 60 km/h of the basic switch
	auto const inward{switch_curve::branch_speed(500.0, 12.0, 1200.0, 60.0, false)};
	CHECK(inward.radius == doctest::Approx(352.69).epsilon(1e-5));
	CHECK(inward.cant == 60.0);
	CHECK(inward.rounded == 70.0);
	CHECK(inward.table == 60.0);
	CHECK(inward.speed == 60.0);
	// Rz 60E1-190-1:9 in R 600 m with 60 mm: outward 34.3 km/h, 30; inward 45.6 km/h, 40
	CHECK(switch_curve::branch_speed(190.0, 9.0, -600.0, -60.0, false).speed == 30.0);
	CHECK(switch_curve::branch_speed(190.0, 9.0, 600.0, 60.0, false).speed == 40.0);
	CHECK(switch_curve::branch_speed(190.0, 9.0, -600.0, -60.0, false).formula == doctest::Approx(34.3).epsilon(0.002));
	CHECK(switch_curve::branch_speed(190.0, 9.0, 600.0, 60.0, false).formula == doctest::Approx(45.6).epsilon(0.002));
	// the cant toward the side the diverging track leaves to is taken against it where the bending turns the diverging track the other
	// way: R 400 m against a 500 m switch turns it past straight
	auto const past{switch_curve::branch_speed(500.0, 12.0, -400.0, 60.0, false)};
	CHECK(past.radius < 0.0);
	CHECK(past.cant == -60.0);
}

TEST_CASE("switch speed: the basic switches by table 11.2, a movable nose kept to it, a straight diverging track") {
	CHECK(switch_curve::branch_speed(190.0, 9.0, 0.0, 0.0, false).speed == 40.0);
	CHECK(switch_curve::branch_speed(300.0, 9.0, 0.0, 0.0, false).speed == 50.0);
	CHECK(switch_curve::branch_speed(500.0, 12.0, 0.0, 0.0, false).speed == 60.0);
	CHECK(switch_curve::branch_speed(760.0, 14.0, 0.0, 0.0, false).speed == 80.0);
	CHECK(switch_curve::branch_speed(1200.0, 18.5, 0.0, 0.0, false).speed == 100.0);
	// the 2500-1:26.5 with a movable nose: 166 km/h by the formula at 130 mm, 130 by the table and Id-4
	auto const fast{switch_curve::branch_speed(2500.0, 26.5, 0.0, 0.0, true)};
	CHECK(fast.deficiency == 130.0);
	CHECK(fast.rounded == 160.0);
	CHECK(fast.speed == 130.0);
	// the main track as sharp as the diverging curve against it straightens the diverging track: the table alone
	auto const straight{switch_curve::branch_speed(500.0, 12.0, -500.0, -60.0, false)};
	CHECK(straight.radius == 0.0);
	CHECK(std::isinf(straight.formula));
	CHECK(straight.speed == 60.0);
	// the table of the editor is the one of the switches
	for (auto const radius : {150.0, 190.0, 299.9, 300.0, 499.0, 500.0, 760.0, 1199.0, 1200.0, 2499.0, 2500.0, 5000.0}) {
		CAPTURE(radius);
		CHECK(editor_track::diverging_speed(radius) == switch_curve::basic_speed(radius));
	}
}

TEST_CASE("switch speed: one velocity for both tracks, the main track's limit where it's the lower, else the diverging one's below 0") {
	CHECK(switch_curve::written_velocity(80.0, 60.0) == -60.0);
	CHECK(switch_curve::written_velocity(40.0, 60.0) == 40.0);
	CHECK(switch_curve::written_velocity(60.0, 60.0) == 60.0);
	CHECK(switch_curve::written_velocity(-1.0, 60.0) == -60.0);
	// never -1, which is no limit at all
	CHECK(switch_curve::written_velocity(80.0, 1.0) == -2.0);
}
