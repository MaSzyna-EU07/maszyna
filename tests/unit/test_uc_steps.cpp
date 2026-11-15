#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "drive.h"

#include <cmath>

// the universal controller (EIMCtrlType 3) setting its power demand in steps (UCList CtrlValStep), as the 6Dg does
// in steps of 5% (eu07/maszyna#412). the 214M railbus's controller: 3 lowers the power to 0, 4 holds it, 5 raises it
// to 1, at 0.15 a second after IniCDelay (1 s)

using test::definition;
using test::vehicle;

namespace {

constexpr int lower{3};
constexpr int hold{4};
constexpr int raise{5};

// the railbus started up, its brakes released and its direction set, its controller holding no power
void get_ready(vehicle &Railbus, test::track const &Track) {
	auto driven{test::driving(Railbus)};
	REQUIRE(test::start_up(driven, Track));
	test::release_brakes(driven.cab);
	for (int i = 0; i < 3 && Railbus->DirActive <= 0; ++i) {
		Railbus->DirectionForward();
	}
	while (Railbus->MainCtrlPos < hold && Railbus->IncMainCtrl(1)) {
		;
	}
	while (Railbus->MainCtrlPos > hold && Railbus->DecMainCtrl(1)) {
		;
	}
	REQUIRE(Railbus->MainCtrlPos == hold);
	Railbus.run(2.0, Track);
	REQUIRE(Railbus->eimic == doctest::Approx(0.0));
}

void move_to(vehicle &Railbus, int const Position) {
	while (Railbus->MainCtrlPos < Position && Railbus->IncMainCtrl(1)) {
		;
	}
	while (Railbus->MainCtrlPos > Position && Railbus->DecMainCtrl(1)) {
		;
	}
	REQUIRE(Railbus->MainCtrlPos == Position);
}

bool on_step(double const Value, double const Step) {
	return std::abs(Value / Step - std::round(Value / Step)) < 1e-6;
}

} // namespace

TEST_CASE("controls: the universal controller with CtrlValStep sets its power in whole steps") {
	test::track const line{0.15, 0.0};
	vehicle railbus{definition::fiz("21xm_v1/214ma").set("UCList", "CtrlValStep", 0.05), 0.0, "test", 1};
	REQUIRE(railbus->UniCtrlValStep == doctest::Approx(0.05));
	get_ready(railbus, line);

	// a tap on the raising position: one step at once, kept by the holding one
	move_to(railbus, raise);
	railbus.step(line);
	CHECK(railbus->eimic == doctest::Approx(0.05));
	move_to(railbus, hold);
	railbus.run(2.0, line);
	CHECK(railbus->eimic == doctest::Approx(0.05));

	// held there: after the initial delay a step each third of a second (0.15 a second), never in between
	move_to(railbus, raise);
	for (int i = 0; i < 310; ++i) { // 3.1 s
		railbus.step(line);
		CAPTURE(i);
		CHECK(on_step(railbus->eimic, 0.05));
	}
	// the first step at once, 1 s of the initial delay, then 2.1 s of 0.15 a second (6 steps)
	CHECK(railbus->eimic == doctest::Approx(0.40));
	move_to(railbus, hold);
	railbus.run(2.0, line);
	CHECK(railbus->eimic == doctest::Approx(0.40));

	// up to the full power, and not past it
	move_to(railbus, raise);
	railbus.run(10.0, line);
	CHECK(railbus->eimic == doctest::Approx(1.0));

	// down the same way: a tap is one step, holding it goes on in steps down to nothing
	move_to(railbus, lower);
	railbus.step(line);
	CHECK(railbus->eimic == doctest::Approx(0.95));
	move_to(railbus, hold);
	railbus.run(1.0, line);
	CHECK(railbus->eimic == doctest::Approx(0.95));
	move_to(railbus, lower);
	for (int i = 0; i < 800; ++i) { // 8 s
		railbus.step(line);
		CAPTURE(i);
		CHECK(on_step(railbus->eimic, 0.05));
	}
	CHECK(railbus->eimic == doctest::Approx(0.0));
}

TEST_CASE("controls: the universal controller without CtrlValStep sets its power continuously, as before") {
	test::track const line{0.15, 0.0};
	vehicle railbus{definition::fiz("21xm_v1/214ma"), 0.0, "test", 1};
	REQUIRE(railbus->UniCtrlValStep == 0.0);
	get_ready(railbus, line);
	// a tap moves it by 0.01, held it goes on at 0.15 a second after the initial delay
	move_to(railbus, raise);
	railbus.step(line);
	CHECK(railbus->eimic == doctest::Approx(0.01));
	railbus.run(2.0 - 0.01, line);
	CHECK(railbus->eimic == doctest::Approx(0.01 + 0.15).epsilon(0.05));
}
