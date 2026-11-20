#include "stdafx.h"
#include "doctest.h"

#include "vehicle/uphillstart.h"

// starting uphill, the train brake is applied further, a step at a time, until the brake of the unpowered vehicles
// holds the train without the locomotive brake (eu07/maszyna#396). the numbers are those of an ST43 with 37 empty
// wagons on 12.5 per mille: 1040 t, 128.6 kN of grade force, the wagons on G with about 22 s to apply

using uphill_start::brake_top_up;
using uphill_start::top_up;
using uphill_start::top_up_action;

namespace {

double const grade{128600.0}; // [N]
double const application{22.0}; // [s]

// the train standing with the brake handle at Position, held there long enough for the cylinders to fill, while
// each step adds Gain of brake force (as a multiple of the grade force) to the Start it had. runs the top-up until it
// lets the locomotive brake off, and returns the handle position it stopped at and the number of steps
struct outcome {
	double position;
	int steps;
};
outcome run_top_up(double Start, double Gain, double Position = 1.25) {
	top_up state;
	auto force{Start * grade};
	auto steps{0};
	for (auto guard{0}; guard < 100; ++guard) {
		auto const action{brake_top_up(state, force, grade, Position, application, application)};
		if (action == top_up_action::none) {
			break;
		}
		REQUIRE(action == top_up_action::step);
		Position += 0.25;
		force += Gain * grade;
		++steps;
	}
	return {Position, steps};
}

} // namespace

TEST_CASE("uphill start: a brake that holds the train isn't applied further") {
	top_up state;
	CHECK(brake_top_up(state, 1.1 * grade, grade, 1.25, application, application) == top_up_action::none);
	CHECK(state.last_force < 0.0);
}

TEST_CASE("uphill start: each step waits the full application time of the brake") {
	top_up state;
	CHECK(brake_top_up(state, 0.9 * grade, grade, 1.25, 0.5 * application, application) == top_up_action::wait);
	CHECK(brake_top_up(state, 0.9 * grade, grade, 1.25, application, application) == top_up_action::step);
	// the handle has just moved
	CHECK(brake_top_up(state, 0.95 * grade, grade, 1.5, 0.0, application) == top_up_action::wait);
	CHECK(brake_top_up(state, 0.95 * grade, grade, 1.5, application - 0.1, application) == top_up_action::wait);
}

TEST_CASE("uphill start: a light application is topped up until the brake holds the train") {
	// before Nowy Zagorz on linia_107_objazdy: 0.64 bar in the wagons, 128 kN against 129 kN
	auto const result{run_top_up(0.995, 0.15)};
	CHECK(result.steps == 1);
	CHECK(result.position == doctest::Approx(1.5));
}

TEST_CASE("uphill start: a brake that doesn't get stronger ends the top-up after one more step") {
	// all the wagon brakes cut out: no step adds anything
	auto const result{run_top_up(0.0, 0.0)};
	CHECK(result.steps == 1);
	CHECK(result.position < uphill_start::top_up_position);
}

TEST_CASE("uphill start: a brake that gets stronger by too little ends the top-up") {
	// a step adds 1% of the grade force, below the 2% that counts
	auto const result{run_top_up(0.3, 0.01)};
	CHECK(result.steps == 1);
	SUBCASE("and one that gets stronger by enough goes on") {
		auto const going{run_top_up(0.3, 0.05)};
		CHECK(going.steps > 1);
	}
}

TEST_CASE("uphill start: the top-up goes no further than full service") {
	// every step adds enough to go on, but never enough to hold the train
	auto const result{run_top_up(0.0, 0.03)};
	CHECK(result.position == doctest::Approx(uphill_start::top_up_position));
	CHECK(result.steps == 11);
	top_up state;
	CHECK(brake_top_up(state, 0.0, grade, uphill_start::top_up_position, application, application) == top_up_action::none);
}

TEST_CASE("uphill start: once ended, the top-up stays ended for the start") {
	top_up state;
	REQUIRE(brake_top_up(state, 0.2 * grade, grade, 1.25, application, application) == top_up_action::step);
	REQUIRE(brake_top_up(state, 0.2 * grade, grade, 1.5, application, application) == top_up_action::none);
	CHECK(state.done);
	// the brake gets stronger later all the same: the start has gone on without the top-up
	CHECK(brake_top_up(state, 0.5 * grade, grade, 1.5, application, application) == top_up_action::none);
	// a new start begins with a new top-up
	state = {};
	CHECK(brake_top_up(state, 0.5 * grade, grade, 1.5, application, application) == top_up_action::step);
}
