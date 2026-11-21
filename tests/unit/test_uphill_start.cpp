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

// the rest of a start: when it begins, when it is given up, when the train brake may be released, when the release
// phase is over and how much power is applied against the releasing brake

namespace {

// the ST43 train standing at the signal on its train brake, asked to go
uphill_start::situation standing_train() {
	uphill_start::situation now;
	now.standing = true;
	now.speed_up = true;
	now.speed_asked = true;
	now.train_brake_applied = true;
	now.holding_force = grade;
	return now;
}

} // namespace

TEST_CASE("uphill start: begins with the whole train at a stand on the train brake, asked to go") {
	auto now{standing_train()};
	CHECK(uphill_start::begins(now));
	SUBCASE("not while the head stands for a moment and the train rolls back") {
		now.consist_velocity = -0.8;
		CHECK_FALSE(uphill_start::begins(now));
	}
	SUBCASE("not without the train brake to hold the train") {
		now.train_brake_applied = false;
		CHECK_FALSE(uphill_start::begins(now));
	}
	SUBCASE("not before the brakes of the train are charged") {
		now.consist_charged = false;
		CHECK_FALSE(uphill_start::begins(now));
	}
	SUBCASE("not when not asked to go") {
		now.speed_up = false;
		CHECK_FALSE(uphill_start::begins(now));
		now.speed_up = true;
		now.speed_asked = false;
		CHECK_FALSE(uphill_start::begins(now));
	}
	SUBCASE("not under way") {
		now.standing = false;
		CHECK_FALSE(uphill_start::begins(now));
	}
}

TEST_CASE("uphill start: given up when the train as a whole rolls back, not its head") {
	auto now{standing_train()};
	// the locomotive runs back into the slack, the rest of the train stands
	now.velocity = -1.0;
	now.consist_velocity = -0.1;
	CHECK_FALSE(uphill_start::rolls_back(now));
	now.consist_velocity = -0.6;
	CHECK(uphill_start::rolls_back(now));
}

TEST_CASE("uphill start: the train brake is released once the power holds the train") {
	auto now{standing_train()};
	now.tractive_force = 1.05 * grade;
	CHECK_FALSE(uphill_start::power_holds(now, 10.0));
	now.tractive_force = 1.15 * grade;
	CHECK(uphill_start::power_holds(now, 10.0));
	SUBCASE("or the train is under way") {
		now.tractive_force = 0.5 * grade;
		now.velocity = 1.2;
		CHECK(uphill_start::power_holds(now, 10.0));
	}
	SUBCASE("or a locomotive that can't hold the train has had its time") {
		now.tractive_force = 0.5 * grade;
		CHECK_FALSE(uphill_start::power_holds(now, uphill_start::hold_time - 1.0));
		CHECK(uphill_start::power_holds(now, uphill_start::hold_time + 1.0));
	}
}

TEST_CASE("uphill start: the release phase ends when the brakes are off, or after its time") {
	auto now{standing_train()};
	CHECK_FALSE(uphill_start::released(now, 30.0));
	now.consist_released = true;
	CHECK(uphill_start::released(now, 30.0));
	SUBCASE("a brake that doesn't come off ends it all the same") {
		now.consist_released = false;
		CHECK_FALSE(uphill_start::released(now, uphill_start::release_time - 1.0));
		CHECK(uphill_start::released(now, uphill_start::release_time + 1.0));
	}
}

TEST_CASE("uphill start: against the releasing brake no more power is added than holds the train") {
	// outside the release phase the start doesn't limit the power
	CHECK(uphill_start::may_add_power(false, 3.0 * grade, grade));
	CHECK(uphill_start::may_add_power(true, 1.05 * grade, grade));
	CHECK_FALSE(uphill_start::may_add_power(true, 1.15 * grade, grade));
}

TEST_CASE("uphill start: on coarse notches the power stays on the first one that holds the train") {
	// an EU07 at a standstill on 3 kV gives about 29, 52, 79 and 104 kN on notches 1-4 (eu07/maszyna#396): with a
	// train of 300 t on 10 per mille, 29.4 kN of grade force, notch 2 jumps from 0.99 to 1.77 x the grade force
	double const notches[]{29000.0, 52000.0, 79000.0, 104000.0};
	double const holding{300000.0 * 9.81 * 0.01};
	auto notch{0};
	auto force{0.0};
	// in the release phase, a notch added an update as long as power may be added
	for (auto update{0}; update < 10; ++update) {
		if (uphill_start::may_add_power(true, force, holding) && (notch < 4)) {
			force = notches[notch++];
		}
	}
	CHECK(notch == 2);
	CHECK(force / holding == doctest::Approx(1.77).epsilon(0.01));
}
