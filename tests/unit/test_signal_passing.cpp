#include "stdafx.h"
#include "doctest.h"

#include "vehicle/signalpassing.h"

// when a signal in the AI's speed table counts as passed by the train (TController::CheckSignalPassed): the test
// mode reports a signal at stop passed that way as "passed at danger", the blackbox records every passed signal. a
// train set or standing past a signal at the start of a scenario didn't pass it; within 5 m of the signal the test
// mode used to report it all the same.

namespace {

// a point of the speed table the way TSpeedPos keeps it: found by the scan of the route, then updated as the head
// moves on (TController::TableCheck: UpdateDistance, Update, CheckSignalPassed)
struct table_point {
	double distance; // ahead of the head [m], negative behind it
	bool seen_ahead;
	bool elapsed{false};
	int passes{0}; // reports of the point passed

	table_point(double const Distance, bool const Standing)
	    : distance(Distance), seen_ahead(signal_passing::seen_ahead_when_found(Distance, Standing)) {
		move(0.0); // the first update comes with the next check of the table, the train hasn't moved yet
	}

	// the head moves Metres on, and the table updates the point
	void move(double const Metres) {
		distance -= Metres;
		auto const elapsedbefore{elapsed};
		elapsed = elapsed || distance < 0.0;
		seen_ahead = signal_passing::seen_ahead_on_update(distance, seen_ahead);
		if (signal_passing::passed_now(elapsedbefore, elapsed, seen_ahead)) {
			++passes;
		}
	}

	// the head moves Metres on in steps of a metre
	void run(double const Metres) {
		for (int moved{0}; moved < Metres; ++moved) {
			move(1.0);
		}
	}
};

auto constexpr standing{true};
auto constexpr moving{false};

} // namespace

TEST_CASE("signal passing: a train standing past a signal when its table is traced hasn't passed it") {
	auto behind{0.0};
	// 1-4.5 m: reported as passed at danger before; 20 m: the train standing across the signal of #409
	SUBCASE("0.5 m") { behind = 0.5; }
	SUBCASE("1 m") { behind = 1.0; }
	SUBCASE("2 m") { behind = 2.0; }
	SUBCASE("3 m") { behind = 3.0; }
	SUBCASE("4 m") { behind = 4.0; }
	SUBCASE("4.9 m") { behind = 4.9; }
	SUBCASE("20 m") { behind = 20.0; }
	CAPTURE(behind);

	table_point signal{-behind, standing};
	CHECK_FALSE(signal.seen_ahead);
	// standing there a while, then pulling away from it
	for (auto check{0}; check < 10; ++check) {
		signal.move(0.0);
	}
	signal.run(200.0);
	CHECK(signal.passes == 0);
}

TEST_CASE("signal passing: a train standing short of a signal passes it when it moves on") {
	auto ahead{0.0};
	SUBCASE("at the signal") { ahead = 0.0; }
	SUBCASE("0.5 m") { ahead = 0.5; }
	SUBCASE("10 m") { ahead = 10.0; }
	SUBCASE("150 m") { ahead = 150.0; }
	CAPTURE(ahead);

	table_point signal{ahead, standing};
	CHECK(signal.seen_ahead);
	signal.move(0.0);
	CHECK(signal.passes == 0);
	signal.run(ahead + 50.0);
	CHECK(signal.passes == 1);
}

TEST_CASE("signal passing: a moving train passing a signal its table finds just behind the head passes it") {
	// the table traced anew in motion, the head right at the signal: the distance is found in steps of a metre,
	// so a signal just ahead may come out a little behind
	auto behind{0.0};
	SUBCASE("0.5 m") { behind = 0.5; }
	SUBCASE("3 m") { behind = 3.0; }
	SUBCASE("at the tolerance") { behind = signal_passing::moving_tolerance; }
	CAPTURE(behind);

	table_point signal{-behind, moving};
	CHECK(signal.seen_ahead);
	CHECK(signal.passes == 1);
	signal.run(100.0);
	CHECK(signal.passes == 1);
}

TEST_CASE("signal passing: a signal found further behind a moving train than the tolerance wasn't passed by it") {
	table_point signal{-(signal_passing::moving_tolerance + 0.5), moving};
	CHECK_FALSE(signal.seen_ahead);
	signal.run(100.0);
	CHECK(signal.passes == 0);
}

TEST_CASE("signal passing: a train that overruns a signal and stops past it passes it once") {
	// the overrun counts while the train moves; a table traced again with the train standing 3 m past the signal
	// (a change of direction and back, a new order) doesn't count it again
	table_point approach{150.0, moving};
	approach.run(153.0);
	CHECK(approach.passes == 1);

	table_point traced_again{-3.0, standing};
	traced_again.run(50.0);
	CHECK(traced_again.passes == 0);
}

TEST_CASE("signal passing: the report of a pass needs the point to become elapsed in this very update") {
	CHECK(signal_passing::passed_now(false, true, true));
	CHECK_FALSE(signal_passing::passed_now(true, true, true)); // elapsed before
	CHECK_FALSE(signal_passing::passed_now(false, false, true)); // still ahead
	CHECK_FALSE(signal_passing::passed_now(false, true, false)); // the head never had it ahead
}
