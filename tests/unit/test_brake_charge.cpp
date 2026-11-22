#include "stdafx.h"
#include "doctest.h"

#include "vehicle/brakecharge.h"

// a standing train starts only with its brake pipe and auxiliary reservoirs charged (TController, the consist check)

using brake_charge::charged;
using brake_charge::state;

TEST_CASE("brake charge: a train charged to within the margin may start") {
	state s;
	CHECK(charged(s, 5.0, 5.0, 0.0));
	s = {};
	CHECK(charged(s, 5.0 - brake_charge::margin, 5.0, 0.0));
	s = {};
	// the last wagon of a long train, for good below the working pressure from what leaks along the pipe
	CHECK(charged(s, 4.62, 5.0, 0.0));
}

TEST_CASE("brake charge: a train prepared from cold waits while its pipe fills") {
	state s;
	// the pipe of the last wagon empty, then filling by 0.1 bar every 5 s
	auto level{0.0};
	auto time{0.0};
	while (level < 5.0 - brake_charge::margin - 0.05) {
		CHECK_FALSE(charged(s, level, 5.0, time));
		CHECK_FALSE(s.stalled);
		level += 0.1;
		time += 5.0;
	}
	CHECK(charged(s, 5.0 - brake_charge::margin, 5.0, time));
	CHECK(s.going);
}

TEST_CASE("brake charge: a charge that stops rising lets the train go after the stall time") {
	state s;
	// a hose not connected: the last wagon stays at 3.2 bar
	CHECK_FALSE(charged(s, 3.2, 5.0, 100.0));
	CHECK_FALSE(charged(s, 3.21, 5.0, 130.0)); // less than the gain that counts
	CHECK_FALSE(charged(s, 3.21, 5.0, 100.0 + brake_charge::stall_time - 0.1));
	CHECK(charged(s, 3.21, 5.0, 100.0 + brake_charge::stall_time));
	CHECK(s.stalled);
	// and stays let go
	CHECK(charged(s, 3.2, 5.0, 300.0));
}

TEST_CASE("brake charge: a train let go stays let go while its charge rises again as it starts") {
	// the train went on a stalled charge; starting, the brakes release and the pipe refills behind them
	state s;
	REQUIRE_FALSE(charged(s, 4.3, 5.0, 0.0));
	REQUIRE(charged(s, 4.3, 5.0, brake_charge::stall_time));
	CHECK(charged(s, 4.35, 5.0, brake_charge::stall_time + 1.0));
	CHECK(charged(s, 4.4, 5.0, brake_charge::stall_time + 2.0));
	SUBCASE("until the train is to stand again: the next start is checked anew") {
		s.going = false;
		CHECK_FALSE(charged(s, 4.42, 5.0, brake_charge::stall_time + 3.0));
		CHECK(charged(s, 4.6, 5.0, brake_charge::stall_time + 4.0));
	}
}

TEST_CASE("brake charge: a train with nothing to charge with yet isn't let go for a stall") {
	// prepared from cold: the pipe empty, the engine not started, no air in the main reservoir
	state s;
	// a look every 5 s for ten times the stall time
	auto const looks{static_cast<int>(10.0 * brake_charge::stall_time / 5.0)};
	for (auto i{0}; i < looks; ++i) {
		CHECK_FALSE(charged(s, 0.0, 5.0, 5.0 * i, false));
	}
	CHECK_FALSE(s.stalled);
	CHECK_FALSE(s.going);
	// the engine runs, the pipe fills
	auto const start{10.0 * brake_charge::stall_time};
	CHECK_FALSE(charged(s, 0.0, 5.0, start, true));
	// the stall time counts from when there was air to charge with
	CHECK_FALSE(charged(s, 0.0, 5.0, start + brake_charge::stall_time - 10.0, true));
	CHECK_FALSE(charged(s, 2.0, 5.0, start + brake_charge::stall_time + 10.0, true));
	CHECK(charged(s, 4.6, 5.0, start + brake_charge::stall_time + 60.0, true));
}

TEST_CASE("brake charge: a slow but steady charge isn't taken for a stall") {
	state s;
	auto time{0.0};
	auto level{2.0};
	// 0.03 bar every 50 s: above the gain within the stall time
	for (auto i{0}; i < 20; ++i) {
		CHECK_FALSE(charged(s, level, 5.0, time));
		level += 0.03;
		time += 50.0;
	}
	CHECK_FALSE(s.stalled);
}

TEST_CASE("brake charge: a lower working pressure (the reductor turned down) lowers the charge asked for") {
	state s;
	CHECK(charged(s, 4.35, 4.8, 0.0));
	s = {};
	CHECK_FALSE(charged(s, 4.25, 4.8, 0.0));
}

TEST_CASE("brake charge: the locomotive brake holds the train only with the margin over the grade force") {
	using brake_charge::locomotive_brake_holds;
	// the 752 with about 720 t: 17 kN on -2.4 per mille, 30 kN on -4.3, against about 100 kN of its brake
	CHECK(locomotive_brake_holds(100000.0, 17000.0));
	CHECK(locomotive_brake_holds(100000.0, -30000.0));
	CHECK(locomotive_brake_holds(110001.0, 100000.0));
	CHECK_FALSE(locomotive_brake_holds(100000.0, 100000.0));
	CHECK_FALSE(locomotive_brake_holds(0.0, 1000.0));
	CHECK(locomotive_brake_holds(0.0, 0.0));
}

TEST_CASE("brake charge: standing not charged, the release of the locomotive brake waits") {
	using brake_charge::release_waits;
	// the train brake already off: what is left on is the locomotive brake, which holds the train while it charges
	CHECK(release_waits(false, true, false));
}

TEST_CASE("brake charge: the release doesn't wait once the train is charged, or once it moves") {
	using brake_charge::release_waits;
	CHECK_FALSE(release_waits(true, true, false));
	// in motion a release is braking control, not a start
	CHECK_FALSE(release_waits(false, false, false));
	CHECK_FALSE(release_waits(true, false, false));
}

TEST_CASE("brake charge: the train brake may come off while the train charges") {
	using brake_charge::release_waits;
	// released, the train brake fills the pipe, which is what charges the train
	CHECK_FALSE(release_waits(false, true, true));
}
