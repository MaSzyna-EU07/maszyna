#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "golden.h"
#include "real_vehicles.h"

#include "McZapkie/hamulce.h"

using test::definition;
using test::vehicle;

// characterisation of the brakes of every real vehicle: the brake pipe driven through a whole cycle, as the
// locomotive of a train would drive it, and the distributor valve, the cylinders and the reservoirs followed
// in golden traces (data/golden/brake cycle/). pressures in bar, the pipe runs at 5.0

namespace {

struct stage {
	char const *name;
	double pipe; // held brake pipe pressure
	double seconds;
};

// charging, the service application in steps, release, the overcharge used to speed the release up and its
// drop back, the emergency application and recharging after it
stage const cycle[]{
    {"charge", 5.0, 90.0},   {"light", 4.6, 20.0},         {"medium", 4.2, 20.0},      {"full service", 3.5, 20.0},
    {"release", 5.0, 40.0},  {"overcharge", 5.4, 15.0},    {"back to 5", 5.0, 45.0},   {"emergency", 0.0, 15.0},
    {"recharge", 5.0, 90.0},
};

void run_cycle(vehicle &Vehicle, test::trace &Trace) {
	for (auto const &stage : cycle) {
		Trace.mark(stage.name);
		Vehicle.hold_brake_pipe(stage.pipe);
		Vehicle.run(stage.seconds);
	}
}

} // namespace

TEST_CASE("brake cycle: every real vehicle through charge, service steps, release, overcharge and emergency") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle real{definition::fiz(path)};
		test::trace trace{"brake cycle/" + path, test::channels::brakes(), 100};
		real.record(trace);
		run_cycle(real, trace);
		trace.check();
	}
}

TEST_CASE("brake cycle: every brake delay setting of the real vehicles") {
	// G, P, R and their mixes change how fast the cylinders fill and empty; every vehicle offering more
	// than one setting goes through the service part of the cycle on each
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle probe{definition::fiz(path)};
		for (auto const delay : {bdelay_G, bdelay_P, bdelay_R, bdelay_R + bdelay_M, bdelay_G + bdelay_M}) {
			if ((probe->BrakeDelays & delay) != delay || delay == probe->BrakeDelayFlag) {
				continue;
			}
			CAPTURE(delay);
			test::reseed_random();
			vehicle real{definition::fiz(path)};
			REQUIRE(real->BrakeDelaySwitch(delay));
			test::trace trace{"brake delays/" + path + " " + std::to_string(delay), test::channels::brakes(), 100};
			real.record(trace);
			for (auto const &stage : {stage{"charge", 5.0, 90.0}, stage{"full service", 3.5, 30.0}, stage{"release", 5.0, 60.0}}) {
				trace.mark(stage.name);
				real.hold_brake_pipe(stage.pipe);
				real.run(stage.seconds);
			}
			trace.check();
		}
	}
}

TEST_CASE("brake cycle: the valves the game knows but none of its vehicles has, fitted to a wagon") {
	// the variants of the Westinghouse and Knorr valves, the Hik ones, the old ESt3 and the CV1 ones, with a name
	// the game doesn't know at the end
	for (auto const *name : {"W_Lu_L", "W_Lu_XR", "W_Lu_VI", "Kg", "Kp", "Kss", "Kkg", "Kkp", "Kks", "Hikp1", "Hikss", "Hikg1", "SW", "NESt3", "M483", "CV1", "CV1_L_TR",
	                         "no such valve"}) {
		CAPTURE(name);
		test::reseed_random();
		vehicle real{definition::fiz("3wl_v2/3wl").set("Brake", "BrakeValve", name)};
		test::trace trace{std::string{"brake cycle/fitted/"} + name, test::channels::brakes(), 100};
		real.record(trace);
		run_cycle(real, trace);
		trace.check();
	}
}
