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
	// the game doesn't know at the end. any other name with ESt in it is an Oerlikon valve (NESt3) whose name sets
	// its relays: -ED the electrodynamic brake's one, AL2 the load-weighing one, without ESt3 the rapid one
	for (auto const *name : {"W_Lu_L", "W_Lu_XR", "W_Lu_VI", "Kg", "Kp", "Kss", "Kkg", "Kkp", "Kks", "Hikp1", "Hikss", "Hikg1", "SW", "NESt3", "M483", "CV1", "CV1_L_TR",
	                         "CV1_R", "ESt4-ED", "ESt4AL2", "no such valve"}) {
		CAPTURE(name);
		test::reseed_random();
		vehicle real{definition::fiz("3wl_v2/3wl").set("Brake", "BrakeValve", name)};
		test::trace trace{std::string{"brake cycle/fitted/"} + name, test::channels::brakes(), 100};
		real.record(trace);
		run_cycle(real, trace);
		trace.check();
	}
}

TEST_CASE("brake cycle: the rapid setting at speed, the anti-slip brake and the releaser button") {
	// R gives the cylinders more air at speed (RM); the anti-slip request the game makes while the wheels slip and
	// slow down (3) and the releaser button (ub_Release) let the LSt's cylinders out, the KE's not
	struct valve {
		char const *name;
		char const *path;
		int delay;
		double service; // in the cylinders, full service over 100 km/h
		bool releases; // by the button
	};
	for (auto const &[name, path, delay, service, releases] : {valve{"KE", "z1_v2/152a", bdelay_R, 3.78, false}, valve{"KE in P", "z1_v2/152a", bdelay_P, 2.70, false},
	                                                           valve{"LSt", "4e_v1/4e", bdelay_R, 5.79, true}, valve{"LSt in P", "4e_v1/4e", bdelay_P, 3.68, true}}) {
		CAPTURE(name);
		test::reseed_random();
		vehicle real{definition::fiz(path), 160.0};
		real->DecManualBrakeLevel(ManualBrakePosNo);
		real->BrakeDelaySwitch(delay); // false when already set
		REQUIRE(real->BrakeDelayFlag == delay);
		using test::channels::operator+;
		test::trace trace{std::string{"brake cycle/at speed/"} + name, test::channels::motion() + test::channels::brakes(), 100};
		real.record(trace);
		trace.mark("charge");
		real.hold_brake_pipe(5.0);
		real.run(60.0);
		trace.mark("full service");
		real.hold_brake_pipe(3.5);
		real.run(10.0);
		CHECK(real->Vel > 100.0);
		CHECK(real->BrakePress == doctest::Approx(service).epsilon(0.01));
		trace.mark("anti-slip");
		real->Hamulec->ASB(3);
		real.run(2.0);
		real->Hamulec->ASB(0);
		real.run(3.0);
		trace.mark("releaser button");
		real->Hamulec->SetUniversalFlag(TUniversalBrake::ub_Release);
		real.run(3.0);
		real->Hamulec->SetUniversalFlag(0);
		CHECK((real->BrakePress < 1.0) == releases);
		// released for good, the pipe still at full service
		real.run(40.0);
		CHECK((real->BrakePress < 0.1) == releases);
		trace.check();
	}
}

TEST_CASE("brake cycle: every material of the brake shoes and disks, braking a wagon from speed") {
	// the friction of the shoes (BM) against the speed and the pressing force (friction.cpp): the game's wagons
	// have P10-Bg, P10-Bgu, Cosid and the disks; FR513, FR510, P10yBg and P10yBgu none of them, nor an unknown one
	// (P10, as with no BM). Mg adds the magnetic rail brake under 3 bar in the pipe
	for (auto const *material : {"P10-Bg", "P10-Bgu", "FR513", "FR510", "Cosid", "P10yBg", "P10yBgu", "Disk1", "Disk1+Mg", "Disk2", "no such material"}) {
		CAPTURE(material);
		test::reseed_random();
		vehicle real{definition::fiz("3wl_v2/3wl").set("Brake", "BM", material), 100.0};
		real->DecManualBrakeLevel(ManualBrakePosNo);
		using test::channels::operator+;
		test::trace trace{std::string{"brake cycle/materials/"} + material, test::channels::motion() + test::channels::brakes(), 100};
		real.record(trace);
		trace.mark("charge");
		real.hold_brake_pipe(5.0);
		real.run(30.0);
		auto const speed{real->Vel};
		trace.mark("full service");
		real.hold_brake_pipe(3.5);
		real.run(20.0);
		CHECK(real->Vel < speed - 10.0);
		CHECK(real->Fb > 0.0);
		trace.mark("emergency");
		real.hold_brake_pipe(0.0);
		real.run(60.0);
		trace.check();
	}
}
