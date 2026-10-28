#include "stdafx.h"

#include <array>

#include "doctest.h"
#include "vehicle.h"
#include "drive.h"
#include "golden.h"
#include "real_vehicles.h"
#include "scenario.h"

#include "McZapkie/hamulce.h"

using test::definition;
using test::vehicle;

// characterisation of the drive of every real powered vehicle, driven alone from its front cab under a 3 kV
// catenary through test::drive_cycle(), in golden traces (data/golden/traction/). the cars of the electric
// units which take power or battery from the rest of their unit don't start alone: their traces record
// that, test_consist.cpp drives the whole units

TEST_CASE("traction: every real powered vehicle started up, driven to full power and braked to a stop") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle real{definition::fiz(path), 0.0, "test", 1};
		if (false == test::is_powered(real.mover())) {
			continue;
		}
		using namespace test::channels;
		test::trace trace{"traction/" + path, motion() + traction() + air() + brakes(), 100};
		real.record(trace);
		test::drive_cycle(test::driving(real), test::track{0.15, 0.0, test::catenary_3kv}, trace);
		trace.check();
	}
}

namespace {

using test::scenario;
using namespace test::channels;

void full_power(scenario &Scenario, double Seconds) {
	auto &controlling{Scenario.driver().controlling};
	// the line contactors close at the first position, after their delay (InitialCtrlDelay) and once the pressure
	// switch lets them (the cylinders under 2 bar, the pipe over 3.6), as the AI waits
	controlling.IncMainCtrl(1);
	for (int i = 0; i < 90 && false == controlling.StLinFlag; ++i) {
		Scenario.wait(1.0);
	}
	for (int i = 0; i < 60 && false == controlling.IsMainCtrlMaxPowerPos(); ++i) {
		controlling.IncMainCtrl(1);
		Scenario.wait(1.0);
	}
	Scenario.wait(Seconds);
}

// the main controller back to the given position, a notch a second
void notch_back(scenario &Scenario, int Position) {
	auto &controlling{Scenario.driver().controlling};
	while (controlling.MainCtrlPos > Position && controlling.DecMainCtrl(1)) {
		Scenario.wait(1.0);
	}
}

void stop(scenario &Scenario) {
	notch_back(Scenario, 0);
	Scenario.act("brake");
	Scenario.cab().BrakeLevelSet(Scenario.cab().Handle->GetPos(bh_FB));
	Scenario.wait(60.0);
}

} // namespace

TEST_CASE("traction: the line breaker opened by the catenary lost, and by too high a voltage") {
	// the voltage relays of the series motors, and the induction motors' own limits: under MinV of the current
	// collector, or over MaxV (with the overvoltage protection, OVP) and over MaxV + 200 V for the induction motors
	for (auto const &[name, vehicles] : std::vector<std::pair<std::string, std::vector<char const *>>>{
	         {"eu07", {"4e_v1/4e"}}, {"et42", {"et42_v2/112e-a", "et42_v2/112e-b"}}, {"eu47", {"e186_v2/eu47"}}}) {
		CAPTURE(name);
		scenario scenario{"traction/catenary/" + name, motion() + traction()};
		for (auto const *path : vehicles) {
			scenario.add(path, scenario.train().size() == 0 ? 1 : 0);
		}
		scenario.add("bdhpumn_v3/bdhpumn");
		REQUIRE(scenario.start_up());
		scenario.get_going();
		auto &controlling{scenario.driver().controlling};
		full_power(scenario, 10.0);
		REQUIRE(controlling.Mains);
		scenario.act("catenary lost");
		scenario.line().voltage = 0.0;
		scenario.wait(5.0);
		CHECK_FALSE(controlling.Mains);
		CHECK(controlling.Im == doctest::Approx(0.0));
		scenario.act("catenary back");
		scenario.line().voltage = test::catenary_3kv;
		notch_back(scenario, 0);
		CHECK(scenario.start_up());
		full_power(scenario, 5.0);
		scenario.act("overvoltage");
		scenario.line().voltage = controlling.EnginePowerSource.CollectorParameters.MaxV + 300.0;
		scenario.wait(5.0);
		CHECK(controlling.Mains == (controlling.EngineType == TEngineType::ElectricSeriesMotor && false == controlling.EnginePowerSource.CollectorParameters.OVP));
		scenario.line().voltage = test::catenary_3kv;
		stop(scenario);
		scenario.check();
	}
}

TEST_CASE("traction: the ET42 at the high current setting, its halves in series, and its switched electrodynamic brake") {
	for (auto const high : {false, true}) {
		CAPTURE(high);
		scenario scenario{std::string{"traction/et42/"} + (high ? "high current" : "low current"), motion() + traction() + brakes()};
		scenario.add("et42_v2/112e-a", 1).add("et42_v2/112e-b").add("3wl_v2/3wl").add("3wl_v2/3wl").add("3wl_v2/3wl");
		REQUIRE(scenario.start_up());
		auto &et42{scenario.cab()};
		scenario.act("current setting");
		CHECK(et42.CurrentSwitch(high) == high);
		// the relay takes the setting at the next step, with the controller at zero
		scenario.get_going();
		CHECK((et42.Imax == et42.ImaxHi) == high);
		full_power(scenario, 60.0);
		// at the high current setting the controller stops at the last series position: the halves' motors stay
		// in series, at half the voltage each, for a heavy start, and the train crawls
		CHECK((et42.RList[et42.MainCtrlPos].Bn > 1) != high);
		CHECK(et42.Vel > (high ? 30.0 : 90.0));
		CHECK(et42.Vel < (high ? 50.0 : 110.0));
		scenario.act("controller to zero");
		while (et42.DecMainCtrl(2)) {
		}
		scenario.wait(2.0);
		scenario.act("dynamic brake");
		CHECK(et42.DynamicBrakeSwitch(true));
		scenario.wait(5.0);
		full_power(scenario, 20.0);
		// the controller sets the brake's strength. from the low setting's top speed, over 100 km/h, the motors'
		// current goes over 300 A at the first position and the overload relay trips, leaving the train to roll;
		// from the high setting's, under 40 km/h, the brake stops the train
		CHECK(et42.FuseFlag != high);
		CHECK((et42.Vel == doctest::Approx(0.0)) == high);
		scenario.act("dynamic brake off");
		notch_back(scenario, 0);
		et42.DynamicBrakeSwitch(false);
		stop(scenario);
		scenario.check();
	}
}

TEST_CASE("traction: the field shunting relays of the diesel-electric locomotives, at full power and notched back") {
	// each kind of the automatic relays (RelayType) the game's diesel-electrics have, run with a wagon up to full
	// shunting (and past their top speed, which nothing limits) and notched back through the positions where the
	// relays let the shunting go: by the current (0), by the speed alone (1, which keeps it at any position), by
	// the current at the full position only (41), by the position and the speed (45, 46)
	struct locomotive {
		char const *path;
		int relay;
		std::array<int, 4> shunting; // after notching back to 11, 10, 7 and 3
	};
	for (auto const &[path, relay, shunting] : {locomotive{"cd/754_v1/754", 0, {3, 3, 3, 0}}, locomotive{"sm31_v1/411d", 1, {2, 2, 2, 2}},
	                                            locomotive{"sm40_v1/sm40", 41, {1, 1, 1, 1}}, locomotive{"su45_v2/301d", 45, {3, 2, 0, 0}},
	                                            locomotive{"su46_v2/303d", 46, {4, 2, 0, 0}}}) {
		CAPTURE(path);
		scenario scenario{std::string{"traction/relays/"} + path, motion() + traction()};
		scenario.add(path, 1).add("bdhpumn_v3/bdhpumn");
		REQUIRE(scenario.start_up());
		auto &controlling{scenario.driver().controlling};
		CHECK(controlling.RelayType == relay);
		scenario.get_going();
		full_power(scenario, 120.0);
		CHECK(controlling.ScndCtrlPos == controlling.ScndCtrlPosNo);
		auto expected{shunting.begin()};
		for (auto const position : {11, 10, 7, 3}) {
			CAPTURE(position);
			scenario.act("notch back to " + std::to_string(position));
			notch_back(scenario, position);
			scenario.wait(15.0);
			CHECK(controlling.ScndCtrlPos == *expected++);
		}
		stop(scenario);
		scenario.check();
	}
}
