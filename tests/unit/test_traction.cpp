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

void stop(scenario &Scenario) {
	Scenario.notch_back(0);
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
		scenario.full_power(10.0);
		REQUIRE(controlling.Mains);
		scenario.act("catenary lost");
		scenario.line().voltage = 0.0;
		scenario.wait(5.0);
		CHECK_FALSE(controlling.Mains);
		CHECK(controlling.Im == doctest::Approx(0.0));
		scenario.act("catenary back");
		scenario.line().voltage = test::catenary_3kv;
		scenario.notch_back(0);
		CHECK(scenario.start_up());
		scenario.full_power(5.0);
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
		scenario.full_power(60.0);
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
		scenario.full_power(20.0);
		// the controller sets the brake's strength. from the low setting's top speed, over 100 km/h, the motors'
		// current goes over 300 A at the first position and the overload relay trips, leaving the train to roll;
		// from the high setting's, under 40 km/h, the brake stops the train
		CHECK(et42.FuseFlag != high);
		CHECK((et42.Vel == doctest::Approx(0.0)) == high);
		scenario.act("dynamic brake off");
		scenario.notch_back(0);
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
		scenario.full_power(120.0);
		CHECK(controlling.ScndCtrlPos == controlling.ScndCtrlPosNo);
		auto expected{shunting.begin()};
		for (auto const position : {11, 10, 7, 3}) {
			CAPTURE(position);
			scenario.act("notch back to " + std::to_string(position));
			scenario.notch_back(position);
			scenario.wait(15.0);
			CHECK(controlling.ScndCtrlPos == *expected++);
		}
		stop(scenario);
		scenario.check();
	}
}

TEST_CASE("traction: the stages of the EW58's electrodynamic brake, set by the positions of its ep handle") {
	// the FVE408's positions 1 to 5 give the ep brake's strength, whose whole part is the stage of the motors'
	// brake (TUHEX): 1, 1, 2, 3, 3; over 100 km/h the motors brake at the first stage alone
	scenario scenario{"traction/ew58 brake stages", motion() + traction() + brakes()};
	for (auto const *car : {"ew58_v1/ew58-sa", "ew58_v1/ew58-d", "ew58_v1/ew58-sb"}) {
		scenario.add(car, scenario.train().size() == 0 ? 1 : 0, test::consist::full_coupling, 115.0);
	}
	REQUIRE(scenario.start_up());
	auto &ew58{scenario.cab()};
	for (auto const &[position, stage] : {std::pair{1, 1}, std::pair{3, 2}, std::pair{4, 3}, std::pair{2, 1}, std::pair{5, 3}}) {
		CAPTURE(position);
		scenario.act("position " + std::to_string(position));
		ew58.BrakeLevelSet(position);
		scenario.wait(6.0);
		CHECK(ew58.TUHEX_StageActual == stage);
		CHECK(ew58.DynamicBrakeFlag);
		CHECK(ew58.Im < 0.0);
	}
	scenario.act("release");
	ew58.BrakeLevelSet(ew58.Handle->GetPos(bh_RP));
	scenario.wait(10.0);
	CHECK(ew58.TUHEX_StageActual == 0);
	scenario.check();
}

TEST_CASE("traction: the induction motor drive's speed control, its electrodynamic brake by the local brake, and at the voltage's limit") {
	scenario scenario{"traction/eu47 drive", motion() + traction() + brakes()};
	scenario.add("e186_v2/eu47", 1).add("bdhpumn_v3/bdhpumn").add("bdhpumn_v3/bdhpumn");
	REQUIRE(scenario.start_up());
	auto &eu47{scenario.cab()};
	scenario.get_going();
	// the second controller of the EU47 sets the speed to hold (SpeedCtrlType=Time): position 2 keeps the speed
	// set, 3 raises it by 10 km/h every SpeedCtrlDelay, 4 sets the top speed, 1 lowers it, 0 drops it
	REQUIRE(eu47.ScndCtrlPosNo == 4);
	REQUIRE(eu47.SpeedCtrlTypeTime);
	scenario.full_power(0.0);
	scenario.act("speed up");
	for (int i = 0; i < 3; ++i) {
		eu47.IncScndCtrl(1);
	}
	scenario.wait(3.0);
	scenario.act("hold");
	eu47.DecScndCtrl(1);
	scenario.wait(1.0);
	CHECK(eu47.SpeedCtrlValue == doctest::Approx(60.0));
	scenario.wait(60.0);
	CHECK(eu47.Vel == doctest::Approx(60.0).epsilon(0.01));
	scenario.act("slow down");
	eu47.DecScndCtrl(1);
	scenario.wait(1.5);
	eu47.IncScndCtrl(1);
	scenario.wait(1.0);
	CHECK(eu47.SpeedCtrlValue == doctest::Approx(20.0));
	// a lower speed set only takes the power off, the train rolls on
	scenario.wait(40.0);
	CHECK(eu47.Vel > 50.0);
	scenario.act("top speed");
	eu47.IncScndCtrl(2);
	scenario.wait(5.0);
	CHECK(eu47.SpeedCtrlValue == doctest::Approx(eu47.Vmax));
	scenario.act("speed control off");
	while (eu47.DecScndCtrl(1)) {
	}
	scenario.notch_back(0);
	scenario.wait(2.0);
	CHECK(eu47.SpeedCtrlValue == doctest::Approx(0.0));
	// the local brake sets the flag of the electrodynamic brake, which keeps the train brake's air out of the
	// cylinders, but the motors' brake follows the train brake alone: the local brake's air does the braking
	scenario.act("local brake");
	eu47.IncLocalBrakeLevel(LocalBrakePosNo);
	scenario.wait(5.0);
	CHECK(eu47.DynamicBrakeFlag);
	CHECK(eu47.Im == doctest::Approx(0.0));
	CHECK(eu47.BrakePress > 2.0);
	scenario.act("local brake off");
	eu47.DecLocalBrakeLevel(LocalBrakePosNo);
	scenario.wait(10.0);
	// braking back into the catenary: at its usual voltage, and at its highest, over the drive's MaxV, where
	// the drive lets less of the current go back to it (Vadd)
	for (auto const voltage : {test::catenary_3kv, eu47.EnginePowerSource.CollectorParameters.MaxV + 100.0}) {
		CAPTURE(voltage);
		scenario.line().voltage = voltage;
		scenario.act("service brake at " + std::to_string(static_cast<int>(voltage)) + " V");
		eu47.BrakeLevelSet(eu47.Handle->GetPos(bh_FB));
		scenario.wait(6.0);
		CHECK(eu47.DynamicBrakeFlag);
		CHECK(eu47.Im < -100.0);
		CHECK((eu47.Vadd > 1.0) == (voltage > eu47.EnginePowerSource.CollectorParameters.MaxV));
		eu47.BrakeLevelSet(eu47.Handle->GetPos(bh_RP));
		scenario.wait(6.0);
	}
	scenario.line().voltage = test::catenary_3kv;
	scenario.act("line breaker off");
	eu47.MainSwitch(false);
	scenario.wait(5.0);
	CHECK(eu47.Im == doctest::Approx(0.0));
	stop(scenario);
	scenario.check();
}
