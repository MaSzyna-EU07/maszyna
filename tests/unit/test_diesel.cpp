#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"
#include "drive.h"

#include "McZapkie/hamulce.h"

// characterisation of the diesel engines besides their drive, recorded in golden traces (data/golden/diesel/):
// the heat of the engine, its oil and its water circuits, which keep a too cold or too hot engine from starting

using test::scenario;
using namespace test::channels;

namespace {

std::vector<test::channel> heat() {
	return {
	    {"Ts", [](auto const &M) { return M.dizel_heat.Ts; }},
	    {"To", [](auto const &M) { return M.dizel_heat.To; }},
	    {"Twy", [](auto const &M) { return M.dizel_heat.Twy; }},
	    {"Twy2", [](auto const &M) { return M.dizel_heat.Twy2; }},
	    {"PA", [](auto const &M) { return M.dizel_heat.PA ? 1.0 : 0.0; }},
	    {"WaterHeater", [](auto const &M) { return M.WaterHeater.is_active ? 1.0 : 0.0; }},
	    {"WaterPump", [](auto const &M) { return M.WaterPump.is_active ? 1.0 : 0.0; }},
	    {"zaluzje1", [](auto const &M) { return M.dizel_heat.zaluzje1 ? 1.0 : 0.0; }},
	    {"zaluzje2", [](auto const &M) { return M.dizel_heat.zaluzje2 ? 1.0 : 0.0; }},
	};
}

} // namespace

TEST_CASE("diesel: an engine too cold to start, warmed by its heater") {
	// at -10 degrees an engine with the lowest temperature of its water set (WaterMinTemperature) doesn't start,
	// until the heater, which the driver switches on with the pumps, warms the water up; one without starts
	struct engine {
		char const *path;
		bool starts; // cold
	};
	for (auto const &[path, starts] : {engine{"su45_v2/301d", false}, engine{"sm42_v1/6d1", false}, engine{"st44_v2/m62-1151", true}}) {
		CAPTURE(path);
		scenario scenario{std::string{"diesel/cold/"} + path, traction() + heat(), test::track{}};
		scenario.add(path, 1);
		auto &locomotive{scenario.cab()};
		locomotive.dizel_HeatSet(-10.f);
		CHECK(scenario.start_up() == starts);
		CHECK(locomotive.dizel_heat.PA != starts);
		CHECK(locomotive.WaterHeater.is_active);
		scenario.act("heating");
		scenario.wait(1200.0);
		CHECK_FALSE(locomotive.dizel_heat.PA);
		CHECK(locomotive.dizel_heat.Twy > 50.f);
		CHECK(scenario.start_up());
		scenario.check();
	}
}

TEST_CASE("diesel: an engine started hot, and run at full power") {
	// at 88 degrees the engines with a second water circuit are over its highest temperature
	// (WaterAuxMaxTemperature) and don't start while it cools; the SM42 has none, starts and gets cooled by its
	// fans and shutters at full power. alone: a train's pipe, leaking under 2.75 bar while the engine
	// doesn't start, gets cut off the FV4a handle of an LSt locomotive (lock_old in UpdatePipePressure)
	struct engine {
		char const *path;
		bool starts;
	};
	for (auto const &[path, starts] : {engine{"su45_v2/301d", false}, engine{"sm42_v1/6d1", true}, engine{"st44_v2/m62-1151", false}}) {
		CAPTURE(path);
		scenario scenario{std::string{"diesel/hot/"} + path, traction() + heat(), test::track{}};
		scenario.add(path, 1);
		auto &locomotive{scenario.cab()};
		locomotive.dizel_HeatSet(88.f);
		CHECK(scenario.start_up() == starts);
		CHECK(locomotive.dizel_heat.PA != starts);
		if (starts) {
			scenario.get_going();
			scenario.full_power(300.0);
			CHECK(locomotive.Vel > 50.0);
			CHECK(locomotive.dizel_heat.zaluzje1);
			CHECK(locomotive.dizel_heat.Twy < locomotive.dizel_heat.water.config.temp_max);
		}
		scenario.check();
	}
}

TEST_CASE("diesel: the 214M railbus's torque converter, gears and retarder") {
	// the controller of the railbus (EIMCtrlType 3) runs from the air brake (0, 1) through the retarder (2) and
	// neutral (3) to holding (4) and raising (5) the power; the torque converter locks up at speed and the
	// gearbox shifts by itself
	scenario scenario{"diesel/214m", motion() + traction() + brakes(), test::track{}};
	scenario.add("21xm_v1/214ma", 1);
	REQUIRE(scenario.start_up());
	auto &railbus{scenario.cab()};
	scenario.get_going();
	scenario.full_power(60.0);
	CHECK(railbus.MainCtrlPos == 5);
	CHECK(railbus.eimic_real == doctest::Approx(1.0));
	CHECK(railbus.hydro_TC_Lockup);
	CHECK(railbus.ScndCtrlActualPos == railbus.ScndCtrlPosNo);
	struct position {
		double eimic;
		bool retarder;
		bool air;
	};
	auto number{railbus.MainCtrlPos};
	for (auto const &[eimic, retarder, air] : {position{1.0, false, false}, position{0.0, false, false}, position{-1.0, true, false}, position{0.0, false, true}}) {
		--number;
		CAPTURE(number);
		scenario.act("position " + std::to_string(number));
		railbus.DecMainCtrl(1);
		scenario.wait(8.0);
		CHECK(railbus.eimic_real == doctest::Approx(eimic));
		CHECK((railbus.hydro_R_Fill > 0.99) == retarder);
		CHECK((railbus.hydro_R_Torque > 1000.0) == retarder);
		CHECK((railbus.BrakePress > 2.0) == air);
	}
	scenario.act("position 0");
	railbus.DecMainCtrl(1);
	scenario.wait(30.0);
	CHECK(railbus.Vel == doctest::Approx(0.0));
	// the gearbox shifted down as the railbus slowed
	CHECK(railbus.ScndCtrlActualPos < railbus.ScndCtrlPosNo);
	scenario.check();
}

TEST_CASE("diesel: the 214M railbus's speed control, up to the speed of a button and braking down to a lower one") {
	// the second controller's first position takes the speed of the moment to hold; the buttons, the steps and
	// the power limit change it. set lower than the speed, it brakes with the retarder (and under
	// BrakeInterventionVel with the air); with the main controller off its top positions it stands by
	scenario scenario{"diesel/214m speed control", motion() + traction() + brakes(), test::track{}};
	scenario.add("21xm_v1/214ma", 1);
	REQUIRE(scenario.start_up());
	auto &railbus{scenario.cab()};
	REQUIRE(railbus.SpeedCtrl);
	scenario.get_going();
	scenario.full_power(10.0);
	scenario.act("speed control on");
	REQUIRE(railbus.IncScndCtrl(1));
	CHECK(railbus.SpeedCtrlUnit.IsActive);
	CHECK(railbus.SpeedCtrlValue == doctest::Approx(std::round(railbus.Vel)).epsilon(0.02));
	scenario.act("button 80");
	railbus.SpeedCtrlButton(6);
	CHECK(railbus.SpeedCtrlValue == doctest::Approx(80.0));
	scenario.wait(90.0);
	CHECK(railbus.Vel == doctest::Approx(80.0).epsilon(0.02));
	CHECK_FALSE(railbus.SpeedCtrlUnit.Standby);
	scenario.act("step down, up, power down and up");
	railbus.SpeedCtrlDec();
	CHECK(railbus.SpeedCtrlValue == doctest::Approx(75.0));
	railbus.SpeedCtrlInc();
	railbus.SpeedCtrlInc();
	CHECK(railbus.SpeedCtrlValue == doctest::Approx(85.0));
	CHECK(railbus.SpeedCtrlUnit.DesiredPower == doctest::Approx(1.0));
	CHECK(railbus.SpeedCtrlPowerDec());
	CHECK(railbus.SpeedCtrlUnit.DesiredPower == doctest::Approx(0.9));
	CHECK(railbus.SpeedCtrlPowerInc());
	CHECK(railbus.SpeedCtrlUnit.DesiredPower == doctest::Approx(1.0));
	scenario.wait(20.0);
	scenario.act("button 40");
	railbus.SpeedCtrlButton(2);
	scenario.wait(5.0);
	// the retarder does it at this speed, the air isn't needed
	CHECK(railbus.eimic_real == doctest::Approx(-1.0));
	CHECK(railbus.hydro_R_Fill == doctest::Approx(1.0));
	CHECK(railbus.BrakePress < 0.1);
	CHECK_FALSE(railbus.SpeedCtrlUnit.BrakeInterventionBraking);
	scenario.wait(60.0);
	CHECK(railbus.Vel == doctest::Approx(40.0).epsilon(0.05));
	scenario.act("main controller back, standing by");
	railbus.DecMainCtrl(1);
	railbus.DecMainCtrl(1);
	railbus.DecMainCtrl(1);
	scenario.wait(5.0);
	CHECK(railbus.SpeedCtrlUnit.Standby);
	scenario.act("speed control off");
	railbus.DecScndCtrl(2);
	scenario.wait(5.0);
	CHECK_FALSE(railbus.SpeedCtrlUnit.IsActive);
	scenario.check();
}
