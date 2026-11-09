#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"
#include "drive.h"

// characterisation of the auxiliary circuits of vehicles: the battery drained and charged, and the air of the
// pantographs kept in their own tank, whose pressure switch opens the line breaker, each in a scenario on real
// vehicles recorded in golden traces (data/golden/auxiliaries/)

using test::scenario;
using namespace test::channels;

namespace {

std::vector<test::channel> auxiliaries() {
	return {
	    {"BatteryVoltage", [](auto const &M) { return M.BatteryVoltage; }},
	    {"Battery", [](auto const &M) { return M.Battery ? 1.0 : 0.0; }},
	    {"ConverterFlag", [](auto const &M) { return M.ConverterFlag ? 1.0 : 0.0; }},
	    {"Power110vIsAvailable", [](auto const &M) { return M.Power110vIsAvailable ? 1.0 : 0.0; }},
	    {"PantPress", [](auto const &M) { return M.PantPress; }},
	    {"PantVolume", [](auto const &M) { return M.PantVolume; }},
	    {"PantCompFlag", [](auto const &M) { return M.PantCompFlag ? 1.0 : 0.0; }},
	    {"PantPressSwitchActive", [](auto const &M) { return M.PantPressSwitchActive ? 1.0 : 0.0; }},
	    {"PantPressLockActive", [](auto const &M) { return M.PantPressLockActive ? 1.0 : 0.0; }},
	};
}

} // namespace

TEST_CASE("auxiliaries: the battery drained by the lights and the pantographs' compressor, until nothing starts") {
	// without the converter the battery falls quickly to 90 V, then slowly as the lights and the small compressor
	// draw on it; at 70 V (110 / 1.57) it opens the line breaker, and the locomotive doesn't start again
	scenario scenario{"auxiliaries/battery/eu07 drained", traction() + auxiliaries()};
	scenario.add("4e_v1/4e", 1);
	REQUIRE(scenario.start_up());
	auto &eu07{scenario.cab()};
	REQUIRE(eu07.BatteryVoltage == doctest::Approx(eu07.NominalBatteryVoltage));
	scenario.act("converter off, lights on, small compressor on");
	eu07.ConverterSwitch(false);
	eu07.iLights[end::front] = 1 | 2 | 4;
	eu07.iLights[end::rear] = 16;
	eu07.PantCompFlag = true;
	scenario.wait(30.0);
	CHECK(eu07.BatteryVoltage < 0.83 * eu07.NominalBatteryVoltage);
	CHECK(eu07.Mains);
	scenario.wait(5000.0);
	CHECK(eu07.BatteryVoltage < eu07.NominalBatteryVoltage / 1.57);
	CHECK_FALSE(eu07.Mains);
	scenario.act("start up again");
	CHECK_FALSE(scenario.start_up());
	CHECK(eu07.BatteryVoltage < eu07.NominalBatteryVoltage / 1.57);
	scenario.check();
}

TEST_CASE("auxiliaries: the battery switched off, and charged by the converter") {
	scenario scenario{"auxiliaries/battery/eu07 switched off", traction() + auxiliaries()};
	scenario.add("4e_v1/4e", 1);
	REQUIRE(scenario.start_up());
	auto &eu07{scenario.cab()};
	scenario.act("converter off");
	eu07.ConverterSwitch(false);
	scenario.wait(10.0);
	auto const discharged{eu07.BatteryVoltage};
	CHECK(discharged < 0.83 * eu07.NominalBatteryVoltage);
	// the converter charges it back at once
	scenario.act("converter on");
	eu07.ConverterSwitch(true);
	scenario.wait(10.0);
	CHECK(eu07.BatteryVoltage == doctest::Approx(eu07.NominalBatteryVoltage));
	// switched off, the battery falls to 90 V at once, then holds
	scenario.act("battery off");
	eu07.MainSwitch(false);
	eu07.ConverterSwitch(false);
	eu07.BatterySwitch(false);
	scenario.wait(10.0);
	CHECK(eu07.BatteryVoltage == doctest::Approx(discharged).epsilon(0.01));
	scenario.check();
}

TEST_CASE("auxiliaries: the pantographs on their own tank, leaking until the pressure switch opens the line breaker") {
	// the pantographs cut off the main reservoir (bPantKurek3 off) are held up by the air of their own small tank,
	// which leaks; under MinPress the pressure switch, primed at 4.6 bar, opens the line breaker. the small
	// compressor pumps the tank up to 4.8 bar, priming the switch again. (an electric unit's switch turns its
	// converter off instead; the EW58's pantographs have an automatic valve to the main reservoir, which keeps
	// their tank up)
	scenario scenario{"auxiliaries/pantograph tank/eu07", traction() + auxiliaries()};
	scenario.add("4e_v1/4e", 1);
	REQUIRE(scenario.start_up());
	auto &eu07{scenario.cab()};
	// started up on the small compressor, the tank gets joined to the main reservoir, which primes the switch
	scenario.act("tank joined to the main reservoir");
	eu07.bPantKurek3 = true;
	scenario.wait(30.0);
	REQUIRE(eu07.PantPressSwitchActive);
	scenario.act("tank cut off the main reservoir");
	eu07.bPantKurek3 = false;
	eu07.PantCompFlag = false;
	scenario.wait(600.0);
	CHECK(eu07.PantPress < eu07.EnginePowerSource.CollectorParameters.MinPress);
	CHECK_FALSE(eu07.PantPressSwitchActive);
	CHECK_FALSE(eu07.Mains);
	scenario.act("small compressor on");
	eu07.PantCompFlag = true;
	for (int i = 0; i < 60 && eu07.PantCompFlag; ++i) {
		scenario.wait(10.0);
	}
	CHECK_FALSE(eu07.PantCompFlag);
	CHECK(eu07.PantPress >= 4.6); // stopped at 4.8, leaking since
	CHECK(eu07.PantPressSwitchActive);
	scenario.check();
}
