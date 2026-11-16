#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"
#include "drive.h"

#include <cmath>

// the cruise control (tempomat, SpeedCtrl) and the secondary controller (ScndCtrl) of the vehicles which have one
// of them or both (eu07/maszyna#69). on the vehicles with the cruise control alone the secondary controller is
// its switch, as it always was; on those whose secondary controller shunts the field of the traction motors
// (the 6Dg, the ST45 and the BR285 among them) the cruise control has a switch of its own, apart from it

using test::scenario;
using namespace test::channels;

namespace {

std::vector<test::channel> cruise() {
	return {
	    {"SpeedCtrlActive", [](auto const &M) { return M.SpeedCtrlUnit.IsActive ? 1.0 : 0.0; }},
	    {"SpeedCtrlStandby", [](auto const &M) { return M.SpeedCtrlUnit.Standby ? 1.0 : 0.0; }},
	    {"SpeedCtrlValue", [](auto const &M) { return M.SpeedCtrlValue; }},
	    {"eimic", [](auto const &M) { return M.eimic; }},
	    {"eimicSpeedCtrl", [](auto const &M) { return M.eimicSpeedCtrl; }},
	    {"eimic_real", [](auto const &M) { return M.eimic_real; }},
	};
}

// the universal controller (EIMCtrlType 3) of the 6Dg: 3 no power, 4 lowers the demand, 5 holds it, 6 raises it
constexpr int dg_no_power{3};
constexpr int dg_lower{4};
constexpr int dg_hold{5};
constexpr int dg_raise{6};

void move_to(TMoverParameters &Vehicle, int const Position) {
	while (Vehicle.MainCtrlPos < Position && Vehicle.IncMainCtrl(1)) {
		// all the work is done in the condition
	}
	while (Vehicle.MainCtrlPos > Position && Vehicle.DecMainCtrl(1)) {
		// all the work is done in the condition
	}
	REQUIRE(Vehicle.MainCtrlPos == Position);
}

// the 6Dg started up, its brakes released, its controller through the no power and lowering positions to holding,
// for the line contactors to close
void get_6dg_going(scenario &Scenario) {
	auto &loco{Scenario.cab()};
	REQUIRE(Scenario.start_up());
	Scenario.get_going();
	move_to(loco, dg_no_power);
	for (int second = 0; second < 180 && loco.BrakePress > 0.1; ++second) {
		Scenario.wait(1.0);
	}
	REQUIRE(loco.BrakePress <= 0.1);
	move_to(loco, dg_lower);
	Scenario.wait(1.0);
	move_to(loco, dg_hold);
	Scenario.wait(5.0);
	REQUIRE(loco.StLinFlag);
}

} // namespace

// the vehicles with the field shunting alone, and those with the cruise control alone: as before the split

TEST_CASE("speed control: the field shunting of the vehicles without a cruise control, as before") {
	// the EU07's and the SM42's shunting by hand, the SU45's by itself (RelayType 45, from position 11 of its
	// controller on); none has a cruise control, and the secondary controller shunts the field
	SUBCASE("EU07 and SM42, by hand") {
		for (auto const *path : {"4e_v1/4e", "sm42_v1/6d1"}) {
			CAPTURE(path);
			scenario scenario{std::string{"speed control/shunting/"} + path, motion() + traction() + cruise()};
			scenario.add(path, 1);
			auto &loco{scenario.cab()};
			REQUIRE_FALSE(loco.SpeedCtrl);
			REQUIRE_FALSE(loco.AutoRelayFlag);
			REQUIRE(scenario.start_up());
			scenario.get_going();
			scenario.full_power(60.0);
			auto const speed{loco.Vel};
			scenario.act("shunt");
			CHECK(loco.IncScndCtrl(1));
			CHECK(loco.ScndCtrlPos == 1);
			scenario.wait(10.0);
			CHECK(loco.Vel > speed);
			CHECK(loco.ScndCtrlPos == 1);
			CHECK_FALSE(loco.SpeedCtrlUnit.IsActive);
			CHECK(loco.SpeedCtrlValue == 0.0);
			scenario.act("shunt off");
			CHECK(loco.DecScndCtrl(2));
			CHECK(loco.ScndCtrlPos == 0);
			scenario.wait(5.0);
			CHECK_FALSE(loco.SpeedCtrlUnit.IsActive);
			scenario.check();
		}
	}
	SUBCASE("SU45, by itself") {
		scenario scenario{"speed control/shunting/su45_v2/301d", motion() + traction() + cruise()};
		scenario.add("su45_v2/301d", 1);
		auto &loco{scenario.cab()};
		REQUIRE_FALSE(loco.SpeedCtrl);
		REQUIRE(loco.AutoRelayFlag);
		REQUIRE(scenario.start_up());
		scenario.get_going();
		scenario.full_power(5.0);
		auto most{0};
		for (int i = 0; i < 18; ++i) {
			scenario.wait(10.0);
			most = std::max(most, loco.ScndCtrlPos);
		}
		CHECK(most == loco.ScndCtrlPosNo);
		// the driver can't move it
		CHECK_FALSE(loco.DecScndCtrl(2));
		CHECK_FALSE(loco.SpeedCtrlUnit.IsActive);
		scenario.act("notch back");
		scenario.notch_back(0);
		scenario.wait(5.0);
		CHECK(loco.ScndCtrlPos == 0);
		scenario.check();
	}
}

TEST_CASE("speed control: the vehicles whose secondary controller is the cruise control's switch, as before") {
	SUBCASE("Impuls: on at the speed of the moment, off by the controller") {
		scenario scenario{"speed control/cruise/impuls_v1/31we-a_pr", motion() + traction() + cruise()};
		scenario.add("impuls_v1/31we-a_pr", 1).add("impuls_v1/31we-b_pr").add("impuls_v1/31we-c_pr").add("impuls_v1/31we-d_pr");
		auto &unit{scenario.cab()};
		REQUIRE(unit.SpeedCtrl);
		REQUIRE(unit.ScndCtrlPosNo == 1);
		REQUIRE(scenario.start_up());
		scenario.get_going();
		scenario.full_power(40.0);
		scenario.act("cruise control on");
		CHECK(unit.IncScndCtrl(1));
		CHECK(unit.ScndCtrlPos == 1);
		CHECK(unit.SpeedCtrlUnit.IsActive);
		CHECK(unit.SpeedCtrlValue == doctest::Approx(unit.Vel));
		auto const held{unit.SpeedCtrlValue};
		scenario.wait(60.0);
		CHECK(unit.Vel == doctest::Approx(held).epsilon(0.05));
		scenario.act("cruise control off");
		CHECK(unit.DecScndCtrl(2));
		CHECK(unit.ScndCtrlPos == 0);
		CHECK_FALSE(unit.SpeedCtrlUnit.IsActive);
		CHECK(unit.SpeedCtrlValue == 0.0);
		scenario.wait(5.0);
		scenario.check();
	}
	SUBCASE("EU47: the speed raised by tens while the controller is held up, set when it's let go") {
		scenario scenario{"speed control/cruise/e186_v2/eu47", motion() + traction() + cruise()};
		scenario.add("e186_v2/eu47", 1);
		auto &loco{scenario.cab()};
		REQUIRE(loco.SpeedCtrl);
		REQUIRE(loco.SpeedCtrlTypeTime);
		REQUIRE(loco.ScndCtrlPosNo == 4);
		REQUIRE(scenario.start_up());
		scenario.get_going();
		scenario.act("raise");
		while (loco.ScndCtrlPos < 3 && loco.IncScndCtrl(1)) {
			// all the work is done in the condition
		}
		REQUIRE(loco.ScndCtrlPos == 3);
		scenario.wait(10.0);
		CHECK(loco.NewSpeed > 0.0);
		scenario.act("set");
		CHECK(loco.DecScndCtrl(1));
		scenario.wait(1.0);
		CHECK(loco.SpeedCtrlValue == doctest::Approx(loco.NewSpeed));
		CHECK(loco.SpeedCtrlUnit.IsActive);
		scenario.full_power(60.0);
		CHECK(loco.Vel == doctest::Approx(loco.SpeedCtrlValue).epsilon(0.1));
		scenario.act("off");
		CHECK(loco.DecScndCtrl(2));
		scenario.wait(1.0);
		CHECK_FALSE(loco.SpeedCtrlUnit.IsActive);
		scenario.check();
	}
}

// the vehicles with both: the field shunting as before, the cruise control on a switch of its own

TEST_CASE("speed control: the field shunting of the vehicles with a cruise control besides, as before") {
	SUBCASE("6Dg") {
		scenario scenario{"speed control/both/6dg_v1/6dg", motion() + traction() + cruise(), test::track{}};
		scenario.add("6dg_v1/6dg", 1);
		auto &loco{scenario.cab()};
		REQUIRE(loco.SpeedCtrl);
		REQUIRE(loco.AutoRelayFlag);
		REQUIRE(loco.ScndCtrlPosNo == 1);
		get_6dg_going(scenario);
		scenario.act("raise");
		move_to(loco, dg_raise);
		// the shunt engages by itself at about 40 km/h (RelayType 0, by the current of the motors)
		scenario.wait(10.0);
		CHECK(loco.Vel < 35.0);
		CHECK(loco.ScndCtrlPos == 0);
		scenario.wait(110.0);
		CHECK(loco.Vel > 100.0);
		CHECK(loco.ScndCtrlPos == 1);
		CHECK_FALSE(loco.SpeedCtrlUnit.IsActive);
		CHECK(loco.eimic_real == doctest::Approx(loco.eimic));
		// the driver can't move the shunt
		CHECK_FALSE(loco.IncScndCtrl(1));
		CHECK_FALSE(loco.DecScndCtrl(2));
		scenario.check();
	}
	SUBCASE("ST45") {
		scenario scenario{"speed control/both/st45_v2/301dd", motion() + traction() + cruise(), test::track{}};
		scenario.add("st45_v2/301dd", 1);
		auto &loco{scenario.cab()};
		REQUIRE(loco.SpeedCtrl);
		REQUIRE(loco.AutoRelayFlag);
		REQUIRE(scenario.start_up());
		scenario.get_going();
		scenario.full_power(5.0);
		// its relays (RelayType 45) engage the shunt from position 11 of the controller on, which the universal
		// controller of the ST45, of 4 positions, never gets to: the shunt stays off
		auto most{0};
		for (int i = 0; i < 9; ++i) {
			scenario.wait(10.0);
			most = std::max(most, loco.ScndCtrlPos);
		}
		CHECK(loco.Vel > 130.0);
		CHECK(most == 0);
		CHECK_FALSE(loco.SpeedCtrlUnit.IsActive);
		CHECK(loco.eimic_real == doctest::Approx(loco.eimic));
		// the speed buttons do nothing with the cruise control off
		loco.SpeedCtrlButton(4);
		loco.SpeedCtrlInc();
		CHECK(loco.SpeedCtrlValue == 0.0);
		CHECK_FALSE(loco.IncScndCtrl(1));
		scenario.check();
	}
}

TEST_CASE("speed control apart: which vehicles' cruise control has a switch of its own") {
	struct kind {
		char const *path;
		bool separate;
	};
	for (auto const &[path, separate] : {
	         // the secondary controller shunts the field: a switch of its own
	         kind{"6dg_v1/6dg", true}, kind{"st45_v2/301dd", true}, kind{"br285_v1/br285", true},
	         // the secondary controller is the switch, the speed setter or the gears' selector of the automatic gearbox
	         kind{"21xm_v1/214ma", false}, kind{"21xm_v1/218ma_a_kd", false}, kind{"sa134_v1/218mc_a", false}, kind{"impuls_v1/31we-a_pr", false},
	         kind{"elf_v1/21we-a", false}, kind{"e186_v2/eu47", false}, kind{"e6act_v2/e6acta", false}, kind{"en57akl_v1/akl_ra", false},
	         // no cruise control
	         kind{"4e_v1/4e", false}, kind{"sm42_v1/6d1", false}, kind{"su45_v2/301d", false}, kind{"ep09_v2/104e", false}}) {
		CAPTURE(path);
		test::vehicle vehicle{test::definition::fiz(path)};
		CHECK(vehicle->IsSpeedCtrlSeparate() == separate);
		CHECK_FALSE(vehicle->IsSpeedCtrlEngaged());
	}
	// with a cruise control and no secondary controller, the switch is its own too
	test::vehicle eu07{test::definition::fiz("4e_v1/4e").set("Cntrl", "SCPN", 0).set("SpeedControl", "SpeedCtrl", "Yes")};
	CHECK(eu07->IsSpeedCtrlSeparate());
}

TEST_CASE("speed control apart: the 6Dg's cruise control on its own switch, the field shunting going on by itself") {
	scenario scenario{"speed control apart/6dg_v1/6dg", motion() + traction() + cruise(), test::track{}};
	scenario.add("6dg_v1/6dg", 1);
	auto &loco{scenario.cab()};
	get_6dg_going(scenario);
	scenario.act("raise");
	move_to(loco, dg_raise);
	scenario.wait(30.0);
	REQUIRE(loco.Vel > 50.0);
	REQUIRE(loco.ScndCtrlPos == 1); // the shunt in
	// the shunt is no cruise control: its buttons do nothing yet, the secondary controller can't switch it on
	CHECK_FALSE(loco.IsSpeedCtrlEngaged());
	loco.SpeedCtrlButton(2);
	CHECK(loco.SpeedCtrlValue == 0.0);
	CHECK_FALSE(loco.IncScndCtrl(1));
	CHECK_FALSE(loco.SpeedCtrlUnit.IsActive);

	scenario.act("cruise control on");
	CHECK(loco.SpeedCtrlSwitch(true));
	CHECK_FALSE(loco.SpeedCtrlSwitch(true)); // on already
	CHECK(loco.SpeedCtrlUnit.IsActive);
	CHECK(loco.IsSpeedCtrlEngaged());
	CHECK(loco.SpeedCtrlValue == doctest::Approx(std::round(loco.Vel)).epsilon(0.02));
	CHECK(loco.ScndCtrlPos == 1); // the shunt as it was
	auto const held{loco.SpeedCtrlValue};
	// the shunt goes by itself all the while, by the current of the motors, which the cruise control's demand sets
	auto shunted{0};
	auto unshunted{0};
	auto const run{[&scenario, &loco, &shunted, &unshunted](int const Seconds) {
		for (int second = 0; second < Seconds; ++second) {
			scenario.wait(1.0);
			++(loco.ScndCtrlPos > 0 ? shunted : unshunted);
		}
	}};
	run(60);
	CHECK(loco.Vel == doctest::Approx(held).epsilon(0.03));
	CHECK_FALSE(loco.SpeedCtrlUnit.Standby);
	CHECK(loco.eimic_real < loco.eimic); // the cruise control holding the power back

	scenario.act("two steps up");
	loco.SpeedCtrlInc();
	loco.SpeedCtrlInc();
	auto const higher{loco.SpeedCtrlValue};
	CHECK(higher == doctest::Approx(held + 10.0).epsilon(0.1));
	run(60);
	CHECK(loco.Vel == doctest::Approx(higher).epsilon(0.03));
	CHECK(loco.StLinFlag);
	CHECK(shunted > 0);
	CHECK(unshunted > 0);
	CHECK_FALSE(loco.IncScndCtrl(1)); // still not the driver's

	scenario.act("button 30");
	loco.SpeedCtrlButton(0);
	CHECK(loco.SpeedCtrlValue == doctest::Approx(30.0));
	scenario.wait(60.0);
	// it brakes down with the air (BrakeIntervention), which opens the line contactors (the pressure switch), and
	// they close again only through the first power position of the controller, as without the cruise control
	CHECK(loco.Vel < 35.0);
	CHECK_FALSE(loco.StLinFlag);
	scenario.act("controller to the first power position and back up");
	move_to(loco, dg_lower);
	scenario.wait(2.0);
	move_to(loco, dg_raise);
	scenario.wait(60.0);
	CHECK(loco.StLinFlag);
	CHECK(loco.Vel == doctest::Approx(30.0).epsilon(0.05));
	CHECK(loco.ScndCtrlPos == 0); // out at this speed

	scenario.act("cruise control off");
	CHECK(loco.SpeedCtrlSwitch(false));
	CHECK_FALSE(loco.SpeedCtrlSwitch(false)); // off already
	CHECK_FALSE(loco.SpeedCtrlUnit.IsActive);
	CHECK(loco.SpeedCtrlValue == 0.0);
	scenario.wait(5.0);
	CHECK(loco.eimic_real == doctest::Approx(loco.eimic)); // the controller's demand again
	scenario.check();
}

TEST_CASE("speed control apart: the ST45's cruise control on its own switch, with its buttons and the power limit") {
	scenario scenario{"speed control apart/st45_v2/301dd", motion() + traction() + cruise(), test::track{}};
	scenario.add("st45_v2/301dd", 1);
	auto &loco{scenario.cab()};
	REQUIRE(scenario.start_up());
	scenario.get_going();
	scenario.full_power(0.0);
	REQUIRE(loco.Vel < 50.0);
	scenario.act("cruise control on, button 60");
	CHECK(loco.SpeedCtrlSwitch(true));
	CHECK(loco.SpeedCtrlUnit.IsActive);
	loco.SpeedCtrlButton(4);
	CHECK(loco.SpeedCtrlValue == doctest::Approx(60.0));
	scenario.wait(90.0);
	CHECK(loco.Vel == doctest::Approx(60.0).epsilon(0.03));
	CHECK(loco.StLinFlag);
	scenario.act("step up, power down");
	loco.SpeedCtrlInc();
	CHECK(loco.SpeedCtrlValue == doctest::Approx(65.0));
	CHECK(loco.SpeedCtrlPowerDec());
	CHECK(loco.SpeedCtrlUnit.DesiredPower == doctest::Approx(0.9));
	scenario.wait(60.0);
	CHECK(loco.Vel == doctest::Approx(65.0).epsilon(0.03));
	CHECK(loco.eimic_real <= 0.9 + 1e-9);
	scenario.act("cruise control off");
	CHECK(loco.SpeedCtrlSwitch(false));
	// off, its buttons do nothing
	loco.SpeedCtrlButton(6);
	loco.SpeedCtrlInc();
	CHECK_FALSE(loco.SpeedCtrlPowerInc());
	CHECK(loco.SpeedCtrlValue == 0.0);
	scenario.wait(5.0);
	CHECK(loco.ScndCtrlPos == 0);
	scenario.check();
}

TEST_CASE("speed control apart: the switch of a cruise control without one of its own is the secondary controller, as before") {
	// the 214M railbus: on, its first position; off, back to zero
	scenario scenario{"speed control apart/21xm_v1/214ma", motion() + traction() + cruise(), test::track{}};
	scenario.add("21xm_v1/214ma", 1);
	auto &railbus{scenario.cab()};
	REQUIRE(scenario.start_up());
	scenario.get_going();
	scenario.full_power(20.0);
	scenario.act("cruise control on");
	CHECK(railbus.SpeedCtrlSwitch(true));
	CHECK(railbus.ScndCtrlPos == 1);
	CHECK(railbus.SpeedCtrlUnit.IsActive);
	CHECK(railbus.IsSpeedCtrlEngaged());
	scenario.wait(10.0);
	scenario.act("cruise control off");
	CHECK(railbus.SpeedCtrlSwitch(false));
	CHECK(railbus.ScndCtrlPos == 0);
	CHECK_FALSE(railbus.SpeedCtrlUnit.IsActive);
	CHECK_FALSE(railbus.IsSpeedCtrlEngaged());
	scenario.wait(5.0);
	scenario.check();
}
