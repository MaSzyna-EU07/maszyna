#include "stdafx.h"
#include "doctest.h"
#include "scenario.h"

#include "McZapkie/hamulce.h"

// characterisation of the controls a driver and a crew use besides driving and braking: doors, direction and
// cab change, sanding against slip, the emergency brake, releasers, the ep and the dynamic brake, heating,
// loading, collisions and the frame of several physics iterations, each in a scenario on real vehicles,
// recorded in golden traces (data/golden/controls/)

using test::scenario;
using namespace test::channels;

namespace {

void notch_up(scenario &Scenario, int Positions, double Seconds) {
	for (int i = 0; i < Positions; ++i) {
		Scenario.driver().controlling.IncMainCtrl(1);
		Scenario.wait(Seconds);
	}
}

void notch_down(scenario &Scenario) {
	while (Scenario.driver().controlling.DecMainCtrl(1)) {
		Scenario.wait(0.5);
	}
	Scenario.driver().controlling.DecScndCtrl(2);
}

void brake_to_stop(scenario &Scenario, double Seconds = 60.0) {
	Scenario.act("brake to stop");
	Scenario.cab().BrakeLevelSet(Scenario.cab().Handle->GetPos(bh_FB));
	Scenario.wait(Seconds);
	Scenario.cab().BrakeLevelSet(Scenario.cab().Handle->GetPos(bh_RP));
	Scenario.wait(20.0);
}

} // namespace

TEST_CASE("controls: the doors of an electric unit, opened, locked, permitted and closed by driving off") {
	for (auto const &[name, cars] : std::vector<std::pair<std::string, std::vector<char const *>>>{
	         {"elf", {"elf_v1/21we-a", "elf_v1/21we-b", "elf_v1/21we-c"}},
	         {"en57", {"en57_v1/5bgaiii_led_spot", "en57_v1/6ba_840", "en57_v1/5bgb"}},
	         {"impuls", {"impuls_v1/31we-a_pr", "impuls_v1/31we-b_pr", "impuls_v1/31we-c_pr", "impuls_v1/31we-d_pr"}}}) {
		CAPTURE(name);
		scenario scenario{"controls/doors/" + name, motion() + doors()};
		for (auto const *car : cars) {
			scenario.add(car, scenario.train().size() == 0 ? 1 : 0);
		}
		scenario.start_up();
		auto &cab{scenario.cab()};
		scenario.act("permit left");
		cab.PermitDoors(side::left, true);
		scenario.wait(2.0);
		scenario.act("open left");
		cab.OperateDoors(side::left, true);
		scenario.wait(10.0);
		scenario.act("close left");
		cab.OperateDoors(side::left, false);
		scenario.wait(10.0);
		scenario.act("open right without permit");
		cab.OperateDoors(side::right, true);
		scenario.wait(5.0);
		scenario.act("permit right, open");
		cab.PermitDoors(side::right, true);
		cab.OperateDoors(side::right, true);
		scenario.wait(10.0);
		scenario.act("lock");
		cab.LockDoors(true);
		scenario.wait(5.0);
		scenario.act("permit preset, steps off, remote control");
		cab.ChangeDoorPermitPreset(1);
		cab.PermitDoorStep(false);
		cab.ChangeDoorControlMode(true);
		scenario.wait(5.0);
		cab.ChangeDoorPermitPreset(-1);
		cab.PermitDoorStep(true);
		cab.ChangeDoorControlMode(false);
		scenario.act("close all, unlock");
		cab.OperateDoors(side::right, false);
		cab.OperateDoors(side::left, false);
		cab.LockDoors(false);
		scenario.wait(10.0);
		scenario.act("open both and drive off");
		cab.PermitDoors(side::left, true);
		cab.PermitDoors(side::right, true);
		cab.OperateDoors(side::left, true);
		cab.OperateDoors(side::right, true);
		scenario.wait(5.0);
		scenario.get_going();
		notch_up(scenario, 4, 2.0);
		scenario.wait(20.0);
		notch_down(scenario);
		brake_to_stop(scenario);
		scenario.check();
	}
}

TEST_CASE("controls: the doors of wagons, worked from the locomotive") {
	scenario scenario{"controls/doors/eu07 with wagons", motion() + doors()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("11xa_v2/93a");
	scenario.start_up();
	auto &cab{scenario.cab()};
	for (auto const range : {range_t::consist, range_t::unit, range_t::local}) {
		scenario.act("open right " + std::to_string(static_cast<int>(range)));
		cab.PermitDoors(side::right, true, range);
		cab.OperateDoors(side::right, true, range);
		scenario.wait(10.0);
		scenario.act("close right");
		cab.OperateDoors(side::right, false, range);
		scenario.wait(10.0);
	}
	scenario.act("wagon opens its own");
	scenario.vehicle(1).OperateDoors(side::left, true, range_t::local);
	scenario.wait(10.0);
	scenario.vehicle(1).OperateDoors(side::left, false, range_t::local);
	scenario.wait(10.0);
	scenario.check();
}

TEST_CASE("controls: reversing and changing the cab of a locomotive") {
	scenario scenario{"controls/reverse/eu07", motion() + traction()};
	scenario.add("4e_v1/4e", 1);
	scenario.start_up();
	scenario.get_going();
	notch_up(scenario, 10, 1.5);
	scenario.wait(10.0);
	notch_down(scenario);
	brake_to_stop(scenario);
	auto &cab{scenario.cab()};
	scenario.act("backward");
	cab.DirectionBackward();
	cab.DirectionBackward();
	scenario.wait(2.0);
	notch_up(scenario, 10, 1.5);
	scenario.wait(10.0);
	notch_down(scenario);
	brake_to_stop(scenario);
	scenario.act("neutral, change cab");
	cab.DirectionForward();
	scenario.wait(1.0);
	cab.CabDeactivisation();
	cab.ChangeCab(-1);
	cab.CabActivisation();
	scenario.wait(5.0);
	scenario.act("drive from the rear cab");
	scenario.get_going();
	notch_up(scenario, 10, 1.5);
	scenario.wait(10.0);
	notch_down(scenario);
	brake_to_stop(scenario);
	scenario.check();
}

TEST_CASE("controls: wheel slip on a slippery track, sand and the anti-slip brake") {
	for (auto const *path : {"4e_v1/4e", "sm42_v1/6d1", "e186_v2/eu47"}) {
		CAPTURE(path);
		scenario scenario{std::string{"controls/slip/"} + path, motion() + traction() + brakes(), test::track{0.04, 0.0, test::catenary_3kv}};
		scenario.add(path, 1);
		scenario.start_up();
		scenario.get_going();
		scenario.act("full power at once");
		while (scenario.driver().controlling.IncMainCtrl(1)) {
		}
		scenario.wait(10.0);
		scenario.act("sand");
		scenario.cab().Sandbox(true);
		scenario.wait(10.0);
		scenario.cab().Sandbox(false);
		scenario.act("manual sand");
		scenario.cab().SandboxManual(true);
		scenario.wait(5.0);
		scenario.cab().SandboxManual(false);
		scenario.act("anti-slip brake");
		scenario.cab().AntiSlippingButton();
		scenario.cab().AntiSlippingBrake();
		scenario.wait(10.0);
		notch_down(scenario);
		brake_to_stop(scenario);
		scenario.check();
	}
}

TEST_CASE("controls: emergency stops by the alarm chain and the radio-stop") {
	scenario scenario{"controls/emergency/eu07 with wagons", motion() + brakes()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("3wl_v2/3wl");
	scenario.start_up();
	scenario.get_going();
	notch_up(scenario, 20, 1.0);
	scenario.wait(20.0);
	notch_down(scenario);
	scenario.act("alarm chain in the wagon");
	scenario.vehicle(1).AlarmChainSwitch(true);
	scenario.wait(40.0);
	scenario.act("alarm chain reset");
	scenario.vehicle(1).AlarmChainSwitch(false);
	scenario.wait(60.0);
	scenario.get_going();
	notch_up(scenario, 20, 1.0);
	scenario.wait(20.0);
	notch_down(scenario);
	scenario.act("radio-stop");
	scenario.cab().RadiostopSwitch(true);
	scenario.wait(40.0);
	scenario.act("radio-stop reset");
	scenario.cab().RadiostopSwitch(false);
	scenario.wait(60.0);
	scenario.check();
}

TEST_CASE("controls: releasers and the universal brake buttons") {
	scenario scenario{"controls/releaser/eu07 with wagons", brakes() + air()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("3wl_v2/3wl");
	scenario.start_up();
	scenario.get_going();
	auto &cab{scenario.cab()};
	scenario.act("service brake");
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_FB));
	scenario.wait(20.0);
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_NP));
	scenario.act("releaser of the locomotive");
	cab.BrakeReleaser(1);
	scenario.wait(10.0);
	cab.BrakeReleaser(0);
	scenario.act("releaser of the wagon");
	scenario.vehicle(2).BrakeReleaser(1);
	scenario.wait(10.0);
	scenario.vehicle(2).BrakeReleaser(0);
	for (int button = 0; button < 3; ++button) {
		scenario.act("universal button " + std::to_string(button));
		cab.UniversalBrakeButton(button, 1);
		scenario.wait(5.0);
		cab.UniversalBrakeButton(button, 0);
		scenario.wait(5.0);
	}
	scenario.act("running");
	cab.BrakeLevelSet(cab.Handle->GetPos(bh_RP));
	scenario.wait(60.0);
	scenario.check();
}

TEST_CASE("controls: the ep brake of electric units") {
	for (auto const &[name, cars] : std::vector<std::pair<std::string, std::vector<char const *>>>{
	         {"ed72", {"ed72_v1/5bs-ra", "ed72_v1/6bs-sa", "ed72_v1/6bs-sb", "ed72_v1/5bs-rb"}},
	         {"ew58", {"ew58_v1/ew58-sa", "ew58_v1/ew58-d", "ew58_v1/ew58-sb"}}}) {
		CAPTURE(name);
		scenario scenario{"controls/ep brake/" + name, motion() + brakes() + handles()};
		for (auto const *car : cars) {
			scenario.add(car, scenario.train().size() == 0 ? 1 : 0);
		}
		scenario.start_up();
		scenario.get_going();
		auto &cab{scenario.cab()};
		scenario.act("ep on");
		cab.SwitchEPBrake(1);
		scenario.wait(5.0);
		for (auto const position : {bh_EPB, bh_EPN, bh_EPR, bh_EPB, bh_RP}) {
			scenario.act("handle " + std::to_string(position));
			cab.BrakeLevelSet(cab.Handle->GetPos(position));
			scenario.wait(15.0);
		}
		scenario.act("ep off");
		cab.SwitchEPBrake(0);
		scenario.wait(30.0);
		scenario.check();
	}
}

TEST_CASE("controls: the electrodynamic brake of locomotives having one") {
	for (auto const *path : {"ep09_v2/104e", "e186_v2/eu47", "e6act_v2/e6acta", "et22_v3/201e-rn"}) {
		CAPTURE(path);
		scenario scenario{std::string{"controls/dynamic brake/"} + path, motion() + traction() + brakes()};
		scenario.add(path, 1).add("bdhpumn_v3/bdhpumn").add("bdhpumn_v3/bdhpumn");
		scenario.start_up();
		scenario.get_going();
		notch_up(scenario, 60, 1.0);
		scenario.wait(30.0);
		notch_down(scenario);
		auto &cab{scenario.cab()};
		scenario.act("dynamic brake on");
		cab.DynamicBrakeSwitch(true);
		scenario.wait(5.0);
		scenario.act("brake");
		cab.BrakeLevelSet(cab.Handle->GetPos(bh_FB));
		scenario.wait(40.0);
		scenario.act("dynamic brake off");
		cab.DynamicBrakeSwitch(false);
		brake_to_stop(scenario, 30.0);
		scenario.check();
	}
}

TEST_CASE("controls: train heating from the locomotive") {
	scenario scenario{"controls/heating/eu07 with wagons", traction() + air()};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("bdhpumn_v3/bdhpumn");
	scenario.start_up();
	scenario.act("heating on");
	scenario.cab().HeatingSwitch(true);
	scenario.wait(30.0);
	scenario.act("heating off");
	scenario.cab().HeatingSwitch(false);
	scenario.wait(10.0);
	scenario.check();
}

TEST_CASE("controls: a freight wagon loaded and braked") {
	for (auto const *load : {"", "coal", "ruda", "unknown"}) {
		CAPTURE(load);
		scenario scenario{std::string{"controls/load/3wl "} + (*load ? load : "empty"), motion() + brakes()};
		scenario.add("4e_v1/4e", 1).add("3wl_v2/3wl");
		CHECK(scenario.vehicle(1).AssignLoad(load, 30.f) == (std::string{load} == "coal" || std::string{load} == "ruda" || std::string{load}.empty()));
		scenario.vehicle(1).ComputeMass();
		scenario.start_up();
		scenario.get_going();
		notch_up(scenario, 20, 1.0);
		scenario.wait(20.0);
		notch_down(scenario);
		brake_to_stop(scenario);
		scenario.check();
	}
}

TEST_CASE("controls: a locomotive running into a standing wagon") {
	for (auto const speed : {5.0, 15.0, 40.0}) {
		CAPTURE(speed);
		scenario scenario{"controls/collision/" + std::to_string(static_cast<int>(speed)) + " kmh", motion()};
		// the wagon stands in front, the locomotive comes from behind it
		scenario.add("bdhpumn_v3/bdhpumn").add("4e_v1/4e", 0, coupling::faux, speed, 30.0);
		scenario.wait(30.0);
		scenario.check();
	}
}

TEST_CASE("controls: a train run with five physics steps in a frame") {
	scenario scenario{"controls/full physics/eu07 with wagons", motion() + traction() + brakes(),
	            test::track{0.15, 0.0, test::catenary_3kv, 0, 5}};
	scenario.add("4e_v1/4e", 1).add("bdhpumn_v3/bdhpumn").add("3wl_v2/3wl");
	scenario.start_up();
	scenario.get_going();
	notch_up(scenario, 25, 1.0);
	scenario.wait(30.0);
	notch_down(scenario);
	brake_to_stop(scenario);
	scenario.check();
}
