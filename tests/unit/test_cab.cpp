#include "stdafx.h"
#include "doctest.h"
#include "consist.h"
#include "scenario.h"
#include "drive.h"

#include "McZapkie/hamulce.h"
#include "utilities/utilities.h"

// characterisation of what the cab (Train.cpp) and the AI (Driver.cpp, driverhints.cpp) call on a vehicle
// besides the controls of test_controls.cpp: the gauges and lamps reading other vehicles of the train, the
// controller of the electrodynamic brake apart from the local brake, the spring brake, the ep brake's fuse,
// cutting off damaged motors, the cab activated by itself, a road vehicle's place across the road, and the
// lamps and the buzzer of the security system

using test::consist;
using test::definition;
using test::scenario;
using test::vehicle;
using namespace test::channels;

TEST_CASE("cab: the engine rotation of up to three vehicles in multiple, on the tachometers of the SN61") {
	consist train;
	auto &first{train.add(definition::fiz("sn61_v2/sn61"), 1).mover()};
	auto &second{train.add(definition::fiz("sn61_v2/sn61")).mover()};
	auto &third{train.add(definition::fiz("sn61_v2/sn61")).mover()};
	first.enrot = -10.0;
	second.enrot = 20.0;
	third.enrot = 30.0;
	CHECK(first.ShowEngineRotation(1) == doctest::Approx(10.0));
	CHECK(first.ShowEngineRotation(2) == doctest::Approx(20.0));
	CHECK(first.ShowEngineRotation(3) == doctest::Approx(30.0));
	CHECK(first.ShowEngineRotation(4) == doctest::Approx(0.0));
	// the others look the same way along their couplers, front first
	CHECK(second.ShowEngineRotation(2) == doctest::Approx(10.0));
	CHECK(third.ShowEngineRotation(2) == doctest::Approx(20.0));
	CHECK(third.ShowEngineRotation(3) == doctest::Approx(10.0));
	// no control cable, no reading
	consist apart;
	auto const &alone{apart.add(definition::fiz("sn61_v2/sn61"), 1).mover()};
	apart.add(definition::fiz("sn61_v2/sn61"), 0, coupling::coupler | coupling::brakehose);
	CHECK(alone.ShowEngineRotation(2) == doctest::Approx(0.0));
}

TEST_CASE("cab: the lamps of the overload relay and the starting resistors, read through the control cable") {
	consist unit;
	auto &trailer{unit.add(definition::fiz("ed72_v1/5bs-ra"), 1).mover()};
	auto &motor{unit.add(definition::fiz("ed72_v1/6bs-sa")).mover()};
	REQUIRE(trailer.Power < 0.01);
	REQUIRE(motor.Power > 0.01);
	CHECK_FALSE(trailer.FuseFlagCheck());
	CHECK_FALSE(trailer.ResistorsFlagCheck());
	motor.FuseFlag = true;
	motor.ResistorsFlag = true;
	CHECK(trailer.FuseFlagCheck());
	CHECK(trailer.ResistorsFlagCheck());
	CHECK(motor.FuseFlagCheck());
	CHECK(motor.ResistorsFlagCheck());
}

TEST_CASE("cab: the engine's idle speed, and the ET42's second controller shown in its dynamic brake") {
	vehicle sn61{definition::fiz("sn61_v2/sn61")};
	CHECK(sn61->EngineIdleRPM() == doctest::Approx(sn61->dizel_nmin * 60.0));
	vehicle su45{definition::fiz("su45_v2/301d")};
	CHECK(su45->EngineIdleRPM() == doctest::Approx(su45->DElist[su45->MainCtrlNoPowerPos()].RPM));
	vehicle eu07{definition::fiz("4e_v1/4e")};
	CHECK(eu07->EngineIdleRPM() == std::numeric_limits<double>::max());

	vehicle et42{definition::fiz("et42_v2/112e-a")};
	CHECK(et42->GetVirtualScndPos() == 0);
	et42->DynamicBrakeFlag = true;
	CHECK(et42->GetVirtualScndPos() == -1);
	et42->ScndCtrlPos = 1;
	CHECK(et42->GetVirtualScndPos() == 1);
	// with a speed control in place of the second controller, either end of it, for the AI
	vehicle railbus{definition::fiz("21xm_v1/214ma")};
	REQUIRE(railbus->SpeedCtrl);
	railbus->ScndCtrlPos = 1;
	CHECK(railbus->IsScndCtrlNoPowerPos());
	CHECK(railbus->IsScndCtrlMaxPowerPos());
	CHECK(eu07->IsScndCtrlNoPowerPos());
	CHECK_FALSE(eu07->IsScndCtrlMaxPowerPos());
}

TEST_CASE("cab: the controller of the electrodynamic brake apart from the local brake") {
	// none of the game's vehicles has one yet (SplitEDPneumaticBrake): the electrodynamic brake follows its own
	// controller of DBPN positions, the local brake is pneumatic alone
	scenario scenario{"cab/split electrodynamic brake", motion() + traction() + brakes()};
	scenario.add("e186_v2/eu47", 1, consist::full_coupling, 100.0).add("bdhpumn_v3/bdhpumn", 0, consist::full_coupling, 100.0);
	auto &eu47{scenario.cab()};
	CHECK_FALSE(eu47.IncDynamicBrakeLevel(1.f));
	CHECK(eu47.DynamicBrakeRatio() == doctest::Approx(0.0));

	scenario.train()[0].mover().SplitEDPneumaticBrake = true;
	REQUIRE(scenario.start_up());
	REQUIRE(eu47.DynamicBrakeCtrlPosNo == 10);
	CHECK(eu47.DynamicBrakeAvailable());
	scenario.act("dynamic brake up");
	for (int i = 0; i < 5; ++i) {
		CHECK(eu47.IncDynamicBrakeLevel(1.f));
	}
	CHECK(eu47.DynamicBrakeRatio() == doctest::Approx(0.5));
	scenario.wait(10.0);
	CHECK(eu47.DynamicBrakeFlag);
	CHECK(eu47.Im < -50.0);
	CHECK(eu47.BrakePress < 0.1);
	scenario.act("dynamic brake full");
	CHECK(eu47.DynamicBrakeLevelSet(2.0));
	CHECK(eu47.DynamicBrakeRatio() == doctest::Approx(1.0));
	CHECK_FALSE(eu47.IncDynamicBrakeLevel(1.f));
	scenario.wait(10.0);
	scenario.act("dynamic brake off");
	for (int i = 0; i < 20 && eu47.DecDynamicBrakeLevel(1.f); ++i) {
		// down a notch at a time
	}
	CHECK(eu47.DynamicBrakeRatio() == doctest::Approx(0.0));
	// the local brake no longer brings the motors in
	scenario.act("local brake");
	eu47.IncLocalBrakeLevel(LocalBrakePosNo);
	scenario.wait(5.0);
	CHECK(eu47.BrakePress > 1.0);
	scenario.check();
}

TEST_CASE("cab: the spring brake shut off and released by hand, the ep brake's fuse, the train brake handle stepped") {
	scenario scenario{"cab/214m spring brake", motion() + brakes()};
	scenario.add("21xm_v1/214ma", 1);
	REQUIRE(scenario.start_up());
	auto &railbus{scenario.cab()};
	scenario.act("spring brake on");
	railbus.SpringBrakeActivate(true);
	scenario.wait(10.0);
	CHECK(railbus.SpringBrake.IsActive);
	// released by hand only with its cylinder empty of the air holding it off
	auto const pressure{railbus.SpringBrake.Cylinder->P()};
	CHECK(railbus.SpringBrakeRelease() == (pressure < railbus.SpringBrake.MinForcePressure));
	scenario.act("shut off");
	CHECK(railbus.SpringBrakeShutOff(true));
	CHECK_FALSE(railbus.SpringBrakeShutOff(true));
	scenario.wait(5.0);
	CHECK(railbus.SpringBrakeShutOff(false));
	scenario.act("spring brake off");
	railbus.SpringBrakeActivate(false);
	scenario.wait(10.0);
	// the ep brake's fuse
	CHECK(railbus.EpFuseSwitch(false));
	CHECK_FALSE(railbus.EpFuseSwitch(false));
	CHECK(railbus.EpFuseSwitch(true));
	// the train brake handle stepped as the AI does, false at its ends
	scenario.act("handle stepped");
	railbus.BrakeLevelSet(railbus.Handle->GetPos(bh_RP));
	CHECK(railbus.BrakeLevelAdd(1.0));
	CHECK(railbus.BrakeCtrlPos == static_cast<int>(railbus.Handle->GetPos(bh_RP)) + 1);
	CHECK(railbus.BrakeLevelAdd(-1.0));
	CHECK_FALSE(railbus.BrakeLevelAdd(-10.0));
	CHECK_FALSE(railbus.BrakeLevelAdd(100.0));
	scenario.wait(5.0);
	scenario.check();
}

TEST_CASE("cab: damaged motors cut off in a car without a driver") {
	// cutting off takes the damage off the engine and half of the powered axles with it; only in a car nobody
	// drives, with its engine damaged
	vehicle driven{definition::fiz("ew58_v1/ew58-sa"), 0.0, "test", 1};
	driven->BatterySwitch(true);
	REQUIRE(driven->CabActivisation());
	driven->DamageFlag |= dtrain_engine;
	CHECK_FALSE(driven->CutOffEngine());
	vehicle car{definition::fiz("ew58_v1/ew58-sa")};
	auto const axles{car->NPoweredAxles};
	CHECK_FALSE(car->CutOffEngine());
	car->DamageFlag |= dtrain_engine;
	CHECK(car->CutOffEngine());
	CHECK(car->NPoweredAxles == axles / 2);
	CHECK_FALSE(TestFlag(car->DamageFlag, dtrain_engine));
	vehicle wagon{definition::fiz("3wl_v2/3wl")};
	wagon->DamageFlag |= dtrain_engine;
	CHECK_FALSE(wagon->CutOffEngine());
}

TEST_CASE("cab: activated by itself, where the vehicle allows it") {
	for (auto const allowed : {false, true}) {
		CAPTURE(allowed);
		vehicle eu07{definition::fiz("4e_v1/4e").set("Cntrl", "AutomaticCabActivation", allowed ? "Yes" : "No"), 0.0, "test", 1};
		eu07->BatterySwitch(true);
		CHECK(eu07->CabActivisationAuto() == allowed);
		CHECK((eu07->CabActive != 0) == allowed);
		CHECK(eu07->CabDeactivisationAuto() == allowed);
		CHECK(eu07->CabActive == 0);
	}
}

TEST_CASE("cab: a road vehicle moved across its road") {
	vehicle bus{definition::fiz("road/man/nl293")};
	bus.step(test::track{0.15, 0.0, 0.0, 2, 1, 7.0});
	REQUIRE(TestFlag(bus->CategoryFlag, 2));
	CHECK(bus->ChangeOffsetH(0.5));
	CHECK(bus->OffsetTrackH == doctest::Approx(0.5));
	// off the edge of the road
	CHECK_FALSE(bus->ChangeOffsetH(100.0));
	vehicle eu07{definition::fiz("4e_v1/4e")};
	CHECK_FALSE(eu07->ChangeOffsetH(0.5));
}

TEST_CASE("cab: the lamps and the buzzer of the security system left unattended") {
	scenario scenario{"cab/security system", motion() + brakes()};
	scenario.add("4e_v1/4e", 1);
	REQUIRE(scenario.start_up());
	auto &eu07{scenario.cab()};
	scenario.get_going();
	scenario.full_power(10.0);
	auto const &security{eu07.SecuritySystem};
	CHECK_FALSE(security.is_beeping());
	scenario.act("unattended");
	// the train runs with nobody acknowledging the vigilance
	bool blinked{false};
	bool beeped{false};
	for (int i = 0; i < 120; ++i) {
		scenario.train().run(1.0, scenario.line());
		blinked |= security.is_vigilance_blinking();
		beeped |= security.is_beeping();
	}
	CHECK(blinked);
	CHECK(beeped);
	CHECK_FALSE(security.is_cabsignal_beeping());
	CHECK(eu07.PipePress < 1.0);
	CHECK(security.radiostop_available());
	scenario.check();
}

namespace {

// the SM42, its shunt mode switching its vigilance device (CA) off or not (AwareOffInShuntMode), set in the
// test whatever the converted definition says
definition sm42_vigilance(bool const Off) {
	return definition::fiz("sm42_v1/6d1").set("Security", "AwareOffInShuntMode", Off ? "Yes" : "No");
}

// runs the train for Seconds with nobody acknowledging the vigilance; returns whether the vigilance alarm came
bool unattended(scenario &Scenario, int const Seconds) {
	bool blinked{false};
	for (int i = 0; i < Seconds; ++i) {
		Scenario.train().run(1.0, Scenario.line());
		blinked |= Scenario.cab().SecuritySystem.is_vigilance_blinking();
	}
	return blinked;
}

} // namespace

TEST_CASE("cab: the SM42's vigilance device switched off in the shunt mode, where its definition says so") {
	// eu07/maszyna#457: with AwareOffInShuntMode=Yes the shunt mode of the SM42 switches its vigilance device (CA)
	// off, it alerts again once the shunt mode is off
	scenario scenario{"cab/sm42 vigilance in shunt mode", motion() + brakes(), test::track{}};
	scenario.train().add(sm42_vigilance(true), 1);
	REQUIRE(scenario.start_up());
	auto &sm42{scenario.cab()};
	auto const &security{sm42.SecuritySystem};
	REQUIRE(sm42.CurrentSwitch(true));
	REQUIRE(sm42.ShuntMode);
	scenario.get_going();
	sm42.AnPos = 0.3;
	sm42.IncMainCtrl(1);
	scenario.act("unattended in shunt mode");
	bool blinked{false};
	for (int i = 0; i < 120; ++i) {
		scenario.train().run(1.0, scenario.line());
		blinked |= security.is_blinking();
	}
	CHECK(sm42.Vel > 1.0);
	CHECK_FALSE(blinked);
	CHECK(sm42.PipePress > 3.5);
	scenario.act("shunt mode off");
	scenario.notch_back(0);
	REQUIRE(sm42.CurrentSwitch(false));
	sm42.IncMainCtrl(1);
	scenario.act("unattended");
	CHECK(unattended(scenario, 120));
	scenario.check();
}

TEST_CASE("cab: the vigilance device of the SM42 kept in the shunt mode, by default") {
	// without AwareOffInShuntMode the shunt mode leaves the vigilance device alone: unattended, it alerts and brakes
	scenario scenario{"cab/sm42 vigilance kept in shunt mode", motion() + brakes(), test::track{}};
	scenario.train().add(sm42_vigilance(false), 1);
	REQUIRE(scenario.start_up());
	auto &sm42{scenario.cab()};
	REQUIRE(sm42.CurrentSwitch(true));
	REQUIRE(sm42.ShuntMode);
	scenario.get_going();
	sm42.AnPos = 0.3;
	sm42.IncMainCtrl(1);
	scenario.act("unattended in shunt mode");
	CHECK(unattended(scenario, 120));
	CHECK(sm42.ShuntMode);
	CHECK(sm42.PipePress < 1.0);
	scenario.check();
}

TEST_CASE("cab: the SM42's vigilance alarm on, not cleared by switching into the shunt mode") {
	// the shunt mode switches the vigilance device off for the time to come: an alarm already on goes on to the
	// emergency braking, until the driver acknowledges it (#275 review)
	scenario scenario{"cab/sm42 vigilance alarm into shunt mode", motion() + brakes(), test::track{}};
	scenario.train().add(sm42_vigilance(true), 1);
	REQUIRE(scenario.start_up());
	auto &sm42{scenario.cab()};
	auto const &security{sm42.SecuritySystem};
	scenario.get_going();
	sm42.IncMainCtrl(1);
	scenario.act("unattended");
	// the alerter counts only on the move, which takes a while on the first notch
	for (int i = 0; i < 300 && false == security.is_vigilance_blinking(); ++i) {
		scenario.train().run(1.0, scenario.line());
	}
	REQUIRE(security.is_vigilance_blinking());
	REQUIRE_FALSE(security.is_braking());
	scenario.act("shunt mode on");
	for (int i = 0; i < 20 && false == sm42.IsMainCtrlNoPowerPos(); ++i) {
		sm42.DecMainCtrl(1);
	}
	REQUIRE(sm42.CurrentSwitch(true));
	REQUIRE(sm42.ShuntMode);
	scenario.train().run(15.0, scenario.line());
	CHECK(security.is_vigilance_blinking());
	CHECK(security.is_beeping());
	CHECK(security.is_braking());
	CHECK(sm42.PipePress < 1.0);
	scenario.act("acknowledged");
	sm42.SecuritySystemReset();
	scenario.train().run(1.0, scenario.line());
	CHECK_FALSE(security.is_blinking());
	CHECK_FALSE(security.is_braking());
	scenario.check();
}

namespace {

TSecuritySystem sm42_security(bool const OffInShuntMode) {
	TSecuritySystem security;
	security.load(std::string{"Security: AwareSystem=Active AwareMinSpeed=0.0 AwareDelay=60 SoundSignalDelay=2.5 MaxHoldTime=1.0 "
	                          "EmergencyBrakeDelay=2.5 AwareOffInShuntMode="} +
	                  (OffInShuntMode ? "Yes" : "No"),
	              90.0);
	security.set_enabled(true);
	security.update(0.1, 20.0, true, 1);
	return security;
}

void run(TSecuritySystem &Security, double const Seconds) {
	for (int i = 0; i < std::lround(Seconds / 0.1); ++i) {
		Security.update(0.1, 20.0, true, 1);
	}
}

} // namespace

TEST_CASE("cab: the alerter in the shunt mode, with and without AwareOffInShuntMode") {
	for (auto const off : {false, true}) {
		CAPTURE(off);
		auto security{sm42_security(off)};
		security.set_shunt_mode(true);
		run(security, 120.0);
		// without the key the shunt mode changes nothing: the alerter alerts and brakes
		CHECK(security.is_vigilance_blinking() != off);
		CHECK(security.is_braking() != off);
	}
}

TEST_CASE("cab: the alerter's alarm on, not cleared by the shunt mode") {
	auto security{sm42_security(true)};
	run(security, 61.0);
	REQUIRE(security.is_vigilance_blinking());
	REQUIRE_FALSE(security.is_braking());
	security.set_shunt_mode(true);
	run(security, 10.0);
	CHECK(security.is_vigilance_blinking());
	CHECK(security.is_beeping());
	CHECK(security.is_braking());
	security.acknowledge_press();
	security.acknowledge_release();
	run(security, 0.1);
	CHECK_FALSE(security.is_blinking());
	CHECK_FALSE(security.is_braking());
	// then nothing more on the move in the shunt mode
	run(security, 300.0);
	CHECK_FALSE(security.is_blinking());
	// and back in the line mode the alerter counts again
	security.set_shunt_mode(false);
	run(security, 61.0);
	CHECK(security.is_vigilance_blinking());
}

TEST_CASE("cab: the vigilance button held too long, its alarm not cleared by the shunt mode") {
	// the alarm of the button held too long stays on in the shunt mode too, until the button is let go
	TSecuritySystem security;
	security.load("Security: AwareSystem=Active AwareMinSpeed=0.0 AwareDelay=60 SoundSignalDelay=2.5 MaxHoldTime=1.0 "
	              "EmergencyBrakeDelay=2.5 AwareOffInShuntMode=Yes",
	              90.0);
	security.set_enabled(true);
	security.update(0.1, 20.0, true, 1);
	security.acknowledge_press();
	for (int i = 0; i < 20; ++i) {
		security.update(0.1, 20.0, true, 1);
	}
	REQUIRE(security.is_vigilance_blinking());
	security.set_shunt_mode(true);
	for (int i = 0; i < 60; ++i) {
		security.update(0.1, 20.0, true, 1);
	}
	CHECK(security.is_vigilance_blinking());
	CHECK(security.is_braking());
	security.acknowledge_release();
	security.update(0.1, 20.0, true, 1);
	CHECK_FALSE(security.is_blinking());
	CHECK_FALSE(security.is_braking());
	// held again in the shunt mode, no new alarm
	security.acknowledge_press();
	for (int i = 0; i < 100; ++i) {
		security.update(0.1, 20.0, true, 1);
	}
	CHECK_FALSE(security.is_blinking());
	security.acknowledge_release();
	// out of the shunt mode, held too long again
	security.set_shunt_mode(false);
	security.acknowledge_press();
	for (int i = 0; i < 20; ++i) {
		security.update(0.1, 20.0, true, 1);
	}
	CHECK(security.is_vigilance_blinking());
}
