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
	auto &alone{apart.add(definition::fiz("sn61_v2/sn61"), 1).mover()};
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
	auto &security{eu07.SecuritySystem};
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
