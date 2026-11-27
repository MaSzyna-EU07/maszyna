#include "stdafx.h"
#include "doctest.h"
#include "drive.h"

#include "McZapkie/hamulce.h"

// the pressure switch of the EP05 (git.eu07.pl eu07/maszyna#48). the pressure switch of the simulator is two
// conditions in one: it cuts the traction motors off above 2 bar in the brake cylinders and below 3.6 bar in the
// brake pipe. the EP05 takes current whatever the pressure in its brake cylinders (Engine: PressureSwitchBrakeCylinder=No
// leaves the brake pipe half alone), and an emergency brake application made with the master controller on a power
// position opens its line breaker (Cntrl. EmergencyBrakeMainOff=<bar in the brake pipe>), which on this locomotive
// drops the pantographs too, as they stay up only through the closed line breaker (Cntrl. PantRequiresMain)

namespace {

constexpr test::track line{0.15, 0.0, test::catenary_3kv};

// the brake pipe pressure taken for an emergency application in these tests [bar]: below the 2.8 bar the FV4a
// gives on its last service position, above what is left in the pipe once it's vented
constexpr double emergency_pressure{2.5};

struct keys {
	bool brakecylinder{true}; // Engine: PressureSwitchBrakeCylinder
	bool mainoff{false}; // Cntrl. EmergencyBrakeMainOff
	bool mainsfirst{true}; // Cntrl. PantRequiresMain, which the game's EP05 has
};

test::definition ep05(keys const &Keys) {
	auto definition{test::definition::fiz("ep05_v2/ep05")};
	if (Keys.mainsfirst) {
		// the game's definition got it after the one of the tests was converted
		definition.set("Cntrl", "PantRequiresMain", "Yes");
	}
	if (false == Keys.brakecylinder) {
		definition.set("Engine", "PressureSwitchBrakeCylinder", "No");
	}
	if (Keys.mainoff) {
		definition.set("Cntrl", "EmergencyBrakeMainOff", emergency_pressure);
	}
	return definition;
}

// the locomotive started up the way the driver does it and its brakes released, until the pressure switch lets go
void get_ready(test::vehicle &Loco, test::driven const &Driven) {
	REQUIRE(test::start_up(Driven, line));
	test::release_brakes(Driven.cab);
	test::drive(Driven, line, 20.0);
	for (int i = 0; i < 120 && Loco->ControlPressureSwitch; ++i) {
		test::drive(Driven, line, 1.0);
	}
	REQUIRE_FALSE(Loco->ControlPressureSwitch);
	REQUIRE(Loco->Mains);
	REQUIRE(Loco->Pantographs[end::rear].is_active);
}

// the master controller taken to the given position a notch a second
void notch_up(test::vehicle &Loco, test::driven const &Driven, int const Position) {
	while (Loco->MainCtrlPos < Position && Loco->IncMainCtrl(1)) {
		test::drive(Driven, line, 1.0);
	}
	test::drive(Driven, line, 2.0);
	REQUIRE(Loco->MainCtrlPos == Position);
}

void get_under_power(test::vehicle &Loco, test::driven const &Driven, int const Position) {
	get_ready(Loco, Driven);
	notch_up(Loco, Driven, Position);
	REQUIRE(Loco->StLinFlag);
	REQUIRE(std::abs(Loco->Im) > 10.0);
}

// the independent brake applied in full: 3.8 bar in the brake cylinders of the locomotive (Brake: MaxBP)
void apply_independent_brake(test::vehicle &Loco, test::driven const &Driven) {
	Loco->IncLocalBrakeLevel(LocalBrakePosNo);
	test::drive(Driven, line, 20.0);
	REQUIRE(Loco->BrakePress == doctest::Approx(3.8).epsilon(0.03));
	REQUIRE(Loco->PipePress > 4.8);
}

void emergency_application(test::vehicle &Loco, test::driven const &Driven) {
	Loco->BrakeLevelSet(Loco->Handle->GetPos(bh_EB));
	test::drive(Driven, line, 10.0);
	REQUIRE(Loco->PipePress < 1.0);
}

} // namespace

TEST_CASE("pressure switch: the keys are off by default") {
	test::vehicle loco{ep05({}), 0.0, "test", 1};
	CHECK(loco->HasControlPressureSwitch);
	CHECK(loco->PressureSwitchBrakeCylinder);
	CHECK(loco->EmergencyBrakeMainOffPressure == 0.0);

	test::vehicle changed{ep05({.brakecylinder = false, .mainoff = true}), 0.0, "test", 1};
	CHECK(changed->HasControlPressureSwitch);
	CHECK_FALSE(changed->PressureSwitchBrakeCylinder);
	CHECK(changed->EmergencyBrakeMainOffPressure == doctest::Approx(emergency_pressure));
}

TEST_CASE("pressure switch: with the brake cylinders at 3.8 bar the EP05 takes no current, as before") {
	test::vehicle loco{ep05({}), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_ready(loco, driven);
	apply_independent_brake(loco, driven);
	CHECK(loco->ControlPressureSwitch);
	CHECK_FALSE(loco->MainCircuitLampsEnabled());

	notch_up(loco, driven, 3);
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->Im == 0.0);
	CHECK(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);
}

TEST_CASE("pressure switch: without its brake cylinder half the EP05 takes current with the brake cylinders at 3.8 bar") {
	test::vehicle loco{ep05({.brakecylinder = false}), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_ready(loco, driven);
	apply_independent_brake(loco, driven);
	CHECK_FALSE(loco->ControlPressureSwitch);
	// the lamps of the main circuit tell the start can be made
	CHECK(loco->MainCircuitLampsEnabled());
	CHECK(loco->LineContactorsLampLit());

	notch_up(loco, driven, 3);
	CHECK(loco->StLinFlag);
	CHECK(loco->MainCtrlActualPos == 3);
	CHECK(std::abs(loco->Im) > 100.0);
	CHECK(loco->BrakePress == doctest::Approx(3.8).epsilon(0.03));
	MESSAGE("current with 3.8 bar in the brake cylinders, position 3: " << std::abs(loco->Im) << " A");
}

TEST_CASE("pressure switch: without its brake cylinder half the brake pipe still cuts the traction motors off") {
	test::vehicle loco{ep05({.brakecylinder = false}), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);

	SUBCASE("a service application") {
		loco->BrakeLevelSet(loco->Handle->GetPos(bh_FB));
		test::drive(driven, line, 15.0);
		REQUIRE(loco->PipePress < 3.6);
		REQUIRE(loco->PipePress > 3.0);
	}
	SUBCASE("an emergency application") {
		emergency_application(loco, driven);
	}
	CHECK(loco->ControlPressureSwitch);
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->Im == 0.0);
	// nothing else lets go without the other key
	CHECK(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);
}

TEST_CASE("pressure switch: an emergency application with the master controller on a power position opens the line breaker and drops the pantographs") {
	test::vehicle loco{ep05({.brakecylinder = false, .mainoff = true}), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);

	SUBCASE("made with the brake handle") {
		loco->BrakeLevelSet(loco->Handle->GetPos(bh_EB));
	}
	SUBCASE("made with the emergency brake of the train") {
		loco->AlarmChainSwitch(true);
	}
	test::drive(driven, line, 10.0);
	REQUIRE(loco->PipePress < emergency_pressure);
	REQUIRE(loco->MainCtrlPos == 5);
	CHECK_FALSE(loco->Mains);
	CHECK_FALSE(loco->Pantographs[end::front].is_active);
	CHECK_FALSE(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == 0.0);
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->Im == 0.0);

	// the line breaker doesn't close again while the master controller is where it was
	loco->MainSwitch(true);
	test::drive(driven, line, 2.0);
	CHECK_FALSE(loco->Mains);
	CHECK_FALSE(loco->Pantographs[end::rear].is_active);
}

TEST_CASE("pressure switch: an emergency application with the master controller at zero leaves the line breaker and the pantographs alone") {
	test::vehicle loco{ep05({.brakecylinder = false, .mainoff = true}), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);
	while (loco->DecMainCtrl(1)) {
		// down to zero
	}
	test::drive(driven, line, 2.0);
	REQUIRE(loco->MainCtrlPos == 0);

	emergency_application(loco, driven);
	CHECK(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	CHECK(loco->Im == 0.0);

	// and the master controller moved with the pipe empty is the same as the application made with it moved
	loco->IncMainCtrl(1);
	test::drive(driven, line, 2.0);
	CHECK_FALSE(loco->Mains);
	CHECK_FALSE(loco->Pantographs[end::rear].is_active);
}

TEST_CASE("pressure switch: a service application with the master controller on a power position doesn't open the line breaker") {
	test::vehicle loco{ep05({.brakecylinder = false, .mainoff = true}), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);

	// the last service position of the FV4a, 2.8 bar in the pipe
	loco->BrakeLevelSet(5.0);
	auto lowest{loco->PipePress};
	for (int second = 0; second < 60; ++second) {
		test::drive(driven, line, 1.0);
		lowest = std::min(lowest, loco->PipePress);
	}
	MESSAGE("the lowest brake pipe pressure of the service application: " << lowest << " bar");
	REQUIRE(lowest < 3.0);
	CHECK(lowest > emergency_pressure);
	CHECK(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);
	// the pressure switch has cut the motors off
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->Im == 0.0);
}

TEST_CASE("pressure switch: without the key an emergency application leaves the line breaker closed and the pantographs up, as before") {
	test::vehicle loco{ep05({}), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);

	emergency_application(loco, driven);
	REQUIRE(loco->MainCtrlPos == 5);
	CHECK(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	// the pressure switch cuts the motors off
	CHECK(loco->ControlPressureSwitch);
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->Im == 0.0);
}

TEST_CASE("pressure switch: where the pantographs don't hang on the line breaker the emergency application opens only the line breaker") {
	test::vehicle loco{ep05({.brakecylinder = false, .mainoff = true, .mainsfirst = false}), 0.0, "test", 1};
	REQUIRE_FALSE(loco->PantographsRequireMains);
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);

	emergency_application(loco, driven);
	CHECK_FALSE(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->Im == 0.0);
}

TEST_CASE("pressure switch: after the emergency application the driver's start-up gets the locomotive going again") {
	// test::start_up() takes the AI's steps of TController::PrepareEngine(): the master controller to zero, the relays
	// reset, the line breaker closed. the pantograph switches stay as they were, so the pantograph rises with the line
	// breaker
	test::vehicle loco{ep05({.brakecylinder = false, .mainoff = true}), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);
	emergency_application(loco, driven);
	REQUIRE_FALSE(loco->Mains);
	REQUIRE_FALSE(loco->Pantographs[end::rear].is_active);
	REQUIRE(loco->Pantographs[end::rear].valve.is_enabled);

	// with the brake pipe still empty
	REQUIRE(test::start_up(driven, line, 60.0));
	CHECK(loco->MainCtrlPos == 0);
	CHECK(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	REQUIRE(loco->PipePress < emergency_pressure);
	// and it stays so
	test::drive(driven, line, 30.0);
	CHECK(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);

	// the brakes released (the AI fills an emptied pipe with the releaser held, TController::control_releaser()),
	// the pressure switch lets the start be made
	test::release_brakes(driven.cab);
	for (int i = 0; i < 180 && loco->ControlPressureSwitch; ++i) {
		loco->BrakeReleaser(loco->PipePress < 3.0 ? 1 : 0);
		test::drive(driven, line, 1.0);
	}
	loco->BrakeReleaser(0);
	REQUIRE_FALSE(loco->ControlPressureSwitch);
	notch_up(loco, driven, 3);
	CHECK(loco->Mains);
	CHECK(loco->StLinFlag);
	CHECK(std::abs(loco->Im) > 10.0);
}
