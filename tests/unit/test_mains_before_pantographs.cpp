#include "stdafx.h"
#include "doctest.h"
#include "drive.h"

// the order of the line breaker and the pantographs (eu07/maszyna#68): on Polish locomotives the pantographs
// come first and the line breaker closes on their voltage; on Czech ones (Cntrl.PantRequiresMain) the line
// breaker closes first, without voltage, and the pantographs rise and stay up only while it's closed

namespace {

constexpr test::track line{0.15, 0.0, test::catenary_3kv};

test::definition ep05(bool const Mainsfirst) {
	auto definition{test::definition::fiz("ep05_v2/ep05")};
	if (Mainsfirst) {
		definition.set("Cntrl", "PantRequiresMain", "Yes");
	}
	return definition;
}

// battery and cab on, direction set and the relays reset, the pantographs on the main reservoir's air
void prepare(test::vehicle &Loco) {
	Loco.hold_main_reservoir(6.0);
	Loco->bPantKurek3 = true;
	Loco->BatterySwitch(true);
	Loco->CabActivisation();
	for (int i = 0; i < 3 && Loco->DirActive <= 0; ++i) {
		Loco->DirectionForward();
	}
	Loco.run(2.0, line);
	Loco->RelayReset(relay_t::maincircuitground | relay_t::tractionnmotoroverload | relay_t::primaryconverteroverload);
	Loco.run(2.0, line);
}

void raise_pantograph(test::vehicle &Loco) {
	Loco->OperatePantographsValve(operation_t::enable);
	Loco->OperatePantographValve(end::rear, operation_t::enable);
	Loco.run(5.0, line);
}

void close_line_breaker(test::vehicle &Loco) {
	Loco->MainSwitch(true);
	Loco.run(2.0, line);
}

} // namespace

TEST_CASE("mains before pantographs: without the flag the line breaker needs the pantographs' voltage, as before") {
	test::vehicle loco{ep05(false), 0.0, "test", 1};
	REQUIRE_FALSE(loco->PantographsRequireMains);
	prepare(loco);
	close_line_breaker(loco);
	CHECK_FALSE(loco->Mains);
	raise_pantograph(loco);
	CHECK(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	close_line_breaker(loco);
	CHECK(loco->Mains);
	// the pantographs stay up with the line breaker open, and the line breaker opens without voltage
	loco->MainSwitch(false);
	loco.run(2.0, line);
	CHECK(loco->Pantographs[end::rear].is_active);
	close_line_breaker(loco);
	REQUIRE(loco->Mains);
	loco->OperatePantographValve(end::rear, operation_t::disable);
	loco.run(2.0, line);
	CHECK_FALSE(loco->Mains);
}

TEST_CASE("mains before pantographs: with the flag the pantographs rise only through the closed line breaker") {
	test::vehicle loco{ep05(true), 0.0, "test", 1};
	REQUIRE(loco->PantographsRequireMains);
	prepare(loco);
	// the pantographs stay down while the line breaker is open
	raise_pantograph(loco);
	CHECK_FALSE(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == 0.0);
	// the line breaker closes without voltage, and the pantograph rises through it
	close_line_breaker(loco);
	CHECK(loco->Mains);
	raise_pantograph(loco);
	CHECK(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	CHECK(loco->Mains);
	// opening the line breaker drops the pantograph, which stays down until the line breaker closes again
	loco->MainSwitch(false);
	loco.run(2.0, line);
	CHECK_FALSE(loco->Mains);
	CHECK_FALSE(loco->Pantographs[end::rear].is_active);
	CHECK(loco->PantographVoltage == 0.0);
	// a pantograph lowered under the closed line breaker leaves it closed
	close_line_breaker(loco);
	raise_pantograph(loco);
	REQUIRE(loco->Pantographs[end::rear].is_active);
	loco->OperatePantographValve(end::rear, operation_t::disable);
	loco.run(5.0, line);
	CHECK_FALSE(loco->Pantographs[end::rear].is_active);
	CHECK(loco->Mains);
}

TEST_CASE("mains before pantographs: with the flag the line breaker still needs the battery") {
	test::vehicle loco{ep05(true), 0.0, "test", 1};
	loco.hold_main_reservoir(6.0);
	loco.run(2.0, line);
	close_line_breaker(loco);
	CHECK_FALSE(loco->Mains);
}

TEST_CASE("mains before pantographs: with the flag the driver's start-up gets the locomotive ready") {
	// test::start_up() takes the AI's steps of TController::PrepareEngine(), the line breaker first here
	test::vehicle loco{ep05(true), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	REQUIRE(test::start_up(driven, line));
	CHECK(loco->Mains);
	CHECK(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	CHECK(test::is_ready(driven));
}
