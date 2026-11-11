#include "stdafx.h"
#include "doctest.h"
#include "drive.h"

// the order of the line breaker and the pantographs (eu07/maszyna#68): on Polish locomotives the pantographs
// come first and the line breaker closes on their voltage; on Czech ones (Cntrl.PantRequiresMain) the line
// breaker closes first, without voltage, and the pantographs rise and stay up only while it's closed. what the
// main circuit feeds still needs the voltage there: the converter and the rest don't run behind a line breaker
// closed on the battery alone, and the traction motors get cut off when the voltage goes

namespace {

constexpr test::track line{0.15, 0.0, test::catenary_3kv};
// a catenary without voltage, and one below the voltage the locomotive works on (Power: MinV, 1900 V)
constexpr test::track dead{0.15, 0.0, 0.0};
constexpr test::track weak{0.15, 0.0, 1000.0};

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

void raise_pantograph(test::vehicle &Loco, test::track const &Track = line) {
	Loco->OperatePantographsValve(operation_t::enable);
	Loco->OperatePantographValve(end::rear, operation_t::enable);
	Loco.run(5.0, Track);
}

void close_line_breaker(test::vehicle &Loco) {
	Loco->MainSwitch(true);
	Loco.run(2.0, line);
}

// the converter and the compressor switched on, the main reservoir left to the compressor from now on
void switch_auxiliaries_on(test::vehicle &Loco) {
	Loco->ConverterSwitch(true);
	Loco->CompressorSwitch(true);
	Loco.hold_main_reservoir(-1.0);
}

// what the main reservoir gains over the given time [bar]; the compressor of the locomotive adds some 0.07 bar a second
double reservoir_gain(test::vehicle &Loco, double const Seconds, test::track const &Track) {
	auto const before{Loco->CompressedVolume};
	Loco.run(Seconds, Track);
	return (Loco->CompressedVolume - before) / Loco->VeselVolume;
}

// the locomotive started up the way the driver does it, the brakes released, then the master controller taken to the
// given position a notch a second, the camshaft following it
void get_under_power(test::vehicle &Loco, test::driven const &Driven, int const Position) {
	REQUIRE(test::start_up(Driven, line));
	test::release_brakes(Driven.cab);
	test::drive(Driven, line, 20.0);
	// until the pressure switch lets the line contactors close
	for (int i = 0; i < 120 && Loco->ControlPressureSwitch; ++i) {
		test::drive(Driven, line, 1.0);
	}
	for (int i = 0; i < Position; ++i) {
		Loco->IncMainCtrl(1);
		test::drive(Driven, line, 1.0);
	}
	test::drive(Driven, line, 2.0);
	REQUIRE(Loco->MainCtrlPos == Position);
	REQUIRE(Loco->MainCtrlActualPos == Position);
	REQUIRE(Loco->RList[Position].R > 0.0); // a resistance position
	REQUIRE(Loco->StLinFlag);
	REQUIRE(std::abs(Loco->Im) > 10.0);
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

// the closed line breaker used to tell there's traction voltage, the undervoltage trip opening it without one. closed
// ahead of the pantographs it tells nothing of the kind, so what the main circuit feeds checks the voltage itself

TEST_CASE("mains before pantographs: with the flag the converter and the compressor wait for the traction voltage") {
	test::vehicle loco{ep05(true), 0.0, "test", 1};
	prepare(loco);
	close_line_breaker(loco);
	REQUIRE(loco->Mains);
	REQUIRE(loco->PantographVoltage == 0.0);
	switch_auxiliaries_on(loco);
	REQUIRE(loco->ConverterAllow);
	REQUIRE(loco->CompressorAllow);

	// the line breaker closed on the battery, the pantographs down: there's nothing to run them on
	CHECK(reservoir_gain(loco, 20.0, line) < 0.01);
	CHECK(loco->Mains);
	CHECK_FALSE(loco->MainCircuitPowered());
	CHECK_FALSE(loco->ConverterFlag);
	CHECK_FALSE(loco->Power110vIsAvailable);
	CHECK_FALSE(loco->CompressorFlag);

	// the pantograph up: the converter starts, and the compressor on its 110 V
	raise_pantograph(loco);
	REQUIRE(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	CHECK(reservoir_gain(loco, 10.0, line) > 0.1);
	CHECK(loco->MainCircuitPowered());
	CHECK(loco->ConverterFlag);
	CHECK(loco->Power110vIsAvailable);
	CHECK(loco->CompressorFlag);

	// the voltage gone they stop, behind the line breaker which stays closed
	auto const *track{&line};
	SUBCASE("the pantograph lowered again") {
		loco->OperatePantographValve(end::rear, operation_t::disable);
		loco.run(5.0, line);
		REQUIRE_FALSE(loco->Pantographs[end::rear].is_active);
	}
	SUBCASE("no voltage in the catenary") {
		track = &dead;
		loco.run(5.0, dead);
		REQUIRE(loco->Pantographs[end::rear].is_active);
	}
	SUBCASE("the voltage in the catenary below the one the locomotive works on") {
		track = &weak;
		loco.run(5.0, weak);
		REQUIRE(loco->PantographVoltage == doctest::Approx(weak.voltage));
	}
	CHECK(reservoir_gain(loco, 10.0, *track) < 0.01);
	CHECK(loco->Mains);
	CHECK_FALSE(loco->MainCircuitPowered());
	CHECK_FALSE(loco->ConverterFlag);
	CHECK_FALSE(loco->Power110vIsAvailable);
	CHECK_FALSE(loco->CompressorFlag);
}

TEST_CASE("mains before pantographs: with the flag whatever else the main circuit feeds waits for the traction voltage too") {
	// a compressor on the main circuit rather than the converter's 110 V, the motor blowers, the blowers of the starting
	// resistors, the heating
	test::vehicle loco{ep05(true).set("Brake", "CompressorPower", "Main"), 0.0, "test", 1};
	REQUIRE(loco->CompressorPower == 0);
	REQUIRE(loco->HeatingPowerSource.SourceType == TPowerSource::Main);
	prepare(loco);
	close_line_breaker(loco);
	REQUIRE(loco->Mains);
	loco->CompressorSwitch(true);
	loco->MotorBlowersSwitch(true, end::front);
	loco->RVentForceOn = true;
	loco->HeatingSwitch(true);
	loco.hold_main_reservoir(-1.0);

	CHECK(reservoir_gain(loco, 10.0, line) < 0.01);
	CHECK_FALSE(loco->CompressorFlag);
	CHECK_FALSE(loco->MotorBlowers[end::front].is_active);
	CHECK(loco->RventRot == 0.0);
	CHECK_FALSE(loco->Heating);

	SUBCASE("nor do they run on a voltage below the one the locomotive works on") {
		raise_pantograph(loco, weak);
		REQUIRE(loco->PantographVoltage == doctest::Approx(weak.voltage));
		CHECK(reservoir_gain(loco, 10.0, weak) < 0.01);
		CHECK(loco->Mains);
		CHECK_FALSE(loco->CompressorFlag);
		CHECK_FALSE(loco->MotorBlowers[end::front].is_active);
		CHECK(loco->RventRot == 0.0);
		CHECK_FALSE(loco->Heating);
	}
	SUBCASE("with the voltage they do") {
		raise_pantograph(loco);
		REQUIRE(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
		CHECK(reservoir_gain(loco, 10.0, line) > 0.1);
		CHECK(loco->CompressorFlag);
		CHECK(loco->MotorBlowers[end::front].is_active);
		CHECK(loco->RventRot > 0.0);
		CHECK(loco->Heating);
	}
}

TEST_CASE("mains before pantographs: without the flag the converter and the compressor run behind the closed line breaker, as before") {
	test::vehicle loco{ep05(false), 0.0, "test", 1};
	prepare(loco);
	raise_pantograph(loco);
	close_line_breaker(loco);
	REQUIRE(loco->Mains);
	switch_auxiliaries_on(loco);
	CHECK(reservoir_gain(loco, 10.0, line) > 0.1);
	CHECK(loco->MainCircuitPowered());
	CHECK(loco->ConverterFlag);
	CHECK(loco->Power110vIsAvailable);
	CHECK(loco->CompressorFlag);
	// too low a voltage opens the line breaker, and they stop with it
	loco.run(5.0, weak);
	CHECK_FALSE(loco->Mains);
	CHECK_FALSE(loco->MainCircuitPowered());
	CHECK_FALSE(loco->ConverterFlag);
	CHECK_FALSE(loco->Power110vIsAvailable);
	CHECK_FALSE(loco->CompressorFlag);
}

// the undervoltage trip was also what cut the traction motors off when the voltage went: the line contactors open
// with the line breaker, which then doesn't close on a running position. where the line breaker holds without
// voltage, its undervoltage relay opens the line contactors itself, until the master controller is back where a
// start begins

TEST_CASE("mains before pantographs: with the flag the line contactors open when the traction voltage goes, and the start begins anew") {
	test::vehicle loco{ep05(true), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);

	// the voltage goes: the line breaker holds, the pantograph stays up, the motors are cut off
	test::drive(driven, dead, 3.0);
	CHECK(loco->Mains);
	CHECK(loco->Pantographs[end::rear].is_active);
	CHECK_FALSE(loco->NoVoltRelay);
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->Im == 0.0);

	// the voltage back, with the master controller still where it was: the camshaft went down to zero and stays
	// there, the motors still cut off; the converter is back on its own
	test::drive(driven, line, 10.0);
	REQUIRE(loco->PantographVoltage == doctest::Approx(test::catenary_3kv));
	REQUIRE(loco->MainCtrlPos == 5);
	CHECK(loco->Mains);
	CHECK(loco->ConverterFlag);
	CHECK_FALSE(loco->NoVoltRelay);
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->MainCtrlActualPos == 0);
	CHECK(loco->Im == 0.0);

	SUBCASE("the master controller back at zero: the start goes as usual") {
		while (loco->MainCtrlPos > 0 && loco->DecMainCtrl(1)) {
			// down to zero
		}
		REQUIRE(loco->MainCtrlPos == 0);
		test::drive(driven, line, 1.0);
		CHECK(loco->NoVoltRelay);
		CHECK_FALSE(loco->StLinFlag);
		for (int i = 0; i < 3; ++i) {
			loco->IncMainCtrl(1);
			test::drive(driven, line, 2.0);
		}
		CHECK(loco->StLinFlag);
		CHECK(loco->MainCtrlActualPos == 3);
		CHECK(std::abs(loco->Im) > 10.0);
	}
	SUBCASE("the master controller back at the first position, where the line contactors close on a start") {
		while (loco->MainCtrlPos > 1 && loco->DecMainCtrl(1)) {
			// down to the first position
		}
		REQUIRE(loco->MainCtrlPos == 1);
		test::drive(driven, line, 2.0);
		CHECK(loco->NoVoltRelay);
		CHECK(loco->StLinFlag);
		CHECK(loco->MainCtrlActualPos == 1);
	}
}

TEST_CASE("mains before pantographs: without the flag the line breaker opens when the traction voltage goes, as before") {
	test::vehicle loco{ep05(false), 0.0, "test", 1};
	auto driven{test::driving(loco)};
	get_under_power(loco, driven, 5);

	test::drive(driven, dead, 3.0);
	CHECK_FALSE(loco->Mains);
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->Im == 0.0);
	// and stays open with the voltage back
	test::drive(driven, line, 10.0);
	CHECK_FALSE(loco->Mains);
	CHECK_FALSE(loco->StLinFlag);
	CHECK(loco->MainCtrlActualPos == 0);
	CHECK(loco->Im == 0.0);
}
