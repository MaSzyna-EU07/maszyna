#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// the compressor waiting a while after getting its power before it starts (CompressorStartDelay,
// MaSzyna-EU07/maszyna#67): in a diesel it starts some time after the engine, rather than with it

namespace {

double const step{0.01};

// a compressor which starts on its own, fed from the main circuit unless given another Power, its pressure
// switch between MinCP and MaxCP, and an empty main reservoir for it to fill
vehicle compressor_vehicle(double const Delay, std::string const &Power = "Main", double const MinCP = 6.5) {
	auto definition{definition::load("wagon")};
	definition.set("Brake", "Vv", 1.0).set("Brake", "MinCP", MinCP).set("Brake", "MaxCP", 8.0);
	definition.set("Brake", "CompressorSpeed", 0.01);
	if (false == Power.empty()) {
		definition.set("Brake", "CompressorPower", Power);
	}
	definition.set("Cntrl", "CompressorStart", "Automatic");
	if (Delay > 0.0) {
		definition.set("Brake", "CompressorStartDelay", Delay);
	}
	vehicle vehicle{definition};
	vehicle->CompressedVolume = 0.0;
	vehicle->LastSwitchingTime = 10.0; // nothing was switched lately
	return vehicle;
}

// runs the compressor for Seconds, the main circuit as it is, the time since the last switching going on as in
// ComputeTotalForce()
void run_compressor(vehicle &Vehicle, double const Seconds) {
	for (auto i{std::lround(Seconds / step)}; i > 0; --i) {
		Vehicle->LastSwitchingTime += step;
		Vehicle->CompressorCheck(step);
	}
}

} // namespace

TEST_CASE("compressor: without a start delay the compressor starts with its power") {
	auto vehicle{compressor_vehicle(0.0)};
	run_compressor(vehicle, 1.0);
	REQUIRE_FALSE(vehicle->CompressorFlag); // no power, no compressor

	vehicle->Mains = true;
	run_compressor(vehicle, step);
	CHECK(vehicle->CompressorFlag);
}

TEST_CASE("compressor: the compressor starts the delay after getting its power") {
	auto vehicle{compressor_vehicle(5.0)};
	vehicle->Mains = true;

	run_compressor(vehicle, 4.9);
	CHECK_FALSE(vehicle->CompressorFlag);
	CHECK(vehicle->Compressor == doctest::Approx(0.0));

	run_compressor(vehicle, 0.2);
	CHECK(vehicle->CompressorFlag);
	run_compressor(vehicle, 10.0);
	CHECK(vehicle->Compressor > 0.0); // and fills the reservoir
}

TEST_CASE("compressor: the delay starts over once the power is gone") {
	auto vehicle{compressor_vehicle(5.0)};
	vehicle->Mains = true;
	run_compressor(vehicle, 10.0);
	REQUIRE(vehicle->CompressorFlag);

	vehicle->Mains = false; // the engine stopped
	run_compressor(vehicle, 1.0);
	CHECK_FALSE(vehicle->CompressorFlag);

	vehicle->Mains = true; // and started again
	run_compressor(vehicle, 4.9);
	CHECK_FALSE(vehicle->CompressorFlag);
	run_compressor(vehicle, 0.2);
	CHECK(vehicle->CompressorFlag);
}

TEST_CASE("compressor: a compressor fed from the low voltage waits from the low voltage coming on") {
	auto vehicle{compressor_vehicle(5.0, "")}; // the default supply: the converter, the 110 V
	vehicle->Mains = true; // which alone doesn't feed it
	run_compressor(vehicle, 10.0);
	REQUIRE_FALSE(vehicle->CompressorFlag);

	vehicle->Power110vIsAvailable = true;
	run_compressor(vehicle, 4.9);
	CHECK_FALSE(vehicle->CompressorFlag);
	run_compressor(vehicle, 0.2);
	CHECK(vehicle->CompressorFlag);
}

TEST_CASE("compressor: the pressure switch stops the compressor above the cut-out pressure") {
	auto vehicle{compressor_vehicle(0.0)};
	vehicle->Mains = true;
	vehicle->CompressedVolume = 8.5; // over MaxCP, in the reservoir of 1 m3
	run_compressor(vehicle, step);
	CHECK(vehicle->CompressorGovernorLock);
	CHECK_FALSE(vehicle->CompressorFlag);
}

TEST_CASE("compressor: without a pressure switch the air over the cut-out pressure is vented") {
	auto vehicle{compressor_vehicle(0.0, "Main", 8.0)}; // MinCP the same as MaxCP: no pressure switch
	vehicle->Mains = true;
	vehicle->CompressedVolume = 8.5;
	run_compressor(vehicle, step);
	REQUIRE(vehicle->CompressorFlag); // nothing stops it
	CHECK(vehicle->CompressedVolume < 8.5 * 0.81); // a fifth of the air let out
}

TEST_CASE("compressor: an engine stopped before the delay passed counts the whole delay again") {
	auto vehicle{compressor_vehicle(30.0)};
	vehicle->Mains = true; // engine started
	run_compressor(vehicle, 20.0);
	REQUIRE_FALSE(vehicle->CompressorFlag);

	vehicle->Mains = false; // engine stopped, 10 s short of the delay
	run_compressor(vehicle, 1.0);
	vehicle->Mains = true;
	run_compressor(vehicle, 15.0); // 36 s since the first start, but only 15 s since this one
	CHECK_FALSE(vehicle->CompressorFlag);

	run_compressor(vehicle, 15.1);
	CHECK(vehicle->CompressorFlag);
}

TEST_CASE("compressor: the start delay holds back a compressor switched on by hand too") {
	auto vehicle{compressor_vehicle(30.0)};
	vehicle->CompressorStart = start_t::manual;
	vehicle->CompressorAllow = true; // switch in the cabinet on before the engine start
	vehicle->Mains = true;

	run_compressor(vehicle, 29.9);
	CHECK_FALSE(vehicle->CompressorFlag);
	run_compressor(vehicle, 0.2);
	CHECK(vehicle->CompressorFlag);
}

TEST_CASE("compressor: the start delay is read from the brake section") {
	auto const delay{[](std::string const &Section) {
		return vehicle{definition::load("wagon").set(Section, "CompressorStartDelay", 30)}->CompressorStartDelay;
	}};
	CHECK(delay("Brake") == doctest::Approx(30.0));
	CHECK(delay("Cntrl") == doctest::Approx(0.0)); // the key isn't taken from the control section
}

namespace {

// 401Da: the compressor runs from the 110 V of the auxiliary generator driven by the engine, here the converter
// started with the main circuit, which gets up to voltage the converter start delay after the engine start
definition generator_vehicle(double const ConverterDelay, double const CompressorDelay) {
	return definition::load("cab_vehicle")
	    .set("Param", "PWR", 370)
	    .set("Light", "Light", "Generator")
	    .set("Light", "LMaxVoltage", 110)
	    .set("Brake", "Vv", 0.6)
	    .set("Brake", "MinCP", 6.5)
	    .set("Brake", "MaxCP", 8.0)
	    .set("Brake", "CompressorSpeed", 0.003)
	    .set("Brake", "CompressorPower", "Converter")
	    .set("Brake", "CompressorStartDelay", CompressorDelay)
	    .set("Cntrl", "CompressorStart", "Automatic")
	    .set("Cntrl", "ConverterStart", "Automatic")
	    .set("Cntrl", "ConverterStartDelay", ConverterDelay);
}

} // namespace

TEST_CASE("compressor: fed from the converter it waits for the converter start delay") {
	vehicle loco{generator_vehicle(30.0, 0.0)};
	REQUIRE(loco->CompressorPower == 2);
	REQUIRE(loco->ConverterStartDelay == doctest::Approx(30.0));

	loco.run(1.0); // engine off
	CHECK_FALSE(loco->Power110vIsAvailable);

	loco->Mains = true; // engine started
	loco.run(25.0);
	CHECK_FALSE(loco->Power110vIsAvailable);
	CHECK_FALSE(loco->CompressorFlag);

	loco.run(10.0);
	CHECK(loco->Power110vIsAvailable);
	CHECK(loco->CompressorFlag);
}

TEST_CASE("compressor: fed from the converter the start delay counts from the converter voltage") {
	vehicle loco{generator_vehicle(20.0, 10.0)};
	REQUIRE(loco->CompressorStartDelay == doctest::Approx(10.0));

	loco.run(1.0); // engine off
	loco->Mains = true; // engine started
	loco.run(25.0);
	REQUIRE(loco->Power110vIsAvailable); // the converter is up
	CHECK_FALSE(loco->CompressorFlag); // but the compressor waits for its own delay as well

	loco.run(10.0);
	CHECK(loco->CompressorFlag);
}
