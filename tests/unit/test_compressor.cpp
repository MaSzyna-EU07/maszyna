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
