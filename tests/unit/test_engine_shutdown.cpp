#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

// stopping the engine of a diesel shunter like the SM03 (eu07/maszyna#427): its fuel pump starts on its own with the
// engine and the engine runs the converter which feeds the low voltage circuit, so neither the fuel pump switch nor the
// battery switch stop it. the cab has only the starter button, and the command opening the main circuit has to stop it

namespace {

// the shunter with its engine started from the cab and idling
test::vehicle running_shunter() {
	test::vehicle shunter{test::definition::load("diesel_shunter")};
	shunter->CabOccupied = 1;
	shunter->CabActivisation();
	shunter->BatterySwitch(true);
	shunter->OilPumpSwitch(true);
	shunter.run(20.0); // the oil pressure builds up
	shunter->MainSwitch(true);
	shunter.run(15.0);
	return shunter;
}

} // namespace

TEST_CASE("engine shutdown: the shunter starts with the starter button") {
	auto shunter{running_shunter()};
	REQUIRE(shunter->EngineType == TEngineType::DieselEngine);
	REQUIRE(shunter->FuelPump.start_type == start_t::automatic);
	CHECK(shunter->Mains);
	CHECK(shunter->FuelPump.is_active);
	CHECK(shunter->enrot > 0.9 * shunter->dizel_nmin);
}

TEST_CASE("engine shutdown: neither the fuel pump switch nor the battery switch stop the engine") {
	auto shunter{running_shunter()};
	shunter->FuelPumpSwitchOff(true, range_t::local); // the pump started automatically ignores its switch
	shunter.run(10.0);
	CHECK(shunter->Mains);
	shunter->BatterySwitch(false);
	shunter.run(10.0);
	CHECK(shunter->Power110vIsAvailable); // the converter run by the engine keeps feeding the fuel pump
	CHECK(shunter->Mains);
	CHECK(shunter->enrot > 0.9 * shunter->dizel_nmin);
}

TEST_CASE("engine shutdown: the command opening the main circuit stops the engine and it stays stopped") {
	auto shunter{running_shunter()};
	REQUIRE(shunter->MainCircuitOpensWithStarterOnly()); // the cab passes the command on with only the starter button
	CHECK(shunter->MainSwitch(false));
	shunter.run(20.0);
	CHECK_FALSE(shunter->Mains);
	CHECK_FALSE(shunter->FuelPump.is_active);
	CHECK(shunter->enrot == 0.0);
}

TEST_CASE("engine shutdown: the engine starts again after it was stopped") {
	auto shunter{running_shunter()};
	shunter->MainSwitch(false);
	shunter.run(20.0);
	REQUIRE(shunter->enrot == 0.0);
	shunter->MainSwitch(true);
	shunter.run(15.0);
	CHECK(shunter->Mains);
	CHECK(shunter->enrot > 0.9 * shunter->dizel_nmin);
}

TEST_CASE("engine shutdown: vehicles without a diesel keep ignoring the command with only the button closing the circuit") {
	test::vehicle wagon{test::definition::load("wagon")};
	CHECK_FALSE(wagon->MainCircuitOpensWithStarterOnly());
	test::vehicle electric{test::definition::load("wagon").set("Engine", "EngineType", "ElectricSeriesMotor")};
	REQUIRE(electric->EngineType == TEngineType::ElectricSeriesMotor);
	CHECK_FALSE(electric->MainCircuitOpensWithStarterOnly());
}
