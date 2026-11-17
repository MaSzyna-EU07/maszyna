#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

// the joint oil and fuel pump switch (MaSzyna-EU07/maszyna#81) works the pumps which aren't started automatically

namespace {

test::vehicle pumps_vehicle(std::string const &Fuelstart, std::string const &Oilstart) {
	return test::vehicle{test::definition::load("wagon").set("Cntrl", "FuelStart", Fuelstart).set("Cntrl", "OilStart", Oilstart)};
}

// what the two-state joint switch does when pressed, as TTrain::OnCommand_oilfuelpumptoggle()
void press_joint_switch(test::vehicle &Vehicle) {
	auto const enable{false == Vehicle->OilFuelPumpsEnabled()};
	Vehicle->FuelPumpSwitch(enable, range_t::local);
	Vehicle->FuelPumpSwitchOff(false == enable, range_t::local);
	Vehicle->OilPumpSwitch(enable, range_t::local);
	Vehicle->OilPumpSwitchOff(false == enable, range_t::local);
}

} // namespace

TEST_CASE("pumps: the joint switch turns both manual pumps on, then off") {
	auto vehicle{pumps_vehicle("Manual", "Manual")};
	REQUIRE_FALSE(vehicle->OilFuelPumpsEnabled());
	press_joint_switch(vehicle);
	CHECK(vehicle->FuelPump.is_enabled);
	CHECK(vehicle->OilPump.is_enabled);
	CHECK(vehicle->OilFuelPumpsEnabled());
	press_joint_switch(vehicle);
	CHECK_FALSE(vehicle->FuelPump.is_enabled);
	CHECK_FALSE(vehicle->OilPump.is_enabled);
}

TEST_CASE("pumps: with one pump started automatically the joint switch still turns the other one off") {
	auto vehicle{pumps_vehicle("Manual", "Automatic")};
	press_joint_switch(vehicle);
	REQUIRE(vehicle->FuelPump.is_enabled);
	CHECK(vehicle->OilFuelPumpsEnabled()); // the switch shows on
	press_joint_switch(vehicle);
	CHECK_FALSE(vehicle->FuelPump.is_enabled);
}

TEST_CASE("pumps: with one manual pump on and the other off the joint switch turns both on") {
	auto vehicle{pumps_vehicle("Manual", "Manual")};
	vehicle->FuelPumpSwitch(true, range_t::local);
	REQUIRE_FALSE(vehicle->OilFuelPumpsEnabled());
	press_joint_switch(vehicle);
	CHECK(vehicle->FuelPump.is_enabled);
	CHECK(vehicle->OilPump.is_enabled);
}
