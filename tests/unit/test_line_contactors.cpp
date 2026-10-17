#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

// the lamp of the open line contactors (eu07/maszyna#470): lit, it tells the start can be made, so it should go out
// where the start is cut off, by the pressure switch at 2 bar in the brake cylinders, not at 1 bar already

namespace {

test::vehicle locomotive(bool const Pressureswitch) {
	test::vehicle vehicle{test::definition::load("wagon").set("Engine", "PressureSwitch", Pressureswitch)};
	vehicle->StLinFlag = false; // the line contactors open
	vehicle->PipePress = 5.0;
	return vehicle;
}

} // namespace

TEST_CASE("line contactors: with a pressure switch the lamp goes out with the start cut off") {
	auto vehicle{locomotive(true)};
	REQUIRE(vehicle->HasControlPressureSwitch);

	vehicle->BrakePress = 1.5; // braked, the start can still be made
	vehicle->MotorConnectorsCheck(); // updates the pressure switch
	REQUIRE_FALSE(vehicle->ControlPressureSwitch);
	CHECK(vehicle->LineContactorsLampLit());

	vehicle->BrakePress = 2.5; // the pressure switch cuts the start off
	vehicle->MotorConnectorsCheck();
	REQUIRE(vehicle->ControlPressureSwitch);
	CHECK_FALSE(vehicle->LineContactorsLampLit());
}

TEST_CASE("line contactors: without a pressure switch the lamp goes out with the brakes applied, as before") {
	auto vehicle{locomotive(false)};
	vehicle->BrakePress = 0.5;
	vehicle->MotorConnectorsCheck();
	CHECK(vehicle->LineContactorsLampLit());
	vehicle->BrakePress = 1.5;
	vehicle->MotorConnectorsCheck();
	CHECK_FALSE(vehicle->LineContactorsLampLit());
}

TEST_CASE("line contactors: closed contactors put the lamp out") {
	auto vehicle{locomotive(true)};
	vehicle->StLinFlag = true;
	CHECK_FALSE(vehicle->LineContactorsLampLit());
}

TEST_CASE("line contactors: the ground relay and overload lamps follow the same rule") {
	// all the lamps of the main circuit go out together, with the pressure switch where it's fitted
	auto vehicle{locomotive(true)};
	vehicle->BrakePress = 1.5;
	vehicle->MotorConnectorsCheck();
	CHECK(vehicle->MainCircuitLampsEnabled());
	vehicle->BrakePress = 2.5;
	vehicle->MotorConnectorsCheck();
	CHECK_FALSE(vehicle->MainCircuitLampsEnabled());
}
