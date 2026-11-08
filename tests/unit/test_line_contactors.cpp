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

// on the diesel-electric SU45 and SU46 (eu07/maszyna#263) the lamp tells just the line contactors are open: lit with the
// master controller at 0 and with the start cut off by too low a pressure in the main pipe, out on the power positions

namespace {

// the diesel-electric locomotive with its engine running and the brakes released, the master controller at 0
test::vehicle diesel_electric() {
	test::vehicle vehicle{test::definition::load("diesel_electric")};
	vehicle->CabOccupied = 1;
	vehicle->CabActivisation();
	vehicle->BatterySwitch(true);
	vehicle->OilPumpSwitch(true);
	vehicle->FuelPumpSwitch(true);
	vehicle.run(20.0); // the oil pressure builds up
	vehicle.hold_brake_pipe(5.0); // bar
	vehicle->MainSwitch(true);
	vehicle.run(30.0);
	vehicle->DirectionForward();
	vehicle.run(60.0); // the brakes release
	return vehicle;
}

} // namespace

TEST_CASE("line contactors: on a diesel-electric the lamp is lit at 0 and goes out on a power position") {
	auto vehicle{diesel_electric()};
	REQUIRE(vehicle->EngineType == TEngineType::DieselElectric);
	REQUIRE(vehicle->Mains);
	REQUIRE_FALSE(vehicle->ControlPressureSwitch);
	CHECK_FALSE(vehicle->StLinFlag);
	CHECK(vehicle->LineContactorsLampLit());

	vehicle->IncMainCtrl(1);
	vehicle.run(3.0);
	CHECK(vehicle->StLinFlag);
	CHECK_FALSE(vehicle->LineContactorsLampLit());

	vehicle->DecMainCtrl(1);
	vehicle.run(3.0);
	CHECK_FALSE(vehicle->StLinFlag);
	CHECK(vehicle->LineContactorsLampLit());
}

TEST_CASE("line contactors: on a diesel-electric the lamp lights with the contactors opened by too low a main pipe pressure") {
	auto vehicle{diesel_electric()};
	vehicle->IncMainCtrl(1);
	vehicle.run(1.0);
	vehicle->IncMainCtrl(1);
	vehicle.run(2.0);
	REQUIRE(vehicle->StLinFlag);

	vehicle.hold_brake_pipe(2.5); // bar, below the 3.6 bar of the pressure switch
	vehicle.run(10.0);
	REQUIRE(vehicle->ControlPressureSwitch);
	CHECK_FALSE(vehicle->StLinFlag);
	CHECK(vehicle->LineContactorsLampLit()); // it went out here
	CHECK_FALSE(vehicle->MainCircuitLampsEnabled()); // the ground relay and overload lamps still go out with the start cut off

	// the pressure back, the contactors stay open on the second position and the lamp stays lit, as it should;
	// they close again only on the first one
	vehicle.hold_brake_pipe(5.0);
	vehicle.run(60.0);
	REQUIRE_FALSE(vehicle->ControlPressureSwitch);
	CHECK_FALSE(vehicle->StLinFlag);
	CHECK(vehicle->LineContactorsLampLit());
	vehicle->DecMainCtrl(1);
	vehicle.run(3.0);
	CHECK(vehicle->StLinFlag);
	CHECK_FALSE(vehicle->LineContactorsLampLit());
}
