#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

// the radio works only switched on and with the low voltage (eu07/maszyna#429): its channel and volume buttons,
// the calls and the radio-stop all ask RadioPowered()

TEST_CASE("radio: a radio switched on with the low voltage works") {
	test::vehicle vehicle{test::definition::load("wagon")};
	vehicle->Radio = true;
	SUBCASE("24 V") { vehicle->Power24vIsAvailable = true; }
	SUBCASE("110 V") { vehicle->Power110vIsAvailable = true; }
	CHECK(vehicle->RadioPowered());
}

TEST_CASE("radio: a radio switched off or without the low voltage doesn't") {
	test::vehicle vehicle{test::definition::load("wagon")};
	SUBCASE("switched off") {
		vehicle->Radio = false;
		vehicle->Power24vIsAvailable = true;
	}
	SUBCASE("batteries off") {
		vehicle->Radio = true;
		vehicle->Power24vIsAvailable = false;
		vehicle->Power110vIsAvailable = false;
	}
	CHECK_FALSE(vehicle->RadioPowered());
}
