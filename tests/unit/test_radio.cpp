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

// the cut-off in the radio's power supply (eu07/maszyna#438): a fuse, a desk lever or the switch on the radio's
// power supply, closed unless the driver opens it

TEST_CASE("radio: the radio fuse is closed by default") {
	test::vehicle vehicle{test::definition::load("wagon")};
	CHECK(vehicle->RadioFuse);
}

TEST_CASE("radio: an opened radio fuse cuts the radio off") {
	test::vehicle vehicle{test::definition::load("wagon")};
	vehicle->Radio = true;
	vehicle->Power24vIsAvailable = true;
	vehicle->RadioFuse = false;
	CHECK_FALSE(vehicle->RadioPowered());
	vehicle->RadioFuse = true;
	CHECK(vehicle->RadioPowered());
}

TEST_CASE("radio: opening the radio fuse at a standstill resets the radio-stop, as switching the radio off does") {
	test::vehicle loco{test::definition::fiz("4e_v1/4e")};
	loco->Battery = true;
	loco->Radio = true;
	REQUIRE(loco->RadiostopSwitch(true));
	loco->SecuritySystemCheck(0.1);
	CHECK(loco->RadioStopFlag);
	loco->RadioFuse = false;
	loco->SecuritySystemCheck(0.1);
	CHECK_FALSE(loco->RadioStopFlag);
}
