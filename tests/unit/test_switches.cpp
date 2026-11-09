#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

// the kinds of the cab switches, given in the Switches: section of the vehicle definition

TEST_CASE("switches: the train heating switch can spring back (MaSzyna-EU07/maszyna#85)") {
	SUBCASE("impulse") {
		test::vehicle vehicle{test::definition::load("cab_vehicle").set("Switches", "TrainHeating", "Impulse")};
		CHECK(vehicle->HeatingSwitchImpulse); // whichever the case it's written in
	}
	SUBCASE("a plain switch by default") {
		test::vehicle vehicle{test::definition::load("cab_vehicle")};
		CHECK_FALSE(vehicle->HeatingSwitchImpulse);
	}
}
