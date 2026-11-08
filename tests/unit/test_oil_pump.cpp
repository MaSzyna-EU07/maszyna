#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// the oil pump lamp of the cab (i-oilpump) shows OilPump.is_active: the pre-lubrication pump of 401Da, which works
// before the engine start and stops once the engine runs and makes the oil pressure itself (eu07/maszyna#546)
// NOTE: the test vehicle has no diesel engine, so the pump check is called the way dizel_Update() calls it

TEST_CASE("oil pump: the pre-lubrication pump works before the engine start and stops once the engine runs") {
	vehicle loco{definition::load("cab_vehicle").set("Cntrl", "OilStart", "Manual")};
	REQUIRE(loco->OilPump.start_type == start_t::manual);
	loco->Power24vIsAvailable = true; // battery on

	loco->OilPumpCheck(0.01);
	CHECK_FALSE(loco->OilPump.is_active); // not switched on yet

	loco->OilPump.is_enabled = true; // pump switch on: pre-lubrication
	loco->OilPumpCheck(0.01);
	CHECK(loco->OilPump.is_active);

	loco->Mains = true; // engine started
	loco->OilPumpCheck(0.01);
	CHECK_FALSE(loco->OilPump.is_active);
}

TEST_CASE("oil pump: the pump started with the engine start procedure stops once the engine runs") {
	vehicle loco{definition::load("cab_vehicle").set("Cntrl", "OilStart", "Automatic")};
	REQUIRE(loco->OilPump.start_type == start_t::automatic);
	loco->Power24vIsAvailable = true;

	loco->dizel_startup = true; // starter button pressed
	loco->OilPumpCheck(0.01);
	CHECK(loco->OilPump.is_active);

	loco->dizel_startup = false; // start procedure done, the pump keeps working until the engine runs
	loco->OilPumpCheck(0.01);
	CHECK(loco->OilPump.is_active);

	loco->Mains = true;
	loco->OilPumpCheck(0.01);
	CHECK_FALSE(loco->OilPump.is_active);
}

TEST_CASE("oil pump: without power the pump and its lamp stay off") {
	vehicle loco{definition::load("cab_vehicle")};
	loco->Power24vIsAvailable = false;
	loco->Power110vIsAvailable = false;

	loco->OilPump.is_enabled = true;
	loco->OilPumpCheck(0.01);
	CHECK_FALSE(loco->OilPump.is_active);
}
