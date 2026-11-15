#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// the hints for the AI driver from the AI section of the .fiz

TEST_CASE("ai hints: without the AI section the pantographs follow the built-in logic") {
	vehicle wagon{definition::load("wagon")};

	CHECK(wagon->AIHintPantstate == 0);
	CHECK(wagon->AIHintShuntPantstate == 0);
	CHECK(wagon->AIHintPantUpIfIdle);
}

TEST_CASE("ai hints: the suggested pantograph setup for shunting is read apart from the one for train mode") {
	// eu07/maszyna#461: so the ET41 doesn't shunt with all of its four pantographs up
	vehicle shunting{definition::load("wagon").set("AI", "Pantstate", 1).set("AI", "ShuntPantstate", 2)};

	CHECK(shunting->AIHintPantstate == 1);
	CHECK(shunting->AIHintShuntPantstate == 2);
}

TEST_CASE("ai hints: a .fiz with the setup for train mode only has none for shunting") {
	vehicle train{definition::load("wagon").set("AI", "Pantstate", 3)};

	CHECK(train->AIHintPantstate == 3);
	CHECK(train->AIHintShuntPantstate == 0);
}
