#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

TEST_CASE("definition: the wagon from data/wagon.json loads with its values") {
	vehicle wagon{definition::load("wagon")};

	CHECK(wagon->Mass == doctest::Approx(42200.0));
	CHECK(wagon->Vmax == doctest::Approx(120.0));
	CHECK(wagon->Dim.L == doctest::Approx(26.8));
	CHECK(wagon->WheelDiameter == doctest::Approx(0.96));
	CHECK(wagon->NAxles == 4);
	CHECK(wagon->MaxBrakePress[3] == doctest::Approx(3.8));
	CHECK(wagon->BrakeSystem == TBrakeSystem::Pneumatic);
}

TEST_CASE("definition: keys set by the test reach the vehicle") {
	vehicle heavy{definition::load("wagon").set("Param", "M", 60000).set("Wheels", "D", 1.0)};

	CHECK(heavy->Mass == doctest::Approx(60000.0));
	CHECK(heavy->WheelDiameter == doctest::Approx(1.0));
}

TEST_CASE("definition: an unknown test vehicle is an error, not a silently empty one") {
	CHECK_THROWS(definition::load("no such vehicle"));
}

TEST_CASE("definition: a misspelt section is an error, not silently skipped by the loader") {
	CHECK_THROWS(definition::load("wagon").set("Brakes", "MBF", 30));
}

TEST_CASE("definition: rows of a table section reach the vehicle") {
	auto const pc1{light::headlight_left | light::headlight_upper | light::headlight_right};
	vehicle wagon{definition::load("wagon")
	                  .set("LightsList", "Size", 2)
	                  .set("LightsList", "Default", 1)
	                  .set("LightsList", "rows", {{0, 0}, {pc1, 0}})};

	CHECK(wagon->LightsPosNo == 2);
	CHECK(wagon->LightsPos == 1);
	CHECK(wagon->Lights[end::front][1] == pc1);
}
