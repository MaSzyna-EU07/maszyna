#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// the first number of a WiperList row is the mask of the wipers working in that position of the wiper switch,
// bit n for the wiper n. it's a number: read as a character "5" was 53 and "12" was '1', the wiper 0 alone

namespace {

definition wiper_list(nlohmann::ordered_json const &Rows) {
	return definition::load("cab_vehicle")
	    .set("WiperList", "rows", Rows)
	    .set("WiperList", "Size", static_cast<int>(Rows.size()))
	    .set("WiperList", "Angle", 58);
}

} // namespace

TEST_CASE("wipers: the mask of the wipers of a WiperList row is read as a number") {
	vehicle cab{wiper_list(nlohmann::ordered_json::array({
	    {"0", "0.7", "0.0", "0.5"},
	    {"5", "0.7", "7.0", "0.5"},
	    {"12", "0.6", "0.7", "0.0"},
	    {"15", "0.5", "0.5", "0.25"},
	}))};
	REQUIRE(cab->WiperListSize == 4);

	CHECK(cab->WiperList[0].byteSum == 0);
	CHECK(cab->WiperList[1].byteSum == 5);
	CHECK(cab->WiperList[2].byteSum == 12);
	CHECK(cab->WiperList[3].byteSum == 15);

	// and the numbers after it still land in their own fields
	CHECK(cab->WiperList[1].interval == doctest::Approx(7.0));
	CHECK(cab->WiperList[2].WiperSpeed == doctest::Approx(0.6));
	CHECK(cab->WiperList[3].outBackDelay == doctest::Approx(0.25));
}

TEST_CASE("wipers: a mask out of the range of a byte is clamped to it") {
	vehicle cab{wiper_list(nlohmann::ordered_json::array({
	    {"-1", "0.7", "0.0", "0.5"},
	    {"300", "0.7", "0.0", "0.5"},
	}))};

	CHECK(cab->WiperList[0].byteSum == 0);
	CHECK(cab->WiperList[1].byteSum == 255);
}
