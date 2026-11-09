#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

using test::definition;
using test::vehicle;

// the headlight dimmer's position indexes the DimmerList of the .fiz. a short or an empty list, or a DefaultPos
// past its end, put the dimmer outside the list, where the lights and the cab read past the end of it

namespace {

// rows: high beam, dimmed, off
nlohmann::ordered_json const plain{"no", "no", "no"};
nlohmann::ordered_json const dimmed{"no", "yes", "no"};
nlohmann::ordered_json const off{"no", "no", "yes"};

definition dimmer_list(nlohmann::ordered_json const &Rows, int const Defaultposition, bool const Cycle = false) {
	return definition::load("cab_vehicle")
	    .set("DimmerList", "rows", Rows)
	    .set("DimmerList", "Size", static_cast<int>(Rows.size()))
	    .set("DimmerList", "DefaultPos", Defaultposition)
	    .set("DimmerList", "Cycle", Cycle ? "yes" : "no");
}

} // namespace

TEST_CASE("dimmer: a position outside the list reads as a plain one, neither off nor dimmed") {
	vehicle cab{dimmer_list(nlohmann::ordered_json::array({off, dimmed}), 0)};
	REQUIRE(cab->dimPositions.size() == 2);

	for (auto const position : {-1, 2, 100}) {
		CAPTURE(position);
		cab->modernDimmerPosition = position;
		auto const entry{cab->currentDimPosition()};
		CHECK_FALSE(entry.isOff);
		CHECK_FALSE(entry.isDimmed);
		CHECK_FALSE(entry.isHighBeam);
	}
	cab->modernDimmerPosition = 0;
	CHECK(cab->currentDimPosition().isOff);
	cab->modernDimmerPosition = 1;
	CHECK(cab->currentDimPosition().isDimmed);
}

TEST_CASE("dimmer: positions are clamped to the list") {
	vehicle cab{dimmer_list(nlohmann::ordered_json::array({plain, dimmed, off}), 0)};

	CHECK(cab->clampDimmerPosition(-3) == 0);
	CHECK(cab->clampDimmerPosition(1) == 1);
	CHECK(cab->clampDimmerPosition(3) == 2);
	CHECK(cab->clampDimmerPosition(100) == 2);
}

TEST_CASE("dimmer: a DefaultPos past the end of the list starts the dimmer at its last position") {
	vehicle cab{dimmer_list(nlohmann::ordered_json::array({plain, dimmed}), 7)};

	CHECK(cab->modernDimmerDefaultPosition == 1);
	// the cab puts the modern dimmer to its default position when it's entered (TTrain::set_cab_controls())
	cab->modernDimmerPosition = cab->modernDimmerDefaultPosition;
	CHECK(cab->currentDimPosition().isDimmed);
}

TEST_CASE("dimmer: without the modern dimmer a DefaultPos past the end of the list puts the dimmer at its last position") {
	vehicle cab{dimmer_list(nlohmann::ordered_json::array({plain, dimmed}), 7).set("Switches", "ModernDimmer", "No")};
	REQUIRE_FALSE(cab->enableModernDimmer);

	CHECK(cab->modernDimmerPosition == 1);
	CHECK(cab->currentDimPosition().isDimmed);
}

TEST_CASE("dimmer: an empty list leaves the dimmer at 0, reading as a plain position, and it doesn't move") {
	vehicle cab{dimmer_list(nlohmann::ordered_json::array(), 3, true)};
	REQUIRE(cab->dimPositions.empty());

	CHECK(cab->modernDimmerDefaultPosition == 0);
	CHECK(cab->modernDimmerPosition == 0);
	CHECK(cab->clampDimmerPosition(1) == 0);
	CHECK_FALSE(cab->currentDimPosition().isOff);
	CHECK_FALSE(cab->stepDimmerPosition(1));
	CHECK_FALSE(cab->stepDimmerPosition(-1));
	CHECK(cab->modernDimmerPosition == 0);
}

TEST_CASE("dimmer: stepping stays within the list") {
	SUBCASE("without cycling it stops at the ends") {
		vehicle cab{dimmer_list(nlohmann::ordered_json::array({plain, dimmed, off}), 0)};
		CHECK_FALSE(cab->stepDimmerPosition(-1));
		CHECK(cab->modernDimmerPosition == 0);
		CHECK(cab->stepDimmerPosition(1));
		CHECK(cab->stepDimmerPosition(1));
		CHECK(cab->modernDimmerPosition == 2);
		CHECK_FALSE(cab->stepDimmerPosition(1));
		CHECK(cab->modernDimmerPosition == 2);
	}
	SUBCASE("with cycling it goes around to the other end") {
		vehicle cab{dimmer_list(nlohmann::ordered_json::array({plain, dimmed, off}), 0, true)};
		CHECK(cab->stepDimmerPosition(-1));
		CHECK(cab->modernDimmerPosition == 2);
		CHECK(cab->stepDimmerPosition(1));
		CHECK(cab->modernDimmerPosition == 0);
	}
	SUBCASE("a position past the end steps from the last one") {
		vehicle cab{dimmer_list(nlohmann::ordered_json::array({plain, dimmed, off}), 0)};
		cab->modernDimmerPosition = 9;
		CHECK(cab->stepDimmerPosition(-1));
		CHECK(cab->modernDimmerPosition == 1);
		cab->modernDimmerPosition = 9;
		CHECK(cab->stepDimmerPosition(1)); // can't go further, but comes back into the list
		CHECK(cab->modernDimmerPosition == 2);
	}
}
