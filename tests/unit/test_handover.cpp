#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"

#include "vehicle/handover.h"

using test::definition;
using test::vehicle;

// what the human driver gets when taking a vehicle over from the AI (Q key). the AI operates some controls
// directly and only the AI ever released them, so the player inherited e.g. a horn blowing forever.

namespace {

int const pc1{light::headlight_left | light::headlight_upper | light::headlight_right};
int const allheadlights{pc1 | light::redmarker_left | light::redmarker_right};

vehicle cab_vehicle() {
	vehicle cab{definition::load("cab_vehicle")};
	cab->DirActive = 1; // the AI drives: reverser set
	return cab;
}

} // namespace

TEST_CASE("handover: the horn the AI was sounding goes quiet (eu07/maszyna#511)") {
	auto cab{cab_vehicle()};
	SUBCASE("low tone") { cab->WarningSignal = 1; }
	SUBCASE("high tone") { cab->WarningSignal = 2; }
	SUBCASE("both tones") { cab->WarningSignal = 1 | 2; }
	REQUIRE(cab->WarningSignal != 0);

	handover::release_ai_controls(cab.mover(), cab.mover());

	CHECK(cab->WarningSignal == 0);
}

TEST_CASE("handover: sanding the AI started stops") {
	auto cab{cab_vehicle()};
	cab->Sandbox(true); // the way the AI sands, bypassing the sanding button
	REQUIRE(cab->SandDose);

	handover::release_ai_controls(cab.mover(), cab.mover());

	CHECK_FALSE(cab->SandDose);
}

TEST_CASE("handover: sanding the player or the automatic system asked for goes on") {
	auto cab{cab_vehicle()};
	SUBCASE("sanding button held") { cab->SandboxManual(true); }
	SUBCASE("automatic sanding") {
		cab->SandboxAutoAllow(true);
		cab->SandboxAuto(true);
	}
	REQUIRE(cab->SandDose);

	handover::release_ai_controls(cab.mover(), cab.mover());

	CHECK(cab->SandDose);
}

TEST_CASE("handover: the departure signal the AI was giving stops") {
	auto cab{cab_vehicle()};
	cab->signal_departure(true);
	REQUIRE(cab->DepartureSignal);

	handover::release_ai_controls(cab.mover(), cab.mover());

	CHECK_FALSE(cab->DepartureSignal);
}

TEST_CASE("handover: the brake releaser the AI was holding lets go") {
	auto cab{cab_vehicle()};
	cab->BrakeReleaser(1);
	REQUIRE(cab->Hamulec->Releaser());

	handover::release_ai_controls(cab.mover(), cab.mover());

	CHECK_FALSE(cab->Hamulec->Releaser());
}

TEST_CASE("handover: nothing the AI didn't latch changes") {
	auto cab{cab_vehicle()};

	handover::release_ai_controls(cab.mover(), cab.mover());

	CHECK(cab->WarningSignal == 0);
	CHECK_FALSE(cab->SandDose);
	CHECK_FALSE(cab->DepartureSignal);
	CHECK_FALSE(cab->Hamulec->Releaser());
}

TEST_CASE("handover: the light selector moves to the position giving the head lights the AI has set") {
	auto cab{cab_vehicle()};
	REQUIRE(cab->LightsPos == 1); // untouched selector: lights off

	SUBCASE("Pc1") {
		CHECK(handover::light_selector_position(cab.mover(), end::front, pc1, allheadlights) == 2);
	}
	SUBCASE("lights off") {
		CHECK(handover::light_selector_position(cab.mover(), end::front, 0, allheadlights) == 1);
	}
	SUBCASE("vehicle without the upper headlight, which the AI's Pc1 can't light") {
		auto const inventory{allheadlights & ~light::headlight_upper};
		CHECK(handover::light_selector_position(cab.mover(), end::front, pc1 & inventory, inventory) == 2);
	}
	SUBCASE("no position matches: the selector stays") {
		CHECK(handover::light_selector_position(cab.mover(), end::front, light::headlight_left, allheadlights) == 1);
	}
}

TEST_CASE("handover: the light selector stays within its table") {
	auto cab{cab_vehicle()};

	SUBCASE("selector bigger than the table, as a vehicle definition can declare") {
		// only the positions the table holds are looked at, the rest can't match
		for (auto &table : cab->Lights) {
			std::fill(std::begin(table) + cab->LightsPosNo, std::end(table), 0);
		}
		cab->LightsPosNo = 100;
		CHECK(handover::light_selector_position(cab.mover(), end::front, pc1, allheadlights) == 2);
		CHECK(handover::light_selector_position(cab.mover(), end::front, light::headlight_left, allheadlights) == 1);
	}
	SUBCASE("table other than the front or rear one: the selector stays") {
		CHECK(handover::light_selector_position(cab.mover(), -1, pc1, allheadlights) == 1);
		CHECK(handover::light_selector_position(cab.mover(), 2, pc1, allheadlights) == 1);
	}
}

TEST_CASE("handover: the light selector already matching the lights stays") {
	auto cab{cab_vehicle()};
	cab->LightsPos = 2; // Pc1

	CHECK(handover::light_selector_position(cab.mover(), end::front, pc1, allheadlights) == 2);
}

TEST_CASE("handover: the headlight dimmer leaves the off position when the head lights are lit") {
	auto cab{cab_vehicle()};
	REQUIRE(cab->enableModernDimmer);
	REQUIRE(cab->dimPositions.size() == 5);
	cab->modernDimmerPosition = 2; // off, left there by the player before the AI took over

	SUBCASE("to the default position") {
		CHECK(handover::dimmer_position(cab.mover(), true) == 3);
	}
	SUBCASE("to the first regular position, when the default one is off as well") {
		cab->modernDimmerDefaultPosition = 2;
		CHECK(handover::dimmer_position(cab.mover(), true) == 3);
	}
	SUBCASE("unless the head lights are off anyway") {
		CHECK(handover::dimmer_position(cab.mover(), false) == 2);
	}
}

TEST_CASE("handover: the headlight dimmer in any other position stays") {
	auto cab{cab_vehicle()};
	for (auto const position : {0, 1, 3, 4}) {
		CAPTURE(position);
		cab->modernDimmerPosition = position;
		CHECK(handover::dimmer_position(cab.mover(), true) == position);
	}
}

TEST_CASE("handover: a vehicle without the modern dimmer keeps its dimmer state") {
	vehicle wagon{definition::load("wagon")};
	REQUIRE_FALSE(wagon->enableModernDimmer);

	CHECK(handover::dimmer_position(wagon.mover(), true) == wagon->modernDimmerPosition);
}
