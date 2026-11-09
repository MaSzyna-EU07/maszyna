#include "stdafx.h"
#include "doctest.h"

#include "vehicle/lightsprogrammer.h"

// the lights the light preset selector (programmer) of the occupied cab gives the vehicles of the consist,
// TDynamicObject::SetLights(): the cab's front lights at the end the cab faces, its back lights at the other
// end, and no lights at an end with another vehicle coupled

namespace {

int const pc1{light::headlight_left | light::headlight_upper | light::headlight_right};
int const tailmarkers{light::redmarker_left | light::redmarker_right};
std::array<int, 2> const cablights{pc1, tailmarkers}; // Lights[end::front], Lights[end::rear]

// the lights at the vehicle's own end, placed the way TDynamicObject::RaLightsSet() places them
int lights_at(lights_programmer::consist_lights const &Lights, int const Direction, end const Side) {
	auto const headend{Direction > 0 ? end::front : end::rear};
	return Side == headend ? Lights.head : Lights.rear;
}

struct setup {
	int cab; // CabOccupied of the vehicle with the programmer
	int direction; // its iDirection
	std::array<bool, 2> coupled; // vehicle coupled at its end::front / end::rear
};

lights_programmer::consist_lights lights_for(setup const &Setup) {
	// TDynamicObject::SetLights(): the cab faces the head of the consist when CabOccupied * DirectionGet() >= 0
	auto const cabfacinghead{Setup.cab * (Setup.direction > 0 ? 1 : -1) >= 0};
	return lights_programmer::vehicle_lights(cablights, cabfacinghead, Setup.direction, Setup.coupled);
}

} // namespace

TEST_CASE("lights programmer: a lone vehicle gets the cab's front lights ahead of the cab and its back lights behind it") {
	for (auto const direction : {0, 1}) {
		CAPTURE(direction);
		// cab 1 faces end::front
		auto const fromcab1{lights_for({1, direction, {false, false}})};
		CHECK(lights_at(fromcab1, direction, end::front) == pc1);
		CHECK(lights_at(fromcab1, direction, end::rear) == tailmarkers);
		// cab -1 faces end::rear
		auto const fromcab2{lights_for({-1, direction, {false, false}})};
		CHECK(lights_at(fromcab2, direction, end::rear) == pc1);
		CHECK(lights_at(fromcab2, direction, end::front) == tailmarkers);
	}
}

TEST_CASE("lights programmer: the end with the train coupled goes dark, not the other one (eu07/maszyna#395)") {
	// a locomotive driven from either cab, with the train coupled at either end, either way round in the consist.
	// the end with the train goes dark, the free end shows the lights meant for it
	for (auto const direction : {0, 1}) {
		for (auto const cab : {1, -1}) {
			for (auto const trainend : {end::front, end::rear}) {
				CAPTURE(direction);
				CAPTURE(cab);
				CAPTURE(trainend);
				std::array<bool, 2> coupled{false, false};
				coupled[trainend] = true;
				auto const freeend{trainend == end::front ? end::rear : end::front};
				auto const cabfront{cab > 0 ? end::front : end::rear};

				auto const lights{lights_for({cab, direction, coupled})};

				CHECK(lights_at(lights, direction, trainend) == 0);
				CHECK(lights_at(lights, direction, freeend) == (freeend == cabfront ? pc1 : tailmarkers));
			}
		}
	}
}

TEST_CASE("lights programmer: 6Dg driven backwards from its second cab position pulls the train with its head lights lit (eu07/maszyna#395)") {
	// the reported case: the train coupled behind the driver (end::front, which cab -1 has at its back), the
	// locomotive running towards end::rear, the train in tow. the head lights stay lit, the end with the train goes dark;
	// before the fix the couplings were looked up at the swapped ends and it was the other way round
	auto const lights{lights_for({-1, 1, {true, false}})};

	CHECK(lights_at(lights, 1, end::rear) == pc1);
	CHECK(lights_at(lights, 1, end::front) == 0);
}

TEST_CASE("lights programmer: a vehicle coupled at both ends stays dark") {
	for (auto const direction : {0, 1}) {
		for (auto const cab : {1, -1}) {
			CAPTURE(direction);
			CAPTURE(cab);
			auto const lights{lights_for({cab, direction, {true, true}})};
			CHECK(lights.head == 0);
			CHECK(lights.rear == 0);
		}
	}
}
