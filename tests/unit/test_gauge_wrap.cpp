#include "stdafx.h"
#include "doctest.h"

#include "vehicle/gaugewrap.h"
#include "vehicle/gaugepositions.h"

#include <cmath>
#include <map>

// a light preset selector with Wrap=Yes turned on past its last position to the first one, or back past the first one
// to the last (eu07/maszyna#464): the EN57AKM selector has 0 as its first position and A1 as its last one, and the
// knob, rather than jump between them with no sound, turns on to the next position as it does between any two others

TEST_CASE("gauge wrap: a knob whose positions make a full turn turns on past its end") {
	CHECK(gauge_wrap::full_turn(-0.08333f, 12) == 12); // E186, BR285: 12 positions, 30 degrees each
	CHECK(gauge_wrap::full_turn(-0.0625f, 16) == 16); // E189: 16 positions
	CHECK(gauge_wrap::full_turn(-0.167f, 6) == 6); // EN57AL Zachodniopomorskie: a sixth given as 0.167
}

TEST_CASE("gauge wrap: a knob with a free position in its turn turns on across it (eu07/maszyna#464)") {
	// EN57AKM: 7 positions, 45 degrees each, A1 at 270 degrees, 0 at 0 degrees: 90 degrees on from A1 to 0
	CHECK(gauge_wrap::full_turn(0.125f, 7) == 8);
	// the way on is half a turn at most
	CHECK(gauge_wrap::full_turn(0.25f, 3) == 4);
	CHECK_FALSE(gauge_wrap::full_turn(0.125f, 4).has_value());
}

TEST_CASE("gauge wrap: a switch whose positions don't make a full turn doesn't turn on past its end") {
	CHECK_FALSE(gauge_wrap::full_turn(0.1f, 12).has_value()); // the positions would overlap in a turn
	CHECK_FALSE(gauge_wrap::full_turn(0.15f, 6).has_value()); // 6.67 positions in a turn
	CHECK_FALSE(gauge_wrap::full_turn(0.f, 6).has_value()); // no motion
	CHECK_FALSE(gauge_wrap::full_turn(0.5f, 1).has_value()); // nothing to turn to
}

TEST_CASE("gauge wrap: turned round, the knob stands the same one turn earlier or later") {
	// EN57AKM, from A1 (value 6) on to 0: from -2 up to 0, 90 degrees the way it was turned
	CHECK(gauge_wrap::turned_round(6.f, 8, true) == doctest::Approx(-2.f));
	// from 0 back to A1 (value 6): from 8 down to 6
	CHECK(gauge_wrap::turned_round(0.f, 8, false) == doctest::Approx(8.f));
	// the angle stays: value * scale differs by one full turn
	CHECK((6.f - gauge_wrap::turned_round(6.f, 8, true)) * 0.125f == doctest::Approx(1.f));
}

TEST_CASE("gauge wrap: turned round, the knob sounds the position it reaches (eu07/maszyna#464)") {
	// the EN57AKM selector has a sound for each position 0-7, as values of the gauge
	std::map<int, int> sounds{{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}, {6, 6}, {7, 7}};
	auto const reached{[&sounds](float const From, float const To) {
		auto const position{gauge_positions::last_reached(sounds, static_cast<int>(std::round(100.f * From)), static_cast<int>(std::round(100.f * To)))};
		return position != sounds.end() ? position->second : -1;
	}};
	// A1 (6) on to 0
	CHECK(reached(gauge_wrap::turned_round(6.f, 8, true), 0.f) == 0);
	// 0 back to A1 (6)
	CHECK(reached(gauge_wrap::turned_round(0.f, 8, false), 6.f) == 6);
}
