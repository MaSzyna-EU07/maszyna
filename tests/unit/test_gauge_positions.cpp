#include "stdafx.h"
#include "doctest.h"

#include "vehicle/gaugepositions.h"

#include <string>

// the sounds of the positions of a cab control (MaSzyna-EU07/maszyna#83): a brake lever moved smoothly (Num3/Num9)
// passes its positions instead of stopping at them, and sounds each one it reaches, as it does when put there

namespace {

// a brake lever with sounds at positions 0, 2 and 5; values in hundredths of a position
std::map<int, std::string> lever_sounds() {
	return {{0, "release"}, {2, "running"}, {5, "service"}};
}

std::string reached(int const Previous, int const Desired) {
	auto positions{lever_sounds()};
	auto const position{gauge_positions::last_reached(positions, Previous, Desired)};
	return position != positions.end() ? position->second : "";
}

} // namespace

TEST_CASE("gauge positions: a lever put at a position sounds it") {
	CHECK(reached(100, 200) == "running");
	CHECK(reached(600, 500) == "service");
}

TEST_CASE("gauge positions: a lever moved smoothly past a position sounds it (MaSzyna-EU07/maszyna#83)") {
	CHECK(reached(190, 205) == "running"); // moving up
	CHECK(reached(510, 495) == "service"); // moving down
}

TEST_CASE("gauge positions: of the positions passed in one go the last one reached sounds") {
	CHECK(reached(50, 520) == "service"); // up past 2 and 5
	CHECK(reached(520, 10) == "running"); // down past 5 and 2, short of 0
	CHECK(reached(520, 0) == "release");
}

TEST_CASE("gauge positions: moving between the positions or off one sounds nothing") {
	CHECK(reached(210, 290) == "");
	CHECK(reached(200, 230) == ""); // leaving a position isn't reaching it
	CHECK(reached(200, 170) == "");
}

TEST_CASE("gauge positions: positions below zero count the same way") {
	// a brake lever with the release position at -1
	std::map<int, std::string> positions{{-1, "release"}, {0, "running"}};
	CHECK(gauge_positions::last_reached(positions, 50, -120)->second == "release"); // down past 0 and -1
	CHECK(gauge_positions::last_reached(positions, -150, -90)->second == "release"); // up past -1
}

TEST_CASE("gauge positions: a control without position sounds reaches none") {
	std::map<int, std::string> positions;
	CHECK(gauge_positions::last_reached(positions, 0, 500) == positions.end());
}
