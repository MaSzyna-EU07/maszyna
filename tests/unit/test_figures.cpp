#include "stdafx.h"
#include "doctest.h"

#include <algorithm>

#include "figures.h"
#include "vehicle.h"

// the key figures of the golden traces (figures.h): the parts of a run between the events, and the comparison
// with the recorded ones, which lets the last decimal and half a percent go and tells the rest in readable lines

using test::definition;
using test::vehicle;

TEST_CASE("figures: a run split into parts by the events, each from the last step of the part before") {
	vehicle eu07{definition::fiz("4e_v1/4e")};
	test::figures figures;
	auto &mover{eu07.mover()};
	mover.V = 10.0;
	mover.PipePress = 5.0;
	figures.sample(0.0, mover);
	figures.mark("brake");
	figures.mark("hard");
	for (int i = 1; i <= 10; ++i) {
		mover.V = 10.0 - i;
		mover.DistCounter += 0.005;
		mover.PipePress = 3.5;
		mover.BrakePress = 0.4 * i;
		figures.sample(i, mover);
	}
	figures.mark("release");
	mover.PipePress = 4.0;
	figures.sample(11.0, mover);
	mover.PipePress = 5.0;
	figures.sample(12.0, mover);

	auto const lines{figures.lines()};
	auto const has{[&lines](std::string const &Line) { return std::find(lines.begin(), lines.end(), Line) != lines.end(); }};
	CHECK(lines.front() == "[start]");
	// two events with nothing run between them make one part
	CHECK(has("[brake, hard]"));
	CHECK(has("duration = 10.00 s"));
	CHECK(has("speed at the start = 36.0 km/h"));
	CHECK(has("distance = 50.0 m"));
	CHECK(has("stopped after = 10.00 s"));
	CHECK(has("brake pipe lowest = 3.500 bar"));
	CHECK(has("brake cylinders peak = 4.000 bar"));
	CHECK(has("brake pipe back at 4.9 bar after = 2.00 s"));
	// no current nor force anywhere in the run: left out of every part
	CHECK_FALSE(has("motor current peak = 0 A"));
}

TEST_CASE("figures: compared within half a percent or the last decimal, the differences told by part") {
	std::vector<std::string> const recorded{"[brake]", "distance = 412.3 m", "stopped after = 38.21 s", "brake cylinders peak = 3.800 bar"};
	CHECK(test::figures::compare(recorded, recorded).empty());
	CHECK(test::figures::compare(recorded, {"[brake]", "distance = 413.9 m", "stopped after = 38.22 s", "brake cylinders peak = 3.810 bar"}).empty());

	auto const differences{test::figures::compare(recorded, {"[brake]", "distance = 398.0 m", "brake cylinders peak = 3.800 bar", "motor current peak = 10 A"})};
	REQUIRE(differences.size() == 3);
	CHECK(differences[0] == "[brake] distance: 398.0 m now, 412.3 m recorded");
	CHECK(differences[1] == "[brake] stopped after: none now, 38.21 s recorded");
	CHECK(differences[2] == "[brake] motor current peak: 10 A now, none recorded");

	auto const parts{test::figures::compare(recorded, {"[brake hard]", "distance = 412.3 m"})};
	REQUIRE(parts.size() == 1);
	CHECK(parts[0] == "the parts differ from part 1: [brake hard] now, [brake] recorded");
}
