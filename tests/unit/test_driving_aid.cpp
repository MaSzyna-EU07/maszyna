#include "stdafx.h"
#include "doctest.h"
#include "application/drivingaid.h"

#include <limits>

// the next limit the driving aid tells the driver of (the panel of F1 and the hud of RmlUi), from what the driver knows of the
// line ahead
namespace
{
using driving_aid::next_speed_limit;
using driving_aid::speed_limit_inputs;

// a driver going at Desired with no limit active, no timetable, the proximity distance far off and nothing ahead
speed_limit_inputs open_line(double const Desired)
{
	speed_limit_inputs inputs;
	inputs.desired = Desired;
	inputs.schedule = -1;
	inputs.last_speed = 0.0;
	inputs.last_distance = -1.0;
	inputs.next_speed = Desired;
	inputs.proximity = 1000.0;
	inputs.obstacle = 5000.0;
	inputs.scan_range = 10000.0;
	return inputs;
}
} // namespace

TEST_CASE("a driver who may not move is told of no limit ahead")
{
	auto inputs{open_line(0.0)};
	inputs.next_speed = 60.0;
	auto const ahead{next_speed_limit(inputs)};
	CHECK(ahead.limit == 0);
	CHECK(ahead.distance == std::numeric_limits<double>::max());
}

TEST_CASE("the same speed ahead is no new limit")
{
	auto const ahead{next_speed_limit(open_line(80.0))};
	CHECK(ahead.limit == 80);
	CHECK(ahead.distance == doctest::Approx(1000.0));
}

TEST_CASE("a stricter limit ahead is told of with its distance")
{
	auto inputs{open_line(100.0)};
	inputs.next_speed = 60.0;
	inputs.proximity = 1200.0;
	auto const ahead{next_speed_limit(inputs)};
	CHECK(ahead.limit == 60);
	CHECK(ahead.distance == doctest::Approx(1200.0));
}

TEST_CASE("the end of the current limit is told of")
{
	auto inputs{open_line(40.0)};
	inputs.last_speed = 100.0;
	inputs.last_distance = 300.0;
	inputs.next_speed = 100.0;
	inputs.proximity = 2000.0;
	SUBCASE("as the speed after it")
	{
		auto const ahead{next_speed_limit(inputs)};
		CHECK(ahead.limit == 100);
		CHECK(ahead.distance == doctest::Approx(300.0));
	}
	SUBCASE("held to the timetable")
	{
		inputs.schedule = 70;
		auto const ahead{next_speed_limit(inputs)};
		CHECK(ahead.limit == 70);
		CHECK(ahead.distance == doctest::Approx(300.0));
	}
}

TEST_CASE("a stricter limit after the end of the current lower one")
{
	auto inputs{open_line(40.0)};
	inputs.last_speed = 100.0;
	inputs.last_distance = 500.0;
	inputs.next_speed = 60.0;
	SUBCASE("is told of when it lies past the end")
	{
		inputs.proximity = 800.0;
		auto const ahead{next_speed_limit(inputs)};
		CHECK(ahead.limit == 60);
		CHECK(ahead.distance == doctest::Approx(800.0));
	}
	SUBCASE("isn't when it lies within the current limit, which is lower")
	{
		inputs.proximity = 400.0;
		auto const ahead{next_speed_limit(inputs)};
		CHECK(ahead.limit == 100);
		CHECK(ahead.distance == doctest::Approx(500.0));
	}
}

TEST_CASE("a limit beyond the line scanned isn't told of")
{
	auto inputs{open_line(60.0)};
	inputs.last_speed = 100.0;
	inputs.last_distance = 12000.0;
	inputs.next_speed = 100.0;
	CHECK(next_speed_limit(inputs).limit == 60);
}

TEST_CASE("the vehicle ahead isn't told of as a limit")
{
	auto inputs{open_line(60.0)};
	inputs.next_speed = 0.0;
	inputs.proximity = 150.0;
	inputs.obstacle = 150.0;
	CHECK(next_speed_limit(inputs).limit == 60);
	inputs.obstacle = 400.0;
	CHECK(next_speed_limit(inputs).limit == 0);
}
