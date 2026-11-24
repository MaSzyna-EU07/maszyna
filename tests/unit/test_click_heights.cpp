#include "stdafx.h"
#include "doctest.h"
#include "editor/editorClickHeights.hpp"

#include <limits>
#include <vector>

// The heights of the points clicked while laying: a point clicked where there was no ground yet takes its height again once the terrain
// under it comes, the points level with it go along, the rest stay; Enter waits for the terrain on its way, a while at most
namespace {

using click_heights::source;

double constexpr nothing{std::numeric_limits<double>::quiet_NaN()};

} // namespace

TEST_CASE("a point clicked before the terrain came takes the height of the ground once it's there")
{
	// the first point clicked on the empty place: its height a guess (0); the next ones on the terrain made meanwhile, at 94 m
	std::vector<glm::dvec3> points{{0.0, 0.0, 0.0}, {0.0, 94.2, 30.0}, {0.0, 94.6, 130.0}};
	std::vector<source> sources{source::none, source::ground, source::ground};
	std::vector<double> ground{nothing, 95.0, 96.0};
	auto const under = [&](std::size_t const Index) { return ground[Index]; };
	// no ground under it yet: it waits as it is
	CHECK(click_heights::settle(points, sources, under) == 0);
	CHECK(points[0].y == 0.0);
	CHECK(sources[0] == source::none);
	CHECK(click_heights::guessed(sources));
	CHECK(click_heights::guessed_count(sources) == 1);
	// the terrain came: the point takes its height, the others clicked on the ground keep theirs though the ground is read otherwise now
	ground[0] = 94.0;
	CHECK(click_heights::settle(points, sources, under) == 1);
	CHECK(points[0].y == doctest::Approx(94.0));
	CHECK(points[1].y == doctest::Approx(94.2));
	CHECK(points[2].y == doctest::Approx(94.6));
	CHECK(sources[0] == source::ground);
	CHECK_FALSE(click_heights::guessed(sources));
	// taken once: the ground changing under it later, the terrain shaped by the track, doesn't move it again
	ground[0] = 90.0;
	CHECK(click_heights::settle(points, sources, under) == 0);
	CHECK(points[0].y == doctest::Approx(94.0));
}

TEST_CASE("the points on a track and those of the ground are never read again")
{
	std::vector<glm::dvec3> points{{0.0, 3.5, 0.0}, {0.0, 4.0, 100.0}, {0.0, 0.0, 200.0}};
	std::vector<source> sources{source::fixed, source::ground, source::none};
	std::vector<std::size_t> asked;
	auto const under = [&](std::size_t const Index) {
		asked.push_back(Index);
		return 7.0;
	};
	CHECK(click_heights::settle(points, sources, under) == 1);
	// only the point of a guessed height asked about the ground
	REQUIRE(asked.size() == 1);
	CHECK(asked[0] == 2);
	CHECK(points[0].y == 3.5);
	CHECK(points[1].y == 4.0);
	CHECK(points[2].y == 7.0);
}

TEST_CASE("the points laid level go along with the point of a guessed height they're level with")
{
	// the heights laid level: the first point on the empty place, the next ones at its height
	std::vector<glm::dvec3> points{{0.0, 0.0, 0.0}, {0.0, 0.0, 100.0}, {0.0, 0.0, 200.0}};
	std::vector<source> sources{source::none, source::level, source::level};
	auto const under = [](std::size_t const Index) { return Index == 0 ? 112.5 : 140.0; };
	CHECK(click_heights::settle(points, sources, under) == 3);
	for (auto const &point : points)
		CHECK(point.y == doctest::Approx(112.5));
	// level after a fixed one: stays with it
	std::vector<glm::dvec3> joined{{0.0, 5.0, 0.0}, {0.0, 5.0, 100.0}};
	std::vector<source> held{source::fixed, source::level};
	CHECK(click_heights::settle(joined, held, under) == 0);
	CHECK(joined[1].y == 5.0);
}

TEST_CASE("points beyond the sources known are kept")
{
	// the points a self-test put in by itself have no sources: nothing is read for them
	std::vector<glm::dvec3> points{{0.0, 0.0, 0.0}, {0.0, 0.0, 100.0}};
	std::vector<source> sources{source::none};
	auto const under = [](std::size_t) { return 50.0; };
	CHECK(click_heights::settle(points, sources, under) == 1);
	CHECK(points[0].y == 50.0);
	CHECK(points[1].y == 0.0);
}

TEST_CASE("Enter waits for the terrain on its way under a point of a guessed height, a while at most")
{
	std::vector<source> sources{source::none, source::ground, source::ground};
	auto const coming = [](std::size_t) { return true; };
	auto const gone = [](std::size_t) { return false; };
	// the terrain under the guessed point on its way: waited for
	CHECK(click_heights::waiting(sources, coming));
	CHECK(click_heights::wait_on(sources, coming, 0.0));
	CHECK(click_heights::wait_on(sources, coming, click_heights::wait_longest - 0.1));
	// for long enough: laid at the heights there are
	CHECK_FALSE(click_heights::wait_on(sources, coming, click_heights::wait_longest));
	// none coming there, no data in the model, the terrain off: laid at once
	CHECK_FALSE(click_heights::waiting(sources, gone));
	CHECK_FALSE(click_heights::wait_on(sources, gone, 0.0));
	// only the guessed points wait: the terrain still coming under the ones on the ground doesn't hold the line
	auto const under_second = [](std::size_t const Index) { return Index == 1; };
	CHECK_FALSE(click_heights::waiting(sources, under_second));
	// every point on the ground: nothing to wait for
	std::vector<source> settled{source::ground, source::level, source::fixed};
	CHECK_FALSE(click_heights::waiting(settled, coming));
}
