#include "stdafx.h"
#include "doctest.h"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <cstdlib>
#include <set>
#include <utility>
#include <vector>

namespace {

std::set<std::pair<int, int>> squares(std::vector<glm::dvec2> const &Polygon, double const Size) {
	auto const found{geometry::squares_in(Polygon, Size)};
	return {found.begin(), found.end()};
}

} // namespace

TEST_CASE("download area: the tiles a triangle touches") {
	// x + y < 1000: the squares of 256 m whose corner nearest the origin lies inside
	auto const found{squares({{0.0, 0.0}, {1000.0, 0.0}, {0.0, 1000.0}}, 256.0)};
	std::set<std::pair<int, int>> expected;
	for (int x = 0; x < 4; ++x)
		for (int y = 0; x + y < 4; ++y)
			expected.emplace(x, y);
	CHECK(found == expected);
}

TEST_CASE("download area: the tiles of a concave polygon leave out the ones in its notch") {
	// an L, 300 m wide, its arms 1000 m long
	auto const found{squares({{0.0, 0.0}, {1000.0, 0.0}, {1000.0, 300.0}, {300.0, 300.0}, {300.0, 1000.0}, {0.0, 1000.0}}, 256.0)};
	std::set<std::pair<int, int>> expected;
	for (int x = 0; x < 4; ++x)
		expected.insert({{x, 0}, {x, 1}});
	for (int y = 2; y < 4; ++y)
		expected.insert({{0, y}, {1, y}});
	CHECK(found == expected);
	CHECK(found.count({2, 2}) == 0);
}

TEST_CASE("download area: a thin band along a line touches the tiles it crosses, none further") {
	// 4 m wide along the diagonal, from (10, 10) to (2000, 2000)
	auto const found{squares({{10.0, 8.0}, {2000.0, 1998.0}, {2000.0, 2002.0}, {10.0, 12.0}}, 256.0)};
	for (int i = 0; i < 8; ++i)
		CHECK(found.count({i, i}) == 1);
	CHECK(std::none_of(found.begin(), found.end(), [](std::pair<int, int> const &Key) { return std::abs(Key.first - Key.second) > 1; }));
}

TEST_CASE("download area: a polygon inside one tile, or around a corner of four") {
	CHECK(squares({{10.0, 10.0}, {20.0, 10.0}, {20.0, 20.0}, {10.0, 20.0}}, 256.0) == std::set<std::pair<int, int>>{{0, 0}});
	CHECK(squares({{250.0, 250.0}, {262.0, 250.0}, {262.0, 262.0}, {250.0, 262.0}}, 256.0) == std::set<std::pair<int, int>>{{0, 0}, {1, 0}, {0, 1}, {1, 1}});
	// in the grid of the coordinates of Poland, about 500 km from its origin
	auto const distant{squares({{500100.0, 600100.0}, {500200.0, 600100.0}, {500200.0, 600200.0}}, 256.0)};
	CHECK(distant == std::set<std::pair<int, int>>{{1953, 2344}});
}

TEST_CASE("download area: a point inside or outside, a square of no size") {
	std::vector<glm::dvec2> const triangle{{0.0, 0.0}, {100.0, 0.0}, {0.0, 100.0}};
	CHECK(geometry::square_touches(triangle, 10.0, 10.0, 0.0));
	CHECK_FALSE(geometry::square_touches(triangle, 80.0, 80.0, 0.0));
	// a square holding the whole polygon touches it, with no corner of its inside
	CHECK(geometry::square_touches(triangle, -50.0, -50.0, 300.0));
	// fewer than three points: no area
	CHECK(geometry::squares_in({{0.0, 0.0}, {100.0, 100.0}}, 256.0).empty());
}

TEST_CASE("download area: its size, whichever way round it's drawn") {
	std::vector<glm::dvec2> square{{0.0, 0.0}, {2000.0, 0.0}, {2000.0, 2000.0}, {0.0, 2000.0}};
	CHECK(geometry::polygon_area(square) == doctest::Approx(4.0e6));
	std::reverse(square.begin(), square.end());
	CHECK(geometry::polygon_area(square) == doctest::Approx(4.0e6));
	// the L of 300 m, its arms 1000 m long
	CHECK(geometry::polygon_area({{0.0, 0.0}, {1000.0, 0.0}, {1000.0, 300.0}, {300.0, 300.0}, {300.0, 1000.0}, {0.0, 1000.0}}) == doctest::Approx(510000.0));
	CHECK(geometry::polygon_area({{0.0, 0.0}, {100.0, 100.0}}) == 0.0);
}

TEST_CASE("download area: edges crossing each other, as when the corners are clicked out of order") {
	// the corners of a square clicked across: a bow tie
	CHECK(geometry::polygon_crosses_itself({{0.0, 0.0}, {1000.0, 1000.0}, {1000.0, 0.0}, {0.0, 1000.0}}));
	CHECK_FALSE(geometry::polygon_crosses_itself({{0.0, 0.0}, {1000.0, 0.0}, {1000.0, 1000.0}, {0.0, 1000.0}}));
	// a concave polygon isn't a crossing one
	CHECK_FALSE(geometry::polygon_crosses_itself({{0.0, 0.0}, {1000.0, 0.0}, {1000.0, 300.0}, {300.0, 300.0}, {300.0, 1000.0}, {0.0, 1000.0}}));
	// a triangle can't cross itself
	CHECK_FALSE(geometry::polygon_crosses_itself({{0.0, 0.0}, {1000.0, 0.0}, {0.0, 1000.0}}));
	// the last edge, back to the first point, across the first one
	CHECK(geometry::polygon_crosses_itself({{0.0, 0.0}, {1000.0, 0.0}, {1000.0, 1000.0}, {500.0, -500.0}}));
}
