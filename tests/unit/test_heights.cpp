#include "stdafx.h"
#include "doctest.h"
#include "editor/editorGroundFollow.hpp"

#include <cmath>
#include <limits>

namespace {

segment_data straight(double const From, double const To) {
	segment_data piece;
	piece.points[segment_data::point::start] = {From, 0.0, 0.0};
	piece.points[segment_data::point::end] = {To, 0.0, 0.0};
	return piece;
}

} // namespace

TEST_CASE("heights: from point to point the tracks meet at one height where a point is clicked just past a joint") {
	// three tracks of 40 m, the grade breaking 3 m past the first joint and 2 m short of the second one, steeply
	std::vector<segment_data> pieces{straight(0.0, 40.0), straight(40.0, 80.0), straight(80.0, 120.0)};
	auto const unknown{std::numeric_limits<double>::quiet_NaN()};

	ground_follow::follow_points(pieces, {{43.0, -5.0, 0.0}, {78.0, 3.0, 0.0}}, {0.0, unknown, 0.0, unknown});

	REQUIRE(pieces.size() >= 3);
	for (std::size_t i = 0; i + 1 < pieces.size(); ++i)
		CHECK(pieces[i].points[segment_data::point::end].y == doctest::Approx(pieces[i + 1].points[segment_data::point::start].y).epsilon(1e-9));
	CHECK(pieces.front().points[segment_data::point::start].y == doctest::Approx(0.0));
	CHECK(pieces.back().points[segment_data::point::end].y == doctest::Approx(0.0));
}

TEST_CASE("heights: with no point between the ends the grade is even from one end to the other") {
	std::vector<segment_data> pieces{straight(0.0, 50.0), straight(50.0, 100.0)};
	auto const unknown{std::numeric_limits<double>::quiet_NaN()};

	ground_follow::follow_points(pieces, {}, {10.0, unknown, 12.0, unknown});

	REQUIRE(pieces.size() == 2);
	CHECK(pieces[0].points[segment_data::point::end].y == doctest::Approx(11.0));
	CHECK(pieces[1].points[segment_data::point::start].y == doctest::Approx(11.0));
}
