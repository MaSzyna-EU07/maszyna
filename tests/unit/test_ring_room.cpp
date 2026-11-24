#include "stdafx.h"
#include "doctest.h"
#include "editor/editorRingRoom.hpp"

#include <cmath>

// the room a road leaving the side of a roundabout takes of the ring (eu07/maszyna GH#349): with the ring's bend
// counted, as many roads fit on a small roundabout as there's room for between their edges
namespace {

double constexpr pi{3.14159265358979323846};
// as the road editor has them: a junction takes the road it's made on past where the edges meet by the corner room,
// and between two junctions there's at least the shortest road left
double constexpr kCornerRoom{4.0};
double constexpr kShortestRoad{2.0};
// the roads the editor builds unless told otherwise: a lane each way, 3.5 m each, and a roundabout of a single lane
double constexpr kRoadHalfwidth{3.5};
double constexpr kRingHalfwidth{1.75};

glm::dvec2 at(double const Radius, double const Angle) { return glm::dvec2{std::cos(Angle), std::sin(Angle)} * Radius; }
glm::dvec2 ahead(double const Angle) { return {-std::sin(Angle), std::cos(Angle)}; }

// room of a road leaving the ring of the radius at the angle, turned from the way straight out by the slant (to the way ahead)
double room(double const Radius, double const Slant, bool const Ahead, double const Halfwidth = kRingHalfwidth, double const Branchhalfwidth = kRoadHalfwidth) {
	double const angle{0.3};
	auto const outwards{at(1.0, angle)};
	auto const direction{outwards * std::cos(Slant) + ahead(angle) * std::sin(Slant)};
	return editor_ring::edge_room(glm::dvec2{0.0}, Radius, at(Radius, angle), direction, Ahead ? ahead(angle) : -ahead(angle), Halfwidth, Branchhalfwidth);
}

// where the edges of two straight roads meet, measured along the first one, as the editor has it for a road leaving a straight road.
// Lean: the second one leans towards the side measured (the angle between them there is the acute one)
double straight_reach(double const Halfwidth, double const Branchhalfwidth, double const Angle, bool const Lean) {
	return (Branchhalfwidth + (Lean ? 1.0 : -1.0) * Halfwidth * std::cos(Angle)) / std::sin(Angle);
}

// whether the roads leaving straight out at the angles, all around the ring, each get a junction: the junctions with what's left between them
bool fit(double const Radius, int const Roads, bool const Bend) {
	auto const side{Bend ? room(Radius, 0.0, true) : straight_reach(kRingHalfwidth, kRoadHalfwidth, pi / 2.0, true)};
	auto const junction{2.0 * (side + kCornerRoom)};
	return Roads * (junction + kShortestRoad) <= 2.0 * pi * Radius;
}

} // namespace

TEST_CASE("ring room: on a very large ring the room is the one of a straight road") {
	for (double const slant : {0.0, pi / 12.0, pi / 6.0, pi / 4.0, pi / 3.0}) {
		CAPTURE(slant);
		double const angle{pi / 2.0 - slant};
		CHECK(room(1.0e6, slant, true) == doctest::Approx(straight_reach(kRingHalfwidth, kRoadHalfwidth, angle, true)).epsilon(1e-4));
		CHECK(room(1.0e6, slant, false) == doctest::Approx(straight_reach(kRingHalfwidth, kRoadHalfwidth, angle, false)).epsilon(1e-4));
	}
}

TEST_CASE("ring room: a road leaving straight out meets the outer edge where the circle reaches its edge") {
	for (double const radius : {8.0, 15.0, 40.0}) {
		CAPTURE(radius);
		double const expected{radius * std::asin(kRoadHalfwidth / (radius + kRingHalfwidth))};
		CHECK(room(radius, 0.0, true) == doctest::Approx(expected));
		CHECK(room(radius, 0.0, false) == doctest::Approx(expected));
		// never more than on a straight road
		CHECK(room(radius, 0.0, true) < kRoadHalfwidth);
	}
}

TEST_CASE("ring room: a slanted road takes less on the side it leans away from") {
	for (double const radius : {8.0, 15.0, 50.0}) {
		for (double const slant : {pi / 6.0, pi / 4.0}) {
			CAPTURE(radius);
			CAPTURE(slant);
			auto const towards{room(radius, slant, true)};
			auto const away{room(radius, slant, false)};
			CHECK(away >= 0.0);
			CHECK(away < towards);
			// the editor took the room of the acute corner on both sides
			CHECK(towards < straight_reach(kRingHalfwidth, kRoadHalfwidth, pi / 2.0 - slant, true));
		}
	}
	// on a straight road the obtuse corner takes less than the acute one too
	CHECK(room(1.0e6, pi / 4.0, false) < room(1.0e6, pi / 4.0, true));
	// leaving the smallest ring as sharply as the editor allows, 30 degrees off it, the edge on the side the road leans away from
	// is outside the ring all along: there's no point to measure to, and the editor keeps the room it took before
	CHECK(room(8.0, pi / 3.0, false) < 0.0);
	CHECK(room(8.0, pi / 3.0, true) > 0.0);
	CHECK(room(15.0, pi / 3.0, false) >= 0.0);
}

TEST_CASE("ring room: none for a road going inwards") {
	CHECK(room(15.0, pi * 0.75, true) < 0.0);
	CHECK(room(15.0, pi, true) < 0.0);
	CHECK(editor_ring::edge_room(glm::dvec2{0.0}, 0.0, glm::dvec2{1.0, 0.0}, glm::dvec2{1.0, 0.0}, glm::dvec2{0.0, 1.0}, 1.0, 1.0) < 0.0);
}

TEST_CASE("ring room: a road wider than the ring is round still meets its edge, sooner than on a straight road") {
	// two lanes each way on the smallest roundabout: the edge of the road starts outside the ring
	double const halfwidth{7.0};
	double const radius{8.0};
	double const expected{radius * std::atan2(halfwidth, std::sqrt((radius + kRingHalfwidth) * (radius + kRingHalfwidth) - halfwidth * halfwidth))};
	CHECK(room(radius, 0.0, true, kRingHalfwidth, halfwidth) == doctest::Approx(expected));
	CHECK(room(radius, 0.0, true, kRingHalfwidth, halfwidth) < halfwidth);
}

TEST_CASE("ring room: roads joined to roundabouts of the radii the editor allows") {
	// the smallest roundabout took two roads; three were too many for it
	CHECK(fit(8.0, 2, false));
	CHECK_FALSE(fit(8.0, 3, false));
	CHECK(fit(8.0, 3, true));
	// four roads from 11 m
	CHECK(fit(11.0, 4, true));
	CHECK(fit(15.0, 4, true));
	// six from 16 m, where they took 17 m before
	CHECK_FALSE(fit(16.0, 6, false));
	CHECK(fit(16.0, 6, true));
	CHECK(fit(25.0, 6, true));
	// and more on the large ones
	CHECK(fit(50.0, 8, true));
	CHECK(fit(200.0, 30, true));
}
