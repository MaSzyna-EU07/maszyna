#include "stdafx.h"
#include "doctest.h"

#include "vehicle/aiwire.h"

#include <cmath>
#include <cstdint>
#include <vector>

// where the overhead wire is for the AI driver (vehicle/aiwire.h): a pantograph against a span of the wire, and how far
// short of its target a train stops to keep its pantograph under the wire. the scene query over these
// (basic_region::wire_over) needs the scene, which the unit tests don't link

namespace {

// a vehicle running along +z on level track, the pantograph's base point 4 m above the origin
glm::dvec3 const front{0.0, 0.0, 1.0};
glm::dvec3 const up{0.0, 1.0, 0.0};
glm::dvec3 const left{1.0, 0.0, 0.0};
glm::dvec3 const pantograph{0.0, 4.0, 0.0};
double const halfwidth{0.5};

// a 100 m span along the track, at height Height and Offset to the side, its middle at Middle along the track
ai_wire::contact along(double const Offset, double const Height, double const Middle = 0.0) {
	return ai_wire::span_contact({Offset, Height, Middle - 50.0}, {0.0, 0.0, 100.0}, pantograph, front, up, left,
	                             halfwidth);
}

// the wire over the route up to Last metres from the head of the train
auto wire_to(double const Last) {
	return [Last](double const Distance) { return Distance <= Last; };
}

} // namespace

TEST_CASE("ai wire: a span centred over the pantograph is on the slide") {
	auto const contact{along(0.0, 6.0)};

	CHECK(contact.within);
	CHECK(contact.vertical == doctest::Approx(2.0));
	CHECK(contact.horizontal == doctest::Approx(-halfwidth));
	CHECK(ai_wire::reaches(contact, 1.0, 3.0));
}

TEST_CASE("ai wire: a span at the very edge of the slide still touches it, one beyond it doesn't") {
	auto const edge{along(halfwidth, 6.0)};
	CHECK(edge.horizontal == doctest::Approx(0.0));
	CHECK(ai_wire::reaches(edge, 1.0, 3.0));
	// either side
	CHECK(ai_wire::reaches(along(-halfwidth, 6.0), 1.0, 3.0));

	auto const beyond{along(halfwidth + 0.1, 6.0)};
	CHECK(beyond.within);
	CHECK(beyond.horizontal == doctest::Approx(0.1));
	CHECK_FALSE(ai_wire::reaches(beyond, 1.0, 3.0));
}

TEST_CASE("ai wire: the wire is reached only within the given heights") {
	auto const contact{along(0.0, 6.0)};

	CHECK_FALSE(ai_wire::reaches(contact, 3.0, 9.0)); // too low: below the lowest height
	CHECK_FALSE(ai_wire::reaches(contact, 0.0, 1.5)); // too high: a bridge's wire above the line
	CHECK(ai_wire::reaches(contact, 2.0, 2.0)); // the limits are included
}

TEST_CASE("ai wire: a span below the pantograph is crossed but out of reach") {
	auto const contact{along(0.0, 3.0)};

	CHECK(contact.within);
	CHECK(contact.vertical == doctest::Approx(-1.0));
	CHECK_FALSE(ai_wire::reaches(contact, 0.0, 9.0));
}

TEST_CASE("ai wire: a span ahead of or behind the pantograph isn't crossed") {
	auto const ahead{along(0.0, 6.0, 60.0)};
	CHECK_FALSE(ahead.within);
	CHECK_FALSE(ai_wire::reaches(ahead, 0.0, 9.0));

	CHECK_FALSE(along(0.0, 6.0, -60.0).within);
	// the ends of the span have the tolerance of the vehicles: 0.001 of the span's length
	CHECK(along(0.0, 6.0, 50.05).within);
	CHECK(along(0.0, 6.0, -50.05).within);
	CHECK_FALSE(along(0.0, 6.0, 50.2).within);
}

TEST_CASE("ai wire: a span skewed against the track is met where it crosses the plane of the pantograph") {
	// the zigzag of the wire: from 0.3 m to the right to 0.3 m to the left over 100 m
	auto const zigzag{ai_wire::span_contact({-0.3, 6.0, -50.0}, {0.6, 0.0, 100.0}, pantograph + glm::dvec3{0, 0, 25.0},
	                                        front, up, left, halfwidth)};
	CHECK(zigzag.within);
	CHECK(zigzag.horizontal == doctest::Approx(0.15 - halfwidth));
	CHECK(ai_wire::reaches(zigzag, 1.0, 3.0));

	// a sloping span from a tunnel portal, 1 m lower at its far end: the height where the plane crosses it
	auto const sloping{ai_wire::span_contact({0.0, 6.0, -50.0}, {0.0, -1.0, 100.0}, pantograph + glm::dvec3{0, 0, 25.0},
	                                         front, up, left, halfwidth)};
	CHECK(sloping.vertical == doctest::Approx(1.25));

	// a span of a wire crossing the line at a right angle lies in the plane of the pantograph over the track, and is
	// out of reach a step along the track (the division by zero trap)
	auto const across{ai_wire::span_contact({-5.0, 6.0, 0.0}, {10.0, 0.0, 0.0}, pantograph, front, up, left, halfwidth)};
	CHECK(across.within);
	CHECK(std::isfinite(across.vertical));
	CHECK_FALSE(ai_wire::reaches(across, 0.0, 9.0)); // its end, 5 m to the side
	CHECK_FALSE(ai_wire::span_contact({-5.0, 6.0, 0.0}, {10.0, 0.0, 0.0}, pantograph + glm::dvec3{0, 0, 1.0}, front, up,
	                                  left, halfwidth)
	                .within);
}

TEST_CASE("ai wire: the direction of travel doesn't change what the pantograph meets") {
	auto const forward{along(0.3, 6.0)};
	auto const backward{ai_wire::span_contact({0.3, 6.0, -50.0}, {0.0, 0.0, 100.0}, pantograph, -front, up, -left,
	                                          halfwidth)};

	CHECK(backward.within);
	CHECK(backward.vertical == forward.vertical);
	CHECK(backward.horizontal == forward.horizontal);
}

TEST_CASE("ai wire: the span test gives the very numbers of the code the vehicles had before it was moved here") {
	// the math of basic_cell::update_traction before it called span_contact, as it was there
	struct legacy {
		bool within;
		double vertical;
		double horizontal;
	};
	auto const legacy_contact{[](glm::dvec3 const &pPoint1, glm::dvec3 const &vParametric,
	                             glm::dvec3 const &pantographposition, glm::dvec3 const &vFront, glm::dvec3 const &vUp,
	                             glm::dvec3 const &vLeft, double const fWidth) {
		auto const paramfrontdot = glm::dot(vParametric, vFront);
		auto const fRaParam =
		    -(glm::dot(pPoint1, vFront) - glm::dot(pantographposition, vFront)) / (paramfrontdot != 0.0 ? paramfrontdot : 0.001);
		if (fRaParam < -0.001 || fRaParam > 1.001) {
			return legacy{false, 0.0, 0.0};
		}
		auto const vStyk = pPoint1 + fRaParam * vParametric;
		auto const vGdzie = vStyk - pantographposition;
		auto const fVertical = glm::dot(vGdzie, vUp);
		auto const fHorizontal = std::abs(glm::dot(vGdzie, vLeft)) - fWidth;
		return legacy{true, fVertical, fHorizontal};
	}};

	// pseudo-random vehicles and spans around a point far from the origin, as in a real scenery
	std::uint32_t seed{12345};
	auto const next{[&seed](double const Low, double const High) {
		seed = seed * 1664525u + 1013904223u;
		return Low + (High - Low) * (seed >> 8) / double(1u << 24);
	}};
	int within{0};
	for (int sample = 0; sample < 2000; ++sample) {
		auto const heading{next(0.0, 6.283185307179586)};
		auto const cant{next(-0.1, 0.1)};
		glm::dvec3 const vFront{std::sin(heading), next(-0.03, 0.03), std::cos(heading)};
		glm::dvec3 const vUp{std::sin(cant) * std::cos(heading), std::cos(cant), -std::sin(cant) * std::sin(heading)};
		auto const vLeft{glm::cross(vUp, vFront)};
		glm::dvec3 const position{next(-20000.0, 20000.0), next(100.0, 300.0), next(-20000.0, 20000.0)};
		auto const pantographposition{position + vUp * 4.2};
		auto const spanheading{heading + next(-0.05, 0.05)};
		glm::dvec3 const vParametric{60.0 * std::sin(spanheading), next(-0.5, 0.5), 60.0 * std::cos(spanheading)};
		auto const pPoint1{position + glm::dvec3{next(-1.0, 1.0), next(5.0, 7.0), next(-1.0, 1.0)} - vParametric * next(-0.2, 1.2)};
		auto const fWidth{next(0.4, 0.7)};

		auto const old{legacy_contact(pPoint1, vParametric, pantographposition, vFront, vUp, vLeft, fWidth)};
		auto const now{ai_wire::span_contact(pPoint1, vParametric, pantographposition, vFront, vUp, vLeft, fWidth)};
		CAPTURE(sample);
		REQUIRE(now.within == old.within);
		if (old.within) {
			++within;
			// bit for bit
			REQUIRE(now.vertical == old.vertical);
			REQUIRE(now.horizontal == old.horizontal);
		}
	}
	// both kinds were tried
	CHECK(within > 200);
	CHECK(within < 1800);
}

TEST_CASE("ai wire: a stop with the wire over the pantograph stays where it is") {
	auto const everywhere{[](double) { return true; }};
	CHECK(ai_wire::wire_shortfall(everywhere, 20.0, 130.0) == 0.0);

	// the wire ends between the pantographs: the rear one the train relies on is under it, with the margin ahead
	CHECK(ai_wire::wire_shortfall(wire_to(118.0), 20.0, 130.0) == 0.0);
	// gaps behind the pantograph don't matter
	auto const gap_behind{[](double const Distance) { return Distance <= 90.0 || Distance >= 95.0; }};
	CHECK(ai_wire::wire_shortfall(gap_behind, 20.0, 130.0) == 0.0);
}

TEST_CASE("ai wire: a stop past the end of the wire is moved back to put the pantograph under the wire") {
	// the head stops at 130 m, the pantograph 20 m behind it at 110 m; the wire ends at 100 m. 2 m steps back to the
	// first place with the wire over the pantograph and 3 m ahead of it: 96 m
	CHECK(ai_wire::wire_shortfall(wire_to(100.0), 20.0, 130.0) == doctest::Approx(14.0));
	// the wire ends right at the pantograph: no margin ahead
	CHECK(ai_wire::wire_shortfall(wire_to(110.0), 20.0, 130.0) == doctest::Approx(4.0));
	// a gap in the wire at the pantograph: the stop goes before the gap
	auto const gap{[](double const Distance) { return Distance <= 107.0 || Distance >= 112.0; }};
	CHECK(ai_wire::wire_shortfall(gap, 20.0, 130.0) == doctest::Approx(6.0));
}

TEST_CASE("ai wire: the margin of wire ahead of the pantograph") {
	// exactly the margin ahead: fine
	CHECK(ai_wire::wire_shortfall(wire_to(113.0), 20.0, 130.0) == 0.0);
	// a little less: one step back
	CHECK(ai_wire::wire_shortfall(wire_to(112.9), 20.0, 130.0) == doctest::Approx(2.0));
	// other margins and steps
	CHECK(ai_wire::wire_shortfall(wire_to(100.0), 20.0, 130.0, 0.0, 1.0) == doctest::Approx(10.0));
	CHECK(ai_wire::wire_shortfall(wire_to(100.0), 20.0, 130.0, 5.0, 1.0) == doctest::Approx(15.0));
}

TEST_CASE("ai wire: without the wire within the walk back the stop stays where it is") {
	auto const nowhere{[](double) { return false; }};
	CHECK(ai_wire::wire_shortfall(nowhere, 20.0, 130.0) == 0.0);

	// the pantograph at 110 m: the wire ends more than 300 m back
	CHECK(ai_wire::wire_shortfall(wire_to(-200.0), 20.0, 130.0) == 0.0);
	// the last place looked at is 300 m back
	CHECK(ai_wire::wire_shortfall(wire_to(-187.0), 20.0, 130.0) == doctest::Approx(300.0));
	CHECK(ai_wire::wire_shortfall(wire_to(-187.0), 20.0, 130.0, 3.0, 2.0, 298.0) == 0.0);
}

TEST_CASE("ai wire: a stop without a coverage or a step to walk with stays where it is") {
	CHECK(ai_wire::wire_shortfall({}, 20.0, 130.0) == 0.0);
	CHECK(ai_wire::wire_shortfall(wire_to(100.0), 20.0, 130.0, 3.0, 0.0) == 0.0);
	CHECK(ai_wire::wire_shortfall(wire_to(100.0), 20.0, 130.0, 3.0, -2.0) == 0.0);
}

TEST_CASE("ai wire: the walk back asks for the places the 2 m steps give") {
	std::vector<double> asked;
	auto const recording{[&asked](double const Distance) {
		asked.push_back(Distance);
		return Distance <= 100.0;
	}};
	ai_wire::wire_shortfall(recording, 20.0, 130.0);

	// the pantograph's place and the margin ahead, then the steps back; the margin is asked only where the place is
	// under the wire
	std::vector<double> const expected{110.0, 108.0, 106.0, 104.0, 102.0, 100.0, 103.0, 98.0, 101.0, 96.0, 99.0};
	CHECK(asked == expected);
}

TEST_CASE("ai wire: which stops keep the pantograph under the wire") {
	using ai_wire::stop_kind;
	// the stops of a train that runs with the wire
	CHECK(ai_wire::stops_under_wire(true, true, false, stop_kind::track_end));
	CHECK(ai_wire::stops_under_wire(true, true, false, stop_kind::timetable_stop));
	// a signal is a wait, the vehicle to couple to may stand without the wire
	CHECK_FALSE(ai_wire::stops_under_wire(true, true, false, stop_kind::signal));
	CHECK_FALSE(ai_wire::stops_under_wire(true, true, false, stop_kind::coupling));
	// without a current collector, with the voltage made up, where the scenery says to coast or lower the pantographs
	CHECK_FALSE(ai_wire::stops_under_wire(false, true, false, stop_kind::track_end));
	CHECK_FALSE(ai_wire::stops_under_wire(true, false, false, stop_kind::track_end));
	CHECK_FALSE(ai_wire::stops_under_wire(true, true, true, stop_kind::timetable_stop));
}
