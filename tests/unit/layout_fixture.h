#pragma once

// the lines of the track geometry the tests of editor/layout edit and make tracks of: a line of three curves between free ends, the
// norms of a line of 80 km/h of mixed traffic, and the invariants of the lines and the nodes checked on their own, apart from the
// checks of the model

#include "doctest.h"
#include "editor/editorGeometry.hpp"
#include "editor/layout/layoutCommands.hpp"

#include <cmath>
#include <string>

namespace test::layout_fixture {

inline norms::line_class line_class() {
	norms::line_class result;
	result.vmax = 80.0;
	result.kind = norms::traffic::mixed;
	result.track = norms::track_kind::main;
	return result;
}

// a vertex with the cant and the transitions the norms prefer for the radius
inline layout::vertex curve(glm::dvec2 const &Point, double const Radius) {
	auto const preferred{layout::preferred_curve(norms::profile_id::plk, line_class(), Radius)};
	layout::vertex result;
	result.point = Point;
	result.radius = Radius;
	result.cant = preferred.cant;
	result.transition_in = result.transition_out = preferred.transition;
	return result;
}

// three curves, R 800, 600 and 900, with straights between them, a break of the grade line with a vertical curve
inline layout::create_line three_curves(layout::transition_shape const Shape = layout::transition_shape::clothoid, glm::dvec3 const &Offset = glm::dvec3{0.0}) {
	layout::create_line result;
	result.start = glm::dvec3{0.0, 100.0, 0.0} + Offset;
	result.end = glm::dvec3{2200.0, 104.0, 600.0} + Offset;
	glm::dvec2 const shift{Offset.x, Offset.z};
	result.vertices = {curve(glm::dvec2{600.0, 0.0} + shift, 800.0), curve(glm::dvec2{1100.0, 300.0} + shift, 600.0), curve(glm::dvec2{1600.0, 300.0} + shift, 900.0)};
	result.grades = {{1000.0, 103.0 + Offset.y, 10000.0}};
	result.transition = Shape;
	result.ramp = Shape == layout::transition_shape::bloss ? layout::ramp_shape::bloss : layout::ramp_shape::linear;
	result.category = line_class();
	return result;
}

inline std::string reasons_of(layout::outcome const &Outcome) {
	std::string result;
	for (auto const &reason : Outcome.reasons) {
		result += reason + "; ";
	}
	return result;
}

// the invariants, checked here on their own: each line laid and running on from element to element, the cant running on, the ends at
// their nodes and running on through them, the topology of the ends; and the model's own check, the norms along the chains with it
inline void check_invariants(layout::network const &Network) {
	for (auto const &[id, line] : Network.lines()) {
		CAPTURE(layout::number(id));
		auto const &solved{line.solved};
		REQUIRE(solved.valid());
		auto const *start{Network.find(line.start)};
		auto const *end{Network.find(line.end)};
		REQUIRE(start != nullptr);
		REQUIRE(end != nullptr);
		CHECK(line.start != line.end);
		// the line begins at its node, to the bit, and ends there within the sum of the rounding along it
		CHECK(solved.elements.front().origin == geometry::plan_of(start->position));
		auto const &last{solved.elements.back()};
		CHECK(glm::distance(last.position(last.length), geometry::plan_of(end->position)) < 1e-6);
		CHECK(solved.start.position == start->position);
		CHECK(solved.end.position == end->position);
		for (std::size_t i = 1; i < solved.elements.size(); ++i) {
			auto const &a{solved.elements[i - 1]};
			auto const &b{solved.elements[i]};
			CHECK(glm::distance(a.position(a.length), b.origin) < 1e-6);
			CHECK(std::abs(geometry::signed_angle(a.tangent(a.length), b.direction)) < 1e-9);
			CHECK(a.end_chainage() == doctest::Approx(b.chainage).epsilon(1e-12));
		}
		for (std::size_t i = 1; i < solved.cant.size(); ++i) {
			CHECK(std::abs(solved.cant[i - 1].end - solved.cant[i].start) <= 0.5);
			CHECK(solved.cant[i - 1].to == doctest::Approx(solved.cant[i].from).epsilon(1e-12));
		}
		CHECK(std::abs(solved.grade(0.0) - solved.start.grade) < 1e-12);
	}
	for (auto const &[id, node] : Network.nodes()) {
		CAPTURE(layout::number(id));
		REQUIRE_FALSE(node.ends.empty());
		REQUIRE(node.ends.size() <= 2);
		if (node.ends.size() == 2) {
			CHECK(node.ends[0].side() != node.ends[1].side());
		}
		for (auto const &end : node.ends) {
			auto const *line{Network.find(end.line)};
			REQUIRE(line != nullptr);
			CHECK(line->node_at(end.end) == id);
			auto const &values{end.end == layout::line_end::start ? line->solved.start : line->solved.end};
			auto const sense{static_cast<double>(end.sense)};
			CHECK(values.position == node.position);
			CHECK(std::abs(geometry::signed_angle(node.heading, values.heading * sense)) <= 1e-9);
			CHECK(std::abs(values.cant * sense - node.cant) <= 0.5);
			CHECK(std::abs(values.grade * sense - node.grade) <= 0.002);
		}
	}
	auto const verdict{layout::check_all(Network)};
	std::string reasons;
	for (auto const &reason : verdict.reasons) {
		reasons += reason + "; ";
	}
	CAPTURE(reasons);
	CHECK(verdict.fine());
}

} // namespace test::layout_fixture
