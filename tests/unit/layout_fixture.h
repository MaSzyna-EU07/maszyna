#pragma once

// the lines of the track geometry the tests of editor/layout edit and make tracks of: a line of three curves between free ends, the
// norms of a line of 80 km/h of mixed traffic, and the invariants of the lines and the nodes checked on their own, apart from the
// checks of the model

#include "doctest.h"
#include "editor/editorGeometry.hpp"
#include "editor/editorSwitchCurve.hpp"
#include "editor/layout/layoutCommands.hpp"
#include "editor/layout/layoutTurnout.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

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

inline std::string reasons_of(layout::verdict const &Verdict) {
	std::string result;
	for (auto const &reason : Verdict.refusals()) {
		result += reason + "; ";
	}
	return result;
}

// the invariants, checked here on their own: each line laid and running on from element to element, the cant running on, the ends at
// their nodes and running on through them, the topology of the ends (two at a node, three at the start of a turnout: its two tracks on one
// side of it); the turnouts whole, their main tracks as long as their types, their tracks from their start, their velocity by the speed rule;
// and the model's own check, the norms along the chains and the rules of the turnouts with it (check_invariants; check_structure leaves it out)
inline void check_structure(layout::network const &Network) {
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
			// a track a turnout lays may turn where its secant blades meet the stock rails, by less than 2 degrees
			auto const kink{std::abs(geometry::signed_angle(a.tangent(a.length), b.direction))};
			CHECK((kink < 1e-9 || (layout::laid_by_turnout(Network, id) && kink < 0.035)));
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
		std::vector<layout::end_ref> through;
		for (auto const &end : node.ends) {
			if (layout::branch_end(Network, end)) {
				// a branch of a turnout beside a track of it carrying the line through the node, on its side
				auto const *switched{layout::owner_of(Network, end.line)};
				REQUIRE(switched != nullptr);
				CHECK(std::ranges::any_of(node.ends, [&](layout::end_ref const &Each) {
					auto const *owner{layout::owner_of(Network, Each.line)};
					return owner != nullptr && owner->id == switched->id && false == layout::branch_end(Network, Each) && Each.side() == end.side();
				}));
			}
			else {
				through.push_back(end);
			}
		}
		REQUIRE(through.size() <= 2);
		if (through.size() == 2) {
			CHECK(through[0].side() != through[1].side());
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
	for (auto const &[id, turnout] : Network.turnouts()) {
		CAPTURE(layout::number(id));
		CAPTURE(turnout.type);
		auto const type{layout::turnout_type_of(turnout.type)};
		REQUIRE(type.has_value());
		using layout::turnout_kind;
		auto const kind{type->kind};
		CHECK(layout::kind_of(turnout) == kind);
		for (auto const track : layout::tracks_of(turnout)) {
			auto const *line{Network.find(track)};
			REQUIRE(line != nullptr);
			CHECK(line->turnout == id);
		}
		auto const *main{Network.find(turnout.main)};
		auto const between = [&](layout::line_id const Track, layout::node_id const From, layout::node_id const To, int const Direction) {
			auto const *line{Network.find(Track)};
			REQUIRE(line != nullptr);
			CHECK((Direction > 0 ? line->start : line->end) == From);
			CHECK((Direction > 0 ? line->end : line->start) == To);
			CHECK(line->solved.start.position == Network.find(line->start)->position);
			CHECK(line->solved.end.position == Network.find(line->end)->position);
		};
		switch (kind) {
		case turnout_kind::twofold:
			between(turnout.main, turnout.start, turnout.second, turnout.direction);
			between(turnout.main2, turnout.second, turnout.main_end, turnout.direction);
			between(turnout.diverging, turnout.start, turnout.diverging_end, 1);
			between(turnout.diverging2, turnout.second, turnout.diverging2_end, 1);
			CHECK(main->solved.length == doctest::Approx(type->stagger).epsilon(1e-9));
			CHECK(Network.find(turnout.main2)->solved.length == doctest::Approx(type->length - type->stagger).epsilon(1e-9));
			break;
		case turnout_kind::symmetric:
			between(turnout.main, turnout.start, turnout.main_end, 1);
			between(turnout.diverging, turnout.start, turnout.diverging_end, 1);
			break;
		case turnout_kind::slip:
		case turnout_kind::crossing:
			between(turnout.main, turnout.start, turnout.main_end, turnout.direction);
			between(turnout.crossing, turnout.cross_start, turnout.cross_end, 1);
			if (kind == turnout_kind::slip) {
				between(turnout.diverging, turnout.start, turnout.cross_end, 1);
			}
			if (turnout.diverging2 != layout::line_id::none) {
				between(turnout.diverging2, turnout.cross_start, turnout.main_end, 1);
			}
			CHECK(main->solved.length == doctest::Approx(type->length).epsilon(1e-9));
			break;
		default:
			between(turnout.main, turnout.start, turnout.main_end, turnout.direction);
			between(turnout.diverging, turnout.start, turnout.diverging_end, 1);
			CHECK(main->solved.length == doctest::Approx(type->length).epsilon(1e-9));
			break;
		}
		CHECK(turnout.speed_diverging > 0.0);
		CHECK(turnout.speed_main > 0.0);
		if (kind == turnout_kind::crossing) {
			CHECK(turnout.velocity == turnout.speed_main);
		}
		else {
			CHECK(turnout.velocity == switch_curve::written_velocity(turnout.speed_main, turnout.speed_diverging));
		}
	}
}

inline void check_invariants(layout::network const &Network) {
	check_structure(Network);
	auto const verdict{layout::check_all(Network)};
	auto const reasons{reasons_of(verdict)};
	CAPTURE(reasons);
	CHECK(verdict.fine());
}

} // namespace test::layout_fixture
