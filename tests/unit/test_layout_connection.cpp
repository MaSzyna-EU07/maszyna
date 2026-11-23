// the solver of the connections of the tracks (editor/layout/layoutConnection, editor-tracks-domain.md §7 "Połącz tory", stage E5): a
// crossover between two lines alongside, straight or curved, of two turnouts and the line between their diverging tracks, one step to undo;
// the spacing of the lines measured along the whole crossover, not at one point; the least straight between the turnouts from the norms
// (ST-T1-A6 11.3 pkt 7: max(0.15 V; 6) m, 30 m on a line of 160 km/h or more); the least spacing of tab. 11.2; refused with the best which
// fits where the norms can't be met

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"

#include <cmath>

namespace {

using namespace test::layout_fixture;
using layout::line_id;

// a straight from the point along x, 1 km
layout::create_line along_x(glm::dvec3 const &Start, glm::dvec3 const &End, double const Speed = 80.0) {
	layout::create_line result;
	result.start = Start;
	result.end = End;
	result.category = line_class();
	result.category.vmax = Speed;
	return result;
}

// two lines, the second one made after the first
std::pair<line_id, line_id> two_lines(layout::model &Model, layout::create_line const &First, layout::create_line const &Second) {
	auto const a{Model.execute(First)};
	auto const b{Model.execute(Second)};
	REQUIRE(a.done);
	REQUIRE(b.done);
	return {a.lines.front(), b.lines.front()};
}

} // namespace

TEST_CASE("layout connection: a crossover between straights alongside, one straight between the turnouts, one step to undo") {
	for (auto const direction : {1, -1}) {
		for (auto const side : {1.0, -1.0}) {
			CAPTURE(direction);
			CAPTURE(side);
			layout::model model;
			auto const [a, b]{two_lines(model, along_x({0.0, 100.0, 0.0}, {1000.0, 100.0, 0.0}), along_x({0.0, 100.0, side * 4.75}, {1000.0, 100.0, side * 4.75}))};
			auto const before{model.state()};
			layout::connect_tracks const request{a, direction > 0 ? 300.0 : 700.0, direction, b, "Rz 60E1-500-1:12"};
			auto const plan{layout::plan_connection(model.state(), request)};
			CAPTURE(plan.reasons.empty() ? std::string{} : plan.reasons.front());
			REQUIRE(plan.fits);
			// the spacing the same all along it, the straight between the turnouts: 4.75 m less the offsets of the two ends, at 1:12
			CHECK(plan.spacing_least == doctest::Approx(4.75).epsilon(1e-9));
			CHECK(plan.spacing_most == doctest::Approx(4.75).epsilon(1e-9));
			CHECK(plan.straight == doctest::Approx((4.75 - layout::crossover_spacing("Rz 60E1-500-1:12", 0.0)) / std::sin(std::atan(1.0 / 12.0))).epsilon(1e-9));
			CHECK(plan.straight_needed == doctest::Approx(9.0));
			CHECK(plan.turn == doctest::Approx(0.0).scale(1.0).epsilon(1e-12));
			auto const made{model.execute(request)};
			CAPTURE(reasons_of(made));
			REQUIRE(made.done);
			auto const &state{model.state()};
			check_invariants(state);
			REQUIRE(made.turnouts.size() == 2);
			// the line between the ends of the diverging tracks a straight, the ends facing each other on it
			auto const &first{*state.find(made.turnouts[0])};
			auto const &second{*state.find(made.turnouts[1])};
			auto const *from{state.find(first.diverging_end)};
			auto const *to{state.find(second.diverging_end)};
			CHECK(std::abs(geometry::signed_angle(from->heading, -to->heading)) < 1e-12);
			auto const joining{made.lines.back()};
			CHECK(state.find(joining)->shape.vertices.empty());
			CHECK(state.find(joining)->solved.length == doctest::Approx(plan.straight).epsilon(1e-9));
			CHECK(first.hand == static_cast<int>(side) * direction);
			// one step to undo
			REQUIRE(model.undo());
			CHECK(model.state().same(before));
		}
	}
}

TEST_CASE("layout connection: the spacing measured along the whole crossover, not at its start") {
	layout::model model;
	auto const needed{layout::crossover_spacing("Rz 60E1-500-1:12", 9.0)};
	// the second line closing in on the first, by 2 mm a metre: as wide as needs be where the crossover starts, narrower on along it
	auto const at_start{needed + 0.05};
	auto const [a, b]{two_lines(model, along_x({0.0, 100.0, 0.0}, {1000.0, 100.0, 0.0}), along_x({0.0, 100.0, at_start + 0.6}, {1000.0, 100.0, at_start - 1.4}))};
	layout::connect_tracks const request{a, 300.0, 1, b, "Rz 60E1-500-1:12"};
	auto const plan{layout::plan_connection(model.state(), request)};
	CHECK_FALSE(plan.fits);
	CHECK(plan.spacing_most > needed);
	CHECK(plan.spacing_least < needed);
	CHECK(plan.spacing_most - plan.spacing_least > 0.15);
	std::string reasons;
	for (auto const &reason : plan.reasons) {
		reasons += reason + "; ";
	}
	CAPTURE(reasons);
	CHECK(reasons.find("apart along the crossover") != std::string::npos);
	auto const before{model.state()};
	CHECK_FALSE(model.execute(request).done);
	CHECK(model.state().same(before));
}

TEST_CASE("layout connection: the least straight between the turnouts from the norms, refused with the type which fits") {
	layout::model model;
	auto const spacing{layout::crossover_spacing("Rz 60E1-500-1:12", 9.0) - 0.15};
	auto const [a, b]{two_lines(model, along_x({0.0, 100.0, 0.0}, {1000.0, 100.0, 0.0}), along_x({0.0, 100.0, spacing}, {1000.0, 100.0, spacing}))};
	auto const plan{layout::plan_connection(model.state(), {a, 300.0, 1, b, "Rz 60E1-500-1:12"})};
	CHECK_FALSE(plan.fits);
	// 60 km/h through it: max(0.15 V; 6) = 9 m (ST-T1-A6 11.3 pkt 7d)
	CHECK(plan.straight_needed == doctest::Approx(9.0));
	CHECK(plan.straight < 9.0);
	std::string reasons;
	for (auto const &reason : plan.reasons) {
		reasons += reason + "; ";
	}
	CAPTURE(reasons);
	CHECK(reasons.find("11.3 pkt 7") != std::string::npos);
	// the proposal: a type of the catalogue which fits these lines, laid as proposed
	REQUIRE_FALSE(plan.proposal.empty());
	auto const proposed{plan.proposal.substr(0, plan.proposal.find(" fits "))};
	CAPTURE(proposed);
	REQUIRE(layout::turnout_type_of(proposed).has_value());
	auto const made{model.execute(layout::connect_tracks{a, 300.0, 1, b, proposed})};
	CAPTURE(reasons_of(made));
	CHECK(made.done);
	check_invariants(model.state());
	// on a line of 160 km/h the norms ask for 30 m between them (11.3 pkt 7e)
	layout::model fast;
	auto const [c, d]{two_lines(fast, along_x({0.0, 100.0, 0.0}, {1000.0, 100.0, 0.0}, 160.0), along_x({0.0, 100.0, 4.5}, {1000.0, 100.0, 4.5}, 160.0))};
	auto const fast_plan{layout::plan_connection(fast.state(), {c, 300.0, 1, d, "Rz 60E1-500-1:12"})};
	CHECK(fast_plan.straight_needed == doctest::Approx(30.0));
	CHECK_FALSE(fast_plan.fits);
}

TEST_CASE("layout connection: the least spacing of tab. 11.2 for the type where it's more than its geometry needs") {
	layout::model model;
	// Rz 60E1-760-1:14: 4.75 m by tab. 11.2, a little less by its geometry with 12 m between its turnouts
	auto const geometric{layout::crossover_spacing("Rz 60E1-760-1:14", 12.0)};
	REQUIRE(geometric < 4.75);
	auto const [a, b]{two_lines(model, along_x({0.0, 100.0, 0.0}, {1000.0, 100.0, 0.0}), along_x({0.0, 100.0, 4.74}, {1000.0, 100.0, 4.74}))};
	auto const plan{layout::plan_connection(model.state(), {a, 300.0, 1, b, "Rz 60E1-760-1:14"})};
	CHECK(plan.straight >= plan.straight_needed);
	CHECK(plan.spacing_needed == doctest::Approx(4.75));
	CHECK_FALSE(plan.fits);
	std::string reasons;
	for (auto const &reason : plan.reasons) {
		reasons += reason + "; ";
	}
	CAPTURE(reasons);
	CHECK(reasons.find("tab. 11.2") != std::string::npos);
}

TEST_CASE("layout connection: a crossover between curved lines alongside, one curve between the ends turning alike at both") {
	layout::model model;
	// two curves of 1200 m and 1195.5 m turning left by 30 degrees, 4.5 m apart, no cant
	auto const curved = [](double const Offset, double const Radius) {
		// the tangents of the first line moved to their left by the offset
		layout::create_line result;
		auto const turn{3.14159265358979323846 / 6.0};
		glm::dvec2 const out{std::cos(turn), std::sin(turn)};
		glm::dvec2 const left{-std::sin(turn), std::cos(turn)};
		auto const at{glm::dvec2{800.0, 0.0} + left * Offset + out * (Offset * std::tan(turn * 0.5))};
		result.start = {0.0, 100.0, Offset};
		auto const end{glm::dvec2{800.0, 0.0} + out * 800.0 + left * Offset};
		result.end = {end.x, 100.0, end.y};
		layout::vertex vertex;
		vertex.point = at;
		vertex.radius = Radius;
		vertex.transition_in = vertex.transition_out = 80.0;
		result.vertices = {vertex};
		result.category = line_class();
		return result;
	};
	auto const [a, b]{two_lines(model, curved(0.0, 1200.0), curved(4.5, 1195.5))};
	double middle{0.0};
	for (auto const &e : model.state().find(a)->solved.elements) {
		if (e.role == layout::element_role::arc) {
			middle = e.chainage + 0.5 * e.length - 60.0;
		}
	}
	layout::connect_tracks const request{a, middle, 1, b, "Rz 60E1-500-1:12"};
	auto const plan{layout::plan_connection(model.state(), request)};
	std::string reasons;
	for (auto const &reason : plan.reasons) {
		reasons += reason + "; ";
	}
	CAPTURE(reasons);
	REQUIRE(plan.fits);
	MESSAGE("between curves: spacing " << plan.spacing_least << " to " << plan.spacing_most << " m, " << plan.straight << " m between the ends, turning by " << glm::degrees(plan.turn) << " deg");
	CHECK(plan.spacing_least == doctest::Approx(4.5).epsilon(0.01));
	CHECK(std::abs(plan.turn) > 1e-4);
	auto const before{model.state()};
	auto const made{model.execute(request)};
	CAPTURE(reasons_of(made));
	REQUIRE(made.done);
	check_invariants(model.state());
	// both bent with their lines, a curve between their ends
	CHECK(model.state().find(made.turnouts[0])->bent());
	CHECK(model.state().find(made.turnouts[1])->bent());
	CHECK(model.state().find(made.lines.back())->shape.vertices.size() == 1);
	REQUIRE(model.undo());
	CHECK(model.state().same(before));
}

TEST_CASE("layout connection: refused for what isn't a crossover of two ordinary turnouts between two lines alongside") {
	layout::model model;
	auto const [a, b]{two_lines(model, along_x({0.0, 100.0, 0.0}, {1000.0, 100.0, 0.0}), along_x({0.0, 100.0, 4.75}, {1000.0, 100.0, 4.75}))};
	auto const across{model.execute(along_x({500.0, 100.0, -300.0}, {500.0, 100.0, 300.0}))};
	REQUIRE(across.done);
	CHECK_FALSE(layout::plan_connection(model.state(), {a, 300.0, 1, b, "Rkpd 60E1-190-1:9"}).fits);
	CHECK_FALSE(layout::plan_connection(model.state(), {a, 300.0, 1, a, "Rz 60E1-500-1:12"}).fits);
	CHECK_FALSE(layout::plan_connection(model.state(), {a, 300.0, 1, across.lines.front(), "Rz 60E1-500-1:12"}).fits);
	CHECK_FALSE(layout::plan_connection(model.state(), {a, 300.0, 1, b, "Rz 60E1-999-1:99"}).fits);
}

TEST_CASE("layout connection: the element between two turnouts in the norms, ST-T1-A6 11.3 pkt 7: max(0.15 V; 6) m, 30 m at 160 km/h and more") {
	norms::query where;
	where.track = norms::track_kind::main;
	for (auto const &[speed, least] : std::vector<std::pair<double, double>>{{30.0, 6.0}, {40.0, 6.0}, {60.0, 9.0}, {100.0, 15.0}, {130.0, 19.5}}) {
		CAPTURE(speed);
		where.speed = speed;
		auto const limits{norms::limits(norms::profile_id::plk, norms::parameter::switch_straight, where)};
		REQUIRE(limits.warn.has_value());
		CHECK(limits.warn->value == doctest::Approx(least));
		CHECK_FALSE(limits.block.has_value());
		CHECK(limits.warn->from.place == "11.3 pkt 7d");
	}
	// on a line of 160 km/h or more: 30 m, which may not be less
	where.speed = 60.0;
	where.features = norms::feature::fast_line;
	auto const fast{norms::limits(norms::profile_id::plk, norms::parameter::switch_straight, where)};
	REQUIRE(fast.block.has_value());
	CHECK(fast.block->value == doctest::Approx(30.0));
	CHECK(fast.warn->value == doctest::Approx(30.0));
	// a side track: 6 m recommended
	where.features = 0;
	where.track = norms::track_kind::side;
	auto const side{norms::limits(norms::profile_id::plk, norms::parameter::switch_straight, where)};
	REQUIRE(side.prefer.has_value());
	CHECK(side.prefer->value == doctest::Approx(6.0));
	CHECK_FALSE(side.warn.has_value());
	CHECK(norms::minimum(norms::parameter::switch_straight));
}
