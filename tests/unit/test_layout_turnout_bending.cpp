// the bending of the turnouts of the track geometry exact (editor/layout, editor-tracks-domain.md §4 "Rozjazd", §7, stage E5): table 11.8
// for the whole diverging track, every curve of it by the formula of the table with the tangent of the basic switch, as long as keeps its
// turn off the main track; the main track on a transition curve taken as it is, the diverging track bent in steps by its curvature where each
// runs, not by its mean; the curvature of the diverging track of one sign all along it, from its elements

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"

#include <algorithm>
#include <cmath>

namespace {

using namespace test::layout_fixture;

// a line along x into a curve of the radius turning left by 30 degrees, of the transitions given, no cant, a side track at 60 km/h
layout::create_line bend(double const Radius, double const Transition) {
	layout::create_line result;
	result.start = {0.0, 100.0, 0.0};
	auto const turn{3.14159265358979323846 / 6.0};
	glm::dvec2 const corner{800.0, 0.0};
	auto const end{corner + glm::dvec2{std::cos(turn), std::sin(turn)} * 800.0};
	result.end = {end.x, 100.0, end.y};
	layout::vertex curve;
	curve.point = corner;
	curve.radius = Radius;
	curve.cant = 0.0;
	curve.transition_in = curve.transition_out = Transition;
	result.vertices = {curve};
	result.category = line_class();
	result.category.vmax = 60.0;
	result.category.track = norms::track_kind::side;
	return result;
}

double table_11_8(double const Basic, double const Main, double const Tangent) {
	return (Basic + Main) / (1.0 - Basic * Main * Tangent * Tangent);
}

} // namespace

TEST_CASE("layout turnout bending: table 11.8 for the whole diverging track, each curve of it by its formula, as long as keeps its turn") {
	// a diverging track of a curve of another radius, the curve of the basic switch and a transition off it
	layout::turnout_type type;
	type.name = "test";
	type.radius = 500.0;
	type.ratio = 12.0;
	type.parts = {{10.0, 800.0, 800.0}, {20.0, 500.0, 500.0}, {8.0, 500.0, 0.0}};
	auto const tangent{type.radius * std::tan(0.5 * std::atan(1.0 / type.ratio))};
	layout::turnout_frame frame;
	frame.heading = {1.0, 0.0};
	frame.valid = true;
	// on a straight: the parts as they are, to the bit
	auto const straight{layout::lay_diverging(frame, type, 1)};
	REQUIRE(straight.valid());
	REQUIRE(straight.elements.size() == 3);
	CHECK(straight.elements[0].curvature_start == 1.0 / 800.0);
	CHECK(straight.elements[0].length == 10.0);
	CHECK(straight.elements[1].curvature_start == 1.0 / 500.0);
	CHECK(straight.elements[2].length == 8.0);
	for (auto const main : {1.0 / 1200.0, -1.0 / 1200.0, 1.0 / 3000.0}) {
		CAPTURE(main);
		frame.curvature = main;
		auto const laid{layout::lay_diverging(frame, type, 1)};
		REQUIRE(laid.valid());
		REQUIRE(laid.elements.size() == 3);
		// the curve of another radius and the transition by the formula of the table, the curve of the basic switch by the table itself
		CHECK(laid.elements[0].curvature_start == doctest::Approx(table_11_8(1.0 / 800.0, main, tangent)).epsilon(1e-12));
		CHECK(laid.elements[1].curvature_start == doctest::Approx(1.0 / switch_curve::bent_radius(layout::bending_of(main, 1), 500.0, 12.0)).epsilon(1e-12));
		CHECK(laid.elements[1].curvature_start == doctest::Approx(table_11_8(1.0 / 500.0, main, tangent)).epsilon(1e-9));
		CHECK(laid.elements[2].curvature_start == doctest::Approx(table_11_8(1.0 / 500.0, main, tangent)).epsilon(1e-12));
		CHECK(laid.elements[2].curvature_end == doctest::Approx(main).epsilon(1e-12));
		// each turning off the main track by as much as in the basic switch: the angle of the frog kept
		std::array<double, 3> const turns{10.0 / 800.0, 20.0 / 500.0, 0.5 * 8.0 / 500.0};
		for (std::size_t i = 0; i < 3; ++i) {
			auto const &e{laid.elements[i]};
			CHECK((0.5 * (e.curvature_start + e.curvature_end) - main) * e.length == doctest::Approx(turns[i]).epsilon(1e-12));
		}
		// not the plain sum of the curvatures, which E2b took for the curves of other radii
		CHECK(std::abs(laid.elements[0].curvature_start - (1.0 / 800.0 + main)) > 1e-8);
	}
}

TEST_CASE("layout turnout bending: the main track on a transition curve taken as it is, the diverging track bent where each step of it runs") {
	layout::model model;
	REQUIRE(model.execute(bend(1200.0, 200.0)).done);
	auto const line{model.state().lines().begin()->first};
	double transition{0.0};
	for (auto const &e : model.state().find(line)->solved.elements) {
		if (e.role == layout::element_role::transition_in) {
			transition = e.chainage;
		}
	}
	// a 1:12 starting 60 m into the transition, toward the curve: the main track from R 4000 m to R 3000 m and on under it
	auto const made{model.execute(layout::insert_turnout{line, transition + 60.0, "Rz 60E1-500-1:12", 1, 1})};
	CAPTURE(reasons_of(made));
	REQUIRE(made.done);
	auto const &state{model.state()};
	check_invariants(state);
	auto const &turnout{*state.find(made.turnouts.front())};
	auto const frame{layout::frame_of(state, turnout)};
	CHECK(frame.varying);
	auto const &main{state.find(turnout.main)->solved};
	auto const &diverging{state.find(turnout.diverging)->solved};
	// in steps, its curvature growing as the main track's does: at its start by the table with the curvature of the main track there
	CHECK(diverging.elements.size() >= 10);
	auto const t{500.0 * std::tan(0.5 * std::atan(1.0 / 12.0))};
	CHECK(diverging.elements.front().curvature_start == doctest::Approx(table_11_8(1.0 / 500.0, main.curvature(0.0), t)).epsilon(1e-9));
	CHECK(diverging.elements.back().curvature_end > diverging.elements.front().curvature_start + 1e-7);
	// the angle of the frog kept against the main track where the diverging track ends beside it
	auto const end{geometry::plan_of(diverging.end.position)};
	double station{0.0};
	for (int i = 0; i < 40; ++i) {
		station = std::clamp(station + glm::dot(end - main.position(station), glm::normalize(main.tangent(station))), 0.0, main.length);
	}
	auto const frog{geometry::signed_angle(glm::normalize(main.tangent(station)), diverging.end.heading)};
	// the mean curvature of the main track, as E2b took it, laid off the same start
	auto mean{frame};
	mean.varying = false;
	auto const old{layout::lay_diverging(mean, *layout::turnout_type_of(turnout.type), turnout.hand)};
	auto const old_frog{geometry::signed_angle(glm::normalize(main.tangent(station)), old.end.heading)};
	MESSAGE("1:12 on a transition: the frog at 1:" << 1.0 / std::tan(frog) << " exactly, at 1:" << 1.0 / std::tan(old_frog) << " by the mean curvature; the end "
	                                               << glm::distance(geometry::plan_of(old.end.position), end) * 1000.0 << " mm apart");
	CHECK(frog == doctest::Approx(std::atan(1.0 / 12.0)).epsilon(2e-4));
	CHECK(std::abs(frog - std::atan(1.0 / 12.0)) < std::abs(old_frog - std::atan(1.0 / 12.0)));
	// moved along the transition and back: laid again the same
	auto const before{state};
	REQUIRE(model.execute(layout::move_turnout{turnout.id, 20.0}).done);
	check_invariants(model.state());
	REQUIRE(model.undo());
	CHECK(model.state().same(before));
}

TEST_CASE("layout turnout bending: the curvature of the diverging track of one sign all along it, from its elements (KZW II refused)") {
	layout::model model;
	auto slow{bend(400.0, 60.0)};
	slow.category.vmax = 40.0;
	REQUIRE(model.execute(slow).done);
	auto const line{model.state().lines().begin()->first};
	double middle{0.0};
	for (auto const &e : model.state().find(line)->solved.elements) {
		if (e.role == layout::element_role::arc) {
			middle = e.chainage + 0.5 * e.length;
		}
	}
	auto const before{model.state()};
	// Rz 60E1-190-1:9 bent two-sided: its curve still turns off to its side, the straight through its frog goes with the main track the other way
	auto const refused{model.execute(layout::insert_turnout{line, middle, "Rz 60E1-190-1:9", -1, 1})};
	CHECK_FALSE(refused.done);
	CHECK(reasons_of(refused).find("changes its sign within the turnout (KZW II)") != std::string::npos);
	CHECK(model.state().same(before));
	// bent one-sided it turns one way all along it
	auto const laid{model.execute(layout::insert_turnout{line, middle, "Rz 60E1-190-1:9", 1, 1})};
	CAPTURE(reasons_of(laid));
	CHECK(reasons_of(laid).find("changes its sign") == std::string::npos);
}
