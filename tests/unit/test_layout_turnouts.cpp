// the turnouts of the track geometry (editor/layout, editor-tracks-domain.md §4 "Rozjazd", §7, §8): the types of the catalogue the
// model lays, the legs of a turnout against the catalogue (a, b, the angle), the bending of table 11.8 and the speeds of Q2 as the model
// computes them, the rules of Q2 and §11.4 refusing, the commands laying, moving, changing and taking out a turnout kept whole or refused
// with the state as it was, one step to undo; the edits E2a left (a vertex put in and taken out, the grade line, the class and the profile
// of a line, a line going on from the curve of a free end) and the edits which may add no BLOCK finding to an old track; random edits with
// turnouts, the seed in the name of the case

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"

#include <algorithm>
#include <map>
#include <random>
#include <vector>

namespace {

using namespace test::layout_fixture;
using layout::line_id;
using layout::node_id;
using layout::turnout_id;

double constexpr kPi{3.14159265358979323846};

// a straight line along x, 1 km long, at 80 km/h
layout::create_line straight_line(glm::dvec3 const &Start = {0.0, 100.0, 0.0}, double const Length = 1000.0) {
	layout::create_line result;
	result.start = Start;
	result.end = Start + glm::dvec3{Length, 0.0, 0.0};
	result.category = line_class();
	return result;
}

// a line along x into one curve turning left by 30 degrees, of the radius and the cant given, the transitions 70 m long
layout::create_line one_curve(double const Radius, double const Cant, norms::track_kind const Track = norms::track_kind::main) {
	layout::create_line result;
	result.start = {0.0, 100.0, 0.0};
	auto const turn{kPi / 6.0};
	glm::dvec2 const corner{700.0, 0.0};
	auto const end{corner + glm::dvec2{std::cos(turn), std::sin(turn)} * 700.0};
	result.end = {end.x, 100.0, end.y};
	layout::vertex curve;
	curve.point = corner;
	curve.radius = Radius;
	curve.cant = Cant;
	curve.transition_in = curve.transition_out = 70.0;
	result.vertices = {curve};
	result.category = line_class();
	result.category.track = Track;
	return result;
}

line_id only_line(layout::model const &Model) {
	REQUIRE(Model.state().lines().size() == 1);
	return Model.state().lines().begin()->first;
}

// the chainage of the middle of the arc of the line
double middle_of_arc(layout::line const &Line) {
	for (auto const &e : Line.solved.elements) {
		if (e.role == layout::element_role::arc) {
			return e.chainage + 0.5 * e.length;
		}
	}
	FAIL("no arc");
	return 0.0;
}

layout::insert_turnout turnout_into(line_id const Line, double const Chainage, std::string const &Type, int const Hand = 1, int const Direction = 1) {
	return {Line, Chainage, Type, Hand, Direction};
}

layout::turnout const &turnout_of(layout::model const &Model, layout::outcome const &Made) {
	REQUIRE(Made.turnouts.size() == 1);
	auto const *found{Model.state().find(Made.turnouts.front())};
	REQUIRE(found != nullptr);
	return *found;
}

bool says(std::vector<std::string> const &Reasons, std::string const &What) {
	return std::ranges::any_of(Reasons, [&](std::string const &Reason) { return Reason.find(What) != std::string::npos; });
}

} // namespace

TEST_CASE("layout turnouts: the ordinary switches of the catalogue are laid, double, symmetric and secant ones refused with the reason") {
	auto const names{layout::turnout_types()};
	for (auto const *name : {"Rz 60E1-190-1:9", "Rz 60E1-300-1:9", "Rz 60E1-500-1:12", "Rz 60E1-760-1:14", "Rz 60E1-1200-1:18,5", "Rz 60E1-2500-1:26,5 movable nose", "Rz 49E1-500-1:14"}) {
		CAPTURE(name);
		CHECK(std::ranges::find(names, name) != names.end());
	}
	auto const type{layout::turnout_type_of("Rz 60E1-500-1:12")};
	REQUIRE(type.has_value());
	CHECK(type->radius == doctest::Approx(500.0));
	CHECK(type->ratio == doctest::Approx(12.0).epsilon(1e-9));
	CHECK(type->a == doctest::Approx(20.797));
	CHECK(type->b == doctest::Approx(20.797));
	CHECK(type->length == doctest::Approx(41.594));
	CHECK_FALSE(type->movable);
	// the diverging track: the arc of the angle 1:12, a = R tan(a/2) to its centre
	CHECK(type->diverging_length() == doctest::Approx(500.0 * std::atan(1.0 / 12.0)));
	CHECK(layout::turnout_type_of("Rz 60E1-2500-1:26,5 movable nose")->movable);
	std::string why;
	CHECK_FALSE(layout::turnout_type_of("Rls 49E1-215-1:4,8", &why).has_value());
	CHECK(why.find("symmetric") != std::string::npos);
	CHECK_FALSE(layout::turnout_type_of("Rpd 49E1-190-1:9/1:9", &why).has_value());
	CHECK(why.find("double") != std::string::npos);
	CHECK_FALSE(layout::turnout_type_of("Rz S42-190-1:7", &why).has_value());
	CHECK(why.find("angle") != std::string::npos);
	CHECK_FALSE(layout::turnout_type_of("Rz 60E1-999-1:99", &why).has_value());
}

TEST_CASE("layout turnouts: no diverging track is laid from a main track that isn't laid") {
	auto const type{layout::turnout_type_of("Rz 60E1-300-1:9")};
	REQUIRE(type.has_value());
	// a frame left as it is when the main track isn't solved: at the origin, along x
	layout::turnout_frame frame;
	frame.heading = {1.0, 0.0};
	REQUIRE_FALSE(frame.valid);
	auto const laid{layout::lay_diverging(frame, *type, 1)};
	CHECK_FALSE(laid.valid());
	REQUIRE_FALSE(laid.errors.empty());
	CHECK(laid.errors.front().find("main track isn't laid") != std::string::npos);
}

TEST_CASE("layout turnouts: the legs on a straight against the catalogue: the main track a + b, the centre at a, the diverging end at 1:n") {
	for (auto const *name : {"Rz 60E1-190-1:9", "Rz 60E1-500-1:12", "Rz 60E1-1200-1:18,5", "Rz 49E1-500-1:14"}) {
		for (auto const hand : {1, -1}) {
			for (auto const direction : {1, -1}) {
				CAPTURE(name);
				CAPTURE(hand);
				CAPTURE(direction);
				layout::model model;
				REQUIRE(model.execute(straight_line()).done);
				auto const line{only_line(model)};
				auto const made{model.execute(turnout_into(line, 400.0, name, hand, direction))};
				CAPTURE(reasons_of(made));
				REQUIRE(made.done);
				check_invariants(model.state());
				auto const &turnout{turnout_of(model, made)};
				auto const type{layout::turnout_type_of(name)};
				auto const &state{model.state()};
				auto const start{geometry::plan_of(state.find(turnout.start)->position)};
				auto const main_end{geometry::plan_of(state.find(turnout.main_end)->position)};
				auto const end{geometry::plan_of(state.find(turnout.diverging_end)->position)};
				glm::dvec2 const along{static_cast<double>(direction), 0.0};
				// the start where it was put, the main track a + b along the line
				CHECK(glm::distance(start, glm::dvec2{400.0, 0.0}) < 1e-9);
				CHECK(glm::distance(main_end, start + along * type->length) < 1e-9);
				// the end of the diverging track at the angle of the type, its tangent meeting the main track at the centre, a from the start
				auto const *diverging{state.find(turnout.diverging)};
				auto const heading{diverging->solved.end.heading};
				CHECK(geometry::signed_angle(along, heading) == doctest::Approx(hand * std::atan(1.0 / type->ratio)).epsilon(1e-9));
				double first{0.0};
				double second{0.0};
				REQUIRE(geometry::lines_cross(start, along, end, heading, first, second));
				auto const radius{type->radius};
				auto const arc_end{radius * std::tan(0.5 * std::atan(1.0 / type->ratio))};
				CHECK(first == doctest::Approx(arc_end).epsilon(1e-9));
				// Id-4 gives a to the millimetre
				CHECK(std::abs(first - type->a) < 0.0005);
				// to the hand's side, as far from the centre as the catalogue has the diverging track go on
				CHECK(geometry::cross(along, end - start) * hand > 0.0);
				CHECK(-second == doctest::Approx(arc_end + (type->diverging_length() - radius * std::atan(1.0 / type->ratio))).epsilon(1e-9));
				// straight: the speeds of table 11.2 and the line's on the main track, the diverging one written below 0
				CHECK(turnout.main_radius == 0.0);
				CHECK(turnout.speed_diverging == switch_curve::basic_speed(radius));
				CHECK(turnout.speed_main == 80.0);
				CHECK(turnout.velocity == (turnout.speed_diverging < 80.0 ? -turnout.speed_diverging : 80.0));
				// the nodes: three ends at the start, two at the end of the main track, one at the free end of the diverging track
				CHECK(state.find(turnout.start)->ends.size() == 3);
				CHECK(state.find(turnout.main_end)->ends.size() == 2);
				CHECK(state.find(turnout.diverging_end)->ends.size() == 1);
				CHECK(turnout.chainage == doctest::Approx(400.0));
			}
		}
	}
}

TEST_CASE("layout turnouts: bent into a curve by table 11.8, the angle of the frog kept, the speeds of Q2 with the cant with its sign") {
	struct row {
		char const *type;
		double radius;
		double cant;
		int hand;
		double bent; // the main radius as table 11.8 takes it
		double diverging; // r"
		double speed;
		double velocity;
	};
	// Rz 60E1-500-1:12 in R 1200 m with 60 mm: outward 857.8 m, 60 km/h; inward 352.7 m, 60 by table 11.2
	row const rows[]{{"Rz 60E1-500-1:12", 1200.0, 60.0, -1, -1200.0, 857.76, 60.0, -60.0}, {"Rz 60E1-500-1:12", 1200.0, 60.0, 1, 1200.0, 352.69, 60.0, -60.0}};
	for (auto const &each : rows) {
		CAPTURE(each.hand);
		layout::model model;
		auto const made_line{model.execute(one_curve(each.radius, each.cant))};
		CAPTURE(reasons_of(made_line));
		REQUIRE(made_line.done);
		auto const line{only_line(model)};
		auto const made{model.execute(turnout_into(line, middle_of_arc(*model.state().find(line)), each.type, each.hand))};
		CAPTURE(reasons_of(made));
		REQUIRE(made.done);
		check_invariants(model.state());
		auto const &turnout{turnout_of(model, made)};
		CHECK(turnout.main_radius == doctest::Approx(each.bent).epsilon(1e-9));
		CHECK(turnout.diverging_radius == doctest::Approx(each.diverging).epsilon(1e-5));
		CHECK(turnout.cant == doctest::Approx(each.hand * each.cant));
		CHECK(turnout.speed_diverging == each.speed);
		CHECK(turnout.speed_main == 80.0);
		CHECK(turnout.velocity == each.velocity);
		// the diverging curve of r", as long as keeps the angle of the frog off the main track: 1:12 against the main track as far along
		auto const &state{model.state()};
		auto const &diverging{state.find(turnout.diverging)->solved};
		REQUIRE(diverging.elements.size() == 1);
		CHECK(std::abs(diverging.elements.front().curvature_start) == doctest::Approx(1.0 / each.diverging).epsilon(1e-5));
		auto const &main{state.find(turnout.main)->solved};
		auto const off{geometry::signed_angle(main.tangent(diverging.length), diverging.end.heading)};
		CHECK(off == doctest::Approx(each.hand * std::atan(1.0 / 12.0)).epsilon(1e-9));
		// the cant of the line over all of it
		CHECK(diverging.cant_at(0.0) == doctest::Approx(each.cant));
		CHECK(diverging.cant_at(diverging.length) == doctest::Approx(each.cant));
	}
}

TEST_CASE("layout turnouts: the rules of Q2 and 11.4 refuse: the curvature turning over, a ramp within or near, a steep switch bent in a main track") {
	// Rz 60E1-190-1:9 bent outward into R 600 m: its diverging curve turns away, the straight through its frog with the curve (KZW II)
	{
		layout::model model;
		REQUIRE(model.execute(one_curve(600.0, 60.0)).done);
		auto const line{only_line(model)};
		auto const before{model.state()};
		auto const refused{model.execute(turnout_into(line, middle_of_arc(*model.state().find(line)), "Rz 60E1-190-1:9", -1))};
		CHECK_FALSE(refused.done);
		CHECK(says(refused.reasons, "KZW II"));
		CHECK(model.state().same(before));
	}
	// a ramp under the turnout, an end of one 3 m off it; right at the end of the ramp it's laid
	{
		layout::model model;
		REQUIRE(model.execute(one_curve(1200.0, 60.0)).done);
		auto const line{only_line(model)};
		auto const before{model.state()};
		auto const &solved{model.state().find(line)->solved};
		double arc_start{0.0};
		for (auto const &e : solved.elements) {
			if (e.role == layout::element_role::arc) {
				arc_start = e.chainage;
			}
		}
		auto const under{model.execute(turnout_into(line, arc_start - 20.0, "Rz 60E1-500-1:12", 1))};
		CHECK_FALSE(under.done);
		CHECK(says(under.reasons, "11.4 pkt 7"));
		auto const close{model.execute(turnout_into(line, arc_start + 3.0, "Rz 60E1-500-1:12", 1))};
		CHECK_FALSE(close.done);
		CHECK(says(close.reasons, "6 m at least"));
		CHECK(model.state().same(before));
		auto const at{model.execute(turnout_into(line, arc_start + 1e-7, "Rz 60E1-500-1:12", 1))};
		CAPTURE(reasons_of(at));
		CHECK(at.done);
	}
	// Rz 49E1-190-1:7,5 bent outward into R 600 m: refused in a main track, laid in a side track
	{
		layout::model model;
		REQUIRE(model.execute(one_curve(600.0, 60.0)).done);
		auto const line{only_line(model)};
		auto const refused{model.execute(turnout_into(line, middle_of_arc(*model.state().find(line)), "Rz 49E1-190-1:7,5", -1))};
		CHECK_FALSE(refused.done);
		CHECK(says(refused.reasons, "11.4 pkt 4"));
		layout::model side;
		auto const side_line{side.execute(one_curve(600.0, 60.0, norms::track_kind::side))};
		CAPTURE(reasons_of(side_line));
		REQUIRE(side_line.done);
		auto const id{only_line(side)};
		auto const laid{side.execute(turnout_into(id, middle_of_arc(*side.state().find(id)), "Rz 49E1-190-1:7,5", -1))};
		CAPTURE(reasons_of(laid));
		CHECK(laid.done);
		check_invariants(side.state());
	}
	// a type not laid, a hand of none, a turnout longer than the line has room for
	layout::model model;
	REQUIRE(model.execute(straight_line()).done);
	auto const line{only_line(model)};
	auto const before{model.state()};
	CHECK_FALSE(model.execute(turnout_into(line, 300.0, "Rls 49E1-215-1:4,8")).done);
	CHECK_FALSE(model.execute(turnout_into(line, 300.0, "Rz 60E1-500-1:12", 0)).done);
	CHECK_FALSE(model.execute(turnout_into(line, 980.0, "Rz 60E1-500-1:12")).done);
	CHECK_FALSE(model.execute(turnout_into(line, 20.0, "Rz 60E1-500-1:12", 1, -1)).done);
	CHECK(model.state().same(before));
	CHECK(model.undoable() == 1);
}

TEST_CASE("layout turnouts: moved along its line, changed, taken out with the line laid again as it was; its tracks edited only through it") {
	for (auto const curved : {false, true}) {
		CAPTURE(curved);
		layout::model model;
		REQUIRE(model.execute(curved ? one_curve(1200.0, 60.0) : straight_line()).done);
		auto const line{only_line(model)};
		auto const whole{model.state()};
		auto const at{curved ? middle_of_arc(*model.state().find(line)) - 60.0 : 400.0};
		auto const made{model.execute(turnout_into(line, at, "Rz 60E1-500-1:12", 1))};
		CAPTURE(reasons_of(made));
		REQUIRE(made.done);
		auto const id{made.turnouts.front()};
		auto const laid{model.state()};
		auto const turnout{*laid.find(id)};
		// its tracks aren't edited as lines
		CHECK_FALSE(model.execute(layout::split_line{turnout.main, 10.0}).done);
		CHECK_FALSE(model.execute(layout::delete_line{turnout.diverging}).done);
		CHECK_FALSE(model.execute(layout::set_grades{turnout.main, {}}).done);
		CHECK(model.state().same(laid));
		// moved 25 m on: the start 25 m further, its ids kept, the lines on both sides one step to undo
		auto const moved{model.execute(layout::move_turnout{id, 25.0})};
		CAPTURE(reasons_of(moved));
		REQUIRE(moved.done);
		check_invariants(model.state());
		auto const &now{*model.state().find(id)};
		CHECK(now.chainage == doctest::Approx(at + 25.0).epsilon(1e-9));
		CHECK(now.start == turnout.start);
		CHECK(now.main == turnout.main);
		CHECK(now.diverging_end == turnout.diverging_end);
		CHECK(model.state().find(line)->solved.length == doctest::Approx(at + 25.0).epsilon(1e-9));
		CHECK(model.state().lines().size() == 4);
		REQUIRE(model.undo());
		CHECK(model.state().same(laid));
		REQUIRE(model.redo());
		// changed to a 1:14 of 760 m, its start where it was, its main track as long as the new type
		auto const changed{model.execute(layout::change_turnout{id, "Rz 60E1-760-1:14"})};
		CAPTURE(reasons_of(changed));
		REQUIRE(changed.done);
		check_invariants(model.state());
		CHECK(model.state().find(id)->chainage == doctest::Approx(at + 25.0).epsilon(1e-9));
		CHECK(model.state().find(id)->speed_diverging == 80.0);
		CHECK(model.state().find(model.state().find(id)->main)->solved.length == doctest::Approx(layout::turnout_type_of("Rz 60E1-760-1:14")->length));
		// and to the other hand
		auto const turned{model.execute(layout::change_turnout{id, {}, -1})};
		CAPTURE(reasons_of(turned));
		REQUIRE(turned.done);
		CHECK(model.state().find(id)->hand == -1);
		check_invariants(model.state());
		// taken out: the line one again, the track as it was before the turnout
		auto const removed{model.execute(layout::remove_turnout{id})};
		CAPTURE(reasons_of(removed));
		REQUIRE(removed.done);
		check_invariants(model.state());
		CHECK(model.state().turnouts().empty());
		REQUIRE(model.state().lines().size() == 1);
		auto const &again{model.state().lines().begin()->second.solved};
		auto const &original{whole.find(line)->solved};
		CHECK(again.length == doctest::Approx(original.length).epsilon(1e-12));
		for (int i{0}; i * 3.7 <= original.length; ++i) {
			auto const s{i * 3.7};
			CAPTURE(s);
			CHECK(glm::distance(again.point(s), original.point(s)) < 1e-6);
			CHECK(std::abs(again.cant_at(s) - original.cant_at(s)) < 1e-6);
		}
		// each a step to undo, back to the line with no turnout
		while (model.undoable() > 1) {
			REQUIRE(model.undo());
		}
		CHECK(model.state().same(whole));
		// moved off its line, or into the next turnout: refused
		REQUIRE(model.redo());
		CHECK_FALSE(model.execute(layout::move_turnout{id, 2000.0}).done);
		CHECK(model.state().same(laid));
	}
}

TEST_CASE("layout turnouts: a line going on from the end of the diverging track is carried with the turnout moved") {
	layout::model model;
	REQUIRE(model.execute(straight_line()).done);
	auto const main_line{only_line(model)};
	auto const made{model.execute(turnout_into(main_line, 200.0, "Rz 60E1-500-1:12", 1))};
	REQUIRE(made.done);
	auto const id{made.turnouts.front()};
	// a line along x 30 m to the left, from 800 m on, its free start facing back: joined to the end of the diverging track by one curve
	auto parallel{straight_line({800.0, 100.0, 30.0}, 300.0)};
	auto const other{model.execute(parallel)};
	REQUIRE(other.done);
	auto const free_start{model.state().find(other.lines.front())->start};
	auto const joined{model.execute(layout::join_ends{model.state().find(id)->diverging_end, free_start})};
	CAPTURE(reasons_of(joined));
	REQUIRE(joined.done);
	check_invariants(model.state());
	auto const link{joined.lines.front()};
	auto const before{model.state()};
	auto const moved{model.execute(layout::move_turnout{id, 10.0})};
	CAPTURE(reasons_of(moved));
	REQUIRE(moved.done);
	check_invariants(model.state());
	// the line starts at the end of the diverging track where it is now, its curve moved along with it
	auto const &state{model.state()};
	auto const &end{*state.find(state.find(id)->diverging_end)};
	CHECK(state.find(link)->solved.start.position == end.position);
	CHECK(state.find(link)->shape.vertices.front().point.x == doctest::Approx(before.find(link)->shape.vertices.front().point.x + 10.0));
	CHECK(state.find(link)->shape.vertices.front().point.y == doctest::Approx(before.find(link)->shape.vertices.front().point.y));
	REQUIRE(model.undo());
	CHECK(model.state().same(before));
}

TEST_CASE("layout commands: a vertex put in and taken out, the grade line, the class and the profile of a line") {
	layout::model model;
	REQUIRE(model.execute(straight_line()).done);
	auto const line{only_line(model)};
	auto const plain{model.state()};
	// a vertex off the line with a curve of the radius the norms prefer: the line turns there; taken out, the straight again
	auto bend{curve({500.0, 20.0}, 1500.0)};
	auto const put{model.execute(layout::insert_vertex{line, layout::vertex_id::none, bend})};
	CAPTURE(reasons_of(put));
	REQUIRE(put.done);
	REQUIRE(put.vertices.size() == 1);
	check_invariants(model.state());
	CHECK(model.state().find(line)->shape.vertices.size() == 1);
	CHECK(model.state().find(line)->solved.elements.size() == 5);
	// another after it, then the first one taken out
	auto const second{model.execute(layout::insert_vertex{line, put.vertices.front(), curve({800.0, 0.0}, 1500.0)})};
	CAPTURE(reasons_of(second));
	REQUIRE(second.done);
	CHECK(model.state().find(line)->shape.vertices.back().id == second.vertices.front());
	REQUIRE(model.execute(layout::remove_vertex{line, second.vertices.front()}).done);
	REQUIRE(model.execute(layout::remove_vertex{line, put.vertices.front()}).done);
	CHECK(model.state().same(plain));
	CHECK_FALSE(model.execute(layout::remove_vertex{line, put.vertices.front()}).done);
	CHECK_FALSE(model.execute(layout::insert_vertex{line, layout::vertex_id{999}, bend}).done);
	// the grade line: a break with its vertical curve; two overlapping, a break of no radius refused
	REQUIRE(model.execute(layout::set_grades{line, {{400.0, 103.0, 10000.0}}}).done);
	check_invariants(model.state());
	CHECK(model.state().find(line)->solved.elevation(400.0) < 103.0);
	auto const graded{model.state()};
	CHECK_FALSE(model.execute(layout::set_grades{line, {{400.0, 103.0, 10000.0}, {420.0, 100.0, 10000.0}}}).done);
	CHECK_FALSE(model.execute(layout::set_grades{line, {{400.0, 103.0, 0.0}}}).done);
	CHECK(model.state().same(graded));
	// the class: a curve fine at 80 km/h passes a BLOCK limit at 200; the profile of the line's own judges it
	layout::model curved;
	REQUIRE(curved.execute(three_curves()).done);
	auto const id{only_line(curved)};
	auto fast{line_class()};
	fast.vmax = 200.0;
	auto const before{curved.state()};
	auto const refused{curved.execute(layout::set_line_class{id, fast})};
	CHECK_FALSE(refused.done);
	CHECK(curved.state().same(before));
	auto slow{line_class()};
	slow.vmax = 60.0;
	REQUIRE(curved.execute(layout::set_line_class{id, slow}).done);
	CHECK(curved.state().find(id)->category.vmax == 60.0);
	// R 280 m at 80 km/h warns by PLK (P1 300 m), by the TSI alone it doesn't
	REQUIRE(curved.execute(layout::set_line_class{id, line_class()}).done);
	auto const tight{layout::preferred_curve(norms::profile_id::plk, line_class(), 280.0)};
	auto const v2{curved.state().find(id)->shape.vertices[1].id};
	auto const warned{curved.execute(layout::set_vertex{id, v2, 280.0, tight.transition, tight.transition, tight.cant})};
	REQUIRE(warned.done);
	auto const radius_warning = [](layout::outcome const &Outcome) {
		return std::ranges::any_of(Outcome.notices, [](layout::notice const &Notice) { return Notice.finding.what == norms::parameter::radius && Notice.finding.level == norms::tier::warn; });
	};
	CHECK(radius_warning(warned));
	auto const tsi{curved.execute(layout::set_line_profile{id, norms::profile_id::tsi})};
	REQUIRE(tsi.done);
	CHECK(curved.state().find(id)->profile == norms::profile_id::tsi);
	CHECK_FALSE(radius_warning(tsi));
	REQUIRE(curved.execute(layout::set_line_profile{id, std::nullopt}).done);
	CHECK_FALSE(curved.state().find(id)->profile.has_value());
}

TEST_CASE("layout commands: a line joined to a free end in a curve goes on from its curve, the track as it was") {
	layout::model model;
	REQUIRE(model.execute(one_curve(800.0, 50.0)).done);
	auto const line{only_line(model)};
	auto const original{model.state().find(line)->solved};
	auto const &vertex{model.state().find(line)->shape.vertices.front()};
	auto const transition{vertex.transition_out};
	// cut in the middle of the arc, the part after it taken away: a free end in the curve
	auto const at{middle_of_arc(*model.state().find(line))};
	auto const cut{model.execute(layout::split_line{line, at})};
	REQUIRE(cut.done);
	auto const end_point{model.state().find(cut.lines.front())->solved.end.position};
	REQUIRE(model.execute(layout::delete_line{cut.lines.front()}).done);
	auto const free_curve{cut.nodes.front()};
	CHECK(model.state().find(free_curve)->curvature != 0.0);
	// a line on along the tangent the curve ended on, its start free
	auto const out{original.end.heading};
	layout::create_line on;
	on.start = end_point;
	on.end = end_point + glm::dvec3{out.x, 0.0, out.y} * 200.0;
	on.category = line_class();
	auto const beyond{model.execute(on)};
	REQUIRE(beyond.done);
	auto const joined{model.execute(layout::join_ends{free_curve, model.state().find(beyond.lines.front())->start})};
	CAPTURE(reasons_of(joined));
	REQUIRE(joined.done);
	check_invariants(model.state());
	// the arc going on, the transition out as long as it was: the track the part taken away had
	auto const &link{*model.state().find(joined.lines.front())};
	CHECK(link.start_continues);
	REQUIRE(link.shape.vertices.size() == 1);
	CHECK(link.shape.vertices.front().radius == doctest::Approx(800.0));
	CHECK(link.shape.vertices.front().transition_out == doctest::Approx(transition).epsilon(1e-6));
	for (int i{0}; i * 5.0 <= link.solved.length; ++i) {
		auto const s{i * 5.0};
		CAPTURE(s);
		CHECK(glm::distance(link.solved.point(s), original.point(at + s)) < 1e-6);
	}
	// both ends in curves: not yet
	layout::model two;
	REQUIRE(two.execute(three_curves()).done);
	auto const id{only_line(two)};
	auto const &curves{*two.state().find(id)};
	auto const first{two.execute(layout::split_line{id, middle_of_arc(curves)})};
	REQUIRE(first.done);
	double second_arc{0.0};
	for (auto const &e : two.state().find(first.lines.front())->solved.elements) {
		if (e.role == layout::element_role::arc) {
			second_arc = e.chainage + 0.5 * e.length;
			break;
		}
	}
	auto const second{two.execute(layout::split_line{first.lines.front(), second_arc})};
	REQUIRE(second.done);
	auto const a{two.state().find(first.lines.front())->start};
	auto const b{two.state().find(first.lines.front())->end};
	REQUIRE(two.execute(layout::delete_line{first.lines.front()}).done);
	auto const both{two.execute(layout::join_ends{a, b})};
	CHECK_FALSE(both.done);
	CHECK(says(both.reasons, "both lie in curves"));
}

TEST_CASE("layout commands: an old track breaking the norms is edited with no BLOCK added, the BLOCK it had not refusing") {
	// R 170 m at 40 km/h laid by the TSI alone (150 m), then judged by PLK: below its 180 m, a BLOCK there before any edit
	norms::area_map lenient;
	lenient.scenery = norms::profile_id::tsi;
	layout::model laid{lenient};
	auto line{one_curve(170.0, 60.0)};
	line.category.vmax = 40.0;
	line.vertices.front().transition_in = line.vertices.front().transition_out = 40.0;
	auto const made{laid.execute(line)};
	CAPTURE(reasons_of(made));
	REQUIRE(made.done);
	auto old{laid.state()};
	old.areas.scenery = norms::profile_id::plk;
	auto const existing{layout::check_all(old)};
	REQUIRE_FALSE(existing.blocks.empty());
	CHECK(std::ranges::any_of(existing.blocks, [](layout::notice const &Block) { return Block.finding.what == norms::parameter::radius; }));
	layout::model model{old};
	auto const id{only_line(model)};
	auto const vertex{model.state().find(id)->shape.vertices.front().id};
	auto const before{model.state()};
	// refused when any BLOCK refuses, kept when it isn't one the edit adds
	CHECK_FALSE(model.execute(layout::set_vertex{id, vertex, std::nullopt, std::nullopt, std::nullopt, 55.0}).done);
	CHECK(model.state().same(before));
	auto const kept{model.execute(layout::set_vertex{id, vertex, std::nullopt, std::nullopt, std::nullopt, 55.0}, layout::edit_policy::no_new_block)};
	CAPTURE(reasons_of(kept));
	CHECK(kept.done);
	// a sharper curve makes it worse, a cant past (R - 50) / 1.5 adds one: refused; a wider one makes it better: kept
	auto const now{model.state()};
	CHECK_FALSE(model.execute(layout::set_vertex{id, vertex, 160.0}, layout::edit_policy::no_new_block).done);
	auto const steep{model.execute(layout::set_vertex{id, vertex, std::nullopt, std::nullopt, std::nullopt, 100.0}, layout::edit_policy::no_new_block)};
	CHECK_FALSE(steep.done);
	CHECK(says(steep.reasons, "cant"));
	CHECK(model.state().same(now));
	auto const wider{model.execute(layout::set_vertex{id, vertex, 175.0}, layout::edit_policy::no_new_block)};
	CAPTURE(reasons_of(wider));
	CHECK(wider.done);
	// the invariants refuse in any case
	CHECK_FALSE(model.execute(layout::move_vertex{id, vertex, {0.0, 0.0}}, layout::edit_policy::no_new_block).done);
}

namespace {

std::vector<std::string> const kTypes{"Rz 60E1-190-1:9", "Rz 60E1-300-1:9", "Rz 60E1-500-1:12", "Rz 60E1-760-1:14", "Rz 60E1-1200-1:18,5", "Rz 49E1-500-1:14"};

// random edits with turnouts: laid, moved, changed and taken out, with the edits of the lines (vertices moved and set, put in and taken
// out, splits, joins, deletions), undoing and redoing; after each the invariants, a refusal changing nothing, an undo giving the state
// before. Policy: of the norms; with no_new_block the network starts from one breaking them and no BLOCK finding is added
void random_turnout_edits(unsigned const Seed, int const Count, layout::edit_policy const Policy = layout::edit_policy::refuse_block) {
	std::mt19937 random{Seed}; // NOSONAR: seeded for repeatable tests, nothing secret is drawn
	auto const uniform = [&](double const Low, double const High) { return std::uniform_real_distribution<double>{Low, High}(random); };
	auto const pick = [&](std::size_t const Size) { return std::uniform_int_distribution<std::size_t>{0, Size - 1}(random); };
	layout::model model;
	std::vector<layout::network> history;
	REQUIRE(model.execute(three_curves()).done);
	REQUIRE(model.execute(straight_line({0.0, 100.0, -300.0}, 1200.0)).done);
	if (Policy == layout::edit_policy::no_new_block) {
		// the lines judged at 120 km/h, which they weren't laid for: an old track breaking the norms in its curves
		auto old{model.state()};
		layout::transaction faster{old};
		auto fast{line_class()};
		fast.vmax = 120.0;
		for (auto const &[id, line] : model.state().lines()) {
			faster.change(id).category = fast;
		}
		layout::settle(faster);
		REQUIRE_FALSE(layout::check_all(old).blocks.empty());
		model = layout::model{old};
	}
	// the invariants, and no BLOCK at all where the norms refuse any
	auto const checked = [&](layout::network const &State) {
		if (Policy == layout::edit_policy::refuse_block) {
			check_invariants(State);
		}
		else {
			check_structure(State);
			CHECK(layout::check_all(State).reasons.empty());
		}
	};
	history.push_back(model.state());
	checked(model.state());
	std::vector<layout::network> redone;
	std::map<std::string, std::pair<int, int>> tally;
	for (int step = 0; step < Count; ++step) {
		CAPTURE(step);
		auto const &state{model.state()};
		std::vector<line_id> lines;
		std::vector<line_id> plain;
		for (auto const &[id, line] : state.lines()) {
			lines.push_back(id);
			if (line.turnout == turnout_id::none) {
				plain.push_back(id);
			}
		}
		std::vector<turnout_id> turnouts;
		for (auto const &[id, turnout] : state.turnouts()) {
			turnouts.push_back(id);
		}
		std::vector<node_id> free;
		for (auto const &[id, node] : state.nodes()) {
			if (node.ends.size() == 1) {
				free.push_back(id);
			}
		}
		auto const roll{uniform(0.0, 1.0)};
		if (roll < 0.08 && history.size() > 1) {
			REQUIRE(model.undo());
			redone.push_back(history.back());
			history.pop_back();
			CHECK(model.state().same(history.back()));
			checked(model.state());
			++tally["undo"].first;
			continue;
		}
		if (roll < 0.11 && false == redone.empty()) {
			REQUIRE(model.redo());
			history.push_back(redone.back());
			redone.pop_back();
			CHECK(model.state().same(history.back()));
			checked(model.state());
			++tally["redo"].first;
			continue;
		}
		layout::command command;
		if (roll < 0.36 && false == plain.empty()) {
			auto const &line{*state.find(plain[pick(plain.size())])};
			command = layout::insert_turnout{line.id, uniform(0.0, line.solved.length), kTypes[pick(kTypes.size())], uniform(0.0, 1.0) < 0.5 ? 1 : -1, uniform(0.0, 1.0) < 0.5 ? 1 : -1};
		}
		else if (roll < 0.52 && false == turnouts.empty()) {
			command = layout::move_turnout{turnouts[pick(turnouts.size())], uniform(-40.0, 40.0)};
		}
		else if (roll < 0.58 && false == turnouts.empty()) {
			command = layout::change_turnout{turnouts[pick(turnouts.size())], kTypes[pick(kTypes.size())]};
		}
		else if (roll < 0.64 && false == turnouts.empty()) {
			command = layout::remove_turnout{turnouts[pick(turnouts.size())]};
		}
		else if (roll < 0.72 && false == lines.empty()) {
			// mostly a vertex of a line of its own, else a track of a turnout, which is refused
			auto const &line{*state.find(lines[pick(lines.size())])};
			if (line.shape.vertices.empty()) {
				command = layout::split_line{line.id, uniform(0.0, line.solved.length)};
			}
			else {
				auto const &vertex{line.shape.vertices[pick(line.shape.vertices.size())]};
				if (uniform(0.0, 1.0) < 0.5) {
					command = layout::move_vertex{line.id, vertex.id, vertex.point + glm::dvec2{uniform(-8.0, 8.0), uniform(-8.0, 8.0)}};
				}
				else {
					layout::set_vertex set{line.id, vertex.id};
					set.radius = vertex.radius * uniform(0.85, 1.2);
					command = set;
				}
			}
		}
		else if (roll < 0.78 && false == plain.empty()) {
			auto const &line{*state.find(plain[pick(plain.size())])};
			command = layout::split_line{line.id, uniform(0.0, line.solved.length)};
		}
		else if (roll < 0.86 && free.size() >= 2) {
			auto const a{pick(free.size())};
			auto b{pick(free.size() - 1)};
			b += b >= a ? 1 : 0;
			command = layout::join_ends{free[a], free[b]};
		}
		else if (roll < 0.92 && false == plain.empty()) {
			// a vertex put into a straight line and taken out again
			auto const &line{*state.find(plain[pick(plain.size())])};
			if (line.shape.vertices.empty() && line.solved.length > 300.0) {
				auto const s{uniform(100.0, line.solved.length - 100.0)};
				auto const across{layout::left_of(line.solved.tangent(s)) * uniform(-15.0, 15.0)};
				command = layout::insert_vertex{line.id, layout::vertex_id::none, curve(line.solved.position(s) + across, 2000.0)};
			}
			else if (false == line.shape.vertices.empty()) {
				command = layout::remove_vertex{line.id, line.shape.vertices[pick(line.shape.vertices.size())].id};
			}
			else {
				continue;
			}
		}
		else if (false == plain.empty()) {
			auto const &line{*state.find(plain[pick(plain.size())])};
			command = layout::delete_line{line.id};
		}
		else {
			continue;
		}
		std::string const kind{layout::command_name(command)};
		auto const before{model.state()};
		auto const blocks_before{layout::check_all(before).blocks};
		auto const result{model.execute(command, Policy)};
		if (result.done) {
			++tally[kind].first;
			history.push_back(model.state());
			redone.clear();
		}
		else {
			++tally[kind].second;
			CHECK_FALSE(result.reasons.empty());
			CHECK(model.state().same(before));
		}
		checked(model.state());
		if (Policy == layout::edit_policy::no_new_block) {
			// no BLOCK finding added: each one there now was there before, of the parameter, on the same stretch, no worse
			auto const now{layout::check_all(model.state())};
			for (auto const &block : now.blocks) {
				CAPTURE(block.text);
				CHECK(std::ranges::any_of(blocks_before, [&](layout::notice const &Old) { return layout::same_finding(Old, block); }));
			}
		}
	}
	std::string summary;
	for (auto const &[kind, counts] : tally) {
		summary += kind + " " + std::to_string(counts.first) + "/" + std::to_string(counts.first + counts.second) + ", ";
	}
	CAPTURE(summary);
	MESSAGE("seed " << Seed << ": " << summary << model.state().turnouts().size() << " turnouts at the end");
	CHECK(tally["insert turnout"].first >= 3);
	CHECK(tally["remove turnout"].first >= 1);
	CHECK(tally["insert turnout"].second >= 1);
	CHECK(tally["undo"].first >= 1);
	if (Policy == layout::edit_policy::refuse_block) {
		CHECK(tally["move turnout"].first >= 2);
	}
	else {
		// the old BLOCK findings didn't refuse every edit of the lines they're on
		CHECK(tally["move vertex"].first + tally["set vertex"].first >= 1);
	}
}

} // namespace

TEST_CASE("layout turnouts P: 250 random edits with turnouts, seed 20261124: the legs and the nodes whole, no BLOCK, a refusal changes nothing") {
	random_turnout_edits(20261124u, 250);
}

TEST_CASE("layout turnouts P: 250 random edits with turnouts, seed 31: the legs and the nodes whole, no BLOCK, a refusal changes nothing") {
	random_turnout_edits(31u, 250);
}

TEST_CASE("layout turnouts P: 250 random edits with turnouts, seed 977: the legs and the nodes whole, no BLOCK, a refusal changes nothing") {
	random_turnout_edits(977u, 250);
}

TEST_CASE("layout turnouts P: 250 random edits of an old track breaking the norms, seed 4242: no BLOCK added, a refusal changes nothing") {
	random_turnout_edits(4242u, 250, layout::edit_policy::no_new_block);
}
