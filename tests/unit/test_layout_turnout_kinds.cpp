// the kinds of turnouts of the catalogue in the domain model of the track geometry (editor/layout, editor-tracks-domain.md §4 "Rozjazd",
// §7, stage E5): the double switches with their second switch on the main track, the symmetric ones at the free end of a line, the S42
// switches with secant blades, the slips and the diamond crossings with their crossing track and slip curves; every type laid on a straight
// both ways to both sides, its legs where the catalogue puts them, its tracks and legs nodes; the kinds which aren't bent refused in a curve;
// random edits with every type along lines with arcs and transitions, the seed in the name of the case; every type made into the tracks of
// the simulator and read back by TTrack::Load, the legs at the nodes to the bit; the project file keeping them

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"
#include "track_text.h"
#include "editor/layout/layoutGenerator.hpp"
#include "editor/project/projectFile.hpp"

#include <algorithm>
#include <map>
#include <numbers>
#include <random>
#include <set>
#include <tuple>
#include <vector>

namespace {

using namespace test::layout_fixture;
using layout::line_id;
using layout::node_id;
using layout::turnout_id;
using layout::turnout_kind;

// a straight line along x on a grade of 4 per mille
layout::create_line straight(glm::dvec3 const &Start = {0.0, 100.0, 0.0}, double const Length = 1000.0) {
	layout::create_line result;
	result.start = Start;
	result.end = Start + glm::dvec3{Length, Length * 0.004, 0.0};
	result.category = line_class();
	return result;
}

// the chainage a turnout of the type starts at on the line of the length: in its middle, or at its end for a symmetric switch
double start_for(layout::turnout_type const &Type, double const Length, int const Direction) {
	if (Type.kind == turnout_kind::symmetric) {
		return Direction > 0 ? Length : 0.0;
	}
	return Direction > 0 ? Length * 0.4 : Length * 0.6;
}

glm::dvec2 plan(layout::network const &Network, node_id const Node) {
	auto const *at{Network.find(Node)};
	REQUIRE(at != nullptr);
	return geometry::plan_of(at->position);
}

// the turn of the parts of a diverging track and the angle of its blade
double turn_of(std::vector<switch_design::part> const &Parts, double const Entry) {
	auto result{Entry};
	for (auto const &part : Parts) {
		auto const k1{part.radius1 != 0.0 ? 1.0 / part.radius1 : 0.0};
		auto const k2{part.radius2 != 0.0 ? 1.0 / part.radius2 : 0.0};
		result += 0.5 * (k1 + k2) * part.length;
	}
	return result;
}

// the legs of the turnout where the type puts them on a straight from its start along the direction, to the hand
void check_legs(layout::network const &State, layout::turnout const &Turnout, layout::turnout_type const &Type, glm::dvec2 const &Along) {
	auto const start{plan(State, Turnout.start)};
	auto const hand{static_cast<double>(Turnout.hand)};
	auto const heading_at_end = [&](line_id const Track) { return State.find(Track)->solved.end.heading; };
	switch (Type.kind) {
	case turnout_kind::symmetric: {
		// each track turning by half of the angle between them, to both sides
		auto const half{0.5 * std::atan(1.0 / Type.ratio)};
		CHECK(geometry::signed_angle(Along, heading_at_end(Turnout.main)) == doctest::Approx(-hand * half).epsilon(1e-9));
		CHECK(geometry::signed_angle(Along, heading_at_end(Turnout.diverging)) == doctest::Approx(hand * half).epsilon(1e-9));
		CHECK(State.find(Turnout.start)->ends.size() == 3);
		break;
	}
	case turnout_kind::slip:
	case turnout_kind::crossing: {
		// the crossing track across the middle at the angle of the slip, its ends half of its length from the crossing
		auto const centre{start + Along * Type.half};
		auto const across{geometry::turned(Along, hand * Type.angle)};
		CHECK(glm::distance(plan(State, Turnout.main_end), start + Along * 2.0 * Type.half) < 1e-9);
		CHECK(glm::distance(plan(State, Turnout.cross_start), centre - across * Type.half) < 1e-9);
		CHECK(glm::distance(plan(State, Turnout.cross_end), centre + across * Type.half) < 1e-9);
		CHECK(std::abs(geometry::signed_angle(across, heading_at_end(Turnout.crossing))) < 1e-12);
		if (Type.kind == turnout_kind::slip) {
			// the slip curves from the start to the end of the crossing track, from its start to the end of the main one
			CHECK(std::abs(geometry::signed_angle(across, heading_at_end(Turnout.diverging))) < 1e-12);
			// along the track it leaves up to the slip curve, the curve, along the track it meets after it
			CHECK(State.find(Turnout.diverging)->solved.length == doctest::Approx(2.0 * (Type.half - Type.curve.tangent()) + Type.curve.length()).epsilon(1e-12));
			CHECK(State.find(Turnout.cross_end)->ends.size() == 2);
			if (Type.slip_curves == 2) {
				CHECK(std::abs(geometry::signed_angle(Along, heading_at_end(Turnout.diverging2))) < 1e-12);
				CHECK(State.find(Turnout.cross_start)->ends.size() == 2);
				CHECK(State.find(Turnout.main_end)->ends.size() == 3);
			}
			// the speed of the slip curves, the line's on the tracks crossing
			CHECK(Turnout.speed_diverging == switch_curve::branch_speed(Type.radius, Type.ratio, 0.0, 0.0, false).speed);
		}
		CHECK(Turnout.speed_main == 80.0);
		break;
	}
	case turnout_kind::twofold: {
		// the second switch at the stagger along the main track, its diverging track to the other side or the same one
		CHECK(glm::distance(plan(State, Turnout.second), start + Along * Type.stagger) < 1e-9);
		CHECK(glm::distance(plan(State, Turnout.main_end), start + Along * Type.length) < 1e-9);
		CHECK(geometry::signed_angle(Along, heading_at_end(Turnout.diverging)) == doctest::Approx(hand * turn_of(Type.parts, Type.entry)).epsilon(1e-9));
		auto const hand2{Type.opposite ? -hand : hand};
		CHECK(geometry::signed_angle(Along, heading_at_end(Turnout.diverging2)) == doctest::Approx(hand2 * turn_of(Type.parts2, Type.entry2)).epsilon(1e-9));
		CHECK(State.find(Turnout.second)->ends.size() == 3);
		break;
	}
	default: {
		// the main track a + b, the end of the diverging track at 1:n to the hand
		CHECK(glm::distance(plan(State, Turnout.main_end), start + Along * Type.length) < 1e-9);
		CHECK(geometry::signed_angle(Along, heading_at_end(Turnout.diverging)) == doctest::Approx(hand * std::atan(1.0 / Type.ratio)).epsilon(1e-9));
		CHECK(geometry::cross(Along, plan(State, Turnout.diverging_end) - start) * hand > 0.0);
		// the speed rule of the editor on a straight: tab. 11.2, no more than formula 9.9 gives the small radii of the industrial ones
		CHECK(Turnout.speed_diverging == switch_curve::branch_speed(Type.radius, Type.ratio, 0.0, 0.0, Type.movable).speed);
		break;
	}
	}
}

} // namespace

TEST_CASE("layout turnout kinds: every type of the catalogue is a type the model lays: ordinary, secant, symmetric, double, slips, crossings") {
	auto const names{layout::turnout_types()};
	auto const &entries{switch_design::catalogue_entries()};
	CHECK(names.size() == entries.size());
	std::map<turnout_kind, int> kinds;
	for (auto const &entry : entries) {
		CAPTURE(entry.name);
		std::string why;
		auto const type{layout::turnout_type_of(entry.name, &why)};
		CAPTURE(why);
		REQUIRE(type.has_value());
		++kinds[type->kind];
		if (entry.blades == "secant" && type->kind == turnout_kind::ordinary) {
			// the blades meet the stock rail at their angle at the end of the pre-blade stretch
			CHECK(type->entry == doctest::Approx(glm::radians(entry.entry)));
			CHECK(type->lead > 0.0);
		}
	}
	MESSAGE("the kinds: ordinary " << kinds[turnout_kind::ordinary] << ", symmetric " << kinds[turnout_kind::symmetric] << ", double " << kinds[turnout_kind::twofold] << ", slips "
	                               << kinds[turnout_kind::slip] << ", crossings " << kinds[turnout_kind::crossing]);
	CHECK(kinds[turnout_kind::symmetric] == 3);
	CHECK(kinds[turnout_kind::twofold] == 15);
	CHECK(kinds[turnout_kind::slip] == 18);
	CHECK(kinds[turnout_kind::crossing] == 20);
	std::string why;
	CHECK_FALSE(layout::turnout_type_of("Rz 60E1-999-1:99", &why).has_value());
}

TEST_CASE("layout turnout kinds: every type laid on a straight both ways to both sides: its legs where the catalogue puts them, one step to undo") {
	for (auto const &name : layout::turnout_types()) {
		auto const type{layout::turnout_type_of(name)};
		REQUIRE(type.has_value());
		for (auto const hand : {1, -1}) {
			for (auto const direction : {1, -1}) {
				CAPTURE(name);
				CAPTURE(hand);
				CAPTURE(direction);
				layout::model model;
				auto const length{type->kind == turnout_kind::symmetric ? 400.0 : 1000.0};
				REQUIRE(model.execute(straight({0.0, 100.0, 0.0}, length)).done);
				auto const line{model.state().lines().begin()->first};
				auto const before{model.state()};
				auto const chainage{start_for(*type, length, direction)};
				auto const made{model.execute(layout::insert_turnout{line, chainage, name, hand, direction})};
				CAPTURE(reasons_of(made));
				if (false == made.done && type->name.find("16/12") != std::string::npos) {
					// fitted to the tangents measured on a scenery, not of Id-4: its transition curve too short for the norms at its speed
					CHECK(reasons_of(made).find("ST-T1-A6 tab. 9.14") != std::string::npos);
					CHECK(model.state().same(before));
					continue;
				}
				REQUIRE(made.done);
				auto const &state{model.state()};
				check_invariants(state);
				REQUIRE(made.turnouts.size() == 1);
				auto const &turnout{*state.find(made.turnouts.front())};
				CHECK(layout::kind_of(turnout) == type->kind);
				// the start where it was put, the turnout running along the line or against it; a symmetric switch away from its line
				glm::dvec2 const along{static_cast<double>(direction), 0.0};
				CHECK(glm::distance(plan(state, turnout.start), glm::dvec2{chainage, 0.0}) < 1e-9);
				check_legs(state, turnout, *type, along);
				// moved along its line and back, the same to the micrometre; a symmetric switch isn't moved off the end of its line
				auto const moved{model.execute(layout::move_turnout{turnout.id, 7.5})};
				CAPTURE(reasons_of(moved));
				CHECK(moved.done == (type->kind != turnout_kind::symmetric));
				if (moved.done) {
					check_invariants(model.state());
					CHECK(glm::distance(plan(model.state(), model.state().find(turnout.id)->start), glm::dvec2{chainage + direction * 7.5, 0.0}) < 1e-9);
					REQUIRE(model.undo());
				}
				// taken out: the line one again, as it was laid
				auto const removed{model.execute(layout::remove_turnout{turnout.id})};
				REQUIRE(removed.done);
				check_invariants(model.state());
				CHECK(model.state().lines().size() == 1);
				CHECK(model.state().turnouts().empty());
				REQUIRE(model.undo());
				REQUIRE(model.undo());
				CHECK(model.state().same(before));
			}
		}
	}
}

TEST_CASE("layout turnout kinds: the secant blades of S42 meet the stock rail at their angle at the end of the pre-blade stretch") {
	layout::model model;
	REQUIRE(model.execute(straight()).done);
	auto const line{model.state().lines().begin()->first};
	auto const made{model.execute(layout::insert_turnout{line, 400.0, "Rz S42-190-1:7", 1, 1})};
	CAPTURE(reasons_of(made));
	REQUIRE(made.done);
	auto const &state{model.state()};
	auto const &turnout{*state.find(made.turnouts.front())};
	auto const type{layout::turnout_type_of("Rz S42-190-1:7")};
	auto const &elements{state.find(turnout.diverging)->solved.elements};
	REQUIRE(elements.size() >= 2);
	// along the main track for the pre-blade stretch, then turned by 25' at the tip of the blade, the arc turning the rest of 1:7
	CHECK(elements.front().kind == layout::element_kind::straight);
	CHECK(elements.front().length == doctest::Approx(type->lead));
	auto const kink{geometry::signed_angle(elements[0].tangent(elements[0].length), elements[1].direction)};
	CHECK(kink == doctest::Approx(glm::radians(25.0 / 60.0)));
	CHECK(geometry::signed_angle({1.0, 0.0}, state.find(turnout.diverging)->solved.end.heading) == doctest::Approx(std::atan(1.0 / 7.0)).epsilon(1e-9));
	// the turn at the tip isn't a step of the line: the node at the start holds the heading of the main track
	check_invariants(state);
}

TEST_CASE("layout turnout kinds: double switches, slips, crossings and symmetric switches aren't laid into a curve (ST-T1-A6 11.4 pkt 4)") {
	layout::model model;
	layout::create_line curved;
	curved.start = {0.0, 100.0, 0.0};
	curved.end = {1400.0, 100.0, 400.0};
	layout::vertex corner;
	corner.point = {700.0, 0.0};
	corner.radius = 1200.0;
	corner.transition_in = corner.transition_out = 70.0;
	curved.vertices = {corner};
	curved.category = line_class();
	curved.category.track = norms::track_kind::side;
	REQUIRE(model.execute(curved).done);
	auto const line{model.state().lines().begin()->first};
	auto const &laid{model.state().find(line)->solved};
	double middle{0.0};
	for (auto const &e : laid.elements) {
		if (e.role == layout::element_role::arc) {
			middle = e.chainage + 0.5 * e.length;
		}
	}
	auto const before{model.state()};
	for (auto const *name : {"Rpd 49E1-190-1:9/1:9", "Rkpd 60E1-190-1:9", "Rkp 49E1-190-1:9", "Skrz 49E1-1:9"}) {
		CAPTURE(name);
		auto const refused{model.execute(layout::insert_turnout{line, middle, name, 1, 1})};
		CHECK_FALSE(refused.done);
		CAPTURE(reasons_of(refused));
		CHECK(reasons_of(refused).find("11.4 pkt 4") != std::string::npos);
		CHECK(model.state().same(before));
	}
	// an ordinary switch with secant blades is bent, in a side track
	auto const bent{model.execute(layout::insert_turnout{line, middle, "Rz S42-205-1:9 I", 1, 1})};
	CAPTURE(reasons_of(bent));
	CHECK(bent.done);
	if (bent.done) {
		check_invariants(model.state());
		CHECK(model.state().find(bent.turnouts.front())->bent());
		REQUIRE(model.undo());
	}
	// a symmetric switch at the end of a line ending in its curve
	auto const split{model.execute(layout::split_line{line, middle})};
	REQUIRE(split.done);
	REQUIRE(model.execute(layout::delete_line{split.lines.front()}).done);
	auto const symmetric{model.execute(layout::insert_turnout{line, model.state().find(line)->solved.length, "Rls 49E1-215-1:4,8", 1, 1})};
	CHECK_FALSE(symmetric.done);
	CAPTURE(reasons_of(symmetric));
	CHECK(reasons_of(symmetric).find("symmetric") != std::string::npos);
}

namespace {

// random edits with every type of the catalogue: laid along the lines (symmetric switches at their free ends), moved, changed to any
// other type, taken out, with splits, joins and deletions of the lines, undoing and redoing; after each the invariants, a refusal changing
// nothing, an undo giving the state before
void random_kind_edits(unsigned const Seed, int const Count) {
	std::mt19937 random{Seed};
	auto const uniform = [&](double const Low, double const High) { return std::uniform_real_distribution<double>{Low, High}(random); };
	auto const pick = [&](std::size_t const Size) { return std::uniform_int_distribution<std::size_t>{0, Size - 1}(random); };
	auto const types{layout::turnout_types()};
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	REQUIRE(model.execute(straight({0.0, 100.0, -300.0}, 1200.0)).done);
	REQUIRE(model.execute(straight({0.0, 100.0, -600.0}, 300.0)).done);
	std::vector<layout::network> history{model.state()};
	std::vector<layout::network> redone;
	std::map<std::string, std::pair<int, int>> tally;
	std::map<turnout_kind, int> laid;
	check_invariants(model.state());
	for (int step = 0; step < Count; ++step) {
		CAPTURE(step);
		auto const &state{model.state()};
		std::vector<line_id> plain;
		for (auto const &[id, line] : state.lines()) {
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
		if (roll < 0.07 && history.size() > 1) {
			REQUIRE(model.undo());
			redone.push_back(history.back());
			history.pop_back();
			CHECK(model.state().same(history.back()));
			check_invariants(model.state());
			++tally["undo"].first;
			continue;
		}
		if (roll < 0.10 && false == redone.empty()) {
			REQUIRE(model.redo());
			history.push_back(redone.back());
			redone.pop_back();
			CHECK(model.state().same(history.back()));
			check_invariants(model.state());
			++tally["redo"].first;
			continue;
		}
		layout::command command;
		auto const &type{types[pick(types.size())]};
		if (roll < 0.40 && false == plain.empty()) {
			auto const &line{*state.find(plain[pick(plain.size())])};
			auto chainage{uniform(0.0, line.solved.length)};
			auto direction{uniform(0.0, 1.0) < 0.5 ? 1 : -1};
			// a symmetric switch at an end of the line
			if (layout::turnout_type_of(type)->kind == turnout_kind::symmetric) {
				chainage = direction > 0 ? line.solved.length : 0.0;
			}
			command = layout::insert_turnout{line.id, chainage, type, uniform(0.0, 1.0) < 0.5 ? 1 : -1, direction};
		}
		else if (roll < 0.55 && false == turnouts.empty()) {
			command = layout::move_turnout{turnouts[pick(turnouts.size())], uniform(-30.0, 30.0)};
		}
		else if (roll < 0.62 && false == turnouts.empty()) {
			command = layout::change_turnout{turnouts[pick(turnouts.size())], type};
		}
		else if (roll < 0.70 && false == turnouts.empty()) {
			command = layout::remove_turnout{turnouts[pick(turnouts.size())]};
		}
		else if (roll < 0.80 && false == plain.empty()) {
			auto const &line{*state.find(plain[pick(plain.size())])};
			command = layout::split_line{line.id, uniform(0.0, line.solved.length)};
		}
		else if (roll < 0.90 && free.size() >= 2) {
			auto const a{pick(free.size())};
			auto b{pick(free.size() - 1)};
			b += b >= a ? 1 : 0;
			command = layout::join_ends{free[a], free[b]};
		}
		else if (false == plain.empty() && plain.size() > 2) {
			command = layout::delete_line{plain[pick(plain.size())]};
		}
		else {
			continue;
		}
		std::string const kind{layout::command_name(command)};
		auto const before{model.state()};
		auto const result{model.execute(command)};
		if (result.done) {
			++tally[kind].first;
			history.push_back(model.state());
			redone.clear();
			for (auto const id : result.turnouts) {
				if (auto const *made{model.state().find(id)}) {
					++laid[layout::kind_of(*made)];
				}
			}
		}
		else {
			++tally[kind].second;
			CHECK_FALSE(result.reasons.empty());
			CHECK(model.state().same(before));
		}
		check_invariants(model.state());
	}
	std::string summary;
	for (auto const &[kind, counts] : tally) {
		summary += kind + " " + std::to_string(counts.first) + "/" + std::to_string(counts.first + counts.second) + ", ";
	}
	MESSAGE("seed " << Seed << ": " << summary << "laid: ordinary " << laid[turnout_kind::ordinary] << ", symmetric " << laid[turnout_kind::symmetric] << ", double "
	                << laid[turnout_kind::twofold] << ", slips " << laid[turnout_kind::slip] << ", crossings " << laid[turnout_kind::crossing] << "; " << model.state().turnouts().size()
	                << " turnouts at the end");
	CHECK(tally["insert turnout"].first >= 5);
	CHECK(tally["insert turnout"].second >= 1);
	CHECK(tally["move turnout"].first >= 2);
	CHECK(tally["remove turnout"].first >= 1);
	CHECK(laid.size() >= 3);
	// every state undone back to the start, the lines laid first left
	while (model.undoable() > 3) {
		REQUIRE(model.undo());
		check_invariants(model.state());
	}
	CHECK(model.state().same(history.front()));
}

} // namespace

TEST_CASE("layout turnout kinds P: 200 random edits with every type along arcs and transitions, seed 5150: valid nodes or a refusal, undo exact") {
	random_kind_edits(5150u, 200);
}

TEST_CASE("layout turnout kinds P: 200 random edits with every type along arcs and transitions, seed 2718: valid nodes or a refusal, undo exact") {
	random_kind_edits(2718u, 200);
}

TEST_CASE("layout turnout kinds P: 200 random edits with every type along arcs and transitions, seed 1618: valid nodes or a refusal, undo exact") {
	random_kind_edits(1618u, 200);
}

TEST_CASE("layout turnout kinds G: every type made into the tracks of the simulator, read back by TTrack::Load, the legs at the nodes to the bit") {
	test::track_reading const reading;
	for (auto const &name : layout::turnout_types()) {
		CAPTURE(name);
		auto const type{layout::turnout_type_of(name)};
		layout::model model;
		auto const length{type->kind == turnout_kind::symmetric ? 400.0 : 1000.0};
		REQUIRE(model.execute(straight({0.0, 100.0, 0.0}, length)).done);
		auto const line{model.state().lines().begin()->first};
		auto const made{model.execute(layout::insert_turnout{line, start_for(*type, length, 1), name, -1, 1})};
		CAPTURE(reasons_of(made));
		if (false == made.done && type->name.find("16/12") != std::string::npos) {
			continue;
		}
		REQUIRE(made.done);
		auto const &state{model.state()};
		auto const &turnout{*state.find(made.turnouts.front())};
		auto const generated{layout::generate(state, turnout.id)};
		CAPTURE(generated.text);
		for (auto const &error : generated.errors) {
			CAPTURE(error);
			CHECK(false);
		}
		REQUIRE_FALSE(generated.parts.empty());
		// the legs of the turnout, the nodes of its own and those of the line it lies on
		std::vector<glm::dvec3> legs;
		for (auto const id : {turnout.start, turnout.main_end, turnout.diverging_end, turnout.diverging2_end, turnout.cross_start, turnout.cross_end}) {
			if (id != node_id::none && std::ranges::find(legs, state.find(id)->position) == legs.end()) {
				legs.push_back(state.find(id)->position);
			}
		}
		// each end of each path read back: at a leg, or meeting the end of another path of the turnout there to the bit
		std::vector<glm::dvec3> ends;
		std::set<std::size_t> legs_met;
		for (auto const &part : generated.parts) {
			CAPTURE(part.name);
			if (part.is_switch) {
				auto const track{test::read_track(part.text)};
				REQUIRE(track->eType == tt_Switch);
				REQUIRE(track->m_paths.size() == 2);
				CHECK(track->name() == part.name);
				REQUIRE(track->SwitchExtension != nullptr);
				CHECK(track->SwitchExtension->fVelocity == static_cast<float>(turnout.velocity));
				CHECK(track->SwitchExtension->m_movablenose == type->movable);
				for (auto const &path : track->m_paths) {
					auto const &pieces{path.pieces.empty() ? std::vector<segment_data>{path} : path.pieces};
					for (std::size_t k = 1; k < pieces.size(); ++k) {
						CHECK(pieces[k - 1].points[segment_data::point::end] == pieces[k].points[segment_data::point::start]);
					}
					ends.push_back(path.points[segment_data::point::start]);
					ends.push_back(path.points[segment_data::point::end]);
				}
				continue;
			}
			for (auto const &piece : part.pieces[0]) {
				auto const track{test::read_track(piece.text)};
				REQUIRE(track->eType == tt_Normal);
				CHECK(track->name() == piece.name);
				ends.push_back(track->m_paths.front().points[segment_data::point::start]);
				ends.push_back(track->m_paths.front().points[segment_data::point::end]);
			}
		}
		// the paths of a switch from its start: its two starts at one point, counted once
		std::map<std::tuple<double, double, double>, int> counts;
		for (auto const &end : ends) {
			++counts[{end.x, end.y, end.z}];
		}
		for (auto const &end : ends) {
			CAPTURE(end.x);
			CAPTURE(end.z);
			auto const at_leg{std::ranges::find(legs, end)};
			if (at_leg != legs.end()) {
				legs_met.insert(static_cast<std::size_t>(at_leg - legs.begin()));
				continue;
			}
			// inside the turnout: two ends meet, or the ends of the paths of the switch which ends there and of the track going on
			CHECK(counts[{end.x, end.y, end.z}] >= 2);
		}
		CHECK(legs_met.size() == legs.size());
	}
}

TEST_CASE("layout turnout kinds G: the project file keeps the double switches, slips and crossings and lays their tracks again the same to the bit") {
	layout::model model;
	REQUIRE(model.execute(straight()).done);
	auto const line{model.state().lines().begin()->first};
	auto const twofold{model.execute(layout::insert_turnout{line, 100.0, "Rpd 49E1-190-1:9/1:9", 1, 1})};
	CAPTURE(reasons_of(twofold));
	REQUIRE(twofold.done);
	std::vector<line_id> plain;
	for (auto const &[id, each] : model.state().lines()) {
		if (each.turnout == turnout_id::none && each.solved.length > 200.0) {
			plain.push_back(id);
		}
	}
	REQUIRE_FALSE(plain.empty());
	auto const tail{plain.back()};
	REQUIRE(model.execute(layout::insert_turnout{tail, 200.0, "Rkpd 49E1-300-1:9", -1, -1}).done);
	for (auto const &[id, each] : model.state().lines()) {
		if (each.turnout == turnout_id::none && each.solved.length > 300.0) {
			REQUIRE(model.execute(layout::insert_turnout{id, 150.0, "Skrz 49E1-1:9", 1, 1}).done);
			break;
		}
	}
	REQUIRE(model.state().turnouts().size() == 3);
	check_invariants(model.state());
	// the network as settle() leaves it (test_project_file.cpp): the tracks a turnout lays laid before the node at its start takes the grade
	// of the line laid beyond it, 1e-16 off the ones laid again from the nodes as they are
	auto settled{model.state()};
	{
		layout::transaction change{settled};
		for (auto const &[id, each] : settled.turnouts()) {
			change.change(id);
		}
		layout::settle(change);
	}
	project::document written;
	written.network = settled;
	auto const text{project::write(written)};
	CAPTURE(text);
	auto const read{project::read(text)};
	CAPTURE(read.error);
	REQUIRE(read.project.has_value());
	CHECK(read.notes.empty());
	auto const &network{read.project->network};
	CHECK(network.same(settled));
	for (auto const &[id, each] : settled.lines()) {
		CAPTURE(layout::number(id));
		CHECK(network.find(id)->solved == each.solved);
	}
	CHECK(project::write(*read.project) == text);
}
