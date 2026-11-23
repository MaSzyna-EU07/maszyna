// the commands editing the lines and the nodes of the track geometry (editor/layout, editor-tracks-domain.md §4 and §8): each one kept
// whole and undone in one step, or refused with its reasons and the state left as it was; the track kept as it was by a split, on a
// straight, in an arc and in a transition; ends joined again by a line solved between them; and random edits with a fixed seed, the
// seed in the name of the case, after each of which the nodes run on (place, heading, curvature by the profile), the cant runs on and
// no BLOCK limit of the norms is passed

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

layout::line const &only_line(layout::model const &Model) {
	REQUIRE(Model.state().lines().size() >= 1);
	return Model.state().lines().begin()->second;
}

// the track along the line at the chainages, before and after a change which should keep it
struct track_points {
	std::vector<glm::dvec3> points;
	std::vector<double> cants;
};

track_points along(layout::network const &Network, std::vector<line_id> const &Lines, double const Step) {
	track_points result;
	for (auto const id : Lines) {
		auto const &solved{Network.find(id)->solved};
		for (double s = 0.0; s < solved.length; s += Step) {
			result.points.push_back(solved.point(s));
			result.cants.push_back(solved.cant_at(s));
		}
	}
	return result;
}

// the chainage the share of the way along the element of the role in the curve of the vertex (a straight: the one leading to it, 0 to the end)
double inside(layout::line const &Line, layout::element_role const Role, unsigned const Vertex, double const Share = 0.5) {
	for (auto const &e : Line.solved.elements) {
		if (e.role == Role && e.vertex == Vertex) {
			return e.chainage + Share * e.length;
		}
	}
	FAIL("no such element");
	return 0.0;
}

void check_same_track(layout::network const &Before, line_id const Whole, layout::network const &After, line_id const First, line_id const Second) {
	auto const &whole{Before.find(Whole)->solved};
	auto const &first{After.find(First)->solved};
	auto const &second{After.find(Second)->solved};
	CHECK(first.length + second.length == doctest::Approx(whole.length).epsilon(1e-9));
	for (double s = 0.0; s <= whole.length; s += 7.3) {
		CAPTURE(s);
		auto const &part{s < first.length ? first : second};
		auto const local{s < first.length ? s : s - first.length};
		CHECK(glm::distance(whole.point(s), part.point(local)) < 1e-6);
		CHECK(std::abs(whole.cant_at(s) - part.cant_at(local)) < 1e-6);
		CHECK(std::abs(whole.curvature(s) - part.curvature(local)) < 1e-9);
	}
}

} // namespace

TEST_CASE("layout commands: a line laid between two new nodes, its ends at them, its curves by the norms preferred") {
	layout::model model;
	auto const made{model.execute(three_curves())};
	CAPTURE(reasons_of(made));
	REQUIRE(made.done);
	REQUIRE(made.lines.size() == 1);
	REQUIRE(made.nodes.size() == 2);
	auto const &line{only_line(model)};
	CHECK(line.solved.elements.size() == 3 * 3 + 4);
	CHECK(line.solved.length > 2300.0);
	check_invariants(model.state());
	// the free ends take the heading of the line
	auto const *start{model.state().find(line.start)};
	CHECK(start->heading == glm::dvec2{1.0, 0.0});
	CHECK(start->curvature == 0.0);
	CHECK(model.undoable() == 1);
	// the ids are given once: undone and made again, the line is a new one
	REQUIRE(model.undo());
	CHECK(model.state().lines().empty());
	CHECK(model.state().nodes().empty());
	auto const again{model.execute(three_curves())};
	REQUIRE(again.done);
	CHECK(layout::number(again.lines.front()) > layout::number(made.lines.front()));
	CHECK(layout::number(again.nodes.front()) > layout::number(made.nodes.back()));
}

TEST_CASE("layout commands: a command breaking a BLOCK limit or an invariant is refused with its reasons, the state as it was") {
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const before{model.state()};
	auto const &line{only_line(model)};
	auto const v1{line.shape.vertices[0].id};
	auto const v2{line.shape.vertices[1].id};
	// a radius below the least of the norms (PLK P2 180 m, TSI 150 m)
	auto const tiny{model.execute(layout::set_vertex{line.id, v2, 120.0})};
	CHECK_FALSE(tiny.done);
	CHECK(reasons_of(tiny).find("radius") != std::string::npos);
	CHECK(model.state().same(before));
	// a cant beyond the greatest
	auto const cant{model.execute(layout::set_vertex{line.id, v2, std::nullopt, std::nullopt, std::nullopt, 190.0})};
	CHECK_FALSE(cant.done);
	CHECK(model.state().same(before));
	// a vertex moved so its curve overlaps the next one
	auto const overlap{model.execute(layout::move_vertex{line.id, v1, {980.0, 260.0}})};
	CHECK_FALSE(overlap.done);
	CHECK_FALSE(overlap.reasons.empty());
	CHECK(model.state().same(before));
	// transitions too long for the deflection
	auto const turn{model.execute(layout::set_vertex{line.id, v1, std::nullopt, 400.0, 400.0})};
	CHECK_FALSE(turn.done);
	CHECK(model.state().same(before));
	// no such vertex, no such line
	CHECK_FALSE(model.execute(layout::move_vertex{line.id, layout::vertex_id{999}, {0.0, 0.0}}).done);
	CHECK_FALSE(model.execute(layout::delete_line{line_id{999}}).done);
	CHECK(model.state().same(before));
	CHECK(model.undoable() == 1);
	CHECK(model.redoable() == 0);
}

TEST_CASE("layout commands: a radius allowed with a warning is kept and the warning told; each command one step to undo and redo") {
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const original{model.state()};
	auto const &line{only_line(model)};
	auto const v2{line.shape.vertices[1].id};
	// R 280 m at 80 km/h: below the 300 m of PLK P1, above the 180 m of P2
	auto const tight{layout::preferred_curve(norms::profile_id::plk, line_class(), 280.0)};
	auto const warned{model.execute(layout::set_vertex{line.id, v2, 280.0, tight.transition, tight.transition, tight.cant})};
	CAPTURE(reasons_of(warned));
	REQUIRE(warned.done);
	CHECK(std::ranges::any_of(warned.notices, [](layout::notice const &Notice) { return Notice.finding.what == norms::parameter::radius && Notice.finding.level == norms::tier::warn; }));
	check_invariants(model.state());
	auto const tightened{model.state()};
	REQUIRE(model.execute(layout::move_vertex{line.id, line.shape.vertices[2].id, {1610.0, 305.0}}).done);
	auto const moved{model.state()};
	REQUIRE(model.undo());
	CHECK(model.state().same(tightened));
	REQUIRE(model.undo());
	CHECK(model.state().same(original));
	REQUIRE(model.redo());
	CHECK(model.state().same(tightened));
	REQUIRE(model.redo());
	CHECK(model.state().same(moved));
	CHECK_FALSE(model.redo());
	// a new command after an undo leaves nothing to redo
	REQUIRE(model.undo());
	auto const wide{layout::preferred_curve(norms::profile_id::plk, line_class(), 700.0)};
	auto const widened{model.execute(layout::set_vertex{line.id, v2, 700.0, wide.transition, wide.transition, wide.cant})};
	CAPTURE(reasons_of(widened));
	REQUIRE(widened.done);
	CHECK(model.redoable() == 0);
}

TEST_CASE("layout commands: a line split on a straight, in an arc and in a clothoid keeps the track; the parts meet at the new node") {
	for (auto const role : {layout::element_role::straight, layout::element_role::arc, layout::element_role::transition_out}) {
		layout::model model;
		REQUIRE(model.execute(three_curves()).done);
		auto const before{model.state()};
		auto const &line{only_line(model)};
		auto const id{line.id};
		// on the straight between the first two curves, in the arc of the second, inside its transition out
		auto const second_curve{layout::number(line.shape.vertices[1].id)};
		auto const chainage{inside(line, role, second_curve, role == layout::element_role::straight ? 0.5 : role == layout::element_role::arc ? 0.37 : 0.41)};
		std::string const where{role == layout::element_role::straight ? "straight" : role == layout::element_role::arc ? "arc" : "transition"};
		CAPTURE(where);
		auto const split{model.execute(layout::split_line{id, chainage})};
		CAPTURE(reasons_of(split));
		REQUIRE(split.done);
		REQUIRE(split.lines.size() == 1);
		REQUIRE(split.nodes.size() == 1);
		check_invariants(model.state());
		auto const second{split.lines.front()};
		check_same_track(before, id, model.state(), id, second);
		auto const &cut{*model.state().find(split.nodes.front())};
		CHECK(cut.ends.size() == 2);
		CHECK(model.state().find(id)->end == cut.id);
		CHECK(model.state().find(second)->start == cut.id);
		CHECK(model.state().find(second)->chainage_start == doctest::Approx(chainage));
		if (where != "straight") {
			CHECK(cut.curvature != 0.0);
			CHECK(cut.cant != 0.0);
			CHECK(model.state().find(id)->end_continues);
			CHECK(model.state().find(second)->start_continues);
		}
		// one step back to the line whole
		REQUIRE(model.undo());
		CHECK(model.state().same(before));
	}
}

TEST_CASE("layout commands: a split inside a vertical curve or a Bloss transition is refused; outside them it's kept") {
	layout::model model;
	REQUIRE(model.execute(three_curves(layout::transition_shape::bloss)).done);
	auto const before{model.state()};
	auto const &line{only_line(model)};
	CHECK_FALSE(model.execute(layout::split_line{line.id, 1000.0 + 3.0}).done);
	for (auto const &e : line.solved.elements) {
		if (e.role == layout::element_role::transition_in) {
			CHECK_FALSE(model.execute(layout::split_line{line.id, e.chainage + 0.5 * e.length}).done);
			break;
		}
	}
	CHECK(model.state().same(before));
	auto const kept{model.execute(layout::split_line{line.id, inside(line, layout::element_role::straight, layout::number(line.shape.vertices[2].id))})};
	CAPTURE(reasons_of(kept));
	CHECK(kept.done);
	check_invariants(model.state());
}

TEST_CASE("layout commands: a line deleted leaves two free ends facing each other, a line solved between them joins them again") {
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const id{only_line(model).id};
	// three parts: the curves of the middle one between two straights
	auto const first{model.execute(layout::split_line{id, inside(only_line(model), layout::element_role::straight, layout::number(only_line(model).shape.vertices[1].id))})};
	REQUIRE(first.done);
	auto const middle{first.lines.front()};
	auto const second{model.execute(layout::split_line{middle, inside(*model.state().find(middle), layout::element_role::straight, 0)})};
	REQUIRE(second.done);
	check_invariants(model.state());
	auto const from{model.state().find(middle)->start};
	auto const to{model.state().find(middle)->end};
	auto const curves{model.state().find(middle)->shape.vertices.size()};
	REQUIRE(model.execute(layout::delete_line{middle}).done);
	CHECK(model.state().find(middle) == nullptr);
	CHECK(model.state().find(from)->ends.size() == 1);
	CHECK(model.state().find(to)->ends.size() == 1);
	check_invariants(model.state());
	auto const joined{model.execute(layout::join_ends{from, to})};
	CAPTURE(reasons_of(joined));
	REQUIRE(joined.done);
	auto const &line{*model.state().find(joined.lines.front())};
	CHECK(line.start == from);
	CHECK(line.end == to);
	CHECK(line.shape.vertices.size() == std::min<std::size_t>(curves, 2));
	CHECK(model.state().find(from)->ends.size() == 2);
	check_invariants(model.state());
	// the same once more, the other way round: from the end of the line to its start
	REQUIRE(model.undo());
	auto const back{model.execute(layout::join_ends{to, from})};
	CAPTURE(reasons_of(back));
	REQUIRE(back.done);
	check_invariants(model.state());
	// a free end facing on along one straight with the other is joined by a straight
	layout::model straight;
	auto plain{three_curves()};
	plain.vertices.clear();
	plain.grades.clear();
	plain.end = {1000.0, 101.0, 0.0};
	REQUIRE(straight.execute(plain).done);
	auto const lone{straight.state().lines().begin()->first};
	REQUIRE(straight.execute(layout::split_line{lone, 400.0}).done);
	REQUIRE(straight.execute(layout::split_line{lone, 200.0}).done);
	auto const piece{std::ranges::find_if(straight.state().lines(), [&](auto const &Each) { return Each.second.solved.length < 250.0 && Each.first != lone; })};
	REQUIRE(piece != straight.state().lines().end());
	auto const a{piece->second.start};
	auto const b{piece->second.end};
	REQUIRE(straight.execute(layout::delete_line{piece->first}).done);
	auto const bridged{straight.execute(layout::join_ends{a, b})};
	CAPTURE(reasons_of(bridged));
	REQUIRE(bridged.done);
	CHECK(straight.state().find(bridged.lines.front())->shape.vertices.empty());
	check_invariants(straight.state());
	// a node holding two ends isn't a free end
	CHECK_FALSE(straight.execute(layout::join_ends{a, b}).done);
}

namespace {

// random edits of the network: moved vertices, new radii, transitions and cants, splits, joins, deletions, undoing and redoing
void random_edits(unsigned const Seed, int const Count) {
	std::mt19937 random{Seed};
	auto const uniform = [&](double const Low, double const High) { return std::uniform_real_distribution<double>{Low, High}(random); };
	auto const pick = [&](std::size_t const Size) { return std::uniform_int_distribution<std::size_t>{0, Size - 1}(random); };
	layout::model model;
	std::vector<layout::network> history{model.state()}; // the states the undoes go back to, the one now last
	REQUIRE(model.execute(three_curves()).done);
	history.push_back(model.state());
	// a second line beside it, with Bloss curves and Bloss cant ramps
	auto const second{model.execute(three_curves(layout::transition_shape::bloss, {0.0, 0.0, -400.0}))};
	CAPTURE(reasons_of(second));
	REQUIRE(second.done);
	history.push_back(model.state());
	check_invariants(model.state());
	std::vector<layout::network> redone;
	std::map<std::string, std::pair<int, int>> tally; // done, refused, by the kind of the command
	std::uint32_t last_line{layout::number(second.lines.front())};
	std::vector<node_id> parted; // the free ends a deletion left, to join again
	for (int step = 0; step < Count; ++step) {
		CAPTURE(step);
		auto const &state{model.state()};
		std::vector<line_id> lines;
		std::vector<line_id> curved;
		for (auto const &[id, line] : state.lines()) {
			lines.push_back(id);
			if (false == line.shape.vertices.empty()) {
				curved.push_back(id);
			}
		}
		std::vector<node_id> free;
		for (auto const &[id, node] : state.nodes()) {
			if (node.ends.size() == 1) {
				free.push_back(id);
			}
		}
		std::erase_if(parted, [&](node_id const Id) { return std::ranges::find(free, Id) == free.end(); });
		auto const roll{uniform(0.0, 1.0)};
		if (roll < 0.10 && history.size() > 1) {
			REQUIRE(model.undo());
			redone.push_back(history.back());
			history.pop_back();
			CHECK(model.state().same(history.back()));
			check_invariants(model.state());
			++tally["undo"].first;
			continue;
		}
		if (roll < 0.14 && false == redone.empty()) {
			REQUIRE(model.redo());
			history.push_back(redone.back());
			redone.pop_back();
			CHECK(model.state().same(history.back()));
			check_invariants(model.state());
			++tally["redo"].first;
			continue;
		}
		REQUIRE_FALSE(lines.empty());
		layout::command command;
		if (roll < 0.48 && false == curved.empty()) {
			// mostly a vertex whose tangents no other line holds at a node: a vertex next to a node holding two ends can't turn the
			// tangent there, the other line keeps it
			std::vector<std::pair<line_id, layout::vertex_id>> movable;
			for (auto const id : curved) {
				auto const &line{*state.find(id)};
				auto const &vertices{line.shape.vertices};
				for (std::size_t i = 0; i < vertices.size(); ++i) {
					auto const held_start{i == 0 && state.find(line.start)->ends.size() > 1};
					auto const held_end{i + 1 == vertices.size() && state.find(line.end)->ends.size() > 1};
					if (false == held_start && false == held_end) {
						movable.emplace_back(id, vertices[i].id);
					}
				}
			}
			auto const &line{*state.find(curved[pick(curved.size())])};
			auto target{std::make_pair(line.id, line.shape.vertices[pick(line.shape.vertices.size())].id)};
			if (false == movable.empty() && uniform(0.0, 1.0) < 0.85) {
				target = movable[pick(movable.size())];
			}
			auto const &owner{*state.find(target.first)};
			auto const &vertex{*std::ranges::find_if(owner.shape.vertices, [&](layout::vertex const &Each) { return Each.id == target.second; })};
			command = layout::move_vertex{owner.id, vertex.id, vertex.point + glm::dvec2{uniform(-12.0, 12.0), uniform(-12.0, 12.0)}};
		}
		else if (roll < 0.74 && false == curved.empty()) {
			auto const &line{*state.find(curved[pick(curved.size())])};
			auto const &vertex{line.shape.vertices[pick(line.shape.vertices.size())]};
			layout::set_vertex set{line.id, vertex.id};
			switch (pick(4)) {
			case 0: set.radius = vertex.radius * uniform(0.8, 1.25); break;
			case 1: set.transition_in = vertex.transition_in * uniform(0.7, 1.4); break;
			case 2: set.transition_out = vertex.transition_out * uniform(0.7, 1.4); break;
			default: set.cant = std::clamp(vertex.cant + uniform(-30.0, 30.0), 0.0, 160.0); break;
			}
			command = set;
		}
		else if (roll < 0.84) {
			auto const &line{*state.find(lines[pick(lines.size())])};
			command = layout::split_line{line.id, uniform(0.0, line.solved.length)};
		}
		else if (roll < 0.94 && free.size() >= 2) {
			// the ends a deletion parted, mostly, else any two
			if (parted.size() == 2 && uniform(0.0, 1.0) < 0.8) {
				command = layout::join_ends{parted[0], parted[1]};
			}
			else {
				auto const a{pick(free.size())};
				auto b{pick(free.size() - 1)};
				b += b >= a ? 1 : 0;
				command = layout::join_ends{free[a], free[b]};
			}
		}
		else if (lines.size() > 2) {
			// a part of a track with no curve of its own, mostly one between two others, which leaves two free ends on a straight to
			// join again; the curves stay to be edited
			std::vector<line_id> straight;
			std::vector<line_id> between;
			for (auto const id : lines) {
				auto const &line{*state.find(id)};
				if (false == line.shape.vertices.empty()) {
					continue;
				}
				straight.push_back(id);
				auto const held = [&](node_id const End) { return state.find(End)->ends.size() == 2 && state.find(End)->curvature == 0.0; };
				if (line.solved.length > 50.0 && held(line.start) && held(line.end)) {
					between.push_back(id);
				}
			}
			if (straight.empty()) {
				continue;
			}
			auto const id{false == between.empty() && uniform(0.0, 1.0) < 0.8 ? between[pick(between.size())] : straight[pick(straight.size())]};
			command = layout::delete_line{id};
		}
		else {
			auto const &line{*state.find(lines[pick(lines.size())])};
			command = layout::split_line{line.id, uniform(0.0, line.solved.length)};
		}
		std::string const kind{layout::command_name(command)};
		auto const before{model.state()};
		auto const result{model.execute(command)};
		if (result.done) {
			++tally[kind].first;
			history.push_back(model.state());
			redone.clear();
			for (auto const id : result.lines) {
				CHECK(layout::number(id) > last_line);
				last_line = std::max(last_line, layout::number(id));
			}
			if (auto const *deleted{std::get_if<layout::delete_line>(&command)}) {
				auto const &gone{*before.find(deleted->line)};
				parted.clear();
				for (auto const end : {gone.start, gone.end}) {
					if (model.state().find(end) != nullptr) {
						parted.push_back(end);
					}
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
	// the edits did edit: each kind of command was kept some times, and some refused
	std::string summary;
	for (auto const &[kind, counts] : tally) {
		summary += kind + " " + std::to_string(counts.first) + "/" + std::to_string(counts.first + counts.second) + ", ";
	}
	CAPTURE(summary);
	MESSAGE("seed " << Seed << ": " << summary);
	CHECK(tally["move vertex"].first >= Count / 30);
	CHECK(tally["set vertex"].first >= Count / 30);
	CHECK(tally["split line"].first >= Count / 30);
	CHECK(tally["join ends"].first >= 1);
	CHECK(tally["delete line"].first >= 1);
	CHECK(tally["undo"].first >= 1);
	CHECK(tally["move vertex"].second + tally["set vertex"].second > 0);
}

} // namespace

TEST_CASE("layout commands P: 300 random edits, seed 20261123: the nodes and the cant run on, no BLOCK passed, a refusal changes nothing") {
	random_edits(20261123u, 300);
}

TEST_CASE("layout commands P: 300 random edits, seed 7: the nodes and the cant run on, no BLOCK passed, a refusal changes nothing") {
	random_edits(7u, 300);
}

TEST_CASE("layout commands P: 300 random edits, seed 450: the nodes and the cant run on, no BLOCK passed, a refusal changes nothing") {
	random_edits(450u, 300);
}
