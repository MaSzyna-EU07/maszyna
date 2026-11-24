// the pieces of the model the editor's tools edit it through (editor/layout, stage E4b; editor-tracks-domain.md §8): the chainage of a point
// on a line, a vertex moved and changed in one command, a line laid on from the free end of another one sharing its node, the line alongside
// a line, its arcs concentric and the spacing in its curves widened; and the project session making the tracks of a turnout and recording
// the tracks of a line written into more files than one

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"
#include "scratch.h"

#include "editor/editorGauge.hpp"
#include "editor/layout/layoutParallel.hpp"
#include "editor/layout/layoutTurnout.hpp"
#include "editor/project/projectGenerated.hpp"
#include "editor/project/projectSession.hpp"

#include <cmath>
#include <set>
#include <sstream>

namespace {

using namespace test::layout_fixture;
using layout::line_id;
using layout::node_id;

layout::create_line straight_along_z(double const X, double const From, double const To) {
	layout::create_line result;
	result.start = {X, 10.0, From};
	result.end = {X, 10.0, To};
	result.category = line_class();
	return result;
}

line_id last_line(layout::model const &Model) {
	REQUIRE_FALSE(Model.state().lines().empty());
	return std::prev(Model.state().lines().end())->first;
}

// the free node of the line at the point
node_id node_at(layout::network const &Network, glm::dvec3 const &Point) {
	for (auto const &[id, node] : Network.nodes()) {
		if (glm::distance(node.position, Point) < 1e-9) {
			return id;
		}
	}
	FAIL("no node there");
	return node_id::none;
}

} // namespace

TEST_CASE("layout editing: the chainage of a point of the plan on a line, and how far the point is from it") {
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const &line{model.state().lines().begin()->second};
	auto const &solved{line.solved};
	for (double const at : {0.0, 12.5, 333.3, 0.5 * solved.length, solved.length - 0.25, solved.length}) {
		auto const point{solved.position(at)};
		double distance{-1.0};
		auto const found{layout::chainage_of(solved, point, &distance)};
		CHECK(found == doctest::Approx(at).epsilon(1e-6));
		CHECK(distance < 1e-6);
		// a point 2 m aside: the same chainage, 2 m off
		auto const aside{point + layout::left_of(solved.tangent(at)) * 2.0};
		if (at > 1.0 && at < solved.length - 1.0) {
			CHECK(layout::chainage_of(solved, aside, &distance) == doctest::Approx(at).epsilon(1e-5));
			CHECK(distance == doctest::Approx(2.0).epsilon(1e-6));
		}
	}
	layout::solved_line const empty;
	double distance{0.0};
	CHECK(layout::chainage_of(empty, {0.0, 0.0}, &distance) == 0.0);
	CHECK(std::isinf(distance));
}

TEST_CASE("layout editing: a vertex moved and its curve changed by one set_vertex, one step") {
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const id{model.state().lines().begin()->first};
	auto const vertex{model.state().find(id)->shape.vertices[1]};
	auto const steps{model.undoable()};
	layout::set_vertex set{id, vertex.id, vertex.radius + 100.0};
	set.point = vertex.point + glm::dvec2{0.0, 5.0};
	auto const done{model.execute(set)};
	REQUIRE_MESSAGE(done.done, reasons_of(done));
	CHECK(model.undoable() == steps + 1);
	auto const &now{model.state().find(id)->shape.vertices[1]};
	CHECK(now.point == vertex.point + glm::dvec2{0.0, 5.0});
	CHECK(now.radius == vertex.radius + 100.0);
	REQUIRE(model.undo());
	CHECK(model.state().find(id)->shape.vertices[1] == vertex);
}

TEST_CASE("layout editing: a line laid on from the free end of another one shares its node, its ends at the node's numbers") {
	layout::model model;
	REQUIRE(model.execute(straight_along_z(0.0, 0.0, 400.0)).done);
	auto const first{last_line(model)};
	auto const end{node_at(model.state(), {0.0, 10.0, 400.0})};

	// on from the end, northward: the node holds both lines, one ahead and one behind
	auto on{straight_along_z(0.0, 400.0, 700.0)};
	on.start_node = end;
	auto const laid{model.execute(on)};
	REQUIRE_MESSAGE(laid.done, reasons_of(laid));
	auto const second{last_line(model)};
	CHECK(model.state().nodes().size() == 3);
	auto const *node{model.state().find(end)};
	REQUIRE(node != nullptr);
	CHECK(node->ends.size() == 2);
	CHECK(model.state().find(second)->start == end);
	CHECK(model.state().find(second)->solved.start.position == node->position);
	CHECK(model.state().find(first)->solved.end.position == node->position);
	check_structure(model.state());
	CHECK(layout::check_all(model.state()).fine());

	// back from the start of the first one, southward: against the heading of its node
	auto const start{node_at(model.state(), {0.0, 10.0, 0.0})};
	auto back{straight_along_z(0.0, 0.0, -300.0)};
	back.start_node = start;
	auto const behind{model.execute(back)};
	REQUIRE_MESSAGE(behind.done, reasons_of(behind));
	CHECK(model.state().find(start)->ends.size() == 2);
	CHECK(layout::check_all(model.state()).fine());

	// a node with two ends is no free end; a kink at the node is refused, the network as it was
	auto again{straight_along_z(5.0, 400.0, 900.0)};
	again.start_node = end;
	CHECK_FALSE(model.execute(again).done);
	auto const &farthest{*std::prev(model.state().nodes().end())};
	auto kinked{straight_along_z(0.0, 700.0, 900.0)};
	kinked.end = {50.0, 10.0, 900.0};
	kinked.start_node = farthest.first;
	auto const before{model.state()};
	CHECK_FALSE(model.execute(kinked).done);
	CHECK(model.state().same(before));

	// undone: the node free again
	REQUIRE(model.undo());
	REQUIRE(model.undo());
	CHECK(model.state().find(end)->ends.size() == 1);
	CHECK(model.state().nodes().size() == 2);
}

TEST_CASE("layout editing: the line alongside a straight and alongside the curves, the arcs concentric, the spacing widened in the curves") {
	layout::model model;
	REQUIRE(model.execute(straight_along_z(0.0, 0.0, 400.0)).done);
	auto const straight{last_line(model)};
	// to the left of a line running along z: toward -x
	auto const beside{layout::parallel_line(model.state(), straight, 4.5)};
	REQUIRE(beside.valid);
	CHECK(beside.command.start == glm::dvec3{-4.5, 10.0, 0.0});
	CHECK(beside.command.end == glm::dvec3{-4.5, 10.0, 400.0});
	CHECK(beside.command.vertices.empty());
	CHECK_FALSE(layout::parallel_line(model.state(), straight, 0.0).valid);
	CHECK_FALSE(layout::parallel_line(model.state(), line_id{99}, 4.0).valid);

	REQUIRE(model.execute(three_curves()).done);
	auto const curves{last_line(model)};
	auto const &source{*model.state().find(curves)};
	for (double const offset : {4.0, -4.0}) {
		auto const plan{layout::parallel_line(model.state(), curves, offset)};
		REQUIRE(plan.valid);
		REQUIRE(plan.command.vertices.size() == source.shape.vertices.size());
		REQUIRE(plan.spacings.size() == source.shape.vertices.size());
		auto copy{model};
		auto const laid{copy.execute(plan.command)};
		REQUIRE_MESSAGE(laid.done, reasons_of(laid));
		auto const &alongside{*copy.state().find(last_line(copy))};
		for (std::size_t i = 0; i < source.shape.vertices.size(); ++i) {
			auto const &vertex{source.shape.vertices[i]};
			auto const &made{plan.command.vertices[i]};
			// the inside of the curve the side it turns to: the radius less by the spacing and the widening of both gauges
			auto const inner{std::abs(made.radius) < vertex.radius};
			auto const widening{gauge::curve_widening(inner ? made.radius : vertex.radius, false) + gauge::curve_widening(inner ? vertex.radius : made.radius, true)};
			CHECK(plan.spacings[i] == doctest::Approx(4.0 + widening).epsilon(1e-3));
			CHECK(std::abs(made.radius - vertex.radius) == doctest::Approx(4.0 + widening).epsilon(1e-6));
			CHECK(made.cant == vertex.cant);
			// the arcs concentric: the middle of the arc alongside as far from the line's as the spacing in the curve, within 2 mm
			double middle{-1.0};
			for (auto const &e : source.solved.elements) {
				if (e.role == layout::element_role::arc && e.vertex == layout::number(vertex.id)) {
					middle = e.chainage + 0.5 * e.length;
				}
			}
			REQUIRE(middle >= 0.0);
			double distance{0.0};
			layout::chainage_of(alongside.solved, source.solved.position(middle), &distance);
			CHECK(distance == doctest::Approx(plan.spacings[i]).epsilon(0.002 / 4.0));
		}
		// on the straights, the spacing as asked
		double distance{0.0};
		layout::chainage_of(alongside.solved, source.solved.position(5.0), &distance);
		CHECK(distance == doctest::Approx(4.0).epsilon(1e-6));
	}
}

TEST_CASE("layout editing: alongside a curve under 250 m the spacing is widened by the gauges of both tracks, on the inside by a longer transition") {
	// a siding of 30 km/h with a curve of R 200 m and transitions of 30 m: the gauge widens there (PLK tom II annex 1)
	layout::create_line sharp;
	sharp.start = {0.0, 0.0, 0.0};
	sharp.end = {400.0, 0.0, 400.0};
	sharp.category = line_class();
	sharp.category.vmax = 30.0;
	sharp.category.track = norms::track_kind::side;
	layout::vertex corner;
	corner.point = {0.0, 400.0};
	corner.radius = 200.0;
	corner.transition_in = corner.transition_out = 30.0;
	sharp.vertices = {corner};
	layout::model model;
	auto const laid{model.execute(sharp)};
	REQUIRE_MESSAGE(laid.done, reasons_of(laid));
	auto const id{last_line(model)};
	auto const &source{*model.state().find(id)};
	// the curve turns from +z to +x: to the left of the line (above 0) is the outside of it
	for (double const offset : {-4.0, 4.0}) {
		auto const plan{layout::parallel_line(model.state(), id, offset)};
		REQUIRE(plan.valid);
		REQUIRE(plan.spacings.size() == 1);
		auto const inner{plan.command.vertices.front().radius < 200.0};
		// of the inner track on its outer side and of the outer one on its inner side, at their radii the spacing apart
		auto const widening{inner ? gauge::curve_widening(196.0, false) + gauge::curve_widening(200.0, true) : gauge::curve_widening(200.0, false) + gauge::curve_widening(204.0, true)};
		CHECK(widening > 0.1);
		CHECK(plan.spacings.front() == doctest::Approx(4.0 + widening).epsilon(1e-6));
		CHECK(plan.notes.empty());
		// on the inside the transitions longer, keeping the arc concentric nearer the line; on the outside shorter
		CHECK((plan.command.vertices.front().transition_in > 30.0) == inner);
		auto copy{model};
		auto const alongside{copy.execute(plan.command)};
		REQUIRE_MESSAGE(alongside.done, reasons_of(alongside));
		auto const &made{copy.state().find(last_line(copy))->solved};
		double middle{0.0};
		for (auto const &e : source.solved.elements) {
			if (e.role == layout::element_role::arc) {
				middle = e.chainage + 0.5 * e.length;
			}
		}
		double distance{0.0};
		layout::chainage_of(made, source.solved.position(middle), &distance);
		CHECK(distance == doctest::Approx(plan.spacings.front()).epsilon(0.003 / 4.0));
	}
	// with no transitions to take it, the spacing isn't widened, and told why
	auto plain{sharp};
	plain.vertices.front().transition_in = plain.vertices.front().transition_out = 0.0;
	plain.start.x += 1000.0;
	plain.end.x += 1000.0;
	plain.vertices.front().point.x += 1000.0;
	layout::model other;
	auto const plain_laid{other.execute(plain)};
	if (plain_laid.done) {
		auto const plan{layout::parallel_line(other.state(), last_line(other), -4.0)};
		REQUIRE(plan.valid);
		CHECK(plan.spacings.front() == doctest::Approx(4.0));
		CHECK(plan.notes.size() == 1);
	}
}

TEST_CASE("project session: the switch track of a turnout made by the session, the tracks of a line written into two files recorded") {
	test::scratch_folder folder{"layout_editing_session"};
	folder.write("mapa.scn", "// a new scenery\n");
	project::session session;
	REQUIRE(session.open(folder.root(), "mapa.scn").done);
	auto const laid{session.execute(straight_along_z(0.0, 0.0, 400.0))};
	REQUIRE(laid.outcome.done);
	auto const line{*laid.changed.begin()};
	auto const types{layout::turnout_types()};
	REQUIRE_FALSE(types.empty());
	auto const inserted{session.execute(layout::insert_turnout{line, 100.0, "Rz 60E1-300-1:9", 1, 1})};
	REQUIRE_MESSAGE(inserted.outcome.done, reasons_of(inserted.outcome));
	REQUIRE(session.network().turnouts().size() == 1);
	auto const &turnout{session.network().turnouts().begin()->second};
	auto const made{session.generate(turnout.id)};
	CHECK(made.errors.empty());
	CHECK(made.text.find(" track switch ") != std::string::npos);
	// a track of the turnout isn't made as a line of its own
	CHECK_FALSE(session.generate(turnout.main).errors.empty());

	// the tracks of the line written into two files, the switch track by its main track: all of them recorded, none taken for a change
	// by hand
	auto const pieces{session.generate(line).pieces};
	REQUIRE_FALSE(pieces.empty());
	std::vector<project::session::written_piece> written;
	for (std::size_t i = 0; i < pieces.size(); ++i) {
		written.push_back({pieces[i].name, pieces[i].text, "mapa.scn"});
	}
	// as a track of it written into another file of the map
	written.push_back({pieces.front().name + "_b", pieces.front().text, "mapa_tory.scm"});
	session.record_written(line, written);
	session.record_written(turnout.main, {{made.name, made.text, "mapa_tory.scm", "switch"}});
	std::set<std::string> files;
	std::size_t switches{0};
	for (auto const &node : session.project().generated) {
		if (node.line == line) {
			files.insert(node.file);
		}
		if (node.kind == "switch") {
			++switches;
			CHECK(node.line == turnout.main);
		}
	}
	CHECK(files == std::set<std::string>{"mapa.scn", "mapa_tory.scm"});
	CHECK(switches == 1);
	// written again: the records of the line give way to the new ones, whatever file they were in
	session.record_written(line, {{pieces.front().name, pieces.front().text, "mapa.scn"}});
	std::size_t records{0};
	for (auto const &node : session.project().generated) {
		records += node.line == line ? 1 : 0;
	}
	CHECK(records == 1);
}

TEST_CASE("project generated: the entry of a track of the generator moved for a file under an origin, its control vectors as they were") {
	layout::model model;
	REQUIRE(model.execute(three_curves()).done);
	auto const line{model.state().lines().begin()->first};
	auto const made{layout::generate(model.state(), line)};
	REQUIRE(made.errors.empty());
	auto const &piece{made.pieces[1]};
	glm::dvec3 const shift{2000.0, -5.0, 300.25};
	auto const moved{project::moved_entry(piece.text, shift)};
	REQUIRE(moved.has_value());
	auto const words = [](std::string const &Text) {
		std::vector<std::string> result;
		std::istringstream stream{Text};
		for (std::string word; stream >> word;) {
			result.push_back(word);
		}
		return result;
	};
	auto const before{words(piece.text)};
	auto const after{words(*moved)};
	REQUIRE(before.size() == after.size());
	// node -1 0 name track normal length, 13 words of the surface, then the path: the start and its roll, two control vectors, the end
	std::size_t const path{7 + 13};
	for (std::size_t i = 0; i < before.size(); ++i) {
		auto const offset{i >= path ? i - path : 99};
		auto const axis{offset < 3 ? offset : offset >= 10 && offset < 13 ? offset - 10 : 99};
		if (axis < 3) {
			CHECK(std::stod(after[i]) == std::stod(before[i]) - shift[static_cast<glm::length_t>(axis)]);
		}
		else {
			CHECK(after[i] == before[i]);
		}
	}
	// not moved: the numbers as the generator wrote them, the checksum of the text the same
	auto const same{project::moved_entry(piece.text, glm::dvec3{0.0})};
	REQUIRE(same.has_value());
	CHECK(project::checksum(*same) == project::checksum(piece.text));
	// not an entry of the generator's
	CHECK_FALSE(project::moved_entry("node -1 0 x model 0 0 0 0 a.t3d none", shift).has_value());
}
