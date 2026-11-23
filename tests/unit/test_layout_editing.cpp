// the pieces of the model the editor's tools edit it through (editor/layout, stage E4b; editor-tracks-domain.md §8): the chainage of a point
// on a line, a vertex moved and changed in one command, a line laid on from the free end of another one sharing its node, the line alongside
// a line, its arcs concentric and the spacing in its curves widened; and the project session making the tracks of a turnout and recording
// the tracks of a line written into more files than one

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"
#include "scratch.h"

#include "editor/layout/layoutTurnout.hpp"
#include "editor/project/projectSession.hpp"

#include <cmath>
#include <set>

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
