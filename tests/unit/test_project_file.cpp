// the project file of the track geometry (editor/project/projectFile.cpp, editor-tracks-domain.md §9): written, read and written again it
// comes out the same, to the bit of every number, the turnouts and their diverging tracks too; the golden file of data/project/ read and
// written back the same; a file of a newer schema refused cleanly, and one which isn't a project file or is broken; the identifiers never
// handed out again, after deletions and a save and a load

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"
#include "scratch.h"
#include "editor/project/projectFile.hpp"

#include <format>
#include <fstream>
#include <iterator>
#include <string>

namespace {

using namespace test::layout_fixture;
using layout::line_id;
using layout::node_id;

std::string file_text(std::string const &Path) {
	std::ifstream file{Path, std::ios::binary};
	REQUIRE(file.is_open());
	return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::string golden_path() {
	return std::string{EU07_UNIT_TESTS_DATA} + "/project/golden.eu07proj";
}

// the line ends of the file as git may have checked them out
std::string unix_lines(std::string Text) {
	std::erase(Text, '\r');
	return Text;
}

std::string notes_of(project::read_result const &Read) {
	std::string result{Read.error};
	for (auto const &note : Read.notes) {
		result += "; " + note;
	}
	return result;
}

// two lines and the state of a project around them: a line of three curves, a straight one of another class going on from its end,
// one deleted between them, a profile for an area and one for a line, generated tracks, //$ lines kept
layout::model edited_model() {
	norms::area_map areas;
	areas.scenery = norms::profile_id::plk;
	areas.areas.push_back({"granica", norms::profile_id::ch, {{-500.0, 0.0}, {-100.0, 0.0}, {-100.0, 400.0}}});
	areas.lines.emplace_back("91", norms::profile_id::tsi);
	layout::model model{areas};
	REQUIRE(model.execute(three_curves()).done);
	layout::create_line spare;
	spare.start = {0.0, 0.0, 5000.0};
	spare.end = {0.0, 0.0, 5400.0};
	spare.category = line_class();
	REQUIRE(model.execute(spare).done);
	layout::create_line straight;
	straight.start = {3000.0, 104.0, 600.0};
	straight.end = {3500.0, 105.0, 600.0};
	straight.category = line_class();
	straight.category.vmax = 120.0;
	straight.category.kind = norms::traffic::passenger;
	straight.chainage_start = 2345.678;
	REQUIRE(model.execute(straight).done);
	// the spare one taken away again: its identifiers stay handed out
	REQUIRE(model.execute(layout::delete_line{line_id{2}}).done);
	return model;
}

project::document document_of(layout::model const &Model) {
	project::document result;
	result.network = Model.state();
	result.generated.push_back({"track", "l1_s11", line_id{1}, "trasa/tory.scm", 0x0123456789abcdefull});
	result.generated.push_back({"track", "l3_se1", line_id{3}, "trasa/tory nowe.scm", 0xffull});
	result.hand_changed.insert(line_id{3});
	result.legacy.push_back({'h', "trasa/tory.scm", line_id{1}, {}, {"alignment 0.0000 0.0000 0.0000 1.0000 0.0000 1.0000", "", "end"}});
	result.legacy.push_back({'r', "trasa/tory.scm", line_id::none, "", {"# switches placed in the track editor, the simulation skips these lines", "switch z1 Rz 60E1 500.000"}});
	result.legacy.push_back({'h', "trasa/tory.scm", line_id::none, "a compound curve", {"alignment 1 2 3 4 5 6", "end"}});
	return result;
}

} // namespace

TEST_CASE("project file: written, read and written again it's the same text, the lines and the nodes the same to the bit") {
	auto const model{edited_model()};
	auto const project{document_of(model)};
	auto const text{project::write(project)};
	CAPTURE(text);
	CHECK(text.starts_with("eu07proj 1\n"));
	auto const read{project::read(text)};
	REQUIRE_MESSAGE(read.project.has_value(), notes_of(read));
	CHECK_MESSAGE(read.notes.empty(), notes_of(read));
	// the lines laid again between the nodes as read are the ones the model laid: the solved lines are compared too
	CHECK(read.project->network.same(model.state()));
	CHECK(project::same(*read.project, project));
	CHECK(read.project->network.handed() == model.state().handed());
	CHECK(project::write(*read.project) == text);
	// the class of each line, by value
	CHECK(read.project->network.find(line_id{3})->category.vmax == 120.0);
	CHECK(read.project->network.find(line_id{3})->category.kind == norms::traffic::passenger);
	CHECK(read.project->network.find(line_id{3})->chainage_start == 2345.678);
	CHECK(read.project->network.areas.areas.size() == 1);
	CHECK(read.project->network.areas.at({-300.0, 100.0}, {}) == norms::profile_id::ch);
	CHECK(read.project->network.areas.at({0.0, 0.0}, "91") == norms::profile_id::tsi);
}

TEST_CASE("project file: the golden file read and written back comes out the same; what it holds") {
	auto const golden{unix_lines(file_text(golden_path()))};
	auto const read{project::read(golden)};
	REQUIRE_MESSAGE(read.project.has_value(), notes_of(read));
	CHECK_MESSAGE(read.notes.empty(), notes_of(read));
	auto const &project{*read.project};
	CHECK(project::write(project) == golden);
	auto const &network{project.network};
	CHECK(network.nodes().size() == 3);
	CHECK(network.lines().size() == 2);
	CHECK(network.handed() == layout::network::handed_out{5, 3, 2});
	auto const *curved{network.find(line_id{3})};
	REQUIRE(curved != nullptr);
	CHECK(curved->solved.valid());
	CHECK(curved->shape.transition == layout::transition_shape::cubic_parabola);
	CHECK(curved->shape.ramp == layout::ramp_shape::bloss);
	REQUIRE(curved->shape.vertices.size() == 1);
	CHECK(curved->shape.vertices.front().radius == 300.0);
	REQUIRE(curved->shape.grades.size() == 1);
	CHECK(curved->shape.grades.front().vertical_radius == 5000.0);
	CHECK(curved->category.track == norms::track_kind::side);
	CHECK(curved->category.features == norms::feature::platform);
	CHECK_FALSE(curved->profile.has_value());
	CHECK(network.find(line_id{1})->profile == std::optional<norms::profile_id>{norms::profile_id::tsi});
	// the ends of the lines at their nodes, the line laid from them
	CHECK(curved->solved.start.position == network.find(node_id{2})->position);
	CHECK(curved->solved.end.position == network.find(node_id{5})->position);
	CHECK(network.areas.areas.size() == 2);
	CHECK(network.areas.areas.back().name == "przejscie graniczne");
	REQUIRE(project.generated.size() == 3);
	CHECK(project.generated.back().file == "trasa/tory nowe.scm");
	CHECK(project.generated.front().sum == 0x0123456789abcdefull);
	CHECK(project.hand_changed == std::set<line_id>{line_id{3}});
	CHECK(network.turnouts().empty());
	CHECK(project.anchors.empty());
	REQUIRE(project.legacy.size() == 3);
	CHECK(project.legacy[1].why == "a compound curve");
	CHECK(project.legacy[1].lines == std::vector<std::string>{"alignment 1.0000 2.0000 3.0000 4.0000 5.0000 6.0000", "", "end"});
	CHECK(project.legacy[2].mark == 'r');
	// the line ends of windows read the same
	auto windows{golden};
	for (std::size_t at = 0; (at = windows.find('\n', at)) != std::string::npos; at += 2) {
		windows.insert(at, 1, '\r');
	}
	auto const again{project::read(windows)};
	REQUIRE(again.project.has_value());
	CHECK(project::same(*again.project, project));
}

TEST_CASE("project file: the turnouts and their tracks written and read back, the diverging tracks laid by them the same to the bit") {
	layout::model model;
	layout::create_line straight;
	straight.start = {0.0, 100.0, 0.0};
	straight.end = {1000.0, 104.0, 0.0};
	straight.category = line_class();
	REQUIRE(model.execute(straight).done);
	auto const line{model.state().lines().begin()->first};
	// a 1:12 to the left along the line, a 1:9 to the right against it on the line going on from it; the line a profile of its own
	auto const first{model.execute(layout::insert_turnout{line, 300.0, "Rz 60E1-500-1:12", 1, 1})};
	REQUIRE_MESSAGE(first.done, reasons_of(first));
	auto const second{model.execute(layout::insert_turnout{first.lines[1], 200.0, "Rz 60E1-190-1:9", -1, -1})};
	REQUIRE_MESSAGE(second.done, reasons_of(second));
	REQUIRE(model.execute(layout::set_line_profile{line, norms::profile_id::tsi}).done);
	// the network as settle() leaves it: the model lays the diverging track of the second turnout before the node at its start takes the
	// grade of the line laid beyond it, the grade of the track 1e-16 off the one laid again from the node (E2b); the file lays it from the
	// nodes as they are
	auto settled{model.state()};
	{
		layout::transaction change{settled};
		for (auto const &[id, each] : settled.turnouts()) {
			change.change(id);
		}
		layout::settle(change);
	}
	project::document project;
	project.network = settled;
	auto const text{project::write(project)};
	CAPTURE(text);
	auto const read{project::read(text)};
	REQUIRE_MESSAGE(read.project.has_value(), notes_of(read));
	CHECK_MESSAGE(read.notes.empty(), notes_of(read));
	auto const &network{read.project->network};
	CHECK(network.turnouts().size() == 2);
	// the turnouts, their tracks of them, the diverging ones laid off their main tracks: the same to the bit
	CHECK(network.same(settled));
	CHECK(network.handed() == model.state().handed());
	CHECK(network.handed().turnout == 2);
	CHECK(network.find(line)->profile == std::optional<norms::profile_id>{norms::profile_id::tsi});
	auto const &turnout{network.turnouts().begin()->second};
	CHECK(turnout.type == "Rz 60E1-500-1:12");
	CHECK(network.find(turnout.diverging)->turnout == turnout.id);
	CHECK(network.find(turnout.diverging)->solved.valid());
	CHECK(project::write(*read.project) == text);
	// the model's own state written, read and written again: the same text
	project.network = model.state();
	auto const own{project::write(project)};
	auto const own_read{project::read(own)};
	REQUIRE(own_read.project.has_value());
	CHECK(project::write(*own_read.project) == own);
	// a turnout whose track isn't its, a line of a turnout which hasn't it: refused
	auto broken{text};
	auto const owned{broken.find(std::format("turnout {} profile", layout::number(turnout.id)))};
	REQUIRE(owned != std::string::npos);
	broken.replace(owned, 9, "turnout 7");
	CHECK_FALSE(project::read(broken).project.has_value());
}

TEST_CASE("project file: the section kept for the anchors is written back as it was read") {
	auto text{unix_lines(file_text(golden_path()))};
	auto const anchors{text.find("anchors\nend\n")};
	REQUIRE(anchors != std::string::npos);
	text.replace(anchors, 12, "anchors\nanchor 1 line 1 chainage 120.5 offset -3.2\nend\n");
	auto const read{project::read(text)};
	REQUIRE_MESSAGE(read.project.has_value(), notes_of(read));
	CHECK(read.project->anchors == std::vector<std::string>{"anchor 1 line 1 chainage 120.5 offset -3.2"});
	CHECK(project::write(*read.project) == text);
	// a record the turnouts haven't
	auto turnouts{text};
	turnouts.replace(turnouts.find("turnouts\nend\n"), 13, "turnouts\nturnout 1 type Rz%2060E1-500-1:12 at 0 0 0\nend\n");
	CHECK_FALSE(project::read(turnouts).project.has_value());
}

TEST_CASE("project file: a newer schema is refused cleanly, and a file which isn't a project file or is broken") {
	auto const golden{unix_lines(file_text(golden_path()))};
	auto with = [&](std::string const &What, std::string const &Instead) {
		auto text{golden};
		auto const at{text.find(What)};
		REQUIRE(at != std::string::npos);
		text.replace(at, What.size(), Instead);
		return project::read(text);
	};
	// a newer version: nothing of it read, the reason told
	auto const newer{with("eu07proj 1\n", "eu07proj 2\n")};
	CHECK_FALSE(newer.project.has_value());
	CHECK(newer.error.find("version 2") != std::string::npos);
	CHECK(newer.error.find("newer") != std::string::npos);
	// a newer one with records this one doesn't know isn't read past its first line either
	auto const future{project::read("eu07proj 7\nturnout 1 whatever\n")};
	CHECK_FALSE(future.project.has_value());
	CHECK(future.error.find("version 7") != std::string::npos);
	CHECK_FALSE(project::read("").project.has_value());
	CHECK_FALSE(project::read("node 1 at 0 0 0\n").project.has_value());
	CHECK_FALSE(with("eu07proj 1\n", "eu07proj 0\n").project.has_value());
	CHECK_FALSE(with("eu07proj 1\n", "eu07proj one\n").project.has_value());
	// a record not of this version, a number which isn't one, a node given twice, a line whose end no node holds, a section not ended
	auto const unknown{with("norms plk\n", "norms plk\nswitch z1\n")};
	CHECK_FALSE(unknown.project.has_value());
	CHECK(unknown.error.starts_with("line 6:"));
	CHECK_FALSE(with("radius 300", "radius 3oo").project.has_value());
	CHECK_FALSE(with("node 5 at", "node 2 at").project.has_value());
	CHECK_FALSE(with("ends 2 1 end 1 3 start 1", "ends 1 1 end 1").project.has_value());
	CHECK_FALSE(with("class 2 vmax", "class 7 vmax").project.has_value());
	CHECK_FALSE(with("norms plk", "norms din").project.has_value());
	auto cut{golden};
	cut.erase(cut.rfind("end\n"));
	CHECK_FALSE(project::read(cut).project.has_value());
	// next lower than the identifiers in use: taken past them, told
	auto const lower{with("next node 6 line 4 vertex 3 turnout 1", "next node 1 line 1 vertex 1 turnout 1")};
	REQUIRE(lower.project.has_value());
	CHECK(lower.project->network.handed() == layout::network::handed_out{5, 3, 2});
	CHECK_FALSE(lower.notes.empty());
	// next 0: the identifiers start at 1, so it isn't taken as the last one handed out wrapped round to 0xffffffff
	auto const zero{with("next node 6 line 4 vertex 3 turnout 1", "next node 0 line 4 vertex 3 turnout 1")};
	CHECK_FALSE(zero.project.has_value());
	CHECK(zero.error.find("the next node 0") != std::string::npos);
}

TEST_CASE("project file: the identifiers aren't handed out again after deletions and a save and a load") {
	layout::model model{norms::area_map{}};
	auto const first{model.execute(three_curves())};
	REQUIRE(first.done);
	layout::create_line straight;
	straight.start = {0.0, 0.0, 5000.0};
	straight.end = {0.0, 0.0, 5400.0};
	straight.vertices = {curve({20.0, 5200.0}, 600.0)};
	straight.category = line_class();
	auto const second{model.execute(straight)};
	REQUIRE(second.done);
	auto const handed{model.state().handed()};
	CHECK(handed == layout::network::handed_out{4, 2, 4});
	// the last ones made taken away: their numbers are past the highest in use
	REQUIRE(model.execute(layout::delete_line{second.lines.front()}).done);
	CHECK(model.state().lines().size() == 1);
	CHECK(model.state().nodes().size() == 2);

	test::scratch_folder folder{"project_ids"};
	auto const path{folder.root() / "mapa.eu07proj"};
	project::document saved;
	saved.network = model.state();
	std::string error;
	REQUIRE_MESSAGE(project::save(saved, path, error), error);
	auto const loaded{project::load(path)};
	REQUIRE_MESSAGE(loaded.project.has_value(), notes_of(loaded));
	CHECK(loaded.project->network.handed() == handed);

	// the next ones past those handed out, not past the highest in use
	auto again{loaded.project->network};
	CHECK(again.new_line() == line_id{3});
	CHECK(again.new_node() == node_id{5});
	CHECK(again.new_node() == node_id{6});
	CHECK(layout::number(again.new_vertex()) == 5);
	// handed out and not used, as by a command refused: kept handed out too; written over, the new file in the place of the old one
	saved.network = again;
	REQUIRE_MESSAGE(project::save(saved, path, error), error);
	auto const reloaded{project::load(path)};
	REQUIRE(reloaded.project.has_value());
	CHECK(reloaded.project->network.handed() == layout::network::handed_out{6, 3, 5});
	CHECK_FALSE(std::filesystem::exists(path.string() + ".new"));
	CHECK_FALSE(project::load(folder.root() / "none.eu07proj").project.has_value());
	CHECK(project::path_of("scenery/l27/l27.scm") == std::filesystem::path{"scenery/l27/l27.eu07proj"});
}

TEST_CASE("project file: the words of the names and the files") {
	CHECK(project::word("tory nowe.scm") == "tory%20nowe.scm");
	CHECK(project::word("100%") == "100%25");
	CHECK(project::word("") == "-");
	CHECK(project::word("-") == "%2D");
	for (std::string const text : {"tory nowe.scm", "100%", "", "-", "a%20b", "tab\there"}) {
		CHECK(project::unword(project::word(text)) == text);
	}
}
