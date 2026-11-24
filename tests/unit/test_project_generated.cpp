// the tracks the generator wrote, kept in the project file by their names with the checksums of their text (editor/project/
// projectGenerated.cpp, editor-tracks-domain.md §9): the text of a track of one line edited outside the editor marks that line changed by
// hand and no other, a track taken away too; the blanks, the line ends and the comments another tool writes aren't a change; the line
// written again by the editor isn't marked any more

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"
#include "editor/project/projectGenerated.hpp"
#include "editor/project/sceneryText.hpp"

#include <map>
#include <optional>
#include <string>

namespace {

using namespace test::layout_fixture;
using layout::line_id;

std::string const file_name{"trasa/tory.scm"};

struct generated_scenery {
	layout::model model{norms::area_map{}};
	project::document project;
	std::string text; // of the file the tracks are written into
	std::map<line_id, layout::generated_line> lines;
};

// two lines, their tracks written into one file among other nodes, the project knowing them
generated_scenery scenery() {
	generated_scenery result;
	REQUIRE(result.model.execute(three_curves()).done);
	layout::create_line straight;
	straight.start = {0.0, 0.0, 5000.0};
	straight.end = {0.0, 0.0, 5400.0};
	straight.category = line_class();
	REQUIRE(result.model.execute(straight).done);
	result.text = "// the tracks of the tests of the checksums\nnode -1 0 obcy track normal 10 1.435 0.15 25 20 0 flat unvis 0 0 0 0 0 0 0 0 0 0 0 10 0 0 endtrack\n";
	for (auto const id : {line_id{1}, line_id{2}}) {
		auto made{layout::generate(result.model.state(), id)};
		REQUIRE(made.errors.empty());
		REQUIRE_FALSE(made.pieces.empty());
		result.text += made.text();
		project::record_generated(result.project, made, file_name);
		result.lines.emplace(id, std::move(made));
	}
	result.text += "event obcy_event multiple 0 none endevent\n";
	result.project.network = result.model.state();
	return result;
}

std::vector<project::hand_change> changes_in(project::document const &Project, std::string const &Text) {
	return project::find_hand_changes(Project, [&](std::string const &File) -> std::optional<std::string> {
		if (File == file_name) {
			return Text;
		}
		return std::nullopt;
	});
}

} // namespace

TEST_CASE("project generated: the tracks as written aren't changed; one edited marks its line changed by hand, the other line not") {
	auto scenery{::scenery()};
	auto &project{scenery.project};
	CHECK(project.generated.size() == scenery.lines.at(line_id{1}).pieces.size() + scenery.lines.at(line_id{2}).pieces.size());
	CHECK(changes_in(project, scenery.text).empty());
	// kept through the project file
	auto const read{project::read(project::write(project))};
	REQUIRE(read.project.has_value());
	CHECK(read.project->generated == project.generated);
	CHECK(changes_in(*read.project, scenery.text).empty());

	// the speed of a track of the second line changed by hand
	auto const &piece{scenery.lines.at(line_id{2}).pieces.front()};
	auto edited{scenery.text};
	auto const at{edited.find(piece.text)};
	REQUIRE(at != std::string::npos);
	auto changed{piece.text};
	auto const velocity{changed.rfind("velocity ")};
	REQUIRE(velocity != std::string::npos);
	changed.replace(velocity, std::string::npos, "velocity 40 endtrack\n");
	edited.replace(at, piece.text.size(), changed);
	auto const changes{changes_in(project, edited)};
	REQUIRE(changes.size() == 1);
	CHECK(changes.front().line == line_id{2});
	CHECK(changes.front().name == piece.name);
	CHECK(changes.front().file == file_name);
	CHECK_FALSE(changes.front().missing);

	auto const marked{project::mark_hand_changes(project, changes)};
	CHECK(marked == std::set<line_id>{line_id{2}});
	CHECK(project::may_overwrite(project, line_id{1}));
	CHECK_FALSE(project::may_overwrite(project, line_id{2}));
	// marked once
	CHECK(project::mark_hand_changes(project, changes).empty());
	// the mark kept by the project file, until the editor writes the line again: the author decided
	auto const kept{project::read(project::write(project))};
	REQUIRE(kept.project.has_value());
	CHECK(kept.project->hand_changed == std::set<line_id>{line_id{2}});
	project::record_generated(project, scenery.lines.at(line_id{2}), file_name);
	CHECK(project::may_overwrite(project, line_id{2}));
	CHECK(project.generated.size() == read.project->generated.size());
}

TEST_CASE("project generated: blanks, line ends and comments aren't a change; a track taken away or a file gone is") {
	auto scenery{::scenery()};
	auto const &project{scenery.project};
	// the line ends of windows, the blanks doubled, a comment inside a track
	std::string respaced;
	for (auto const c : scenery.text) {
		if (c == '\n') {
			respaced += "\r\n";
		}
		else if (c == ' ') {
			respaced += "  ";
		}
		else {
			respaced += c;
		}
	}
	auto const velocity{respaced.find("velocity")};
	REQUIRE(velocity != std::string::npos);
	respaced.insert(velocity, "// the speed of the line\r\n /* as the generator wrote it */ ");
	CHECK(changes_in(project, respaced).empty());
	CHECK(project::checksum("node -1 0 a track endtrack") == project::checksum("node  -1\t0 a // x\ntrack\r\nendtrack"));
	CHECK(project::checksum("node -1 0 a track endtrack") != project::checksum("node -1 0 b track endtrack"));

	// the last track of the first line taken away
	auto const &piece{scenery.lines.at(line_id{1}).pieces.back()};
	auto shortened{scenery.text};
	shortened.erase(shortened.find(piece.text), piece.text.size());
	auto const changes{changes_in(project, shortened)};
	REQUIRE(changes.size() == 1);
	CHECK(changes.front().line == line_id{1});
	CHECK(changes.front().missing);
	CHECK(changes.front().name == piece.name);
	// the file can't be read: all of them
	auto const gone{project::find_hand_changes(project, [](std::string const &) { return std::nullopt; })};
	CHECK(gone.size() == project.generated.size());
}

TEST_CASE("project generated: the switch track of a turnout kept by its main track, a change of it by hand marking that line") {
	layout::model model;
	layout::create_line straight;
	straight.start = {0.0, 100.0, 0.0};
	straight.end = {1000.0, 104.0, 0.0};
	straight.category = line_class();
	REQUIRE(model.execute(straight).done);
	auto const made{model.execute(layout::insert_turnout{model.state().lines().begin()->first, 300.0, "Rz 60E1-500-1:12", 1, 1})};
	REQUIRE_MESSAGE(made.done, reasons_of(made));
	project::document project;
	project.network = model.state();
	auto const &turnout{*project.network.find(made.turnouts.front())};
	auto const generated{layout::generate(project.network, turnout.id)};
	REQUIRE(generated.errors.empty());
	project::record_generated(project, generated, file_name);
	REQUIRE(project.generated.size() == 1);
	CHECK(project.generated.front().kind == "switch");
	CHECK(project.generated.front().name == generated.name);
	CHECK(project.generated.front().line == turnout.main);
	auto const text{"// the switch\n" + generated.text};
	CHECK(changes_in(project, text).empty());
	// kept by the project file
	auto const read{project::read(project::write(project))};
	REQUIRE(read.project.has_value());
	CHECK(read.project->generated == project.generated);
	// its speed written by hand
	auto edited{text};
	auto const velocity{edited.find("velocity ")};
	REQUIRE(velocity != std::string::npos);
	edited.insert(velocity + 9, "1");
	auto const changes{changes_in(project, edited)};
	REQUIRE(changes.size() == 1);
	CHECK(changes.front().line == turnout.main);
	project::mark_hand_changes(project, changes);
	CHECK_FALSE(project::may_overwrite(project, turnout.main));
	// made again by the editor
	project::record_generated(project, generated, file_name);
	CHECK(project::may_overwrite(project, turnout.main));
	CHECK(project.generated.size() == 1);
}

TEST_CASE("project generated: the node of the name found in the text of a scenery file, the words as the simulator reads them") {
	std::string const text{"// a track\nnode -1 0 tor_a track normal 10 /* the length */ 1.435\n 0.15 endtrack\nnode -1 0 tor_b track normal 20 endtrack\n"
	                       "node -1 0 komorka memcell 0 0 0 none 0 0 tor_a endmemcell\nevent tor_a multiple 0 none endevent\n"};
	CHECK(scenery_text::node_text(text, "tor_a") == std::optional<std::string>{"node -1 0 tor_a track normal 10 1.435 0.15 endtrack"});
	CHECK(scenery_text::node_text(text, "TOR_B") == std::optional<std::string>{"node -1 0 tor_b track normal 20 endtrack"});
	CHECK(scenery_text::node_text(text, "komorka").has_value());
	CHECK_FALSE(scenery_text::node_text(text, "tor_c").has_value());
	CHECK_FALSE(scenery_text::node_text(text, "endtrack").has_value());
}
