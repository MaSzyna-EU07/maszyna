// the project of the map open in the editor (editor/project/projectSession.cpp, editor-tracks-domain.md §4 "Edycja = polecenie", §9; E4):
// a map opened gets its project file; a command tried changes nothing, carried out tells the lines whose tracks go or are made again;
// each command kept a step the editor's history takes back and brings again by its token, the marks of the lines changed by hand with it;
// the tracks written with the scenery recorded, a track changed in the file afterwards marking its line, the line written over only when
// the author decides

#include "stdafx.h"
#include "doctest.h"
#include "layout_fixture.h"
#include "scratch.h"
#include "editor/project/projectGenerated.hpp"
#include "editor/project/projectSession.hpp"

#include <fstream>
#include <iterator>
#include <string>

namespace {

using namespace test::layout_fixture;
using layout::line_id;

std::string file_text(std::filesystem::path const &Path) {
	std::ifstream file{Path, std::ios::binary};
	REQUIRE(file.is_open());
	return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

// a straight of 400 m along z
layout::create_line straight(double const X) {
	layout::create_line result;
	result.start = {X, 10.0, 0.0};
	result.end = {X, 10.0, 400.0};
	result.category = line_class();
	return result;
}

// the tracks of the line written into the map as the editor writes them: the generator's text
void write_tracks(project::session &Session, test::scratch_folder const &Folder, std::string const &Map, std::vector<line_id> const &Lines) {
	std::string text{"// a map of the tests of the project of the editor\n"};
	for (auto const line : Lines) {
		auto const made{Session.generate(line)};
		REQUIRE(made.errors.empty());
		std::vector<project::session::written_piece> written;
		for (auto const &piece : made.pieces) {
			text += piece.text;
			written.push_back({piece.name, piece.text});
		}
		Session.record_written(line, written, Map);
	}
	Folder.write(Map, text);
}

} // namespace

TEST_CASE("project session: the map opened gets its project, a command tried changes nothing, carried out tells its lines") {
	test::scratch_folder folder{"project_session_commands"};
	folder.write("mapa.scn", "// a new scenery\n");
	project::session session;
	auto const opened{session.open(folder.root(), "mapa.scn")};
	REQUIRE(opened.done);
	CHECK(opened.migrated);
	CHECK(std::filesystem::exists(folder.root() / "mapa.eu07proj"));
	CHECK(session.file() == folder.root() / "mapa.eu07proj");
	CHECK(session.network().lines().empty());

	// a curve sharper than the norms let a line of 80 km/h have: refused with its reasons, tried or carried out, nothing changed
	auto sharp{three_curves()};
	sharp.vertices[1].radius = 120.0;
	auto const tried{session.trial(sharp)};
	CHECK_FALSE(tried.outcome.done);
	CHECK_FALSE(tried.outcome.reasons.empty());
	auto const refused{session.execute(sharp)};
	CHECK_FALSE(refused.outcome.done);
	CHECK(refused.token == 0);
	CHECK(session.network().lines().empty());
	CHECK(session.undoable() == 0);

	// tried: what it'd make, the network as it was (the identifiers a refused command took aren't given again)
	auto const trial{session.trial(three_curves())};
	REQUIRE(trial.outcome.done);
	REQUIRE(trial.outcome.lines.size() == 1);
	auto const id{trial.outcome.lines.front()};
	CHECK(trial.changed == std::set<line_id>{id});
	CHECK(session.network().lines().empty());
	auto const laid{session.execute(three_curves())};
	REQUIRE(laid.outcome.done);
	CHECK(laid.token != 0);
	CHECK(laid.changed == std::set<line_id>{id});
	CHECK(laid.removed.empty());

	// split: the line laid again as it was is the same, the part after the node is new
	auto const *line{session.network().find(id)};
	REQUIRE(line != nullptr);
	auto const split{session.execute(layout::split_line{id, 0.5 * line->solved.length})};
	REQUIRE(split.outcome.done);
	CHECK(split.changed.contains(id));
	CHECK(split.changed.size() == 2);
	auto const second{*std::prev(split.changed.end())};
	// a curve of the second part made wider: that part alone
	auto const *part{session.network().find(second)};
	REQUIRE(part != nullptr);
	REQUIRE_FALSE(part->shape.vertices.empty());
	auto const vertex{part->shape.vertices.back()};
	auto const moved{session.execute(layout::set_vertex{second, vertex.id, vertex.radius + 100.0})};
	REQUIRE(moved.outcome.done);
	CHECK(moved.changed == std::set<line_id>{second});
	// deleted: gone, nothing laid anew
	auto const deleted{session.execute(layout::delete_line{second})};
	REQUIRE(deleted.outcome.done);
	CHECK(deleted.changed.empty());
	CHECK(deleted.removed == std::set<line_id>{second});
	CHECK(session.undoable() == 4);
}

TEST_CASE("project session: each command a step the editor takes back and brings again by its token, in the order of its history") {
	test::scratch_folder folder{"project_session_undo"};
	folder.write("mapa.scn", "// a new scenery\n");
	project::session session;
	REQUIRE(session.open(folder.root(), "mapa.scn").done);
	auto const empty{session.network()};
	auto const first{session.execute(straight(0.0))};
	REQUIRE(first.outcome.done);
	auto const one{session.network()};
	auto const second{session.execute(straight(50.0))};
	REQUIRE(second.outcome.done);
	auto const two{session.network()};

	// only the latest step is taken back, the one before it next; a token of another step changes nothing
	CHECK_FALSE(session.undo(first.token));
	CHECK(session.network().same(two));
	project::session::effect effect;
	REQUIRE(session.undo(second.token, &effect));
	CHECK(session.network().same(one));
	CHECK(effect.removed == std::set<line_id>{line_id{2}});
	CHECK_FALSE(session.redo(first.token));
	REQUIRE(session.undo(first.token, &effect));
	CHECK(session.network().same(empty));
	CHECK(effect.removed == std::set<line_id>{line_id{1}});
	// brought again in the order taken back, the last one taken back first
	CHECK_FALSE(session.redo(second.token));
	REQUIRE(session.redo(first.token, &effect));
	CHECK(effect.changed == std::set<line_id>{line_id{1}});
	CHECK(session.network().same(one));
	REQUIRE(session.redo(second.token));
	CHECK(session.network().same(two));

	// a command kept after a step taken back: that step is gone, as the editor's redo of it is
	REQUIRE(session.undo(second.token));
	auto const third{session.execute(straight(100.0))};
	REQUIRE(third.outcome.done);
	CHECK_FALSE(session.redo(second.token));
	CHECK(session.redoable() == 0);
	// identifiers are never handed out again: the new line isn't the one taken back
	CHECK(third.changed == std::set<line_id>{line_id{3}});
	// a refused command isn't a step
	auto sharp{three_curves()};
	sharp.vertices[1].radius = 120.0;
	CHECK(session.execute(sharp).token == 0);
	CHECK(session.undoable() == 2);
}

TEST_CASE("project session: the tracks written recorded, one changed in the file marks its line, written over only when the author decides") {
	test::scratch_folder folder{"project_session_hand"};
	folder.write("mapa.scn", "// a new scenery\n");
	{
		project::session session;
		REQUIRE(session.open(folder.root(), "mapa.scn").done);
		REQUIRE(session.execute(three_curves()).outcome.done);
		REQUIRE(session.execute(straight(-100.0)).outcome.done);
		write_tracks(session, folder, "mapa.scn", {line_id{1}, line_id{2}});
		std::string error;
		REQUIRE(session.save(error));
		CHECK(session.project().generated.size() > 2);
	}
	auto const saved{file_text(folder.root() / "mapa.eu07proj")};
	project::session session;
	{
		// opened again: nothing changed in the file, the network the same to the bit
		auto const opened{session.open(folder.root(), "mapa.scn")};
		REQUIRE(opened.done);
		CHECK_FALSE(opened.migrated);
		CHECK(opened.hand_changes.empty());
		CHECK(session.project().hand_changed.empty());
		layout::model laid{norms::area_map{}};
		REQUIRE(laid.execute(three_curves()).done);
		CHECK(*session.network().find(line_id{1}) == *laid.state().find(line_id{1}));
		// a vertex moved changes line 1 alone, which nobody changed by hand
		auto const *line{session.network().find(line_id{1})};
		auto const &vertex{line->shape.vertices.front()};
		auto const effect{session.trial(layout::set_vertex{line_id{1}, vertex.id, 850.0})};
		REQUIRE(effect.outcome.done);
		CHECK(session.hand_changed(effect).empty());
		// one the editor saw changed by other tools is
		CHECK(session.hand_changed(effect, {line_id{1}}) == std::set<line_id>{line_id{1}});
	}

	// the speed of a track of line 1 edited outside the editor
	auto text{file_text(folder.root() / "mapa.scn")};
	auto const name{session.generate(line_id{1}).pieces.front().name};
	auto const at{text.find("velocity 80", text.find(name))};
	REQUIRE(at != std::string::npos);
	text.replace(at, 11, "velocity 60");
	folder.write("mapa.scn", text);
	auto const opened{session.open(folder.root(), "mapa.scn")};
	REQUIRE(opened.done);
	REQUIRE(opened.hand_changes.size() == 1);
	CHECK(opened.hand_changes.front().name == name);
	CHECK(session.project().hand_changed == std::set<line_id>{line_id{1}});
	auto const *line{session.network().find(line_id{1})};
	auto const vertex{line->shape.vertices.front()};
	layout::set_vertex const wider{line_id{1}, vertex.id, 850.0};
	auto const effect{session.trial(wider)};
	REQUIRE(effect.outcome.done);
	CHECK(session.hand_changed(effect) == std::set<line_id>{line_id{1}});
	// deleting it is writing over it too
	CHECK(session.hand_changed(session.trial(layout::delete_line{line_id{1}})) == std::set<line_id>{line_id{1}});
	// line 2 isn't
	CHECK(session.hand_changed(session.trial(layout::delete_line{line_id{2}})).empty());

	// written over: the mark goes with the step, and comes back with its undo
	auto const done{session.execute(wider, {line_id{1}})};
	REQUIRE(done.outcome.done);
	CHECK(session.project().hand_changed.empty());
	REQUIRE(session.undo(done.token));
	CHECK(session.project().hand_changed == std::set<line_id>{line_id{1}});
	REQUIRE(session.redo(done.token));
	CHECK(session.project().hand_changed.empty());
	// written with the scenery: recorded again, the file the same as written, nothing changed by hand
	write_tracks(session, folder, "mapa.scn", {line_id{1}, line_id{2}});
	std::string error;
	REQUIRE(session.save(error));
	CHECK(file_text(folder.root() / "mapa.eu07proj") != saved);
	REQUIRE(session.open(folder.root(), "mapa.scn").hand_changes.empty());
	CHECK(session.network().find(line_id{1})->shape.vertices.front().radius == 850.0);

	// a line deleted and saved: its tracks' records go with it, a mark of it too
	session.mark_hand_changed(line_id{2});
	REQUIRE(session.execute(layout::delete_line{line_id{2}}, {line_id{2}}).outcome.done);
	REQUIRE(session.save(error));
	for (auto const &node : session.project().generated) {
		CHECK(node.line == line_id{1});
	}
	CHECK(session.project().hand_changed.empty());
}
