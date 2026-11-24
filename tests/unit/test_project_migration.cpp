// the //$ lines of an old scenery taken into the project file (editor/project/projectMigration.cpp, editor-tracks-domain.md §9, §10): the
// sceneries of data/records/ and of data/project/migration/ made into projects, written and read back to the same; the alignments the
// domain lays as the editor laid them made lines of it, within a millimetre of the old design; the grade line along one made its breaks;
// the rest kept as it was written, each kind read back by editor_records as from the scenery; the scenery files left as they were

#include "stdafx.h"
#include "doctest.h"
#include "scratch.h"
#include "editor/editorAlignment.hpp"
#include "editor/editorRecords.hpp"
#include "editor/project/projectMigration.hpp"

#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>

namespace {

using layout::line_id;
using layout::node_id;

std::string file_text(std::filesystem::path const &Path) {
	std::ifstream file{Path, std::ios::binary};
	REQUIRE(file.is_open());
	return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

std::string notes_of(std::vector<std::string> const &Notes) {
	std::string result;
	for (auto const &note : Notes) {
		result += note + "\n";
	}
	return result;
}

project::marked_file records_file(std::string const &Name) {
	auto const text{file_text(std::string{EU07_UNIT_TESTS_DATA} + "/records/" + Name)};
	return {"records/" + Name, {}, project::marked_lines(text)};
}

project::legacy_record const *legacy_of(project::document const &Project, char const Mark, std::size_t const Nth = 0) {
	std::size_t seen{0};
	for (auto const &record : Project.legacy) {
		if (record.mark == Mark && seen++ == Nth) {
			return &record;
		}
	}
	return nullptr;
}

// written and read back: the same project
project::document round_trip(project::document const &Project) {
	auto const text{project::write(Project)};
	auto const read{project::read(text)};
	REQUIRE_MESSAGE(read.project.has_value(), read.error);
	CHECK(project::same(*read.project, Project));
	CHECK(project::write(*read.project) == text);
	return *read.project;
}

// the line of the domain against the design it was made of: the points along it within a millimetre, the heights too
void check_follows(layout::line const &Line, alignment::result const &Design) {
	REQUIRE(Line.solved.valid());
	CHECK(Line.solved.length == doctest::Approx(Design.length).epsilon(1e-9));
	for (int i = 0; i <= 50; ++i) {
		auto const s{Design.length * i / 50.0};
		auto const old{alignment::evaluate(Design, s)};
		auto const now{Line.solved.point(std::min(s, Line.solved.length))};
		CAPTURE(s);
		CHECK(glm::distance(glm::dvec2{old.position.x, old.position.z}, glm::dvec2{now.x, now.z}) < 0.001);
	}
}

} // namespace

TEST_CASE("project migration: the //$ lines of a scenery file by their marks, the old letters read under the new ones, not those of other tools") {
	auto const lines{project::marked_lines("// a comment\n  //$h alignment 1 2 3\r\n//$h\tend\n//$hx not a mark\n//$f fitted tor_1 00ff\n//$f something else\n//$n gradeline a 1\n//$a point 1 2\n//$k\n"
	                                       "//$r include linia/sygnalizacja.scm end\n//$r # switches\n//$b anything\n")};
	CHECK(lines.at('h') == std::vector<std::string>{"alignment 1 2 3", "end"});
	CHECK(lines.at('t') == std::vector<std::string>{"fitted tor_1 00ff"});
	CHECK(lines.at('w') == std::vector<std::string>{"gradeline a 1"});
	CHECK(lines.at('z') == std::vector<std::string>{"point 1 2"});
	CHECK(lines.at('r') == std::vector<std::string>{"# switches"});
	CHECK(lines.at('b') == std::vector<std::string>{"anything"});
	CHECK_FALSE(lines.contains('k'));
	CHECK_FALSE(lines.contains('f'));
	CHECK_FALSE(lines.contains('x'));
}

TEST_CASE("project migration: the golden //$ lines of data/records made into a project and back, each kind as it was") {
	std::vector<project::marked_file> files;
	for (auto const *name : {"alignment.scn", "profile.scn", "switch.scn", "kilometrage.scn", "fitted.scn", "gradeline.scn", "binding.scn", "station.scn", "camera.scn", "orthophoto.scn", "area.scn"}) {
		files.push_back(records_file(name));
	}
	auto const migrated{project::migrate(files)};
	auto const &project{migrated.project};
	CAPTURE(notes_of(migrated.notes));

	// the first alignment, of a curve with clothoids, is a line; the compound curve kept for a part of its line isn't
	REQUIRE(project.network.lines().size() == 1);
	auto const &line{project.network.lines().begin()->second};
	CHECK(line.id == line_id{1});
	REQUIRE(line.shape.vertices.size() == 1);
	CHECK(line.shape.vertices.front().radius == 400.0);
	CHECK(line.shape.vertices.front().transition_in == 60.0);
	CHECK(line.shape.vertices.front().transition_out == 60.0);
	CHECK(line.shape.vertices.front().cant == 80.0);
	CHECK(line.shape.transition == layout::transition_shape::clothoid);
	CHECK(line.chainage_start == 1200.0);
	CHECK(line.category.vmax == 100.0);
	CHECK(project.network.nodes().size() == 2);
	CHECK(project.network.find(line.start)->position == glm::dvec3{0.0, 0.0, 0.0});
	CHECK(project.network.find(line.end)->position == glm::dvec3{1000.0, 0.0, 1000.0});
	auto const records{editor_records::read_alignments(files.front().lines.at('h'), {}, 0)};
	REQUIRE(records.size() == 2);
	check_follows(line, alignment::compute(records.front().design));

	// the records kept as they were: the alignments with the line each one became, or why it didn't
	auto const *first{legacy_of(project, 'h', 0)};
	auto const *second{legacy_of(project, 'h', 1)};
	REQUIRE(first != nullptr);
	REQUIRE(second != nullptr);
	CHECK(first->line == line_id{1});
	CHECK(first->why.empty());
	CHECK(first->file == "records/alignment.scn");
	CHECK(second->line == line_id::none);
	CHECK(second->why == "a design kept for a part of its line");
	auto const *profile{legacy_of(project, 'p')};
	REQUIRE(profile != nullptr);
	CHECK(profile->line == line_id::none);
	CHECK(profile->why == "along no line of an alignment taken into the project");
	// each kind of the rest kept whole, as the file has it
	for (auto const &[mark, name] : std::vector<std::pair<char, std::string>>{{'r', "switch.scn"}, {'k', "kilometrage.scn"}, {'t', "fitted.scn"}, {'w', "gradeline.scn"}, {'b', "binding.scn"}, {'s', "station.scn"}}) {
		CAPTURE(mark);
		auto const *kept{legacy_of(project, mark)};
		REQUIRE(kept != nullptr);
		CHECK(kept->file == "records/" + name);
		CHECK(kept->lines == records_file(name).lines.at(mark));
		CHECK(kept->line == line_id::none);
	}
	// the camera, the orthophoto and the area aren't the tracks: left in the files, told
	CHECK(legacy_of(project, 'c') == nullptr);
	CHECK(legacy_of(project, 'o') == nullptr);
	CHECK(legacy_of(project, 'z') == nullptr);
	CHECK(notes_of(migrated.notes).find("records/camera.scn: the //$c lines are the editor's own") != std::string::npos);

	// written and read back, the same data: the lines kept read by editor_records as from the scenery files
	auto const read{round_trip(project)};
	CHECK(read.network.find(line_id{1})->solved == line.solved);
	auto const scenery_records{editor_records::read_alignments(files.front().lines.at('h'), {}, 0)};
	std::vector<std::string> kept_lines{legacy_of(read, 'h', 0)->lines};
	kept_lines.insert(kept_lines.end(), legacy_of(read, 'h', 1)->lines.begin(), legacy_of(read, 'h', 1)->lines.end());
	auto const kept_records{editor_records::read_alignments(kept_lines, {}, 0)};
	REQUIRE(kept_records.size() == scenery_records.size());
	for (std::size_t i = 0; i < kept_records.size(); ++i) {
		CHECK(kept_records[i].design.vertices.size() == scenery_records[i].design.vertices.size());
		CHECK(kept_records[i].clicks == scenery_records[i].clicks);
		CHECK(kept_records[i].origin == scenery_records[i].origin);
	}
	CHECK(editor_records::read_switches(legacy_of(read, 'r')->lines, {}, 0).size() == 3);
	CHECK(editor_records::read_fitted(legacy_of(read, 't')->lines).size() == 2);
	auto const station{editor_records::read_station(legacy_of(read, 's')->lines[1])};
	REQUIRE(station.has_value());
	CHECK(station->code == "MMz");
	CHECK(station->name == editor_records::read_station(files[7].lines.at('s')[1])->name);
}

TEST_CASE("project migration: a map with no //$ lines gets an empty project and no file next to it") {
	test::scratch_folder folder{"project_migration_none"};
	std::ofstream{folder.root() / "pusta.scm", std::ios::binary} << "node -1 0 t1 track normal 100 1.435 0.2 25 20 0 flat vis 0 0 0 0 0 0 0 0 0 0 100 0 endtrack\n";
	auto const opened{project::open_project(folder.root(), "pusta.scm")};
	REQUIRE_MESSAGE(opened.project.project.has_value(), opened.project.error);
	CHECK(opened.project.project->network.lines().empty());
	CHECK_FALSE(opened.migrated);
	CHECK_FALSE(std::filesystem::exists(folder.root() / "pusta.eu07proj"));
}

TEST_CASE("project migration: a map opened for the first time gets its project of the //$ lines of its files; the files stay as they were") {
	test::scratch_folder folder{"project_migration"};
	auto const source{std::filesystem::path{EU07_UNIT_TESTS_DATA} / "project" / "migration"};
	std::filesystem::copy(source, folder.root(), std::filesystem::copy_options::recursive);
	auto const map_before{file_text(folder.root() / "trasa.scm")};
	auto const tracks_before{file_text(folder.root() / "trasa" / "tory.scm")};

	// the files of the map with the placement of their includes
	auto const files{project::marked_files(folder.root(), "trasa.scm")};
	REQUIRE(files.size() == 2);
	CHECK(files[0].file == "trasa.scm");
	CHECK(files[1].file == "trasa/tory.scm");
	CHECK(files[1].context.offset == glm::dvec3{1000.0, 0.0, 2000.0});

	auto const opened{project::open_project(folder.root(), "trasa.scm")};
	CAPTURE(notes_of(opened.notes));
	REQUIRE_MESSAGE(opened.project.project.has_value(), opened.project.error);
	CHECK(opened.migrated);
	CHECK(std::filesystem::exists(folder.root() / "trasa.eu07proj"));
	CHECK(file_text(folder.root() / "trasa.scm") == map_before);
	CHECK(file_text(folder.root() / "trasa" / "tory.scm") == tracks_before);
	auto const &project{*opened.project.project};
	auto const &network{project.network};

	// two alignments, a curve and a straight going on from it, joined at a node; the compound curve kept as it was
	REQUIRE(network.lines().size() == 2);
	CHECK(network.nodes().size() == 3);
	auto const *curve{network.find(line_id{1})};
	auto const *straight{network.find(line_id{2})};
	REQUIRE(curve != nullptr);
	REQUIRE(straight != nullptr);
	CHECK(curve->end == straight->start);
	CHECK(network.find(curve->end)->ends.size() == 2);
	CHECK(network.find(curve->start)->position == glm::dvec3{1000.0, 10.0, 2000.0});
	CHECK(network.find(curve->end)->position == glm::dvec3{1600.0, 12.0, 2600.0});
	CHECK(curve->chainage_start == 1200.0);
	CHECK(straight->chainage_start == 2500.0);
	CHECK(straight->category.vmax == 120.0);
	CHECK(straight->shape.transition == layout::transition_shape::cubic_parabola);
	REQUIRE(curve->shape.vertices.size() == 1);
	CHECK(curve->shape.vertices.front().point == glm::dvec2{1000.0, 2600.0});
	CHECK(curve->shape.vertices.front().radius == 500.0);
	// the curve laid as the old design, in the world
	auto const tracks{project::marked_lines(tracks_before)};
	scene::layer_context origin;
	origin.offset = {1000.0, 0.0, 2000.0};
	auto const designs{editor_records::read_alignments(tracks.at('h'), origin, 0)};
	REQUIRE(designs.size() == 3);
	check_follows(*curve, alignment::compute(designs.front().design));

	// the grade line along the straight: its break with its vertical curve, the heights of its ends those of the nodes
	REQUIRE(straight->shape.grades.size() == 1);
	CHECK(straight->shape.grades.front().chainage == 400.0);
	CHECK(straight->shape.grades.front().elevation == 14.0);
	CHECK(straight->shape.grades.front().vertical_radius == 10000.0);
	CHECK(network.find(straight->end)->position.y == 13.0);
	CHECK(straight->solved.elevation(400.0) == doctest::Approx(14.0 - 37.5 * 37.5 / (2.0 * 10000.0)));
	// the one along the curve has a break with no vertical curve: kept as it was, the curve keeps the grade between its nodes
	CHECK(curve->shape.grades.empty());
	auto const *refused{legacy_of(project, 'p', 0)};
	REQUIRE(refused != nullptr);
	CHECK(refused->line == line_id::none);
	CHECK(refused->why == "a break of the grade with no vertical curve at 500.0 m");
	CHECK(legacy_of(project, 'p', 1)->line == line_id{2});
	CHECK(legacy_of(project, 'h', 2)->why == "a design kept for a part of its line");
	// the other kinds kept whole, the old mark of the fitted tracks under the new one; the camera and the orthophoto left
	CHECK(legacy_of(project, 't')->lines.back() == "fitted tor_9 0000000000000001");
	CHECK(legacy_of(project, 'c') == nullptr);
	CHECK(legacy_of(project, 'r')->file == "trasa/tory.scm");
	CHECK(legacy_of(project, 'k')->lines.size() == 4);
	CHECK(legacy_of(project, 'b')->lines.size() == 3);

	// opened again: the project file read, not made again
	auto const again{project::open_project(folder.root(), "trasa.scm")};
	REQUIRE(again.project.project.has_value());
	CHECK_FALSE(again.migrated);
	CHECK(project::same(*again.project.project, project));
	round_trip(project);
}
