// the lines the track editor keeps in the scenery files after its //$ marks, read and written back by editor_records
// (editor/editorRecords.cpp), the one place the stores of the editor read and write them: the lines of data/records/ are written the
// way the editor writes them, so each kind read and written back comes out the same, which a second reading then reads the same as
// the first. The descriptions of the templates (//$e) are read and written by editor_includes, in the template files themselves

#include "stdafx.h"
#include "doctest.h"
#include "editor/editorIncludeInfo.hpp"
#include "editor/editorRecords.hpp"
#include "scratch.h"
#include "utilities/Globals.h"

#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string file_text(std::string const &Path) {
	std::ifstream file{Path, std::ios::binary};
	REQUIRE(file.is_open());
	return {std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
}

// the lines of the file of data/records/ with the mark, the mark taken off as scene::node_layers::scan() takes it: with the blanks
// after it, the line ends left out
std::vector<std::string> marked(std::string const &File, std::string const &Mark) {
	std::vector<std::string> result;
	std::istringstream lines{file_text(std::string{EU07_UNIT_TESTS_DATA} + "/records/" + File)};
	for (std::string line; std::getline(lines, line);) {
		while (false == line.empty() && (line.back() == '\r' || line.back() == '\n')) {
			line.pop_back();
		}
		if (line.starts_with(Mark + ' ')) {
			line.erase(0, Mark.size());
			line.erase(0, line.find_first_not_of(" \t"));
			result.push_back(line);
		}
	}
	REQUIRE_FALSE(result.empty());
	return result;
}

// the lines one under another, for a failed check to show them
std::string text(std::vector<std::string> const &Lines) {
	std::string result;
	for (auto const &line : Lines) {
		result += line + '\n';
	}
	return result;
}

// the lines without the comments, which the readers pass over
std::vector<std::string> uncommented(std::vector<std::string> Lines) {
	std::erase_if(Lines, [](std::string const &Line) { return Line.starts_with("#"); });
	return Lines;
}

// the placement of a file included at an origin: the lines say the same of other places in the world
scene::layer_context moved() {
	scene::layer_context context;
	context.offset = {1000.0, 5.0, -2000.0};
	return context;
}

scene::layer_handle constexpr layer{3};

} // namespace

TEST_CASE("editor records: the horizontal alignments (//$h) read and written back come out the same") {
	auto const lines{marked("alignment.scn", "//$h")};
	for (auto const &context : {scene::layer_context{}, moved()}) {
		auto records{editor_records::read_alignments(lines, context, layer)};
		REQUIRE(records.size() == 2);
		auto const &record{records.front()};
		CHECK(record.layer == layer);
		CHECK(record.design.start == context.to_world({0.0, 0.0, 0.0}));
		CHECK(record.design.end == context.to_world({1000.0, 0.0, 1000.0}));
		CHECK(record.source == "designed");
		CHECK(record.origin == 1200.0);
		CHECK(record.fit == 0.125);
		CHECK(record.design.shape == alignment::transition_shape::clothoid);
		CHECK(record.design.speed == 100.0);
		CHECK(record.design.norms.gauge == 1500.0);
		CHECK(record.design.transition_pieces == 3);
		REQUIRE(record.design.vertices.size() == 1);
		CHECK(record.design.vertices.front().radius == 400.0);
		CHECK(record.design.vertices.front().cant == 80.0);
		REQUIRE(record.clicks.size() == 1);
		CHECK(record.clicks.front() == context.to_world({0.0, 0.0, 50.0}));
		CHECK(record.elements.size() == 5);
		CHECK(record.whole());
		// a compound curve with the cants of its arcs and ramps of its own, from a curved track, kept for a part of its line
		auto const &compound{records.back()};
		CHECK(compound.source == "traced");
		CHECK(compound.design.shape == alignment::transition_shape::cubic_parabola);
		CHECK(compound.design.start_radius == 1200.0);
		CHECK(compound.design.start_turn == 1);
		CHECK(compound.design.start_curved);
		CHECK(compound.design.start_cant == 60.0);
		CHECK(compound.from == 10.0);
		CHECK(compound.to == 1500.0);
		REQUIRE(compound.design.vertices.size() == 1);
		auto const &vertex{compound.design.vertices.front()};
		CHECK(vertex.compound);
		REQUIRE(vertex.arcs.size() == 1);
		CHECK(vertex.arcs.front().radius == 900.0);
		CHECK(vertex.first_cant == 70.0);
		CHECK(vertex.arcs.front().cant == 50.0);
		CHECK(vertex.ramp_out_length == 60.0);
		// laid slower than the line, as a curve traced on the photo sharper than the norms allow at its speed
		CHECK(vertex.speed == 70.0);
		CHECK(record.design.vertices.front().speed == 0.0);
		// the lines of the elements are the ones the designs make, as alignment_store makes them before it writes the records
		for (auto &entry : records) {
			entry.result = alignment::compute(entry.design);
			REQUIRE(entry.result.valid);
		}
		CHECK(text(editor_records::alignment_lines(records, context)) == text(lines));
	}
}

TEST_CASE("editor records: the switches placed (//$r) read and written back come out the same") {
	auto const lines{marked("switch.scn", "//$r")};
	for (auto const &context : {scene::layer_context{}, moved()}) {
		auto const records{editor_records::read_switches(lines, context, layer)};
		REQUIRE(records.size() == 3);
		auto const &plain{records[0]};
		CHECK(plain.kind == "switch");
		CHECK(plain.name == "z1");
		CHECK(plain.type == "Rz");
		CHECK(plain.rail == "60E1");
		CHECK(plain.radius == 500.0);
		CHECK(plain.ratio == 12.0);
		CHECK(plain.hand == 'L');
		CHECK(plain.parts == std::vector<std::string>{"z1"});
		CHECK(plain.shaped);
		CHECK(plain.start == context.to_world({100.0, 0.0, 200.0}));
		REQUIRE(plain.km.has_value());
		CHECK(*plain.km == doctest::Approx(12345.0));
		auto const &connection{records[1]};
		CHECK(connection.main_radius == -1200.0);
		CHECK_FALSE(connection.km.has_value());
		CHECK(connection.source == "measured");
		CHECK(connection.lift == 0.06);
		CHECK(connection.lift_end == 0.04);
		CHECK(connection.lift_to == context.to_world({150.0, 0.0, 240.0}));
		CHECK(connection.ramp == 60.0);
		CHECK(records[2].parts.size() == 4);
		CHECK(records[2].middle == 300.0);
		CHECK(text(editor_records::switch_lines(records, context)) == text(lines));
	}
}

TEST_CASE("editor records: the kilometrage (//$k) read and written back comes out the same") {
	auto const lines{marked("kilometrage.scn", "//$k")};
	for (auto const &context : {scene::layer_context{}, moved()}) {
		std::vector<kilometrage::mark> marks;
		std::vector<kilometrage::via> vias;
		editor_records::read_kilometrage(lines, context, layer, marks, vias);
		REQUIRE(marks.size() == 2);
		REQUIRE(vias.size() == 1);
		CHECK(marks[0].line == "91");
		CHECK(marks[0].point == context.to_world({100.0, 0.0, 200.0}));
		CHECK(marks[0].km == 12.345);
		CHECK_FALSE(marks[0].beyond.has_value());
		CHECK(marks[1].beyond == 14.0);
		CHECK(vias[0].point == context.to_world({105.0, 0.0, 700.0}));
		CHECK(text(editor_records::kilometrage_lines(marks, vias, context)) == text(lines));
	}
}

TEST_CASE("editor records: the grade lines designed (//$p) read and written back come out the same") {
	auto const lines{marked("profile.scn", "//$p")};
	for (auto const &context : {scene::layer_context{}, moved()}) {
		auto const profiles{editor_records::read_profiles(lines, context, layer)};
		REQUIRE(profiles.size() == 1);
		auto const &profile{profiles.front()};
		CHECK(profile.length == 500.0);
		CHECK(profile.start == context.to_world({0.0, 100.0, 0.0}));
		CHECK(profile.line.speed == 120.0);
		CHECK(profile.origin == 1000.0);
		CHECK(profile.line.join_start);
		CHECK_FALSE(profile.line.join_end);
		CHECK(profile.line.norms.radius_min == 2000.0);
		CHECK(profile.departure == 0.015);
		REQUIRE(profile.line.points.size() == 3);
		CHECK(profile.line.points[1].elevation == context.to_world({0.0, 101.0, 0.0}).y);
		CHECK(profile.line.points[1].radius == 10000.0);
		CHECK(profile.line.points[1].automatic);
		CHECK(text(editor_records::profile_lines(profiles, context)) == text(lines));
	}
}

TEST_CASE("editor records: the tracks fitted (//$t) and sharing a grade line (//$w) read and written back come out the same") {
	auto const fitted{marked("fitted.scn", "//$t")};
	auto const tracks{editor_records::read_fitted(fitted)};
	REQUIRE(tracks.size() == 2);
	CHECK(tracks[0].name == "tor_1");
	CHECK(tracks[0].shape == 0x00ff00ff12345678ull);
	CHECK_FALSE(tracks[0].hand);
	CHECK(tracks[1].hand);
	CHECK(text(editor_records::fitted_lines(tracks)) == text(fitted));

	auto const shared{marked("gradeline.scn", "//$w")};
	auto const lines{editor_records::read_grade_lines(shared)};
	REQUIRE(lines.size() == 2);
	CHECK(lines[0].key == "tor_1@100,200");
	CHECK(lines[0].number == 3);
	CHECK_FALSE(lines[0].hand);
	CHECK(lines[1].hand);
	CHECK(text(editor_records::grade_line_lines(lines)) == text(shared));
}

TEST_CASE("editor records: the objects bound to the paths (//$b) read and written back come out the same") {
	auto const lines{marked("binding.scn", "//$b")};
	for (auto const &context : {scene::layer_context{}, moved()}) {
		std::vector<std::string> written{editor_records::binding_heading()};
		std::vector<editor_records::binding> bindings;
		for (auto const &line : uncommented(lines)) {
			auto const binding{editor_records::read_binding(line, context)};
			REQUIRE(binding.has_value());
			bindings.push_back(*binding);
			written.push_back(editor_records::binding_line(*binding, context));
		}
		REQUIRE(bindings.size() == 2);
		CHECK(bindings[0].kind == "model");
		CHECK(bindings[0].category == 2);
		CHECK(bindings[0].location == context.to_world({10.0, 0.5, 20.0}));
		CHECK(bindings[0].key == "models/slupy/slup.t3d");
		CHECK(bindings[0].turns);
		REQUIRE(bindings[1].anchors.size() == 2);
		CHECK(bindings[1].anchors[1].path == 1);
		CHECK(bindings[1].anchors[1].end == context.to_world({5.0, 0.0, 100.0}));
		CHECK(text(written) == text(lines));
	}
}

TEST_CASE("editor records: the stations described (//$s) read and written back come out the same, their names in windows-1250") {
	auto const lines{marked("station.scn", "//$s")};
	std::vector<editor_records::station> stations;
	for (auto const &line : uncommented(lines)) {
		auto const station{editor_records::read_station(line)};
		REQUIRE(station.has_value());
		stations.push_back(*station);
	}
	REQUIRE(stations.size() == 2);
	auto const &first{stations[0]};
	CHECK(first.code == "MMz");
	CHECK(first.line == "91");
	CHECK(first.km == "12.345");
	CHECK_FALSE(first.reversed);
	CHECK(first.anchor == "z1");
	CHECK(first.braking == 700);
	CHECK(first.blocks.at("bpd_a") == editor_records::block_kind::four);
	CHECK(first.blocks.at("bpd_b") == editor_records::block_kind::semi);
	CHECK(first.ways.at("bpd_a"));
	REQUIRE(first.joints_added.size() == 1);
	REQUIRE(first.joints_removed.size() == 1);
	CHECK(first.joints_removed.front() == glm::dvec3{110.5, 0.0, 205.25});
	CHECK(first.name == "\xC5\x81\xC3\xB3\x64\xC5\xBA Kaliska"); // Łódź Kaliska, in utf-8
	// the words left empty are read as none
	CHECK(stations[1].code.empty());
	CHECK(stations[1].km.empty());
	CHECK(stations[1].reversed);
	CHECK(stations[1].blocks.at("x") == editor_records::block_kind::three);
	CHECK(text(editor_records::station_lines(stations)) == text(lines));
}

TEST_CASE("editor records: the camera (//$c), the orthophoto (//$o) and its area (//$z) read and written back come out the same") {
	auto const camera{marked("camera.scn", "//$c")};
	auto const place{editor_records::read_camera(camera)};
	REQUIRE(place.has_value());
	CHECK(place->position == glm::dvec3{1234.5, 56.25, -789.125});
	CHECK(place->angle == glm::vec3{0.125f, -1.5f, 0.f});
	CHECK(editor_records::camera_line(*place) == camera.front());

	for (auto const &line : marked("orthophoto.scn", "//$o")) {
		CAPTURE(line);
		bool enabled{false};
		double north{0.0};
		double east{0.0};
		REQUIRE(editor_records::read_orthophoto(line, enabled, north, east));
		// the editor writes where the scenery lies when it lies somewhere
		CHECK(editor_records::orthophoto_line(enabled, north != 0.0 || east != 0.0, north, east) == line);
	}

	auto const area{marked("area.scn", "//$z")};
	auto const polygon{editor_records::read_area(area)};
	REQUIRE(polygon.size() == 3);
	CHECK(polygon[2] == glm::dvec2{646000.5, 513000.25});
	CHECK(text(editor_records::area_lines(polygon)) == text(area));
}

TEST_CASE("editor records: the description of a template (//$e) read and written back comes out the same, and the rest of the file with it") {
	test::scratch_folder const folder{"editor_records_templates"};
	auto const original{file_text(std::string{EU07_UNIT_TESTS_DATA} + "/records/semaphore.inc")};
	folder.write("semaphore.inc", original);
	// the templates are read from the folder of the scenery
	struct scenery_folder {
		explicit scenery_folder(std::string const &Path) : before{Global.asCurrentSceneryPath} { Global.asCurrentSceneryPath = Path; }
		~scenery_folder() { Global.asCurrentSceneryPath = before; }
		scenery_folder(scenery_folder const &) = delete;
		scenery_folder &operator=(scenery_folder const &) = delete;
		std::string before;
	} const scenery{folder.path()};

	include_info first;
	std::string error;
	REQUIRE(editor_includes::load("semaphore.inc", first, error));
	CHECK(first.name == "Semaphore, 2 arms");
	CHECK(first.category == "signals");
	CHECK(first.signal_kind == "semaphore");
	CHECK(first.signal_arms == 2);
	CHECK(first.signal_height == 5.5);
	REQUIRE(first.parameters.size() == 3);
	CHECK(first.parameters[0].role == "name");
	CHECK(first.parameters[2].value == "tor1");
	REQUIRE(editor_includes::save("semaphore.inc", first, error));
	CHECK(file_text(folder.path("semaphore.inc")) == original);
	include_info second;
	REQUIRE(editor_includes::load("semaphore.inc", second, error));
	CHECK(second.name == first.name);
	CHECK(second.description == first.description);
	CHECK(second.signal_aspects == first.signal_aspects);
	REQUIRE(second.parameters.size() == first.parameters.size());
	for (std::size_t index = 0; index < first.parameters.size(); ++index) {
		CHECK(second.parameters[index].id == first.parameters[index].id);
		CHECK(second.parameters[index].role == first.parameters[index].role);
		CHECK(second.parameters[index].label == first.parameters[index].label);
		CHECK(second.parameters[index].value == first.parameters[index].value);
	}
}
