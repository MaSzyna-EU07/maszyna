#include "stdafx.h"
#include "doctest.h"
#include "vehicle.h"
#include "golden.h"
#include "real_vehicles.h"
#include "fiz.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "utilities/parser.h"

using test::definition;
using test::vehicle;

// the real vehicle definitions in data/fiz load, as the game loads them, and give the parameters they gave
// when the traces got recorded: the loader (LoadFIZ_Parse and the parsing of every section) stays as it was

TEST_CASE("fiz: every real vehicle loads and keeps its parameters") {
	for (auto const &path : test::real_vehicles()) {
		CAPTURE(path);
		test::reseed_random();
		vehicle real{definition::fiz(path)};
		test::trace trace{"fiz/" + path, test::channels::parameters(), 1};
		trace.sample(0.0, real.mover());
		trace.check();
	}
}

TEST_CASE("fiz: changes made by a test reach the vehicle") {
	auto const eu07{definition::fiz("4e_v1/4e")};
	CHECK(vehicle{eu07}->Mass == doctest::Approx(80000.0));
	CHECK(vehicle{definition{eu07}.set("Param", "M", 90000)}->Mass == doctest::Approx(90000.0));
	// a key the file doesn't have is added to its section
	CHECK(vehicle{definition{eu07}.set("Brake", "MaxBP", 3.5)}->MaxBrakePress[3] == doctest::Approx(3.5));
	// removing a key brings its default back
	CHECK(vehicle{definition{eu07}.remove("Param", "Vmax")}->Vmax != doctest::Approx(125.0));
}

namespace {

// a section of the game's definition, as data/fiz keeps it
struct converted_section {
	std::string name;
	nlohmann::ordered_json keys = nlohmann::ordered_json::object();
	nlohmann::ordered_json rows = nlohmann::ordered_json::array();
};

// the sections of the lines of the game's definition: the values of each section line, and the rows of a
// list until the line closing it. lines the loader skips, and the lines closing the lists, are left out
std::vector<converted_section> convert(std::vector<std::string> const &Lines) {
	std::vector<converted_section> result;
	auto reading_rows{false};
	for (auto const &line : Lines) {
		if (auto const section{test::fiz::opened_by(line)}; section != nullptr) {
			converted_section converted{std::string{section->name}};
			for (auto const &key : test::fiz::keys(line)) {
				if (auto const value{test::fiz::value(key, line)}; false == value.empty()) {
					converted.keys[key] = value;
				}
			}
			result.push_back(std::move(converted));
			reading_rows = section->list != 0;
			continue;
		}
		auto const list{result.empty() ? nullptr : test::fiz::named(result.back().name)};
		if (list == nullptr || list->end == nullptr) {
			continue;
		}
		// an empty line closes the brake table of the Cntrl section, the other lists have lines of their own
		if (std::string_view const end{list->end}; line.empty() ? end.empty() : (false == end.empty() && line.starts_with(end))) {
			reading_rows = false;
			continue;
		}
		if (reading_rows && false == line.empty()) {
			auto row = nlohmann::ordered_json::array();
			for (auto const &word : test::fiz::words(line)) {
				row.push_back(word);
			}
			result.back().rows.push_back(std::move(row));
		}
	}
	return result;
}

// the JSON of data/fiz: a section a line, a row of a list a line
std::string json_text(std::string const &Source, std::vector<converted_section> const &Sections) {
	std::string text{"{\n  \"_source\": " + nlohmann::ordered_json(Source).dump() + ",\n"};
	text += "  \"_comment\": \"converted by test_fiz.cpp from the game's definition with the files it includes, keeping what the game's loader reads\"";
	for (auto const &section : Sections) {
		text += ",\n  " + nlohmann::ordered_json(section.name).dump() + ": {";
		auto first{true};
		for (auto const &[key, value] : section.keys.items()) {
			text += (first ? "" : ", ") + nlohmann::ordered_json(key).dump() + ": " + value.dump();
			first = false;
		}
		if (test::fiz::named(section.name)->list != 0) {
			text += std::string{first ? "" : ", "} + "\"rows\": [";
			for (std::size_t row = 0; row < section.rows.size(); ++row) {
				text += (row == 0 ? "\n    " : ",\n    ") + section.rows[row].dump();
			}
			text += section.rows.empty() ? "]" : "\n  ]";
		}
		text += "}";
	}
	return text + "\n}\n";
}

std::vector<std::string> split(std::string const &List) {
	std::vector<std::string> result;
	std::size_t start{0};
	while (start < List.size()) {
		auto const end{std::min(List.find(',', start), List.size())};
		if (end > start) {
			result.push_back(List.substr(start, end - start));
		}
		start = end + 1;
	}
	return result;
}

} // namespace

// converts the game's definitions of the real vehicles to data/fiz, from the folder of the game's vehicles given
// in EU07_UNIT_TESTS_FIZ_SOURCE (dynamic/): the vehicles there already, and the ones named in EU07_UNIT_TESTS_FIZ_ADD
// (paths without pkp/ and the extension, separated by commas). the game's parser reads the file with its includes,
// the conversion keeps what the loader reads of it, and checks the loader reads the same of the converted one
TEST_CASE("fiz: conversion of the game's definitions, when asked for") {
	auto const *source{std::getenv("EU07_UNIT_TESTS_FIZ_SOURCE")};
	if (source == nullptr) {
		return;
	}
	auto paths{test::real_vehicles()};
	if (auto const *added{std::getenv("EU07_UNIT_TESTS_FIZ_ADD")}; added != nullptr) {
		for (auto const &path : split(added)) {
			paths.push_back(path);
		}
	}
	auto const root{std::filesystem::path{source}};
	for (auto const &path : paths) {
		CAPTURE(path);
		// the vehicles of the Polish railways are in pkp/, the others under their operators
		auto file{root / (path + ".fiz")};
		if (false == std::filesystem::exists(file)) {
			file = root / "pkp" / (path + ".fiz");
		}
		REQUIRE(std::filesystem::exists(file));
		cParser parser(file.filename().string(), cParser::buffer_FILE, file.parent_path().generic_string() + "/");
		auto const original{test::fiz::lines(parser)};

		auto const target{std::filesystem::path{EU07_UNIT_TESTS_DATA} / "fiz" / (path + ".json")};
		std::filesystem::create_directories(target.parent_path());
		{
			std::ofstream output(target, std::ios::binary);
			output << json_text(std::filesystem::relative(file, root).generic_string(), convert(original));
		}
		cParser converted(definition::fiz(path).text(), cParser::buffer_TEXT);
		auto const expected{test::fiz::reading(original)};
		auto const actual{test::fiz::reading(test::fiz::lines(converted))};
		auto const mismatch{std::ranges::mismatch(expected, actual)};
		if (mismatch.in1 != expected.end() || mismatch.in2 != actual.end()) {
			std::filesystem::remove(target);
			FAIL_CHECK("the loader reads the converted definition differently, at " << (mismatch.in1 != expected.end() ? *mismatch.in1 : "the end") << " against "
			                                                                      << (mismatch.in2 != actual.end() ? *mismatch.in2 : "the end"));
		}
	}
}
