// the scenarios using a map and what they name in it (editor/project/scenarioIndex.cpp, editor-tracks-domain.md §3 "Indeks
// scenariuszy"): the small scenery of data/project/index, a map of tracks, events and memory cells (some from a template included with
// parameters) and three scenarios, two of them using the map (one through a file of its own), one another map with tracks of the same
// names; the trainsets on the tracks of the map, the events of the map called, its memory cells used, the rest of the names left out

#include "stdafx.h"
#include "doctest.h"
#include "editor/project/scenarioIndex.hpp"
#include "editor/project/sceneryText.hpp"

#include <format>
#include <string>

namespace {

std::filesystem::path folder() {
	return std::filesystem::path{EU07_UNIT_TESTS_DATA} / "project" / "index";
}

// a reference as a line: the kind, the name, the use, what names it, where
std::string described(project::reference const &Reference) {
	return std::format("{} {} {} {} {}:{}", project::kind_name(Reference.kind), Reference.name, project::use_name(Reference.use), Reference.from, Reference.file, Reference.line);
}

std::string described(std::vector<project::reference> const &References) {
	std::string result;
	for (auto const &reference : References) {
		result += described(reference) + "\n";
	}
	return result;
}

} // namespace

TEST_CASE("scenario index: the scenarios using the map, what the map defines and what each scenario names in it") {
	auto const index{project::index_scenarios(folder(), "mapa/mapa.scm")};
	CHECK(index.problems.empty());
	CHECK(index.map == "mapa/mapa.scm");
	// the template of the signal gives the names of its parameters; a comment defines nothing
	CHECK(index.tracks == std::set<std::string>{"tor_1", "tor_2", "tor_3", "zwr_1"});
	CHECK(index.events == std::set<std::string>{"sem_a_s1", "sem_a_sr2", "tor_1_zajety", "zwr_1_plus"});
	CHECK(index.memcells == std::set<std::string>{"sem_a_mem", "stacja_mem"});
	// the scenario of another map isn't among them, the one including the map through a file of its own is
	REQUIRE(index.scenarios.size() == 2);
	CHECK(index.scenarios[0].scenario == "poranek.scn");
	CHECK(index.scenarios[1].scenario == "wieczor.scn");

	// the trainset on a track of the map, the events of the map called by an event, by a track and by a launcher of the scenario, the
	// switch and the cell acted on, the cell checked by the launcher, the track whose event the scenario names after it; not the trainset
	// on a track of the scenario, the events of the scenario, the names in a comment, an event of none_, a name defined nowhere
	CHECK(described(index.scenarios[0].references) ==
	      "track tor_1 trainset rano poranek.scn:13\n"
	      "track tor_2 named event tor_2:event1 poranek.scn:9\n"
	      "track zwr_1 event target zwrotnica poranek.scn:7\n"
	      "event sem_a_s1 multiple event start_poranek poranek.scn:6\n"
	      "event sem_a_s1 event launcher przycisk poranek.scn:10\n"
	      "event sem_a_sr2 track event bocznica poranek.scn:5\n"
	      "event tor_1_zajety multiple event start_poranek poranek.scn:6\n"
	      "memcell sem_a_mem event launcher condition przycisk poranek.scn:10\n"
	      "memcell stacja_mem event target komorka poranek.scn:8\n");
	// the isolated circuit, the vehicle on its own, the trainset, the conditions of the events on a track and on a cell, the cell copied
	// from, the track of a cell of the scenario; not the cell copied into, which is the scenario's
	CHECK(described(index.scenarios[1].references) ==
	      "track tor_1 isolated odc_1 wieczor.scn:5\n"
	      "track tor_2 vehicle stojak wieczor.scn:10\n"
	      "track tor_2 isolated odc_1 wieczor.scn:5\n"
	      "track tor_3 trainset wieczorny wieczor.scn:11\n"
	      "track tor_3 condition zajetosc wieczor.scn:8\n"
	      "track tor_3 memory cell track moja_mem wieczor.scn:4\n"
	      "event sem_a_s1 multiple event warunek wieczor.scn:7\n"
	      "event sem_a_sr2 multiple event zajetosc wieczor.scn:8\n"
	      "memcell sem_a_mem condition warunek wieczor.scn:7\n"
	      "memcell stacja_mem copy source kopia wieczor.scn:6\n");

	// a file of the map is used by the same scenarios; a map nobody uses
	auto const part{project::index_scenarios(folder(), "mapa/tory.scm")};
	CHECK(part.scenarios.size() == 2);
	CHECK(part.events.empty());
	auto const none{project::index_scenarios(folder(), "nie/ma.scm")};
	CHECK(none.scenarios.empty());
	CHECK(none.tracks.empty());
}

TEST_CASE("scenario index: the scenery read as the simulator reads it, the includes followed with their parameters and placements") {
	scenery_text::read_options options;
	options.map = "mapa/mapa.scm";
	auto const scenery{scenery_text::read(folder(), "wieczor.scn", options)};
	CHECK(scenery.problems.empty());
	CHECK(scenery.uses_map());
	// the scenario, its file, the map and the files of the map
	REQUIRE(scenery.files.size() == 6);
	CHECK(scenery.files[0].name == "wieczor.scn");
	CHECK_FALSE(scenery.files[0].map);
	CHECK(scenery.files[1].name == "wieczor/wstep.scm");
	CHECK_FALSE(scenery.files[1].map);
	CHECK(scenery.files[2].name == "mapa/mapa.scm");
	CHECK(scenery.files[2].map);
	CHECK(scenery.files[5].name == "mapa/semafor.inc");
	CHECK(scenery.files[5].map);
	CHECK(scenery.files[5].parameters == std::vector<std::string>{"sem_a", "10.0", "5.0", "tor_2"});
	CHECK(scenery.files[5].context.offset == glm::dvec3{100.0, 0.0, 0.0});
	CHECK(scenery.files[3].context.offset == glm::dvec3{0.0});
	// the memory cell of the template, its parameters in place
	auto const cell{std::ranges::find_if(scenery.statements, [](scenery_text::statement const &Statement) { return Statement.keyword == "node" && Statement.word(2) == "sem_a_mem"; })};
	REQUIRE(cell != scenery.statements.end());
	CHECK(cell->map);
	CHECK(cell->word(4) == "10.0");
	CHECK(cell->word(10) == "tor_2");
	CHECK(cell->words.back().text == "endmemcell");
	// a file which isn't there is told, the rest read
	auto const missing{scenery_text::read(folder(), "brak.scn", options)};
	CHECK(missing.problems.size() == 1);
	CHECK(missing.statements.empty());
	CHECK_FALSE(missing.uses_map());
}

TEST_CASE("scenario index: the words of the text as the parser splits them") {
	auto const tokens{scenery_text::split("node -1 0 a;track // a comment\n\"two words\" /* a\ncomment */ endtrack//glued\nx")};
	REQUIRE(tokens.size() == 8);
	CHECK(tokens[3].text == "a");
	CHECK(tokens[4].text == "track");
	CHECK(tokens[5].text == "\"two words\"");
	CHECK(tokens[5].line == 2);
	CHECK(tokens[6].text == "endtrack");
	CHECK(tokens[6].line == 3);
	CHECK(tokens[7].text == "x");
	CHECK(tokens[7].line == 4);
	CHECK(scenery_text::lowered("Tor_A:Event1 \xC5\xBB") == "tor_a:event1 \xC5\xBB");
	CHECK(scenery_text::same_file("Mapa\\Tory.SCM", "mapa/tory.scm"));
}
