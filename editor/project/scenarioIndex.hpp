/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <filesystem>
#include <set>
#include <string>
#include <vector>

// The scenarios using a map and what they name in it (editor-tracks-domain.md §0 pkt 7, §3 "Indeks scenariuszy"): a change of the map
// mustn't take away a track, an event or a memory cell a scenario relies on. Read only, from the text of the files as the simulator reads
// it: the scenarios are the *.scn files of the scenery folder, a scenario uses the map where the file of the map is among its files (its
// includes followed, as the simulator follows them: relative to the folder, with the parameters of the templates), and what it names in
// the map is what its statements outside the files of the map name of the tracks, the events and the memory cells the files of the map
// define, the trainsets wherever they are.
//
// What isn't seen: the names made in the simulator (the tracks of the roads, the parts of the turntables), the names in the scripts
// (lua), in the timetables and in the commands the memory cells give the drivers, the random includes; the names compared as the parser
// makes them, the letters A to Z small.
namespace project
{

enum class reference_kind
{
	track,
	event,
	memcell
};

enum class reference_use
{
	trainset, // a trainset starts on the track
	vehicle, // a vehicle standing on its own is put on the track
	event_target, // an event acts on it: switch, trackvel, trackbuckle on a track; updatevalues, getvalues, copyvalues, whois, logvalues on a cell
	condition, // the condition of an event checks it: trackoccupied, trackfree, memcompare
	copy_source, // copyvalues takes from the cell
	event_child, // a multiple event calls the event
	launcher, // an event launcher calls the event
	launcher_condition, // an event launcher checks the cell
	track_event, // a track of the scenario calls the event (event0, event1, event2, eventall0...)
	memcell_track, // a memory cell of the scenario names the track
	isolated, // an isolated circuit of the scenario holds the track
	named_event // an event of the scenario named after the track (<track>:event1 ...), which the track calls
};

char const *kind_name(reference_kind const Kind);
char const *use_name(reference_use const Use);

struct reference
{
	reference_kind kind{reference_kind::track};
	std::string name; // made small, as the simulator compares the names
	reference_use use{reference_use::trainset};
	std::string from; // the event, the trainset, the node of the scenario which names it
	std::string file; // where that is, relative to the scenery folder
	std::size_t line{0};

	auto operator<=>(reference const &) const = default;
};

struct scenario_use
{
	std::string scenario; // the file, relative to the scenery folder
	std::vector<reference> references; // in the order of their kinds, names and uses
};

struct scenario_index
{
	std::string map;
	// what the files of the map define, the names made small
	std::set<std::string> tracks;
	std::set<std::string> events;
	std::set<std::string> memcells;
	std::vector<scenario_use> scenarios; // the ones using the map, in the order of their files
	std::vector<std::string> problems; // includes which can't be read and the like, by scenario
};

// the scenarios of the scenery folder using the map (relative to the folder), what each one names in it
scenario_index index_scenarios(std::filesystem::path const &Folder, std::string const &Map);

} // namespace project
