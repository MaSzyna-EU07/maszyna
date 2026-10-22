/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class TTrack;

// a station as the signalling sees it: its switches, the tracks between them and the lines leading out, and its schematic,
// the tracks laid out along the main track through it
namespace station
{

struct options
{
	double link{1500.0}; // m, switches joined by plain track no longer than this belong to the same station
	double approach{400.0}; // m, of each line leading out, taken beyond the outermost switch
};

struct path
{
	TTrack *track{nullptr};
	int index{0}; // of the path of the track
	std::vector<glm::dvec3> points; // along the path, world space
	std::vector<glm::dvec2> schematic; // the chainage along the axis and the offset to the left of it, as the map shows it
	double length{0.0}; // in the plan
	bool approach{false}; // on a line leading out of the station
};

struct layout
{
	std::vector<TTrack *> switches; // switches, crossings and turntables, by the chainage of their start
	std::vector<path> paths;
	std::vector<glm::dvec3> axis; // the main route through the station
	std::vector<double> chainage; // of the points of the axis
	double from{0.0}; // the chainage over which the switches lie
	double to{0.0};
	double low{0.0}; // the offsets of the outermost tracks of the station
	double high{0.0};
	bool kilometrage{false}; // the chainage runs with the km of the switches
	double km{0.0}; // at chainage 0, when it does
	std::vector<std::vector<TTrack *>> lines; // the lines leading out, their tracks from the station on, as far as the approach
	std::vector<TTrack *> main; // the tracks of the axis
	std::vector<TTrack *> second; // of the other main track of a double-track line, none on a single-track one
};

// what shares a number of the station: a switch, or the switches of a slip, the two of a double switch, the paths of a crossing
struct unit
{
	std::vector<TTrack *> parts;
	std::string record; // the name of its record of the switches, empty without one
	double chainage{0.0}; // of the start of its first part along the station
};

// a joint of the track circuits; Track is the plain track to be cut there, none where two tracks meet already
struct joint
{
	TTrack *track{nullptr};
	glm::dvec3 point{0.0};
};

// a track circuit of the station, once its tracks are cut at the joints: of a switch, or of a station track
struct circuit
{
	std::vector<TTrack *> tracks;
	int unit{-1}; // of the switch, -1 for a station track
	double offset{0.0}; // the mean one of a station track, in the schematic
	double from{0.0}; // the chainage it spans
	double to{0.0};
	double length{0.0};
};

// the station of the switch, or of the plain track near one
std::optional<layout> find(TTrack &From, options const &Options = {});
// the chainage along the axis and the offset to the left of it, as the map shows it
glm::dvec2 project(layout const &Layout, glm::dvec3 const &Point);
// km at the chainage, when the station knows its kilometrage
double km_of(layout const &Layout, double const Chainage);
// one switch of each station of the scenery
std::vector<TTrack *> stations(options const &Options = {});
// the switches of the station by their numbers: along the chainage, or against it when Reversed; turntables have none
std::vector<unit> numbering(layout const &Layout, bool const Reversed);
// the names of the switches and the track circuits of the station begin with it: bpd_
std::string prefix(std::string const &Code);
// <code>_<number>, each part of a unit of several with a letter of its own: bpd_15a
std::string switch_name(std::string const &Code, int const Number, std::size_t const Part, std::size_t const Parts);
// the joints of the track circuits: 1.5 m before the start of a switch, beyond the fouling points of its branches where the axes
// are 4 m apart, in the middle of the links between switches too short for both, and so where the lines leave the station
std::vector<joint> joints(layout const &Layout, std::vector<unit> const &Units, options const &Options = {});
// the track circuits the joints part the station into; the tracks of the lines beyond the joints belong to none
std::vector<circuit> circuits(layout const &Layout, std::vector<unit> const &Units, std::vector<glm::dvec3> const &Joints);
// the numbers of the station tracks as PKP PLK gives them, one for each circuit, empty for those of switches: 1 and 2 the main
// tracks, track 1 on the right along the chainage, the odd ones beyond it on its side, the even ones on the other; a track goes
// on through the switches along their straighter paths, its circuits lettered along the chainage: 5a, 5b
std::vector<std::string> track_numbers(layout const &Layout, std::vector<circuit> const &Circuits);
// the switches of the station of the switch, without laying it out
std::vector<TTrack *> switches_of(TTrack &Switch, options const &Options = {});

} // namespace station
