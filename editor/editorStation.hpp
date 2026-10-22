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
};

// the station of the switch, or of the plain track near one
std::optional<layout> find(TTrack &From, options const &Options = {});
// the chainage along the axis and the offset to the left of it, as the map shows it
glm::dvec2 project(layout const &Layout, glm::dvec3 const &Point);
// km at the chainage, when the station knows its kilometrage
double km_of(layout const &Layout, double const Chainage);
// one switch of each station of the scenery
std::vector<TTrack *> stations(options const &Options = {});
// the switches of the station of the switch, without laying it out
std::vector<TTrack *> switches_of(TTrack &Switch, options const &Options = {});

} // namespace station
