/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "scene/scenelayers.h"

class TTrack;

// switches placed in the track editor, kept in the scenery files as "//$r" comment lines: their type, size and side, the
// paths a slip or a crossing is made of, their km and number, so a switch is known again without guessing from its shape
namespace switch_store
{

struct record
{
	scene::layer_handle layer{null_handle};
	std::string kind{"switch"}; // switch, slip, crossing, double, connection
	std::string name; // the switch, or the base name of a slip or a crossing
	std::string type{"Rz"}; // Rz, Rlj, Rld, Rkp, Rkpd, Skrz, Rpd, Rpj, - when unknown
	std::string rail{"-"};
	double radius{0.0}; // of the diverging track of the straight switch the type is made of
	double ratio{0.0}; // n of 1:n
	char hand{'-'}; // L, R as seen along the switch from its start
	double a{0.0};
	double b{0.0};
	double main_radius{0.0}; // of the main track, signed: + to the right, - to the left, 0 straight
	std::vector<std::string> parts; // the switch, the switches of a slip or of a double switch, the paths of a crossing
	glm::dvec3 start{0.0}; // of the first part
	glm::dvec2 direction{0.0, 1.0};
	// ends of the main and the diverging track of the first part seen from its start: along, across, along, across. it stays the
	// same as long as the switch is moved or turned as a whole, it changes when the switch gets bent out of its shape
	glm::dvec4 shape{0.0};
	bool shaped{false};
	std::optional<double> km;
	std::string number{"-"};
	std::string source{"designed"}; // designed: from the catalogue, measured: copied from a switch of the scenery
	double middle{0.0}; // the radius in the middle of the slip curves of a double slip with the blades outside of the diamond
	// a crossover laid along a curve: the outer track went up by lift over it, level between the two points, with ramps of
	// ramp m on both sides
	double lift{0.0};
	glm::dvec3 lift_from{0.0}, lift_to{0.0};
	double ramp{0.0};
};

void reset();
void load();
std::vector<record> const &records();
// a record of the name is kept
bool has(std::string const &Name);
// the record of the switch, slip or crossing the track is a part of, while its first part keeps the shape the record says
std::optional<record> of(TTrack const &Track);
// the record of the track by its name alone, also when the switch lost its shape
std::optional<record> named(TTrack const &Track);
// the connections of switches the switch is a part of
std::vector<record> connections(TTrack const &Track);
// records whose switches were moved or turned as a whole take their new place and km; returns their names
std::vector<std::string> follow();
// radius of the diverging track: of the type, bent with the main track for an arc switch
double branch_radius(record const &Record);
// keeps the record in the file of the layer, in place of the one with the same name
void put(record Entry, scene::layer_handle const Layer);
void place(record &Entry, TTrack const &First);
void classify(record &Entry, TTrack const &Switch);
// of the first path of the switch, signed as record::main_radius: + to the right, - to the left, 0 straight
double main_radius_of(TTrack const &Switch);
void forget(std::string const &Name);
// the records follow renamed switches: their parts and the records named after them; Bases renames the records of slips and
// crossings, whose names aren't the names of their parts
void rename(std::map<std::string, std::string> const &Parts, std::map<std::string, std::string> const &Bases);
// the number of the switch, slip or crossing of the record of the name, as the lists of the switches give it
void number(std::string const &Name, std::string const &Number);
// on save: the records follow the renamed switches, the ones whose paths are gone are dropped
void tidy();
// "Rz 60E1-500-1:12 L", as the type is written on the lists of the switches
std::string label(record const &Record);
// the records as they were before the changes made since the last call, if there were any; the current ones become the base
std::optional<std::vector<record>> take_changes();
// the current records become the base, the changes made so far aren't a part of the undo history
void settle();
// puts the records back as they were, for undo and redo
void restore(std::vector<record> Records);

} // namespace switch_store
