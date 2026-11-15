/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <string>
#include <vector>

#include "editor/editorTrack.hpp"

// connections of the straight tracks of a station made of switches: a crossover of two switches joined by a straight insert or
// by an aligning curve between straight inserts, a double crossover of two crossovers one after the other, a scissors crossover
// of two crossovers around one centre with a crossing between their inserts, a ladder of switches leading onto a fan of tracks
namespace connection
{

enum class kind
{
	crossover,
	double_crossover,
	scissors,
	ladder
};
int constexpr kind_count{4};
char const *word(kind const Kind);
bool parse(std::string const &Word, kind &Kind);

struct line
{
	glm::dvec3 start{0.0};
	glm::dvec2 direction{0.0, 1.0};
	double grade{0.0};
	double length{0.0};
	glm::dvec3 point(double const Along) const;
	double along(glm::dvec3 const &Point) const;
	// to the left of the direction
	double lateral(glm::dvec3 const &Point) const;
};
line of(editor_track::straight const &Straight);

struct placed
{
	int line{0}; // 0 the starting track, 1 the other one, or the tracks of a fan in their order from 1, -1 the ladder track
	double along{0.0}; // of the start of the switch along its line
	int forward{1};
	int side{1};
	bool second{false}; // made of the template of the other tracks
	std::vector<segment_data> paths;
};

// the switch on a track facing the end of a diverging track and the straight insert joining the two
struct facing
{
	bool valid{false};
	placed partner;
	segment_data insert;
	double length{0.0};
	std::string error;
};
facing face(glm::dvec3 const &End, glm::dvec2 const &Heading, line const &To, int const Index, editor_track::switch_template const &Shape);

struct request
{
	kind type{kind::crossover};
	line from; // the track the connection starts on
	std::vector<line> to; // the other track; for a ladder the tracks of the fan, the nearest first, or none
	double along{0.0}; // of the start of the first switch on the starting track
	int forward{1};
	int side{0}; // of a ladder with no tracks of the fan to lead onto
	double target{0.0}; // where the switch on the other track starts along it when no straight insert fixes the place
	editor_track::switch_template first;
	editor_track::switch_template second; // on the other tracks and along the ladder
	double radius{0.0}; // of the aligning curves and of the end of a ladder, 0: of the sharper switch
	double insert{6.0}; // the shortest straight between curves turning the opposite ways
	double gap{6.0}; // of a double crossover: between the starts of the switches meeting on one track
	bool together{false}; // of a double crossover: the switches meet on the starting track, not on the other one
	std::vector<double> spacings; // of a ladder with no tracks of the fan: from the starting track to the first one and on
	double stub{25.0}; // of a ladder: length of the new tracks of the fan
};

struct result
{
	bool valid{false};
	std::string error;
	std::string warning;
	std::vector<placed> switches;
	std::vector<segment_data> inserts;
	std::vector<double> straights; // lengths of the straight inserts between the switches
	double spacing{0.0}; // of the tracks at the start of the first switch
	double points{0.0}; // between the mathematical points of the switches of the first crossover
	double radius{0.0}; // of the aligning curve, 0 when a straight insert joins the switches
	std::vector<double> fan; // offsets of the tracks of a ladder from the starting track
	bool crossing{false}; // of a scissors crossover, at the meeting of its two inserts
	std::array<glm::dvec3, 4> ends{}; // of the two inserts crossing: along the first one, then along the second one
	glm::dvec3 centre{0.0};
	double angle{0.0};
};
result plan(request const &Request);

} // namespace connection
