/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutNetwork.hpp"
#include "world/Segment.h"

#include <array>
#include <map>
#include <string>
#include <vector>

// The generator of the tracks of the simulator (editor-tracks-domain.md §3 and §5): the layer between the domain and the simulator,
// which knows nothing of the domain. A line is cut into cubic Bezier pieces, each one a track entry of the text TTrack::Load reads:
// within the tolerance of the element it follows (half a millimetre by default, the curve measured as the simulator draws it, a straight
// of zero control vectors where it takes the curve for one), the rolls and the heights from the functions of the line, the radius from
// the element. A piece ends at a node with the numbers of the node, the same on both sides of it; the names of the tracks come from the
// identifiers of the line and of its vertices, so they don't change with a change of the shape which keeps its elements. A turnout is one
// switch track of two paths, its main and its diverging track, each made of the pieces of its line (pieces1, pieces2), the ends of its legs
// the numbers of its nodes, its velocity as the speed rule gives it, its frog and its blades as its type has them; a double switch, a slip
// and a crossing the switch tracks and the plain tracks the switch tool of the editor makes of them, their joints inside the turnout the
// numbers of the points of its lines there.
//
// A line taken over from old tracks (stage E6) keeps their joints as hard points of the pieces (line::hard_splits, §10 "stare złącza jako
// twarde punkty podziału"): the pieces end at each, and the pieces of the stretch of an old track carry its name, its surface and its
// attributes (the speed, the events, the isolated circuits), each written the way the old track ran; where the stretch takes more than one
// piece, the one the old track ran into first keeps the name and the events, the others are <name>_2, <name>_3... with the rest.
namespace layout
{

struct generator_options
{
	std::string prefix{"l"}; // of the names of the tracks: <prefix><line>_<element><piece>
	std::string turnout_prefix{"t"}; // of the names of the switch tracks: <turnout_prefix><turnout>
	double tolerance{0.0005}; // m, the farthest a piece may lie from its line, in the plan and in the height
	double cant_tolerance{2.0}; // mm the cant may differ from the line between the ends of a piece, where the simulator turns it evenly
	// what an entry has between the type and the path: the width, the friction, the sound distance, the quality, the damage, the
	// environment and the visibility with the materials
	std::string surface{"1.435 0.15 25 20 0 flat vis rail_screw_used1 4 tpbps-new2 0.2 0.5 1.1"};
};

struct generated_piece
{
	std::string name;
	double from{0.0}; // chainage along the line
	double to{0.0};
	segment_data path; // its control vectors relative to its ends, zero for a straight, as the entry has them
	double deviation{0.0}; // m, the farthest the curve as the simulator draws it lies from the line, measured when cut
	std::string text; // the entry
};

struct generated_line
{
	line_id line{line_id::none};
	std::vector<generated_piece> pieces;
	std::vector<std::string> errors;
	std::string text() const;
};

// the pieces of a line of its own; a track of a turnout is made with its turnout
generated_line generate(network const &Network, line_id const Line, generator_options const &Options = {});

// a track of the simulator a turnout is made of: a switch track of two paths, or a plain track (each of its pieces an entry)
struct generated_part
{
	std::string name;
	bool is_switch{false};
	// of a switch the main and the diverging path from its start, each the pieces of its line, as the paths of the switch have them: the
	// path the pieces make, whole, with them (none where one piece makes it); of a plain track its pieces
	std::array<std::vector<generated_piece>, 2> pieces;
	std::array<segment_data, 2> paths;
	std::string text; // the entries
};

struct generated_turnout
{
	turnout_id turnout{turnout_id::none};
	std::string name;
	// the main and the diverging track from the start of the turnout, each the pieces of its line, as the paths of the switch have them:
	// the path the pieces make, whole, with them (none where one piece makes it); of its first switch track, where it has more
	std::array<std::vector<generated_piece>, 2> pieces;
	std::array<segment_data, 2> paths;
	double velocity{0.0};
	std::vector<std::string> errors;
	// the tracks of the simulator it's made of: an ordinary or a symmetric switch one switch track; a double switch two and a short plain
	// track between them (<name>, <name>_l, <name>_2); a slip a switch at each leg a slip curve leaves (<name>_a ... _d, plain tracks at the
	// others), the tracks crossing and the middles of the slip curves plain ones (_m, _x, _s1, _s2); a crossing two plain tracks (_1, _2)
	std::vector<generated_part> parts;
	std::string text; // the entries, of all the parts
};
generated_turnout generate(network const &Network, turnout_id const Turnout, generator_options const &Options = {});

// all the lines of the network in the order of their identifiers, then all the turnouts; the errors of the ones which couldn't be made in
// Errors
std::string generate_all(network const &Network, generator_options const &Options, std::vector<std::string> &Errors);

// the curve the simulator draws of the path: a straight where its control vectors are zero, or so close to the third of the chord that
// TTrack takes it for one (its check of 2 cm, TTrack::straightened)
struct drawn_curve
{
	glm::dvec3 p0;
	glm::dvec3 p1;
	glm::dvec3 p2;
	glm::dvec3 p3;
	glm::dvec3 point(double const T) const;
};
drawn_curve drawn(segment_data const &Path);
bool straightened(segment_data const &Path);
// the farthest the curve drawn lies from the line between the chainages: each point of it set square onto the line, the distance in the
// plan and in the height together
double deviation(solved_line const &Line, drawn_curve const &Curve, double const From, double const To);

} // namespace layout
