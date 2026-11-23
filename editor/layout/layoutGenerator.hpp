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

#include <map>
#include <string>
#include <vector>

// The generator of the tracks of the simulator (editor-tracks-domain.md §3 and §5): the layer between the domain and the simulator,
// which knows nothing of the domain. A line is cut into cubic Bezier pieces, each one a track entry of the text TTrack::Load reads:
// within the tolerance of the element it follows (half a millimetre by default, the curve measured as the simulator draws it, a straight
// of zero control vectors where it takes the curve for one), the rolls and the heights from the functions of the line, the radius from
// the element. A piece ends at a node with the numbers of the node, the same on both sides of it; the names of the tracks come from the
// identifiers of the line and of its vertices, so they don't change with a change of the shape which keeps its elements.
namespace layout
{

// a joint of an old track the pieces are to end at, the name of the old track kept for the piece before it (stage E6)
struct hard_split
{
	double chainage{0.0};
	std::string name;
};

struct generator_options
{
	std::string prefix{"l"}; // of the names of the tracks: <prefix><line>_<element><piece>
	double tolerance{0.0005}; // m, the farthest a piece may lie from its line, in the plan and in the height
	double cant_tolerance{2.0}; // mm the cant may differ from the line between the ends of a piece, where the simulator turns it evenly
	// what an entry has between the type and the path: the width, the friction, the sound distance, the quality, the damage, the
	// environment and the visibility with the materials
	std::string surface{"1.435 0.15 25 20 0 flat vis rail_screw_used1 4 tpbps-new2 0.2 0.5 1.1"};
	// the joints of the old tracks as hard points of the pieces, by line: stage E6 (§10, "stare złącza jako twarde punkty podziału").
	// Not yet: a line given any is refused, so nothing is made which would lose the names and the events of the old tracks
	std::map<line_id, std::vector<hard_split>> hard_splits;
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

generated_line generate(network const &Network, line_id const Line, generator_options const &Options = {});
// all the lines of the network, in the order of their identifiers; the errors of the lines which couldn't be made in Errors
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
