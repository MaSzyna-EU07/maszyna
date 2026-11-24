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

#include <string>
#include <vector>

// The solver of the connections of the tracks (editor-tracks-domain.md §7 "Połącz tory", §8): a crossover between two lines running
// alongside, straight or curved, made of a turnout on each, their diverging tracks facing each other, a line between their ends. The second
// turnout is put where the line between the ends runs as one straight (the tangents of both ends on it), or, between curved lines, where it
// turns by as much at both ends (one curve between them). Checked before it's laid: the spacing of the lines measured along the whole
// crossover, not at one point, against the least of ST-T1-A6 tab. 11.2 for the type (or what its geometry needs where the table hasn't it);
// the line between the turnouts no shorter than the norms ask of the element between two turnouts (the module of the norms,
// parameter::switch_straight: max(0.15 V; 6) m, 30 m on a line of 160 km/h or more). Where the norms can't be met it's refused with the best
// which fits: another type of the catalogue, or the spacing the type needs.
namespace layout
{

// the crossover asked for: the first turnout's line, where it starts and which way it runs (the crossover leads that way), the other line
// and the type of both turnouts
struct connect_tracks
{
	line_id from{line_id::none};
	double chainage{0.0};
	int direction{1};
	line_id to{line_id::none};
	std::string type;
};

struct connection_plan
{
	bool fits{false};
	std::vector<std::string> reasons; // why it doesn't, the proposal among them
	int hand{1}; // of the first turnout: the side the other line lies on
	double chainage{0.0}; // where the second turnout starts along its line
	int direction{1}; // the second turnout along its line or against it, its diverging track facing the first one's
	int hand2{1};
	double spacing_least{0.0}; // m, of the lines along the crossover, every metre of it
	double spacing_most{0.0};
	double spacing_needed{0.0}; // m: of tab. 11.2 for the type, or of its geometry with the least straight
	double straight{0.0}; // m, between the ends of the diverging tracks
	double straight_needed{0.0}; // m, of the norms
	double turn{0.0}; // rad, by which the line between the ends turns: 0 between straights alongside
	std::string proposal; // the best which fits where this doesn't
};

// the least spacing of two parallel straight lines a crossover of the type needs with a straight between its diverging tracks of the
// length: twice the offset of the end of a diverging track and the straight across at the angle of the turnout; 0 where it's no ordinary
// switch
double crossover_spacing(std::string const &Type, double const Straight);

connection_plan plan_connection(network const &Network, connect_tracks const &Request);

} // namespace layout
