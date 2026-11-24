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
#include "editor/editorSwitchDesign.hpp"

#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

// The turnouts of the track geometry (editor-tracks-domain.md §4 "Rozjazd", §7): the type of the catalogue the editor lays its switches
// from (editorSwitchDesign, Id-4), the diverging track laid off the start of the main track and bent as the line under it is (table 11.8,
// the curve of the basic switch bent to keep the angle of the frog, the rest of it along with the main track), the speeds by the rule the
// editor writes (switch_curve::branch_speed), and the rules of Q2 and §11.4 as invariants: the hard ones refuse, the others warn.
namespace layout
{

// a type of the catalogue as the model lays it: an ordinary switch (Rz) whose diverging track leaves the main one tangentially
struct turnout_type
{
	std::string name;
	std::string rail;
	double radius{0.0}; // m, of the diverging curve of the basic switch
	double ratio{0.0}; // 1:n at the end of the diverging track
	double a{0.0}; // m, from the start to the centre of the switch
	double b{0.0}; // m, from the centre to the end of the main track
	double length{0.0}; // m, of the main track
	double lead{0.0}; // m, the straight of the diverging track along the main one before its curve
	std::vector<switch_design::part> parts; // of the diverging track after the lead, turning to the side it leaves to
	bool movable{false}; // the frog with a movable nose
	double heel{0.0}; // where the moving blades end, 0: as the simulator makes them
	double bladethrow{0.0};

	double diverging_length() const;
};

// the type of the catalogue by its name; none, and the reason in Why, where there's no such type or the model doesn't lay it yet: double
// switches, slips and symmetric switches, the blades meeting the stock rails at an angle (stage E5)
std::optional<turnout_type> turnout_type_of(std::string const &Name, std::string *Why = nullptr);
// the names of the types the model lays, in the order of the catalogue
std::vector<std::string> turnout_types();

// the turnout on its line, as its start node and its main track tell it: the place, the heading along the turnout, the side of the start
// node it lies on, the curvature of the main track (its mean, 0 on a straight), the cant and the grade, all along the turnout
struct turnout_frame
{
	glm::dvec3 position{0.0};
	glm::dvec2 heading{0.0, 1.0};
	int side{1};
	int along{1}; // 1: the main track runs from the start of the turnout, -1 to it
	double curvature{0.0}; // 1/m, above 0 turning left
	double cant{0.0}; // mm, above 0 leaning left
	double grade{0.0};
	bool valid{false};
};
turnout_frame frame_of(network const &Network, turnout const &Turnout);

// the main track of the bending, signed as table 11.8 takes it: above 0 bent the way the diverging track leaves to, below 0 the other way,
// 0 a straight
double bending_of(double const Curvature, int const Hand);

// the diverging track laid from the frame, to the hand: the lead and the straights through the frog along the main track, the curve of the
// basic switch bent by table 11.8 over the length which keeps the angle of the frog, transitions by the sum of the curvatures; the cant
// of the main track all along it, the grade of its start. Errors where it can't be laid so
solved_line lay_diverging(turnout_frame const &Frame, turnout_type const &Type, int const Hand);

// what the turnout takes from the line under it: the kilometrage of its start, the bending, the cant, the speeds of both tracks (the
// diverging one by switch_curve::branch_speed, the main one by formula 9.9 with the deficiency of the frog where it's bent, no faster than
// the line), the velocity written
void derive_turnout(network const &Network, turnout &Turnout);

// the invariants of the turnout (its type, its legs and its tracks, the length of its main track) into the reasons; the rules of Q2 and
// §11.4 (switch_curve::check: the curvature of each track of one sign, the cant the same over it, the ramps off it by 6 m; no switch of
// 1:9.403 nor steeper than 1:9 bent in a main track) into the blocks, and its warnings (table 11.9, a transition under it) into the notices
void check_turnout(network const &Network, turnout const &Turnout, verdict &Result);

} // namespace layout
