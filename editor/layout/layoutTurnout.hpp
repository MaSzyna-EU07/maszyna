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

// The turnouts of the track geometry (editor-tracks-domain.md §4 "Rozjazd", §7): the types of the catalogue the editor lays its switches
// from (editor/data/turnouts.txt, Id-4), each track the turnout has a line and each leg a node; the diverging track laid off the start of the
// main track and bent as the line under it is (table 11.8, the curve of the basic switch bent to keep the angle of the frog, the rest of it
// along with the main track), the speeds by the rule the editor writes (switch_curve::branch_speed), and the rules of Q2 and §11.4 as
// invariants: the hard ones refuse, the others warn.
//
// The kinds of the catalogue as the model lays them:
//   ordinary   Rz: its main track a part of the line it lies on, the diverging track off its start, tangent to it or with secant blades
//              meeting the stock rail at an angle at the end of the pre-blade stretch (S42)
//   symmetric  Rls: both its tracks turning off its start by half of the angle, to both sides; it ends the line it's laid at the free end of
//   twofold    Rpd, Rpj, Rps: its main track a part of the line cut at the start of its second switch (the node second), the main track of
//              which goes on (main2) and whose diverging track (diverging2) leaves there, to the other side or to the same one
//   slip       Rkp, Rkpd: its main track a part of the line, the crossing track across its middle at the angle of the slip, between two nodes
//              of its own (cross_start, cross_end), the slip curves between the two: the diverging track from its start to the end of the
//              crossing track, and of a double slip the second one (diverging2) from the start of the crossing track to the end of the main one
//   crossing   Skrz: its main track a part of the line, the crossing track across it
// Doubles, slips and crossings aren't bent (11.4 pkt 4 a, b), nor symmetric switches (table 11.8 has no rule for them): their main tracks lie
// on straights.
namespace layout
{

enum class turnout_kind
{
	ordinary,
	symmetric,
	twofold,
	slip,
	crossing
};

// a type of the catalogue as the model lays it
struct turnout_type
{
	std::string name;
	std::string rail;
	turnout_kind kind{turnout_kind::ordinary};
	double radius{0.0}; // m, of the diverging curve of the basic switch; of the slip curves of a slip
	double ratio{0.0}; // 1:n at the end of the diverging track; of the two tracks of a symmetric switch, of the crossing of a slip
	double a{0.0}; // m, from the start to the centre of the switch; a slip or a crossing from its centre to its legs
	double b{0.0}; // m, from the centre to the end of the main track
	double length{0.0}; // m, of the main track; of a symmetric switch how far its tracks reach along it
	double lead{0.0}; // m, the straight of the diverging track along the main one before its curve
	double entry{0.0}; // rad, the angle the secant blades meet the stock rail at, at the end of the lead; 0 for tangent blades
	double tip{0.0}; // m from the start, the tip of the blade
	std::vector<switch_design::part> parts; // of the diverging track after the lead, turning to the side it leaves to
	bool movable{false}; // the frog with a movable nose
	double heel{0.0}; // where the moving blades end, 0: as the simulator makes them
	double bladethrow{0.0};
	// a double switch: where its second switch starts along the main track, its diverging track, to the other side or the same one
	double stagger{0.0};
	double lead2{0.0};
	double entry2{0.0};
	double tip2{0.0};
	std::vector<switch_design::part> parts2;
	bool opposite{false};
	// a slip or a crossing: half of the length of its tracks, the angle they cross at; the slip curves, one of a single slip, two of a
	// double one, none of a crossing
	double half{0.0};
	double angle{0.0};
	switch_design::slip_curve curve;
	int slip_curves{0};

	double diverging_length() const;
	bool bendable() const { return kind == turnout_kind::ordinary; }
};

// the kind of the turnout, by the tracks it has and its type
turnout_kind kind_of(turnout const &Turnout);

// the type of the catalogue by its name; none, and the reason in Why, where there's no such type or the model can't lay it
std::optional<turnout_type> turnout_type_of(std::string const &Name, std::string *Why = nullptr);
// the names of the types the model lays, in the order of the catalogue
std::vector<std::string> turnout_types();
// m, from the crossing to the rails of a slip curve passing it, of a standard gauge track of the rail profile of the simulator: half of
// the gauge, the head and 2 mm, as the switch tool takes it for the slip curves with the blades outside of the diamond
inline constexpr double kSlipPass{0.7175 + 0.0735 + 0.002};

// the turnout on its line, as its start node and its main track tell it: the place, the heading along the turnout, the side of the start
// node it lies on, the curvature of the main track (its mean, 0 on a straight), the cant and the grade, all along the turnout; of the second
// switch of a double one as its node and the main track going on from there tell it
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
turnout_frame second_frame_of(network const &Network, turnout const &Turnout);

// why the turnout of a kind which isn't bent can't lie where it does: its main track in a curve (ST-T1-A6 11.4 pkt 4 a, b; table 11.8 has
// no rule for a symmetric switch); empty where it can, or it's an ordinary switch, which is bent
std::string bent_refusal(network const &Network, turnout const &Turnout, turnout_type const &Type);

// the main track of the bending, signed as table 11.8 takes it: above 0 bent the way the diverging track leaves to, below 0 the other way,
// 0 a straight
double bending_of(double const Curvature, int const Hand);

// the diverging track laid from the frame, to the hand: the lead and the straights through the frog along the main track, the curve of the
// basic switch bent by table 11.8 over the length which keeps the angle of the frog, transitions by the sum of the curvatures; the cant
// of the main track all along it, the grade of its start. Errors where it can't be laid so
solved_line lay_diverging(turnout_frame const &Frame, turnout_type const &Type, int const Hand);

// a track the turnout lays itself, not a part of the line it lies on: where it starts and ends, and the nodes it puts where it starts or
// ends (the nodes of the turnout's own: the ends of the diverging tracks, the ends of the crossing track)
struct laid_track
{
	line_id line{line_id::none};
	node_id start{node_id::none};
	node_id end{node_id::none};
	bool sets_start{false};
	bool sets_end{false};
	solved_line solved;
};
struct laid_tracks
{
	std::vector<laid_track> tracks;
	std::vector<std::string> errors;
};
// the tracks of the turnout it lays off its frame, the parts of the line it lies on laid as they are
laid_tracks lay_tracks(network const &Network, turnout const &Turnout, turnout_type const &Type);

// the lines of the turnout: its main track (and the one of its second switch), its diverging tracks, its crossing track
std::vector<line_id> tracks_of(turnout const &Turnout);
// the line is a track the turnout lays itself, not a part of the line the turnout lies on
bool laid_by_turnout(network const &Network, line_id const Line);

// what the turnout takes from the line under it: the kilometrage of its start, the bending, the cant, the speeds of both tracks (the
// diverging one by switch_curve::branch_speed, the main one by formula 9.9 with the deficiency of the frog where it's bent, no faster than
// the line), the velocity written
void derive_turnout(network const &Network, turnout &Turnout);

// the invariants of the turnout (its type, its legs and its tracks, the length of its main track) into the reasons; the rules of Q2 and
// §11.4 (switch_curve::check: the curvature of each track of one sign, the cant the same over it, the ramps off it by 6 m; no switch of
// 1:9.403 nor steeper than 1:9 bent in a main track) into the blocks, and its warnings (table 11.9, a transition under it) into the notices
void check_turnout(network const &Network, turnout const &Turnout, verdict &Result);

} // namespace layout
