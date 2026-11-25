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
#include <glm/glm.hpp>

#include "world/RoadCrossing.h"
#include "world/Segment.h"

// the surface of a road where it crosses the rails: over each track it lies in the plane of the heads of its rails, across the
// track with its cant and along it with its grade, the tracks joined smoothly where they lie at other heights, and the road led
// in from either side within the grades the regulation allows. The road cuts itself along the rails where it lies so, as it builds
// its geometry (world/RoadCrossing.h): the heads uncovered for the treads of the wheels, the flangeways for their flanges. A crossing
// panel laid on the crossing (the panels CBP made of their drawings, world/CrossingPanels.h, or a model of the slabs) is the surface
// there: the road leaves its own out under it, and stays at the heads, which its lanes and the vehicles on them do too.
// The values of the regulation each come with their source, what isn't confirmed said so, as in the catalogue of the turnouts
namespace level_crossing
{

// Dz.U. 2025 poz. 1105 (the regulation on the crossings of the railway lines with the roads, of 9.07.2025, in force from 27.08.2025; it
// repealed Dz.U. 2015 poz. 1744), annex 5 pt 1: the grade of the road no more than 8% over the length of the crossing, no more than
// 3% on the approaches over 20 m from the outer rail (2.5% in the one of 2015)
double constexpr approach_grade{0.03};
double constexpr approach_length{20.0};
// annex 5 pt 1 there: in good conditions the grade line of the road over the crossing without breaks, the tracks with cant laid at
// levels which give it one (their heads in one plane); pt 3: in difficult conditions the breaks no more than 5% apart, more only with an
// analysis of how the vehicles get over it, which the editor can't make (Id-111 ch. 3 the same 5% for the roads over the panels CBP).
// A hard limit: a road which would break more at the heads of two tracks side by side isn't laid; one breaking less is told of
double constexpr break_limit{0.05};
// § 31 ust. 4 there: the rails of a track on a curve lie level across, or across at the grade of the road over the crossing, at no
// more than 8% (standard and broad gauge), 2% (narrow gauge); annex 5 pt 1: the road over the crossing no steeper than 8%. PLK
// ST-T1-A6 9.2 pkt 6 takes the cant at the crossings from the regulation, within its tab. 9.6. A hard limit: a road over a track canted
// more than that along it isn't laid, the cant of the track can be lowered to it. With the angle the road crosses at and the grade of
// the track, the most cant allowed is 1500 mm * sin(atan((8% - grade * cos(angle)) / sin(angle))): 119 mm square to a level track (the
// slope of 8% a rise of 119.6 mm over the 1500 mm between the rail axes)
double constexpr cross_slope_limit{0.08};
double constexpr narrow_cross_slope_limit{0.02};
// § 31 ust. 3 there: no level crossing on a cant ramp. A hard limit: a road over a track where its cant changes along it isn't laid
// the gaps along the rails, their widths and the plane of the heads: world/RoadCrossing.h
using road_crossing::flangeway_depth;
using road_crossing::flangeway_width;
using road_crossing::flangeway_gauge_depth;
using road_crossing::rail_gap;
using road_crossing::rail_gaps;
using road_crossing::rail_track;
using road_crossing::track_at;
using road_crossing::head_plane;
using road_crossing::plane_of;
using road_crossing::head_top;
// Dz.U. 2025 poz. 1105, § 34: no level crossing is made within the length of a turnout
// how far past the outer edge of the gaps the surface stays in the plane of the heads: the reach of the sleepers, a 2.6 m sleeper
// ending 1.3 m from the axis of a track of the standard gauge (an editor choice, not a regulation)
double constexpr deck_margin{0.5};
// how far the road is reshaped on each side of the crossing: the 20 m of the approach and 10 m more to lead into the road as it was
double constexpr reach{30.0};
// the stretch of the approach next to the outer rail over which the road turns from the slope of the crossing to its own
double constexpr ease{2.0};

// rails by name, with the width of the head where the track has no profile to measure it on
struct rail_type
{
	char const *name;
	double head; // m, the width of the head
	char const *source;
	bool confirmed;
};
std::vector<rail_type> const &rail_types();
// nullptr for a name not on the list
rail_type const *rail_named(std::string const &Name);

// the plane fit the best to the heads of the rails of several tracks lying too close along the road to have a plane each.
// Mismatch: receives how far off the plane the heads are, at most
head_plane common_plane(std::vector<rail_track> const &Tracks, double &Mismatch);

// a crossing panel lying on the road: its outline in the plan and the height of its top; the outline of points around them all
using road_crossing::panel;
using road_crossing::hull;
// true if the segment from A to B and the polygon have a point in common
bool touches(std::vector<glm::dvec2> const &Polygon, glm::dvec2 const &A, glm::dvec2 const &B);

// a piece of the road, laid in the order of the road and along it
struct road_piece
{
	segment_data axis;
	double halfwidth{3.5}; // from the axis to the outer edge of what's laid along it, the sides included
};
struct design_input
{
	std::vector<road_piece> pieces; // the pieces of the road, joined end to end, in order
	std::vector<rail_track> tracks;
	std::vector<panel> panels;
	bool narrow{false}; // the gauge is narrow, for the limits of the regulation
	bool limits{true}; // the cant a crossing allows enforced; off: the plane of the heads laid whatever the cant, to measure it
};
// a piece of the road as it's laid anew: a part of one of the pieces given, from and to its curve parameter, with its axis
struct designed_piece
{
	std::size_t source{0};
	double from{0.0};
	double to{1.0};
	segment_data axis;
	bool deck{false}; // it lies over a track, in the plane of the heads
};
struct design_result
{
	bool valid{false};
	std::string reason; // why there's nothing to do, or why it can't be done
	// the stretch of the road laid anew, as the distances along the road, and the pieces it's made of
	double from{0.0};
	double to{0.0};
	std::vector<designed_piece> pieces;
	std::vector<std::pair<double, double>> stretches; // each stretch laid anew, apart from the others by more than 60 m between the rails
	std::vector<double> starts; // the distance along the road to the start of each piece given, and the length of the road last
	std::vector<std::size_t> crossed; // the tracks given which the road crosses
	std::vector<rail_track> crossed_tracks; // the tracks given
	int tracks{0};
	double cant{0.0}; // mm, the largest of the tracks crossed
	double cant_low{0.0}; // mm, the smallest
	double steepest{0.0}; // the steepest grade of the approaches within 20 m of the outer rails
	double cross_slope{0.0}; // the steepest slope of the road over the tracks, along the road
	double mismatch{0.0}; // m, how far the heads of tracks lying too close along the road to have a plane each are off their plane
	int panels{0}; // crossing panels lying on the road over the tracks
	double angle{90.0}; // degrees, the smallest the road crosses a track at
	// what's against the regulation (Dz.U. 2025 poz. 1105): an approach steeper than 3% within 20 m of the outer rail (annex 5 pt 1),
	// the road over the crossing steeper than 8% (2% narrow gauge; annex 5 pt 1, § 31 ust. 4), the angle out of 60-120 degrees for
	// a crossing of category D, 45-135 for the others (§ 28 ust. 1)
	bool steep_approach{false};
	bool steep_cross{false};
	// the tracks crossed on a cant ramp (§ 31 ust. 3): the road isn't laid over them
	std::vector<std::size_t> on_ramp;
	// the tracks canted more than a crossing at the angle the road crosses them allows (§ 31 ust. 4): the road isn't laid over them
	struct cant_over
	{
		std::size_t track{0}; // of the tracks given
		double cant{0.0}; // mm, as it is
		double allowed{0.0}; // mm, the most there, whole millimetres
		double angle{90.0}; // degrees
	};
	std::vector<cant_over> over;
	// the breaks of the grade line of the road at the heads of two tracks side by side facing each other (annex 5 pt 3), where it turns
	// from the plane of the heads of the one onto the line straight to the other: the larger of its two changes of grade there
	struct grade_break
	{
		std::size_t first{0}, second{0}; // of the tracks given, in the order of the road
		double value{0.0}; // a fraction: 0.05 is 5%
	};
	std::vector<grade_break> breaks;
	double largest_break{0.0};
};
design_result design(design_input const &Input);
// two tracks side by side facing each other across the road: the road goes straight between their heads, the outer panels between them
// bent onto that line (world/RoadCrossing.h)
using road_crossing::facing_gap;
using road_crossing::facing;
// the slope over the plane of the heads of the track, rise per metre going away from the rail, an outer panel on its Side (1 the right,
// -1 the left) is bent to, to lie on the line the road goes along between it and the track next to it on that side (Road: the direction
// of the road, Halfwidth: its reach either way of its axis); 0 with no track facing it on that side (facing())
double outer_slope(rail_track const &Own, int const Side, std::vector<rail_track> const &Tracks, glm::dvec2 const &Road, double const Halfwidth);
// the break of the grade line of a road going Road over the tracks A and B side by side, straight from the field side of the head of the
// one to the head of the other between them (design()): the larger change of its grade at the two heads, a fraction
double grade_break(rail_track const &A, rail_track const &B, glm::dvec2 const &Road);
// the tracks with their cants scaled by Share, the roll of each as it would be
rail_track with_cant(rail_track Track, double const Share);
// the largest share of their cants the tracks A and B can keep for the break at their heads to be break_limit at most (the cants of
// both lowered alike, from what they are), 0 for none
double break_share(rail_track const &A, rail_track const &B, glm::dvec2 const &Road);
// the change of level of the track Moved (B, or A) which puts the heads of both in one plane, and so the road over them without a break
// (annex 5 pt 1): up above 0. false where their levels alone can't: their cants or grades differ, or they aren't side by side
bool one_plane(rail_track const &A, rail_track const &B, double &Raise, int &Moved);
// the most cant, mm over 1500 mm between the rail axes, a level crossing allows on a track crossed at the angle (degrees), with the
// grade of the track (rise per metre); Narrow: the narrow gauge
double allowed_cant(double const Angle, double const Grade, bool const Narrow);
// the stations along the road where its axis crosses the axes of the tracks; -1 for a track it doesn't cross
std::vector<double> crossing_stations(design_input const &Input);
// the height of the road as designed at a point of the plan, on its surface (the roll counted in); false if the point isn't on it.
// Pieces: as the design gave them, Halfwidth: of the road
bool surface_height(std::vector<designed_piece> const &Pieces, double const Halfwidth, glm::dvec2 const &Point, double &Height);
// the grade of the axis of the road along it, at a distance along the pieces
double axis_grade(std::vector<designed_piece> const &Pieces, double const Station);
double axis_height(std::vector<designed_piece> const &Pieces, double const Station);

} // namespace level_crossing
