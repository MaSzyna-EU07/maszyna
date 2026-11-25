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
#include <functional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "world/Segment.h"
#include "model/vertex.h"

// the gaps a road leaves along the rails of a level crossing: where its surface lies in the plane of the heads of the rails of a track
// it crosses (within 2 mm of it), the surface is cut along each rail, the head left uncovered for the treads of the wheels and the
// flangeway on its gauge side for their flanges, the way the rails of the tracks are cut at the frogs (world/RailCrossings.cpp). The
// roads of the sceneries made before don't lie at the heads, so they stay as they were. The editor lays the roads into that plane
// (editor/editorCrossing.hpp); the road cuts itself as it builds its geometry (road_node::create_shapes), in the editor and in the
// simulation alike. The values of the regulation come with their sources, what isn't confirmed said so
namespace road_crossing
{

// the flangeway: Dz.U. 2025 poz. 1105 (the regulation on the crossings of the railway lines with the roads, of 9.07.2025, in force
// from 27.08.2025), § 36 ust. 1, gives no figures: the flangeways as the lower part of the structure gauge needs them, as small as
// that lets them be; ust. 2: their sum may grow by the tolerance of the gauge, by 35 mm at most. The figures are PLK's, for the
// crossings of the concrete panels CBP (STWiORB D.11.01.01, the track works of PKP PLK, p. 311): at least 67 mm wide on straights and
// curves of R 350 m and more, 75 mm on R 250-350 m, 80 mm under R 250 m, measured 14 mm under the top of the head (10 mm for the narrow
// gauge), at least 38 mm deep; the inner panels of 1300 mm in the track of 1435 mm leave 67.5 mm (Id-111, ch. 3). The regulation before
// (Dz.U. 2015 poz. 1744, § 36, repealed) had 60, 70 and 80 mm
double constexpr flangeway_depth{0.038};
double flangeway_width(double const Radius);
// how far from the top of the head the flangeway is measured (10 mm for the narrow gauge)
double flangeway_gauge_depth(double const Gauge);
// the road is cut along the rails of a track where its surface lies this close to the plane of their heads, either way: the rounding
// of the numbers in the scenery file
double constexpr cut_tolerance{0.002};
// the flangeway a groove beside the head, not a bare hole: its wall slopes down from the edge of the surface by groove_run across to
// its bottom, groove_depth under the top of the head, which reaches a little under the head (groove_tuck) so nothing shows through.
// 40 mm deep: at least 38 mm, § 36 ust. 1 pkt 3 above, with 2 mm to spare; the wall an editor choice
double constexpr groove_depth{0.040};
double constexpr groove_run{0.030};
double constexpr groove_tuck{0.005};
// the width of the groove at the surface, for the flangeway the regulation measures 14 mm under the top of the head
double groove_top_width(double const Flangeway);

// the gap cut in the road along a rail, from the axis of the track towards its right (the left rail negative): the flangeway on the
// gauge side of the head, then the head
struct rail_gap
{
	int side{1}; // 1 the right rail, -1 the left one
	double flangeway_from{0.0}; // the inner edge of the flangeway
	double head_from{0.0}; // the gauge face of the rail
	double head_to{0.0}; // the field side of the head
	double width() const
	{
		return std::abs(head_to - flangeway_from);
	}
};
// Gauge: between the gauge faces; Head: the width of the head; Flangeway: its width. The left rail first
std::array<rail_gap, 2> rail_gaps(double const Gauge, double const Head, double const Flangeway);

// a track the road crosses, at the place it crosses it, as it lies there
struct rail_track
{
	std::string name;
	void const *source{nullptr}; // the track it was taken from, to tell the tracks apart
	int path{0}; // of the track
	double distance{0.0}; // along the path, of the point
	glm::dvec3 point{0.0}; // of the axis of the path, at the height of the tops of the rails there (the points of a path are)
	glm::dvec2 along{0.0, 1.0}; // the direction of the path in the plan
	double grade{0.0}; // rise per metre along it
	double roll{0.0}; // radians, a roll above 0 lowers the right rail, as the paths have it
	double gauge{1.435}; // between the gauge faces
	double head{0.072}; // the width of the head
	double radius{0.0}; // of the path there, 0: straight
	double curvature{0.0}; // 1/m in the plan, above 0 bending towards its right
	bool turnout{false}; // a path of a switch or a crossing of tracks
	bool ramp{false}; // the cant changes along the path: a cant ramp
	double length{0.0}; // of the path
	// the right of the path in the plan, the way its rail profile named right lies
	glm::dvec2 right() const
	{
		return {-along.y, along.x};
	}
	// the axis of the path in the plan at a distance along it from the point, taken as an arc there; Right: receives the right of it
	glm::dvec2 axis_at(double const Along, glm::dvec2 &Right) const;
	// the cant, mm, over 1500 mm between the rail axes as the editor shows it
	double cant() const;
};
// the place at a distance along the path, as it lies there: the point, the direction, its curvature, the grade and the roll; the rest is
// the caller's
rail_track track_at(TSegment const &Path, double const Distance);

// the plane of the heads of the rails of a track around a point
struct head_plane
{
	glm::dvec3 origin{0.0};
	glm::dvec2 gradient{0.0}; // rise per metre in the plan
	double at(glm::dvec2 const &Point) const
	{
		return origin.y + glm::dot(Point - glm::dvec2{origin.x, origin.z}, gradient);
	}
};
head_plane plane_of(rail_track const &Track);
// the top of the head of a rail of the track, Side 1 the right one, -1 the left one, at a distance along the track from its point and
// Out from the gauge face towards the field side, as the rail profile turned by the roll about the axis lies
glm::dvec3 head_top(rail_track const &Track, int const Side, double const Along, double const Out);

// the height of the surface of a road piece at a point of the plan, the roll counted in. false off the piece (past its ends, or
// further from its axis than Halfwidth)
bool surface_at(segment_data const &Axis, double const Halfwidth, glm::dvec2 const &Point, double &Height);

// two tracks side by side, each in its own plane, whose heads facing each other are less than this apart along the road (or, at a skew,
// along any line of it across its width): between them the road goes straight from the head of the one to the head of the other, with
// no hump and no vertical curve between the outer rails (Dz.U. 2025 poz. 1105, § 30 ust. 3), the outer panels between them bent onto it
double constexpr facing_gap{8.0};
// whether the tracks A and B lie side by side facing each other across a road going Road (its direction in the plan), Halfwidth either
// way of its axis: nearly parallel, apart, the field sides of their heads facing each other closer along the road than facing_gap and the
// skew of the road across its width. Gap: receives how far apart across the tracks the field sides of their heads are
bool facing(rail_track const &A, rail_track const &B, glm::dvec2 const &Road, double const Halfwidth, double *Gap = nullptr);
// the place of a point of the plan by a track: how far along it from its point and how far to its right in the plan (the left negative),
// the track taken as an arc there
void track_frame(rail_track const &Track, glm::dvec2 const &Point, double &Along, double &Across);
// how far the field side of the heads of a track lies from its axis in the plan, the roll counted in
double band(rail_track const &Track);

// the surface of a level crossing as the editor lays the road over it: over each track, to the field side of its heads, the plane of its
// heads (along its arc); between two tracks facing each other (facing()) the surface going straight from the field side of the head of
// the one to the head of the other across them; on the outer sides the plane of the heads of the track there going on. The pieces of a
// road meet each other along lines square to its axis, the heads run along the tracks: at a skew no piece follows this surface across
// all of its width, so a piece the editor laid over it (follows()) takes it for its geometry (warp()) and for its lanes, which are set
// onto it at their ends
struct crossing_surface
{
	std::vector<rail_track> tracks;
	glm::dvec2 road{1.0, 0.0}; // the direction of the road
	double halfwidth{0.0};
	bool empty() const
	{
		return tracks.empty();
	}
	// the height at a point of the plan; false with no tracks
	bool at(glm::dvec2 const &Point, double &Height) const;
	// the rise per metre at a point going in a direction of the plan, from the point a little way that way: the side of a kink the
	// direction goes to
	double slope(glm::dvec2 const &Point, glm::dvec2 const &Direction) const;
};
// the tracks the road crosses (the ones it runs along or which are turnouts left out), the road going Road
crossing_surface surface_of(std::vector<rail_track> const &Tracks, glm::dvec2 const &Road, double const Halfwidth);
// the axis of a road piece on the surface of the crossing all along it, within cut_tolerance: a piece the editor laid over it
bool follows(segment_data const &Axis, crossing_surface const &Surface);
// the height of the surface of a road piece at a point of the plan as surface_at() has it, or of the crossing (Surface, when given) there
bool height_on(segment_data const &Axis, double const Halfwidth, crossing_surface const *Surface, glm::dvec2 const &Point, double &Height);
// the vertices of the geometry of a road piece over a crossing it follows: the ones of its surface (within 5 cm of it) set onto the
// surface of the crossing, the others (kerbs, sides) moved up or down as the surface under them goes; within Halfwidth of the axis
void warp(std::vector<world_vertex> &Vertices, segment_data const &Axis, double const Halfwidth, crossing_surface const &Surface);
// the surface of a road piece over a crossing split where the surface of the crossing bends, along the field side of the heads of each
// track (the chord of its arc from the one end of the piece to the other), and the edges along its ends, where it meets the pieces next
// to it, split at the same points as the piece next to it splits them: every joint_spacing across the road from the axis and where the
// field side of the heads passes. The road lays its surface as one band across it, an end an edge from the one side to the other: over a
// surface which bends at the heads, and between two tracks a little out of parallel (on curves) is twisted, the ends of two pieces went
// straight between different points and parted by up to a few tenths of a millimetre, a slit to see under the road through. Before
// warp(), and both before the cuts along the rails and under the panels, which then make their corners on the surface
double constexpr joint_spacing{0.25};
void conform(std::vector<world_vertex> &Vertices, segment_data const &Axis, double const Halfwidth, crossing_surface const &Surface);
// a path laid along the axis of a road piece which follows the crossing (a lane of it) set onto the surface of the crossing at its ends: the
// heights there, the slopes going into the path from them, the slope across it for the rolls (as the road has them: its left up for a roll
// above 0); the control vectors keep their lengths in the plan, a straight path gets them at a third of its chord
void fit(segment_data &Path, crossing_surface const &Crossing);
// the corners welded to a grid of 0.01 mm: the pieces meeting at a joint work out the corners they share each from its own axis, a
// millionth of a metre apart, which leaves a crack between them for a pixel to sparkle through; rounded they're the same
void weld(std::vector<world_vertex> &Vertices);
// the value of the curve parameter where the axis of the road crosses the axis of the track, taken straight there; -1 for none
double crossing_parameter(segment_data const &Axis, rail_track const &Track);
// a crossing panel lying on a road: its outline in the plan and the height of its top
struct panel
{
	std::vector<glm::dvec2> outline;
	double top{0.0};
	// the top of the panel as it lies, its triangles facing up: its footprint exactly, the cut corners and a bend along a curve too
	std::vector<std::array<glm::dvec3, 3>> faces;
};
// what a road piece crossing the rails with its surface at their heads gets along each rail: the holes left out of its flat triangles
// over the head and the top of the groove beside it, four corners each in the plan, long enough to reach over the whole road; and
// the triangles of the groove, its wall and its bottom, as far as the piece reaches over the rail
struct road_cut
{
	std::vector<std::vector<glm::dvec2>> holes;
	std::vector<world_vertex> grooves;
	std::vector<world_vertex> fills; // beside the rails past the road, under the panels: see cut()
	std::vector<std::size_t> tracks; // the indices of the tracks the road is cut along
};
// Texturelength: of the surface, for the texture of the grooves laid flat over the plan. Panels: the crossing panels lying around:
// along a rail they reach past the road the flangeway goes on as the groove, with the same width and depth, and the strips beside the
// rail the panels leave open are filled at the plane of the heads (fills, to be cut under the panels as the surface is), so the
// crossing is closed over the whole length of the panels. The groove is closed at its ends
// Surface: the crossing the piece follows (follows()), its heights taken for the surface of the piece
road_cut cut(segment_data const &Axis, double const Halfwidth, std::vector<rail_track> const &Tracks, float const Texturelength = 4.f, std::vector<panel> const &Panels = {},
             crossing_surface const *Surface = nullptr);
// the holes alone. Cut: receives the indices of the tracks the road is cut along
std::vector<std::vector<glm::dvec2>> gaps(segment_data const &Axis, double const Halfwidth, std::vector<rail_track> const &Tracks, std::vector<std::size_t> *Cut = nullptr);
// the outline of points in the plan around them all
std::vector<glm::dvec2> hull(std::vector<glm::dvec2> Points);
// the footprints of the crossing panels a road piece lies under: where the top of a panel overlaps the surface of the piece, however
// little of it, lying at the surface there or up to 0.3 m over it (a slab, not something standing on the road), the triangles of its top
// in the plan (its outline for one without them). The road leaves its surface out under them, the panel is the surface there: no surface
// lowered under it, so its lanes, which the vehicles go along, stay at the top of the panel. Surface: as for cut()
std::vector<std::vector<glm::dvec2>> covers(segment_data const &Axis, double const Halfwidth, std::vector<panel> const &Panels, crossing_surface const *Surface = nullptr);

// the skirts of a surface cut under the panels: along each edge it got where a panel lies, a strip hanging Depth down from the edge,
// slanting back under the road by 0.4 of it. Where the top of the panel comes a hair under the edge of the road (the plane of the road
// and the top of a panel bent along a curve part by a fraction of a millimetre) nothing shows under the road, and a ray slipping down
// the hair of a crack between the edge of the road and the edge of the panel, along it, meets the side of the panel or the skirt
std::vector<world_vertex> skirts(std::vector<world_vertex> const &Surface, std::vector<std::vector<glm::dvec2>> const &Covers, double const Depth = 0.05);

// the heads of the rails of a track as triangles, their tops in the plane of the heads and their sides 50 mm down, from and to a distance
// along the track from its point: what a ray meets there
std::vector<std::array<glm::dvec3, 3>> rail_heads(rail_track const &Track, double const From, double const To);

// a crossing as built, to be checked for what shows through it: the surface of the road with its sides and its grooves, the panels,
// the heads of the rails, as triangles in the world; the pieces of the road with how far from their axes they reach, and the tracks
struct built_crossing
{
	std::vector<std::array<glm::dvec3, 3>> road;
	std::vector<std::array<glm::dvec3, 3>> panels;
	std::vector<std::array<glm::dvec3, 3>> rails;
	std::vector<std::pair<segment_data, double>> footprint;
	std::vector<rail_track> tracks;
};
struct defect
{
	glm::dvec3 point{0.0};
	// hole: a ray straight down meets nothing; seam: the same beside an edge of the surface of the road, within a few millimetres; slit:
	// a ray at a slant gets under the surface; over: the road lying flat over the top of a panel within its footprint, or within 1 mm
	// under it;
	// flangeway: down the middle of a flangeway something else than the bottom of the groove at its depth
	std::string kind;
};
// a ray slipping through a crack narrower than this isn't a defect: the corners of the road are welded to a grid of 0.01 mm, and a
// corner made by a cut on an edge of a triangle the triangle beside it doesn't have (a T) leaves a line a few micrometres wide between
// them. What shows through no more than this a pixel never sees
double constexpr crack_width{0.00002};
// the defects found: rays straight down on a grid of Step over the footprint of the road (the plan between Low and High), beside every
// edge of the surface of the road every centimetre, at slants of 10 and 30 degrees along the road and along the tracks over the
// panels and the rails, and down the middle of each flangeway every centimetre along the road and the panels; a ray meeting nothing
// where a ray crack_width aside meets something goes through a crack. Most: how many to report. Cracks: receives how many rays went
// through a crack
std::vector<defect> check(built_crossing const &Built, glm::dvec2 const &Low, glm::dvec2 const &High, double const Step, std::size_t const Most = 200, std::size_t *Cracks = nullptr);

// leaves the gaps out of the triangles lying flat enough to be a part of a surface: each gap a convex outline in the plan (x, z). A
// triangle is cut along the lines of the outline, the parts outside the gap kept, with the vertices made where it's cut laid on its
// edges, so they take the height, the normal and the texture coordinates the triangle has there. Bridges: receives the parts left
// out, as they lay: the surface over the gaps, for what asks for the height of the road where a wheel or a foot goes, which the gaps
// and the grooves in them aren't a part of (they're only seen)
void cut_gaps(std::vector<world_vertex> &Vertices, std::vector<std::vector<glm::dvec2>> const &Gaps, std::vector<world_vertex> *Bridges = nullptr);

// what building a road piece over a crossing does to its geometry, in the order road_node::create_shapes does it, to its batches of
// triangles (Batches): the holes along the rails cut out of them (the parts cut out kept, as they lay, in bridges); the geometry set onto
// the surface of the crossing where the piece follows it (Surface); the panels lying on it cut out of it (covers), the skirts hung
// along their edges; the grooves and the strips beside the rails past the road, these cut under the panels, given for the batch of the
// surface. Reach: how far from its axis the piece reaches, its sides too
struct crossing_build
{
	std::vector<std::vector<glm::dvec2>> holes;
	std::vector<std::vector<glm::dvec2>> covers;
	std::vector<world_vertex> bridges;
	std::vector<world_vertex> grooves;
	std::vector<world_vertex> fills;
	std::vector<std::size_t> tracks;
};
crossing_build build(std::vector<std::vector<world_vertex> *> const &Batches, segment_data const &Axis, double const Reach, std::vector<rail_track> const &Rails, float const Texturelength,
                     std::vector<panel> const &Panels, crossing_surface const *Surface);

} // namespace road_crossing
