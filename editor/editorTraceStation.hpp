/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <vector>
#include <glm/glm.hpp>

struct segment_data;

// Stations and heads of turnouts traced on the aerial photo (quirky-crafting-summit, Etap 5). A line traced from a click on a track
// starts with a turnout into it, one ending with a click on a track ends with one: of the catalogue, its shape (radius and angle), its
// start along the track, the way it runs and its side fitted to the points clicked along its diverging track. A run of the points
// keeping one spacing from a track beside it within the noise of the clicks is laid exactly alongside it, at the spacing measured,
// rounded to the usual ones. A turnout put nearer to the next one on its track than the norms let it is moved to where they do.
//
// The domain alone: no scene, no tracks; the plan in metres.
namespace trace_station
{

// a straight track: a point of it and its direction, a unit vector; to the left is where the cross product of the direction with the
// offset is positive, the side a switch template diverges to with its side 1
struct straight
{
	glm::dvec2 origin{0.0};
	glm::dvec2 direction{1.0, 0.0};
	double along(glm::dvec2 const &Point) const;
	double lateral(glm::dvec2 const &Point) const;
	glm::dvec2 point(double Along, double Lateral = 0.0) const;
};

// a turnout of the catalogue as the fit sees it: its diverging track in its own frame, x along the main track from the start of the
// switch, y to the side it diverges to, sampled from (0, 0) to its end, x growing; its length along the main track; the curvature its
// diverging track ends with, 1/m, 0 where it ends with a straight through the frog
struct turnout_shape
{
	std::vector<glm::dvec2> diverging;
	double length{0.0};
	double end_curvature{0.0};
};

// the shape of a switch template: its diverging track as the template has it in its own space (z along the main track, x to the side it
// diverges to, as switch_design builds it), sampled every Step metres or so, its pieces each on its own
turnout_shape shape_of(segment_data const &Diverging, double Length, double EndCurvature, double Step = 0.5);

struct turnout_fit
{
	bool valid{false};
	int shape{-1};
	double start{0.0}; // along the main track, where the switch starts (its blade tips)
	int direction{1}; // the way it runs from there along the main track: with its direction (+1) or against it
	int side{1}; // its diverging track to the left (+1) of the way it runs, or the right
	std::size_t used{0}; // the points, from the first, along the turnout: the line goes on from its end with the rest
	std::size_t told{0}; // the points off the main track within the turnout and the straight just beyond it, which told the shape
	double rms{0.0}; // m, of those off its diverging track
	bool preferred{false}; // the shape taken as no points told one
	glm::dvec2 end{0.0}; // of its diverging track
	glm::dvec2 heading{1.0, 0.0}; // there, away from the switch
	double curvature{0.0}; // 1/m there, along the heading, turning left +
};

// The turnout of the shapes fitting the points clicked: the first one on the main track where the turnout starts, the next ones along its
// diverging track and on, each with the noise of its click (m). Its start is fitted along the main track within the length of the
// turnout either way of the first point, the way it runs and its side are the ones the points go. The shapes are told apart by the points
// along the diverging track and the straight just beyond it; where none is there, or they fit two shapes alike, Preferred is taken, at the
// first point. Not valid with no point after the first one
turnout_fit fit_turnout(straight const &Main, std::vector<turnout_shape> const &Shapes, std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, int Preferred);

// the end of the diverging track of the turnout as fitted at Start, the way and the side kept: its position, its heading away from the
// switch and its curvature; the points used counted again
turnout_fit turnout_at(straight const &Main, std::vector<turnout_shape> const &Shapes, turnout_fit const &Fit, double Start, std::vector<glm::dvec2> const &Points);

// the spacing of a track alongside another one, from the one measured: the usual ones, 4.0, 4.5, 4.75 and 5.0 m, within 0.1 m of it, else
// rounded to 0.05 m; the sign, the side, kept
double round_spacing(double Measured);

// a point as a track seen as a line of points in the plan (its axis, in order) has it: abreast of a segment of it, how far along the
// line, how far to the left of it, the foot of the square on it and the direction of the line there
struct abreast
{
	bool found{false};
	double along{0.0};
	double lateral{0.0};
	glm::dvec2 foot{0.0};
	glm::dvec2 direction{1.0, 0.0};
};
abreast abreast_of(std::vector<glm::dvec2> const &Axis, glm::dvec2 const &Point);

// m, from the point to the nearest point of the line of points
double off_line(std::vector<glm::dvec2> const &Line, glm::dvec2 const &Point);

// the longest run of the points, one after another, keeping one spacing from the neighbour (its axis) within the noise of the clicks:
// three points at least, Least m long at least, abreast of it, 1 m off it at least. The spacing measured is their mean offset, to the left +
struct parallel_run
{
	bool found{false};
	std::size_t first{0};
	std::size_t last{0}; // the last point in it
	double measured{0.0}; // m
	double spacing{0.0}; // as laid, round_spacing of it
};
parallel_run find_parallel(std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, std::vector<glm::dvec2> const &Neighbour, double Least = 40.0);

// the point moved square to the neighbour's axis to lie Spacing to the left of it (to the right below 0); as it was where it isn't abreast
glm::dvec2 alongside(std::vector<glm::dvec2> const &Neighbour, glm::dvec2 const &Point, double Spacing);

// The curve joining the straight leaving From at Heading (the diverging track of a turnout beyond its end) onto a straight alongside a
// track, Alongside, running the way the line goes on: one arc tangent to both, the bend of the two straights at the vertex where they
// meet. Its radius fitted to the points clicked along it (in order from From), as large as there's room for where none are, Preferred
// where that fits as well. Not valid where the straights don't meet ahead of From or barely bend
struct joining_arc
{
	bool valid{false};
	glm::dvec2 vertex{0.0};
	double radius{0.0};
	double rms{0.0}; // m, of the points off it
};
joining_arc join_alongside(glm::dvec2 const &From, glm::dvec2 const &Heading, straight const &Alongside, std::vector<glm::dvec2> const &Points, std::vector<double> const &Sigmas, double Preferred);

// A turnout of the length starting at From (the lower end of it along the track) on a straight of plain tracks from Low to High: with a
// turnout at either end of it (Before at Low, After at High) the straight between the two is Least at least (the norms, switch_straight).
// Where the photo put it, if that keeps them; else moved to the nearest place which does, told by how much. Doesn't fit where the straight
// is too short for it
struct head_place
{
	bool fits{false};
	double from{0.0};
	double moved{0.0}; // m, from where it was put, signed along the track
};
head_place place_in_head(double From, double Length, double Low, double High, bool Before, bool After, double Least);

} // namespace trace_station
