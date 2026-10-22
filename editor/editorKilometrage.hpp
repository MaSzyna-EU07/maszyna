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
#include <optional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "scene/scenelayers.h"

class TTrack;

// the kilometrage of the railway lines, set in the track editor and kept in the scenery files as "//$k" comment lines: the
// points of a line where its km is known, from a post or a document, and the points where it jumps, with the km on both
// sides. the km run along the axis of a single-track line and between the tracks of a double-track one, along the longer track
// where the tracks part on formations of their own (PKP GK-1 6.1.12), the other tracks get the km straight across from them
namespace kilometrage
{

struct mark
{
	scene::layer_handle layer{null_handle};
	std::string line; // the number of the line, as 91
	glm::dvec3 point{0.0}; // on the axis of a track of the line
	glm::dvec2 way{0.0, 1.0}; // the km rise this way along the line there, in the plan
	double km{0.0}; // the km the point is reached at, coming along the rising km
	std::optional<double> beyond; // of a jump: the km on from the point
};

void reset();
void load();
std::vector<mark> const &marks();
// a new mark, kept in the file of the layer of the track nearest to it
void put(mark Entry);
void change(std::size_t const Index, mark Entry);
void remove(std::size_t const Index);
// the marks as they were before the changes made since the last call, if there were any; the current ones become the base
std::optional<std::vector<mark>> take_changes();
// the current marks become the base, the changes made so far aren't a part of the undo history
void settle();
// puts the marks back as they were, for undo and redo
void restore(std::vector<mark> Marks);
// the axes found are dropped when the tracks change: Edits counts the changes of the tracks made in the editor
void follow(std::size_t const Edits);
// counts the times the axes were dropped: what is made from them is made again when it changes
std::size_t revision();

// the axis a line is measured along
struct axis
{
	std::string line;
	bool between{false}; // between the tracks of a double-track line, else along the track of the line
	std::vector<glm::dvec3> points; // a couple of metres apart
	std::vector<glm::dvec2> directions; // of the points, in the plan
	std::vector<double> chainage; // of the points, along them
	int growth{1}; // the km rise along the chainage, -1: against it
	struct anchor
	{
		std::size_t mark{0}; // of the marks
		double chainage{0.0};
		double gap{0.0}; // m from the mark to the axis, in the plan
		double before{0.0}; // the km the point is reached at, coming along the rising km
		double after{0.0}; // the km on from it, other than before at a jump
	};
	// the shorter track of a double-track line where the tracks run on formations of their own, the axis along the longer one: its
	// km go by its own length from the end of the stretch the km rise from, the irregular hectometre at the other end takes up the
	// difference (PKP GK-1 6.1.12)
	struct side
	{
		std::vector<glm::dvec3> points;
		std::vector<double> along; // m from the start of the stretch, along the track
		std::vector<double> chainage; // of the axis, by the share of the length: where a mark on the track anchors
		double length{0.0}; // of the track between the ends of the stretch
		double from{0.0}; // the chainage of the ends of the stretch
		double to{0.0};
	};
	std::vector<side> sides;
	std::vector<anchor> anchors; // by the chainage
	std::vector<std::size_t> astray; // the marks of the line away from its axis
	double length() const { return chainage.empty() ? 0.0 : chainage.back(); }
};
// the numbers of the lines with marks
std::vector<std::string> lines();
// the axis of the line through the tracks of its marks, nullptr while it has none; kept until the marks or the tracks change
axis const *axis_of(std::string const &Line);
double km_on(axis const &Axis, double const Chainage);
// where the point lies along the axis, Gap: how far from it in the plan
double chainage_of(axis const &Axis, glm::dvec3 const &Point, double &Gap);
// the same through the tracks running apart from the axis too, where they are nearer: the chainage the point takes the km of
double chainage_at(axis const &Axis, glm::dvec3 const &Point, double &Gap);
glm::dvec3 point_on(axis const &Axis, double const Chainage);
glm::dvec2 direction_on(axis const &Axis, double const Chainage);
// the km of a track running apart from the axis, Along: m from the start of the stretch
double km_on(axis const &Axis, axis::side const &Side, double const Along);
// the km of the line at the point, when it lies within the reach of the axis
bool km_at(std::string const &Line, glm::dvec3 const &Point, double &Km, double const Reach = 30.0);
// the track nearest to the point in the plan within the reach, nullptr for none
TTrack *track_near(glm::dvec3 const &Point, double const Reach);
// "100,234", as PKP PLK write the km
std::string text(double const Km);

} // namespace kilometrage
