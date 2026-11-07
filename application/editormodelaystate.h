/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/editorAlignment.hpp"
#include "editor/editorTrack.hpp"
#include "world/Segment.h"

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

// what a point clicked while laying is: a bend of the straights, eased with a large curve, or a point along the middle of a curve
// seen on the photo, the curve ending at the last of these
enum class lay_mode
{
	straights,
	curve
};
// a curve traced on the photo as it's laid, how far the points clicked along it are from it
struct lay_trace
{
	std::vector<glm::dvec3> points;
	std::vector<double> offsets;
	double rms{0.0};
	double worst{0.0};
	std::size_t curves{0};
};
// the line the points clicked make: its design, the bends among its vertices, the curves traced
struct lay_line_plan
{
	alignment::design design;
	alignment::result shape;
	std::vector<char> corners;
	std::vector<lay_trace> traces;
	// the straights of a fixed direction the next bend slides along: from a free end, out of a curve traced
	std::vector<std::pair<glm::dvec3, glm::dvec3>> guides;
	// where the grade breaks from point to point: the bends kept as such, at the heights clicked; a curve traced, or bends made
	// one curve, goes at one grade from end to end
	std::vector<glm::dvec3> knots;
};

// the points clicked to lay the track through, how each was clicked, and where the cursor is
struct lay_points_state
{
	bool active{false};
	std::vector<glm::dvec3> points;
	std::vector<lay_mode> modes; // of the points, as each was clicked
	lay_mode mode{lay_mode::straights};
	int radii{0}; // of a curve traced: 0 as many as the points need, up to three
	bool straights_between{true}; // a curve traced may be two with a straight between them
	lay_line_plan plan; // of the line shown
	editor_track::snap_target start; // free end the track starts from, if any
	glm::dvec3 mouse{0.0};
	editor_track::snap_target mouse_snap;
};

// the line shown before it's laid: the one of the cursor, the one worked out whole last and the cursor it waits for
struct lay_preview_state
{
	std::vector<segment_data> preview;
	std::string preview_error;
	double preview_length{0.0};
	// the line worked out whole last, all its fits done: shown while the one of the cursor now still waits for them on the thread,
	// so that the curves don't jump from one fit to the next as each comes
	struct worked_out
	{
		std::vector<segment_data> preview;
		std::string error;
		double length{0.0};
		lay_line_plan plan;
		std::optional<alignment::lay_result> eased;
		alignment::join_result join;
		std::vector<glm::dvec3> points; // the ones clicked it was worked out for
	};
	std::optional<worked_out> shown;
	// the cursor the line is being worked out for, kept till all its fits are done: what it was worked out from
	struct settling_cursor
	{
		glm::dvec3 mouse{0.0};
		editor_track::snap_target snap;
		std::vector<glm::dvec3> points;
		std::vector<lay_mode> modes;
		lay_mode mode{lay_mode::straights};
		int edits{0};
	};
	std::optional<settling_cursor> settling;
	bool whole{true}; // false: each fit shown as it comes, as before, for a selftest to compare
};

// lays new track through the clicked points, with no path needed to start from
struct lay_tool : lay_points_state, lay_preview_state
{
	double radius{500.0};
	double piece_length{100.0};
	bool copy_selected{false};
	editor_track::path_style style;
	bool own_speed{false}; // the speed of the style also where the track goes on from one with its own, the new tracks given it
	std::string status;
	double status_time{-100.0};
	std::string skew; // what Alt turned the straight to
	double escape_time{-100.0}; // when Esc was pressed over points not laid: the second Esc drops them
	std::size_t history{0}; // the step of the history the track laid last was begun at
};
