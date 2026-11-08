/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/editorTrack.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;

// the straights found along the paths and listed, the one shown with its neighbours
struct straights_found_state
{
	editor_track::straight_tolerance tolerance;
	double minimum_length{50.0};
	std::vector<editor_track::straight> found;
	int listed{-1};
	TTrack *current_for{nullptr};
	editor_track::straight current;
	std::vector<editor_track::straight> neighbours;
	std::vector<double> spacings{3.5, 4.0, 4.5, 4.75, 5.0, 5.5, 6.0};
	std::array<char, 32> filter{};
};

// a straight dragged in the view, the curves fitted to it and the paths it changes
struct straights_drag_state
{
	int handle{-1};
	bool dragging{false};
	editor_track::straight drag_line;
	std::vector<editor_track::straight> set;
	std::vector<editor_track::straight> drag_lines;
	bool fitting{false};
	editor_track::curve fit_before;
	editor_track::curve fit_after;
	bool fit_has_before{false};
	bool fit_has_after{false};
	std::vector<std::pair<TTrack *, editor_track::state>> drag_states;
	std::size_t drag_step{0};
};

// the straight edited in the panel: its ends, its length and its direction, and the curves between it and the next
struct straights_edit_state
{
	glm::dvec3 preview_start{0.0};
	glm::dvec3 preview_end{0.0};
	glm::dvec3 edit_start{0.0};
	glm::dvec3 edit_end{0.0};
	double edit_length{0.0};
	double edit_azimuth{0.0};
	double distance{4.0};
	double curve_radius{0.0};
	bool auto_transitions{true};
	double transition{0.0};
};

// the tools of the straights placed in the view: a detour, a straight put by the line
struct straights_place_state
{
	int tool{0};
	int tool_before{0};
	bool tool_mouse{false};
	std::vector<glm::dvec3> detour;
	editor_track::straight detour_line;
	glm::dvec3 detour_mouse{0.0};
	bool tool_placed{false};
	double tool_at{0.0};
	glm::dvec3 tool_point{0.0};
	glm::dvec3 tool_handle{0.0};
	editor_track::straight tool_line;
};

// the straights of the paths: found, listed, dragged, edited and placed
struct straights_state : straights_found_state, straights_drag_state, straights_edit_state, straights_place_state
{
	std::string status;
};

// lengthens a path from its free end, or carries on from it
struct extend_tool
{
	bool active{false};
	TTrack *track{nullptr};
	glm::dvec3 point{0.0};
	glm::dvec2 direction{0.0, 1.0};
	double grade{0.0};
	glm::dvec3 mouse{0.0};
	int path{0};
	bool atend{true};
	editor_track::snap_target origin;
	editor_track::snap_target snap;
	glm::vec2 pressed{0.f};
	std::vector<TTrack *> line;
};

// listed spacing of a neighbouring straight the line can snap to, if it is close enough
struct parallel_offer
{
	bool nearby{false};
	bool snaps{false};
	double spacing{0.0};
	double distance{0.0};
	double angle{0.0}; // deg between the two straights
	double correction{0.0}; // m along the line normal to reach the spacing
	editor_track::straight const *neighbour{nullptr};
};
