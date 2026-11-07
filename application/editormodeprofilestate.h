/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/editorProfile.hpp"
#include "editor/editorTrack.hpp"
#include "scene/scenenode.h"
#include "imgui/imgui.h"

#include <cstdint>
#include <limits>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;

// the route the vertical profile is along, and its samples
struct profile_route_state
{
	bool open{false};
	TTrack *from{nullptr};
	TTrack *to{nullptr};
	TTrack *picked_from{nullptr};
	TTrack *picked_to{nullptr};
	double run_length{std::numeric_limits<double>::infinity()}; // the whole line through the track clicked
	std::vector<TTrack *> followed; // the selection the route was last taken from
	editor_track::route route;
	std::vector<editor_track::route_sample> samples;
	std::vector<TTrack *> created; // the paths the last apply split off the route
	int edits{-1}; // of the tracks, when the route was last read
};

// the ground under the route and the earthworks led to the grade line
struct profile_ground_state
{
	std::vector<double> terrain; // NaN where there's no terrain
	std::vector<double> model; // the ground of the terrain model (NMT) as measured; NaN where it's not known
	bool model_waiting{false}; // squares of the model under the route still to come
	double model_polled{0.0};
	// depth of the formation under the grade line, the rail top, of the track of each sample: the formation, not the rails,
	// lies on the ground
	std::vector<double> beds;
	profile::ground_fit fit;
	// the heightmap terrain led to the grade line by a modifier: formation under the track, slopes down or up to the ground
	struct earthworks
	{
		bool from_ballast{true};
		double depth{0.7}; // m, formation below the top of the rail
		double half_width{3.5}; // m, from the axis of the route to the edge of the formation
		double slope{1.5}; // horizontal run of the slopes per metre of height
		double reach{60.0}; // m, from the edge of the formation, where the slopes end even if they don't get to the ground
		double rounding{1.5}; // m, over which the slopes bend into the formation and into the ground
		bool generate{true};
		// chunks of terrain made under the route by the last shaping
		std::vector<std::pair<int, int>> generated;
		// the modifier of the terrain made by the last shaping
		std::string modifier;
	} earthworks;
};

// the grade line designed along the route, and what the checks of it found
struct profile_line_state
{
	// what the joined ends take their height and grade from, when it's a switch: shown on the plot
	std::string start_inherits;
	std::string end_inherits;
	profile::line line;
	profile::context context;
	std::vector<profile::issue> issues;
	double origin{0.0}; // chainage of the start of the route
	double departure{0.0}; // largest difference between the grade line and the track
	profile::line line_read; // the line before it was edited
	bool fitting{false}; // the line comes from the fit to the terrain model: any error of it refuses it
	bool whole_grade_line{true}; // applied, the tracks sharing the grade line of the route go with it
};

// the part of the route the plot shows, and what it shows
struct profile_view_state
{
	double view_from{0.0};
	double view_to{100.0};
	double view_centre{0.0};
	float exaggeration{20.0f};
	bool show_terrain{true};
	bool show_model{true};
	bool show_track{true};
	bool show_plan{true};
	bool show_remarks{false};
	float strip_top{-1.0f}; // the top of the strip over the 3d view, -1 while it isn't shown
};

// the tracks sharing the grade line of the route
struct profile_grade_line_state
{
	std::vector<TTrack *> beside; // those of them lying by it, for the route and the state of the tracks below
	std::tuple<TTrack const *, std::size_t, std::size_t, std::size_t> beside_of{nullptr, 0, 0, 0};
	bool picking{false}; // a click on a track in the view puts it on the grade line of the route or takes it off
	std::vector<TTrack const *> members; // of the grade line picked, wherever they are
	std::tuple<int, std::size_t, std::size_t> members_of{-1, 0, 0};
};

// the points of the grade line picked, dragged and hovered on the plot
struct profile_edit_state
{
	int selected{-1};
	int dragging{-1};
	// where the point was grabbed: the mouse there, the point against the mouse; the point moves once the mouse does
	ImVec2 drag_press{0.0f, 0.0f};
	glm::dvec2 drag_grab{0.0};
	bool drag_moved{false};
	int curve_grip{-1};
	bool panning{false};
	bool changed{false};
	double hover{-1.0};
	// under the cursor in the strip
	int hover_point{-1};
	int hover_grip{-1};
	bool hover_line{false};
	int shown{-2}; // point in the side panel
};

// vertical profile (grade line) along a route
struct profile_state : profile_route_state, profile_ground_state, profile_line_state, profile_view_state, profile_grade_line_state, profile_edit_state
{
	std::string status;
	std::string error;
};

// grade lines designed in the editor, kept in the scenery files as "//$p" comment lines
struct stored_profile
{
	scene::layer_handle layer{null_handle};
	double length{0.0};
	glm::dvec3 start{0.0}, end{0.0}, middle{0.0}; // of the route, world space
	double origin{0.0};
	profile::line line;
	double departure{-1.0}; // of the track from the design when it was kept; -1 as kept by an older editor
};

// the tracks whose grade line was fitted, by name, with the shape they had then, kept as "//$t" comment lines: a track
// changed since, by hand or taken back, isn't fitted any more
struct fitted_mark
{
	scene::layer_handle layer{null_handle};
	std::uint64_t shape{0};
	bool hand{false};
};
