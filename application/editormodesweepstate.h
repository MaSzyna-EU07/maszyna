/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scene/scenenode.h"
#include "world/Segment.h"
#include "world/Sweep.h"

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class TAnimModel;

// the model laid along the track: its file, its skin and the parameters of a template
struct sweep_model_state
{
	std::array<char, 256> model{};
	std::array<char, 128> skin{"none"};
	std::array<char, 64> search{};
	std::vector<std::string> models;
	bool scanned{false};
	int revision{-1}; // of the include bank the templates were taken from
	// parameters of a template laid along the track, with their names
	std::string parameters_for;
	std::vector<std::array<char, 128>> parameters;
	std::vector<std::string> parameter_labels;
	std::vector<bool> parameter_placement; // filled in with zeros: the template is placed in its own space
	std::vector<std::vector<std::string>> parameter_choices; // variants of the textures a parameter is a part of the name of
};

// how the model is laid along the track
struct sweep_layout_state
{
	int length_mode{2}; // 0: one copy as long as the original, 1: the length given, 2: the whole stretch of the line
	double length{200.0};
	sweep_node::state settings;
	int axis{0}; // 0: the longer one of the model, 1: z, 2: x
	double model_length{0.0};
	std::string measured; // model the length is of
};

// the curve: the axis of the track
struct sweep_curve_state
{
	TTrack *curve_for{nullptr};
	std::size_t curve_history{0};
	struct piece
	{
		TTrack *track{nullptr};
		int path{0};
		bool forward{true};
	};
	std::vector<piece> curve_pieces;
	std::vector<segment_data> curve;
	double curve_length{0.0};
	std::vector<glm::dvec3> outline; // of the curve, for the preview
	std::vector<double> stations;
};

// a model laid along the track: bent to follow it, or repeated along it
struct sweep_tool : sweep_model_state, sweep_layout_state, sweep_curve_state
{
	bool open{false};
	bool expand{false};
	// marking the stretch in the view: press where it starts, the distance from the track sets the offset, drag to the end
	bool marking{false};
	bool dragging{false};
	double anchor{0.0}; // station the drag started at
	sweep_node *edited{nullptr};
	std::string status;
};

// paths the models along curves were laid on, so they follow the changes of the paths
struct sweep_link
{
	sweep_node *sweep{nullptr};
	std::size_t piece{0};
	TTrack *track{nullptr};
	int path{0};
	bool reversed{false};
};

struct bend_source
{
	TAnimModel *model{nullptr};
	scene::instance_handle instance{0};
	std::string file;
	std::string skin{"none"};
	std::vector<std::string> parameters;
	glm::dmat4 transform{1.0};
	glm::dvec3 scale{1.0};
	scene::layer_handle layer{null_handle};
};

// where the bent model lies by the track: its axis and its reference point
struct bend_reference_state
{
	int axis{0};
	bool turned{false};
	int along_anchor{1};
	int side_anchor{0};
	int height_anchor{0};
	glm::dvec3 point{0.0};
	bool picking{false};
};

// the tracks found by the model to be bent
struct bend_tracks_state
{
	void const *tracks_for{nullptr};
	glm::dvec3 tracks_at{0.0};
	std::vector<std::pair<TTrack *, double>> tracks;
	int track{0};
};

// the marker of the bent model dragged in the view
struct bend_drag_state
{
	int dragging{0};
	sweep_node::state drag_state;
	glm::dvec3 drag_anchor{0.0};
	double drag_station{0.0};
	std::string drag_label;
};

// a model bent along the track: the one edited, its marker and what the cursor shows
struct bend_tool : bend_reference_state, bend_tracks_state, bend_drag_state
{
	sweep_node *edited{nullptr};
	std::string signature;
	bool marker_valid{false};
	glm::dvec3 marker{0.0};
	std::string status;
	sweep_node *hover{nullptr};
	bool over_marker{false};
	std::string label;
	std::string details;
};
