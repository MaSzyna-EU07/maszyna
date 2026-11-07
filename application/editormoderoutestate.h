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
#include "imgui/imgui.h"

#include <functional>
#include <map>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class TTrack;

// the line designed between two paths: its chain, its design and what came out of it, and the vertex worked on
struct route_line_state
{
	TTrack *from{nullptr};
	TTrack *to{nullptr};
	editor_track::chain chain;
	std::string error;
	std::string status;
	alignment::design design;
	alignment::result result;
	int vertex{-1};
	int grip{-1};
	bool single{true}; // one curve between its straights, instead of the whole line
};

// the crossovers taken out with the line, and where its design stands against the alignment kept in the scenery
struct route_relay_state
{
	bool relaying{false}; // the crossovers of the line were taken out, they go back once it's laid
	// crossovers taken out with a line which didn't fit it laid again, tried again as the other lines are laid
	struct crossover_out
	{
		int shape{-1};
		glm::dvec3 press{0.0};
		glm::dvec3 release{0.0};
		std::string name;
		std::vector<std::string> names; // of the switches at the press and at the release
	};
	std::vector<crossover_out> crossovers_out;
	double origin{0.0}; // km of the start, m
	int designed{0}; // vertices whose parameters came from the alignment kept in the scenery
	int departed{-1}; // alignment kept in the scenery near the fragment, which its paths no longer follow
	double departure{0.0};
};

// the points clicked when the fragment was laid, and how the line was at the revision they were measured
struct route_clicks_state
{
	// the points clicked when the fragment was laid, kept with its design: the curves keep to them as their transitions and cants
	// change, while hold is on
	std::vector<glm::dvec3> clicks;
	bool hold{true};
	struct click_state
	{
		double offset{0.0}; // from the line, m
		int vertex{-1}; // whose curve it's a point of
		bool corner{false}; // a bend of straights clicked, the vertex itself rather than a point of its curve
	};
	std::vector<click_state> click_states; // as the line was at the revision measured
	int revision{0}; // of the result, counted as it's computed
	int measured{-1};
};

// the line designed between two paths, with the crossovers taken out with it, the points it was laid through and the panel of its curves
struct route_design : route_line_state, route_relay_state, route_clicks_state
{
	// the band of the curvature in the panel: the element under the mouse, the point of it, and what's dragged in it
	struct curvature_band
	{
		int hot{-1};
		double at{-1.0};
		int vertex{-1};
		int part{0}; // 1 the radius, 2 the transition in, 3 the transition out, 4 the transition between two arcs of a compound curve
		int arc{0}; // the arc of a compound curve the radius is of, 0 the first; of a transition between arcs the one after it
		double value{0.0};
		ImVec2 press{0.0f, 0.0f};
		float metres{1.0f}; // pixels of a metre and of a unit of the curvature, kept as the drag goes
		float curvature{1.0f};
	};
	curvature_band band;
	// the part of a curve whose field the mouse is over in the panel, or which is being edited there, shown in the view
	struct curve_focus
	{
		enum class part
		{
			whole,
			entry,
			exit,
			radius_arc, // of the radius `arc`, the first one 0
			between // the transition leading into the arc of the radius `arc`
		};
		int vertex{-1};
		part what{part::whole};
		int arc{0};
		bool held{false}; // by a field being edited, the mouse over the others doesn't move it
	};
	curve_focus focus;
	// points clicked along the middle of the track seen on the orthophoto, the radii of the curves and their shares fitted to them after each click
	struct trace_state
	{
		bool active{false};
		int radii{0}; // 0: as many as the points need, up to three
		std::vector<glm::dvec3> points;
		alignment::design before; // taken back by Esc
		double rms{0.0};
		double worst{0.0};
	};
	trace_state trace;
};

// the band of the curvature in the panel as it's drawn: where it lies, its plot with the axis of no curvature, how many pixels a
// metre and a unit of the curvature take, and the element under the mouse with what of it a drag changes
struct route_band_view
{
	// what of its vertex an element is: part 1 an arc, the arc of a compound curve counted from 0, 2 the transition in, 3 the
	// transition out, 4 the transition into the arc
	struct role
	{
		int part{0};
		int arc{0};
	};
	float font{0.0f};
	float pad{0.0f};
	float line{0.0f}; // of the text
	ImVec2 top{0.0f, 0.0f};
	ImVec2 bottom{0.0f, 0.0f};
	float inner_left{0.0f};
	float inner_width{0.0f};
	float plot_top{0.0f};
	float plot_bottom{0.0f};
	float axis{0.0f};
	bool both{false}; // curves to both sides
	float flip{1.0f};
	float metres{1.0f};
	float curvature{0.0f};
	bool dragging{false};
	ImVec2 mouse{0.0f, 0.0f};
	int under{-1};
	int part{0};
	int arc{0};
	float x_of(double const Chainage) const { return inner_left + static_cast<float>(Chainage) * metres; }
	float y_of(double const K) const { return axis - flip * static_cast<float>(K) * curvature; }
	float label_y() const { return bottom.y - pad * 0.25f - line; }
};

// what the digitization of the scenery did, or would do
struct digitize_report
{
	int lines{0};
	int lines_failed{0};
	int switches{0};
	int known{0}; // kept in the scenery already
	int designed{0}; // of a template of the catalogue
	int measured{0};
	int slips{0};
	int unnamed{0}; // no record finds them
	int odd{0};
	int recorded{0};
	int crossovers{0};
	int crossovers_known{0};
	int crossovers_recorded{0};
	int crossovers_odd{0};
	int found{0};
	int bound{0};
	std::map<std::string, int, std::less<>> types; // of the catalogue
	std::map<std::string, int, std::less<>> shapes; // measured
	std::vector<std::string> notes;
};
