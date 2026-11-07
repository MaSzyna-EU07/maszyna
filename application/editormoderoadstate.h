/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/editorRoad.hpp"
#include "world/Road.h"
#include "world/RoadPoint.h"
#include "world/Segment.h"

#include <array>
#include <string>
#include <vector>

#include <glm/glm.hpp>

// roads: a menu and a window of their own, independent from the edit modes of the toolset
// what a click of the build tool would make
struct road_plan
{
	std::vector<segment_data> pieces; // axes of the pieces, in the order they're laid
	editor_road::loose_end snap; // loose end of a road or arm of a junction the road is led to
	editor_road::branch start; // junction made, or given another arm, for a road led out of the side of a road or out of a junction
	bool hasstart{false};
	editor_road::branch end; // the same for a road led into the side of a road or into a junction
	bool hasend{false};
	std::string error; // why there's nothing to make
};

// what the road window has selected, and the layout it shows
struct road_selection_state
{
	road_node *selected{nullptr};
	junction_node *junction{nullptr}; // selected junction
	roadpoint_node *marker{nullptr}; // selected level crossing or traffic point
	std::vector<glm::dvec3> points; // selected points where the pieces meet or end
	road_node::state settings; // layout shown in the window: of the selected piece, or of the pieces about to be built
	bool whole{true}; // layout changes go to every piece of the road the selected one belongs to
	bool varied{false}; // the selected piece is shown with lanes of its own for its end
};

// where the build tool carries on from
struct road_chain_state
{
	bool chain{false}; // the point to build from is set
	glm::dvec3 point{0.0}; // where the next piece starts
	glm::dvec2 direction{0.0, 1.0}; // the way it has to leave that point
	bool hasdirection{false};
	bool reversed{false}; // the pieces are laid from their ends, to carry on a road from its start
	double grade{0.0}; // slope the next piece has to start with, to carry on what it's attached to
	bool hasgrade{false};
	// the road being built leaves the side of a road, or a junction between its roads. the point to build from is the centre
	// of the junction, which gets made, or gets another arm, along with the first piece
	road_node *branchroad{nullptr};
	double branchat{0.0}; // value of the curve parameter of the axis of that road
	junction_node *branchjunction{nullptr};
};

// the settings of the build tool
struct road_build_settings
{
	float offset{0.1f}; // height above the ground the roads are built at
	bool follow{true}; // built roads are led over the ground between the clicked points
	float crossingspeed{30.f}; // speed limit on the junctions being made
	float heightstep{0.1f}; // how much the selected points are raised or lowered at a time
	bool ring{false}; // the build tool makes roundabouts
	float ringradius{15.f}; // of the middle of the road going around
	std::string island{"none"}; // what the middle of a roundabout is covered with
	float bankgrade{1.5f}; // metres a bank led to the ground runs for each metre it drops
};

// level crossings and traffic points: what the place tool puts down
struct road_place_state
{
	int placekind{0}; // what the place tool puts down: 0: level crossing, 1: spawn point, 2: removal point, 3: pedestrian crossing
	std::array<roadpoint_node::state, 4> placing; // what the next point of each kind is going to be like
	float stopmargin{5.f}; // distance from the outermost track of a new crossing to the places the vehicles stop at
	roadpoint_node::state target; // the point a click of the place tool would make at the moment
	bool hastarget{false};
	glm::dvec3 targetdirection{0.0}; // the way the traffic goes there
	std::string targetnote; // what's worth knowing about that point, or why there's none
	glm::dvec3 aimed{0.0}; // where the cursor was when the target was worked out, and for what kind of point
	int aimedkind{-1};
	std::array<char, 4096> vehicles{}; // vehicles to add to the set of a spawn point, written the way they're put in a scenery
};

// what the road tools show over the view at the moment
struct road_shown_state
{
	road_plan plan; // what a click would make at the moment, worked out once a frame for display
	glm::dvec3 hover{0.0}; // point of a road or a junction under the cursor a road can be led out of
	bool hashover{false};
	glm::dvec3 mouse{0.0};
	std::string status;
};

// the state of the road window and its tools, its parts grouped by what they're about
struct road_tool : road_selection_state, road_chain_state, road_build_settings, road_place_state, road_shown_state
{
	bool window{false}; // the road window is open, the left mouse button belongs to its tools
	bool lanes{true}; // lanes of the roads are drawn as trajectories
	int tool{0}; // 0: select, 1: build, 2: place
};
