/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editortool.h"
#include "application/editormodebuildservices.h"
#include "application/editormoderoadstate.h"

#include <utility>
#include <vector>

// the road window: roads, junctions, level crossings and traffic points selected, built and placed; its state is editor_mode's
// road_tool, reached through the build services
class road_editor final : public editor_tool
{
  public:
	explicit road_editor(editor_build_services &Build) : m_build{Build}, m_roadtool{Build.road_state()} {}
	void update() override;
	void draw_overlay() const override;
	void render_options() override;
	void render_road_selection();
	// the change of the road tool
	void road_choose_tool(int const Tool);
	void road_click();
	void road_select(road_node *Road);
	bool road_delete();
	bool road_split();
	void road_cancel();
	// the selection of the road window checked against what an undo or a redo brought back
	void reselect_after_history();

  private:
	editor_build_services &m_build;
	road_tool &m_roadtool;
	bool render_road_layout(road_node::state &State);
	void render_junction_layout(junction_node &Junction);
	void junction_apply(junction_node &Junction, junction_node::state const &State);
	// leads the banks of the corners of a junction to the ground beside it
	void junction_bank_to_ground(junction_node &Junction);
	// how far from specified point, and how far down, a slope of the grade set for the banks gets to the ground.
	// Outwards: the way the slope leads, seen from above
	std::pair<float, float> bank_reach(glm::dvec3 const &Edge, glm::dvec3 const &Outwards);
	void junction_select(junction_node *Junction);
	void road_apply();
	// sets the width of the road at the selected points, or the height of each of them
	void road_point_width(float const Width);
	void road_point_heights(std::vector<double> const &Heights);
	bool road_points_apply(std::vector<std::pair<road_node *, road_node::state>> const &Changes, std::vector<std::pair<junction_node *, junction_node::state>> const &Junctions);
	// takes the selected points out of their roads
	bool road_points_delete();
	// puts a roundabout around specified point
	void road_roundabout(glm::dvec3 const &Ground);
	// leads the banks of the selected piece, or of the whole road, to the ground beside it
	void road_bank_to_ground();
	// level crossings and traffic points
	void roadpoint_select(roadpoint_node *Point);
	// works out the point a click of the place tool would make. Fresh: even if the cursor is where it was the last time
	void roadpoint_aim(bool const Fresh);
	void roadpoint_place();
	void roadpoint_apply(roadpoint_node &Point, roadpoint_node::state const &State);
	// Point: the point the controls are for, nullptr for the one about to be made
	bool render_roadpoint_layout(roadpoint_node::state &State, roadpoint_node const *Point);
	// Fresh: the ground which wasn't looked up yet is gathered on the spot, instead of a bit of it each frame
	road_plan road_preview(bool const Fresh);
	// Firstgrade, Lastgrade: slopes the road has to start and end with, nullptr for none
	void road_profile(std::vector<segment_data> &Pieces, double const *Firstgrade, double const *Lastgrade, bool const Fresh);
};
