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
#include "application/editormodebuildshared.h"
#include "imgui/imgui.h"

#include <cstddef>
#include <functional>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class lay_planner;
class nmt_auto_fit;
class straights_editor;

// a free end lengthened: the drag from it, the pieces to the cursor or to another free end, a branch or a crossover laid from it
class extend_editor final : public editor_tool
{
  public:
	extend_editor(editor_services &Services, editor_build_services &Build, lay_planner &Planner, nmt_auto_fit &Nmt_auto, straights_editor &Straight_edit) : m_services{Services}, m_build{Build}, m_planner{Planner}, m_nmt_auto{Nmt_auto}, m_straight_edit{Straight_edit}, m_nmt_state{Build.terrain_state()}, m_lay{Build.lay_state()}, m_extend{Build.extend_state()}, m_straights{Build.straight_lines()} {}
	void update_extend_drag(glm::vec2 const &Cursor, bool const Pointing);
	bool start_extend(editor_track::snap_target const &End);
	bool extend_snap(editor_track::snap_target &Target) const;
	std::vector<segment_data> extend_pieces() const;
	void finish_extend();
	// adds a straight (Side 0) or a curve to the left (1) or the right (-1) at the free end of the selected track
	void extend_add(int const Side);
	// the buttons adding a straight or a curve at the free end, and their length and radius
	void render_extend_ui();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	lay_planner &m_planner;
	nmt_auto_fit &m_nmt_auto;
	straights_editor &m_straight_edit;
	nmt_state &m_nmt_state;
	lay_tool &m_lay;
	extend_tool &m_extend;
	straights_state &m_straights;
	// the length of the straight and the radius of the curve added at a free end by the buttons of the panel
	double m_extend_length{25.0};
	double m_extend_radius{300.0};
	// the track dragged on from the diverging end of a switch, in the radius of its branch; nullopt for any other
	std::optional<std::vector<segment_data>> extend_branch_pieces() const;
	// the parts of it: a press let go where it was made
	void extend_tap();
	// the free end new pieces are added at: the selected point if it's one, else the end, else the start
	bool chosen_free_end(extend_tool &Tool) const;
};
