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
class route_designer;
class route_relay_tool;

// the straights of the paths: found and listed, dragged and fitted with curves, edited in the panel, and the tools placing a detour or a straight by the line
class straights_editor final : public editor_tool
{
  public:
	straights_editor(editor_services &Services, editor_build_services &Build, build_shared &Shared, route_designer &Designer, route_relay_tool &Route_relay) : m_services{Services}, m_build{Build}, m_designer{Designer}, m_route_relay{Route_relay}, m_route{Build.route_state()}, m_straights{Build.straight_lines()}, m_infra_carried{Shared.infra_carried} {}
	void draw_straights_overlay() const;
	void find_neighbour_straights();
	parallel_offer straight_parallel_offer(editor_track::straight const &Line, glm::dvec3 const &Offset) const;
	void render_straights_ui();
	void render_straight_ui();
	bool straights_active();
	int straight_handle_hit();
	void straight_refresh();
	void render_straight_gizmo();
	double straight_tool_radius(editor_track::straight const &Line) const;
	bool place_straight_tool();
	void start_straight_gesture(int const Tool);
	void finish_straight_gesture();
	void toggle_straight_set(TTrack &Track);
	std::vector<glm::dvec3> detour_outline() const;
	void add_detour_point();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	route_designer &m_designer;
	route_relay_tool &m_route_relay;
	route_design &m_route;
	straights_state &m_straights;
	std::set<std::pair<scene::basic_node const *, scene::instance_handle>> &m_infra_carried;
	glm::dvec3 snap_straight_offset(editor_track::straight const &Line, glm::dvec3 const &Offset) const;
	glm::dvec3 snap_straight_direction(glm::dvec3 const &Pivot, glm::dvec3 const &Moved) const;
	// the exact values of the straight edited: its ends, its length and its azimuth
	void render_straight_exact(editor_track::straight const &Line);
	bool in_straight_set(editor_track::straight const &Line) const;
	void straight_apply(editor_track::straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End);
	void straights_apply(std::vector<editor_track::straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends);
	// the drag of the gizmo of the straight: begun on the line, the straights moved to where the gizmo is, ended
	bool start_straight_drag(editor_track::straight const &Line);
	void drag_straight(glm::dvec3 const &Handled);
	void end_straight_drag();
	void straight_reshape(editor_track::straight const &Line, double const From, double const To, std::function<glm::dvec3(glm::dvec3 const &)> const &Tail, double const Radius);
	void apply_straight_tool();
	void apply_detour();
	bool start_curve_fit(editor_track::straight const &Line, int const Handle);
	void update_curve_fit(glm::dvec3 const &Start, glm::dvec3 const &End);
};
