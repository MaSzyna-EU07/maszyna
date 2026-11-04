/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/


#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include "application/editormodetrackshared.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>

using namespace track_mode;

void editor_mode::render_track_gizmo()
{
	auto *track = selected_track();
	std::string reason;
	if (track == nullptr || false == editor_track::can_edit_geometry(*track, reason))
	{
		m_track_gizmo_using = false;
		return;
	}
	bool const pointmode = ui()->mode() == nodebank_panel::TRACK && m_track_point.valid() && m_track_point.path < static_cast<int>(track->m_paths.size());

	gizmo_frame const frame;

	if (false == m_track_gizmo_using)
	{
		auto const anchor = pointmode ? editor_track::point_position(*track, m_track_point) : editor_track::pivot(*track);
		m_track_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(anchor - frame.camera));
	}

	glm::mat4 delta(1.0f);
	frame.manipulate(ImGuizmo::TRANSLATE, m_track_gizmo, m_gizmo_snap, &delta);

	if (ImGuizmo::IsUsing())
	{
		if (false == m_track_gizmo_using)
		{
			m_track_gizmo_using = true;
			m_track_pivot = editor_track::pivot(*track);
			m_track_drag = {track};
			m_track_drag_points.clear();
			m_track_joints.clear();
			m_track_snap = {};
			if (false == pointmode && m_track_drag_connected)
			{
				m_track_joints = editor_track::joints(*track);
				for (auto const &joint : m_track_joints)
					if (std::find(m_track_drag.begin(), m_track_drag.end(), joint.other) == m_track_drag.end())
						m_track_drag.emplace_back(joint.other);
			}
			if (pointmode && editor_track::is_end(m_track_point.kind) && m_track_drag_connected)
			{
				for (auto const &connection : editor_track::connected_points(*track, editor_track::point_position(*track, m_track_point)))
				{
					std::string neighbourreason;
					if (false == editor_track::can_edit_geometry(*connection.first, neighbourreason))
						continue;
					m_track_drag_points.emplace_back(connection);
					if (std::find(m_track_drag.begin(), m_track_drag.end(), connection.first) == m_track_drag.end())
						m_track_drag.emplace_back(connection.first);
				}
			}
			std::vector<std::pair<TTrack *, editor_track::state>> states;
			for (auto *dragged : m_track_drag)
				states.emplace_back(dragged, editor_track::capture(*dragged));
			push_track_snapshot(std::move(states));
		}

		if (pointmode)
		{
			glm::dvec3 const position = frame.camera + glm::dvec3(m_track_gizmo[3]);
			editor_track::move_point(*track, m_track_point, position);
			for (auto const &connection : m_track_drag_points)
				editor_track::move_point(*connection.first, connection.second, position);
			m_track_dirty = true;
			if (editor_track::is_end(m_track_point.kind) && m_track_drag_points.empty())
			{
				std::vector<TTrack const *> const exclude(m_track_drag.begin(), m_track_drag.end());
				screen_projection const projection;
				ImVec2 screen;
				m_track_snap = projection.project(position, screen) ? snap_free_end(position, {screen.x, screen.y}, track, track->iCategoryFlag & 15, exclude) : editor_track::snap_target{};
			}
		}
		else
		{
			glm::dvec3 const offset{delta[3]};
			if (glm::length(offset) > 1e-9)
			{
				editor_track::translate(*track, offset);
				editor_track::follow(*track, m_track_joints);
				m_track_dirty = true;
			}
		}
		commit_track_drag(false);
	}
	else if (m_track_gizmo_using)
	{
		if (m_track_snap.track != nullptr && pointmode)
		{
			editor_track::snap_point(*track, m_track_point, m_track_snap, m_track_align_tangent);
			m_track_dirty = true;
		}
		m_track_snap = {};
		commit_track_drag(true);
		m_track_gizmo_using = false;
		m_track_drag.clear();
		m_track_drag_points.clear();
		m_track_joints.clear();
	}
}
