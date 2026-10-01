/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// path (track) editing part of the editor mode

#include "stdafx.h"
#include "application/editormode.h"
#include "application/editoruilayer.h"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{

// projects world points onto the screen, with the same camera-relative view and clean perspective the gizmo uses
class screen_projection
{
  public:
	screen_projection()
	{
		ImGuiIO const &io = ImGui::GetIO();
		m_size = io.DisplaySize;
		m_camera = GfxRenderer->Camera_Position();
		float const fovy = glm::radians(Global.FieldOfView / Global.ZoomFactor);
		float const aspect = m_size.y > 0.0f ? m_size.x / m_size.y : 1.0f;
		m_viewprojection = glm::perspective(fovy, aspect, 0.1f, 10000.0f) * GfxRenderer->Camera_View_Matrix();
	}

	glm::vec4 clip(glm::dvec3 const &Point) const
	{
		return m_viewprojection * glm::vec4(glm::vec3(Point - m_camera), 1.0f);
	}
	ImVec2 screen(glm::vec4 const &Clip) const
	{
		return ImVec2((Clip.x / Clip.w * 0.5f + 0.5f) * m_size.x, (0.5f - Clip.y / Clip.w * 0.5f) * m_size.y);
	}
	// returns: true if the point is in front of the camera
	bool project(glm::dvec3 const &Point, ImVec2 &Screen) const
	{
		auto const c = clip(Point);
		if (c.w < kNear)
			return false;
		Screen = screen(c);
		return true;
	}
	// draws a line, clipped against the near plane so points behind the camera don't flip across the screen
	void line(ImDrawList *Drawlist, glm::dvec3 const &A, glm::dvec3 const &B, ImU32 const Color, float const Thickness) const
	{
		glm::vec4 a = clip(A);
		glm::vec4 b = clip(B);
		if (a.w < kNear && b.w < kNear)
			return;
		if (a.w < kNear)
			a = glm::mix(a, b, (kNear - a.w) / (b.w - a.w));
		else if (b.w < kNear)
			b = glm::mix(b, a, (kNear - b.w) / (a.w - b.w));
		Drawlist->AddLine(screen(a), screen(b), Color, Thickness);
	}

  private:
	static constexpr float kNear{0.1f};
	ImVec2 m_size;
	glm::dvec3 m_camera;
	glm::mat4 m_viewprojection;
};

// point on the course of a path, matching the curve TSegment builds from the same data
glm::dvec3 path_point(segment_data const &Path, double const T)
{
	auto const &p0 = Path.points[segment_data::point::start];
	auto const &p3 = Path.points[segment_data::point::end];
	auto const &cp1 = Path.points[segment_data::point::control1];
	auto const &cp2 = Path.points[segment_data::point::control2];
	if (cp1 == glm::dvec3{} && cp2 == glm::dvec3{})
		return glm::mix(p0, p3, T);
	auto const p1 = p0 + cp1;
	auto const p2 = p3 + cp2;
	double const u = 1.0 - T;
	return u * u * u * p0 + 3.0 * u * u * T * p1 + 3.0 * u * T * T * p2 + T * T * T * p3;
}

char const *point_label(editor_track::point_kind const Kind)
{
	switch (Kind)
	{
	case editor_track::point_kind::start: return "Start";
	case editor_track::point_kind::control1: return "Control 1";
	case editor_track::point_kind::control2: return "Control 2";
	default: return "End";
	}
}

bool is_end(editor_track::point_kind const Kind)
{
	return Kind == editor_track::point_kind::start || Kind == editor_track::point_kind::end;
}

editor_track::point_kind const kPointKinds[] = {editor_track::point_kind::start, editor_track::point_kind::control1, editor_track::point_kind::control2, editor_track::point_kind::end};

// screen distance within which a click grabs a point handle
float const kHandleRadius{10.0f};

} // namespace

TTrack *editor_mode::selected_track() const
{
	return dynamic_cast<TTrack *>(m_node);
}

void editor_mode::select_track(scene::basic_node *Node)
{
	auto *track = dynamic_cast<TTrack *>(Node);
	if (track == nullptr)
	{
		// a click beside the selected path releases its point handle
		m_track_point = {};
		return;
	}
	// paths are long, so the distance is measured to the nearest point of the path rather than its centre
	glm::dvec3 const camera{Global.pCamera.Pos};
	if (glm::distance(glm::dvec3(track->get_nearest_point(camera)), camera) > static_cast<double>(kMaxPlacementDistance))
		return;

	if (track != m_node)
		m_track_point = {};
	m_node = track;
	ui()->set_node(m_node);
}

bool editor_mode::pick_track_handle()
{
	auto *track = selected_track();
	if (track == nullptr || false == editor_track::is_supported(*track))
		return false;

	screen_projection const projection;
	ImVec2 const mouse = ImGui::GetIO().MousePos;
	float best = kHandleRadius * kHandleRadius;
	editor_track::point_ref hit;
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		for (auto const kind : kPointKinds)
		{
			editor_track::point_ref const point{i, kind};
			ImVec2 screen;
			if (false == projection.project(editor_track::point_position(*track, point), screen))
				continue;
			float const dx = screen.x - mouse.x;
			float const dy = screen.y - mouse.y;
			// end points take precedence over control points placed at the same spot
			float const distance = (dx * dx + dy * dy) * (is_end(kind) ? 0.5f : 1.0f);
			if (distance < best)
			{
				best = distance;
				hit = point;
			}
		}
	}
	if (false == hit.valid())
		return false;
	m_track_point = hit;
	return true;
}

void editor_mode::draw_track_overlay() const
{
	auto const *track = selected_track();
	if (track == nullptr)
		return;

	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList();
	bool const trackmode = ui()->mode() == nodebank_panel::TRACK;

	// course of the path, from its source data so it follows the edits before the path is rebuilt
	ImU32 const coursecolor = IM_COL32(40, 220, 255, 220);
	int constexpr samples{32};
	for (auto const &path : track->m_paths)
	{
		auto previous = path_point(path, 0.0);
		for (int i = 1; i <= samples; ++i)
		{
			auto const next = path_point(path, static_cast<double>(i) / samples);
			projection.line(drawlist, previous, next, coursecolor, 2.5f);
			previous = next;
		}
	}
	if (false == trackmode || false == editor_track::is_supported(*track))
		return;

	// control vectors
	ImU32 const vectorcolor = IM_COL32(200, 200, 200, 160);
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		projection.line(drawlist, editor_track::point_position(*track, {i, editor_track::point_kind::start}), editor_track::point_position(*track, {i, editor_track::point_kind::control1}), vectorcolor, 1.0f);
		projection.line(drawlist, editor_track::point_position(*track, {i, editor_track::point_kind::end}), editor_track::point_position(*track, {i, editor_track::point_kind::control2}), vectorcolor, 1.0f);
	}
	// handles: end points coloured by connection state (green: connected, red: free), control points as squares
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		for (auto const kind : kPointKinds)
		{
			editor_track::point_ref const point{i, kind};
			ImVec2 screen;
			if (false == projection.project(editor_track::point_position(*track, point), screen))
				continue;
			bool const selected = (point == m_track_point);
			if (is_end(kind))
			{
				ImU32 const color = editor_track::is_connected(*track, point) ? IM_COL32(60, 220, 60, 255) : IM_COL32(240, 60, 60, 255);
				drawlist->AddCircleFilled(screen, 6.0f, color);
			}
			else
			{
				drawlist->AddRectFilled(ImVec2(screen.x - 4.0f, screen.y - 4.0f), ImVec2(screen.x + 4.0f, screen.y + 4.0f), IM_COL32(230, 230, 230, 255));
			}
			if (selected)
				drawlist->AddCircle(screen, 10.0f, IM_COL32(255, 220, 40, 255), 16, 2.5f);
		}
	}
	// connection to be made when the ongoing drag ends
	if (m_track_snap.track != nullptr)
	{
		ImVec2 screen;
		if (projection.project(m_track_snap.position, screen))
			drawlist->AddCircle(screen, 14.0f, IM_COL32(255, 60, 255, 255), 20, 3.0f);
		projection.line(drawlist, m_track_snap.position, m_track_snap.position + m_track_snap.direction * 5.0, IM_COL32(255, 60, 255, 200), 2.0f);
	}
}

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

	ImGuizmo::BeginFrame();
	ImGuizmo::SetOrthographic(false);
	ImGuiIO const &io = ImGui::GetIO();
	ImGuizmo::SetRect(0.0f, 0.0f, io.DisplaySize.x, io.DisplaySize.y);

	// same camera-relative view and clean perspective as the transform gizmo of the other nodes
	glm::mat4 const view = GfxRenderer->Camera_View_Matrix();
	glm::dvec3 const camerapos = GfxRenderer->Camera_Position();
	float const fovy = glm::radians(Global.FieldOfView / Global.ZoomFactor);
	float const aspect = io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f;
	glm::mat4 const projection = glm::perspective(fovy, aspect, 0.1f, 10000.0f);

	// point handles only move; the whole path moves or turns around the vertical axis
	ImGuizmo::OPERATION const operation = (false == pointmode && m_gizmo_op == gizmo_operation::rotate) ? ImGuizmo::ROTATE_Y : ImGuizmo::TRANSLATE;

	if (false == m_track_gizmo_using)
	{
		auto const anchor = pointmode ? editor_track::point_position(*track, m_track_point) : editor_track::pivot(*track);
		m_track_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(anchor - camerapos));
	}

	glm::vec3 snapvalue(operation == ImGuizmo::ROTATE_Y ? 5.0f : m_gizmo_snap);
	float const *snap = Global.ctrlState && snapvalue.x > 0.0f ? glm::value_ptr(snapvalue) : nullptr;
	glm::mat4 delta(1.0f);
	ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(projection), operation, ImGuizmo::WORLD, glm::value_ptr(m_track_gizmo), glm::value_ptr(delta), snap);

	if (ImGuizmo::IsUsing())
	{
		if (false == m_track_gizmo_using)
		{
			// drag start: gather the paths it modifies and record their state for undo
			m_track_gizmo_using = true;
			m_track_pivot = editor_track::pivot(*track);
			m_track_drag = {track};
			m_track_drag_points.clear();
			m_track_snap = {};
			if (pointmode && is_end(m_track_point.kind) && m_track_drag_connected)
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

		if (operation == ImGuizmo::ROTATE_Y)
		{
			double const angle = std::atan2(delta[2][0], delta[0][0]);
			if (std::abs(angle) > 1e-9)
			{
				editor_track::rotate_y(*track, m_track_pivot, angle);
				m_track_dirty = true;
			}
		}
		else if (pointmode)
		{
			glm::dvec3 const position = camerapos + glm::dvec3(m_track_gizmo[3]);
			editor_track::move_point(*track, m_track_point, position);
			for (auto const &connection : m_track_drag_points)
				editor_track::move_point(*connection.first, connection.second, position);
			m_track_dirty = true;
			// a free end point can be connected with another path; a joint moved as a whole stays as it is
			if (is_end(m_track_point.kind) && m_track_drag_points.empty())
			{
				std::vector<TTrack const *> const exclude(m_track_drag.begin(), m_track_drag.end());
				m_track_snap = editor_track::find_snap_target(*track, position, m_track_snap_radius, exclude);
			}
		}
		else
		{
			glm::dvec3 const offset{delta[3]};
			if (glm::length(offset) > 1e-9)
			{
				editor_track::translate(*track, offset);
				m_track_dirty = true;
			}
		}
		commit_track_drag(false);
	}
	else if (m_track_gizmo_using)
	{
		// drag end: make the offered connection and rebuild everything for good
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
	}
}

void editor_mode::commit_track_drag(bool const Force)
{
	if (false == m_track_dirty || m_track_drag.empty())
		return;
	// each rebuild re-uploads geometry of the affected sections, so it isn't done on every frame of a drag
	auto const now = std::chrono::steady_clock::now();
	if (false == Force && now - m_track_last_commit < std::chrono::milliseconds(100))
		return;
	editor_track::commit(m_track_drag);
	m_track_dirty = false;
	m_track_last_commit = now;
}

void editor_mode::push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States)
{
	if (States.empty())
		return;

	if (m_max_history_size >= 0 && (int)m_history.size() >= m_max_history_size)
		m_history.erase(m_history.begin(), m_history.begin() + ((int)m_history.size() - m_max_history_size + 1));

	auto *track = States.front().first;
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::TrackEdit;
	snap.node_name = track->name();
	snap.node_ptr = track;
	snap.position = track->location();
	snap.uuid = track->uuid;
	snap.tracks = std::move(States);
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::restore_track_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite)
{
	if (Snapshot.tracks.empty())
		return;

	EditorSnapshot current{Snapshot};
	std::vector<TTrack *> tracks;
	for (auto &entry : current.tracks)
	{
		tracks.emplace_back(entry.first);
		entry.second = editor_track::capture(*entry.first);
	}
	Opposite.push_back(std::move(current));

	for (auto const &entry : Snapshot.tracks)
		editor_track::apply(*entry.first, entry.second);
	editor_track::commit(tracks);

	m_track_snap = {};
	m_track_dirty = false;
	if (m_node != tracks.front())
		m_track_point = {};
	m_node = tracks.front();
	ui()->set_node(m_node);
}

void editor_mode::render_track_ui()
{
	ImGui::TextDisabled("LMB: select path or point handle   Esc: release point\nGizmo moves the selected point, otherwise the whole path (W: turn)");
	auto *track = selected_track();
	if (track == nullptr)
	{
		ImGui::TextDisabled("No path selected");
		return;
	}

	char const *type = track->eType == tt_Normal ? "normal" : track->eType == tt_Switch ? "switch" : track->eType == tt_Cross ? "cross" : track->eType == tt_Table ? "turntable" : track->eType == tt_Tributary ? "tributary" : "unknown";
	char const *category = (track->iCategoryFlag & 1) ? "rail" : (track->iCategoryFlag & 2) ? "road" : (track->iCategoryFlag & 4) ? "river" : "other";
	ImGui::Text("%s  (%s %s, %.2f m)", track->name().empty() ? "(noname)" : track->name().c_str(), category, type, track->Length());

	if (false == editor_track::is_supported(*track))
	{
		ImGui::TextDisabled("Editing of this path type isn't supported");
		return;
	}
	std::string reason;
	bool const geometry = editor_track::can_edit_geometry(*track, reason);
	if (false == geometry)
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), "Geometry locked: %s", reason.c_str());

	auto const before = editor_track::capture(*track);
	// records undo state when a field edit starts, and rebuilds the path once it's finished
	enum class rebuild { none, parameters, geometry };
	auto const finish_edit = [&](rebuild const Rebuild) {
		if (ImGui::IsItemActivated())
			m_track_field_before = before;
		if (ImGui::IsItemDeactivatedAfterEdit())
		{
			push_track_snapshot({{track, m_track_field_before}});
			switch (Rebuild)
			{
			case rebuild::geometry: editor_track::commit({track}); break;
			case rebuild::parameters: editor_track::commit_parameters(*track); break;
			default: track->mark_dirty(); break;
			}
		}
	};
	// applies a change made with a widget which reports it at once, like a combo box
	auto const apply_now = [&](rebuild const Rebuild) {
		push_track_snapshot({{track, before}});
		if (Rebuild == rebuild::parameters)
			editor_track::commit_parameters(*track);
		else
			track->mark_dirty();
	};

	if (ImGui::CollapsingHeader("Path parameters", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::PushItemWidth(120.0f);
		double velocity = editor_track::velocity(*track);
		if (ImGui::InputDouble("Velocity (km/h, -1: none)", &velocity, 0.0, 0.0, "%.1f"))
			editor_track::velocity(*track, velocity);
		finish_edit(rebuild::none);
		if (ImGui::InputFloat("Friction", &track->fFriction, 0.0f, 0.0f, "%.3f"))
			track->fFriction = std::max(0.0f, track->fFriction);
		finish_edit(rebuild::none);
		ImGui::InputFloat("Sound distance", &track->fSoundDistance, 0.0f, 0.0f, "%.1f");
		finish_edit(rebuild::none);
		ImGui::InputInt("Quality", &track->iQualityFlag, 0);
		finish_edit(rebuild::none);
		int damage = track->iDamageFlag;
		if (ImGui::InputInt("Damage flags", &damage, 0))
			editor_track::damage(*track, damage);
		finish_edit(rebuild::none);

		char const *environments[] = {"flat", "mountains", "canyon", "tunnel", "bridge", "bank"};
		int environment = std::clamp(static_cast<int>(track->eEnvironment), 0, 5);
		if (ImGui::Combo("Environment", &environment, environments, IM_ARRAYSIZE(environments)))
		{
			track->eEnvironment = static_cast<TEnvironmentType>(environment);
			apply_now(rebuild::none);
		}

		if (ImGui::InputFloat("Width", &track->fTrackWidth, 0.0f, 0.0f, "%.3f"))
			track->fTrackWidth = std::max(0.01f, track->fTrackWidth);
		finish_edit(rebuild::parameters);
		ImGui::PopItemWidth();

		if (track->m_visible)
		{
			// material names are edited in local buffers, refreshed from the path whenever no field is being edited
			material_handle *materials[] = {&track->m_material1, &track->m_material2, track->SwitchExtension ? &track->SwitchExtension->m_material3 : nullptr};
			char const *labels[] = {"Texture 1 (rails/surface)", "Texture 2 (trackbed/side)", "Trackbed (switch)"};
			if (false == ImGui::IsAnyItemActive())
			{
				for (int i = 0; i < 3; ++i)
				{
					auto const name = materials[i] ? editor_track::material_name(*materials[i]) : std::string{};
					std::strncpy(m_track_materials[i].data(), name.c_str(), m_track_materials[i].size() - 1);
					m_track_materials[i].back() = '\0';
				}
			}
			for (int i = 0; i < 3; ++i)
			{
				if (materials[i] == nullptr || (i == 2 && track->eType != tt_Switch))
					continue;
				ImGui::PushItemWidth(-160.0f);
				if (ImGui::InputText(labels[i], m_track_materials[i].data(), m_track_materials[i].size(), ImGuiInputTextFlags_EnterReturnsTrue))
				{
					*materials[i] = editor_track::fetch_material(m_track_materials[i].data());
					apply_now(rebuild::parameters);
				}
				ImGui::PopItemWidth();
			}
			ImGui::TextDisabled("Enter applies a texture name, \"none\" removes the texture");

			ImGui::PushItemWidth(120.0f);
			if (ImGui::InputFloat("Texture length", &track->fTexLength, 0.0f, 0.0f, "%.2f"))
				track->fTexLength = std::max(0.01f, track->fTexLength);
			finish_edit(rebuild::parameters);
			float height = editor_track::texture_height(*track);
			if (ImGui::InputFloat("Trackbed height", &height, 0.0f, 0.0f, "%.3f"))
				editor_track::texture_height(*track, height);
			finish_edit(rebuild::parameters);
			ImGui::InputFloat("Trackbed width", &track->fTexWidth, 0.0f, 0.0f, "%.3f");
			finish_edit(rebuild::parameters);
			ImGui::InputFloat("Trackbed slope", &track->fTexSlope, 0.0f, 0.0f, "%.3f");
			finish_edit(rebuild::parameters);
			ImGui::PopItemWidth();
		}
	}

	if (ImGui::CollapsingHeader("Path geometry", ImGuiTreeNodeFlags_DefaultOpen))
	{
		ImGui::PushItemWidth(120.0f);
		ImGui::InputFloat("Snap radius (m)", &m_track_snap_radius, 0.0f, 0.0f, "%.2f");
		m_track_snap_radius = std::max(0.0f, m_track_snap_radius);
		ImGui::PopItemWidth();
		ImGui::Checkbox("Align with snapped path", &m_track_align_tangent);
		ImGui::SameLine();
		ImGui::Checkbox("Drag connected ends", &m_track_drag_connected);

		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			ImGui::PushID(i);
			if (track->m_paths.size() > 1)
				ImGui::Text("Path %d", i + 1);
			// point selection, same as clicking the handle in the viewport
			for (auto const kind : kPointKinds)
			{
				editor_track::point_ref const point{i, kind};
				if (kind != editor_track::point_kind::start)
					ImGui::SameLine();
				std::string label = point_label(kind);
				if (is_end(kind))
					label += editor_track::is_connected(*track, point) ? " (+)" : " (-)";
				if (ImGui::Selectable(label.c_str(), m_track_point == point, 0, ImVec2(90.0f, 0.0f)))
					m_track_point = (m_track_point == point ? editor_track::point_ref{} : point);
			}

			auto &path = track->m_paths[i];
			if (geometry)
			{
				ImGui::PushItemWidth(-160.0f);
				for (auto const kind : {editor_track::point_kind::start, editor_track::point_kind::end})
				{
					auto position = path.points[kind == editor_track::point_kind::start ? segment_data::point::start : segment_data::point::end];
					if (ImGui::InputScalarN(point_label(kind), ImGuiDataType_Double, glm::value_ptr(position), 3, nullptr, nullptr, "%.3f"))
						editor_track::move_point(*track, {i, kind}, position); // keeps the common start of a switch together
					finish_edit(rebuild::geometry);
				}
				ImGui::InputScalarN("Control 1 (relative)", ImGuiDataType_Double, glm::value_ptr(path.points[segment_data::point::control1]), 3, nullptr, nullptr, "%.3f");
				finish_edit(rebuild::geometry);
				ImGui::InputScalarN("Control 2 (relative)", ImGuiDataType_Double, glm::value_ptr(path.points[segment_data::point::control2]), 3, nullptr, nullptr, "%.3f");
				finish_edit(rebuild::geometry);
				ImGui::InputFloat2("Roll (deg)", path.rolls.data(), "%.2f");
				finish_edit(rebuild::geometry);
				ImGui::InputFloat("Radius", &path.radius, 0.0f, 0.0f, "%.1f");
				finish_edit(rebuild::geometry);
				ImGui::PopItemWidth();
				if (ImGui::Button("Straighten"))
				{
					push_track_snapshot({{track, before}});
					path.points[segment_data::point::control1] = glm::dvec3{};
					path.points[segment_data::point::control2] = glm::dvec3{};
					path.radius = 0.0f;
					editor_track::commit({track});
				}
			}
			else
			{
				auto const &start = path.points[segment_data::point::start];
				auto const &end = path.points[segment_data::point::end];
				ImGui::TextDisabled("Start (%.2f, %.2f, %.2f)  End (%.2f, %.2f, %.2f)", start.x, start.y, start.z, end.x, end.y, end.z);
			}
			ImGui::PopID();
		}
	}
}
