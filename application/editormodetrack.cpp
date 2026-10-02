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

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "utilities/Logs.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>

namespace
{

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
	bool project(glm::dvec3 const &Point, ImVec2 &Screen) const
	{
		auto const c = clip(Point);
		if (c.w < kNear)
			return false;
		Screen = screen(c);
		return true;
	}
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

float const kHandleRadius{10.0f};

}

TTrack *editor_mode::selected_track() const
{
	return dynamic_cast<TTrack *>(m_node);
}

void editor_mode::select_track(scene::basic_node *Node)
{
	auto *track = dynamic_cast<TTrack *>(Node);
	if (track == nullptr)
	{
		m_track_point = {};
		return;
	}
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
	if (track == nullptr || false == editor_track::is_supported(*track) || m_route_tab)
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
	if (false == trackmode || m_route_tab || false == editor_track::is_supported(*track))
		return;

	ImU32 const vectorcolor = IM_COL32(200, 200, 200, 160);
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		projection.line(drawlist, editor_track::point_position(*track, {i, editor_track::point_kind::start}), editor_track::point_position(*track, {i, editor_track::point_kind::control1}), vectorcolor, 1.0f);
		projection.line(drawlist, editor_track::point_position(*track, {i, editor_track::point_kind::end}), editor_track::point_position(*track, {i, editor_track::point_kind::control2}), vectorcolor, 1.0f);
	}
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

	glm::mat4 const view = GfxRenderer->Camera_View_Matrix();
	glm::dvec3 const camerapos = GfxRenderer->Camera_Position();
	float const fovy = glm::radians(Global.FieldOfView / Global.ZoomFactor);
	float const aspect = io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f;
	glm::mat4 const projection = glm::perspective(fovy, aspect, 0.1f, 10000.0f);

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
	auto const now = std::chrono::steady_clock::now();
	if (false == Force && now - m_track_last_commit < std::chrono::milliseconds(100))
		return;
	editor_track::commit(m_track_drag);
	m_track_dirty = false;
	m_track_last_commit = now;
}

void editor_mode::push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created)
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
	snap.created = std::move(Created);
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::restore_track_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo)
{
	if (Snapshot.tracks.empty())
		return;

	std::vector<TTrack *> tracks;
	for (auto const &entry : Snapshot.tracks)
		tracks.emplace_back(entry.first);
	EditorSnapshot current{Snapshot};
	current.tracks.clear();
	auto involved{tracks};
	if (Undo)
		involved.insert(involved.end(), Snapshot.created.begin(), Snapshot.created.end());
	for (auto *track : involved)
		current.tracks.emplace_back(track, editor_track::capture(*track));
	Opposite.push_back(std::move(current));

	if (Undo)
	{
		for (auto *track : Snapshot.created)
			editor_track::retire(*track);
	}
	else
	{
		for (auto *track : Snapshot.created)
			track->m_editorremoved = false;
	}
	for (auto const &entry : Snapshot.tracks)
		editor_track::apply(*entry.first, entry.second);
	editor_track::commit(tracks);
	if (false == m_route.chain.tracks.empty())
	{
		std::string error;
		editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error);
	}

	m_track_snap = {};
	m_track_dirty = false;
	if (m_node != tracks.front())
		m_track_point = {};
	m_node = tracks.front();
	ui()->set_node(m_node);
}

void editor_mode::render_track_ui()
{
	if (ImGui::BeginTabBar("##trackediting"))
	{
		if (ImGui::BeginTabItem("Route design"))
		{
			m_route_tab = true;
			render_route_ui();
			ImGui::EndTabItem();
		}
		if (ImGui::BeginTabItem("Single path"))
		{
			m_route_tab = false;
			render_path_ui();
			ImGui::EndTabItem();
		}
		ImGui::EndTabBar();
	}
}

void editor_mode::render_path_ui()
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
						editor_track::move_point(*track, {i, kind}, position);
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


bool editor_mode::route_active() const
{
	return ui()->mode() == nodebank_panel::TRACK && m_route_tab && false == m_route.chain.tracks.empty();
}

void editor_mode::route_recommend(alignment::vertex &Vertex) const
{
	auto const recommended{alignment::recommend(m_route.design.speed, Vertex.radius, m_route.design.norms)};
	Vertex.cant = recommended.cant;
	Vertex.transition_in = recommended.transition;
	Vertex.transition_out = recommended.transition;
	Vertex.vertical_radius = 0.0;
}

void editor_mode::route_reset()
{
	auto &route{m_route};
	route.chain = {};
	route.error.clear();
	route.status.clear();
	route.vertex = -1;
	route.result = {};
	if (route.from == nullptr || route.to == nullptr)
		return;
	if (false == editor_track::find_chain(route.from, route.to, route.chain, route.error))
		return;

	auto const &chain{route.chain};
	alignment::design design;
	design.norms = route.design.norms;
	design.shape = route.design.shape;
	design.start = chain.start;
	design.end = chain.end;
	auto const plan = [](glm::dvec3 const &Direction) { return glm::normalize(glm::dvec2{Direction.x, Direction.z}); };
	auto const slope = [](glm::dvec3 const &Direction) { return Direction.y / std::max(1e-9, std::hypot(Direction.x, Direction.z)); };
	design.start_direction = plan(chain.start_direction);
	design.end_direction = plan(chain.end_direction);
	design.start_grade = slope(chain.start_direction);
	design.end_grade = slope(chain.end_direction);
	design.start_radius = chain.start_radius;
	design.end_radius = chain.end_radius;
	design.speed = chain.velocity > 0.0 ? chain.velocity : 100.0;
	route.design = design;

	glm::dvec2 intersection;
	if (false == alignment::collinear(design))
	{
		alignment::vertex vertex;
		vertex.radius = chain.radius > 0.0 ? std::max(10.0, std::round(chain.radius / 10.0) * 10.0) : 1000.0;
		route_recommend(vertex);
		if (alignment::tangent_intersection(design, intersection))
		{
			route.design.vertices.push_back(vertex);
		}
		else
		{
			auto const distance{glm::distance(glm::dvec2{design.start.x, design.start.z}, glm::dvec2{design.end.x, design.end.z})};
			vertex.offset = distance / 3.0;
			route.design.vertices.push_back(vertex);
			route.design.vertices.push_back(vertex);
		}
	}
	route_update();
}

void editor_mode::route_apply()
{
	auto &route{m_route};
	route.status.clear();
	if (false == route.result.valid || route.chain.tracks.empty())
		return;
	editor_track::chain chain;
	if (false == editor_track::find_chain(route.from, route.to, chain, route.error))
		return;
	if (glm::distance(chain.start, route.chain.start) > 1e-3 || glm::distance(chain.end, route.chain.end) > 1e-3)
	{
		route.error = "The fragment changed since it was selected, select it again";
		return;
	}
	for (auto *track : chain.tracks)
	{
		std::string reason;
		if (false == editor_track::can_edit_geometry(*track, reason))
		{
			route.error = track->name() + ": " + reason;
			return;
		}
	}

	auto const pieces{alignment::pieces(route.result, route.design, chain.tracks.size())};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : chain.tracks)
		states.emplace_back(track, editor_track::capture(*track));
	auto created{editor_track::relay(chain, pieces)};
	auto const added{created.size()};
	push_track_snapshot(std::move(states), std::move(created));

	editor_track::find_chain(route.from, route.to, route.chain, route.error);
	route.status = "Re-laid " + std::to_string(chain.tracks.size()) + " paths" + (added > 0 ? ", added " + std::to_string(added) : std::string{});
	WriteLog("Editor: route design - " + route.status, logtype::generic);
}

glm::dvec3 editor_mode::route_vertex_position(int const Vertex) const
{
	auto const &design{m_route.design};
	auto const &result{m_route.result};
	auto const count{static_cast<int>(design.vertices.size())};
	glm::dvec2 const start{design.start.x, design.start.z};
	glm::dvec2 const end{design.end.x, design.end.z};
	glm::dvec2 position;
	if (Vertex < static_cast<int>(result.vertices.size()) && false == result.vertices.empty())
		position = result.vertices[Vertex];
	else if (count == 1)
	{
		if (false == alignment::tangent_intersection(design, position))
			position = (start + end) * 0.5;
	}
	else if (Vertex == 0)
		position = start + glm::normalize(design.start_direction) * design.vertices.front().offset;
	else if (Vertex == count - 1)
		position = end - glm::normalize(design.end_direction) * design.vertices.back().offset;
	else
		position = design.vertices[Vertex].position;

	double height;
	if (Vertex < static_cast<int>(result.vertex_elevations.size()))
		height = result.vertex_elevations[Vertex];
	else if (false == design.vertices[Vertex].auto_elevation)
		height = design.vertices[Vertex].elevation;
	else
		height = design.start.y + (design.end.y - design.start.y) * (Vertex + 1) / (count + 1.0);
	return {position.x, height, position.y};
}

bool editor_mode::pick_route_vertex()
{
	if (false == route_active())
		return false;
	screen_projection const projection;
	ImVec2 const mouse = ImGui::GetIO().MousePos;
	float best = kHandleRadius * kHandleRadius;
	int hit{-1};
	for (int i = 0; i < static_cast<int>(m_route.design.vertices.size()); ++i)
	{
		ImVec2 screen;
		if (false == projection.project(route_vertex_position(i), screen))
			continue;
		float const dx = screen.x - mouse.x;
		float const dy = screen.y - mouse.y;
		if (dx * dx + dy * dy < best)
		{
			best = dx * dx + dy * dy;
			hit = i;
		}
	}
	if (hit < 0)
		return false;
	m_route.vertex = hit;
	return true;
}

void editor_mode::draw_route_overlay() const
{
	auto const &route{m_route};
	auto const &design{route.design};
	auto const &result{route.result};
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList();

	std::vector<glm::dvec3> polygon{design.start};
	for (int i = 0; i < static_cast<int>(design.vertices.size()); ++i)
		polygon.push_back(route_vertex_position(i));
	polygon.push_back(design.end);
	for (std::size_t i = 0; i + 1 < polygon.size(); ++i)
		projection.line(drawlist, polygon[i], polygon[i + 1], IM_COL32(255, 210, 60, 150), 1.5f);

	if (result.length > 0.0 && result.profile.size() >= 2)
	{
		for (auto const &e : result.elements)
		{
			ImU32 const color = e.kind == alignment::element_kind::straight ? IM_COL32(255, 255, 255, 230) : e.kind == alignment::element_kind::arc ? IM_COL32(60, 230, 90, 255) : IM_COL32(255, 140, 30, 255);
			int const steps = std::clamp(static_cast<int>(e.length / 2.0), 1, 400);
			auto previous{alignment::evaluate(result, e.chainage).position};
			for (int j = 1; j <= steps; ++j)
			{
				auto const next{alignment::evaluate(result, e.chainage + e.length * j / steps).position};
				projection.line(drawlist, previous, next, color, 3.0f);
				previous = next;
			}
			ImVec2 screen;
			if (projection.project(alignment::evaluate(result, e.chainage).position, screen))
				drawlist->AddCircleFilled(screen, 3.5f, IM_COL32(255, 255, 255, 255));
		}
	}

	for (auto const atend : {false, true})
	{
		auto const &point{atend ? design.end : design.start};
		auto const direction{atend ? design.end_direction : design.start_direction};
		ImVec2 screen;
		if (projection.project(point, screen))
			drawlist->AddRectFilled(ImVec2(screen.x - 5.0f, screen.y - 5.0f), ImVec2(screen.x + 5.0f, screen.y + 5.0f), IM_COL32(70, 140, 255, 255));
		auto const arrow{glm::dvec3{direction.x, 0.0, direction.y} * (atend ? 15.0 : -15.0)};
		projection.line(drawlist, point, point + arrow, IM_COL32(70, 140, 255, 255), 2.0f);
	}
	for (int i = 0; i < static_cast<int>(design.vertices.size()); ++i)
	{
		ImVec2 screen;
		if (false == projection.project(route_vertex_position(i), screen))
			continue;
		drawlist->AddCircleFilled(screen, 6.0f, IM_COL32(255, 210, 60, 255));
		if (i == route.vertex)
			drawlist->AddCircle(screen, 11.0f, IM_COL32(255, 255, 255, 255), 16, 2.5f);
		char label[8];
		std::snprintf(label, sizeof(label), "W%d", i + 1);
		drawlist->AddText(ImVec2(screen.x + 9.0f, screen.y - 18.0f), IM_COL32(255, 210, 60, 255), label);
	}
}

void editor_mode::render_route_gizmo()
{
	auto &route{m_route};
	auto const count{static_cast<int>(route.design.vertices.size())};
	if (route.vertex < 0 || route.vertex >= count || false == m_gizmo_enabled)
	{
		m_route_gizmo_using = false;
		return;
	}

	ImGuizmo::BeginFrame();
	ImGuizmo::SetOrthographic(false);
	ImGuiIO const &io = ImGui::GetIO();
	ImGuizmo::SetRect(0.0f, 0.0f, io.DisplaySize.x, io.DisplaySize.y);
	glm::mat4 const view = GfxRenderer->Camera_View_Matrix();
	glm::dvec3 const camerapos = GfxRenderer->Camera_Position();
	float const fovy = glm::radians(Global.FieldOfView / Global.ZoomFactor);
	float const aspect = io.DisplaySize.y > 0.0f ? io.DisplaySize.x / io.DisplaySize.y : 1.0f;
	glm::mat4 const projection = glm::perspective(fovy, aspect, 0.1f, 10000.0f);

	auto const position{route_vertex_position(route.vertex)};
	if (false == m_route_gizmo_using)
		m_route_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(position - camerapos));
	glm::vec3 snapvalue(m_gizmo_snap);
	float const *snap = Global.ctrlState && snapvalue.x > 0.0f ? glm::value_ptr(snapvalue) : nullptr;
	ImGuizmo::Manipulate(glm::value_ptr(view), glm::value_ptr(projection), ImGuizmo::TRANSLATE, ImGuizmo::WORLD, glm::value_ptr(m_route_gizmo), nullptr, snap);

	if (false == ImGuizmo::IsUsing())
	{
		m_route_gizmo_using = false;
		return;
	}
	m_route_gizmo_using = true;
	glm::dvec3 const moved{camerapos + glm::dvec3(m_route_gizmo[3])};
	auto &vertex{route.design.vertices[route.vertex]};
	glm::dvec2 const point{moved.x, moved.z};
	if (count >= 2)
	{
		if (route.vertex == 0)
			vertex.offset = std::max(0.1, glm::dot(point - glm::dvec2{route.design.start.x, route.design.start.z}, glm::normalize(route.design.start_direction)));
		else if (route.vertex == count - 1)
			vertex.offset = std::max(0.1, glm::dot(glm::dvec2{route.design.end.x, route.design.end.z} - point, glm::normalize(route.design.end_direction)));
		else
			vertex.position = point;
	}
	if (std::abs(moved.y - position.y) > 1e-3)
	{
		vertex.auto_elevation = false;
		vertex.elevation = moved.y;
	}
	route_update();
}

void editor_mode::render_route_ui()
{
	auto &route{m_route};
	auto &design{route.design};
	ImGui::TextDisabled("Re-lays a chain of regular paths along an alignment designed with intersection points.\n"
	                    "The outer ends of the chain and their directions stay fixed.\n"
	                    "LMB: select a path or a vertex (W1, W2...)   Esc: release the vertex");

	auto *track = selected_track();
	auto const name = [](TTrack const *Track) { return Track == nullptr ? std::string{"-"} : (Track->name().empty() ? std::string{"(noname)"} : Track->name()); };
	ImGui::Text("First path: %s", name(route.from).c_str());
	ImGui::SameLine(ImGui::GetWindowContentRegionWidth() - 110.0f);
	if (ImGui::Button("Use selected##routefrom", ImVec2(110.0f, 0.0f)) && track != nullptr)
	{
		route.from = track;
		if (route.to == nullptr)
			route.to = track;
		route_reset();
	}
	ImGui::Text("Last path: %s", name(route.to).c_str());
	ImGui::SameLine(ImGui::GetWindowContentRegionWidth() - 110.0f);
	if (ImGui::Button("Use selected##routeto", ImVec2(110.0f, 0.0f)) && track != nullptr)
	{
		route.to = track;
		if (route.from == nullptr)
			route.from = track;
		route_reset();
	}
	if (false == route.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", route.error.c_str());
	if (route.chain.tracks.empty())
		return;

	ImGui::Text("Fragment: %zu paths, %.2f m", route.chain.tracks.size(), route.chain.length);
	ImGui::SameLine();
	if (ImGui::SmallButton("Reset design"))
	{
		route_reset();
		return;
	}

	bool changed{false};
	ImGui::PushItemWidth(120.0f);
	changed |= ImGui::InputDouble("Design speed (km/h)", &design.speed, 0.0, 0.0, "%.0f");
	design.speed = std::max(1.0, design.speed);
	int shape = static_cast<int>(design.shape);
	char const *shapes[] = {"Cubic parabola", "Clothoid"};
	if (ImGui::Combo("Transition curve", &shape, shapes, IM_ARRAYSIZE(shapes)))
	{
		design.shape = static_cast<alignment::transition_shape>(shape);
		changed = true;
	}
	ImGui::PopItemWidth();

	if (ImGui::TreeNode("Limits"))
	{
		auto &norms{design.norms};
		ImGui::PushItemWidth(100.0f);
		changed |= ImGui::InputDouble("Rail axes distance (mm)", &norms.gauge, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble("Unbalanced acceleration (m/s2)", &norms.unbalanced, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble("Maximum cant (mm)", &norms.cant_max, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble("Cant change rate (mm/s)", &norms.cant_rate, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble("Cant ramp 1:(k*V), k", &norms.ramp_factor, 0.0, 0.0, "%.1f");
		changed |= ImGui::InputDouble("Unbalanced acc. change rate (m/s3)", &norms.jerk, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble("Vertical curve R >= k*V^2, k", &norms.vertical_factor, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble("Vertical curve R min (m)", &norms.vertical_min, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble("Straight between curves (s of travel)", &norms.tangent_min_time, 0.0, 0.0, "%.1f");
		ImGui::PopItemWidth();
		norms.gauge = std::max(100.0, norms.gauge);
		norms.cant_rate = std::max(1.0, norms.cant_rate);
		norms.jerk = std::max(0.01, norms.jerk);
		ImGui::TreePop();
	}

	auto &vertices{design.vertices};
	if (ImGui::Button("Add vertex"))
	{
		alignment::vertex vertex;
		vertex.radius = vertices.empty() ? 1000.0 : vertices.back().radius;
		route_recommend(vertex);
		auto const distance{glm::distance(glm::dvec2{design.start.x, design.start.z}, glm::dvec2{design.end.x, design.end.z})};
		if (vertices.size() == 1)
		{
			vertices.front().offset = distance / 3.0;
			vertex.offset = distance / 3.0;
			vertices.push_back(vertex);
		}
		else if (vertices.empty())
		{
			vertices.push_back(vertex);
		}
		else
		{
			auto const index{std::clamp(route.vertex, 0, static_cast<int>(vertices.size()) - 2)};
			auto const a{route_vertex_position(index)};
			auto const b{route_vertex_position(index + 1)};
			vertex.position = glm::dvec2{(a.x + b.x) * 0.5, (a.z + b.z) * 0.5};
			for (int i = 0; i < static_cast<int>(vertices.size()); ++i)
			{
				auto const p{route_vertex_position(i)};
				vertices[i].position = {p.x, p.z};
			}
			vertices.insert(vertices.begin() + index + 1, vertex);
		}
		changed = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Remove vertex") && false == vertices.empty())
	{
		auto const index{route.vertex >= 0 && route.vertex < static_cast<int>(vertices.size()) ? route.vertex : static_cast<int>(vertices.size()) - 1};
		vertices.erase(vertices.begin() + index);
		route.vertex = -1;
		changed = true;
	}

	auto const count{static_cast<int>(vertices.size())};
	for (int i = 0; i < count; ++i)
	{
		auto &vertex{vertices[i]};
		ImGui::PushID(i);
		char header[32];
		std::snprintf(header, sizeof(header), "Vertex W%d", i + 1);
		if (ImGui::Selectable(header, route.vertex == i))
			route.vertex = (route.vertex == i ? -1 : i);
		ImGui::Indent();
		ImGui::PushItemWidth(110.0f);
		if (count == 1)
			ImGui::TextDisabled("At the intersection of the end tangents");
		else if (i == 0)
			changed |= ImGui::InputDouble("Distance from the start (m)", &vertex.offset, 0.0, 0.0, "%.2f");
		else if (i == count - 1)
			changed |= ImGui::InputDouble("Distance from the end (m)", &vertex.offset, 0.0, 0.0, "%.2f");
		else
		{
			ImGui::PushItemWidth(220.0f);
			changed |= ImGui::InputScalarN("Position (x, z)", ImGuiDataType_Double, glm::value_ptr(vertex.position), 2, nullptr, nullptr, "%.2f");
			ImGui::PopItemWidth();
		}
		changed |= ImGui::InputDouble("Radius R (m)", &vertex.radius, 0.0, 0.0, "%.1f");
		changed |= ImGui::InputDouble("Transition in (m)", &vertex.transition_in, 0.0, 0.0, "%.1f");
		changed |= ImGui::InputDouble("Transition out (m)", &vertex.transition_out, 0.0, 0.0, "%.1f");
		changed |= ImGui::InputDouble("Cant (mm)", &vertex.cant, 0.0, 0.0, "%.0f");
		changed |= ImGui::Checkbox("Elevation from the ends", &vertex.auto_elevation);
		if (false == vertex.auto_elevation)
			changed |= ImGui::InputDouble("Elevation (m)", &vertex.elevation, 0.0, 0.0, "%.3f");
		changed |= ImGui::InputDouble("Vertical curve R (0: min)", &vertex.vertical_radius, 0.0, 0.0, "%.0f");
		ImGui::PopItemWidth();
		vertex.radius = std::max(1.0, vertex.radius);
		vertex.transition_in = std::max(0.0, vertex.transition_in);
		vertex.transition_out = std::max(0.0, vertex.transition_out);
		vertex.cant = std::max(0.0, vertex.cant);
		vertex.vertical_radius = std::max(0.0, vertex.vertical_radius);
		if (ImGui::SmallButton("Apply recommended for V"))
		{
			route_recommend(vertex);
			changed = true;
		}

		auto const recommended{alignment::recommend(design.speed, vertex.radius, design.norms)};
		ImGui::TextDisabled("For %.0f km/h: R min %.0f m, cant %.0f mm (equilibrium %.0f, min %.0f), transition %.0f m", design.speed, recommended.radius_min, recommended.cant, recommended.cant_equilibrium, recommended.cant_min, recommended.transition);
		for (auto const &curve : route.result.curves)
		{
			if (curve.vertex != i)
				continue;
			ImGui::Text("Deflection %.4f deg, tangents %.2f / %.2f m, arc %.2f m", glm::degrees(curve.deflection), curve.tangent_in, curve.tangent_out, curve.arc_length);
			ImGui::Text("Unbalanced acc. %.3f m/s2, cant rate %.1f / %.1f mm/s, acc. rate %.3f / %.3f m/s3", curve.unbalanced, curve.cant_rate_in, curve.cant_rate_out, curve.jerk_in, curve.jerk_out);
		}
		ImGui::Unindent();
		ImGui::PopID();
	}
	if (changed)
		route_update();

	ImGui::Separator();
	auto const &result{route.result};
	if (result.length > 0.0)
	{
		auto const pieces{alignment::minimum_pieces(result, design)};
		ImGui::Text("Length %.2f m (now %.2f m), %zu elements", result.length, route.chain.length, result.elements.size());
		ImGui::TextDisabled("Paths: %zu in the fragment, %zu needed%s", route.chain.tracks.size(), pieces, pieces > route.chain.tracks.size() ? " (copies of the neighbours will be added)" : "");
	}
	for (auto const &error : result.errors)
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", error.c_str());
	for (auto const &warning : result.warnings)
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", warning.c_str());

	if (ImGui::TreeNode("Elements"))
	{
		char const *kinds[] = {"straight", "transition", "curve", "transition"};
		for (auto const &e : result.elements)
			ImGui::Text("%8.2f m  %-10s  L %8.2f m%s", e.chainage, kinds[static_cast<int>(e.kind)], e.length, e.kind == alignment::element_kind::straight ? "" : ("  R " + std::to_string(static_cast<int>(std::round(e.radius))) + " m").c_str());
		ImGui::TreePop();
	}

	if (result.valid)
	{
		if (ImGui::Button("Apply to the scenery"))
			route_apply();
	}
	else
	{
		ImGui::TextDisabled("Correct the errors to apply the design");
	}
	if (false == route.status.empty())
	{
		ImGui::SameLine();
		ImGui::TextUnformatted(route.status.c_str());
	}
}
