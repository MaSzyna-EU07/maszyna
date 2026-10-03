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

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "scene/scene.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>

namespace
{

using geometry::bezier;
using geometry::cross;
using geometry::grade_of;
using geometry::plan_distance;
using geometry::plan_of;
using geometry::turned;

std::string describe(editor_track::straight const &Line)
{
	return format("L %.2f m  az %.4f deg  i %.2f per mille  %zu paths", Line.length, Line.azimuth, Line.grade * 1000.0, Line.tracks.size());
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
	if (ui()->mode() != nodebank_panel::TRACK)
		return;
	m_track_window_open = true;
	if (track->eType == tt_Switch)
	{
		m_track_tab = track_tab::turnout;
	}
	else if (editor_track::is_straight(*track, m_straights.tolerance))
	{
		m_track_tab = track_tab::straights;
	}
	else if (std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) != m_route.chain.tracks.end() || route_from_curve(*track))
	{
		m_track_tab = track_tab::route;
	}
}

bool editor_mode::pick_track_handle()
{
	auto *track = selected_track();
	if (track == nullptr || false == editor_track::is_supported(*track) || m_route_tab)
		return false;

	screen_projection const projection;
	float best = kHandleRadius * kHandleRadius;
	editor_track::point_ref hit;
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		for (auto const kind : kPointKinds)
		{
			editor_track::point_ref const point{i, kind};
			float const distance = projection.mouse_distance2(editor_track::point_position(*track, point)) * (editor_track::is_end(kind) ? 0.5f : 1.0f);
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
		auto previous = bezier{path}.point(0.0);
		for (int i = 1; i <= samples; ++i)
		{
			auto const next = bezier{path}.point(static_cast<double>(i) / samples);
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
			if (editor_track::is_end(kind))
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

	gizmo_frame const frame;

	ImGuizmo::OPERATION const operation = (false == pointmode && m_gizmo_op == gizmo_operation::rotate) ? ImGuizmo::ROTATE_Y : ImGuizmo::TRANSLATE;

	if (false == m_track_gizmo_using)
	{
		auto const anchor = pointmode ? editor_track::point_position(*track, m_track_point) : editor_track::pivot(*track);
		m_track_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(anchor - frame.camera));
	}

	glm::mat4 delta(1.0f);
	frame.manipulate(operation, m_track_gizmo, operation == ImGuizmo::ROTATE_Y ? 5.0f : m_gizmo_snap, &delta);

	if (ImGuizmo::IsUsing())
	{
		if (false == m_track_gizmo_using)
		{
			m_track_gizmo_using = true;
			m_track_pivot = editor_track::pivot(*track);
			m_track_drag = {track};
			m_track_drag_points.clear();
			m_track_snap = {};
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
			glm::dvec3 const position = frame.camera + glm::dvec3(m_track_gizmo[3]);
			editor_track::move_point(*track, m_track_point, position);
			for (auto const &connection : m_track_drag_points)
				editor_track::move_point(*connection.first, connection.second, position);
			m_track_dirty = true;
			if (editor_track::is_end(m_track_point.kind) && m_track_drag_points.empty())
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

void editor_mode::trim_history()
{
	if (m_max_history_size >= 0 && (int)m_history.size() >= m_max_history_size)
		m_history.erase(m_history.begin(), m_history.begin() + ((int)m_history.size() - m_max_history_size + 1));
}

void editor_mode::push_track_snapshot(std::vector<std::pair<TTrack *, editor_track::state>> States, std::vector<TTrack *> Created, std::vector<TTrack *> Removed)
{
	if (States.empty() && Created.empty() && Removed.empty())
		return;

	trim_history();
	auto *track = false == States.empty() ? States.front().first : false == Created.empty() ? Created.front() : Removed.front();
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::TrackEdit;
	snap.node_name = track->name();
	snap.node_ptr = track;
	snap.position = track->location();
	snap.uuid = track->uuid;
	snap.tracks = std::move(States);
	snap.created = std::move(Created);
	infra_attach(snap);
	snap.removed = std::move(Removed);
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::restore_track_snapshot(EditorSnapshot const &Snapshot, std::vector<EditorSnapshot> &Opposite, bool const Undo)
{
	if (Snapshot.tracks.empty() && Snapshot.created.empty() && Snapshot.removed.empty())
		return;

	std::vector<TTrack *> tracks;
	for (auto const &entry : Snapshot.tracks)
		tracks.emplace_back(entry.first);
	m_infra_suspended = true;
	EditorSnapshot current{Snapshot};
	current.tracks.clear();
	auto involved{tracks};
	if (Undo)
		involved.insert(involved.end(), Snapshot.created.begin(), Snapshot.created.end());
	for (auto *track : involved)
		current.tracks.emplace_back(track, editor_track::capture(*track));
	current.infra.clear();
	for (auto const &entry : Snapshot.infra)
		if (auto const *binding{infra_find(entry.saved)})
			current.infra.push_back(infra_state_of(*binding));
	Opposite.push_back(std::move(current));

	if (Undo)
	{
		for (auto *track : Snapshot.created)
			editor_track::retire(*track);
		for (auto *track : Snapshot.removed)
		{
			track->m_editorremoved = false;
			tracks.push_back(track);
		}
	}
	else
	{
		for (auto *track : Snapshot.created)
			track->m_editorremoved = false;
		for (auto *track : Snapshot.removed)
			editor_track::retire(*track);
	}
	for (auto const &entry : Snapshot.tracks)
		editor_track::apply(*entry.first, entry.second);
	editor_track::commit(tracks);
	for (auto const &entry : Snapshot.infra)
		infra_restore_state(entry);
	m_infra_buffer.clear();
	m_infra_suspended = false;
	if (false == m_route.chain.tracks.empty())
	{
		std::string error;
		if (m_route.from->m_editorremoved || m_route.to->m_editorremoved || false == editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error))
		{
			m_route.chain = {};
			m_route.result = {};
			m_route.vertex = m_route.grip = -1;
		}
		else
		{
			route_bind_ends();
			route_update();
		}
	}

	m_track_snap = {};
	m_track_dirty = false;
	m_track_point = {};
	m_node = tracks.empty() || tracks.front()->m_editorremoved ? nullptr : tracks.front();
	ui()->set_node(m_node);
	straight_refresh();
	profile_after_undo();
}

void editor_mode::render_track_ui()
{
	ImGui::TextDisabled("LMB on a path opens its editor: straight, curve or switch");
	if (ImGui::Button("Open the editor window"))
		m_track_window_open = true;
	ImGui::SameLine();
	if (ImGui::Button("Vertical profile (grade line)"))
	{
		if (auto *track{selected_track()}; track != nullptr && m_profile.route.spans.empty())
			profile_open_run(*track);
		m_profile.open = true;
	}
	ImGui::SameLine();
	if (ImGui::Button("Infrastructure"))
		m_infra.open = true;
	if (ImGui::CollapsingHeader("Straights in the scenery"))
		render_straights_ui();
	render_switch_ui();
}

void editor_mode::render_track_window()
{
	if (false == m_track_window_open || selected_track() == nullptr)
		return;
	char const *title{"Path###trackeditor"};
	switch (m_track_tab)
	{
	case track_tab::straights: title = "Straight###trackeditor"; break;
	case track_tab::route: title = "Curve###trackeditor"; break;
	case track_tab::turnout: title = "Switch###trackeditor"; break;
	default: break;
	}
	ImGui::SetNextWindowSize(ImVec2(460.0f, 520.0f), ImGuiCond_FirstUseEver);
	if (false == ImGui::Begin(title, &m_track_window_open))
	{
		ImGui::End();
		return;
	}
	switch (m_track_tab)
	{
	case track_tab::straights:
		m_route_tab = true;
		render_straight_ui();
		break;
	case track_tab::route:
		m_route_tab = true;
		render_route_ui();
		break;
	case track_tab::turnout:
		m_route_tab = true;
		render_turnout_ui();
		break;
	default:
		m_route_tab = false;
		render_path_ui();
		break;
	}
	if (m_track_tab != track_tab::path)
	{
		ImGui::Separator();
		if (ImGui::SmallButton("Edit points and parameters of the single path"))
			m_track_tab = track_tab::path;
	}
	if (ImGui::SmallButton("Vertical profile along this line") && selected_track() != nullptr)
		profile_open_run(*selected_track());
	ImGui::SameLine();
	if (ImGui::SmallButton("Infrastructure along this line"))
	{
		m_infra.open = true;
		m_infra.scope = 1;
		infra_recognize();
	}
	ImGui::End();
}

void editor_mode::render_turnout_ui()
{
	auto *track{selected_track()};
	if (track == nullptr || track->eType != tt_Switch || track->m_paths.size() < 2)
		return;
	auto const &main{track->m_paths[0]};
	auto const &diverging{track->m_paths[1]};
	auto const origin{main.points[segment_data::point::start]};
	glm::dvec2 const axis{glm::normalize(plan_of(main.points[segment_data::point::end] - origin))};
	glm::dvec2 const normal{-axis.y, axis.x};
	auto const length{glm::dot(plan_of(main.points[segment_data::point::end] - origin), axis)};
	glm::dvec2 const end{diverging.points[segment_data::point::end].x - origin.x, diverging.points[segment_data::point::end].z - origin.z};
	auto const offset{glm::dot(end, normal)};
	auto const &control{diverging.points[segment_data::point::control2]};
	glm::dvec2 const tangent{glm::normalize(glm::dvec2{-control.x, -control.z})};
	auto const angle{std::atan2(std::abs(tangent.x * axis.y - tangent.y * axis.x), glm::dot(tangent, axis))};
	ImGui::Text("%s", track->name().empty() ? "(noname)" : track->name().c_str());
	ImGui::Text("Length %.3f m, diverging %s, end offset %.3f m", length, offset > 0.0 ? "left" : "right", std::abs(offset));
	if (angle > 1e-6)
		ImGui::Text("Angle 1:%.2f (%.4f deg), R in the entry %.0f m", 1.0 / std::tan(angle), glm::degrees(angle), diverging.radius);
	ImGui::TextDisabled("Gizmo moves the whole switch (W: turn), drag from a red ring at a free end to add a path");

	auto &tool{m_switch};
	if (false == tool.collected)
	{
		tool.templates = editor_track::standard_switch_templates();
		auto const found{editor_track::find_switch_templates()};
		tool.templates.insert(tool.templates.end(), found.begin(), found.end());
		tool.collected = true;
	}
	if (tool.templates.empty())
		return;
	m_turnout_template = std::clamp(m_turnout_template, 0, static_cast<int>(tool.templates.size()) - 1);
	ImGui::Separator();
	ImGui::PushItemWidth(-1.0f);
	if (ImGui::BeginCombo("##turnouttemplate", tool.templates[m_turnout_template].label.c_str()))
	{
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		{
			auto const label{(tool.templates[i].source != nullptr ? "Scenery: " : "PLK: ") + tool.templates[i].label};
			if (ImGui::Selectable(label.c_str(), i == m_turnout_template))
				m_turnout_template = i;
		}
		ImGui::EndCombo();
	}
	ImGui::PopItemWidth();
	auto const replace = [&](editor_track::switch_template const &Template, int const Side) {
		auto const grade{length > 0.0 ? (main.points[segment_data::point::end].y - origin.y) / length : 0.0};
		auto const paths{editor_track::place_switch(Template, origin, axis, Side, grade)};
		push_track_snapshot({{track, editor_track::capture(*track)}});
		track->m_paths.assign(paths.begin(), paths.begin() + 2);
		editor_track::commit({track});
	};
	int const side{offset > 0.0 ? 1 : -1};
	if (ImGui::Button("Replace the geometry with the template"))
		replace(tool.templates[m_turnout_template], side);
	ImGui::SameLine();
	if (ImGui::Button("Flip the side"))
	{
		auto const current{editor_track::find_switch_templates()};
		editor_track::switch_template own;
		own.source = track;
		for (int i = 0; i < 2; ++i)
		{
			auto &local{own.local[i]};
			local = track->m_paths[i];
			auto const tolocal = [&](glm::dvec3 const &Vector) { return glm::dvec3{glm::dot(plan_of(Vector), normal) * side, Vector.y, glm::dot(plan_of(Vector), axis)}; };
			local.points[segment_data::point::start] = tolocal(track->m_paths[i].points[segment_data::point::start] - origin);
			local.points[segment_data::point::end] = tolocal(track->m_paths[i].points[segment_data::point::end] - origin);
			local.points[segment_data::point::control1] = tolocal(track->m_paths[i].points[segment_data::point::control1]);
			local.points[segment_data::point::control2] = tolocal(track->m_paths[i].points[segment_data::point::control2]);
			if (side < 0)
				local.rolls = {-local.rolls[0], -local.rolls[1]};
		}
		replace(own, -side);
	}
	ImGui::Separator();
	render_path_parameters(*track);
}

void editor_mode::render_path_parameters(TTrack &Track)
{
	auto *track{&Track};
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
			char const *pathlabels[] = {"Texture 1 (rails/surface)", "Texture 2 (trackbed/side)", "Trackbed (switch)"};
			char const *switchlabels[] = {"Rails P1-P2", "Rails P3-P4", "Trackbed"};
			auto const &labels{track->eType == tt_Switch ? switchlabels : pathlabels};
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

	render_path_parameters(*track);

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
				if (editor_track::is_end(kind))
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
	return ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::route && false == m_route.chain.tracks.empty();
}

void editor_mode::route_recommend(alignment::vertex &Vertex) const
{
	auto const recommended{alignment::recommend(m_route.design.speed, Vertex.radius, m_route.design.norms)};
	Vertex.cant = recommended.cant;
	Vertex.transition_in = recommended.transition;
	Vertex.transition_out = recommended.transition;
}

void editor_mode::route_bind_ends()
{
	auto const &chain{m_route.chain};
	auto &design{m_route.design};
	design.start = chain.start;
	design.end = chain.end;
	design.start_direction = glm::normalize(plan_of(chain.start_direction));
	design.end_direction = glm::normalize(plan_of(chain.end_direction));
	design.start_grade = grade_of(chain.start_direction);
	design.end_grade = grade_of(chain.end_direction);
	design.start_radius = chain.start_radius;
	design.end_radius = chain.end_radius;
	design.start_reserve = editor_track::run_reserve(editor_track::straight_beyond(chain, false, m_straights.tolerance));
	design.end_reserve = editor_track::run_reserve(editor_track::straight_beyond(chain, true, m_straights.tolerance));
}

void editor_mode::route_reset()
{
	auto &route{m_route};
	route.chain = {};
	route.error.clear();
	route.status.clear();
	route.vertex = -1;
	route.grip = -1;
	route.result = {};
	if (route.from == nullptr || route.to == nullptr)
		return;
	if (false == editor_track::find_chain(route.from, route.to, route.chain, route.error))
		return;

	auto const &chain{route.chain};
	alignment::design design;
	design.norms = route.design.norms;
	design.shape = route.design.shape;
	design.transition_pieces = route.design.transition_pieces;
	design.speed = chain.velocity > 0.0 ? chain.velocity : 100.0;
	route.design = design;
	route_bind_ends();
	design = route.design;

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
			auto const distance{plan_distance(design.start, design.end)};
			vertex.offset = distance / 3.0;
			route.design.vertices.push_back(vertex);
			route.design.vertices.push_back(vertex);
		}
	}
	route_update();
}

bool editor_mode::route_from_curve(TTrack &Track)
{
	editor_track::curve curve;
	if (false == editor_track::find_curve(Track, m_straights.tolerance, m_route.design.norms.gauge, curve))
		return false;
	m_route.from = curve.from;
	m_route.to = curve.to;
	route_reset();
	if (m_route.chain.tracks.empty())
		return false;
	auto &design{m_route.design};
	alignment::vertex vertex;
	vertex.radius = curve.radius > 0.0 ? std::max(10.0, std::round(curve.radius)) : 1000.0;
	route_recommend(vertex);
	if (curve.transition_in > 0.0)
		vertex.transition_in = std::round(curve.transition_in);
	if (curve.transition_out > 0.0)
		vertex.transition_out = std::round(curve.transition_out);
	if (curve.cant > 0.0)
		vertex.cant = std::round(curve.cant);
	if (curve.reversals == 0)
	{
		vertex.reverse_turn = std::abs(curve.turn) > glm::pi<double>();
		if (curve.compound)
		{
			vertex.compound = true;
			vertex.radius2 = std::round(curve.radius2);
			vertex.transition_middle = std::round(curve.transition_middle);
			vertex.split = curve.split;
		}
		design.vertices = {vertex};
	}
	else
	{
		auto const distance{plan_distance(design.start, design.end)};
		vertex.offset = distance / 3.0;
		design.vertices = {vertex, vertex};
	}
	route_update();
	m_route.status = "Curve of " + std::to_string(m_route.chain.tracks.size()) + " paths, turning " + std::to_string(static_cast<int>(std::round(glm::degrees(curve.turn)))) + " deg";
	return true;
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

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	auto *from{route.from};
	auto *to{route.to};
	auto const startextension{route.result.start_extension};
	auto const endextension{route.result.end_extension};
	if (startextension > 0.0 || endextension > 0.0)
	{
		auto const before{chain};
		if (startextension > 0.0)
			from = editor_track::take_straight(editor_track::straight_beyond(before, false, m_straights.tolerance), from, before.start, route.result.start, states, created);
		if (endextension > 0.0)
			to = editor_track::take_straight(editor_track::straight_beyond(before, true, m_straights.tolerance), to, before.end, route.result.end, states, created);
		std::string error;
		if (false == editor_track::find_chain(from, to, chain, error) || plan_distance(chain.start, route.result.start) > 5.0 || plan_distance(chain.end, route.result.end) > 5.0)
		{
			for (auto *track : created)
				editor_track::retire(*track);
			for (auto entry = states.rbegin(); entry != states.rend(); ++entry)
			{
				editor_track::apply(*entry->first, entry->second);
				editor_track::commit({entry->first});
			}
			route.error = error.empty() ? "The adjoining straights can't be cut at the ends of the curve" : error;
			return;
		}
	}
	for (auto *track : chain.tracks)
		if (std::none_of(states.begin(), states.end(), [&](auto const &Entry) { return Entry.first == track; }))
			states.emplace_back(track, editor_track::capture(*track));
	auto pieces{alignment::pieces(route.result, route.design, chain.tracks.size())};
	editor_track::keep_heights(chain, pieces);
	auto relaid{editor_track::relay(chain, pieces)};
	auto const added{relaid.size()};
	created.insert(created.end(), relaid.begin(), relaid.end());
	push_track_snapshot(std::move(states), std::move(created));

	route.from = from;
	route.to = to;
	editor_track::find_chain(route.from, route.to, route.chain, route.error);
	if (startextension > 0.0 || endextension > 0.0)
	{
		route_bind_ends();
		auto &vertices{route.design.vertices};
		if (vertices.size() >= 2)
		{
			vertices.front().offset += startextension;
			vertices.back().offset += endextension;
		}
		route_update();
	}
	route.status = "Re-laid " + std::to_string(chain.tracks.size()) + " paths" + (added > 0 ? ", added " + std::to_string(added) : std::string{});
	if (startextension > 0.0 || endextension > 0.0)
		route.status += format(", the curve took over %.2f / %.2f m of the adjoining straights", startextension, endextension);
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

	auto const height{Vertex < static_cast<int>(result.vertex_elevations.size()) ? result.vertex_elevations[Vertex] : design.start.y + (design.end.y - design.start.y) * (Vertex + 1) / (count + 1.0)};
	return {position.x, height, position.y};
}

bool editor_mode::pick_route_vertex()
{
	if (false == route_active())
		return false;
	screen_projection const projection;
	float best = kHandleRadius * kHandleRadius;
	int hit{-1};
	for (int i = 0; i < static_cast<int>(m_route.design.vertices.size()); ++i)
	{
		if (auto const distance{projection.mouse_distance2(route_vertex_position(i))}; distance < best)
		{
			best = distance;
			hit = i;
		}
	}
	int griphit{-1};
	for (int i = 0; i < static_cast<int>(m_route.result.vertex_chainages.size()) && m_route.result.length > 0.0; ++i)
	{
		if (auto const distance{projection.mouse_distance2(alignment::evaluate(m_route.result, m_route.result.vertex_chainages[i]).position)}; distance < best)
		{
			best = distance;
			griphit = i;
			hit = -1;
		}
	}
	if (hit < 0 && griphit < 0)
		return false;
	m_route.vertex = hit;
	m_route.grip = griphit;
	return true;
}

void editor_mode::draw_route_overlay() const
{
	auto const &route{m_route};
	auto const &design{route.design};
	auto const &result{route.result};
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList();
	auto const &start{result.valid ? result.start : design.start};
	auto const &end{result.valid ? result.end : design.end};

	std::vector<glm::dvec3> polygon{start};
	for (int i = 0; i < static_cast<int>(design.vertices.size()); ++i)
		polygon.push_back(route_vertex_position(i));
	polygon.push_back(end);
	for (std::size_t i = 0; i + 1 < polygon.size(); ++i)
		projection.line(drawlist, polygon[i], polygon[i + 1], IM_COL32(255, 210, 60, 150), 1.5f);

	if (result.length > 0.0 && result.profile.size() >= 2)
	{
		for (auto const &e : result.elements)
		{
			ImU32 const color = e.kind == alignment::element_kind::straight ? IM_COL32(255, 255, 255, 230) : e.kind == alignment::element_kind::arc ? overlay_color::grip : IM_COL32(255, 140, 30, 255);
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
				drawlist->AddCircleFilled(screen, 3.5f, overlay_color::highlight);
		}
	}

	for (auto const atend : {false, true})
	{
		auto const &point{atend ? end : start};
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
		drawlist->AddCircleFilled(screen, 6.0f, overlay_color::marked);
		if (i == route.vertex)
			drawlist->AddCircle(screen, 11.0f, overlay_color::highlight, 16, 2.5f);
		if (i < static_cast<int>(result.vertex_chainages.size()) && result.length > 0.0 && projection.project(alignment::evaluate(result, result.vertex_chainages[i]).position, screen))
		{
			drawlist->AddCircleFilled(screen, 6.0f, overlay_color::grip, 4);
			if (i == route.grip)
				drawlist->AddCircle(screen, 11.0f, overlay_color::highlight, 16, 2.5f);
			auto const *report{i < static_cast<int>(result.curves.size()) ? &result.curves[i] : nullptr};
			auto const radius{report != nullptr && report->radius2 > 0.0 ? format("R %.0f / %.0f", report->radius, report->radius2) : format("R %.0f", report != nullptr ? report->radius : design.vertices[i].radius)};
			drawlist->AddText(ImVec2(screen.x + 9.0f, screen.y + 4.0f), overlay_color::grip, radius.c_str());
		}
		drawlist->AddText(ImVec2(screen.x + 9.0f, screen.y - 18.0f), overlay_color::marked, format("W%d", i + 1).c_str());
	}
}

void editor_mode::render_route_gizmo()
{
	auto &route{m_route};
	auto const count{static_cast<int>(route.design.vertices.size())};
	bool const grip{route.grip >= 0 && route.grip < count && route.grip < static_cast<int>(route.result.vertex_chainages.size())};
	if ((false == grip && (route.vertex < 0 || route.vertex >= count)) || false == m_gizmo_enabled)
	{
		m_route_gizmo_using = false;
		return;
	}

	gizmo_frame const frame;

	auto const position{grip ? alignment::evaluate(route.result, route.result.vertex_chainages[route.grip]).position : route_vertex_position(route.vertex)};
	if (false == m_route_gizmo_using)
		m_route_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(position - frame.camera));
	frame.manipulate(ImGuizmo::TRANSLATE, m_route_gizmo, m_gizmo_snap);

	if (false == ImGuizmo::IsUsing())
	{
		if (m_route_gizmo_using)
		{
			m_route_gizmo_using = false;
			route_apply();
		}
		return;
	}
	m_route_gizmo_using = true;
	glm::dvec3 const moved{frame.camera + glm::dvec3(m_route_gizmo[3])};
	if (grip)
	{
		auto &vertex{route.design.vertices[route.grip]};
		auto const corner{route_vertex_position(route.grip)};
		auto const distance{plan_distance(moved, corner)};
		for (auto const &curve : route.result.curves)
		{
			if (curve.vertex != route.grip || vertex.reverse_turn)
				continue;
			auto const secant{1.0 / std::cos(curve.deflection * 0.5) - 1.0};
			if (secant > 1e-6)
				vertex.radius = std::max(10.0, std::round(distance / secant));
		}
		route_update();
		return;
	}
	auto &vertex{route.design.vertices[route.vertex]};
	glm::dvec2 const point{moved.x, moved.z};
	if (count >= 2)
	{
		if (route.vertex == 0)
			vertex.offset = std::max(0.1, glm::dot(point - plan_of(route.design.start), glm::normalize(route.design.start_direction)));
		else if (route.vertex == count - 1)
			vertex.offset = std::max(0.1, glm::dot(plan_of(route.design.end) - point, glm::normalize(route.design.end_direction)));
		else
			vertex.position = point;
	}
	route_update();
}

void editor_mode::render_route_ui()
{
	auto &route{m_route};
	auto &design{route.design};
	ImGui::TextDisabled("LMB on a curve: loads the whole curve between the adjoining straights.\n"
	                    "The ends of the curve and their directions stay fixed.\n"
	                    "LMB on a vertex (W1, W2...): moves it with the gizmo   Esc: release the vertex");

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
	if (ImGui::SliderInt("Paths per transition curve", &design.transition_pieces, 1, 4))
		route_update();
	if (ImGui::IsItemDeactivatedAfterEdit())
		route_apply();
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
		auto const distance{plan_distance(design.start, design.end)};
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
		if (ImGui::Selectable(format("Vertex W%d", i + 1).c_str(), route.vertex == i))
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
		changed |= ImGui::InputDouble(vertex.compound ? "Radius R1 (m)" : "Radius R (m)", &vertex.radius, 0.0, 0.0, "%.1f");
		changed |= ImGui::Checkbox("Compound curve (two radii)", &vertex.compound);
		if (vertex.compound)
		{
			changed |= ImGui::InputDouble("Radius R2 (m)", &vertex.radius2, 0.0, 0.0, "%.1f");
			changed |= ImGui::InputDouble("Transition R1-R2 (m)", &vertex.transition_middle, 0.0, 0.0, "%.1f");
			float split{static_cast<float>(vertex.split)};
			if (ImGui::SliderFloat("Share of R1", &split, 0.0f, 1.0f, "%.2f"))
			{
				vertex.split = split;
				changed = true;
			}
			vertex.radius2 = std::max(1.0, vertex.radius2);
			vertex.transition_middle = std::max(0.0, vertex.transition_middle);
		}
		changed |= ImGui::InputDouble("Transition in (m)", &vertex.transition_in, 0.0, 0.0, "%.1f");
		changed |= ImGui::InputDouble("Transition out (m)", &vertex.transition_out, 0.0, 0.0, "%.1f");
		changed |= ImGui::InputDouble("Cant (mm)", &vertex.cant, 0.0, 0.0, "%.0f");
		changed |= ImGui::Checkbox("Turn the longer way (over 180 deg)", &vertex.reverse_turn);
		ImGui::PopItemWidth();
		vertex.radius = std::max(1.0, vertex.radius);
		vertex.transition_in = std::max(0.0, vertex.transition_in);
		vertex.transition_out = std::max(0.0, vertex.transition_out);
		vertex.cant = std::max(0.0, vertex.cant);
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
		if (result.start_extension > 0.0 || result.end_extension > 0.0)
			ImGui::Text("Takes over %.2f m of the straight at the start, %.2f m at the end", result.start_extension, result.end_extension);
	}
	ImGui::TextDisabled("Straight available beyond the ends: %.2f / %.2f m", design.start_reserve, design.end_reserve);
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

editor_track::straight const &editor_mode::current_straight()
{
	auto *track{selected_track()};
	if (track != m_straights.current_for)
	{
		m_straights.current_for = track;
		m_straights.handle = -1;
		m_straights.current = track != nullptr ? editor_track::find_straight(*track, m_straights.tolerance) : editor_track::straight{};
		find_neighbour_straights();
	}
	return m_straights.current;
}

void editor_mode::draw_straights_overlay() const
{
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList();
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const nearby = [&](editor_track::straight const &Line) {
		auto const offset{camera - Line.start};
		auto const along{std::clamp(glm::dot(plan_of(offset), Line.direction), 0.0, Line.length)};
		auto const closest{Line.start + glm::dvec3{Line.direction.x, 0.0, Line.direction.y} * along};
		return plan_distance(closest, camera) < 3000.0;
	};
	auto const draw = [&](editor_track::straight const &Line, ImU32 const Color, float const Thickness) {
		if (Line.tracks.empty())
			return;
		projection.line(drawlist, Line.start, Line.end, Color, Thickness);
		for (auto const &point : {Line.start, Line.end})
		{
			ImVec2 screen;
			if (projection.project(point, screen))
				drawlist->AddCircleFilled(screen, Thickness + 2.0f, Color);
		}
	};
	for (int i = 0; i < static_cast<int>(m_straights.found.size()); ++i)
	{
		auto const &line{m_straights.found[i]};
		if (nearby(line))
			draw(line, i == m_straights.listed ? overlay_color::selected : IM_COL32(255, 255, 255, 110), i == m_straights.listed ? 4.0f : 2.0f);
	}
	for (auto const &member : m_straights.set)
		draw(member, IM_COL32(255, 150, 30, 255), 4.0f);
	draw(m_straights.current, overlay_color::selected, 4.0f);
	if (m_straights.tool_placed)
	{
		auto const &state{m_straights};
		auto const &line{state.tool_line};
		ImVec2 screen;
		if (projection.project(state.tool_point, screen))
			drawlist->AddCircleFilled(screen, 7.0f, overlay_color::invalid);
		if (projection.project(state.tool_handle, screen))
			drawlist->AddCircleFilled(screen, 7.0f, overlay_color::marked);
		if (state.tool == 1)
		{
			auto const direction{plan_of(state.tool_handle - state.tool_point)};
			if (glm::length(direction) > 1e-3)
			{
				auto const unit{glm::normalize(direction)};
				auto const rest{std::max(0.0, line.length - state.tool_at)};
				projection.line(drawlist, state.tool_point, state.tool_point + glm::dvec3{unit.x, line.grade, unit.y} * rest, overlay_color::marked, 3.0f);
			}
		}
		else
		{
			glm::dvec2 const normal{-line.direction.y, line.direction.x};
			auto const offset{plan_of(state.tool_handle - state.tool_point)};
			auto const shift{glm::dot(offset, normal)};
			auto const length{glm::dot(offset, line.direction)};
			auto const shifted{glm::dvec3{normal.x, 0.0, normal.y} * shift};
			auto const along{glm::dvec3{line.direction.x, 0.0, line.direction.y}};
			projection.line(drawlist, state.tool_point, state.tool_point + along * length + shifted, IM_COL32(255, 210, 60, 160), 2.0f);
			projection.line(drawlist, state.tool_point + along * length + shifted, line.end + shifted, overlay_color::marked, 3.0f);
		}
	}
	if (m_straights.dragging && m_straights.fitting)
	{
		auto const &result{m_route.result};
		if (result.length > 0.0 && result.profile.size() >= 2)
		{
			int const steps{std::clamp(static_cast<int>(result.length / 2.0), 2, 2000)};
			auto previous{alignment::evaluate(result, 0.0).position};
			for (int i = 1; i <= steps; ++i)
			{
				auto const next{alignment::evaluate(result, result.length * i / steps).position};
				projection.line(drawlist, previous, next, result.valid ? overlay_color::marked : overlay_color::invalid, 3.0f);
				previous = next;
			}
		}
	}
	else if (m_straights.dragging)
	{
		for (auto const &member : m_straights.drag_lines)
		{
			for (auto const *track : member.tracks)
			{
				auto const &path{track->m_paths.front()};
				projection.line(drawlist, path.points[segment_data::point::start], path.points[segment_data::point::end], overlay_color::marked, 3.0f);
			}
		}
	}
	else if (false == m_straights.current.tracks.empty())
	{
		auto const &line{m_straights.current};
		glm::dvec3 const handles[] = {line.start, line.end, (line.start + line.end) * 0.5};
		for (int i = 0; i < 3; ++i)
		{
			ImVec2 screen;
			if (false == projection.project(handles[i], screen))
				continue;
			if (i < 2)
				drawlist->AddRectFilled(ImVec2(screen.x - 6.0f, screen.y - 6.0f), ImVec2(screen.x + 6.0f, screen.y + 6.0f), overlay_color::selected);
			else
				drawlist->AddCircleFilled(screen, 6.0f, overlay_color::selected, 4);
			if (i == m_straights.handle)
				drawlist->AddCircle(screen, 12.0f, overlay_color::highlight, 16, 2.5f);
		}
	}
	auto const &line{m_straights.current};
	if (line.tracks.empty() || m_straights.dragging)
		return;
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	for (auto const &other : m_straights.neighbours)
	{
		auto const across = [&](glm::dvec3 const &Point) { return glm::dot(plan_of(Point - line.start), normal); };
		auto const a0{line.along(other.start)}, a1{line.along(other.end)};
		auto const from{std::max(0.0, std::min(a0, a1))};
		auto const to{std::min(line.length, std::max(a0, a1))};
		if (to <= from)
			continue;
		auto const middle{(from + to) * 0.5};
		auto const fraction{std::abs(a1 - a0) > 1e-6 ? (middle - a0) / (a1 - a0) : 0.0};
		auto const lateral{across(other.start) + (across(other.end) - across(other.start)) * fraction};
		glm::dvec3 const foot{line.start.x + line.direction.x * middle, line.start.y + line.grade * middle, line.start.z + line.direction.y * middle};
		glm::dvec3 const target{foot.x + normal.x * lateral, foot.y, foot.z + normal.y * lateral};
		projection.line(drawlist, foot, target, IM_COL32(255, 230, 120, 230), 1.5f);
		ImVec2 screen;
		if (projection.project((foot + target) * 0.5, screen))
		{
			auto const angle{glm::degrees(std::abs(std::asin(std::clamp(line.direction.x * other.direction.y - line.direction.y * other.direction.x, -1.0, 1.0))))};
			auto const text{angle > 0.001 ? format("%.3f m  %.3f deg", std::abs(lateral), angle) : format("%.3f m", std::abs(lateral))};
			auto const *label{text.c_str()};
			auto const size{ImGui::CalcTextSize(label)};
			drawlist->AddRectFilled(ImVec2(screen.x - 3.0f, screen.y - 2.0f), ImVec2(screen.x + size.x + 3.0f, screen.y + size.y + 2.0f), IM_COL32(0, 0, 0, 170), 3.0f);
			drawlist->AddText(screen, IM_COL32(255, 230, 120, 255), label);
		}
	}
}

void editor_mode::find_neighbour_straights()
{
	auto &state{m_straights};
	state.neighbours.clear();
	auto const &line{state.current};
	if (line.tracks.empty())
		return;
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	auto const middle{(line.start + line.end) * 0.5};
	std::vector<TTrack const *> visited(line.tracks.begin(), line.tracks.end());
	auto const sections{simulation::Region->sections(middle, static_cast<float>(line.length * 0.5 + 60.0))};
	for (auto *section : sections)
	{
		for (auto const &cell : section->m_cells)
		{
			for (auto *track : cell.m_paths)
			{
				if (std::find(visited.begin(), visited.end(), track) != visited.end() || false == editor_track::is_straight(*track, state.tolerance))
					continue;
				auto other{editor_track::find_straight(*track, state.tolerance)};
				visited.insert(visited.end(), other.tracks.begin(), other.tracks.end());
				if (other.tracks.empty() || std::abs(line.direction.x * other.direction.y - line.direction.y * other.direction.x) > std::sin(glm::radians(10.0)))
					continue;
				auto const offset{glm::dvec2{(other.start + other.end).x * 0.5 - line.start.x, (other.start + other.end).z * 0.5 - line.start.z}};
				if (std::abs(glm::dot(offset, normal)) > 60.0)
					continue;
				auto const a0{line.along(other.start)};
				auto const a1{line.along(other.end)};
				if (std::max(a0, a1) <= 0.0 || std::min(a0, a1) >= line.length)
					continue;
				state.neighbours.push_back(std::move(other));
			}
		}
	}
}

glm::dvec3 editor_mode::snap_straight_offset(editor_track::straight const &Line, glm::dvec3 const &Offset) const
{
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	auto const middle{(Line.start + Line.end) * 0.5 + Offset};
	double best{0.2};
	double correction{0.0};
	for (auto const &other : m_straights.neighbours)
	{
		if (std::abs(Line.direction.x * other.direction.y - Line.direction.y * other.direction.x) > std::sin(glm::radians(0.5)))
			continue;
		auto const distance{glm::dot(plan_of(other.start - middle), normal)};
		for (auto const spacing : m_straights.spacings)
		{
			auto const error{std::abs(distance) - spacing};
			if (std::abs(error) < best)
			{
				best = std::abs(error);
				correction = distance > 0.0 ? error : -error;
			}
		}
	}
	return Offset + glm::dvec3{normal.x, 0.0, normal.y} * correction;
}

glm::dvec3 editor_mode::snap_straight_direction(glm::dvec3 const &Pivot, glm::dvec3 const &Moved) const
{
	glm::dvec2 const offset{Moved.x - Pivot.x, Moved.z - Pivot.z};
	auto const length{glm::length(offset)};
	if (length < 1e-3)
		return Moved;
	auto const direction{offset / length};
	for (auto const &other : m_straights.neighbours)
	{
		auto const sine{direction.x * other.direction.y - direction.y * other.direction.x};
		if (std::abs(sine) > std::sin(glm::radians(0.5)))
			continue;
		auto const aligned{other.direction * (glm::dot(direction, other.direction) < 0.0 ? -1.0 : 1.0)};
		return {Pivot.x + aligned.x * length, Moved.y, Pivot.z + aligned.y * length};
	}
	return Moved;
}

void editor_mode::render_straights_ui()
{
	auto &state{m_straights};
	ImGui::TextDisabled("Straights recognized from chains of collinear straight paths.\nLMB: select a path to see the straight it belongs to");

	ImGui::PushItemWidth(100.0f);
	double angle{glm::degrees(state.tolerance.angle)};
	if (ImGui::InputDouble("Angle tolerance (deg)", &angle, 0.0, 0.0, "%.4f"))
		state.tolerance.angle = glm::radians(std::max(0.0, angle));
	ImGui::InputDouble("Offset tolerance (m)", &state.tolerance.offset, 0.0, 0.0, "%.3f");
	ImGui::InputDouble("Treat as straight above R (m)", &state.tolerance.radius, 0.0, 0.0, "%.0f");
	ImGui::InputDouble("Minimum length (m)", &state.minimum_length, 0.0, 0.0, "%.0f");
	ImGui::PopItemWidth();
	if (ImGui::Button("Recognize straights in the scenery"))
	{
		state.found = editor_track::find_straights(state.minimum_length, state.tolerance);
		state.listed = -1;
		state.current_for = nullptr;
	}
	if (false == state.found.empty())
	{
		ImGui::Text("%zu straights", state.found.size());
		ImGui::SameLine();
		ImGui::PushItemWidth(150.0f);
		ImGui::InputTextWithHint("##straightfilter", "path name", state.filter, sizeof(state.filter));
		ImGui::PopItemWidth();
		ImGui::BeginChild("##straights", ImVec2(0.0f, 160.0f), true);
		for (int i = 0; i < static_cast<int>(state.found.size()); ++i)
		{
			auto const &line{state.found[i]};
			if (state.filter[0] != '\0' && std::none_of(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Track) { return Track->name().find(state.filter) != std::string::npos; }))
				continue;
			auto const label{std::to_string(i + 1) + ".  " + describe(line) + "##straight" + std::to_string(i)};
			if (ImGui::Selectable(label.c_str(), state.listed == i))
			{
				state.listed = i;
				m_node = line.tracks[line.tracks.size() / 2];
				ui()->set_node(m_node);
				start_focus(m_node);
			}
		}
		ImGui::EndChild();
	}

}

void editor_mode::render_straight_ui()
{
	auto &state{m_straights};
	auto const &line{current_straight()};
	if (line.tracks.empty())
	{
		ImGui::TextDisabled(selected_track() != nullptr ? "The selected path isn't straight" : "No path selected");
	}
	else
	{
		ImGui::Text("Straight of the selected path");
		ImGui::Indent();
		ImGui::TextUnformatted(describe(line).c_str());
		ImGui::Text("From (%.3f, %.3f, %.3f)", line.start.x, line.start.y, line.start.z);
		ImGui::Text("To   (%.3f, %.3f, %.3f)", line.end.x, line.end.y, line.end.z);
		ImGui::Text("Paths: %s ... %s", line.tracks.front()->name().c_str(), line.tracks.back()->name().c_str());
		ImGui::Unindent();
		if (in_straight_set(line))
		{
			if (ImGui::Button("Remove from the set"))
				state.set.erase(std::remove_if(state.set.begin(), state.set.end(), [&](editor_track::straight const &Member) { return std::any_of(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Track) { return std::find(Member.tracks.begin(), Member.tracks.end(), Track) != Member.tracks.end(); }); }), state.set.end());
		}
		else if (ImGui::Button("Add to the set"))
		{
			state.set.push_back(line);
		}

		if (false == ImGui::IsAnyItemActive())
		{
			state.edit_start = line.start;
			state.edit_end = line.end;
			state.edit_length = line.length;
			state.edit_azimuth = line.azimuth;
		}
		ImGui::TextDisabled("Drag the ends or the middle of the straight with the gizmo, or enter the values");
		ImGui::PushItemWidth(260.0f);
		ImGui::InputScalarN("Start (x, y, z)", ImGuiDataType_Double, glm::value_ptr(state.edit_start), 3, nullptr, nullptr, "%.3f");
		ImGui::InputScalarN("End (x, y, z)", ImGuiDataType_Double, glm::value_ptr(state.edit_end), 3, nullptr, nullptr, "%.3f");
		ImGui::PopItemWidth();
		if (ImGui::Button("Apply ends"))
			straight_apply(line, state.edit_start, state.edit_end);
		ImGui::PushItemWidth(120.0f);
		ImGui::InputDouble("Length (m)", &state.edit_length, 0.0, 0.0, "%.3f");
		ImGui::SameLine();
		if (ImGui::Button("Set length from the start"))
		{
			glm::dvec3 const direction{line.direction.x, line.grade, line.direction.y};
			straight_apply(line, line.start, line.start + direction * std::max(0.1, state.edit_length));
		}
		ImGui::InputDouble("Azimuth (deg)", &state.edit_azimuth, 0.0, 0.0, "%.5f");
		ImGui::SameLine();
		if (ImGui::Button("Turn around the start"))
		{
			auto const azimuth{glm::radians(state.edit_azimuth)};
			glm::dvec3 const direction{std::sin(azimuth), line.grade, std::cos(azimuth)};
			straight_apply(line, line.start, line.start + direction * line.length);
		}
		ImGui::PopItemWidth();
	}
	if (false == line.tracks.empty())
	{
		ImGui::Separator();
		ImGui::Text("Mouse tools");
		auto const toolbutton = [&](char const *Label, int const Tool) {
			bool const active{state.tool == Tool};
			if (active)
				ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
			if (ImGui::Button(Label))
			{
				state.tool = active ? 0 : Tool;
				state.tool_placed = false;
				state.handle = -1;
			}
			if (active)
				ImGui::PopStyleColor();
		};
		toolbutton("Break", 1);
		ImGui::SameLine();
		toolbutton("S-curve", 2);
		if (state.tool == 1)
			ImGui::TextDisabled(state.tool_placed ? "Drag the handle to turn the rest of the straight, release to fit the curve" : "LMB on the straight: place the break point");
		else if (state.tool == 2)
			ImGui::TextDisabled(state.tool_placed ? "Drag the handle: sideways sets the shift, along sets its length" : "LMB on the straight: place the start of the shift");
		ImGui::PushItemWidth(100.0f);
		ImGui::InputDouble("Curve R (m, 0: automatic)", &state.curve_radius, 0.0, 0.0, "%.0f");
		ImGui::Checkbox("Transitions from the line speed", &state.auto_transitions);
		if (false == state.auto_transitions)
		{
			ImGui::SameLine();
			ImGui::InputDouble("Transition (m)", &state.transition, 0.0, 0.0, "%.1f");
		}
		ImGui::PopItemWidth();
	}
	if (false == state.status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", state.status.c_str());

	if (false == state.set.empty())
	{
		ImGui::Text("Set: %zu straights, dragging any of them moves the whole set, keeping it parallel", state.set.size());
		ImGui::SameLine();
		if (ImGui::SmallButton("Clear the set"))
			state.set.clear();
	}

	if (false == state.neighbours.empty())
	{
		ImGui::Separator();
		ImGui::Text("Neighbouring straights: %zu (distances shown in the view)", state.neighbours.size());
		ImGui::TextDisabled("Dragging the middle snaps the distance to: 3.5, 4.0, 4.5, 4.75, 5.0, 5.5, 6.0 m; dragging an end snaps parallel");
	}
}

bool editor_mode::straights_active()
{
	return ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::straights && false == current_straight().tracks.empty();
}

bool editor_mode::pick_straight_handle()
{
	if (false == straights_active())
		return false;
	auto const &line{current_straight()};
	glm::dvec3 const handles[] = {line.start, line.end, (line.start + line.end) * 0.5};
	screen_projection const projection;
	float best = kHandleRadius * kHandleRadius;
	int hit{-1};
	for (int i = 0; i < 3; ++i)
	{
		if (auto const distance{projection.mouse_distance2(handles[i])}; distance < best)
		{
			best = distance;
			hit = i;
		}
	}
	if (hit < 0)
		return false;
	m_straights.handle = hit;
	return true;
}

void editor_mode::straight_refresh()
{
	auto &state{m_straights};
	state.current_for = selected_track();
	state.current = state.current_for != nullptr ? editor_track::find_straight(*state.current_for, state.tolerance) : editor_track::straight{};
	find_neighbour_straights();
	for (auto &line : state.found)
		if (false == line.tracks.empty())
			line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
	for (auto &line : state.set)
		if (false == line.tracks.empty())
			line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
}

bool editor_mode::in_straight_set(editor_track::straight const &Line) const
{
	for (auto const &member : m_straights.set)
		for (auto const *track : Line.tracks)
			if (std::find(member.tracks.begin(), member.tracks.end(), track) != member.tracks.end())
				return true;
	return false;
}

void editor_mode::straight_apply(editor_track::straight const &Line, glm::dvec3 const &Start, glm::dvec3 const &End)
{
	straights_apply({Line}, {{Start, End}});
}

void editor_mode::straights_apply(std::vector<editor_track::straight> const &Lines, std::vector<std::pair<glm::dvec3, glm::dvec3>> const &Ends)
{
	auto &state{m_straights};
	state.status.clear();
	auto const tracks{editor_track::straight_affected(Lines)};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : tracks)
		states.emplace_back(track, editor_track::capture(*track));
	push_track_snapshot(std::move(states));
	editor_track::move_straights(Lines, Ends);
	editor_track::commit(tracks);
	straight_refresh();
}

void editor_mode::render_straight_gizmo()
{
	auto &state{m_straights};
	if (state.tool_mouse)
	{
		glm::dvec3 const ground{Global.pCamera.Pos + GfxRenderer->Mouse_Position()};
		state.tool_handle = {ground.x, state.tool_handle.y, ground.z};
		return;
	}
	if (state.tool_placed && m_gizmo_enabled)
	{
		render_straight_tool_gizmo();
		return;
	}
	auto const &line{state.dragging ? state.drag_line : current_straight()};
	if (state.handle < 0 || line.tracks.empty() || false == m_gizmo_enabled)
	{
		state.dragging = false;
		return;
	}

	gizmo_frame const frame;

	glm::dvec3 const anchors[] = {line.start, line.end, (line.start + line.end) * 0.5};
	auto const anchor{anchors[state.handle]};
	if (false == state.dragging)
		state.gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(anchor - frame.camera));
	frame.manipulate(ImGuizmo::TRANSLATE, state.gizmo, m_gizmo_snap);

	if (ImGuizmo::IsUsing())
	{
		if (false == state.dragging)
		{
			std::vector<editor_track::straight> lines;
			if (in_straight_set(line))
			{
				for (auto const &member : state.set)
					if (false == member.tracks.empty())
						lines.push_back(editor_track::find_straight(*member.tracks.front(), state.tolerance));
			}
			else
			{
				lines.push_back(line);
			}
			state.fitting = lines.size() == 1 && start_curve_fit(line);
			auto const tracks{state.fitting ? std::vector<TTrack *>{} : editor_track::straight_affected(lines)};
			state.dragging = true;
			state.drag_line = line;
			state.drag_lines = lines;
			state.drag_states.clear();
			for (auto *track : tracks)
				state.drag_states.emplace_back(track, editor_track::capture(*track));
			if (false == state.fitting)
			{
				push_track_snapshot(state.drag_states);
				m_track_drag = tracks;
			}
		}
		auto const &grabbed{state.drag_line};
		glm::dvec3 moved{frame.camera + glm::dvec3(state.gizmo[3])};
		std::function<glm::dvec3(glm::dvec3 const &)> transform;
		if (state.handle == 2)
		{
			auto const offset{snap_straight_offset(grabbed, moved - (grabbed.start + grabbed.end) * 0.5)};
			transform = [offset](glm::dvec3 const &Point) { return Point + offset; };
		}
		else
		{
			auto const pivot{state.handle == 0 ? grabbed.end : grabbed.start};
			auto const original{state.handle == 0 ? grabbed.start : grabbed.end};
			moved = snap_straight_direction(pivot, moved);
			glm::dvec2 const before{original.x - pivot.x, original.z - pivot.z};
			glm::dvec2 const after{moved.x - pivot.x, moved.z - pivot.z};
			auto const lengthbefore{glm::length(before)};
			auto const lengthafter{std::max(0.1, glm::length(after))};
			auto const axis{before / std::max(1e-9, lengthbefore)};
			auto const newaxis{after / lengthafter};
			auto const scale{lengthafter / std::max(1e-9, lengthbefore)};
			auto const rise{moved.y - original.y};
			transform = [=](glm::dvec3 const &Point) {
				glm::dvec2 const offset{Point.x - pivot.x, Point.z - pivot.z};
				auto const along{glm::dot(offset, axis)};
				auto const across{axis.x * offset.y - axis.y * offset.x};
				auto const planar{plan_of(pivot) + newaxis * (along * scale) + glm::dvec2{-newaxis.y, newaxis.x} * across};
				return glm::dvec3{planar.x, Point.y + rise * along / std::max(1e-9, lengthbefore), planar.y};
			};
		}
		std::vector<std::pair<glm::dvec3, glm::dvec3>> ends;
		for (auto const &member : state.drag_lines)
			ends.emplace_back(transform(member.start), transform(member.end));
		state.preview_start = transform(grabbed.start);
		state.preview_end = transform(grabbed.end);
		if (state.fitting)
		{
			update_curve_fit(state.preview_start, state.preview_end);
			return;
		}
		for (auto const &entry : state.drag_states)
			editor_track::apply(*entry.first, entry.second);
		editor_track::move_straights(state.drag_lines, ends);
		m_track_dirty = true;
		commit_track_drag(false);
	}
	else if (state.dragging)
	{
		if (state.fitting)
		{
			state.fitting = false;
			state.dragging = false;
			state.drag_lines.clear();
			if (m_route.result.valid)
				route_apply();
			straight_refresh();
			return;
		}
		commit_track_drag(true);
		state.dragging = false;
		m_track_drag.clear();
		state.drag_states.clear();
		state.drag_lines.clear();
		straight_refresh();
	}
}

void editor_mode::straight_reshape(editor_track::straight const &Line, double const From, double const To, std::function<glm::dvec3(glm::dvec3 const &)> const &Tail, double const Radius)
{
	auto &state{m_straights};
	state.status.clear();
	struct span
	{
		TTrack *track;
		int path;
		double from, to;
		bool forward;
	};
	auto line{Line};
	auto const point = [&](double const Along) {
		auto const planar{plan_of(line.start) + line.direction * Along};
		return glm::dvec3{planar.x, line.start.y + line.grade * Along, planar.y};
	};
	std::vector<span> spans;
	auto const collect = [&]() {
		spans.clear();
		for (std::size_t i = 0; i < line.tracks.size(); ++i)
		{
			auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
			auto const a{line.along(path.points[segment_data::point::start])};
			auto const b{line.along(path.points[segment_data::point::end])};
			spans.push_back({line.tracks[i], line.paths[i], std::min(a, b), std::max(a, b), a < b});
		}
		std::sort(spans.begin(), spans.end(), [](span const &A, span const &B) { return A.from < B.from; });
	};
	int first{-1}, last{-1};
	auto const select = [&](double const End) {
		first = last = -1;
		for (int i = 0; i < static_cast<int>(spans.size()); ++i)
		{
			if (spans[i].to > From && spans[i].from < End)
			{
				if (first < 0)
					first = i;
				last = i;
			}
		}
	};
	auto const keep_regular = [&]() {
		if (first < 0)
			return;
		auto const centre{(std::max(From, 0.0) + std::min(To, line.length)) * 0.5};
		int middle{first};
		for (int i = first; i <= last; ++i)
			if (spans[i].from <= centre && spans[i].to >= centre)
				middle = i;
		if (spans[middle].track->eType != tt_Normal)
		{
			int best{-1};
			for (int i = first; i <= last; ++i)
				if (spans[i].track->eType == tt_Normal && (best < 0 || std::abs(i - middle) < std::abs(best - middle)))
					best = i;
			if (best < 0)
			{
				first = last = -1;
				return;
			}
			middle = best;
		}
		int low{middle}, high{middle};
		while (low > first && spans[low - 1].track->eType == tt_Normal)
			--low;
		while (high < last && spans[high + 1].track->eType == tt_Normal)
			++high;
		first = low;
		last = high;
	};
	collect();
	select(To);
	keep_regular();
	if (first < 0)
		return;

	std::vector<std::pair<TTrack *, editor_track::state>> splitstates;
	std::vector<TTrack *> splitcreated;
	auto end{To};
	if (last + 1 >= static_cast<int>(spans.size()))
	{
		auto const final{spans.back()};
		if (final.track->eType != tt_Normal)
			return;
		auto const length{final.to - final.from};
		auto const cut{To < final.to - 1.0 ? std::max(To, final.from + std::min(1.0, length * 0.5)) : final.from + length * 0.7};
		splitstates.emplace_back(final.track, editor_track::capture(*final.track));
		editor_track::chain split;
		split.tracks = {final.track};
		split.forward = {final.forward};
		std::vector<segment_data> pieces(2);
		pieces[0].points[segment_data::point::start] = point(final.from);
		pieces[0].points[segment_data::point::end] = point(cut);
		pieces[1].points[segment_data::point::start] = point(cut);
		pieces[1].points[segment_data::point::end] = point(final.to);
		splitcreated = editor_track::relay(split, pieces);
		line = editor_track::find_straight(*line.tracks.front(), state.tolerance);
		collect();
		end = std::min(To, cut);
		select(end);
		keep_regular();
		if (first < 0 || last + 1 >= static_cast<int>(spans.size()))
		{
			for (auto *track : splitcreated)
				editor_track::retire(*track);
			editor_track::apply(*splitstates.front().first, splitstates.front().second);
			editor_track::commit({splitstates.front().first});
			return;
		}
	}
	auto const undo_split = [&]() {
		for (auto *track : splitcreated)
			editor_track::retire(*track);
		for (auto const &entry : splitstates)
		{
			editor_track::apply(*entry.first, entry.second);
			editor_track::commit({entry.first});
		}
	};

	editor_track::straight tail;
	tail.direction = line.direction;
	for (int i = last + 1; i < static_cast<int>(spans.size()); ++i)
	{
		tail.tracks.push_back(spans[i].track);
		tail.paths.push_back(spans[i].path);
	}
	tail.start = point(spans[last + 1].from);
	tail.end = line.end;
	tail.length = line.length - spans[last + 1].from;
	tail.grade = line.grade;

	std::vector<TTrack *> chainpaths;
	for (int i = first; i <= last; ++i)
		chainpaths.push_back(spans[i].track);
	auto tracks{editor_track::straight_affected({tail})};
	for (auto *track : chainpaths)
		if (std::find(tracks.begin(), tracks.end(), track) == tracks.end())
			tracks.push_back(track);
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : tracks)
		states.emplace_back(track, editor_track::capture(*track));

	editor_track::move_straights({tail}, {{Tail(tail.start), Tail(tail.end)}});

	m_route.from = spans[first].track;
	m_route.to = spans[last].track;
	route_reset();
	if (m_route.chain.tracks.empty())
	{
		for (auto const &entry : states)
			editor_track::apply(*entry.first, entry.second);
		editor_track::commit(tracks);
		undo_split();
		state.status = m_route.error;
		return;
	}
	m_route.design.start_reserve = m_route.design.end_reserve = 0.0;
	for (auto &vertex : m_route.design.vertices)
	{
		vertex.radius = Radius;
		route_recommend(vertex);
		if (false == state.auto_transitions)
		{
			vertex.transition_in = state.transition;
			vertex.transition_out = state.transition;
		}
	}
	route_update();
	if (false == m_route.result.valid)
	{
		for (auto const &entry : states)
			editor_track::apply(*entry.first, entry.second);
		editor_track::commit(tracks);
		undo_split();
		state.status = m_route.result.errors.empty() ? "The curve can't be fitted" : m_route.result.errors.front();
		return;
	}
	auto pieces{alignment::pieces(m_route.result, m_route.design, m_route.chain.tracks.size())};
	editor_track::keep_heights(m_route.chain, pieces);
	auto created{editor_track::relay(m_route.chain, pieces)};
	editor_track::commit(tracks);
	for (auto const &entry : splitstates)
	{
		auto const found{std::find_if(states.begin(), states.end(), [&](auto const &State) { return State.first == entry.first; })};
		if (found != states.end())
			found->second = entry.second;
		else
			states.push_back(entry);
	}
	created.insert(created.begin(), splitcreated.begin(), splitcreated.end());
	push_track_snapshot(std::move(states), std::move(created));
	std::string error;
	editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error);
	straight_refresh();
	state.status = "Done, the curves can be adjusted in the Route design tab";
}

double editor_mode::straight_tool_radius(editor_track::straight const &Line) const
{
	if (m_straights.curve_radius > 0.0)
		return m_straights.curve_radius;
	double speed{-1.0};
	for (auto const *track : Line.tracks)
		speed = std::max(speed, editor_track::velocity(*track));
	if (speed <= 0.0)
		speed = 100.0;
	auto const recommended{alignment::recommend(speed, 1000.0, m_route.design.norms)};
	return std::max(150.0, std::ceil(recommended.radius_min * 1.5 / 50.0) * 50.0);
}

bool editor_mode::place_straight_tool()
{
	auto &state{m_straights};
	if (ui()->mode() != nodebank_panel::TRACK || m_track_tab != track_tab::straights || state.tool == 0)
		return false;
	if (state.tool == 2)
	{
		add_detour_point();
		return true;
	}
	auto const &line{current_straight()};
	if (line.tracks.empty())
		return false;
	glm::dvec3 const ground{Global.pCamera.Pos + GfxRenderer->Mouse_Position()};
	auto const along{std::clamp(line.along(ground), 0.0, line.length)};
	glm::dvec3 const axis{line.direction.x, line.grade, line.direction.y};
	state.tool_line = line;
	state.tool_at = along;
	state.tool_point = line.start + axis * along;
	auto const reach{std::min(50.0, std::max(1.0, line.length - along))};
	state.tool_handle = state.tool_point + axis * reach;
	state.tool_placed = true;
	state.tool_dragging = false;
	state.handle = -1;
	return true;
}

void editor_mode::render_straight_tool_gizmo()
{
	auto &state{m_straights};
	gizmo_frame const frame;
	if (false == state.tool_dragging)
		state.tool_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(state.tool_handle - frame.camera));
	frame.manipulate(ImGuizmo::TRANSLATE, state.tool_gizmo, 0.0f);
	if (ImGuizmo::IsUsing())
	{
		state.tool_dragging = true;
		auto const moved{frame.camera + glm::dvec3(state.tool_gizmo[3])};
		state.tool_handle = {moved.x, state.tool_handle.y, moved.z};
	}
	else if (state.tool_dragging)
	{
		state.tool_dragging = false;
		apply_straight_tool();
	}
}

void editor_mode::apply_straight_tool()
{
	auto &state{m_straights};
	auto const line{state.tool_line};
	auto const offset{plan_of(state.tool_handle - state.tool_point)};
	auto const radius{straight_tool_radius(line)};
	if (state.tool == 1)
	{
		if (glm::length(offset) < 1e-3)
			return;
		auto const unit{glm::normalize(offset)};
		auto const angle{std::atan2(line.direction.x * unit.y - line.direction.y * unit.x, glm::dot(line.direction, unit))};
		if (std::abs(angle) < 1e-5)
			return;
		glm::dvec2 const pivot{state.tool_point.x, state.tool_point.z};
		double speed{-1.0};
		for (auto const *track : line.tracks)
			speed = std::max(speed, editor_track::velocity(*track));
		auto const transition{state.auto_transitions ? alignment::recommend(speed > 0.0 ? speed : 100.0, radius, m_route.design.norms).transition : state.transition};
		auto const reach{radius * std::tan(std::min(std::abs(angle), 3.0) * 0.5) + transition * 0.5 + 10.0};
		straight_reshape(line, state.tool_at - reach, state.tool_at + reach,
			[pivot, angle](glm::dvec3 const &Point) {
				glm::dvec2 const relative{Point.x - pivot.x, Point.z - pivot.y};
				auto const c{std::cos(angle)};
				auto const s{std::sin(angle)};
				return glm::dvec3{pivot.x + relative.x * c - relative.y * s, Point.y, pivot.y + relative.x * s + relative.y * c};
			},
			radius);
	}
	else
	{
		glm::dvec2 const normal{-line.direction.y, line.direction.x};
		auto const shift{glm::dot(offset, normal)};
		auto const length{glm::dot(offset, line.direction)};
		if (std::abs(shift) < 1e-3 || length < 1.0)
			return;
		auto const translation{normal * shift};
		straight_reshape(line, state.tool_at, state.tool_at + length,
			[translation](glm::dvec3 const &Point) { return glm::dvec3{Point.x + translation.x, Point.y, Point.z + translation.y}; },
			radius);
	}
	state.tool_placed = false;
}

void editor_mode::start_straight_gesture(int const Tool)
{
	auto &state{m_straights};
	auto const previous{state.tool};
	state.tool = Tool;
	if (false == place_straight_tool())
	{
		state.tool = previous;
		return;
	}
	state.tool_mouse = true;
}

void editor_mode::finish_straight_gesture()
{
	auto &state{m_straights};
	state.tool_mouse = false;
	apply_straight_tool();
	state.tool = 0;
	state.tool_placed = false;
}

void editor_mode::toggle_straight_set(TTrack &Track)
{
	auto &state{m_straights};
	auto const line{editor_track::find_straight(Track, state.tolerance)};
	if (line.tracks.empty())
		return;
	if (in_straight_set(line))
		state.set.erase(std::remove_if(state.set.begin(), state.set.end(), [&](editor_track::straight const &Member) { return std::any_of(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Other) { return std::find(Member.tracks.begin(), Member.tracks.end(), Other) != Member.tracks.end(); }); }), state.set.end());
	else
		state.set.push_back(line);
	m_node = &Track;
	ui()->set_node(m_node);
	m_track_tab = track_tab::straights;
}

void editor_mode::delete_selected_track()
{
	auto *track{selected_track()};
	if (track == nullptr || false == track->Dynamics.empty())
		return;
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::TrackEdit;
	snap.node_name = track->name();
	snap.position = track->location();
	snap.removed = {track};
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
	editor_track::retire(*track);
	if (std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) != m_route.chain.tracks.end())
		m_route = {};
	m_node = nullptr;
	ui()->set_node(nullptr);
	m_track_point = {};
	straight_refresh();
}

void editor_mode::draw_track_hints()
{
	std::string hint;
	if (false == m_straights.detour.empty())
		hint = "Shift+click: point " + std::to_string(m_straights.detour.size() + 1) + " of 4 (1: leave the axis, 2: shifted, 3: start back, 4: back on the axis)   Esc: cancel";
	else if (m_extend.active)
		hint = "Drag: curve through the cursor, along the end: straight   Ctrl: straight";
	else if (m_switch.placing)
		hint = "Drag along the straight to set the direction of the switch, sideways to set its side";
	else if (m_switch.armed >= 0 && m_track_tab == track_tab::straights)
		hint = "Press on the selected straight where the switch starts and drag";
	else if (m_track_tab == track_tab::straights)
		hint = current_straight().tracks.empty() ? "LMB: select a straight or a curve" : "Drag the ends or the middle   Ctrl+drag: break   Shift+click x4: shift around   Alt+click: add to the set   Drag from a red ring: new path   Del: delete";
	else if (m_track_tab == track_tab::turnout)
		hint = "Gizmo: move the switch (W: turn)   Drag from a red ring: new path   Del: delete";
	else if (m_track_tab == track_tab::route)
		hint = m_route.chain.tracks.empty() ? "LMB: select a curve" : "Drag the vertex (yellow) or the radius grip (green), the change is applied on release   Ctrl+Z: undo";
	else
		hint = "LMB: select a path or a point handle";
	hint += "   K: split the path under the cursor   O: top view";
	ImGuiIO const &io = ImGui::GetIO();
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	ImVec2 const position{12.0f, io.DisplaySize.y - 28.0f};
	auto const size{ImGui::CalcTextSize(hint.c_str())};
	drawlist->AddRectFilled(ImVec2(position.x - 6.0f, position.y - 4.0f), ImVec2(position.x + size.x + 6.0f, position.y + size.y + 4.0f), IM_COL32(0, 0, 0, 150), 4.0f);
	drawlist->AddText(position, IM_COL32(255, 255, 255, 230), hint.c_str());
}

void editor_mode::update_build_tools()
{
	glm::dvec3 const ground{Global.pCamera.Pos + GfxRenderer->Mouse_Position()};
	m_straights.detour_mouse = ground;
	if (m_extend.active)
		m_extend.mouse = ground;
	if (m_switch.placing)
		m_switch.mouse = ground;
}

bool editor_mode::start_extend()
{
	auto *track{selected_track()};
	if (track == nullptr || (track->eType != tt_Normal && track->eType != tt_Switch) || false == editor_track::is_supported(*track))
		return false;
	screen_projection const projection;
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		auto const &path{track->m_paths[i]};
		for (auto const atend : {false, true})
		{
			editor_track::point_ref const point{i, atend ? editor_track::point_kind::end : editor_track::point_kind::start};
			if (editor_track::is_connected(*track, point))
				continue;
			auto const position{editor_track::point_position(*track, point)};
			if (projection.mouse_distance2(position) > kHandleRadius * kHandleRadius * 1.5f)
				continue;
			auto const &start{path.points[segment_data::point::start]};
			auto const &end{path.points[segment_data::point::end]};
			auto const &control1{path.points[segment_data::point::control1]};
			auto const &control2{path.points[segment_data::point::control2]};
			glm::dvec3 tangent;
			if (atend)
				tangent = control2 != glm::dvec3{} ? -control2 : end - (start + control1);
			else
				tangent = -(control1 != glm::dvec3{} ? control1 : (end + control2) - start);
			glm::dvec2 const planar{tangent.x, tangent.z};
			if (glm::length(planar) < 1e-6)
				return false;
			m_extend.active = true;
			m_extend.path = i;
			m_extend.atend = atend;
			m_extend.track = track;
			m_extend.point = position;
			m_extend.direction = glm::normalize(planar);
			m_extend.grade = tangent.y / glm::length(planar);
			m_extend.mouse = position;
			return true;
		}
	}
	return false;
}

bool editor_mode::extend_snap(editor_track::snap_target &Target) const
{
	auto const &tool{m_extend};
	if (false == tool.active || tool.track == nullptr)
		return false;
	Target = editor_track::find_snap_target(*tool.track, tool.mouse, 5.0, {tool.track});
	return Target.track != nullptr && glm::distance(Target.position, tool.point) > 0.5;
}

std::vector<segment_data> editor_mode::extend_pieces() const
{
	std::vector<segment_data> result;
	auto const &tool{m_extend};
	editor_track::snap_target target;
	if (extend_snap(target))
	{
		alignment::design design;
		design.norms = m_route.design.norms;
		design.shape = m_route.design.shape;
		design.arc_piece_angle = 90.0;
		design.start = tool.point;
		design.end = target.position;
		design.start_direction = tool.direction;
		design.end_direction = glm::normalize(plan_of(target.direction));
		auto const speed{editor_track::velocity(*tool.track)};
		design.speed = speed > 0.0 ? speed : 100.0;
		if (false == alignment::collinear(design))
		{
			alignment::vertex vertex;
			vertex.radius = 100000.0;
			auto const recommended{alignment::recommend(design.speed, 1000.0, design.norms)};
			vertex.transition_in = vertex.transition_out = recommended.transition;
			glm::dvec2 intersection;
			if (alignment::tangent_intersection(design, intersection))
			{
				design.vertices.push_back(vertex);
			}
			else
			{
				vertex.offset = plan_distance(design.start, design.end) / 3.0;
				design.vertices = {vertex, vertex};
			}
		}
		auto const shape{alignment::compute(design)};
		if (shape.valid)
			return alignment::pieces(shape, design, 0);
	}
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const chord{glm::length(offset)};
	if (chord < 0.5)
		return result;
	auto const &direction{tool.direction};
	auto const along{glm::dot(offset, direction)};
	auto const lateral{direction.x * offset.y - direction.y * offset.x};
	auto const height = [&](double const Distance) { return tool.point.y + tool.grade * Distance; };
	if (ImGui::GetIO().KeyCtrl || std::abs(lateral) < 0.005 * chord + 0.05)
	{
		if (along < 0.5)
			return result;
		segment_data path;
		path.points[segment_data::point::start] = tool.point;
		path.points[segment_data::point::end] = {tool.point.x + direction.x * along, height(along), tool.point.z + direction.y * along};
		result.push_back(path);
		return result;
	}
	auto const angle{2.0 * std::atan2(std::abs(lateral), along)};
	auto const radius{chord / (2.0 * std::sin(angle * 0.5))};
	double const side{lateral > 0.0 ? 1.0 : -1.0};
	glm::dvec2 const centre{plan_of(tool.point) + glm::dvec2{-direction.y, direction.x} * (side * radius)};
	auto const count{std::max(1, static_cast<int>(std::ceil(angle / glm::radians(90.0) - 1e-9)))};
	auto const step{angle / count};
	auto const handle{4.0 / 3.0 * std::tan(step / 4.0) * radius};
	glm::dvec2 const radial{plan_of(tool.point) - centre};
	for (int i = 0; i < count; ++i)
	{
		auto const a0{side * step * i};
		auto const a1{side * step * (i + 1)};
		auto const p0{centre + turned(radial, a0)};
		auto const p3{centre + turned(radial, a1)};
		auto const d0{turned(direction, a0)};
		auto const d3{turned(direction, a1)};
		auto const y0{height(radius * step * i)};
		auto const y3{height(radius * step * (i + 1))};
		segment_data path;
		path.points[segment_data::point::start] = {p0.x, y0, p0.y};
		path.points[segment_data::point::end] = {p3.x, y3, p3.y};
		path.points[segment_data::point::control1] = {d0.x * handle, tool.grade * handle, d0.y * handle};
		path.points[segment_data::point::control2] = {-d3.x * handle, -tool.grade * handle, -d3.y * handle};
		path.radius = static_cast<float>(radius);
		result.push_back(path);
	}
	return result;
}

void editor_mode::finish_extend()
{
	crossover_plan plan;
	editor_track::snap_target target;
	if (false == extend_snap(target) && plan_crossover(plan))
	{
		auto *origin{m_extend.track};
		m_extend.active = false;
		std::vector<std::pair<TTrack *, editor_track::state>> states;
		std::vector<TTrack *> created;
		std::vector<TTrack *> removed;
		if (false == place_switch_on_straight(plan.target, plan.shape, origin, plan.along, plan.direction, plan.side, false, states, created, removed))
			return;
		if (glm::distance(plan.insert.points[segment_data::point::start], plan.insert.points[segment_data::point::end]) > 0.01)
		{
			auto *insert{editor_track::create_path(*origin, plan.insert)};
			editor_track::commit({insert});
			created.push_back(insert);
		}
		push_track_snapshot(std::move(states), std::move(created), std::move(removed));
		straight_refresh();
		return;
	}
	auto const pieces{extend_pieces()};
	auto *style{m_extend.track};
	m_extend.active = false;
	if (pieces.empty() || style == nullptr)
		return;
	std::vector<TTrack *> created;
	for (auto const &piece : pieces)
		created.push_back(editor_track::create_path(*style, piece));
	editor_track::commit(created);
	push_track_snapshot({}, created);
	m_node = created.back();
	ui()->set_node(m_node);
	straight_refresh();
}

std::vector<segment_data> editor_mode::switch_preview() const
{
	auto const &tool{m_switch};
	if (false == tool.placing || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return {};
	if (tool.curved)
	{
		auto const sample{sample_at(tool.frame, tool.along)};
		glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
		auto const forward{glm::dot(offset, sample.tangent) >= 0.0 ? 1 : -1};
		glm::dvec2 const direction{sample.tangent * static_cast<double>(forward)};
		auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
		return curved_switch_paths(tool.templates[tool.armed], tool.frame, tool.along, forward, side);
	}
	auto const &line{tool.line};
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const forward{glm::dot(offset, line.direction) >= 0.0 ? 1 : -1};
	glm::dvec2 const direction{line.direction * static_cast<double>(forward)};
	auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
	return editor_track::place_switch(tool.templates[tool.armed], tool.point, direction, side, line.grade * forward);
}

bool editor_mode::start_switch_placement()
{
	auto &tool{m_switch};
	if (tool.armed < 0 || (m_track_tab != track_tab::straights && m_track_tab != track_tab::route))
		return false;
	glm::dvec3 const ground{Global.pCamera.Pos + GfxRenderer->Mouse_Position()};
	tool.curved = false;
	if (m_track_tab == track_tab::route || current_straight().tracks.empty())
	{
		auto *track{selected_track()};
		if (track == nullptr || track->eType != tt_Normal || tool.templates[tool.armed].double_slip)
			return false;
		editor_track::curve curve;
		editor_track::chain chain;
		std::string error;
		if (false == editor_track::find_curve(*track, m_straights.tolerance, m_route.design.norms.gauge, curve) || false == editor_track::find_chain(curve.from, curve.to, chain, error))
			return false;
		tool.frame = sample_chain(chain);
		tool.frame_tracks = chain.tracks;
		if (tool.frame.size() < 2)
			return false;
		tool.curved = true;
		tool.along = nearest_station(tool.frame, ground);
		tool.point = sample_at(tool.frame, tool.along).position;
		tool.mouse = ground;
		tool.placing = true;
		return true;
	}
	auto const &line{current_straight()};
	if (line.tracks.empty())
		return false;
	if (tool.templates[tool.armed].double_slip)
	{
		insert_double_slip(line, ground, tool.templates[tool.armed]);
		return true;
	}
	tool.along = std::clamp(line.along(ground), 0.0, line.length);
	tool.point = line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * tool.along;
	tool.mouse = ground;
	tool.line = line;
	tool.placing = true;
	return true;
}

void editor_mode::finish_switch_placement()
{
	auto &tool{m_switch};
	tool.placing = false;
	if (tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return;
	if (tool.curved)
	{
		auto const sample{sample_at(tool.frame, tool.along)};
		glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
		auto const forward{glm::dot(offset, sample.tangent) >= 0.0 ? 1 : -1};
		glm::dvec2 const direction{sample.tangent * static_cast<double>(forward)};
		auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
		insert_curved_switch(forward, side);
		return;
	}
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const forward{glm::dot(offset, tool.line.direction) >= 0.0 ? 1 : -1};
	glm::dvec2 const direction{tool.line.direction * static_cast<double>(forward)};
	auto const side{direction.x * offset.y - direction.y * offset.x >= 0.0 ? 1 : -1};
	insert_switch(tool.line, tool.along, forward, side);
}

void editor_mode::insert_switch(editor_track::straight const &Line, double const Along, int const Direction, int const Side)
{
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	if (false == place_switch_on_straight(Line, m_switch.templates[m_switch.armed], nullptr, Along, Direction, Side, true, states, created, removed))
		return;
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
}

bool editor_mode::cut_straight(editor_track::straight const &Line, double const From, double const To, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack **Style)
{
	auto const from{From};
	auto const to{To};
	struct span
	{
		TTrack *track;
		int path;
		double from, to;
		bool forward;
	};
	std::vector<span> spans;
	for (std::size_t i = 0; i < Line.tracks.size(); ++i)
	{
		auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
		auto const a{Line.along(path.points[segment_data::point::start])};
		auto const b{Line.along(path.points[segment_data::point::end])};
		spans.push_back({Line.tracks[i], Line.paths[i], std::min(a, b), std::max(a, b), a < b});
	}
	std::sort(spans.begin(), spans.end(), [](span const &A, span const &B) { return A.from < B.from; });
	int first{-1}, last{-1};
	for (int i = 0; i < static_cast<int>(spans.size()); ++i)
	{
		if (spans[i].to > from + 1e-3 && spans[i].from < to - 1e-3)
		{
			if (first < 0)
				first = i;
			last = i;
		}
	}
	if (first < 0)
		return false;
	editor_track::chain chain;
	for (int i = first; i <= last; ++i)
	{
		if (spans[i].track->eType != tt_Normal)
			return false;
		chain.tracks.push_back(spans[i].track);
		chain.forward.push_back(spans[i].forward);
	}
	if (Style != nullptr)
		*Style = spans[first].track;
	auto const before{std::max(0.0, from - spans[first].from)};
	auto const after{std::max(0.0, spans[last].to - to)};
	bool const hasbefore{before > 0.01};
	bool const hasafter{after > 0.01};
	auto const point = [&](double const Distance) { return Line.start + glm::dvec3{Line.direction.x, Line.grade, Line.direction.y} * Distance; };
	if (false == hasbefore && false == hasafter)
	{
		for (auto *track : chain.tracks)
			editor_track::retire(*track);
		Removed.insert(Removed.end(), chain.tracks.begin(), chain.tracks.end());
		return true;
	}
	auto const count{std::max<std::size_t>(chain.tracks.size(), (hasbefore ? 1 : 0) + (hasafter ? 1 : 0))};
	std::size_t countbefore{0};
	if (hasbefore && hasafter)
		countbefore = std::clamp<std::size_t>(static_cast<std::size_t>(std::round(count * before / (before + after))), 1, count - 1);
	else if (hasbefore)
		countbefore = count;
	std::vector<segment_data> pieces;
	auto const straight = [&](double const Start, double const End, std::size_t const Count) {
		for (std::size_t i = 0; i < Count; ++i)
		{
			segment_data path;
			path.points[segment_data::point::start] = point(Start + (End - Start) * i / Count);
			path.points[segment_data::point::end] = point(Start + (End - Start) * (i + 1) / Count);
			pieces.push_back(path);
		}
	};
	if (hasbefore)
		straight(spans[first].from, from, countbefore);
	if (hasafter)
		straight(to, spans[last].to, count - countbefore);
	for (auto *track : chain.tracks)
		States.emplace_back(track, editor_track::capture(*track));
	auto const relaid{editor_track::relay(chain, pieces)};
	Created.insert(Created.end(), relaid.begin(), relaid.end());
	return true;
}

bool editor_mode::place_switch_on_straight(editor_track::straight const &Line, editor_track::switch_template const &Shape, TTrack const *Style, double const Along, int const Direction, int const Side, bool const Snap, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed)
{
	auto const &shape{Shape};
	if (shape.length > Line.length - 0.02)
		return false;
	auto from{std::clamp(Direction > 0 ? Along : Along - shape.length, 0.0, Line.length - shape.length)};
	if (Snap)
	{
		std::vector<double> joints{0.0, Line.length};
		for (std::size_t i = 0; i < Line.tracks.size(); ++i)
		{
			auto const &path{Line.tracks[i]->m_paths[Line.paths[i]]};
			for (auto const index : {segment_data::point::start, segment_data::point::end})
				joints.push_back(Line.along(path.points[index]));
		}
		auto const nearest = [&](double const Value) {
			double best{Value};
			for (auto const joint : joints)
				if (std::abs(joint - Value) < std::abs(best - Value) || best == Value)
					best = std::abs(joint - Value) < 1.0 ? joint : best;
			return best;
		};
		auto const snappedstart{nearest(from)};
		auto const snappedend{nearest(from + shape.length)};
		if (snappedstart != from)
			from = snappedstart;
		else if (snappedend != from + shape.length)
			from = snappedend - shape.length;
		from = std::clamp(from, 0.0, Line.length - shape.length);
	}
	auto const to{from + shape.length};
	auto const origin{Direction > 0 ? from : to};
	TTrack *first{nullptr};
	if (false == cut_straight(Line, from, to, States, Created, Removed, &first))
		return false;
	auto const point = [&](double const Distance) { return Line.start + glm::dvec3{Line.direction.x, Line.grade, Line.direction.y} * Distance; };
	glm::dvec2 const direction{Line.direction * static_cast<double>(Direction)};
	auto const paths{editor_track::place_switch(shape, point(origin), direction, Side, Line.grade * Direction)};
	auto *track{editor_track::create_switch(shape, paths, Style != nullptr ? *Style : *first)};
	if (track != nullptr)
	{
		editor_track::commit({track});
		Created.push_back(track);
	}
	return true;
}

void editor_mode::draw_build_overlay() const
{
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList();
	auto const drawpath = [&](segment_data const &Path, ImU32 const Color) {
		auto previous{bezier{Path}.point(0.0)};
		for (int i = 1; i <= 24; ++i)
		{
			auto const next{bezier{Path}.point(i / 24.0)};
			projection.line(drawlist, previous, next, Color, 3.0f);
			previous = next;
		}
	};
	if (m_extend.active)
	{
		crossover_plan plan;
		editor_track::snap_target target;
		if (extend_snap(target))
		{
			ImVec2 screen;
			if (projection.project(target.position, screen))
				drawlist->AddCircle(screen, 14.0f, IM_COL32(255, 60, 255, 255), 20, 3.0f);
			for (auto const &piece : extend_pieces())
				drawpath(piece, overlay_color::marked);
		}
		else if (plan_crossover(plan))
		{
			for (auto const &piece : plan.paths)
				drawpath(piece, overlay_color::marked);
			drawpath(plan.insert, overlay_color::grip);
		}
		else
		{
			for (auto const &piece : extend_pieces())
				drawpath(piece, overlay_color::marked);
		}
	}
	if (m_switch.placing)
		for (auto const &piece : switch_preview())
			drawpath(piece, overlay_color::marked);
	if (false == m_straights.detour.empty())
	{
		auto const outline{detour_outline()};
		for (std::size_t i = 0; i + 1 < outline.size(); ++i)
			projection.line(drawlist, outline[i], outline[i + 1], overlay_color::marked, 3.0f);
		for (std::size_t i = 0; i < outline.size() && i < m_straights.detour.size(); ++i)
		{
			ImVec2 screen;
			if (projection.project(outline[i], screen))
				drawlist->AddCircleFilled(screen, 6.0f, overlay_color::invalid);
		}
	}
	auto const *track{selected_track()};
	if (track == nullptr || (track->eType != tt_Normal && track->eType != tt_Switch) || m_extend.active)
		return;
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
	{
		for (auto const atend : {false, true})
		{
			editor_track::point_ref const point{i, atend ? editor_track::point_kind::end : editor_track::point_kind::start};
			if (editor_track::is_connected(*track, point))
				continue;
			ImVec2 screen;
			if (projection.project(editor_track::point_position(*track, point), screen))
				drawlist->AddCircle(screen, 10.0f, IM_COL32(240, 60, 60, 255), 16, 3.0f);
		}
	}
}

void editor_mode::render_switch_ui()
{
	auto &tool{m_switch};
	if (false == ImGui::CollapsingHeader("Switches", ImGuiTreeNodeFlags_DefaultOpen))
		return;
	if (false == tool.collected)
	{
		tool.templates = editor_track::standard_switch_templates();
		auto const found{editor_track::find_switch_templates()};
		tool.templates.insert(tool.templates.end(), found.begin(), found.end());
		tool.collected = true;
	}
	ImGui::TextDisabled(tool.armed >= 0 ? "Press on the straight where the switch starts and drag: along sets its direction, sideways its side" : "Click a template, then drag on the selected straight");
	ImGui::BeginChild("##switchtemplates", ImVec2(0.0f, 150.0f), true);
	for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
	{
		auto const &entry{tool.templates[i]};
		auto label{entry.source != nullptr ? "Scenery: " + entry.label + "  (" + std::to_string(entry.count) + "x)" : "PLK: " + entry.label};
		label += "##switch" + std::to_string(i);
		if (ImGui::Selectable(label.c_str(), tool.armed == i))
			tool.armed = (tool.armed == i ? -1 : i);
	}
	ImGui::EndChild();
	if (ImGui::SmallButton("Collect again from the scenery"))
	{
		tool.collected = false;
		tool.armed = -1;
	}
}

std::vector<glm::dvec3> editor_mode::detour_outline() const
{
	auto const &state{m_straights};
	std::vector<glm::dvec3> result;
	if (state.detour.empty())
		return result;
	auto const &line{state.detour_line};
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	auto points{state.detour};
	if (points.size() < 4)
		points.push_back(state.detour_mouse);
	auto const offset{points.size() >= 2 ? glm::dot(plan_of(points[1] - line.start), normal) : 0.0};
	auto const place = [&](double const Along, double const Lateral) {
		auto const planar{plan_of(line.start) + line.direction * Along + normal * Lateral};
		return glm::dvec3{planar.x, line.start.y + line.grade * Along, planar.y};
	};
	for (std::size_t i = 0; i < points.size(); ++i)
		result.push_back(place(line.along(points[i]), (i == 1 || i == 2) ? offset : 0.0));
	return result;
}

void editor_mode::add_detour_point()
{
	auto &state{m_straights};
	if (state.detour.empty())
	{
		auto const &line{current_straight()};
		if (line.tracks.empty())
			return;
		state.detour_line = line;
	}
	state.detour.push_back(Global.pCamera.Pos + GfxRenderer->Mouse_Position());
	if (state.detour.size() >= 4)
	{
		apply_detour();
		state.detour.clear();
	}
}

void editor_mode::apply_detour()
{
	auto &state{m_straights};
	auto line{editor_track::find_straight(*state.detour_line.tracks.front(), state.tolerance)};
	if (line.tracks.empty())
		return;
	glm::dvec2 const normal{-line.direction.y, line.direction.x};
	auto const offset{glm::dot(plan_of(state.detour[1] - line.start), normal)};
	std::array<double, 4> stations{line.along(state.detour[0]), line.along(state.detour[1]), line.along(state.detour[2]), line.along(state.detour[3])};
	if (stations[0] > stations[3])
	{
		std::swap(stations[0], stations[3]);
		std::swap(stations[1], stations[2]);
	}
	for (auto &station : stations)
		station = std::clamp(station, 0.0, line.length);
	stations[1] = std::clamp(stations[1], stations[0] + 1.0, stations[3] - 1.0);
	stations[2] = std::clamp(stations[2], stations[1], stations[3] - 1.0);
	if (std::abs(offset) < 0.01 || stations[3] - stations[0] < 3.0)
		return;

	auto const radius{straight_tool_radius(line)};
	auto const slope{std::atan(std::abs(offset) / std::max(1.0, std::min(stations[1] - stations[0], stations[3] - stations[2])))};
	double speed{-1.0};
	for (auto const *track : line.tracks)
		speed = std::max(speed, editor_track::velocity(*track));
	auto const transition{state.auto_transitions ? alignment::recommend(speed > 0.0 ? speed : 100.0, radius, m_route.design.norms).transition : state.transition};
	auto const reach{radius * std::tan(slope * 0.5) + transition * 0.5 + 5.0};
	auto const from{std::max(0.0, stations[0] - reach)};
	auto const to{std::min(line.length, stations[3] + reach)};

	struct span
	{
		TTrack *track;
		double from, to;
	};
	std::vector<span> spans;
	for (std::size_t i = 0; i < line.tracks.size(); ++i)
	{
		auto const &path{line.tracks[i]->m_paths[line.paths[i]]};
		auto const a{line.along(path.points[segment_data::point::start])};
		auto const b{line.along(path.points[segment_data::point::end])};
		spans.push_back({line.tracks[i], std::min(a, b), std::max(a, b)});
	}
	std::sort(spans.begin(), spans.end(), [](span const &A, span const &B) { return A.from < B.from; });
	int first{-1}, last{-1};
	for (int i = 0; i < static_cast<int>(spans.size()); ++i)
	{
		if (spans[i].to > from && spans[i].from < to && spans[i].track->eType == tt_Normal)
		{
			if (first < 0)
				first = i;
			last = i;
		}
		else if (first >= 0 && spans[i].from < to)
		{
			if (spans[i].from > stations[3])
				break;
			first = last = -1;
		}
	}
	if (first < 0)
		return;

	m_route.from = spans[first].track;
	m_route.to = spans[last].track;
	route_reset();
	if (m_route.chain.tracks.empty())
		return;
	auto &design{m_route.design};
	auto const startalong{line.along(design.start)};
	auto const endalong{line.along(design.end)};
	auto const place = [&](double const Along, double const Lateral) {
		auto const planar{plan_of(line.start) + line.direction * Along + normal * Lateral};
		return planar;
	};
	alignment::vertex vertex;
	vertex.radius = radius;
	route_recommend(vertex);
	if (false == state.auto_transitions)
		vertex.transition_in = vertex.transition_out = state.transition;
	design.vertices.clear();
	auto first_vertex{vertex};
	first_vertex.offset = std::max(0.1, stations[0] - startalong);
	design.vertices.push_back(first_vertex);
	if (stations[2] - stations[1] < 1.0)
	{
		auto middle{vertex};
		middle.position = place((stations[1] + stations[2]) * 0.5, offset);
		design.vertices.push_back(middle);
	}
	else
	{
		auto second{vertex};
		second.position = place(stations[1], offset);
		auto third{vertex};
		third.position = place(stations[2], offset);
		design.vertices.push_back(second);
		design.vertices.push_back(third);
	}
	auto last_vertex{vertex};
	last_vertex.offset = std::max(0.1, endalong - stations[3]);
	design.vertices.push_back(last_vertex);
	route_update();
	if (false == m_route.result.valid)
		return;
	route_apply();
	straight_refresh();
}

bool editor_mode::start_curve_fit(editor_track::straight const &Line)
{
	auto &state{m_straights};
	auto *before{editor_track::outside_neighbour(Line, false)};
	auto *after{editor_track::outside_neighbour(Line, true)};
	if (before == nullptr || after == nullptr)
		return false;
	if (false == editor_track::find_curve(*before, state.tolerance, m_route.design.norms.gauge, state.fit_before) || false == editor_track::find_curve(*after, state.tolerance, m_route.design.norms.gauge, state.fit_after))
		return false;
	if (state.fit_before.reversals != 0 || state.fit_after.reversals != 0)
		return false;
	auto const far = [](editor_track::curve const &Curve, glm::dvec3 const &Joint) { return editor_track::touches(*Curve.from, Joint) ? Curve.to : Curve.from; };
	m_route.from = far(state.fit_before, Line.start);
	m_route.to = far(state.fit_after, Line.end);
	route_reset();
	if (m_route.chain.tracks.empty())
		return false;
	return true;
}

void editor_mode::update_curve_fit(glm::dvec3 const &Start, glm::dvec3 const &End)
{
	auto &state{m_straights};
	auto &design{m_route.design};
	glm::dvec2 const point{Start.x, Start.z};
	glm::dvec2 const direction{glm::normalize(plan_of(End - Start))};
	glm::dvec2 const start{design.start.x, design.start.z};
	glm::dvec2 const end{design.end.x, design.end.z};
	auto const startdirection{glm::normalize(design.start_direction)};
	auto const enddirection{glm::normalize(design.end_direction)};
	auto const first{cross(point - start, direction) / cross(startdirection, direction)};
	auto const last{cross(end - point, direction) / cross(enddirection, direction)};
	auto const make = [&](editor_track::curve const &Curve, double const Offset) {
		alignment::vertex vertex;
		vertex.radius = Curve.radius > 0.0 ? std::round(Curve.radius) : 1000.0;
		route_recommend(vertex);
		if (Curve.transition_in > 0.0)
			vertex.transition_in = std::round(Curve.transition_in);
		if (Curve.transition_out > 0.0)
			vertex.transition_out = std::round(Curve.transition_out);
		if (Curve.cant > 0.0)
			vertex.cant = std::round(Curve.cant);
		if (Curve.compound)
		{
			vertex.compound = true;
			vertex.radius2 = std::round(Curve.radius2);
			vertex.transition_middle = std::round(Curve.transition_middle);
			vertex.split = Curve.split;
		}
		vertex.offset = std::max(0.1, Offset);
		return vertex;
	};
	auto before{make(state.fit_before, first)};
	auto after{make(state.fit_after, last)};
	design.vertices = {before, after};
	route_update();
}

bool editor_mode::plan_crossover(crossover_plan &Plan) const
{
	auto const &tool{m_extend};
	if (false == tool.active || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() < 2 || tool.path != 1 || false == tool.atend)
		return false;
	auto const &own{*tool.track};
	auto const &main{own.m_paths[0]};
	auto const &diverging{own.m_paths[1]};
	auto const origin{main.points[segment_data::point::start]};
	auto const axis{glm::normalize(plan_of(main.points[segment_data::point::end] - origin))};
	glm::dvec2 const perpendicular{-axis.y, axis.x};
	auto const &frog{diverging.points[segment_data::point::end]};
	auto const lateral{glm::dot(plan_of(frog - origin), perpendicular)};
	if (std::abs(lateral) < 0.01)
		return false;
	auto const side{lateral > 0.0 ? 1.0 : -1.0};
	auto const normal{perpendicular * side};
	auto const offset{std::abs(lateral)};
	auto const &control{diverging.points[segment_data::point::control2]};
	auto const tangent{glm::normalize(control != glm::dvec3{} ? -plan_of(control) : plan_of(frog - origin))};
	auto const angle{std::atan2(std::abs(cross(axis, tangent)), glm::dot(axis, tangent))};
	if (angle < 1e-4)
		return false;

	auto const ground{tool.mouse};
	TTrack *target{nullptr};
	double best{8.0};
	auto const sections{simulation::Region->sections(ground, 40.f)};
	for (auto *section : sections)
	{
		for (auto const &cell : section->m_cells)
		{
			for (auto *track : cell.m_paths)
			{
				if (track == &own || track->eType != tt_Normal || track->m_editorremoved || false == editor_track::is_straight(*track, m_straights.tolerance))
					continue;
				auto const &path{track->m_paths.front()};
				auto const a{plan_of(path.points[segment_data::point::start])};
				auto const b{plan_of(path.points[segment_data::point::end])};
				auto const segment{b - a};
				auto const t{std::clamp(glm::dot(plan_of(ground) - a, segment) / std::max(1e-9, glm::dot(segment, segment)), 0.0, 1.0)};
				auto const distance{glm::distance(plan_of(ground), a + segment * t)};
				if (distance < best)
				{
					best = distance;
					target = track;
				}
			}
		}
	}
	if (target == nullptr)
		return false;
	Plan.target = editor_track::find_straight(*target, m_straights.tolerance);
	auto const &line{Plan.target};
	if (line.tracks.empty() || std::abs(cross(line.direction, axis)) > std::sin(glm::radians(1.0)))
		return false;
	auto const distance{glm::dot(plan_of(line.start) - plan_of(origin), normal)};
	auto const remaining{distance - 2.0 * offset};
	if (remaining < 0.01)
		return false;
	auto const length{remaining / std::sin(angle)};
	glm::dvec2 const crossing{axis * std::cos(angle) + normal * std::sin(angle)};
	auto const alongfrog{glm::dot(plan_of(frog - origin), axis)};
	auto const otherfrog{plan_of(frog) + crossing * length};
	auto const otherorigin{otherfrog + axis * alongfrog + normal * offset};
	Plan.along = glm::dot(otherorigin - plan_of(line.start), line.direction);
	if (Plan.along < 0.0 || Plan.along > line.length)
		return false;
	Plan.direction = glm::dot(-axis, line.direction) > 0.0 ? 1 : -1;
	glm::dvec2 const direction{line.direction * static_cast<double>(Plan.direction)};
	Plan.side = glm::dot(glm::dvec2{-direction.y, direction.x}, -normal) > 0.0 ? 1 : -1;

	editor_track::switch_template shape;
	shape.source = tool.track;
	shape.length = glm::dot(plan_of(main.points[segment_data::point::end] - origin), axis);
	for (int i = 0; i < 2; ++i)
	{
		auto const &path{own.m_paths[i]};
		auto &local{shape.local[i]};
		local = path;
		auto const tolocal = [&](glm::dvec3 const &Vector) { return glm::dvec3{glm::dot(plan_of(Vector), perpendicular) * side, Vector.y, glm::dot(plan_of(Vector), axis)}; };
		local.points[segment_data::point::start] = tolocal(path.points[segment_data::point::start] - origin);
		local.points[segment_data::point::end] = tolocal(path.points[segment_data::point::end] - origin);
		local.points[segment_data::point::control1] = tolocal(path.points[segment_data::point::control1]);
		local.points[segment_data::point::control2] = tolocal(path.points[segment_data::point::control2]);
		if (side < 0.0)
			local.rolls = {-local.rolls[0], -local.rolls[1]};
	}
	Plan.shape = shape;
	auto const station{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * Plan.along};
	Plan.paths = editor_track::place_switch(shape, station, direction, Plan.side, line.grade * Plan.direction);
	Plan.insert = segment_data{};
	Plan.insert.points[segment_data::point::start] = frog;
	Plan.insert.points[segment_data::point::end] = Plan.paths[1].points[segment_data::point::end];
	return true;
}

std::vector<TTrack *> editor_mode::build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, double const Radius, TTrack &Style)
{
	auto const at = [](glm::dvec2 const &Planar, double const Y) { return glm::dvec3{Planar.x, Y, Planar.y}; };
	auto const straight = [](glm::dvec3 const &Start, glm::dvec3 const &End) {
		segment_data path;
		path.points[segment_data::point::start] = Start;
		path.points[segment_data::point::end] = End;
		return path;
	};
	double const gap{0.25};
	auto const side{cross(First, Second) > 0.0 ? 1.0 : -1.0};
	auto const arc = [&](glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Turn, glm::dvec3 const &Finish, double const From, double const To) {
		glm::dvec2 const centre{plan_of(Start) + glm::dvec2{-Heading.y, Heading.x} * (Turn * Radius)};
		auto const radial{plan_of(Start) - centre};
		auto const p0{centre + turned(radial, Turn * From)};
		auto const p3{centre + turned(radial, Turn * To)};
		auto const d0{turned(Heading, Turn * From)};
		auto const d3{turned(Heading, Turn * To)};
		auto const handle{4.0 / 3.0 * std::tan((To - From) / 4.0) * Radius};
		auto const y0{Start.y + (Finish.y - Start.y) * From / Angle};
		auto const y3{Start.y + (Finish.y - Start.y) * To / Angle};
		segment_data path;
		path.points[segment_data::point::start] = {p0.x, y0, p0.y};
		path.points[segment_data::point::end] = {p3.x, y3, p3.y};
		path.points[segment_data::point::control1] = {d0.x * handle, 0.0, d0.y * handle};
		path.points[segment_data::point::control2] = {-d3.x * handle, 0.0, -d3.y * handle};
		path.radius = static_cast<float>(Radius);
		return path;
	};
	auto const reversed = [](segment_data const &Path) {
		segment_data result{Path};
		result.points[segment_data::point::start] = Path.points[segment_data::point::end];
		result.points[segment_data::point::end] = Path.points[segment_data::point::start];
		result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
		result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
		result.rolls = {-Path.rolls[1], -Path.rolls[0]};
		return result;
	};
	auto const middle{Angle * 0.5};
	auto const split{gap / Radius};
	auto const centreA{at(Crossing - First * gap, Height)};
	auto const centreB{at(Crossing + First * gap, Height)};
	auto const centreC{at(Crossing - Second * gap, Height)};
	auto const centreD{at(Crossing + Second * gap, Height)};
	std::string base;
	for (int i = 1; base.empty() || simulation::Paths.find(base + "_a") != nullptr; ++i)
		base = "editor_dks" + std::to_string(i);
	editor_track::switch_template shape;
	shape.length = glm::distance(plan_of(PointA), Crossing);
	std::vector<TTrack *> parts;
	parts.push_back(editor_track::create_switch(shape, {straight(PointA, centreA), arc(PointA, First, side, PointD, 0.0, middle - split)}, Style, base + "_a"));
	parts.push_back(editor_track::create_switch(shape, {straight(PointB, centreB), reversed(arc(PointC, Second, -side, PointB, middle + split, Angle))}, Style, base + "_b"));
	parts.push_back(editor_track::create_switch(shape, {straight(PointC, centreC), arc(PointC, Second, -side, PointB, 0.0, middle - split)}, Style, base + "_c"));
	parts.push_back(editor_track::create_switch(shape, {straight(PointD, centreD), reversed(arc(PointA, First, side, PointD, middle + split, Angle))}, Style, base + "_d"));
	parts.push_back(editor_track::create_path(Style, straight(centreA, centreB)));
	parts.push_back(editor_track::create_path(Style, straight(centreC, centreD)));
	parts.push_back(editor_track::create_path(Style, arc(PointA, First, side, PointD, middle - split, middle + split)));
	parts.push_back(editor_track::create_path(Style, arc(PointC, Second, -side, PointB, middle - split, middle + split)));
	parts.erase(std::remove(parts.begin(), parts.end(), nullptr), parts.end());
	return parts;
}

bool editor_mode::insert_double_slip(editor_track::straight const &Line, glm::dvec3 const &Point, editor_track::switch_template const &Shape)
{
	auto &state{m_straights};
	state.status.clear();
	{
		auto const sections{simulation::Region->sections(Point, 40.f)};
		for (auto *section : sections)
			for (auto const &cell : section->m_cells)
				for (auto *track : cell.m_paths)
					if (track->eType == tt_Switch && false == track->m_editorremoved && track->DoubleSlip() && glm::distance(plan_of(track->location()), plan_of(Point)) < 25.0)
						return replace_double_slip(*track, Shape);
	}
	TTrack *other{nullptr};
	glm::dvec2 crossing{0.0};
	double best{40.0};
	auto const sections{simulation::Region->sections(Point, 60.f)};
	for (auto *section : sections)
	{
		for (auto const &cell : section->m_cells)
		{
			for (auto *track : cell.m_paths)
			{
				if (std::find(Line.tracks.begin(), Line.tracks.end(), track) != Line.tracks.end() || track->eType != tt_Normal || track->m_editorremoved || false == editor_track::is_straight(*track, m_straights.tolerance))
					continue;
				auto const &path{track->m_paths.front()};
				auto const a{plan_of(path.points[segment_data::point::start])};
				auto const b{plan_of(path.points[segment_data::point::end])};
				auto const segment{b - a};
				auto const determinant{cross(Line.direction, segment)};
				if (std::abs(determinant) < 1e-9)
					continue;
				auto const offset{a - plan_of(Line.start)};
				auto const along{cross(offset, segment) / determinant};
				auto const fraction{cross(offset, Line.direction) / determinant};
				if (fraction < -0.01 || fraction > 1.01 || along < 0.0 || along > Line.length)
					continue;
				auto const position{plan_of(Line.start) + Line.direction * along};
				auto const distance{glm::distance(position, plan_of(Point))};
				if (distance < best)
				{
					best = distance;
					other = track;
					crossing = position;
				}
			}
		}
	}
	if (other == nullptr)
	{
		state.status = "No straight crossing the selected one near the click";
		return false;
	}
	auto const second{editor_track::find_straight(*other, m_straights.tolerance)};
	if (second.tracks.empty())
		return false;
	auto const first{Line.direction};
	auto direction{second.direction};
	if (glm::dot(first, direction) < 0.0)
		direction = -direction;
	auto const angle{std::atan2(std::abs(cross(first, direction)), glm::dot(first, direction))};
	if (angle < glm::radians(0.5) || angle > glm::radians(30.0))
	{
		state.status = "The straights cross at an angle unsuitable for a double slip";
		return false;
	}
	auto const alongfirst{glm::dot(crossing - plan_of(Line.start), Line.direction)};
	auto const alongsecond{glm::dot(crossing - plan_of(second.start), second.direction)};
	auto const available{std::min({alongfirst, Line.length - alongfirst, alongsecond, second.length - alongsecond}) - 0.5};
	auto radius{Shape.radius};
	auto half{radius * std::tan(angle * 0.5)};
	if (half > available)
	{
		half = available;
		radius = half / std::tan(angle * 0.5);
		if (radius < 50.0)
		{
			state.status = "Not enough straight track around the crossing";
			return false;
		}
		state.status = "R reduced to " + std::to_string(static_cast<int>(radius)) + " m, the straights end " + std::to_string(static_cast<int>(available)) + " m from the crossing";
	}

	auto const heightfirst = [&](double const Along) { return Line.start.y + Line.grade * Along; };
	auto const heightsecond = [&](double const Along) { return second.start.y + second.grade * Along; };
	auto const sign{glm::dot(second.direction, direction) > 0.0 ? 1.0 : -1.0};
	auto const at = [&](glm::dvec2 const &Planar, double const Height) { return glm::dvec3{Planar.x, Height, Planar.y}; };
	auto const pointA{at(crossing - first * half, heightfirst(alongfirst - half))};
	auto const pointB{at(crossing + first * half, heightfirst(alongfirst + half))};
	auto const pointC{at(crossing - direction * half, heightsecond(alongsecond - sign * half))};
	auto const pointD{at(crossing + direction * half, heightsecond(alongsecond + sign * half))};

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	TTrack *style{nullptr};
	if (false == cut_straight(Line, alongfirst - half, alongfirst + half, states, created, removed, &style))
		return false;
	auto const secondline{editor_track::find_straight(*other, m_straights.tolerance)};
	auto const secondalong{glm::dot(crossing - plan_of(secondline.start), secondline.direction)};
	if (secondline.tracks.empty() || false == cut_straight(secondline, secondalong - half, secondalong + half, states, created, removed, nullptr))
	{
		for (auto *track : created)
			editor_track::retire(*track);
		for (auto const &entry : states)
		{
			editor_track::apply(*entry.first, entry.second);
			editor_track::commit({entry.first});
		}
		for (auto *track : removed)
		{
			track->m_editorremoved = false;
			editor_track::commit({track});
		}
		return false;
	}
	auto const parts{build_double_slip(pointA, pointB, pointC, pointD, crossing, heightfirst(alongfirst), first, direction, angle, radius, style != nullptr ? *style : *Line.tracks.front())};
	editor_track::commit(parts);
	created.insert(created.end(), parts.begin(), parts.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	return true;
}

bool editor_mode::replace_double_slip(TTrack &Part, editor_track::switch_template const &Shape)
{
	auto &state{m_straights};
	auto const base{Part.name().substr(0, Part.name().size() - 2)};
	std::array<TTrack *, 4> switches{};
	for (int i = 0; i < 4; ++i)
	{
		switches[i] = simulation::Paths.find(base + "_" + static_cast<char>('a' + i));
		if (switches[i] == nullptr || switches[i]->m_editorremoved || switches[i]->eType != tt_Switch || switches[i]->m_paths.size() < 2)
		{
			state.status = "The double slip isn't complete, it can't be replaced";
			return false;
		}
	}
	struct end
	{
		TTrack *track;
		glm::dvec3 point;
		glm::dvec2 inward;
		TTrack *outer;
	};
	std::vector<end> ends;
	std::vector<TTrack *> parts(switches.begin(), switches.end());
	for (auto *track : switches)
	{
		segment_data const *straight{nullptr};
		for (auto const &path : track->m_paths)
			if (path.points[segment_data::point::control1] == glm::dvec3{} && path.points[segment_data::point::control2] == glm::dvec3{})
				straight = &path;
		if (straight == nullptr)
			straight = &track->m_paths.front();
		auto const start{straight->points[segment_data::point::start]};
		ends.push_back({track, start, glm::normalize(plan_of(straight->points[segment_data::point::end] - start)), track->SwitchExtension->pPrevs[0]});
		for (int i = 0; i < 2; ++i)
		{
			auto *inner{track->SwitchExtension->pNexts[i]};
			if (inner != nullptr && inner->eType == tt_Normal && std::find(parts.begin(), parts.end(), inner) == parts.end() && inner->Length() < 5.0)
				parts.push_back(inner);
		}
	}
	int partner{-1};
	for (int i = 1; i < 4; ++i)
		if (glm::dot(ends[0].inward, ends[i].inward) < -0.999)
			partner = i;
	if (partner < 0)
	{
		state.status = "The double slip geometry isn't recognized";
		return false;
	}
	auto const &a{ends[0]};
	auto const &b{ends[partner]};
	std::vector<int> others;
	for (int i = 1; i < 4; ++i)
		if (i != partner)
			others.push_back(i);
	if (glm::dot(ends[0].inward, ends[others[0]].inward) < 0.0)
		std::swap(others[0], others[1]);
	auto const &c{ends[others[0]]};
	auto const &d{ends[others[1]]};
	auto const first{a.inward};
	auto direction{c.inward};
	auto const determinant{cross(first, direction)};
	if (std::abs(determinant) < 1e-9)
		return false;
	auto const t{cross(plan_of(c.point) - plan_of(a.point), direction) / determinant};
	glm::dvec2 const crossing{plan_of(a.point) + first * t};
	auto const angle{std::atan2(std::abs(cross(first, direction)), glm::dot(first, direction))};
	auto const oldhalf{glm::distance(plan_of(a.point), crossing)};
	auto radius{Shape.radius};
	auto half{radius * std::tan(angle * 0.5)};
	auto const height{(a.point.y + b.point.y) * 0.5};

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	if (half > oldhalf)
	{
		for (auto const *e : {&a, &b, &c, &d})
		{
			if (e->outer == nullptr || false == editor_track::is_straight(*e->outer, m_straights.tolerance))
			{
				half = oldhalf;
				radius = half / std::tan(angle * 0.5);
				state.status = "R limited to " + std::to_string(static_cast<int>(radius)) + " m, a neighbouring path isn't straight";
				break;
			}
			auto const line{editor_track::find_straight(*e->outer, m_straights.tolerance)};
			auto const reach{std::abs(glm::dot(plan_of(line.end) - crossing, -e->inward))};
			auto const reachstart{std::abs(glm::dot(plan_of(line.start) - crossing, -e->inward))};
			if (std::max(reach, reachstart) - 0.5 < half)
			{
				half = std::min(half, std::max(reach, reachstart) - 0.5);
				radius = half / std::tan(angle * 0.5);
				state.status = "R reduced to " + std::to_string(static_cast<int>(radius)) + " m to fit the straights";
			}
		}
	}
	for (auto *track : parts)
		editor_track::retire(*track);
	removed.insert(removed.end(), parts.begin(), parts.end());
	TTrack *style{a.outer != nullptr && a.outer->eType == tt_Normal ? a.outer : switches[0]};
	std::array<glm::dvec3, 4> points;
	std::array<end const *, 4> order{&a, &b, &c, &d};
	for (int i = 0; i < 4; ++i)
	{
		auto const &e{*order[i]};
		auto const outward{-e.inward};
		auto const target{crossing + outward * half};
		points[i] = {target.x, e.point.y, target.y};
		if (half < oldhalf - 0.01)
		{
			auto *filler{editor_track::create_path(*style, [&] {
				segment_data path;
				path.points[segment_data::point::start] = points[i];
				path.points[segment_data::point::end] = e.point;
				return path;
			}())};
			created.push_back(filler);
		}
		else if (half > oldhalf + 0.01 && e.outer != nullptr)
		{
			auto const line{editor_track::find_straight(*e.outer, m_straights.tolerance)};
			auto const alongold{glm::dot(plan_of(e.point) - plan_of(line.start), line.direction)};
			auto const alongnew{glm::dot(target - plan_of(line.start), line.direction)};
			cut_straight(line, std::min(alongold, alongnew), std::max(alongold, alongnew), states, created, removed, nullptr);
		}
	}
	auto const parts2{build_double_slip(points[0], points[1], points[2], points[3], crossing, height, first, direction, angle, radius, *style)};
	std::vector<TTrack *> commitlist(parts2.begin(), parts2.end());
	for (auto *track : created)
		if (std::find(commitlist.begin(), commitlist.end(), track) == commitlist.end())
			commitlist.push_back(track);
	editor_track::commit(commitlist);
	created.insert(created.end(), parts2.begin(), parts2.end());
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	return true;
}

std::vector<editor_mode::curve_sample> editor_mode::sample_chain(editor_track::chain const &Chain)
{
	std::vector<curve_sample> result;
	double station{0.0};
	for (std::size_t i = 0; i < Chain.tracks.size(); ++i)
	{
		auto const &path{Chain.tracks[i]->m_paths.front()};
		auto const forward{Chain.forward[i]};
		bezier const curve{path};
		auto const length{glm::distance(curve.p0, curve.p3)};
		int const steps{std::max(8, static_cast<int>(std::ceil(length / 0.5)))};
		for (int k = (result.empty() ? 0 : 1); k <= steps; ++k)
		{
			auto const t{forward ? static_cast<double>(k) / steps : 1.0 - static_cast<double>(k) / steps};
			auto const position{curve.point(t)};
			auto const derivative{forward ? curve.first(t) : -curve.first(t)};
			if (false == result.empty())
				station += plan_distance(position, result.back().position);
			curve_sample sample;
			sample.station = station;
			sample.position = position;
			sample.tangent = glm::normalize(plan_of(derivative));
			auto const roll{path.rolls[0] + (path.rolls[1] - path.rolls[0]) * t};
			sample.roll = static_cast<float>(forward ? roll : -roll);
			sample.radius = path.radius;
			result.push_back(sample);
		}
	}
	return result;
}

editor_mode::curve_sample editor_mode::sample_at(std::vector<curve_sample> const &Frame, double const Station)
{
	if (Frame.empty())
		return {};
	if (Station <= Frame.front().station)
		return Frame.front();
	if (Station >= Frame.back().station)
		return Frame.back();
	auto const next{std::lower_bound(Frame.begin(), Frame.end(), Station, [](curve_sample const &Sample, double const Value) { return Sample.station < Value; })};
	auto const previous{std::prev(next)};
	auto const span{std::max(1e-9, next->station - previous->station)};
	auto const f{(Station - previous->station) / span};
	curve_sample result;
	result.station = Station;
	result.position = glm::mix(previous->position, next->position, f);
	result.tangent = glm::normalize(glm::mix(previous->tangent, next->tangent, f));
	result.roll = static_cast<float>(previous->roll + (next->roll - previous->roll) * f);
	result.radius = previous->radius;
	return result;
}

double editor_mode::nearest_station(std::vector<curve_sample> const &Frame, glm::dvec3 const &Point)
{
	double best{std::numeric_limits<double>::max()};
	double station{0.0};
	for (std::size_t i = 0; i + 1 < Frame.size(); ++i)
	{
		glm::dvec2 const a{Frame[i].position.x, Frame[i].position.z};
		glm::dvec2 const b{Frame[i + 1].position.x, Frame[i + 1].position.z};
		auto const segment{b - a};
		auto const t{std::clamp(glm::dot(plan_of(Point) - a, segment) / std::max(1e-12, glm::dot(segment, segment)), 0.0, 1.0)};
		auto const distance{glm::distance(plan_of(Point), a + segment * t)};
		if (distance < best)
		{
			best = distance;
			station = Frame[i].station + (Frame[i + 1].station - Frame[i].station) * t;
		}
	}
	return station;
}

std::vector<segment_data> editor_mode::curved_switch_paths(editor_track::switch_template const &Shape, std::vector<curve_sample> const &Frame, double const Station, int const Direction, int const Side)
{
	auto const map = [&](glm::dvec3 const &Local, glm::dvec2 &Tangent, glm::dvec2 const &Localtangent) {
		auto const sample{sample_at(Frame, Station + Direction * Local.z)};
		auto const along{sample.tangent * static_cast<double>(Direction)};
		glm::dvec2 const across{glm::dvec2{-along.y, along.x} * static_cast<double>(Side)};
		Tangent = glm::normalize(along * Localtangent.y + across * Localtangent.x);
		auto const planar{plan_of(sample.position) + across * Local.x};
		auto const cant{glm::radians(static_cast<double>(sample.roll) * Direction)};
		auto const cone{-Local.x * Side * std::tan(cant)};
		return glm::dvec3{planar.x, sample.position.y + Local.y + cone, planar.y};
	};
	auto const hermite = [](glm::dvec3 const &Start, glm::dvec2 const &Startdirection, glm::dvec3 const &End, glm::dvec2 const &Enddirection) {
		auto const handle{plan_distance(Start, End) / 3.0};
		segment_data path;
		path.points[segment_data::point::start] = Start;
		path.points[segment_data::point::end] = End;
		path.points[segment_data::point::control1] = {Startdirection.x * handle, (End.y - Start.y) / 3.0, Startdirection.y * handle};
		path.points[segment_data::point::control2] = {-Enddirection.x * handle, -(End.y - Start.y) / 3.0, -Enddirection.y * handle};
		return path;
	};
	std::vector<segment_data> result;
	for (auto const &local : Shape.local)
	{
		auto const &start{local.points[segment_data::point::start]};
		auto const &end{local.points[segment_data::point::end]};
		auto const &control1{local.points[segment_data::point::control1]};
		auto const &control2{local.points[segment_data::point::control2]};
		glm::dvec2 const chord{end.x - start.x, end.z - start.z};
		auto const localstart{glm::normalize(control1 != glm::dvec3{} ? plan_of(control1) : chord)};
		auto const localend{glm::normalize(control2 != glm::dvec3{} ? -plan_of(control2) : chord)};
		glm::dvec2 startdirection, enddirection;
		auto const p0{map(start, startdirection, localstart)};
		auto const p3{map(end, enddirection, localend)};
		auto path{hermite(p0, startdirection, p3, enddirection)};
		auto const rollstart{sample_at(Frame, Station + Direction * start.z).roll * Direction};
		auto const rollend{sample_at(Frame, Station + Direction * end.z).roll * Direction};
		path.rolls = {static_cast<float>(rollstart), static_cast<float>(rollend)};
		path.radius = local.radius;
		result.push_back(path);
	}
	if (false == result.empty())
		result.front().radius = sample_at(Frame, Station).radius;
	return result;
}

void editor_mode::insert_curved_switch(int const Direction, int const Side)
{
	auto &tool{m_switch};
	auto const &shape{tool.templates[tool.armed]};
	auto const total{tool.frame.back().station};
	auto station{std::clamp(tool.along, Direction > 0 ? 0.0 : shape.length, Direction > 0 ? total - shape.length : total)};
	if (shape.length > total - 0.02)
		return;
	auto const from{Direction > 0 ? station : station - shape.length};
	auto const to{from + shape.length};
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	std::vector<TTrack *> candidates{tool.frame_tracks};
	for (auto const cut : {from, to})
	{
		auto const point{sample_at(tool.frame, cut).position};
		for (auto *candidate : std::vector<TTrack *>(candidates))
		{
			if (candidate->m_editorremoved || candidate->eType != tt_Normal)
				continue;
			auto const t{editor_track::nearest_parameter(*candidate, point)};
			auto const found{editor_track::point_at(*candidate, t)};
			if (plan_distance(found, point) > 0.05)
				continue;
			if (t > 0.002 && t < 0.998)
			{
				if (std::find(created.begin(), created.end(), candidate) == created.end() && std::none_of(states.begin(), states.end(), [&](auto const &Entry) { return Entry.first == candidate; }))
					states.emplace_back(candidate, editor_track::capture(*candidate));
				if (auto *second{editor_track::split_path(*candidate, t)})
				{
					created.push_back(second);
					candidates.push_back(second);
				}
			}
			break;
		}
	}
	for (auto *candidate : candidates)
	{
		if (candidate->m_editorremoved)
			continue;
		auto const middle{editor_track::point_at(*candidate, 0.5)};
		auto const where{nearest_station(tool.frame, middle)};
		if (where > from + 0.01 && where < to - 0.01)
		{
			editor_track::retire(*candidate);
			removed.push_back(candidate);
		}
	}
	auto const paths{curved_switch_paths(shape, tool.frame, station, Direction, Side)};
	auto *track{editor_track::create_switch(shape, paths, *tool.frame_tracks.front())};
	if (track != nullptr)
	{
		editor_track::commit({track});
		created.push_back(track);
	}
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
}
