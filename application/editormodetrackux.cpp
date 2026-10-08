/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
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
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <algorithm>
#include <unordered_set>
#include <unordered_map>
#include <queue>
#include <array>
#include <functional>
#include <cctype>
#include <chrono>
#include <cmath>

namespace
{

using geometry::bezier;

float const kHoverPixels{12.0f};
double const kHoverReachMin{15.0};
double const kHoverReachMax{250.0};
double const kHoverLabelDelay{0.35}; // s the cursor rests on a path before its data shows up
double const kFocusDistance{25.0};
auto const kContextClick{std::chrono::milliseconds(350)};
float const kContextTurn{0.02f}; // rad the camera may turn during a click of the right button
std::size_t const kSearchLimit{200};
int const kRailCategory{1};

bool viewport_click()
{
	return false == ImGui::GetIO().MouseDownOwned[0];
}

bool rail(TTrack const &Track)
{
	return (Track.iCategoryFlag & 15) == kRailCategory && false == Track.m_editorremoved && Track.visible();
}

std::string name_of(TTrack const &Track)
{
	return Track.name().empty() ? std::string{"(noname)"} : Track.name();
}

std::string lower(std::string Text)
{
	std::transform(Text.begin(), Text.end(), Text.begin(), [](unsigned char const Character) { return static_cast<char>(std::tolower(Character)); });
	return Text;
}

char const *type_of(TTrack const &Track)
{
	switch (Track.eType)
	{
	case tt_Normal: return "normal";
	case tt_Switch: return "switch";
	case tt_Cross: return "cross";
	case tt_Table: return "turntable";
	case tt_Tributary: return "tributary";
	default: return "unknown";
	}
}

double segment_distance2(glm::vec2 const &Point, glm::vec2 const &A, glm::vec2 const &B, double &T)
{
	auto const ab{B - A};
	auto const length2{glm::dot(ab, ab)};
	T = length2 > 1e-6f ? std::clamp(static_cast<double>(glm::dot(Point - A, ab) / length2), 0.0, 1.0) : 0.0;
	auto const closest{A + ab * static_cast<float>(T)};
	return glm::dot(Point - closest, Point - closest);
}

} // namespace

editor_mode::track_hover editor_mode::track_under_cursor() const
{
	track_hover result;
	auto const &io{ImGui::GetIO()};
	glm::dvec3 const camera{Global.pCamera.Pos};
	glm::dvec3 const ground{cursor_ground()};
	auto const reach{std::clamp(glm::distance(camera, ground) * 0.1, kHoverReachMin, kHoverReachMax)};
	screen_projection const projection;
	glm::vec2 const mouse{io.MousePos.x, io.MousePos.y};
	double best{kHoverPixels * kHoverPixels};
	for (auto *section : simulation::Region->sections(ground, static_cast<float>(reach)))
	{
		for (auto const &cell : section->m_cells)
		{
			for (auto *track : cell.m_directories.paths)
			{
				if (track == nullptr || false == rail(*track))
					continue;
				for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
				{
					bezier const curve{track->m_paths[i]};
					auto const count{std::clamp(static_cast<int>(curve.plan_length() / 3.0), 6, 64)};
					auto previous{curve.point(0.0)};
					ImVec2 from;
					bool visible{projection.project(previous, from)};
					for (int k = 1; k <= count; ++k)
					{
						auto const next{curve.point(static_cast<double>(k) / count)};
						ImVec2 to;
						bool const shown{projection.project(next, to)};
						if (visible && shown)
						{
							double t;
							auto const distance{segment_distance2(mouse, {from.x, from.y}, {to.x, to.y}, t)};
							auto const point{glm::mix(previous, next, t)};
							if (distance < best && glm::distance(point, camera) <= static_cast<double>(kMaxPlacementDistance))
							{
								best = distance;
								result.track = track;
								result.path = i;
								result.point = point;
							}
						}
						previous = next;
						from = to;
						visible = shown;
					}
				}
			}
		}
	}
	return result;
}

void editor_mode::update_track_hover()
{
	if (typed_meaning() == nullptr)
		m_typed.clear();
	bool const blocked{ImGui::GetIO().WantCaptureMouse || mouseHold || m_track_set_using || m_track_box.active || m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS || m_track_gizmo_using || m_route_gizmo_using || m_extend.active || m_switch.placing ||
	                   m_straights.dragging || m_lay.active};
	if (blocked)
	{
		m_hover = {};
		return;
	}
	auto found{track_under_cursor()};
	found.since = found.track == m_hover.track ? m_hover.since : ImGui::GetTime();
	m_hover = found;
}

void editor_mode::draw_track_hover() const
{
	using kind = track_intent::kind;
	auto const *track{m_hover.track};
	if (track == nullptr || (m_intent.what != kind::select && m_intent.what != kind::box && m_intent.what != kind::straight_set) || m_intent.track != track)
		return;
	screen_projection const projection;
	if (track != selected_track())
	{
		auto *drawlist{ImGui::GetBackgroundDrawList()};
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			bezier const curve{track->m_paths[i]};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 3.0), 6, 64)};
			auto const colour{i == m_hover.path ? IM_COL32(255, 255, 255, 200) : IM_COL32(255, 255, 255, 110)};
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				projection.line(drawlist, previous, next, colour, i == m_hover.path ? 3.5f : 2.0f);
				previous = next;
			}
		}
	}
}

bool editor_mode::track_gesture_on() const
{
	return mouseHold || m_track_set_using || m_track_box.active || m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS || m_track_gizmo_using || m_route_gizmo_using || m_extend.active || m_switch.placing || m_straights.dragging || m_straights.tool_mouse ||
	       m_point_drag.active || m_handle_drag.active || m_sweep.dragging || m_turntable.placing || m_signal.placing || m_signal.moving || ImGuizmo::IsUsing();
}

editor_mode::track_intent editor_mode::track_intent_at(int const Mods)
{
	using kind = track_intent::kind;
	track_intent intent;
	if (ImGui::GetIO().WantCaptureMouse)
		return intent;
	glm::dvec3 const ground{cursor_ground()};
	intent.position = ground;
	bool const shift{(Mods & GLFW_MOD_SHIFT) != 0};
	bool const control{(Mods & GLFW_MOD_CONTROL) != 0};
	bool const alt{(Mods & GLFW_MOD_ALT) != 0};
	auto const make = [&](kind const What, char const *Action) {
		intent.what = What;
		intent.action = Action != nullptr ? STR(Action) : std::string{};
		return intent;
	};
	auto *selected{selected_track()};

	if (m_lay.active)
	{
		auto const &lay{m_lay};
		if (lay.mouse_snap.track != nullptr && (lay.points.empty() || glm::distance(lay.mouse_snap.position, lay.points.front()) > 1.0))
		{
			intent.snap = lay.mouse_snap;
			intent.track = lay.mouse_snap.track;
			intent.position = lay.mouse_snap.position;
			auto const diverging{lay.mouse_snap.track->eType == tt_Switch && lay.mouse_snap.point == editor_track::point_ref{1, editor_track::point_kind::end}};
			if (lay.points.empty() && diverging)
				return make(kind::lay_end, "Click: start from this free end, drag: the curve of the switch goes on, Shift: straight");
			return make(kind::lay_end, lay.points.empty() ? "Click: start from this free end, drag: lead a track out of it" : "Click: join this end and lay");
		}
		intent.position = lay.mouse;
		return make(kind::lay, lay.points.empty() ? "Click: start of the new track" : "Click: next point");
	}
	if (m_track_tab == track_tab::turntable)
		return turntable_intent(ground, shift);
	if (m_track_tab == track_tab::signals)
	{
		if (auto const hit{signal_hit()}; hit >= 0)
		{
			auto const &standing{m_signal.standing[hit]};
			intent.index = hit;
			intent.position = standing.include.location;
			intent.action = format(STR_C("Click: select the signal %s, drag: move it along the track"), standing.name.c_str());
			intent.what = kind::signal_move;
			return intent;
		}
		auto const *armed{signal_armed()};
		if (m_hover.track != nullptr && armed != nullptr)
		{
			signal_spot spot;
			if (signal_spot_at(*m_hover.track, m_hover.path, m_hover.point, cursor_level(m_hover.point.y), 0, 0, *armed, spot))
			{
				intent.track = m_hover.track;
				intent.path = m_hover.path;
				intent.position = spot.position;
				intent.action = signal_intent_label(*armed, spot);
				intent.what = kind::signal_place;
				return intent;
			}
		}
		if (m_hover.track != nullptr)
		{
			intent.track = m_hover.track;
			intent.path = m_hover.path;
			intent.position = m_hover.point;
			return make(kind::select, "Choose the type of the signal in the Tracks window first");
		}
		return intent;
	}
	if (m_track_tab == track_tab::path)
	{
		if (shift && control && m_hover.track != nullptr && rail(*m_hover.track))
		{
			intent.track = m_hover.track;
			intent.position = m_hover.point;
			auto const &preview{track_spread_preview(*m_hover.track)};
			intent.action = format(in_track_set(m_hover.track) ? STR_C("Click: take out the %zu joined paths, %.0f m") : STR_C("Click: add the %zu joined paths, %.0f m"), preview.size(), m_spread.length);
			intent.what = kind::spread;
			return intent;
		}
		if (shift && false == gizmo_frame::over())
		{
			intent.track = m_hover.track;
			return make(kind::box, m_hover.track == nullptr ? "Drag: the tracks in a box join the selection" : in_track_set(m_hover.track) ? "Click: take out of the selection, drag: a box" : "Click: add to the selection, drag: a box");
		}
		if (auto const hit{track_handle_hit()}; hit.valid() && selected != nullptr)
		{
			std::string reason;
			if (editor_track::can_edit_geometry(*selected, reason))
			{
				intent.track = selected;
				intent.point = hit;
				intent.position = editor_track::point_position(*selected, hit);
				if (false == editor_track::is_end(hit.kind))
					return make(kind::point, "Drag: the control vector, the curve follows");
				if (editor_track::is_connected(*selected, hit))
					return make(kind::point, "Drag: the joint, the neighbours follow");
				if (control)
					return make(kind::point, "Drag: the end, dropped on a free end it joins it");
				intent.snap.track = selected;
				intent.snap.point = hit;
				intent.snap.position = intent.position;
				auto const diverging{selected->eType == tt_Switch && hit == editor_track::point_ref{1, editor_track::point_kind::end}};
				return make(kind::extend_end, diverging ? "Drag: the curve of the switch goes on, Shift: straight; Ctrl+drag: move the end" : "Drag: a new track out of this end, as in laying; Ctrl+drag: move the end");
			}
		}
	}
	if (m_track_tab == track_tab::straights)
	{
		auto const &line{current_straight()};
		if (alt)
		{
			intent.track = m_hover.track;
			return m_hover.track != nullptr ? make(kind::straight_set, "Click: move this straight together with the others") : intent;
		}
		if (control && false == line.tracks.empty())
			return make(kind::straight_break, "Press: break the straight here, drag to turn the rest");
		if (shift && false == line.tracks.empty())
			return make(kind::detour, "Click: next point of the shift around an obstacle");
	}
	if (m_track_tab == track_tab::straights && m_straights.tool == 0)
	{
		if (auto const hit{straight_handle_hit()}; hit >= 0)
		{
			auto const &line{current_straight()};
			glm::dvec3 const handles[] = {line.start, line.end, (line.start + line.end) * 0.5};
			intent.index = hit;
			intent.position = handles[hit];
			if (hit == 2)
			{
				auto const offer{straight_parallel_offer(line, {})};
				if (offer.near)
				{
					intent.action = format(offer.snaps ? STR_C("Drag: shift the straight sideways — snaps parallel at %.2f m") : STR_C("Drag: shift the straight sideways — close: parallel snap at %.2f m"), offer.spacing);
					intent.what = kind::straight_handle;
					return intent;
				}
			}
			return make(kind::straight_handle, hit == 2 ? "Drag: shift the straight sideways" : "Drag: move this end, the curves at it follow");
		}
	}
	if (int vertex, grip; route_hit(vertex, grip))
	{
		intent.index = vertex >= 0 ? vertex : grip;
		intent.position = vertex >= 0 ? route_vertex_position(vertex) : alignment::evaluate(m_route.result, m_route.result.vertex_chainages[grip]).position;
		return make(vertex >= 0 ? kind::route_vertex : kind::route_grip, vertex >= 0 ? "Drag: move this vertex, the curve follows" : "Drag: the radius of this curve");
	}
	if (gizmo_frame::over())
		return make(kind::gizmo, nullptr);
	if (m_track_tab == track_tab::straights)
	{
		auto const &line{current_straight()};
		if (m_straights.tool != 0 && false == line.tracks.empty())
		{
			intent.position = line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * std::clamp(line.along(ground), 0.0, line.length);
			return make(kind::straight_tool, m_straights.tool == 1 ? "Click: break the straight here" : "Click: start of the shift");
		}
	}
	if (selected != nullptr && false == selected->m_paths.empty())
	{
		glm::dvec3 point;
		if (switch_reach(cursor_level(selected->m_paths.front().points[segment_data::point::start].y), point))
		{
			intent.track = selected;
			intent.position = point;
			return make(kind::switch_at, "Press and drag: a switch here, along sets its direction, sideways its side");
		}
	}
	if (m_hover.track != nullptr)
	{
		intent.track = m_hover.track;
		intent.path = m_hover.path;
		intent.position = m_hover.point;
		switch (m_track_tab)
		{
		case track_tab::straights: return make(kind::select, "Click: the whole straight through this track");
		case track_tab::route: return make(kind::select, "Click: the curve of this track with its straights");
		case track_tab::turnout: return make(kind::select, m_hover.track->eType == tt_Switch ? "Click: edit this switch" : m_switch.armed < 0 ? "Click: choose this track, then the type of the switch" : "Click: put a switch into this track");
		case track_tab::profile: return make(kind::select, "Click: the grade line through this track");
		case track_tab::lineside: return make(kind::select, "Click: the line the objects go along");
		default: return make(kind::select, "Click: select");
		}
	}
	return intent;
}

void editor_mode::update_track_intent()
{
	using kind = track_intent::kind;
	if (track_gesture_on())
	{
		m_intent = {};
		return;
	}
	auto const &io{ImGui::GetIO()};
	m_intent = track_intent_at((io.KeyShift ? GLFW_MOD_SHIFT : 0) | (io.KeyCtrl ? GLFW_MOD_CONTROL : 0) | (io.KeyAlt ? GLFW_MOD_ALT : 0));
	switch (m_intent.what)
	{
	case kind::none:
	case kind::gizmo: break;
	case kind::point:
	case kind::straight_handle:
	case kind::route_vertex:
	case kind::route_grip: ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll); break;
	default: ImGui::SetMouseCursor(ImGuiMouseCursor_Hand); break;
	}
}

void editor_mode::track_press(track_intent const &Intent)
{
	using kind = track_intent::kind;
	switch (Intent.what)
	{
	case kind::lay_end:
		if (m_lay.points.empty() && start_extend(Intent.snap))
			return;
		[[fallthrough]];
	case kind::lay:
		GfxRenderer->Pick_Node_Callback([this](scene::basic_node * /*node*/) {
			if (viewport_click())
				lay_click();
		});
		return;
	case kind::sweep: sweep_press(); return;
	case kind::box: start_track_box(); return;
	case kind::spread:
		if (Intent.track != nullptr)
			track_spread_apply(*Intent.track);
		return;
	case kind::point: point_drag_start(Intent.point); return;
	case kind::gizmo: return;
	case kind::straight_set:
	{
		auto *hovered{Intent.track};
		GfxRenderer->Pick_Node_Callback([this, hovered](scene::basic_node *node) {
			auto *track{hovered != nullptr ? hovered : dynamic_cast<TTrack *>(node)};
			if (viewport_click() && track != nullptr)
				toggle_straight_set(*track);
		});
		return;
	}
	case kind::straight_break: start_straight_gesture(1); return;
	case kind::detour: add_detour_point(); return;
	case kind::straight_tool: place_straight_tool(); return;
	case kind::straight_handle:
		m_straights.handle = Intent.index;
		handle_drag_start(Intent.position);
		return;
	case kind::route_vertex:
		m_route.vertex = Intent.index;
		m_route.grip = -1;
		handle_drag_start(Intent.position);
		return;
	case kind::route_grip:
		m_route.vertex = -1;
		m_route.grip = Intent.index;
		handle_drag_start(Intent.position);
		return;
	case kind::switch_at: start_switch_placement(); return;
	case kind::extend_end:
		if (false == start_extend(Intent.snap))
			point_drag_start(Intent.snap.point);
		return;
	case kind::turntable_place:
	case kind::turntable_exit:
	case kind::turntable_fit: turntable_press(Intent); return;
	case kind::signal_place:
	case kind::signal_move: signal_press(Intent); return;
	case kind::select:
	case kind::none:
	{
		auto *hovered{Intent.track};
		GfxRenderer->Pick_Node_Callback([this, hovered](scene::basic_node *node) {
			if (false == viewport_click())
				return;
			auto *track{hovered != nullptr ? hovered : dynamic_cast<TTrack *>(node)};
			if (m_track_tab == track_tab::path)
			{
				m_track_set.clear();
				if (track == nullptr)
				{
					m_node = nullptr;
					m_track_point = {};
					ui()->set_node(nullptr);
					return;
				}
			}
			select_track(track != nullptr ? track : node);
		});
		return;
	}
	}
}

void editor_mode::draw_track_intent() const
{
	using kind = track_intent::kind;
	auto const &intent{m_intent};
	auto const &io{ImGui::GetIO()};
	if (intent.what == kind::none || intent.what == kind::gizmo || io.WantCaptureMouse)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	ImVec2 at;
	if (projection.project(intent.position, at))
	{
		switch (intent.what)
		{
		case kind::point:
		case kind::straight_handle:
		case kind::route_vertex:
		case kind::route_grip: drawlist->AddCircle(at, 13.0f, overlay_color::marked, 24, 2.5f); break;
		case kind::switch_at:
		case kind::straight_tool:
			drawlist->AddCircleFilled(at, 4.0f, overlay_color::grip);
			drawlist->AddCircle(at, 11.0f, overlay_color::grip, 20, 2.0f);
			break;
		case kind::lay_end:
		case kind::extend_end:
		case kind::turntable_fit: drawlist->AddCircleFilled(at, 5.0f, IM_COL32(255, 60, 255, 255)); break;
		case kind::turntable_exit: drawlist->AddCircle(at, 11.0f, overlay_color::grip, 20, 2.0f); break;
		case kind::lay: drawlist->AddCircle(at, 6.0f, IM_COL32(255, 255, 255, 200), 12, 1.5f); break;
		default: break;
		}
	}
	if (intent.action.empty() || false == track_readout().empty())
		return;
	std::string details;
	auto const *track{m_hover.track};
	if (intent.what == kind::select && track != nullptr && track == intent.track && ImGui::GetTime() - m_hover.since >= kHoverLabelDelay)
	{
		auto const &path{track->m_paths[std::min<std::size_t>(m_hover.path, track->m_paths.size() - 1)]};
		details = name_of(*track);
		details += "\n" + std::string{STR_C(type_of(*track))};
		if (track->eType == tt_Switch)
			details += m_hover.path == 0 ? STR(" (main)") : STR(" (branch)");
		details += format("   L %.1f m", bezier{path}.plan_length());
		details += path.radius != 0.0f ? format("   R %.0f m", std::abs(path.radius)) : std::string{"   "} + STR_C("straight");
		auto const velocity{editor_track::velocity(*track)};
		details += velocity > 0.0 ? format("   V %.0f km/h", velocity) : std::string{};
	}
	auto *foreground{ImGui::GetForegroundDrawList()};
	auto const actionsize{ImGui::CalcTextSize(intent.action.c_str())};
	auto const detailsize{details.empty() ? ImVec2(0.0f, 0.0f) : ImGui::CalcTextSize(details.c_str())};
	ImVec2 const size{std::max(actionsize.x, detailsize.x), actionsize.y + (details.empty() ? 0.0f : detailsize.y + 4.0f)};
	ImVec2 position{io.MousePos.x + 18.0f, io.MousePos.y + 18.0f};
	position.x = std::min(position.x, io.DisplaySize.x - size.x - 10.0f);
	position.y = std::min(position.y, io.DisplaySize.y - size.y - 10.0f);
	foreground->AddRectFilled(ImVec2(position.x - 6.0f, position.y - 4.0f), ImVec2(position.x + size.x + 6.0f, position.y + size.y + 4.0f), IM_COL32(0, 0, 0, 170), 4.0f);
	foreground->AddText(position, overlay_color::marked, intent.action.c_str());
	if (false == details.empty())
		foreground->AddText(ImVec2(position.x, position.y + actionsize.y + 4.0f), IM_COL32(255, 255, 255, 235), details.c_str());
}

void editor_mode::track_context_press()
{
	auto &context{m_track_context};
	auto const &io{ImGui::GetIO()};
	context.armed = m_hover.track != nullptr;
	context.target = m_hover;
	context.pressed = std::chrono::steady_clock::now();
	context.angle = Camera.Angle;
	context.mouse = {io.MousePos.x, io.MousePos.y};
}

void editor_mode::track_context_release()
{
	auto &context{m_track_context};
	if (false == context.armed)
		return;
	context.armed = false;
	if (std::chrono::steady_clock::now() - context.pressed < kContextClick && glm::length(Camera.Angle - context.angle) < kContextTurn)
		context.pending = true;
}

void editor_mode::split_track_at(TTrack &Track, glm::dvec3 const &Point)
{
	auto const parameter{editor_track::nearest_parameter(Track, Point)};
	auto const before{editor_track::capture(Track)};
	if (auto *created{editor_track::split_path(Track, parameter)})
	{
		push_track_snapshot({{&Track, before}}, {created});
		straight_refresh();
	}
}

void editor_mode::focus_track(TTrack &Track, int const Path, glm::dvec3 const &Point)
{
	auto const &path{Track.m_paths[std::min<std::size_t>(std::max(Path, 0), Track.m_paths.size() - 1)]};
	bezier const curve{path};
	glm::dvec3 look{curve.first(std::clamp(editor_track::nearest_parameter(Track, Point), 0.0, 1.0))};
	look.y = 0.0;
	look = glm::length(look) > 1e-6 ? glm::normalize(look) : glm::dvec3(0.0, 0.0, 1.0);
	if (glm::dot(Camera.Pos - Point, look) > 0.0)
		look = -look;
	look = glm::normalize(look - glm::dvec3(0.0, 0.25, 0.0));
	m_focus_start_pos = Camera.Pos;
	m_focus_start_angle = Camera.Angle;
	m_focus_target_pos = Point - look * kFocusDistance;
	m_focus_target_angle = glm::vec3(static_cast<float>(std::asin(std::clamp(look.y, -1.0, 1.0))), static_cast<float>(std::atan2(-look.x, -look.z)), 0.0f);
	m_focus_active = true;
	m_focus_time = 0.0;
	m_focus_duration = 0.6;
}

void editor_mode::render_track_context()
{
	auto &context{m_track_context};
	auto *track{context.target.track};
	if (context.pending)
	{
		context.pending = false;
		if (track != nullptr && false == track->m_editorremoved)
		{
			select_track(track);
			ImGui::OpenPopup("##trackcontext");
			ImGui::SetNextWindowPos(ImVec2(context.mouse.x, context.mouse.y));
		}
	}
	if (false == ImGui::BeginPopup("##trackcontext"))
		return;
	if (track == nullptr || track->m_editorremoved)
	{
		ImGui::CloseCurrentPopup();
		ImGui::EndPopup();
		return;
	}
	auto const point{context.target.point};
	bool const normal{track->eType == tt_Normal && editor_track::is_supported(*track)};
	ImGui::TextDisabled("%s  (%s, %.1f m)", name_of(*track).c_str(), STR_C(type_of(*track)), track->Length());
	ImGui::Separator();
	if (ImGui::MenuItem(STR_C("Split here"), "K", false, normal))
		split_track_at(*track, point);
	ImGui::Separator();
	if (ImGui::MenuItem(STR_C("Switch"), "T", false, normal || track->eType == tt_Switch))
		show_track_tab(track_tab::turnout);
	if (ImGui::MenuItem(STR_C("Whole straight"), "G", false, normal && editor_track::is_straight(*track, m_straights.tolerance)))
		show_track_tab(track_tab::straights);
	if (ImGui::MenuItem(STR_C("Curve"), "C", false, normal))
		show_track_tab(track_tab::route);
	if (ImGui::MenuItem(STR_C("Parallel track"), "L", false, normal))
		show_track_tab(track_tab::lay);
	if (ImGui::MenuItem(STR_C("Place a vehicle here")))
	{
		show_track_tab(track_tab::lineside);
		vehicle_start(*track, point);
	}
	if (ImGui::MenuItem(STR_C("Hectometre posts from here")))
	{
		show_track_tab(track_tab::lineside);
		hekto_start(*track, point);
	}
	if (ImGui::MenuItem(STR_C("Fouling point markers"), nullptr, false, track->eType == tt_Switch))
	{
		show_track_tab(track_tab::lineside);
		m_fouling.scope = 0;
		m_fouling.dirty = true;
		m_fouling.expand = true;
	}
	if (ImGui::MenuItem(STR_C("Vertical profile of the line"), "P"))
	{
		show_track_tab(track_tab::profile);
		profile_open_run(*track);
	}
	if (ImGui::MenuItem(STR_C("Speed check of the line"), "V"))
	{
		m_speed.scope = 0;
		show_track_tab(track_tab::speed);
		speed_start();
	}
	ImGui::Separator();
	if (ImGui::MenuItem(in_track_set(track) ? STR_C("Take out of the selection") : STR_C("Add to the selection"), "Shift+LMB"))
		track_set_toggle(*track);
	if (ImGui::MenuItem(STR_C("Look at it")))
		focus_track(*track, context.target.path, point);
	if (ImGui::MenuItem(STR_C("Copy the name"), nullptr, false, false == track->name().empty()))
		ImGui::SetClipboardText(track->name().c_str());
	ImGui::Separator();
	if (ImGui::MenuItem(STR_C("Delete"), "Del"))
		delete_selected_track();
	ImGui::EndPopup();
}

void editor_mode::render_track_search()
{
	auto &search{m_track_search};
	if (search.focus)
	{
		ImGui::SetKeyboardFocusHere();
		search.focus = false;
	}
	ImGui::SetNextItemWidth(-1.0f);
	ImGui::InputTextWithHint("##findtrack", STR_C("Find a track by its name (Ctrl+F)"), search.text, sizeof(search.text));
	std::string const text{lower(search.text)};
	if (text.empty())
	{
		search.searched.clear();
		search.found.clear();
		return;
	}
	if (text != search.searched)
	{
		search.searched = text;
		search.found.clear();
		search.total = 0;
		for (auto *track : simulation::Paths.sequence())
		{
			if (track == nullptr || false == rail(*track) || lower(track->name()).find(text) == std::string::npos)
				continue;
			++search.total;
			if (search.found.size() < kSearchLimit)
				search.found.push_back(track);
		}
	}
	if (search.found.empty())
	{
		ImGui::TextDisabled("%s", STR_C("No track of that name"));
		return;
	}
	if (search.total > search.found.size())
		ImGui::TextDisabled(STR_C("%zu found, the first %zu listed"), search.total, search.found.size());
	auto const rows{std::min<std::size_t>(search.found.size(), 8)};
	ImGui::BeginChild("##foundtracks", ImVec2(0.0f, ImGui::GetTextLineHeightWithSpacing() * static_cast<float>(rows) + 6.0f), true);
	for (std::size_t i = 0; i < search.found.size(); ++i)
	{
		auto *track{search.found[i]};
		if (track->m_editorremoved)
			continue;
		auto const label{format("%s##found%zu", name_of(*track).c_str(), i)};
		if (ImGui::Selectable(label.c_str(), track == selected_track()))
		{
			m_node = track;
			m_track_point = {};
			ui()->set_node(m_node);
			focus_track(*track, 0, bezier{track->m_paths.front()}.point(0.5));
		}
		ImGui::SameLine(ImGui::GetWindowContentRegionMax().x * 0.6f);
		ImGui::TextDisabled("%s, %.1f m", STR_C(type_of(*track)), track->Length());
	}
	ImGui::EndChild();
}

char const *editor_mode::typed_meaning() const
{
	auto const &straights{m_straights};
	if (m_extend.active)
		return m_extend_freehand && false == ImGui::GetIO().KeyCtrl ? "type the radius, a space, the length" : "type the length";
	if (straights.dragging)
		return straights.handle == 2 ? "type the shift, + to the right" : "type the length";
	if (straights.tool == 1 && (straights.tool_mouse || straights.tool_dragging))
		return "type the turn in degrees, + to the right";
	if (m_lay.active && false == m_lay.points.empty())
		return "type the length of the next leg, Enter adds the point";
	return nullptr;
}

std::vector<double> editor_mode::typed_values() const
{
	std::vector<double> values;
	std::string text{m_typed};
	std::replace(text.begin(), text.end(), ',', '.');
	std::size_t at{0};
	while (at < text.size())
	{
		auto const end{text.find(' ', at)};
		auto const word{text.substr(at, end == std::string::npos ? std::string::npos : end - at)};
		if (false == word.empty() && word != "-" && word != ".")
			values.push_back(std::strtod(word.c_str(), nullptr));
		if (end == std::string::npos)
			break;
		at = end + 1;
	}
	return values;
}

bool editor_mode::typed_key(int const Key)
{
	if (typed_meaning() == nullptr)
	{
		m_typed.clear();
		return false;
	}
	char character{'\0'};
	if (Key >= GLFW_KEY_0 && Key <= GLFW_KEY_9)
		character = static_cast<char>('0' + (Key - GLFW_KEY_0));
	else if (Key >= GLFW_KEY_KP_0 && Key <= GLFW_KEY_KP_9)
		character = static_cast<char>('0' + (Key - GLFW_KEY_KP_0));
	else if (Key == GLFW_KEY_PERIOD || Key == GLFW_KEY_COMMA || Key == GLFW_KEY_KP_DECIMAL)
		character = '.';
	else if (Key == GLFW_KEY_MINUS || Key == GLFW_KEY_KP_SUBTRACT)
		character = '-';
	else if (Key == GLFW_KEY_SPACE)
		character = ' ';
	else if (Key == GLFW_KEY_BACKSPACE && false == m_typed.empty())
	{
		m_typed.pop_back();
		return true;
	}
	else
		return false;
	auto const start{m_typed.empty() || m_typed.back() == ' '};
	if ((character == '-' && false == start) || (character == ' ' && start) || (character == '.' && m_typed.substr(m_typed.rfind(' ') == std::string::npos ? 0 : m_typed.rfind(' ')).find('.') != std::string::npos))
		return true;
	if (m_typed.size() < 24)
		m_typed += character;
	return true;
}

void editor_mode::typed_extend()
{
	auto const values{typed_values()};
	if (values.empty())
		return;
	auto &tool{m_extend};
	auto const &direction{tool.direction};
	if (false == m_extend_freehand || ImGui::GetIO().KeyCtrl)
	{
		auto const length{std::max(0.0, values[0])};
		tool.mouse = {tool.point.x + direction.x * length, tool.mouse.y, tool.point.z + direction.y * length};
		return;
	}
	auto const radius{std::abs(values[0])};
	if (radius < 1.0)
		return;
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	double const side{geometry::cross(direction, offset) >= 0.0 ? 1.0 : -1.0};
	glm::dvec2 const point{tool.point.x, tool.point.z};
	glm::dvec2 const centre{point + glm::dvec2{-direction.y, direction.x} * (side * radius)};
	auto const radial{point - centre};
	auto angle{values.size() > 1 ? std::abs(values[1]) / radius : side * geometry::signed_angle(radial, glm::dvec2{tool.mouse.x, tool.mouse.z} - centre)};
	if (angle < 0.0)
		angle += 2.0 * glm::pi<double>();
	angle = std::clamp(angle, 1e-3, 2.0 * glm::pi<double>() - 0.1);
	auto const end{centre + geometry::turned(radial, side * angle)};
	tool.mouse = {end.x, tool.mouse.y, end.y};
}

glm::dvec3 editor_mode::typed_lay(glm::dvec3 const &Mouse) const
{
	auto const values{typed_values()};
	if (values.empty() || m_lay.points.empty() || values[0] <= 0.0)
		return Mouse;
	auto const &last{m_lay.points.back()};
	glm::dvec2 heading{Mouse.x - last.x, Mouse.z - last.z};
	if (glm::length(heading) < 1e-6)
		return Mouse;
	heading = glm::normalize(heading) * values[0];
	return {last.x + heading.x, Mouse.y, last.z + heading.y};
}

void editor_mode::typed_turn()
{
	auto &state{m_straights};
	auto const values{typed_values()};
	if (values.empty() || state.tool != 1)
		return;
	auto const reach{std::max(1.0, glm::length(glm::dvec2{state.tool_handle.x - state.tool_point.x, state.tool_handle.z - state.tool_point.z}))};
	auto const heading{geometry::turned(state.tool_line.direction, glm::radians(values[0])) * reach};
	state.tool_handle = {state.tool_point.x + heading.x, state.tool_handle.y, state.tool_point.z + heading.y};
}

bool editor_mode::in_track_set(TTrack const *Track) const
{
	return std::find(m_track_set.begin(), m_track_set.end(), Track) != m_track_set.end();
}

void editor_mode::track_set_prune()
{
	m_track_set.erase(std::remove_if(m_track_set.begin(), m_track_set.end(), [](TTrack const *Track) { return Track == nullptr || Track->m_editorremoved; }), m_track_set.end());
}

void editor_mode::track_set_toggle(TTrack &Track)
{
	track_set_prune();
	if (m_track_set.empty())
		if (auto *selected{selected_track()}; selected != nullptr && selected != &Track)
			m_track_set.push_back(selected);
	if (auto const found{std::find(m_track_set.begin(), m_track_set.end(), &Track)}; found != m_track_set.end())
		m_track_set.erase(found);
	else
		m_track_set.push_back(&Track);
	m_node = m_track_set.empty() ? nullptr : m_track_set.back();
	m_track_point = {};
	ui()->set_node(m_node);
}

void editor_mode::start_track_box()
{
	auto const &io{ImGui::GetIO()};
	m_track_box.active = true;
	m_track_box.from = m_track_box.to = {io.MousePos.x, io.MousePos.y};
	m_track_box.hovered = m_hover.track;
}

void editor_mode::finish_track_box()
{
	auto &box{m_track_box};
	if (false == box.active)
		return;
	box.active = false;
	auto const &io{ImGui::GetIO()};
	box.to = {io.MousePos.x, io.MousePos.y};
	if (glm::length(box.to - box.from) < 6.0f)
	{
		if (box.hovered != nullptr && false == box.hovered->m_editorremoved)
			track_set_toggle(*box.hovered);
		return;
	}
	glm::vec2 const low{std::min(box.from.x, box.to.x), std::min(box.from.y, box.to.y)};
	glm::vec2 const high{std::max(box.from.x, box.to.x), std::max(box.from.y, box.to.y)};
	screen_projection const projection;
	glm::dvec3 const camera{Global.pCamera.Pos};
	track_set_prune();
	if (m_track_set.empty())
		if (auto *selected{selected_track()}; selected != nullptr)
			m_track_set.push_back(selected);
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || false == rail(*track) || in_track_set(track) || glm::distance(track->location(), camera) > static_cast<double>(Global.BaseDrawRange))
			continue;
		bool inside{false};
		for (auto const &path : track->m_paths)
		{
			bezier const curve{path};
			for (int k = 0; k <= 8 && false == inside; ++k)
			{
				ImVec2 screen;
				if (projection.project(curve.point(k / 8.0), screen))
					inside = screen.x >= low.x && screen.x <= high.x && screen.y >= low.y && screen.y <= high.y;
			}
			if (inside)
				break;
		}
		if (inside)
			m_track_set.push_back(track);
	}
	if (false == m_track_set.empty())
	{
		m_node = m_track_set.back();
		m_track_point = {};
		ui()->set_node(m_node);
	}
}

std::vector<TTrack *> editor_mode::track_spread_from(TTrack &Start) const
{
	auto const &rules{m_spread};
	auto const bed = [](TTrack const &Track) { return Track.eType == tt_Switch ? (Track.SwitchExtension ? Track.SwitchExtension->m_material3 : null_handle) : Track.m_material2; };
	auto const startspeed{editor_track::velocity(Start)};
	auto const alike = [&](TTrack const &Track) {
		if (rules.same_rails && Track.m_material1 != Start.m_material1)
			return false;
		if (rules.same_bed && bed(Track) != bed(Start))
			return false;
		if (rules.same_speed && std::abs(editor_track::velocity(Track) - startspeed) > 0.5)
			return false;
		return true;
	};
	auto const exits = [&](TTrack const &Track) {
		std::vector<glm::dvec3> result;
		if (Track.m_paths.empty())
			return result;
		auto const &main{Track.m_paths.front()};
		result = {main.points[segment_data::point::start], main.points[segment_data::point::end]};
		if (Track.eType == tt_Switch && rules.stop == 2)
			for (std::size_t i = 1; i < Track.m_paths.size(); ++i)
				result.push_back(Track.m_paths[i].points[segment_data::point::end]);
		return result;
	};
	std::size_t constexpr limit{20000};
	std::vector<TTrack *> result;
	std::unordered_map<TTrack const *, double> reached{{&Start, 0.0}};
	std::unordered_set<TTrack const *> done;
	using entry = std::pair<double, TTrack *>;
	std::priority_queue<entry, std::vector<entry>, std::greater<entry>> queue;
	queue.push({0.0, &Start});
	while (false == queue.empty() && result.size() < limit)
	{
		auto const [distance, track] = queue.top();
		queue.pop();
		if (false == done.insert(track).second)
			continue;
		result.push_back(track);
		auto const further{distance + track->Length()};
		if (rules.reach > 0.f && further > static_cast<double>(rules.reach))
			continue;
		for (auto const &end : exits(*track))
			for (auto const &[other, point] : editor_track::connected_points(*track, end))
			{
				if (other == nullptr || done.count(other) > 0 || false == rail(*other) || other->eType == tt_Table || false == alike(*other))
					continue;
				if (other->eType == tt_Switch && (rules.stop == 0 || (rules.stop == 1 && point.path != 0)))
					continue;
				auto const known{reached.find(other)};
				if (known != reached.end() && known->second <= further)
					continue;
				reached[other] = further;
				queue.push({further, other});
			}
	}
	return result;
}

std::vector<TTrack *> const &editor_mode::track_spread_preview(TTrack &Start) const
{
	auto &spread{m_spread};
	auto const signature{spread.stop | (spread.same_rails ? 4 : 0) | (spread.same_bed ? 8 : 0) | (spread.same_speed ? 16 : 0) | (static_cast<int>(spread.reach) << 5)};
	if (spread.from != &Start || spread.signature != signature)
	{
		spread.from = &Start;
		spread.signature = signature;
		spread.preview = track_spread_from(Start);
		spread.length = 0.0;
		for (auto const *track : spread.preview)
			spread.length += track->Length();
	}
	return spread.preview;
}

void editor_mode::track_spread_apply(TTrack &Start)
{
	track_set_prune();
	if (m_track_set.empty())
		if (auto *selected{selected_track()}; selected != nullptr && selected != &Start)
			m_track_set.push_back(selected);
	auto const taken{track_spread_from(Start)};
	if (in_track_set(&Start))
	{
		std::unordered_set<TTrack const *> const out(taken.begin(), taken.end());
		m_track_set.erase(std::remove_if(m_track_set.begin(), m_track_set.end(), [&](TTrack const *Track) { return out.count(Track) > 0; }), m_track_set.end());
	}
	else
		for (auto *track : taken)
			if (false == in_track_set(track))
				m_track_set.push_back(track);
	m_spread.from = nullptr;
	m_node = m_track_set.empty() ? nullptr : m_track_set.back();
	m_track_point = {};
	ui()->set_node(m_node);
}

void editor_mode::track_set_grow()
{
	track_set_prune();
	auto const reach{m_spread.reach};
	std::vector<TTrack *> added;
	for (auto *track : m_track_set)
	{
		m_spread.reach = std::max(0.01f, static_cast<float>(track->Length()) - 0.01f);
		for (auto *other : track_spread_from(*track))
			if (false == in_track_set(other) && std::find(added.begin(), added.end(), other) == added.end())
				added.push_back(other);
	}
	m_spread.reach = reach;
	m_spread.from = nullptr;
	m_track_set.insert(m_track_set.end(), added.begin(), added.end());
}

void editor_mode::render_track_spread_ui()
{
	auto &spread{m_spread};
	if (false == ImGui::CollapsingHeader(STR_C("Spread along the joins: Ctrl+Shift+LMB on a path")))
		return;
	char const *const stops[] = {STR_C("Stop before the switches"), STR_C("Through the switches, along their main track"), STR_C("Through everything joined")};
	for (int i = 0; i < IM_ARRAYSIZE(stops); ++i)
		ImGui::RadioButton(stops[i], &spread.stop, i);
	ImGui::TextUnformatted(STR_C("Only to the paths like the clicked one:"));
	ImGui::SameLine();
	ImGui::Checkbox(STR_C("rails"), &spread.same_rails);
	ImGui::SameLine();
	ImGui::Checkbox(STR_C("trackbed"), &spread.same_bed);
	ImGui::SameLine();
	ImGui::Checkbox(STR_C("speed"), &spread.same_speed);
	ImGui::SetNextItemWidth(110.f);
	ImGui::InputFloat(STR_C("Reach along the track, m (0: no limit)"), &spread.reach, 50.f, 500.f, "%.0f");
	spread.reach = std::clamp(spread.reach, 0.f, 100000.f);
	ImGui::TextDisabled("%s", STR_C("Holding Ctrl+Shift shows what the click takes; on a path of the set it takes them out"));
}

void editor_mode::draw_track_spread() const
{
	if (m_intent.what != track_intent::kind::spread || m_spread.from == nullptr)
		return;
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	auto const removing{in_track_set(m_spread.from)};
	auto const colour{removing ? IM_COL32(255, 90, 70, 220) : IM_COL32(90, 220, 255, 220)};
	for (auto const *track : m_spread.preview)
	{
		if (track->m_editorremoved)
			continue;
		for (auto const &path : track->m_paths)
		{
			bezier const curve{path};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 4.0), 4, 48)};
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				projection.line(drawlist, previous, next, colour, 2.5f);
				previous = next;
			}
		}
	}
}

void editor_mode::draw_track_set() const
{
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList()};
	if (m_track_box.active)
	{
		auto const &io{ImGui::GetIO()};
		ImVec2 const from{m_track_box.from.x, m_track_box.from.y};
		drawlist->AddRectFilled(from, io.MousePos, IM_COL32(255, 160, 40, 40));
		drawlist->AddRect(from, io.MousePos, IM_COL32(255, 160, 40, 220), 0.0f, 0, 1.5f);
	}
	if (m_track_set.size() < 2)
		return;
	for (auto const *track : m_track_set)
	{
		if (track->m_editorremoved)
			continue;
		for (auto const &path : track->m_paths)
		{
			bezier const curve{path};
			auto const count{std::clamp(static_cast<int>(curve.plan_length() / 3.0), 6, 64)};
			auto previous{curve.point(0.0)};
			for (int k = 1; k <= count; ++k)
			{
				auto const next{curve.point(static_cast<double>(k) / count)};
				projection.line(drawlist, previous, next, IM_COL32(255, 160, 40, 230), 3.0f);
				previous = next;
			}
		}
	}
}

void editor_mode::track_set_apply(std::function<bool(TTrack &)> const &Change, bool const Rebuild, char const *What)
{
	track_set_prune();
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> changed;
	std::size_t locked{0};
	for (auto *track : m_track_set)
	{
		if (false == scene::Layers.editable(track))
		{
			++locked;
			continue;
		}
		auto const before{editor_track::capture(*track)};
		if (false == Change(*track))
			continue;
		states.emplace_back(track, before);
		changed.push_back(track);
	}
	if (changed.empty())
		return;
	push_track_snapshot(std::move(states));
	for (auto *track : changed)
	{
		if (Rebuild)
			editor_track::commit_parameters(*track);
		else
			track->mark_dirty();
	}
	auto status{format(STR_C("%s set on %zu paths, Ctrl+Z takes it back"), STR_C(What), changed.size())};
	if (locked > 0)
		status += format(STR_C(", %zu in the files which can't be changed are left as they are"), locked);
	ui()->set_status(status, false);
}

void editor_mode::track_set_delete()
{
	track_set_prune();
	std::vector<TTrack *> removed;
	std::size_t kept{0};
	for (auto *track : m_track_set)
	{
		if (false == track->Dynamics.empty() || false == scene::Layers.editable(track))
		{
			++kept;
			continue;
		}
		editor_track::retire(*track);
		removed.push_back(track);
		if (std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) != m_route.chain.tracks.end())
			m_route = {};
	}
	push_track_snapshot({}, {}, removed);
	m_track_set.clear();
	m_node = nullptr;
	ui()->set_node(nullptr);
	m_track_point = {};
	straight_refresh();
	auto status{format(STR_C("%zu paths deleted, Ctrl+Z brings them back"), removed.size())};
	if (kept > 0)
		status += format(STR_C(", %zu with vehicles on them or in the files which can't be changed are left"), kept);
	ui()->set_status(status, false);
}

void editor_mode::render_track_set_ui()
{
	track_set_prune();
	if (m_track_set.size() < 2)
		return;
	auto &fields{m_track_set_fields};
	double length{0.0};
	for (auto const *track : m_track_set)
		length += track->Length();
	ImGui::TextColored(ImVec4(1.0f, 0.65f, 0.2f, 1.0f), STR_C("%zu paths selected, %.1f m"), m_track_set.size(), length);
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Clear")))
	{
		m_track_set.clear();
		return;
	}
	ImGui::TextDisabled("%s", STR_C("The gizmo moves them together; Shift+LMB adds or takes out a path"));
	render_parallel_set_ui();
	if (ImGui::SmallButton(STR_C("Grow by the neighbours")))
		track_set_grow();
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Adds the paths joined to the set, by the rules of the spread"));
	auto const button_width{90.0f};
	ImGui::PushItemWidth(-button_width - ImGui::GetStyle().ItemSpacing.x);
	ImGui::InputDouble("##setvelocity", &fields.velocity, 0.0, 0.0, "%.0f");
	ImGui::SameLine();
	if (ImGui::Button(format("%s##setv", STR_C("Velocity")).c_str(), ImVec2(button_width, 0.0f)))
	{
		auto const velocity{fields.velocity};
		track_set_apply([velocity](TTrack &Track) { editor_track::velocity(Track, velocity); return true; }, false, "Velocity");
	}
	ImGui::InputTextWithHint("##setrail", STR_C("texture of the rails"), fields.rail, sizeof(fields.rail));
	ImGui::SameLine();
	if (ImGui::Button(format("%s##setr", STR_C("Rails")).c_str(), ImVec2(button_width, 0.0f)) && fields.rail[0] != '\0')
	{
		auto const material{editor_track::fetch_material(fields.rail)};
		track_set_apply([material](TTrack &Track) {
			Track.m_material1 = material;
			if (Track.eType == tt_Switch)
				Track.m_material2 = material;
			return true;
		}, true, "Rails");
	}
	ImGui::InputTextWithHint("##settrackbed", STR_C("texture of the trackbed"), fields.trackbed, sizeof(fields.trackbed));
	ImGui::SameLine();
	if (ImGui::Button(format("%s##setb", STR_C("Trackbed")).c_str(), ImVec2(button_width, 0.0f)) && fields.trackbed[0] != '\0')
	{
		auto const material{editor_track::fetch_material(fields.trackbed)};
		track_set_apply([material](TTrack &Track) {
			if (Track.eType == tt_Switch)
			{
				if (Track.SwitchExtension == nullptr)
					return false;
				Track.SwitchExtension->m_material3 = material;
			}
			else
				Track.m_material2 = material;
			return true;
		}, true, "Trackbed");
	}
	ImGui::InputTextWithHint("##setcircuits", STR_C("track circuits, separated by spaces"), fields.circuits, sizeof(fields.circuits));
	ImGui::SameLine();
	if (ImGui::Button(format("%s##setc", STR_C("Circuits")).c_str(), ImVec2(button_width, 0.0f)))
	{
		std::vector<std::string> names;
		std::string const text{fields.circuits};
		std::size_t at{0};
		while (at < text.size())
		{
			auto const end{text.find(' ', at)};
			auto const word{text.substr(at, end == std::string::npos ? std::string::npos : end - at)};
			if (false == word.empty())
				names.push_back(word);
			if (end == std::string::npos)
				break;
			at = end + 1;
		}
		track_set_apply([names](TTrack &Track) { editor_track::isolated(Track, names); return true; }, false, "Circuits");
	}
	char const *environments[] = {"flat", "mountains", "canyon", "tunnel", "bridge", "bank"};
	ImGui::Combo("##setenvironment", &fields.environment, environments, IM_ARRAYSIZE(environments));
	ImGui::SameLine();
	if (ImGui::Button(format("%s##sete", STR_C("Environment")).c_str(), ImVec2(button_width, 0.0f)))
	{
		auto const environment{static_cast<TEnvironmentType>(fields.environment)};
		track_set_apply([environment](TTrack &Track) { Track.eEnvironment = environment; return true; }, false, "Environment");
	}
	ImGui::PopItemWidth();
	if (ImGui::Button(format(STR_C("Delete %zu paths"), m_track_set.size()).c_str()))
		track_set_delete();
	ImGui::Separator();
}

void editor_mode::render_track_set_gizmo()
{
	track_set_prune();
	if (m_track_set.size() < 2)
	{
		m_track_set_using = false;
		return;
	}
	gizmo_frame const frame;
	if (false == m_track_set_using)
	{
		glm::dvec3 centre{0.0};
		for (auto const *track : m_track_set)
			centre += editor_track::pivot(*track);
		m_track_set_pivot = centre / static_cast<double>(m_track_set.size());
		m_track_set_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(m_track_set_pivot - frame.camera));
	}
	glm::mat4 delta(1.0f);
	frame.manipulate(ImGuizmo::TRANSLATE, m_track_set_gizmo, m_gizmo_snap, &delta);
	if (ImGuizmo::IsUsing())
	{
		if (false == m_track_set_using)
		{
			std::string reason;
			if (false == tracks_editable(m_track_set, reason))
			{
				ui()->set_status(reason, true);
				return;
			}
			m_track_set_using = true;
			m_track_drag = m_track_set;
			m_track_set_joints.clear();
			for (auto *track : m_track_set)
			{
				auto joints{editor_track::joints(*track)};
				joints.erase(std::remove_if(joints.begin(), joints.end(), [&](editor_track::joint const &Joint) { return in_track_set(Joint.other); }), joints.end());
				for (auto const &joint : joints)
					if (std::find(m_track_drag.begin(), m_track_drag.end(), joint.other) == m_track_drag.end())
						m_track_drag.push_back(joint.other);
				m_track_set_joints.emplace_back(track, std::move(joints));
			}
			std::vector<std::pair<TTrack *, editor_track::state>> states;
			for (auto *track : m_track_drag)
				states.emplace_back(track, editor_track::capture(*track));
			push_track_snapshot(std::move(states));
		}
		glm::dvec3 const offset{delta[3]};
		if (glm::length(offset) > 1e-9)
		{
			for (auto &entry : m_track_set_joints)
			{
				editor_track::translate(*entry.first, offset);
				editor_track::follow(*entry.first, entry.second);
			}
			m_track_dirty = true;
		}
		commit_track_drag(false);
	}
	else if (m_track_set_using)
	{
		commit_track_drag(true);
		m_track_set_using = false;
		m_track_drag.clear();
		m_track_set_joints.clear();
		straight_refresh();
	}
}

void editor_mode::render_lineside_ui()
{
	struct tab
	{
		char const *label;
		bool *open;
		bool *expand;
		std::function<void()> render;
	};
	std::array<tab, 4> const tabs{{
	    {"Hectometre posts", &m_hekto.open, &m_hekto.expand, [this]() { render_hekto_ui(); }},
	    {"Fouling points", &m_fouling.open, &m_fouling.expand, [this]() { render_fouling_ui(); }},
	    {"Parallel track", &m_parallel.open, &m_parallel.expand, [this]() { render_parallel_ui(); }},
	    {"Vehicle", &m_vehicle.open, &m_vehicle.expand, [this]() { render_vehicle_ui(); }},
	}};
	if (selected_track() == nullptr)
		ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f), "%s", STR_C("LMB on a track: the line the objects go along"));
	if (false == ImGui::BeginTabBar("##lineside"))
		return;
	for (int i = 0; i < static_cast<int>(tabs.size()); ++i)
	{
		auto const &entry{tabs[i]};
		auto const wanted{*entry.expand};
		*entry.expand = false;
		*entry.open = ImGui::BeginTabItem(STR_C(entry.label), nullptr, wanted ? ImGuiTabItemFlags_SetSelected : 0);
		if (false == *entry.open)
			continue;
		m_lineside_tab = i;
		entry.render();
		ImGui::EndTabItem();
	}
	ImGui::EndTabBar();
}
