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

void editor_mode::update_build_tools()
{
	glm::dvec3 const ground{cursor_ground()};
	auto const &io{ImGui::GetIO()};
	glm::vec2 const cursor{io.MousePos.x, io.MousePos.y};
	m_straights.detour_mouse = ground;
	if (m_extend.active)
	{
		screen_projection const projection;
		glm::dvec3 level;
		m_extend.mouse = projection.on_level(io.MousePos, m_extend.point.y, level) && plan_distance(level, m_extend.point) < static_cast<double>(kMaxPlacementDistance) ? level : ground;
		typed_extend();
		m_extend.snap = m_typed.empty() && false == io.KeyShift && m_extend.track != nullptr ? snap_free_end(m_extend.mouse, cursor, m_extend.track, m_extend.track->iCategoryFlag & 15, {m_extend.track}) : editor_track::snap_target{};
	}
	if (m_switch.placing)
		m_switch.mouse = cursor_level(m_switch.point.y);
	auto &lay{m_lay};
	lay.preview.clear();
	lay.preview_error.clear();
	lay.preview_length = 0.0;
	if (false == lay.active)
		return;
	// geometric ray, same as extend: Mouse_Position() is a delayed depth pick and lags the cursor
	screen_projection const lay_projection;
	double const height{false == lay.points.empty() ? lay.points.back().y : lay.start.track != nullptr ? lay.start.position.y : 0.0};
	glm::dvec3 mouse{ground};
	if (m_cursor_override.has_value())
		mouse = glm::dvec3{m_cursor_override->x, height, m_cursor_override->z};
	else if (false == lay_projection.on_level(io.MousePos, height, mouse))
		mouse = ground;
	glm::dvec2 heading;
	bool const straight{io.KeyShift && lay_heading(heading)};
	if (straight)
	{
		auto const &last{lay.points.back()};
		auto const along{std::max(0.0, glm::dot(glm::dvec2{mouse.x - last.x, mouse.z - last.z}, heading))};
		mouse = {last.x + heading.x * along, mouse.y, last.z + heading.y * along};
	}
	lay.mouse = typed_lay(mouse);
	lay.mouse_snap = straight ? editor_track::snap_target{} : m_typed.empty() ? snap_free_end(lay.mouse, cursor, nullptr, kRailCategory, {}) : editor_track::find_free_end(nullptr, kRailCategory, lay.mouse, kLaySnapRadius, {});
	if (lay.points.empty())
		return;
	auto points{lay.points};
	auto end{lay.mouse_snap};
	if (end.track != nullptr && glm::distance(end.position, points.front()) > 1.0)
		points.push_back(end.position);
	else
	{
		end = {};
		if (plan_distance(lay.mouse, points.back()) > 1.0)
			points.push_back(lay.mouse);
	}
	lay.preview = lay_pieces(points, end, lay.preview_length, lay.preview_error);
}

bool editor_mode::free_end(TTrack &Track, int const Path, bool const Atend, extend_tool &Tool)
{
	editor_track::point_ref const point{Path, Atend ? editor_track::point_kind::end : editor_track::point_kind::start};
	if (Path < 0 || Path >= static_cast<int>(Track.m_paths.size()) || editor_track::is_connected(Track, point))
		return false;
	auto const &path{Track.m_paths[Path]};
	auto const &start{path.points[segment_data::point::start]};
	auto const &end{path.points[segment_data::point::end]};
	auto const &control1{path.points[segment_data::point::control1]};
	auto const &control2{path.points[segment_data::point::control2]};
	glm::dvec3 tangent;
	if (Atend)
		tangent = control2 != glm::dvec3{} ? -control2 : end - (start + control1);
	else
		tangent = -(control1 != glm::dvec3{} ? control1 : (end + control2) - start);
	glm::dvec2 const planar{tangent.x, tangent.z};
	if (glm::length(planar) < 1e-6)
		return false;
	Tool.path = Path;
	Tool.atend = Atend;
	Tool.track = &Track;
	Tool.point = editor_track::point_position(Track, point);
	Tool.direction = glm::normalize(planar);
	Tool.grade = tangent.y / glm::length(planar);
	Tool.mouse = Tool.point;
	return true;
}

bool editor_mode::chosen_free_end(extend_tool &Tool)
{
	auto *track{selected_track()};
	if (track == nullptr || (track->eType != tt_Normal && track->eType != tt_Switch) || false == editor_track::is_supported(*track))
		return false;
	if (m_track_point.valid() && editor_track::is_end(m_track_point.kind) && free_end(*track, m_track_point.path, m_track_point.kind == editor_track::point_kind::end, Tool))
		return true;
	for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		for (auto const atend : {true, false})
			if (free_end(*track, i, atend, Tool))
				return true;
	return false;
}

bool editor_mode::start_extend(editor_track::snap_target const &End)
{
	auto *track{End.track};
	if (track == nullptr || (track->eType != tt_Normal && track->eType != tt_Switch) || false == editor_track::is_supported(*track))
		return false;
	extend_tool tool;
	if (false == free_end(*track, End.point.path, End.point.kind == editor_track::point_kind::end, tool))
		return false;
	auto const &io{ImGui::GetIO()};
	m_extend = tool;
	m_extend.origin = End;
	m_extend.pressed = {io.MousePos.x, io.MousePos.y};
	m_extend.active = true;
	return true;
}

bool editor_mode::extend_snap(editor_track::snap_target &Target) const
{
	auto const &tool{m_extend};
	if (false == tool.active || tool.track == nullptr)
		return false;
	Target = tool.snap;
	return Target.track != nullptr && glm::distance(Target.position, tool.point) > 0.5;
}

glm::dvec3 editor_mode::cursor_ground() const
{
	glm::dvec3 const ground{Global.pCamera.Pos + GfxRenderer->Mouse_Position()};
	if (GfxRenderer->Mouse_Hit())
		return ground;
	screen_projection const projection;
	glm::dvec3 level;
	if (projection.on_level(ImGui::GetIO().MousePos, 0.0, level))
		return level;
	return {ground.x, 0.0, ground.z};
}

glm::dvec3 editor_mode::cursor_level(double const Height) const
{
	if (m_cursor_override.has_value())
		return glm::dvec3{m_cursor_override->x, Height, m_cursor_override->z};
	glm::dvec3 const ground{cursor_ground()};
	screen_projection const projection;
	glm::dvec3 level;
	if (false == projection.on_level(ImGui::GetIO().MousePos, Height, level) || (false == Global.editor_tracks && glm::distance(level, glm::dvec3{Global.pCamera.Pos}) > static_cast<double>(kMaxPlacementDistance)))
		return ground;
	return level;
}

editor_track::snap_target editor_mode::snap_free_end(glm::dvec3 const &Near, glm::vec2 const &Screen, TTrack const *Self, int const Category, std::vector<TTrack const *> const &Exclude) const
{
	screen_projection const projection;
	auto const reach{Global.EditorOrtho ? std::max(kJointRange, static_cast<double>(Global.EditorOrthoExtent) * 2.0) : kJointRange};
	editor_track::snap_target result;
	auto best{kSnapPixels * kSnapPixels};
	for (auto const &candidate : editor_track::free_ends(Self, Category, glm::dvec3{Global.pCamera.Pos}, reach, Exclude))
	{
		ImVec2 screen;
		if (false == projection.project(candidate.position, screen))
			continue;
		auto const distance{(screen.x - Screen.x) * (screen.x - Screen.x) + (screen.y - Screen.y) * (screen.y - Screen.y)};
		if (distance < best)
		{
			best = distance;
			result = candidate;
		}
	}
	if (result.track == nullptr && m_track_snap_radius > 0.0f)
		result = editor_track::find_free_end(Self, Category, Near, m_track_snap_radius, Exclude);
	return result;
}

void editor_mode::draw_free_ends(TTrack const *Self, int const Category, std::vector<TTrack const *> const &Exclude) const
{
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const &display{ImGui::GetIO().DisplaySize};
	auto const reach{Global.EditorOrtho ? std::max(kJointRange, static_cast<double>(Global.EditorOrthoExtent) * 2.0) : kJointRange};
	for (auto const &candidate : editor_track::free_ends(Self, Category, glm::dvec3{Global.pCamera.Pos}, reach, Exclude))
	{
		ImVec2 screen;
		if (projection.project(candidate.position, screen) && screen.x >= 0.0f && screen.y >= 0.0f && screen.x <= display.x && screen.y <= display.y)
			drawlist->AddCircle(screen, 7.0f, IM_COL32(255, 60, 255, 170), 12, 1.5f);
	}
}

bool editor_mode::switch_reach(glm::dvec3 const &Ground, glm::dvec3 &Point)
{
	auto *selected{selected_track()};
	if (m_track_tab != track_tab::turnout || m_switch.armed < 0 || selected == nullptr || selected->eType != tt_Normal || selected->m_paths.empty())
		return false;
	auto const &line{current_straight()};
	if (false == line.tracks.empty())
	{
		auto const along{std::clamp(line.along(Ground), 0.0, line.length)};
		Point = line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * along;
	}
	else
	{
		Point = editor_track::point_at(*selected, editor_track::nearest_parameter(*selected, Ground));
	}
	return plan_distance(Point, Ground) <= kSwitchReach;
}

void editor_mode::handle_drag_start(glm::dvec3 const &Anchor)
{
	auto const &io{ImGui::GetIO()};
	m_handle_drag = {};
	m_handle_drag.active = true;
	m_handle_drag.from = {io.MousePos.x, io.MousePos.y};
	m_handle_drag.height = Anchor.y;
	m_handle_drag.position = Anchor;
	screen_projection const projection;
	glm::dvec3 level;
	if (projection.on_level(io.MousePos, Anchor.y, level))
		m_handle_drag.offset = {Anchor.x - level.x, 0.0, Anchor.z - level.z};
}

bool editor_mode::handle_drag_step(glm::dvec3 &Position)
{
	auto &drag{m_handle_drag};
	auto const &io{ImGui::GetIO()};
	if (false == drag.active)
		return false;
	if (false == io.MouseDown[0])
	{
		drag = {};
		return false;
	}
	glm::vec2 const mouse{io.MousePos.x, io.MousePos.y};
	if (false == drag.moved && glm::length(mouse - drag.from) < kDragPixels)
		return false;
	drag.moved = true;
	screen_projection const projection;
	glm::dvec3 level;
	if (projection.on_level(io.MousePos, drag.height, level) && glm::distance(level, glm::dvec3{Global.pCamera.Pos}) <= static_cast<double>(kMaxPlacementDistance))
		drag.position = level + drag.offset;
	Position = drag.position;
	return true;
}

void editor_mode::point_drag_start(editor_track::point_ref const &Point)
{
	auto *track{selected_track()};
	m_track_point = Point;
	m_point_drag = {};
	if (track == nullptr || false == Point.valid() || Point.path >= static_cast<int>(track->m_paths.size()))
		return;
	auto const &io{ImGui::GetIO()};
	m_point_drag.active = true;
	m_point_drag.from = {io.MousePos.x, io.MousePos.y};
	m_point_drag.height = editor_track::point_position(*track, Point).y;
}

void editor_mode::point_drag_update()
{
	auto &drag{m_point_drag};
	if (false == drag.active)
		return;
	auto const &io{ImGui::GetIO()};
	if (false == io.MouseDown[0])
	{
		point_drag_finish();
		return;
	}
	auto *track{selected_track()};
	if (track == nullptr || false == m_track_point.valid() || m_track_point.path >= static_cast<int>(track->m_paths.size()))
	{
		drag = {};
		return;
	}
	glm::vec2 const mouse{io.MousePos.x, io.MousePos.y};
	if (false == drag.moved)
	{
		if (glm::length(mouse - drag.from) < kDragPixels)
			return;
		std::string reason;
		if (false == editor_track::can_edit_geometry(*track, reason))
		{
			ui()->set_status(reason, true);
			drag = {};
			return;
		}
		drag.moved = true;
		m_track_drag = {track};
		m_track_drag_points.clear();
		m_track_joints.clear();
		m_track_snap = {};
		if (editor_track::is_end(m_track_point.kind) && m_track_drag_connected)
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
	screen_projection const projection;
	glm::dvec3 position;
	if (false == projection.on_level(io.MousePos, drag.height, position) || glm::distance(position, glm::dvec3{Global.pCamera.Pos}) > static_cast<double>(kMaxPlacementDistance))
		return;
	m_track_snap = {};
	if (editor_track::is_end(m_track_point.kind) && m_track_drag_points.empty())
	{
		std::vector<TTrack const *> const exclude(m_track_drag.begin(), m_track_drag.end());
		m_track_snap = snap_free_end(position, mouse, track, track->iCategoryFlag & 15, exclude);
		if (m_track_snap.track != nullptr)
			position = m_track_snap.position;
	}
	editor_track::move_point(*track, m_track_point, position);
	for (auto const &connection : m_track_drag_points)
		editor_track::move_point(*connection.first, connection.second, position);
	m_track_dirty = true;
	commit_track_drag(false);
}

void editor_mode::point_drag_finish()
{
	auto const drag{m_point_drag};
	m_point_drag = {};
	if (false == drag.active || false == drag.moved)
		return;
	if (auto *track{selected_track()}; track != nullptr && m_track_snap.track != nullptr && m_track_point.valid())
	{
		editor_track::snap_point(*track, m_track_point, m_track_snap, m_track_align_tangent);
		m_track_dirty = true;
	}
	m_track_snap = {};
	commit_track_drag(true);
	m_track_drag.clear();
	m_track_drag_points.clear();
	m_track_joints.clear();
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
		design.start_grade = tool.grade;
		design.end_grade = grade_of(target.direction);
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
		// a track which doesn't fit to the free end isn't built towards the cursor instead, it'd miss the end
		auto const shape{alignment::compute(design)};
		return shape.valid ? alignment::pieces(shape, design, 0) : result;
	}
	if (false == ImGui::GetIO().KeyShift && tool.track != nullptr && tool.track->eType == tt_Switch && tool.track->m_paths.size() > 1 && tool.path == 1 && tool.atend)
	{
		auto const &diverging{tool.track->m_paths[1]};
		bezier const curve{diverging};
		auto const begin{plan_of(curve.first(0.0))};
		auto const finish{plan_of(curve.first(1.0))};
		auto const turn{glm::length(begin) > 1e-9 && glm::length(finish) > 1e-9 ? geometry::signed_angle(begin, finish) : 0.0};
		if (std::abs(turn) > 1e-4)
		{
			int const side{turn > 0.0 ? 1 : -1};
			auto const radius{curve.plan_length() / std::abs(turn)};
			auto const &direction{tool.direction};
			auto const centre{plan_of(tool.point) + glm::dvec2{-direction.y, direction.x} * (side * radius)};
			auto const angle{geometry::signed_angle(plan_of(tool.point) - centre, plan_of(tool.mouse) - centre) * side};
			if (angle * radius < 0.5)
				return result;
			auto pieces{arc_pieces(tool.point, direction, tool.grade, radius, std::min(angle, glm::pi<double>()), side)};
			for (auto &piece : pieces)
				piece.rolls = {diverging.rolls[1], diverging.rolls[1]};
			return pieces;
		}
	}
	glm::dvec2 const offset{tool.mouse.x - tool.point.x, tool.mouse.z - tool.point.z};
	auto const chord{glm::length(offset)};
	if (chord < 0.5)
		return result;
	auto const &direction{tool.direction};
	auto const along{glm::dot(offset, direction)};
	auto const lateral{direction.x * offset.y - direction.y * offset.x};
	auto const height = [&](double const Distance) { return tool.point.y + tool.grade * Distance; };
	if (false == m_extend_freehand || ImGui::GetIO().KeyCtrl || ImGui::GetIO().KeyShift || std::abs(lateral) < 0.005 * chord + 0.05)
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
	return arc_pieces(tool.point, direction, tool.grade, chord / (2.0 * std::sin(angle * 0.5)), angle, lateral > 0.0 ? 1 : -1);
}

void editor_mode::finish_extend()
{
	auto const &io{ImGui::GetIO()};
	if (glm::length(glm::vec2{io.MousePos.x, io.MousePos.y} - m_extend.pressed) < kDragPixels && m_typed.empty())
	{
		m_extend.active = false;
		if (m_lay.active && m_lay.points.empty() && m_extend.origin.track != nullptr)
		{
			m_lay.status.clear();
			m_lay.start = m_extend.origin;
			m_lay.points.push_back(m_extend.origin.position);
		}
		else if (false == m_lay.active && m_extend.origin.track != nullptr && m_extend.origin.track == selected_track())
			m_track_point = m_extend.origin.point;
		return;
	}
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

void editor_mode::extend_add(int const Side)
{
	extend_tool tool;
	if (false == chosen_free_end(tool))
		return;
	auto const length{std::max(0.5, m_extend_length)};
	std::vector<segment_data> pieces;
	if (Side == 0)
	{
		segment_data path;
		path.points[segment_data::point::start] = tool.point;
		path.points[segment_data::point::end] = {tool.point.x + tool.direction.x * length, tool.point.y + tool.grade * length, tool.point.z + tool.direction.y * length};
		pieces.push_back(path);
	}
	else
	{
		auto const radius{std::max(10.0, m_extend_radius)};
		pieces = arc_pieces(tool.point, tool.direction, tool.grade, radius, std::min(length / radius, glm::radians(180.0)), Side);
	}
	std::vector<TTrack *> created;
	for (auto const &piece : pieces)
		created.push_back(editor_track::create_path(*tool.track, piece));
	editor_track::commit(created);
	push_track_snapshot({}, created);
	m_node = created.back();
	ui()->set_node(m_node);
	m_track_point = {};
	straight_refresh();
}

void editor_mode::render_extend_ui()
{
	extend_tool tool;
	if (false == chosen_free_end(tool))
		return;
	if (false == ImGui::CollapsingHeader(STR_C("Continue from the free end"), ImGuiTreeNodeFlags_DefaultOpen))
		return;
	ImGui::PushItemWidth(100.0f);
	ImGui::DragScalar(STR_C("Length (m)##extend"), ImGuiDataType_Double, &m_extend_length, 0.5f, nullptr, nullptr, "%.1f");
	item_tooltip("Length of the next piece, along the curve for the curves");
	ImGui::SameLine();
	ImGui::DragScalar("R (m)##extend", ImGuiDataType_Double, &m_extend_radius, 5.0f, nullptr, nullptr, "%.0f");
	item_tooltip("Radius of the curves added");
	ImGui::PopItemWidth();
	m_extend_length = std::clamp(m_extend_length, 0.5, 5000.0);
	m_extend_radius = std::clamp(m_extend_radius, 10.0, 100000.0);
	if (ImGui::Button(STR_C("Straight##extend"), ImVec2(100.0f, 0.0f)))
		extend_add(0);
	item_tooltip("Adds a straight at the free end; the new piece gets selected, so the next click goes on from it");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Curve left##extend"), ImVec2(100.0f, 0.0f)))
		extend_add(1);
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Curve right##extend"), ImVec2(100.0f, 0.0f)))
		extend_add(-1);
	ImGui::Checkbox(STR_C("Freehand"), &m_extend_freehand);
	item_tooltip("Dragging from a free end draws a curve through the cursor; off: it draws a straight along the end");
}

std::vector<segment_data> editor_mode::lay_pieces(std::vector<glm::dvec3> const &Points, editor_track::snap_target const &End, double &Length, std::string &Error) const
{
	auto const &tool{m_lay};
	Length = 0.0;
	if (Points.size() < 2)
		return {};
	auto const startdirection{tool.start.track != nullptr ? -plan_of(tool.start.direction) : plan_of(Points[1] - Points[0])};
	auto const enddirection{End.track != nullptr ? plan_of(End.direction) : plan_of(Points.back() - Points[Points.size() - 2])};
	if (glm::length(startdirection) < 1e-3 || glm::length(enddirection) < 1e-3)
	{
		Error = STR_C("Two points lie on one another");
		return {};
	}
	alignment::design design;
	auto const shape{lay_shape(Points, glm::normalize(startdirection), glm::normalize(enddirection), design)};
	if (false == shape.valid)
	{
		Error = shape.errors.empty() ? std::string{STR_C("The track can't be laid through these points")} : shape.errors.front();
		return {};
	}
	Error = shape.warnings.empty() ? std::string{} : shape.warnings.front();
	auto const count = [&](alignment::result const &Shape, alignment::design const &Design) {
		return std::max(alignment::minimum_pieces(Shape, Design), static_cast<std::size_t>(std::ceil(Shape.length / std::max(5.0, tool.piece_length))));
	};
	// the curvature of the track laid has to go on from the one of the track it joins, a jump gets a transition curve between them
	auto const grade{(Points.back().y - Points.front().y) / std::max(1.0, plan_distance(Points.front(), Points.back()))};
	auto const eased = [&](editor_track::snap_target const &Joint, double const Chainage, double const Sign) {
		if (Joint.track == nullptr || Joint.track->eType != tt_Normal)
			return easing{};
		auto result{ease_out_of(*Joint.track, Joint.position, grade * Sign, design.speed, design.norms)};
		auto const own{Sign * curvature_at(shape, Chainage)};
		if (std::abs(own - result.curvature) <= 0.05 * std::abs(result.curvature))
			result.pieces.clear();
		return result;
	};
	auto const in{eased(tool.start, 0.0, 1.0)};
	auto const out{eased(End, shape.length, -1.0)};
	if (false == in.pieces.empty() || false == out.pieces.empty())
	{
		auto points{Points};
		auto starting{design.start_direction};
		auto ending{design.end_direction};
		if (false == in.pieces.empty())
		{
			points.front() = in.end;
			starting = in.direction;
		}
		if (false == out.pieces.empty())
		{
			points.back() = out.end;
			ending = -out.direction;
		}
		alignment::design easeddesign;
		auto const easedshape{lay_shape(points, starting, ending, easeddesign)};
		if (easedshape.valid)
		{
			auto result{in.pieces};
			auto const middle{alignment::pieces(easedshape, easeddesign, count(easedshape, easeddesign))};
			result.insert(result.end(), middle.begin(), middle.end());
			for (auto piece{out.pieces.rbegin()}; piece != out.pieces.rend(); ++piece)
				result.push_back(turned_around(*piece));
			Length = easedshape.length + (in.pieces.empty() ? 0.0 : in.length) + (out.pieces.empty() ? 0.0 : out.length);
			Error = easedshape.warnings.empty() ? std::string{} : easedshape.warnings.front();
			return result;
		}
		Error = STR("The curvature changes abruptly where the track joins, there's no room for a transition curve") + (easedshape.errors.empty() ? std::string{} : ": " + easedshape.errors.front());
	}
	Length = shape.length;
	return alignment::pieces(shape, design, count(shape, design));
}

void editor_mode::lay_click()
{
	auto &tool{m_lay};
	tool.status.clear();
	auto const snap{tool.mouse_snap};
	if (snap.track != nullptr && false == tool.points.empty() && glm::distance(snap.position, tool.points.front()) > 1.0)
	{
		// reaching a free end finishes the track there, in line with the path it joins
		tool.points.push_back(snap.position);
		lay_finish(snap);
		return;
	}
	if (tool.points.empty())
	{
		tool.start = snap;
		tool.points.push_back(snap.track != nullptr ? snap.position : placement_on_ground(tool.mouse));
		return;
	}
	auto const point{placement_on_ground(tool.mouse)};
	if (plan_distance(point, tool.points.back()) < 1.0)
		return;
	tool.points.push_back(point);
	m_typed.clear();
}

void editor_mode::lay_finish(editor_track::snap_target const &End)
{
	auto &tool{m_lay};
	double length;
	std::string error;
	auto const pieces{lay_pieces(tool.points, End, length, error)};
	if (pieces.empty())
	{
		tool.status = error.empty() ? std::string{STR_C("Click at least two points")} : error;
		if (End.track != nullptr && false == tool.points.empty())
			tool.points.pop_back();
		return;
	}
	auto const *style{tool.start.track != nullptr ? tool.start.track : End.track};
	if (style == nullptr && tool.copy_selected)
		style = selected_track();
	std::vector<TTrack *> created;
	for (auto const &piece : pieces)
		created.push_back(style != nullptr ? editor_track::create_path(*style, piece) : editor_track::create_path(tool.style, piece));
	editor_track::commit(created);
	push_track_snapshot({}, created);
	tool.status = format(STR_C("Laid %zu paths, %.1f m"), created.size(), length);
	if (false == error.empty())
		tool.status += "\n" + error;
	{
		std::vector<std::pair<TTrack *, bool>> run;
		if (tool.start.track != nullptr && tool.start.track->eType == tt_Normal)
			run.emplace_back(tool.start.track, tool.start.point.kind == editor_track::point_kind::end);
		for (auto *track : created)
			run.emplace_back(track, true);
		if (End.track != nullptr && End.track->eType == tt_Normal)
			run.emplace_back(End.track, End.point.kind == editor_track::point_kind::start);
		editor_track::curve curve;
		if (run.size() > created.size() && editor_track::analyse_curve(run, m_route.design.norms.gauge, curve) && curve.compound)
			for (std::size_t i = 1; i < curve.arcs.size(); ++i)
				if (curve.arcs[i].transition < 1.0)
				{
					tool.status += "\n" + format(STR_C("Compound curve: R %.0f m meets R %.0f m with no transition curve between them"), curve.arcs[i - 1].radius, curve.arcs[i].radius);
					break;
				}
	}
	if (scene::Layers.active() == null_handle && style == nullptr)
		tool.status += "\nThe scenery isn't opened for editing, the new track won't be saved";
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.points.clear();
	tool.start = {};
	m_node = created.back();
	ui()->set_node(m_node);
	straight_refresh();
}

void editor_mode::lay_enter()
{
	auto &tool{m_lay};
	if (false == m_typed.empty())
	{
		lay_click();
		return;
	}
	if (tool.points.empty())
		return;
	auto const end{tool.mouse_snap};
	if (end.track != nullptr && glm::distance(end.position, tool.points.front()) > 1.0)
	{
		tool.points.push_back(end.position);
		lay_finish(end);
		return;
	}
	auto const point{placement_on_ground(tool.mouse)};
	bool const appended{plan_distance(point, tool.points.back()) > 1.0};
	if (appended)
		tool.points.push_back(point);
	auto const count{tool.points.size()};
	lay_finish({});
	if (appended && tool.points.size() == count)
		tool.points.pop_back();
}

void editor_mode::lay_cancel()
{
	auto &tool{m_lay};
	tool.points.clear();
	tool.start = {};
	tool.preview.clear();
}

void editor_mode::render_lay_ui()
{
	auto &tool{m_lay};
	if (auto *track{selected_track()}; track != nullptr)
	{
		std::string reason;
		if (editor_track::can_edit_geometry(*track, reason))
			render_extend_ui();
	}
	if (ImGui::CollapsingHeader(STR_C("Parallel track"), ImGuiTreeNodeFlags_DefaultOpen))
		render_parallel_ui();
	ImGui::Separator();
	ImGui::PushItemWidth(120.0f);
	ImGui::DragScalar(STR_C("Curve radius (m)"), ImGuiDataType_Double, &tool.radius, 5.0f, nullptr, nullptr, "%.0f");
	item_tooltip("Drag to change, Ctrl+click to type the value");
	tool.radius = std::max(10.0, tool.radius);
	ImGui::DragScalar(STR_C("Speed (km/h)"), ImGuiDataType_Double, &tool.style.velocity, 1.0f, nullptr, nullptr, "%.0f");
	tool.style.velocity = std::clamp(tool.style.velocity, 0.0, 400.0);
	item_tooltip("Speed limit of the new paths, and the design speed of the transition curves and of the cant");
	ImGui::PopItemWidth();
	if (false == ImGui::TreeNode(STR_C("More settings")))
	{
		render_lay_status();
		return;
	}
	ImGui::PushItemWidth(120.0f);
	ImGui::Checkbox(STR_C("Transition curves and cant"), &tool.transitions);
	item_tooltip("Lengths and the cant recommended for the speed and the radius; off: plain circular arcs with no cant");
	ImGui::DragScalar(STR_C("Longest path (m)"), ImGuiDataType_Double, &tool.piece_length, 1.0f, nullptr, nullptr, "%.0f");
	tool.piece_length = std::clamp(tool.piece_length, 5.0, 1000.0);
	ImGui::PopItemWidth();
	ImGui::Checkbox(STR_C("Parameters of the selected track"), &tool.copy_selected);
	item_tooltip("Takes the textures and the other parameters from the selected track.\nA track started or finished at a free end always takes them from the track it joins");
	if (false == tool.copy_selected)
	{
		auto const text_field = [](char const *Label, std::string &Value, char const *Tooltip) {
			char buffer[256];
			std::snprintf(buffer, sizeof(buffer), "%s", Value.c_str());
			if (ImGui::InputText(Label, buffer, sizeof(buffer)))
				Value = buffer;
			item_tooltip(Tooltip);
		};
		ImGui::PushItemWidth(-1.0f);
		text_field("##layrail", tool.style.rail, STR_C("Texture of the rails"));
		text_field("##layballast", tool.style.ballast, STR_C("Texture of the ballast"));
		ImGui::PopItemWidth();
	}
	ImGui::TreePop();
	render_lay_status();
}

void editor_mode::render_lay_status()
{
	auto const &tool{m_lay};
	if (scene::Layers.active() == null_handle)
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), STR_C("The scenery isn't opened for editing: new track won't be saved"));
	if (tool.active && false == tool.preview_error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), "%s", tool.preview_error.c_str());
	if (false == tool.status.empty())
		ImGui::TextWrapped("%s", tool.status.c_str());
}

alignment::result editor_mode::lay_shape(std::vector<glm::dvec3> const &Points, glm::dvec2 const &Startdirection, glm::dvec2 const &Enddirection, alignment::design &design) const
{
	auto const &tool{m_lay};
	design = {};
	design.norms = m_route.design.norms;
	design.shape = m_route.design.shape;
	design.speed = tool.style.velocity > 0.0 ? tool.style.velocity : 100.0;
	design.start = Points.front();
	design.end = Points.back();
	design.start_direction = Startdirection;
	design.end_direction = Enddirection;

	alignment::vertex corner;
	corner.radius = std::max(1.0, tool.radius);
	if (tool.transitions)
	{
		auto const recommended{alignment::recommend(design.speed, corner.radius, design.norms)};
		corner.cant = recommended.cant;
		corner.transition_in = corner.transition_out = recommended.transition;
	}
	if (Points.size() == 2)
	{
		if (false == alignment::collinear(design))
		{
			glm::dvec2 intersection;
			if (alignment::tangent_intersection(design, intersection))
			{
				design.vertices.push_back(corner);
			}
			else
			{
				corner.offset = plan_distance(design.start, design.end) / 3.0;
				design.vertices = {corner, corner};
			}
		}
	}
	else
	{
		for (std::size_t i = 1; i + 1 < Points.size(); ++i)
		{
			corner.position = plan_of(Points[i]);
			design.vertices.push_back(corner);
		}
		if (design.vertices.size() >= 2)
		{
			design.vertices.front().offset = std::max(1.0, glm::dot(plan_of(Points[1] - Points.front()), design.start_direction));
			design.vertices.back().offset = std::max(1.0, glm::dot(plan_of(Points.back() - Points[Points.size() - 2]), design.end_direction));
		}
	}
	return alignment::compute(design);
}

bool editor_mode::lay_heading(glm::dvec2 &Heading) const
{
	auto const &tool{m_lay};
	glm::dvec2 direction{0.0};
	if (tool.points.size() >= 2)
		direction = plan_of(tool.points.back() - tool.points[tool.points.size() - 2]);
	else if (tool.points.size() == 1 && tool.start.track != nullptr)
		direction = -plan_of(tool.start.direction);
	if (glm::length(direction) < 1e-6)
		return false;
	Heading = glm::normalize(direction);
	return true;
}
