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
#include "editor/editorSettings.hpp"
#include "application/editoruilayer.h"
#include "application/editorprojection.h"
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorGroundFollow.hpp"
#include "editor/editorIncludeInfo.hpp"

#include "utilities/Globals.h"
#include "rendering/renderer.h"
#include "world/Track.h"
#include "world/TrackFeatures.h"
#include "scene/scene.h"
#include "scene/scenelayers.h"
#include "simulation/simulation.h"
#include "utilities/Logs.h"
#include "utilities/translation.h"

#include "imgui/imgui.h"
#include "imgui/ImGuizmo.h"
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <set>
#include "application/editormodetrackshared.h"

using namespace track_mode;

void editor_mode::update_build_tools()
{
	glm::dvec3 const ground{cursor_ground()};
	auto const &io{ImGui::GetIO()};
	glm::vec2 const cursor{io.MousePos.x, io.MousePos.y};
	m_straights.detour_mouse = ground;
	auto const joining = [&](editor_track::snap_target const &Target) {
		if (Target.track == m_join_track && Target.point == m_join_point)
			return;
		m_join_track = Target.track;
		m_join_point = Target.point;
		m_join_radius = 0.0;
		m_join_variant = 0;
	};
	bool const pointing{m_cursor_override.has_value() || ImGui::IsMousePosValid()};
	if (m_extend.active)
	{
		if (pointing)
			m_extend.mouse = cursor_terrain(m_extend.point.y);
		m_extend.snap = m_extend.track != nullptr ? snap_free_end(m_extend.mouse, cursor, m_extend.track, m_extend.track->iCategoryFlag & 15, {m_extend.track}) : editor_track::snap_target{};
		if (m_extend.snap.track == nullptr)
			m_extend.mouse.y = EditorSettings.heights() == editorSettings::lay_heights::level ? m_extend.point.y : bed_height(m_extend.mouse, plan_of(m_extend.mouse - m_extend.point), m_extend.track);
		joining(m_extend.snap);
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
	double const height{false == lay.points.empty() ? lay.points.back().y : lay.start.track != nullptr ? lay.start.position.y : ground.y};
	if (pointing)
		lay.mouse = typed_lay(cursor_terrain(height));
	lay.skew.clear();
	if (pointing && ImGui::GetIO().KeyAlt && m_typed.empty() && false == lay.points.empty())
		lay.mouse = lay_angle_snap(lay.mouse, lay.skew);
	lay.mouse_snap = m_typed.empty() ? snap_free_end(lay.mouse, cursor, nullptr, kRailCategory, {}) : editor_track::find_free_end(nullptr, kRailCategory, lay.mouse, kLaySnapRadius, {});
	if (lay.mouse_snap.track == nullptr)
		lay.mouse = lay_bed(lay.mouse);
	if (false == m_extend.active)
		joining(lay.points.empty() ? editor_track::snap_target{} : lay.mouse_snap);
	lay_trim();
	lay.plan = {};
	if (lay.points.empty())
		return;
	// the cursor is the next point, in the mode it would be clicked in
	auto points{lay.points};
	auto modes{lay.modes};
	auto end{lay.mouse_snap};
	if (end.track != nullptr && glm::distance(end.position, points.front()) > 1.0)
	{
		points.push_back(end.position);
		modes.push_back(lay.mode);
	}
	else
	{
		end = {};
		if (plan_distance(lay.mouse, points.back()) > 1.0)
		{
			points.push_back(lay.mouse);
			modes.push_back(lay.mode);
		}
	}
	m_follow_preview = true;
	lay.preview = lay_pieces(points, modes, end, lay.preview_length, lay.preview_error, &lay.plan);
	m_follow_preview = false;
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
	if (track->eType == tt_Switch && End.point.path == 1 && End.point.kind == editor_track::point_kind::end)
		m_extend.line = editor_track::find_straight(*track, m_straights.tolerance).tracks;
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

std::vector<glm::dvec3> const &editor_mode::free_end_points(glm::dvec3 const &Centre, double const Reach) const
{
	auto &cache{m_free_end_cache};
	if (cache.edits != m_track_edits || glm::distance(cache.centre, Centre) > cache.reach * 0.25 || Reach > cache.reach * 1.01 || Reach < cache.reach * 0.5)
	{
		cache.edits = m_track_edits;
		cache.centre = Centre;
		cache.reach = Reach * 1.25;
		cache.points.clear();
		for (auto const &end : editor_track::free_ends(nullptr, kRailCategory, Centre, cache.reach, {}))
			cache.points.push_back(end.position);
	}
	return cache.points;
}

void editor_mode::draw_free_ends(TTrack const *Self, int const Category, std::vector<TTrack const *> const &Exclude) const
{
	screen_projection const projection;
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const &display{ImGui::GetIO().DisplaySize};
	auto const reach{Global.EditorOrtho ? std::max(kJointRange, static_cast<double>(Global.EditorOrthoExtent) * 2.0) : std::min(kJointRange, kDotsRange)};
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

bool editor_mode::handle_drag_step(glm::dvec3 &Position, TTrack const *Ground)
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
	if (projection.on_level(io.MousePos, drag.height, level) && (Global.editor_tracks || glm::distance(level, glm::dvec3{Global.pCamera.Pos}) <= static_cast<double>(kMaxPlacementDistance)))
	{
		drag.position = level + drag.offset;
		if (Ground != nullptr && false == io.KeyShift)
			drag.position.y = bed_height(drag.position, {0.0, 0.0}, Ground);
		else
			drag.position.y = drag.height;
	}
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
		drag.step = m_history.size();
		records_watch_begin();
	}
	screen_projection const projection;
	glm::dvec3 position;
	if (false == projection.on_level(io.MousePos, drag.height, position) || (false == Global.editor_tracks && glm::distance(position, glm::dvec3{Global.pCamera.Pos}) > static_cast<double>(kMaxPlacementDistance)))
		return;
	// an end goes onto the ground at the height of the ballast, as a point laid; with Shift it keeps its height
	if (editor_track::is_end(m_track_point.kind) && false == io.KeyShift)
	{
		auto const &path{track->m_paths[m_track_point.path]};
		auto const tangent{geometry::bezier{path}.first(m_track_point.kind == editor_track::point_kind::end ? 1.0 : 0.0)};
		position.y = bed_height(position, plan_of(tangent), track);
	}
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
	records_watch_end();
	m_track_drag.clear();
	m_track_drag_points.clear();
	m_track_joints.clear();
}

std::optional<alignment::lay_result> editor_mode::eased_layout(TTrack const *From, glm::dvec3 const &Point, glm::dvec2 const &Direction, glm::dvec3 const &Target, double const Radius) const
{
	m_eased_last.reset();
	if (From == nullptr || From->eType != tt_Normal)
		return std::nullopt;
	auto const end{editor_track::bend_at(*From, Point)};
	if (false == end.has_value())
		return std::nullopt;
	alignment::lay_request request;
	request.start.position = Point;
	request.start.direction = Direction;
	request.start.roll = end->roll;
	request.curvature = end->curvature;
	request.target = Target;
	switch (m_track_layout)
	{
	case track_layout::straight: request.kind = alignment::lay_kind::straight; break;
	case track_layout::arc: request.kind = alignment::lay_kind::arc; break;
	case track_layout::straight_arc: request.kind = alignment::lay_kind::straight_arc; break;
	default: request.kind = alignment::lay_kind::arc_straight; break;
	}
	request.radius = Radius;
	request.speed = design_speed(From, nullptr);
	request.piece_length = m_lay.piece_length;
	request.norms = laying_norms();
	if (false == m_eased_cache.has_value() || m_eased_cache->first != request)
		m_eased_cache.emplace(request, alignment::lay_on(request));
	m_eased_last = m_eased_cache->second;
	return m_eased_last;
}

std::vector<segment_data> editor_mode::extend_pieces() const
{
	m_eased_last.reset();
	std::vector<segment_data> result;
	auto const &tool{m_extend};
	editor_track::snap_target target;
	if (extend_snap(target))
	{
		editor_track::branch_link link;
		std::string reason;
		if (plan_branch_link(link, reason))
		{
			m_join_last = {};
			return link.pieces;
		}
		double length;
		std::string error;
		return join_pieces(tool.track, {tool.path, tool.atend ? editor_track::point_kind::end : editor_track::point_kind::start}, tool.point, tool.direction, target, length, error);
	}
	m_join_last = {};
	if (m_track_layout != track_layout::straight && tool.track != nullptr && tool.track->eType == tt_Switch && tool.track->m_paths.size() > 1 && tool.path == 1 && tool.atend)
	{
		auto const &diverging{tool.track->m_paths[1]};
		bezier const curve{diverging};
		auto const begin{plan_of(curve.first(0.0))};
		auto const finish{plan_of(curve.first(1.0))};
		auto const turn{glm::length(begin) > 1e-9 && glm::length(finish) > 1e-9 ? geometry::signed_angle(begin, finish) : 0.0};
		if (auto const radius{editor_track::branch_radius(diverging)}; radius > 0.0)
		{
			int const side{turn > 0.0 ? 1 : -1};
			auto const &direction{tool.direction};
			auto const centre{plan_of(tool.point) + glm::dvec2{-direction.y, direction.x} * (side * radius)};
			auto const angle{geometry::signed_angle(plan_of(tool.point) - centre, plan_of(tool.mouse) - centre) * side};
			if (angle * radius < 0.5)
				return result;
			auto pieces{arc_pieces(tool.point, direction, tool.grade, radius, std::min(angle, glm::pi<double>()), side)};
			for (auto &piece : pieces)
				piece.rolls = {diverging.rolls[1], diverging.rolls[1]};
			follow_ground(pieces, tool.point.y, tool.grade, tool.mouse.y, std::numeric_limits<double>::quiet_NaN(), tool.track);
			return pieces;
		}
	}
	auto radius{m_lay.radius};
	if (auto const values{typed_values()}; false == values.empty() && (m_track_layout == track_layout::arc_straight || m_track_layout == track_layout::straight_arc) && std::abs(values[0]) >= 1.0)
		radius = std::abs(values[0]);
	auto const eased{eased_layout(tool.track, tool.point, tool.direction, tool.mouse, radius)};
	auto pieces{eased.has_value() && false == eased->pieces.empty() ? eased->pieces : editor_track::layout_pieces(m_track_layout, tool.point, tool.direction, tool.grade, tool.mouse, radius)};
	follow_ground(pieces, tool.point.y, tool.grade, tool.mouse.y, std::numeric_limits<double>::quiet_NaN(), tool.track);
	return pieces;
}

std::vector<segment_data> editor_mode::extend_preview() const
{
	m_follow_preview = true;
	auto pieces{extend_pieces()};
	m_follow_preview = false;
	return pieces;
}

TTrack const *editor_mode::speed_track(TTrack const *From, TTrack const *To) const
{
	if (m_lay.own_speed && m_lay.style.velocity > 0.0)
		return nullptr;
	TTrack const *result{nullptr};
	for (auto const *track : {From, To})
		if (track != nullptr && editor_track::velocity(*track) > 0.0 && (result == nullptr || editor_track::velocity(*track) < editor_track::velocity(*result)))
			result = track;
	return result;
}

double editor_mode::design_speed(TTrack const *From, TTrack const *To) const
{
	if (auto const *track{speed_track(From, To)}; track != nullptr)
		return editor_track::velocity(*track);
	return m_lay.style.velocity > 0.0 ? m_lay.style.velocity : 100.0;
}

std::pair<TTrack const *, TTrack const *> editor_mode::laying_ends() const
{
	if (m_extend.active)
	{
		editor_track::snap_target target;
		return {m_extend.track, extend_snap(target) ? target.track : nullptr};
	}
	if (m_lay.active && false == m_lay.points.empty())
		return {m_lay.start.track, m_lay.points.size() == 1 ? m_lay.mouse_snap.track : nullptr};
	return {nullptr, nullptr};
}

std::string editor_mode::design_speed_of(TTrack const *From, TTrack const *To) const
{
	auto const speed{design_speed(From, To)};
	if (auto const *track{speed_track(From, To)}; track != nullptr)
		return format(STR_C("design speed %.0f km/h, that of the track %s"), speed, name_of(*track).c_str());
	return format(STR_C("design speed %.0f km/h, set for the laying"), speed);
}

void editor_mode::give_own_speed(std::vector<TTrack *> const &Created) const
{
	if (false == m_lay.own_speed || m_lay.style.velocity <= 0.0)
		return;
	for (auto *track : Created)
		editor_track::velocity(*track, m_lay.style.velocity);
}

std::vector<segment_data> editor_mode::join_pieces(TTrack const *From, editor_track::point_ref const &Frompoint, glm::dvec3 const &Start, glm::dvec2 const &Direction, editor_track::snap_target const &End, double &Length,
                                                   std::string &Error) const
{
	alignment::join_request request;
	request.start.position = Start;
	request.start.direction = Direction;
	request.start.roll = From != nullptr ? -editor_track::joint_roll(*From, Frompoint) : 0.0;
	request.end.position = End.position;
	request.end.direction = plan_of(End.direction);
	request.end.roll = End.track != nullptr ? editor_track::joint_roll(*End.track, End.point) : 0.0;
	request.radius = m_join_radius;
	request.variant = m_join_variant;
	request.transitions = true;
	request.speed = design_speed(From, End.track);
	request.kink = m_joints.kink_angle() * 0.5;
	request.shape = m_route.design.shape;
	request.norms = laying_norms();
	auto result{alignment::join(request)};
	if (result.kind != alignment::join_kind::straight && result.kind != alignment::join_kind::arc && glm::length(request.end.direction) > 1e-9)
	{
		auto const grade{(End.position.y - Start.y) / std::max(1.0, plan_distance(Start, End.position))};
		auto const in{From != nullptr && From->eType == tt_Normal ? editor_track::ease_out_of(*From, Start, grade, request.speed, request.norms) : editor_track::easing{}};
		auto const out{End.track != nullptr && End.track->eType == tt_Normal ? editor_track::ease_out_of(*End.track, End.position, -grade, request.speed, request.norms) : editor_track::easing{}};
		if (auto eased{eased_join(request, in, out)})
			result = std::move(*eased);
	}
	Length = result.length;
	std::vector<std::string> notes;
	for (auto const &warning : result.warnings)
		if (m_join_radius > 0.0 || std::find(result.radius_warnings.begin(), result.radius_warnings.end(), warning) == result.radius_warnings.end())
			notes.push_back(warning);
	if (result.kind == alignment::join_kind::bezier)
		notes.insert(notes.begin(), STR_C("No curve fits between the ends, they are joined with a free curve"));
	Error = lines_of(notes);
	m_join_last = result;
	return result.pieces;
}

void editor_mode::finish_extend()
{
	auto const &io{ImGui::GetIO()};
	if (glm::length(glm::vec2{io.MousePos.x, io.MousePos.y} - m_extend.pressed) < kDragPixels)
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
	std::string reason;
	if (editor_track::branch_link link; plan_branch_link(link, reason))
	{
		finish_branch_link(link);
		return;
	}
	if (false == reason.empty())
		ui()->set_status(reason, true);
	crossover_plan plan;
	editor_track::snap_target target;
	if (false == extend_snap(target) && plan_crossover(plan))
	{
		auto *origin{m_extend.track};
		m_extend.active = false;
		std::vector<std::pair<TTrack *, editor_track::state>> states;
		std::vector<TTrack *> created;
		std::vector<TTrack *> removed;
		if (false == place_switch_on_straight(plan.target, plan.shape, origin, plan.along, plan.direction, plan.side, false, states, created, removed, switch_name_for_new()))
			return;
		auto *placed{*std::find_if(created.begin(), created.end(), [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; })};
		if (glm::distance(plan.insert.points[segment_data::point::start], plan.insert.points[segment_data::point::end]) > 0.01)
		{
			auto *insert{editor_track::create_path(*origin, plan.insert)};
			editor_track::commit({insert});
			created.push_back(insert);
		}
		push_track_snapshot(std::move(states), std::move(created), std::move(removed));
		straight_refresh();
		if (place_switch_drive(*placed))
			join_last_steps();
		advance_switch_name();
		return;
	}
	auto const pieces{extend_pieces()};
	auto *style{m_extend.track};
	m_extend.active = false;
	if (pieces.empty() || style == nullptr)
		return;
	// the track laid takes only its heights from the ground: the terrain is shaped under it by the buttons of the terrain model
	m_nmt_state.laying = true;
	std::vector<TTrack *> created;
	for (auto const &piece : pieces)
		created.push_back(editor_track::create_path(*style, piece));
	give_own_speed(created);
	editor_track::commit(created);
	digitize_drawn(created);
	auto const from{m_history.size()};
	push_track_snapshot({}, created);
	first_step_done(first_step::laid);
	if (target.track != nullptr)
		first_step_done(first_step::join);
	m_node = created.back();
	ui()->set_node(m_node);
	nmt_fit_laid(from, created.back()->m_paths.front().points[segment_data::point::end]);
	m_nmt_state.laying = false;
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
	digitize_drawn(created);
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
	if (false == ImGui::CollapsingHeader(STR_C("Continue from the free end")))
		return;
	if (begin_fields("##extendfields", 0.3f))
	{
		field_row("Length (m)##extend");
		ImGui::DragScalar("##extendlength", ImGuiDataType_Double, &m_extend_length, 0.5f, nullptr, nullptr, "%.1f");
		item_tooltip("Length of the next piece, along the curve for the curves");
		field_row("R (m)##extend");
		ImGui::DragScalar("##extendradius", ImGuiDataType_Double, &m_extend_radius, 5.0f, nullptr, nullptr, "%.0f");
		item_tooltip("Radius of the curves added");
		ImGui::EndTable();
	}
	m_extend_length = std::clamp(m_extend_length, 0.5, 5000.0);
	m_extend_radius = std::clamp(m_extend_radius, 10.0, 100000.0);
	auto const gap{std::round(ImGui::GetFontSize() * 0.3f)};
	auto const width{std::floor((ImGui::GetContentRegionAvail().x - 2.0f * gap) / 3.0f)};
	auto const at = [](ImVec2 const Low, ImVec2 const High, float const X, float const Y) { return ImVec2(Low.x + (High.x - Low.x) * X, Low.y + (High.y - Low.y) * Y); };
	auto const piece = [&](char const *Id, char const *Label, int const Bend) {
		return sketch_tile(Id, Label, false, width, [&](ImDrawList &List, ImVec2 Low, ImVec2 High, ImU32 Line, ImU32) {
			List.AddLine(at(Low, High, 0.0f, 0.5f), at(Low, High, 0.4f, 0.5f), Line, 2.0f);
			List.AddCircleFilled(at(Low, High, 0.4f, 0.5f), 2.5f, IM_COL32(230, 90, 70, 255));
			if (Bend == 0)
				List.AddLine(at(Low, High, 0.4f, 0.5f), at(Low, High, 1.0f, 0.5f), IM_COL32(255, 210, 60, 255), 2.0f);
			else
				List.AddBezierQuadratic(at(Low, High, 0.4f, 0.5f), at(Low, High, 0.75f, 0.5f), at(Low, High, 0.95f, Bend > 0 ? 0.08f : 0.92f), IM_COL32(255, 210, 60, 255), 2.0f);
		});
	};
	if (piece("##extendstraight", STR_C("Straight"), 0))
		extend_add(0);
	item_tooltip("Adds a straight at the free end; the new piece gets selected, so the next click goes on from it");
	ImGui::SameLine(0.0f, gap);
	if (piece("##extendleft", STR_C("Curve left"), 1))
		extend_add(1);
	ImGui::SameLine(0.0f, gap);
	if (piece("##extendright", STR_C("Curve right"), -1))
		extend_add(-1);
}

// the shapes laid from a free end as four tiles, each with a sketch: the end of the track, what's laid from it, the cursor
void editor_mode::render_layout_combo()
{
	char const *const names[]{"Arc, then straight", "Straight, then arc", "Arc through the cursor", "Straight"};
	auto const current{static_cast<int>(m_track_layout)};
	ImGui::TextDisabled("%s", STR_C("Shape from a free end"));
	auto const font{ImGui::GetFontSize()};
	auto const gap{std::round(font * 0.3f)};
	auto const width{std::floor((ImGui::GetContentRegionAvail().x - 3.0f * gap) * 0.25f)};
	auto const height{std::round(font * 2.4f)};
	auto *list{ImGui::GetWindowDrawList()};
	auto const accent{family_colour(0)};
	for (int i = 0; i < 4; ++i)
	{
		if (i > 0)
			ImGui::SameLine(0.0f, gap);
		ImGui::PushID(i);
		auto const top{ImGui::GetCursorScreenPos()};
		if (ImGui::InvisibleButton("##shape", ImVec2(width, height)))
			m_track_layout = static_cast<track_layout>(i);
		auto const hovered{ImGui::IsItemHovered()};
		if (hovered)
			ImGui::SetTooltip("%s\n%s", STR_C(names[i]), STR_C("While laying from a free end: Shift+wheel"));
		ImGui::PopID();
		auto const chosen{i == current};
		ImVec2 const bottom{top.x + width, top.y + height};
		auto const rounding{std::round(font * 0.35f)};
		list->AddRectFilled(top, bottom, chosen ? (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 46) : ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), rounding);
		list->AddRect(top, bottom, chosen ? accent : ImGui::GetColorU32(ImGuiCol_Border), rounding, 0, chosen ? 2.0f : 1.0f);
		auto const pad{std::round(font * 0.45f)};
		auto const at = [&](float const X, float const Y) { return ImVec2(top.x + pad + X * (width - 2.0f * pad), top.y + pad + Y * (height - 2.0f * pad)); };
		auto const laid{chosen ? accent : ImGui::GetColorU32(ImGuiCol_Text, 0.7f)};
		list->AddLine(at(0.0f, 0.85f), at(0.18f, 0.85f), ImGui::GetColorU32(ImGuiCol_TextDisabled), 3.0f);
		switch (i)
		{
		case 0:
			list->AddBezierQuadratic(at(0.18f, 0.85f), at(0.4f, 0.85f), at(0.61f, 0.6f), laid, 2.0f);
			list->AddLine(at(0.61f, 0.6f), at(1.0f, 0.1f), laid, 2.0f);
			break;
		case 1:
			list->AddLine(at(0.18f, 0.85f), at(0.45f, 0.85f), laid, 2.0f);
			list->AddBezierQuadratic(at(0.45f, 0.85f), at(0.85f, 0.85f), at(1.0f, 0.1f), laid, 2.0f);
			break;
		case 2: list->AddBezierQuadratic(at(0.18f, 0.85f), at(0.7f, 0.85f), at(1.0f, 0.1f), laid, 2.0f); break;
		default:
			list->AddLine(at(0.18f, 0.85f), at(1.0f, 0.85f), laid, 2.0f);
			for (float y{0.15f}; y < 0.8f; y += 0.2f)
				list->AddLine(at(1.0f, y), at(1.0f, y + 0.1f), IM_COL32(255, 210, 60, 160), 1.0f);
			break;
		}
		list->AddCircleFilled(at(0.18f, 0.85f), 2.5f, IM_COL32(230, 80, 230, 255));
		auto const cursor{at(1.0f, 0.1f)};
		list->AddLine(ImVec2(cursor.x - 4.0f, cursor.y), ImVec2(cursor.x + 4.0f, cursor.y), IM_COL32(255, 210, 60, 255), 1.5f);
		list->AddLine(ImVec2(cursor.x, cursor.y - 4.0f), ImVec2(cursor.x, cursor.y + 4.0f), IM_COL32(255, 210, 60, 255), 1.5f);
	}
	ImGui::TextUnformatted(STR_C(names[current]));
	ImGui::SameLine();
	ImGui::TextDisabled("%s", STR_C("(Shift+wheel)"));
}

int editor_mode::wheel_step(double const Offset, bool const Gesture)
{
	auto const &io{ImGui::GetIO()};
	auto const now{ImGui::GetTime()};
	int const keys{(io.KeyCtrl ? 1 : 0) | (io.KeyShift ? 2 : 0) | (io.KeyAlt ? 4 : 0)};
	auto &wheel{m_wheel};
	if (now - wheel.time > 0.3 || keys != wheel.keys || (wheel.rest != 0.0 && (wheel.rest > 0.0) != (Offset > 0.0)))
	{
		wheel.rest = 0.0;
		wheel.spent = false;
	}
	wheel.time = now;
	wheel.keys = keys;
	wheel.rest += Offset;
	if ((Gesture && wheel.spent) || std::abs(wheel.rest) < 1.0 - 1e-6)
		return 0;
	int const step{wheel.rest > 0.0 ? 1 : -1};
	wheel.rest -= step;
	wheel.spent = true;
	return step;
}

bool editor_mode::track_wheel(double const Offset)
{
	auto const &io{ImGui::GetIO()};
	if (m_switch.placing && m_switch.connection >= 0 && Offset != 0.0 && (io.KeyCtrl || io.KeyShift || io.KeyAlt))
	{
		auto &tool{m_switch};
		int const step{wheel_step(Offset, false == io.KeyCtrl)};
		if (step == 0)
			return true;
		if (io.KeyShift)
			tool.connection = (tool.connection + step + connection::kind_count) % connection::kind_count;
		else if (io.KeyCtrl)
		{
			static double const radii[]{0.0, 150.0, 190.0, 250.0, 300.0, 400.0, 500.0, 600.0, 800.0, 1000.0, 1200.0, 1500.0, 2000.0, 3000.0};
			int index{0};
			for (int i = 0; i < static_cast<int>(std::size(radii)); ++i)
				if (radii[i] <= tool.connection_radius + 1e-6)
					index = i;
			tool.connection_radius = radii[std::clamp(index + step, 0, static_cast<int>(std::size(radii)) - 1)];
		}
		else
			tool.connection_together = false == tool.connection_together;
		return true;
	}
	if ((false == io.KeyCtrl && false == io.KeyShift) || Offset == 0.0)
		return false;
	bool const laying{m_lay.active && false == m_lay.points.empty()};
	if (false == m_extend.active && false == laying)
		return false;
	int const step{wheel_step(Offset, false == io.KeyCtrl)};
	if (step == 0)
		return true;
	editor_track::snap_target target;
	bool const joined{m_extend.active ? extend_snap(target) : m_lay.mouse_snap.track != nullptr && m_lay.start.track != nullptr};
	static double const radii[]{100.0, 150.0, 190.0, 250.0, 300.0, 400.0, 500.0, 600.0, 800.0, 1000.0, 1200.0, 1500.0, 2000.0, 3000.0, 4000.0, 6000.0};
	auto const next_radius = [&](double const Current) {
		if (step > 0)
		{
			for (auto const radius : radii)
				if (radius > Current * 1.001)
					return radius;
			return std::end(radii)[-1];
		}
		for (auto radius{std::rbegin(radii)}; radius != std::rend(radii); ++radius)
			if (*radius < Current * 0.999)
				return *radius;
		return radii[0];
	};
	first_step_done(io.KeyCtrl ? first_step::wheel : first_step::shape);
	if (io.KeyCtrl && io.KeyShift)
	{
		static double const speeds[]{20.0, 30.0, 40.0, 50.0, 60.0, 70.0, 80.0, 90.0, 100.0, 120.0, 140.0, 160.0, 200.0, 250.0};
		auto const [from, to]{laying_ends()};
		auto const current{design_speed(from, to)};
		int index{0};
		for (int i = 0; i < static_cast<int>(std::size(speeds)); ++i)
			if (speeds[i] <= current + 1e-6)
				index = i;
		if (step < 0 && speeds[index] < current - 1e-6)
			++index;
		m_lay.style.velocity = speeds[std::clamp(index + step, 0, static_cast<int>(std::size(speeds)) - 1)];
		m_lay.own_speed = true;
		return true;
	}
	if (io.KeyCtrl)
	{
		if (false == joined)
			m_lay.radius = next_radius(m_lay.radius);
		else if (m_join_radius <= 0.0)
			m_join_radius = next_radius(m_join_last.radius > 0.0 ? m_join_last.radius : 500.0);
		else if (step > 0 && m_join_radius >= std::end(radii)[-1])
			m_join_radius = 0.0;
		else
			m_join_radius = next_radius(m_join_radius);
		return true;
	}
	if (joined)
		m_join_variant += step;
	else
		m_track_layout = static_cast<track_layout>((static_cast<int>(m_track_layout) + 4 - step) % 4);
	return true;
}

std::vector<segment_data> editor_mode::lay_pieces(std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes, editor_track::snap_target const &End, double &Length, std::string &Error,
                                                  lay_line_plan *Plan) const
{
	auto const &tool{m_lay};
	Length = 0.0;
	m_eased_last.reset();
	if (Points.size() < 2)
		return {};
	auto const unknown{std::numeric_limits<double>::quiet_NaN()};
	if (Points.size() == 2 && End.track != nullptr)
	{
		// two free ends: the curve between them is the one the join finds
		if (tool.start.track != nullptr)
		{
			auto const leaving{-plan_of(tool.start.direction)};
			if (glm::length(leaving) < 1e-6)
			{
				Error = STR_C("Two points lie on one another");
				return {};
			}
			return join_pieces(tool.start.track, tool.start.point, Points[0], glm::normalize(leaving), End, Length, Error);
		}
		// from a point to a free end: the shape of the layout laid back from the end
		m_join_last = {};
		std::vector<segment_data> pieces;
		if (glm::length(plan_of(End.direction)) > 1e-6)
		{
			auto const eased{eased_layout(End.track, End.position, glm::normalize(-plan_of(End.direction)), Points[0], tool.radius)};
			auto const laid{eased.has_value() && false == eased->pieces.empty()};
			auto const back{laid ? eased->pieces : editor_track::layout_pieces(m_track_layout, End.position, glm::normalize(-plan_of(End.direction)), -grade_of(End.direction), Points[0], tool.radius)};
			for (auto piece{back.rbegin()}; piece != back.rend(); ++piece)
				pieces.push_back(reversed(*piece));
			Error = laid ? narrowed(m_track_layout, tool.radius, eased->radius) : narrowed(pieces, m_track_layout, tool.radius);
			follow_ground(pieces, Points[0].y, unknown, End.position.y, grade_of(End.direction), End.track);
		}
		for (auto const &piece : pieces)
			Length += bezier{piece}.plan_length();
		if (pieces.empty())
			Error = STR_C("The track can't be laid through these points");
		return pieces;
	}
	m_join_last = {};
	lay_line_plan plan;
	if (false == lay_line(Points, Modes, End, plan, Error))
		return {};
	auto const &design{plan.design};
	auto const &shape{plan.shape};
	Error = lines_of(shape.warnings);
	auto const count = [&](alignment::result const &Shape, alignment::design const &Design) {
		return std::max(alignment::minimum_pieces(Shape, Design), static_cast<std::size_t>(std::ceil(Shape.length / std::max(5.0, tool.piece_length))));
	};
	std::vector<segment_data> pieces;
	// the curvature of the track laid has to go on from the one of the track it joins, a jump gets a transition curve between them
	auto const grade{(Points.back().y - Points.front().y) / std::max(1.0, plan_distance(Points.front(), Points.back()))};
	auto const eased = [&](editor_track::snap_target const &Joint, double const Chainage, double const Sign) {
		if (Joint.track == nullptr || Joint.track->eType != tt_Normal)
			return editor_track::easing{};
		auto result{editor_track::ease_out_of(*Joint.track, Joint.position, grade * Sign, design.speed, design.norms)};
		auto const own{Sign * curvature_at(shape, Chainage)};
		if (std::abs(own - result.curvature) <= 0.05 * std::abs(result.curvature))
			result.pieces.clear();
		return result;
	};
	auto const in{eased(tool.start, 0.0, 1.0)};
	auto const out{eased(End, shape.length, -1.0)};
	if ((false == in.pieces.empty() || false == out.pieces.empty()) && editor_track::eases_room(in, out, Points.front(), design.start_direction, Points.back(), design.end_direction))
	{
		auto easeddesign{design};
		if (false == in.pieces.empty())
		{
			easeddesign.start = in.end;
			easeddesign.start_direction = in.direction;
		}
		if (false == out.pieces.empty())
		{
			easeddesign.end = out.end;
			easeddesign.end_direction = -out.direction;
		}
		// the bends stay where they are, measured from the ends moved
		if (easeddesign.vertices.size() >= 2 && shape.vertices.size() == easeddesign.vertices.size())
		{
			easeddesign.vertices.front().offset = std::max(0.1, glm::dot(shape.vertices.front() - plan_of(easeddesign.start), glm::normalize(easeddesign.start_direction)));
			easeddesign.vertices.back().offset = std::max(0.1, glm::dot(plan_of(easeddesign.end) - shape.vertices.back(), glm::normalize(easeddesign.end_direction)));
		}
		auto const easedshape{alignment::ease_corners(easeddesign, plan.corners, {EditorSettings.corner_low(), EditorSettings.corner_high()}, bends_fit())};
		if (easedshape.valid)
		{
			Length = editor_track::eased_length(in, easedshape.length, out);
			Error = lines_of(easedshape.warnings);
			pieces = editor_track::eased_pieces(in, alignment::pieces(easedshape, easeddesign, count(easedshape, easeddesign)), out);
			plan.design = easeddesign;
			plan.shape = easedshape;
		}
		else
			Error = STR("The curvature changes abruptly where the track joins, there's no room for a transition curve") + (easedshape.errors.empty() ? std::string{} : ": " + easedshape.errors.front());
	}
	if (pieces.empty())
	{
		Length = shape.length;
		pieces = alignment::pieces(shape, design, count(shape, design));
	}
	// the heights: fitted to the ground, or from point to point through the points clicked
	TTrack const *style{tool.start.track != nullptr ? tool.start.track : End.track != nullptr ? End.track : tool.copy_selected ? selected_track() : nullptr};
	auto const grade0{tool.start.track != nullptr ? -grade_of(tool.start.direction) : unknown};
	auto const grade1{End.track != nullptr ? grade_of(End.direction) : unknown};
	if (EditorSettings.heights() == editorSettings::lay_heights::fitted)
		follow_ground(pieces, Points.front().y, grade0, Points.back().y, grade1, style);
	else
		ground_follow::follow_points(pieces, std::vector<glm::dvec3>(Points.begin() + 1, Points.end() - 1), {Points.front().y, grade0, Points.back().y, grade1});
	if (Plan != nullptr)
		*Plan = std::move(plan);
	return pieces;
}

bool editor_mode::lay_line(std::vector<glm::dvec3> const &Points, std::vector<lay_mode> const &Modes, editor_track::snap_target const &End, lay_line_plan &Plan, std::string &Error) const
{
	auto const &tool{m_lay};
	Plan = {};
	auto &design{Plan.design};
	design.norms = laying_norms();
	design.shape = m_route.design.shape;
	design.speed = design_speed(tool.start.track, End.track);
	design.start = Points.front();
	auto const joined{End.track != nullptr && glm::length(plan_of(End.direction)) > 1e-6};
	// the points between the start and the end: a free end reached is the last point
	auto const inner{Points.size() - (joined ? 1 : 0)};
	auto const mode_of = [&](std::size_t const Index) { return Index < Modes.size() ? Modes[Index] : lay_mode::straights; };
	auto const lifted = [](glm::dvec2 const &Point, double const Height) { return glm::dvec3{Point.x, Height, Point.y}; };
	auto const apart = [&](glm::dvec2 const &Direction) {
		if (glm::length(Direction) > 1e-6)
			return true;
		Error = STR_C("Two points lie on one another");
		return false;
	};

	// the straight the line goes on along: a point of it and, where it's fixed, its direction
	auto anchor{Points.front()};
	std::optional<glm::dvec2> along;
	if (tool.start.track != nullptr && glm::length(plan_of(tool.start.direction)) > 1e-6)
		along = glm::normalize(-plan_of(tool.start.direction));
	auto const startdirection{along};
	struct bend
	{
		glm::dvec3 position{0.0};
		alignment::vertex vertex;
		bool corner{true};
	};
	std::vector<bend> bends;
	// a bend clicked after a straight of a fixed direction goes onto it, as far along it as it was clicked
	auto const onto = [&](glm::dvec3 const &Point) {
		auto const distance{std::max(1.0, glm::dot(plan_of(Point - anchor), *along))};
		Plan.guides.emplace_back(anchor, lifted(plan_of(anchor) + *along * (distance + std::max(30.0, 0.2 * distance)), anchor.y));
		return lifted(plan_of(anchor) + *along * distance, Point.y);
	};
	// the straights meet where the two lines cross, ahead of both points
	auto const meeting = [](glm::dvec2 const &From, glm::dvec2 const &Along, glm::dvec2 const &To, glm::dvec2 const &Arriving, glm::dvec2 &Point) {
		auto const determinant{cross(Along, Arriving)};
		if (std::abs(determinant) < 1e-9)
			return false;
		auto const t{cross(To - From, Arriving) / determinant};
		auto const u{cross(To - From, Along) / determinant};
		if (t <= 0.0 || u <= 0.0)
			return false;
		Point = From + Along * t;
		return true;
	};
	auto const straight_bend = [&](glm::dvec3 Point) {
		if (along.has_value())
		{
			Point = onto(Point);
			along.reset();
		}
		bends.push_back({Point, {}, true});
		anchor = Point;
	};

	std::size_t i{1};
	while (i < inner)
	{
		auto last{i};
		while (last < inner && mode_of(last) == lay_mode::curve)
			++last;
		auto const reaching{last == inner && joined};
		// a bend of straights, or a point of a curve too alone to make one
		if (last == i || (last - i < 2 && false == reaching))
		{
			straight_bend(Points[i]);
			++i;
			continue;
		}
		lay_trace trace;
		std::vector<glm::dvec2> traced;
		for (auto k{i}; k < last; ++k)
		{
			trace.points.push_back(Points[k]);
			traced.push_back(plan_of(Points[k]));
		}
		i = last;
		// the straight the curve comes from: the one of the free end or of the curve before, else the one through the last two
		// points clicked before the curve, the last of them then no bend
		auto entry{anchor};
		glm::dvec2 entering;
		// how far back the curve may begin before the point it comes from: up to the bend or the start before it, half the way back
		// to a curve traced before, nothing before a free end
		double behind{0.0};
		if (along.has_value())
		{
			entering = *along;
			if (false == bends.empty())
				behind = 0.5 * plan_distance(entry, bends.back().position);
		}
		else if (false == bends.empty() && bends.back().corner)
		{
			entry = bends.back().position;
			bends.pop_back();
			auto const before{bends.empty() ? Points.front() : bends.back().position};
			entering = plan_of(entry - before);
			behind = plan_distance(entry, before) * (bends.empty() || bends.back().corner ? 1.0 : 0.5);
		}
		else
			entering = traced.front() - plan_of(anchor);
		if (false == apart(entering))
			return false;
		entering = glm::normalize(entering);
		// the straight it goes on along: through the last two points, or the one of the free end the line finishes at
		auto exit{trace.points.back()};
		glm::dvec2 leaving;
		if (reaching)
		{
			exit = End.position;
			leaving = glm::normalize(plan_of(End.direction));
		}
		else
		{
			leaving = traced.back() - traced[traced.size() - 2];
			if (false == apart(leaving))
				return false;
			leaving = glm::normalize(leaving);
		}
		if (trace.points.size() < 3 && false == reaching)
		{
			// no point along the curve itself: the bend where the two straights meet, eased the usual way
			glm::dvec2 corner;
			if (false == meeting(plan_of(entry), entering, plan_of(exit), -leaving, corner))
			{
				Error = STR_C("The straight before the curve and the one after it don't meet");
				return false;
			}
			bends.push_back({lifted(corner, entry.y), {}, true});
		}
		else
		{
			auto base{design};
			base.start = entry;
			base.start_direction = entering;
			base.end = exit;
			base.end_direction = leaving;
			// the curve may begin before the point the straight was clicked at, and end past the last point where the line goes on
			base.start_reserve = behind;
			base.end_reserve = last == inner ? 0.0 : 1e4;
			auto const fitted{traced_stretch(base, traced, tool.radii, tool.straights_between)};
			if (false == fitted.has_value())
			{
				Error = STR_C("Fitting the curve to the points...");
				return false;
			}
			auto const &fit{*fitted};
			if (false == fit.valid)
			{
				Error = STR_C("No curve follows the points clicked along it");
				return false;
			}
			for (auto const &vertex : fit.vertices)
				bends.push_back({lifted(vertex.position, entry.y), vertex, false});
			if (fit.vertices.empty())
			{
				// a straight: the two lines are one
				exit = lifted(plan_of(entry) + entering * glm::dot(plan_of(exit - entry), entering), exit.y);
				leaving = entering;
			}
			trace.offsets = fit.offsets;
			trace.rms = fit.rms;
			trace.worst = fit.worst;
			trace.curves = fit.vertices.size();
		}
		Plan.traces.push_back(std::move(trace));
		anchor = exit;
		along = leaving;
	}

	// the end: the free end reached, the end of a curve traced, or the last point clicked
	glm::dvec3 end;
	glm::dvec2 ending;
	if (joined)
	{
		end = End.position;
		ending = glm::normalize(plan_of(End.direction));
		if (along.has_value() && (false == bends.empty() || false == Plan.traces.empty()))
		{
			// from the straight out of a curve traced to the one of the free end: a bend where they meet
			glm::dvec2 corner;
			if (meeting(plan_of(anchor), *along, plan_of(end), -ending, corner))
				bends.push_back({lifted(corner, anchor.y), {}, true});
		}
	}
	else if (along.has_value())
	{
		end = anchor;
		ending = *along;
	}
	else
	{
		if (bends.empty())
		{
			Error = STR_C("Click at least two points");
			return false;
		}
		end = bends.back().position;
		bends.pop_back();
		ending = plan_of(end) - plan_of(bends.empty() ? Points.front() : bends.back().position);
		if (false == apart(ending))
			return false;
		ending = glm::normalize(ending);
	}
	if (plan_distance(end, Points.front()) < 1.0)
	{
		Error = STR_C("Click at least two points");
		return false;
	}
	design.end = end;
	design.end_direction = ending;
	if (startdirection.has_value())
		design.start_direction = *startdirection;
	else
	{
		auto const first{plan_of(bends.empty() ? end : bends.front().position) - plan_of(Points.front())};
		if (false == apart(first))
			return false;
		design.start_direction = glm::normalize(first);
	}
	for (auto const &bend : bends)
	{
		auto vertex{bend.corner ? alignment::vertex{} : bend.vertex};
		vertex.position = plan_of(bend.position);
		design.vertices.push_back(vertex);
		Plan.corners.push_back(bend.corner ? 1 : 0);
	}
	if (design.vertices.size() >= 2)
	{
		design.vertices.front().offset = glm::dot(plan_of(bends.front().position - design.start), design.start_direction);
		design.vertices.back().offset = glm::dot(plan_of(end - bends.back().position), design.end_direction);
	}
	Plan.shape = alignment::ease_corners(design, Plan.corners, {EditorSettings.corner_low(), EditorSettings.corner_high()}, bends_fit());
	if (false == Plan.shape.valid)
	{
		Error = Plan.shape.errors.empty() ? std::string{STR_C("The track can't be laid through these points")} : Plan.shape.errors.front();
		return false;
	}
	return true;
}

std::optional<alignment::trace_stretch> editor_mode::traced_stretch(alignment::design const &Base, std::vector<glm::dvec2> const &Points, int const Radii, bool const Straights, bool const Meanwhile) const
{
	// the stretches traced before the one the cursor moves along are the same from frame to frame: their fits are kept
	std::vector<double> key{Base.start.x, Base.start.z, Base.start_direction.x, Base.start_direction.y, Base.end.x, Base.end.z, Base.end_direction.x, Base.end_direction.y, Base.speed,
	                        static_cast<double>(Base.shape), static_cast<double>(Radii), Straights ? 1.0 : 0.0, Base.start_reserve, Base.end_reserve};
	for (auto const &point : Points)
	{
		key.push_back(point.x);
		key.push_back(point.y);
	}
	auto &cache{m_stretch_cache};
	for (auto entry{cache.begin()}; entry != cache.end(); ++entry)
		if (entry->first == key)
		{
			std::rotate(cache.begin(), entry, std::next(entry));
			return cache.front().second;
		}
	auto const keep = [&](alignment::trace_stretch const &Fit) {
		cache.insert(cache.begin(), {key, Fit});
		if (cache.size() > 16)
			cache.pop_back();
	};
	alignment::trace_stretch fit;
	if (false == m_follow_preview)
	{
		fit = alignment::fit_trace_stretch(Base, Points, Radii, Straights);
		keep(fit);
		return fit;
	}
	// shown while the cursor moves: fitted on the thread, the frames meanwhile drawn with the fit done last for the same points but
	// the one at the cursor, or with no curve yet
	if (m_stretch_fitter.done(key, fit))
	{
		keep(fit);
		return fit;
	}
	m_stretch_fitter.ask({key, Base, Points, Radii, Straights});
	std::vector<double> last;
	auto const header{std::size_t{14}};
	if (Meanwhile && m_stretch_fitter.latest(last, fit) && last.size() >= header + 2 && key.size() >= header + 2 && std::equal(key.begin(), key.begin() + 4, last.begin()) &&
	    std::equal(key.begin() + 8, key.begin() + header, last.begin() + 8))
	{
		auto const common{std::min(last.size(), key.size()) - 2};
		if (std::equal(key.begin() + header, key.begin() + common, last.begin() + header))
			return fit;
	}
	return {};
}

void editor_mode::lay_trim()
{
	auto &tool{m_lay};
	tool.modes.resize(tool.points.size(), lay_mode::straights);
}

void editor_mode::lay_switch_mode()
{
	auto &tool{m_lay};
	tool.mode = tool.mode == lay_mode::straights ? lay_mode::curve : lay_mode::straights;
	tool.status = tool.mode == lay_mode::curve ? STR("Curve: click points along the middle of the track, the last two on the straight after the curve. Tab: back to the straights")
	                                           : STR("Straights: each point clicked is a bend of the straights, eased with a large curve. Tab: a curve traced on the photo");
	tool.status_time = ImGui::GetTime();
}

void editor_mode::lay_click()
{
	auto &tool{m_lay};
	tool.status.clear();
	lay_trim();
	auto const snap{tool.mouse_snap};
	if (snap.track != nullptr && false == tool.points.empty() && glm::distance(snap.position, tool.points.front()) > 1.0)
	{
		// reaching a free end finishes the track there, in line with the path it joins
		tool.points.push_back(snap.position);
		tool.modes.push_back(tool.mode);
		lay_finish(snap);
		return;
	}
	if (tool.points.empty())
	{
		tool.chain.clear();
		tool.start = snap;
		auto point{snap.track != nullptr ? snap.position : tool.mouse};
		if (auto const *axis{snap.track == nullptr && ImGui::GetIO().KeyAlt ? skew_reference(point, point, kSkewSeat) : nullptr})
		{
			auto const &path{axis->m_paths.front()};
			auto const &from{path.points[segment_data::point::start]};
			auto const &to{path.points[segment_data::point::end]};
			auto const along{std::clamp(glm::dot(plan_of(point - from), plan_of(to - from)) / glm::dot(plan_of(to - from), plan_of(to - from)), 0.0, 1.0)};
			point = from + (to - from) * along;
		}
		tool.points.push_back(point);
		tool.modes.push_back(tool.mode);
		return;
	}
	// the points only make the line shown: it's laid by Enter, or by a click on a free end
	auto const &point{tool.mouse};
	if (plan_distance(point, tool.points.back()) < 1.0)
		return;
	m_typed.clear();
	tool.points.push_back(point);
	tool.modes.push_back(tool.mode);
}

void editor_mode::lay_finish(editor_track::snap_target const &End)
{
	auto &tool{m_lay};
	lay_trim();
	double length;
	std::string error;
	auto const pieces{lay_pieces(tool.points, tool.modes, End, length, error)};
	auto const eased{m_eased_last};
	tool.status_time = ImGui::GetTime();
	if (pieces.empty())
	{
		tool.status = error.empty() ? std::string{STR_C("Click at least two points")} : error;
		if (End.track != nullptr && false == tool.points.empty())
		{
			tool.points.pop_back();
			lay_trim();
		}
		return;
	}
	auto const *style{tool.start.track != nullptr ? tool.start.track : End.track};
	if (style == nullptr && tool.copy_selected)
		style = selected_track();
	// the track laid takes only its heights from the ground: the terrain is shaped under it by the buttons of the terrain model
	m_nmt_state.laying = true;
	std::vector<TTrack *> created;
	for (auto const &piece : pieces)
		created.push_back(style != nullptr ? editor_track::create_path(*style, piece) : editor_track::create_path(tool.style, piece));
	give_own_speed(created);
	editor_track::commit(created);
	digitize_drawn(created);
	if (tool.chain.empty())
		tool.history = m_history.size();
	push_track_snapshot({}, created);
	first_step_done(first_step::laid);
	if (End.track != nullptr)
		first_step_done(first_step::join);
	tool.status = format(STR_P("Laid %zu track, %.1f m, Ctrl+Z takes it back", "Laid %zu tracks, %.1f m, Ctrl+Z takes them back", static_cast<long long>(created.size())), created.size(), length);
	if (false == error.empty())
		tool.status += "\n" + error;
	if (eased.has_value() && eased->compound)
		tool.status += "\n" + format(STR_C("Compound curve: R %.0f m goes on into R %.0f m through a transition curve of %.0f m"), eased->from_radius, eased->radius, eased->entry);
	if (eased.has_value() && false == eased->note.empty())
		tool.status += "\n" + eased->note;
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
		tool.status += "\n" + STR("The scenery isn't opened for editing: new track won't be saved");
	m_node = created.back();
	ui()->set_node(m_node);
	if (nmt_fit_laid(tool.history, created.back()->m_paths.front().points[segment_data::point::end]))
		tool.status += "\n" + STR("The grade line of the whole track laid in a row fitted to the terrain model, as the tracks go onto the terrain");
	m_nmt_state.laying = false;
	WriteLog("Editor: " + tool.status, logtype::generic);
	tool.points.clear();
	tool.modes.clear();
	tool.start = {};
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
	lay_trim();
	if (tool.points.empty())
		return;
	auto const end{tool.mouse_snap};
	if (end.track != nullptr && glm::distance(end.position, tool.points.front()) > 1.0)
	{
		tool.points.push_back(end.position);
		tool.modes.push_back(tool.mode);
		lay_finish(end);
		return;
	}
	// with no point clicked since the last Enter the laying of the chain ends
	if (tool.points.size() < 2)
	{
		lay_cancel();
		return;
	}
	lay_tool::step const previous{tool.start, tool.points.front()};
	lay_finish({});
	if (false == tool.points.empty())
		return;
	// the laying goes on from the end of the track just laid, along it
	auto *last{dynamic_cast<TTrack *>(m_node)};
	if (last == nullptr || last->m_paths.empty())
		return;
	auto const &path{last->m_paths.front()};
	editor_track::snap_target next;
	next.track = last;
	next.point = {0, editor_track::point_kind::end};
	next.position = path.points[segment_data::point::end];
	auto const &control{path.points[segment_data::point::control2]};
	next.direction = glm::normalize(control != glm::dvec3{} ? control : path.points[segment_data::point::start] - next.position);
	tool.chain.push_back(previous);
	tool.start = next;
	tool.points = {next.position};
	tool.modes = {tool.mode};
}

void editor_mode::lay_cancel()
{
	auto &tool{m_lay};
	tool.points.clear();
	tool.modes.clear();
	tool.start = {};
	tool.chain.clear();
	tool.preview.clear();
	tool.plan = {};
	tool.status.clear();
}

void editor_mode::lay_after_history(bool const Undo)
{
	auto &tool{m_lay};
	if (tool.points.empty())
		return;
	if (false == Undo)
	{
		lay_cancel();
		return;
	}
	while (tool.start.track != nullptr && tool.start.track->m_editorremoved)
	{
		if (tool.chain.empty())
		{
			lay_cancel();
			return;
		}
		tool.start = tool.chain.back().start;
		tool.points = {tool.chain.back().point};
		tool.modes = {tool.mode};
		tool.chain.pop_back();
	}
}

void editor_mode::lay_back()
{
	auto &tool{m_lay};
	lay_trim();
	// the points not laid yet go first, then the tracks laid in the chain
	if (tool.points.size() > 1)
	{
		tool.points.pop_back();
		tool.modes.pop_back();
		return;
	}
	if (false == tool.chain.empty())
	{
		undo_last();
		tool.status = STR_C("The last piece was taken back");
		tool.status_time = ImGui::GetTime();
		return;
	}
	tool.points.clear();
	tool.modes.clear();
	tool.start = {};
}

glm::dvec3 editor_mode::lay_angle_snap(glm::dvec3 const &Mouse, std::string &Label) const
{
	Label.clear();
	auto const &last{m_lay.points.back()};
	glm::dvec2 const offset{Mouse.x - last.x, Mouse.z - last.z};
	auto const length{glm::length(offset)};
	if (length < 1.0)
		return Mouse;
	auto const leaving{m_lay.start.track != nullptr ? -plan_of(m_lay.start.direction) : glm::dvec2{0.0}};
	glm::dvec2 reference{glm::length(leaving) > 1e-6 ? glm::normalize(leaving) : glm::dvec2{0.0, 1.0}};
	auto const step{glm::radians(15.0)};
	auto angle{std::round(signed_angle(reference, offset / length) / step) * step};
	if (auto const *axis{skew_reference(last, Mouse, kSkewReach)})
	{
		auto const &path{axis->m_paths.front()};
		reference = glm::normalize(plan_of(path.points[segment_data::point::end] - path.points[segment_data::point::start]));
		auto const relative{signed_angle(reference, offset / length)};
		angle = std::round(relative / step) * step;
		double ratio{0.0};
		for (auto const n : kSkewRatios)
			for (auto const base : {0.0, glm::pi<double>()})
				for (auto const side : {-1.0, 1.0})
				{
					auto const candidate{base + side * std::atan(1.0 / n)};
					if (std::abs(std::remainder(relative - candidate, 2.0 * glm::pi<double>())) < std::abs(std::remainder(relative - angle, 2.0 * glm::pi<double>())))
					{
						angle = candidate;
						ratio = n;
					}
				}
		auto const name{name_of(*axis)};
		auto const degrees{glm::degrees(std::abs(std::remainder(angle, glm::pi<double>())))};
		Label = ratio > 0.0 ? format(STR_C("1:%g to %s"), ratio, name.c_str()) : degrees < 0.1 ? format(STR_C("parallel to %s"), name.c_str()) : format(STR_C("%.0f deg to %s"), degrees, name.c_str());
	}
	auto const heading{turned(reference, angle) * length};
	return {last.x + heading.x, Mouse.y, last.z + heading.y};
}

TTrack const *editor_mode::skew_reference(glm::dvec3 const &From, glm::dvec3 const &To, double const Reach) const
{
	auto const from{plan_of(From)};
	auto const leg{plan_of(To) - from};
	auto const length{glm::length(leg)};
	TTrack const *nearest{nullptr};
	auto best{std::numeric_limits<double>::max()};
	for (auto *section : simulation::Region->sections((From + To) * 0.5, static_cast<float>(std::min(0.5 * length + Reach, 2000.0))))
		for (auto const &cell : section->m_cells)
			for (auto *track : cell.m_directories.paths)
			{
				if (track == nullptr || track->eType != tt_Normal || track->m_editorremoved || (track->iCategoryFlag & 15) != kRailCategory || track->m_paths.size() != 1 ||
				    false == editor_track::is_straight(*track, m_straights.tolerance))
					continue;
				auto const &path{track->m_paths.front()};
				auto const a{plan_of(path.points[segment_data::point::start])};
				auto const axis{plan_of(path.points[segment_data::point::end]) - a};
				auto const span{glm::length(axis)};
				if (span < 1.0)
					continue;
				auto const along{std::clamp(glm::dot(from - a, axis) / span, 0.0, span)};
				auto distance{glm::distance(from, a + axis / span * along)};
				if (distance > Reach)
					distance = std::numeric_limits<double>::max();
				if (auto const determinant{cross(leg, axis)}; length > 1.0 && std::abs(determinant) > 1e-9)
				{
					auto const t{cross(a - from, axis) / determinant};
					auto const u{cross(a - from, leg) / determinant};
					if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0)
						distance = std::min(distance, t * length);
				}
				if (distance < best)
				{
					best = distance;
					nearest = track;
				}
			}
	return nearest;
}

editor_mode::track_hover editor_mode::split_spot() const
{
	auto const *selected{selected_track()};
	if (selected == nullptr || ui()->mode() != nodebank_panel::TRACK || ImGui::GetIO().WantCaptureMouse)
		return {};
	return track_under_cursor(selected, 20.0f);
}

alignment::limits editor_mode::laying_norms() const
{
	auto norms{m_route.design.norms};
	norms.checked = true;
	return norms;
}

void editor_mode::render_lay_ui()
{
	auto &tool{m_lay};
	if (int mode{tool.mode == lay_mode::curve ? 1 : 0};
	    segmented("laymode", mode,
	              {{0, STR_C("Straights   Tab"), "Each point clicked is a bend of the straights: the bend eased with two transition curves meeting, with no cant, of a large radius; where they don't fit, an arc with the transitions and the cant of the norms, as large as fits"},
	               {1, STR_C("Curve   Tab"), "Click points along the middle of a curve seen on the photo, the last two on the straight after it: the curve, compound or with a straight in it, is fitted to them. It comes from the straight before it, of the points clicked before or of the free end"}}) &&
	    (mode == 1) != (tool.mode == lay_mode::curve))
		lay_switch_mode();
	ImGui::PushItemWidth(120.0f);
	if (tool.mode == lay_mode::straights)
	{
		double low{EditorSettings.corner_low()};
		double high{EditorSettings.corner_high()};
		auto changed{ImGui::DragScalar(STR_C("Bends: R from (m)"), ImGuiDataType_Double, &low, 10.0f, nullptr, nullptr, "%.0f")};
		item_tooltip("The radius the two transition curves of a bend meet at, at least; where they don't fit at it, an arc of the norms as large as fits");
		changed |= ImGui::DragScalar(STR_C("to (m)"), ImGuiDataType_Double, &high, 10.0f, nullptr, nullptr, "%.0f");
		item_tooltip("The radius tried first; a bend too slight for transitions long enough at the speed takes a larger one");
		ImGui::PushID("bends");
		auto first{true};
		for (auto const &[from, to] : {std::pair{1000.0, 2000.0}, std::pair{2000.0, 4000.0}, std::pair{3000.0, 6000.0}, std::pair{4000.0, 8000.0}})
		{
			if (false == first)
				ImGui::SameLine();
			first = false;
			auto const chosen{std::abs(low - from) < 0.5 && std::abs(high - to) < 0.5};
			if (chosen)
				ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
			if (ImGui::SmallButton(format("%.0f-%.0f", from, to).c_str()))
			{
				low = from;
				high = to;
				changed = true;
			}
			if (chosen)
				ImGui::PopStyleColor();
		}
		ImGui::PopID();
		if (changed)
		{
			low = std::max(10.0, low);
			EditorSettings.corners(low, std::max(low, high));
			EditorSettings.save();
		}
	}
	else
	{
		ImGui::PopItemWidth();
		segmented("layradii", tool.radii,
		          {{0, STR_C("Auto"), "As few radii as follow the points, up to three"}, {1, STR_C("1 R"), "One radius"}, {2, STR_C("2 R"), "A compound curve of two radii"}, {3, STR_C("3 R"), "A compound curve of three radii"}});
		ImGui::Checkbox(STR_C("Two curves with a straight between them, where the points show one"), &tool.straights_between);
		item_tooltip("The points along the curve may make two curves, turning the same way or not, with a straight between them, where these follow the points much closer than one curve");
		if (auto const &traces{tool.plan.traces}; false == traces.empty() && false == traces.back().offsets.empty())
		{
			auto const &trace{traces.back()};
			auto const colour = [](double const Departure) { return Departure <= 0.1 ? IM_COL32(90, 220, 110, 255) : Departure <= 0.3 ? IM_COL32(255, 200, 60, 255) : IM_COL32(255, 90, 70, 255); };
			metric_tiles({{format("%zu", trace.points.size()), STR_C("points"), ImGui::GetColorU32(ImGuiCol_Text)},
			              {format("%.0f cm", trace.rms * 100.0), STR_C("departure"), colour(trace.rms)},
			              {format("%.0f cm", trace.worst * 100.0), STR_C("at most"), colour(trace.worst)}});
		}
		else
		{
			ImGui::PushTextWrapPos(0.0f);
			ImGui::TextDisabled("%s", STR_C("Click points along the middle of the track on the photo, the last two on the straight after the curve"));
			ImGui::PopTextWrapPos();
		}
		ImGui::PushItemWidth(120.0f);
	}
	ImGui::DragScalar(STR_C("Speed (km/h)"), ImGuiDataType_Double, &tool.style.velocity, 1.0f, nullptr, nullptr, "%.0f");
	item_tooltip("Speed limit of the new paths, and the design speed of the transition curves and of the cant; the speed of the track joined comes first, and none higher than the radius allows");
	value_chips("speeds", tool.style.velocity, {40.0, 60.0, 80.0, 100.0, 120.0, 160.0}, "%.0f");
	tool.style.velocity = std::clamp(tool.style.velocity, 0.0, 400.0);
	ImGui::PopItemWidth();
	ImGui::Checkbox(STR_C("This speed also on from a track with its own"), &tool.own_speed);
	item_tooltip("Off: a track laid on from another one, or joined to it, is designed for the speed of that track and takes it.\nOn: it's designed for the speed above and the new tracks get it.\nWhile laying: Ctrl+Shift+wheel");
	if (auto const [from, to]{laying_ends()}; from != nullptr || to != nullptr)
		ImGui::TextDisabled("%s", design_speed_of(from, to).c_str());
	ImGui::TextDisabled("%s", STR_C("Height of the track on the ground"));
	if (int heights{static_cast<int>(EditorSettings.heights())};
	    segmented("layheights", heights,
	              {{0, STR_C("Fitted"), "The grades fitted to the ground as Tracks onto the terrain fits them: vertical curves of the norms, the bumps of the ground gone past; the terrain is left as it is. On the terrain model the whole track laid in a row is fitted again at each click"},
	               {1, STR_C("Point to point"), "Each point clicked at the height of the ground, an even grade between them: the grade changes only where you click"},
	               {2, STR_C("Flat"), "The track goes on at the height it's laid from, whatever the ground; the first point on the ground"}}))
	{
		EditorSettings.heights(static_cast<editorSettings::lay_heights>(heights));
		EditorSettings.save();
	}
	// the curves of the line shown, those below the norms in the colour of a warning: they're laid all the same
	if (auto const &plan{tool.plan}; tool.active && plan.shape.valid && false == plan.shape.curves.empty())
	{
		ImGui::SeparatorText(STR_C("Curves of the track laid"));
		for (auto const &curve : plan.shape.curves)
		{
			auto const meets{alignment::meets_norms(curve, plan.design)};
			auto const text{format("%d. %s", curve.vertex + 1, curve_label(curve, laying_norms()).c_str())};
			if (meets)
				ImGui::TextUnformatted(text.c_str());
			else
				ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.15f, 1.0f), "%s", text.c_str());
			if (ImGui::IsItemHovered())
			{
				std::string notes;
				auto const tag{format(STR_C("Vertex %zu"), static_cast<std::size_t>(curve.vertex + 1))};
				for (auto const &warning : plan.shape.warnings)
					if (warning.rfind(tag + ":", 0) == 0)
						notes += (notes.empty() ? "" : "\n") + warning;
				ImGui::SetTooltip("%s", notes.empty() ? STR_C("Meets the norms for the speed") : notes.c_str());
			}
		}
	}
	if (auto *track{selected_track()}; track != nullptr)
	{
		std::string reason;
		if (editor_track::can_edit_geometry(*track, reason))
			render_extend_ui();
	}
	if (false == ImGui::TreeNode(STR_C("More settings")))
	{
		render_lay_status();
		return;
	}
	ImGui::TextDisabled("%s", STR_C("Dragged from a free end, or laid from a point to one"));
	ImGui::PushItemWidth(120.0f);
	ImGui::DragScalar(STR_C("Curve radius (m)"), ImGuiDataType_Double, &tool.radius, 5.0f, nullptr, nullptr, "%.0f");
	item_tooltip("Drag to change, Ctrl+click to type the value. While dragging: Ctrl+wheel");
	value_chips("radii", tool.radius, {190.0, 300.0, 500.0, 800.0, 1200.0, 2000.0}, "%.0f");
	tool.radius = std::max(10.0, tool.radius);
	ImGui::PopItemWidth();
	render_layout_combo();
	ImGui::Spacing();
	ImGui::PushItemWidth(120.0f);
	ImGui::DragScalar(STR_C("Longest path (m)"), ImGuiDataType_Double, &tool.piece_length, 1.0f, nullptr, nullptr, "%.0f");
	tool.piece_length = std::clamp(tool.piece_length, 5.0, 1000.0);
	ImGui::PopItemWidth();
	ImGui::TextDisabled("%s", STR_C("Look of the new track"));
	int source{tool.copy_selected ? 1 : 0};
	if (segmented("laylook", source,
	              {{0, STR_C("These textures"), "The rails and the trackbed chosen below.\nA track started or finished at a free end always takes them from the track it joins"},
	               {1, STR_C("Like the selected one"), "Takes the textures and the other parameters from the selected track.\nA track started or finished at a free end always takes them from the track it joins"}}))
		tool.copy_selected = source == 1;
	if (false == tool.copy_selected)
	{
		m_textures.loads = 0;
		auto const column{std::round(ImGui::GetCursorPosX() + std::max(ImGui::CalcTextSize(STR_C("Rails")).x, ImGui::CalcTextSize(STR_C("Trackbed")).x) + ImGui::GetStyle().ItemSpacing.x)};
		auto const texture_row = [&](int const Kind, char const *Label, std::string &Value) {
			ImGui::PushID(Kind);
			ImGui::AlignTextToFramePadding();
			ImGui::TextUnformatted(Label);
			ImGui::SameLine(column);
			auto const side{ImGui::GetFrameHeight()};
			auto const corner{ImGui::GetCursorScreenPos()};
			if (auto const image{Value.empty() || Value == "none" ? 0 : track_texture_image(Kind, Value)}; image != 0)
			{
				ImGui::Image(static_cast<ImTextureID>(image), ImVec2(side, side), ImVec2(0, 1), ImVec2(1, 0));
				if (ImGui::IsItemHovered())
				{
					auto const big{std::round(ImGui::GetFontSize() * 14.0f)};
					ImGui::BeginTooltip();
					ImGui::Image(static_cast<ImTextureID>(image), ImVec2(big, big), ImVec2(0, 1), ImVec2(1, 0));
					ImGui::TextUnformatted(Value.c_str());
					ImGui::EndTooltip();
				}
			}
			else
			{
				ImGui::Dummy(ImVec2(side, side));
				ImGui::GetWindowDrawList()->AddRect(corner, ImVec2(corner.x + side, corner.y + side), ImGui::GetColorU32(ImGuiCol_Border));
			}
			ImGui::SameLine(0.0f, 4.0f);
			ImGui::SetNextItemWidth(-FLT_MIN);
			ImGui::SetNextWindowSizeConstraints(ImVec2(ImGui::GetFontSize() * 36.0f, 0.0f), ImVec2(FLT_MAX, FLT_MAX));
			auto const slash{Value.find_last_of('/')};
			auto const preview{Value.empty() || Value == "none" ? std::string{STR_C("No texture")} : slash == std::string::npos ? Value : Value.substr(slash + 1)};
			if (ImGui::BeginCombo("##texture", preview.c_str(), ImGuiComboFlags_HeightLargest))
			{
				if (std::string chosen; track_texture_picker(Kind, Value, false, chosen))
				{
					Value = chosen;
					ImGui::CloseCurrentPopup();
				}
				ImGui::EndCombo();
			}
			item_tooltip(Kind == 0 ? "Texture of the rails.\nA material dragged from the texture browser can be dropped here" : "Texture of the trackbed (ballast) under the rails.\nA material dragged from the texture browser can be dropped here");
			if (std::string dropped; material_drop(dropped))
				Value = dropped;
			ImGui::PopID();
		};
		texture_row(0, STR_C("Rails"), tool.style.rail);
		texture_row(1, STR_C("Trackbed"), tool.style.ballast);
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
