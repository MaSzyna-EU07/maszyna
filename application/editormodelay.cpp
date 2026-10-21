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
			m_extend.mouse.y = bed_height(m_extend.mouse, plan_of(m_extend.mouse - m_extend.point), m_extend.track);
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
	if (projection.on_level(io.MousePos, drag.height, level) && (Global.editor_tracks || glm::distance(level, glm::dvec3{Global.pCamera.Pos}) <= static_cast<double>(kMaxPlacementDistance)))
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
		drag.step = m_history.size();
		records_watch_begin();
	}
	screen_projection const projection;
	glm::dvec3 position;
	if (false == projection.on_level(io.MousePos, drag.height, position) || (false == Global.editor_tracks && glm::distance(position, glm::dvec3{Global.pCamera.Pos}) > static_cast<double>(kMaxPlacementDistance)))
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
	records_watch_end();
	m_track_drag.clear();
	m_track_drag_points.clear();
	m_track_joints.clear();
}

std::optional<alignment::lay_result> editor_mode::eased_layout(TTrack const *From, glm::dvec3 const &Point, glm::dvec2 const &Direction, glm::dvec3 const &Target, double const Radius) const
{
	m_eased_last.reset();
	if (false == m_lay.transitions || From == nullptr || From->eType != tt_Normal)
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
	request.transitions = m_lay.transitions;
	request.speed = design_speed(From, End.track);
	request.kink = m_joints.kink_angle() * 0.5;
	request.shape = m_route.design.shape;
	request.norms = laying_norms();
	auto result{alignment::join(request)};
	if (m_lay.transitions && result.kind != alignment::join_kind::straight && result.kind != alignment::join_kind::arc && glm::length(request.end.direction) > 1e-9)
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
	std::vector<TTrack *> created;
	for (auto const &piece : pieces)
		created.push_back(editor_track::create_path(*style, piece));
	give_own_speed(created);
	editor_track::commit(created);
	digitize_drawn(created);
	push_track_snapshot({}, created);
	first_step_done(first_step::laid);
	if (target.track != nullptr)
		first_step_done(first_step::join);
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

std::vector<segment_data> editor_mode::lay_pieces(std::vector<glm::dvec3> const &Points, editor_track::snap_target const &End, double &Length, std::string &Error) const
{
	auto const &tool{m_lay};
	Length = 0.0;
	m_eased_last.reset();
	if (Points.size() < 2)
		return {};
	if (Points.size() == 2)
	{
		auto const leaving{tool.start.track != nullptr ? -plan_of(tool.start.direction) : plan_of(Points[1] - Points[0])};
		if (glm::length(leaving) < 1e-6)
		{
			Error = STR_C("Two points lie on one another");
			return {};
		}
		if (End.track != nullptr && tool.start.track != nullptr)
			return join_pieces(tool.start.track, tool.start.point, Points[0], glm::normalize(leaving), End, Length, Error);
		m_join_last = {};
		std::vector<segment_data> pieces;
		if (tool.start.track != nullptr)
		{
			auto const grade{-grade_of(tool.start.direction)};
			auto const eased{eased_layout(tool.start.track, Points[0], glm::normalize(leaving), Points[1], tool.radius)};
			auto const laid{eased.has_value() && false == eased->pieces.empty()};
			pieces = laid ? eased->pieces : editor_track::layout_pieces(m_track_layout, Points[0], glm::normalize(leaving), grade, Points[1], tool.radius);
			Error = laid ? narrowed(m_track_layout, tool.radius, eased->radius) : narrowed(pieces, m_track_layout, tool.radius);
			follow_ground(pieces, Points[0].y, grade, Points[1].y, std::numeric_limits<double>::quiet_NaN(), tool.start.track);
		}
		else if (End.track == nullptr)
		{
			auto const length{glm::length(leaving)};
			pieces = straight_piece(Points[0], glm::normalize(leaving), (Points[1].y - Points[0].y) / length, length);
			follow_ground(pieces, Points[0].y, std::numeric_limits<double>::quiet_NaN(), Points[1].y, std::numeric_limits<double>::quiet_NaN(), tool.copy_selected ? selected_track() : nullptr);
		}
		else if (glm::length(plan_of(End.direction)) > 1e-6)
		{
			auto const eased{eased_layout(End.track, End.position, glm::normalize(-plan_of(End.direction)), Points[0], tool.radius)};
			auto const laid{eased.has_value() && false == eased->pieces.empty()};
			auto const back{laid ? eased->pieces : editor_track::layout_pieces(m_track_layout, End.position, glm::normalize(-plan_of(End.direction)), -grade_of(End.direction), Points[0], tool.radius)};
			for (auto piece{back.rbegin()}; piece != back.rend(); ++piece)
				pieces.push_back(reversed(*piece));
			Error = laid ? narrowed(m_track_layout, tool.radius, eased->radius) : narrowed(pieces, m_track_layout, tool.radius);
			follow_ground(pieces, Points[0].y, std::numeric_limits<double>::quiet_NaN(), End.position.y, grade_of(End.direction), End.track);
		}
		for (auto const &piece : pieces)
			Length += bezier{piece}.plan_length();
		if (pieces.empty())
			Error = STR_C("The track can't be laid through these points");
		return pieces;
	}
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
	Error = lines_of(shape.warnings);
	auto const count = [&](alignment::result const &Shape, alignment::design const &Design) {
		return std::max(alignment::minimum_pieces(Shape, Design), static_cast<std::size_t>(std::ceil(Shape.length / std::max(5.0, tool.piece_length))));
	};
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
	auto const in{tool.transitions ? eased(tool.start, 0.0, 1.0) : editor_track::easing{}};
	auto const out{tool.transitions ? eased(End, shape.length, -1.0) : editor_track::easing{}};
	if ((false == in.pieces.empty() || false == out.pieces.empty()) && editor_track::eases_room(in, out, Points.front(), design.start_direction, Points.back(), design.end_direction))
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
			Length = editor_track::eased_length(in, easedshape.length, out);
			Error = lines_of(easedshape.warnings);
			return editor_track::eased_pieces(in, alignment::pieces(easedshape, easeddesign, count(easedshape, easeddesign)), out);
		}
		Error = STR("The curvature changes abruptly where the track joins, there's no room for a transition curve") + (easedshape.errors.empty() ? std::string{} : ": " + easedshape.errors.front());
	}
	Length = shape.length;
	return alignment::pieces(shape, design, count(shape, design));
}

alignment::result editor_mode::lay_shape(std::vector<glm::dvec3> const &Points, glm::dvec2 const &Startdirection, glm::dvec2 const &Enddirection, alignment::design &design) const
{
	auto const &tool{m_lay};
	design = {};
	design.norms = laying_norms();
	design.shape = m_route.design.shape;
	design.speed = design_speed(tool.start.track, nullptr);
	design.start = Points.front();
	design.end = Points.back();
	design.start_direction = Startdirection;
	design.end_direction = Enddirection;

	alignment::vertex corner;
	corner.radius = std::max(1.0, tool.radius);
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
	return alignment::compute_recommended(design, tool.transitions);
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
		return;
	}
	auto const &point{tool.mouse};
	if (plan_distance(point, tool.points.back()) < 1.0)
		return;
	m_typed.clear();
	if (tool.points.size() == 1)
	{
		lay_tool::step const previous{tool.start, tool.points.front()};
		tool.points.push_back(tool.mouse);
		lay_finish({});
		if (false == tool.points.empty())
		{
			tool.points.pop_back();
			return;
		}
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
		return;
	}
	tool.points.push_back(point);
}

void editor_mode::lay_finish(editor_track::snap_target const &End)
{
	auto &tool{m_lay};
	double length;
	std::string error;
	auto const pieces{lay_pieces(tool.points, End, length, error)};
	auto const eased{m_eased_last};
	tool.status_time = ImGui::GetTime();
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
	give_own_speed(created);
	editor_track::commit(created);
	digitize_drawn(created);
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
	tool.chain.clear();
	tool.preview.clear();
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
		tool.chain.pop_back();
	}
}

void editor_mode::lay_back()
{
	auto &tool{m_lay};
	if (false == tool.chain.empty())
	{
		undo_last();
		tool.status = STR_C("The last piece was taken back");
		tool.status_time = ImGui::GetTime();
		return;
	}
	if (false == tool.points.empty())
		tool.points.pop_back();
	if (tool.points.empty())
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
	norms.checked = false == m_norms_free;
	return norms;
}

namespace
{

// a card of the geometry of the new track: its plan, and under it its curvature along the length, a point running along both;
// a plain arc jumps to its curvature, a railway curve leads into it along a transition, the cant with it
float geometry_card_height(char const *Title, char const *Text, float const Width)
{
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 0.5f)};
	auto const wrap{Width - 2.0f * pad};
	return pad + ImGui::CalcTextSize(Title, nullptr, false, wrap).y + pad * 0.5f + std::round(font * 2.6f) + pad * 0.5f + ImGui::CalcTextSize(Text, nullptr, false, wrap).y + pad;
}

bool geometry_card(char const *Id, char const *Title, char const *Text, bool const Railway, bool const Chosen, float const Width, float const Height)
{
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 0.5f)};
	auto const sketch{std::round(font * 2.6f)};
	auto const wrap{Width - 2.0f * pad};
	auto const title{ImGui::CalcTextSize(Title, nullptr, false, wrap)};
	auto const height{Height};
	auto const top{ImGui::GetCursorScreenPos()};
	auto const clicked{ImGui::InvisibleButton(Id, ImVec2(Width, height))};
	auto const hovered{ImGui::IsItemHovered()};
	auto *list{ImGui::GetWindowDrawList()};
	auto const accent{family_colour(0)};
	ImVec2 const bottom{top.x + Width, top.y + height};
	auto const rounding{std::round(font * 0.35f)};
	list->AddRectFilled(top, bottom, Chosen ? (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 46) : ImGui::GetColorU32(hovered ? ImGuiCol_FrameBgHovered : ImGuiCol_FrameBg), rounding);
	list->AddRect(top, bottom, Chosen ? accent : ImGui::GetColorU32(ImGuiCol_Border), rounding, 0, Chosen ? 2.0f : 1.0f);
	auto y{top.y + pad};
	list->AddText(nullptr, 0.0f, ImVec2(top.x + pad, y), ImGui::GetColorU32(ImGuiCol_Text), Title, nullptr, wrap);
	y += title.y + pad * 0.5f;
	// the curvature along the length, as a share of the curvature of the arc
	auto const curvature = [Railway](double const Share) {
		if (Railway)
			return std::clamp((Share - 0.22) / 0.3, 0.0, 1.0);
		return Share < 0.37 ? 0.0 : 1.0;
	};
	int constexpr steps{80};
	std::vector<ImVec2> plan;
	plan.reserve(steps + 1);
	double heading{0.0}, px{0.0}, py{0.0};
	auto const turn{1.45 / (Railway ? 0.63 - 0.15 : 0.63)};
	for (int i = 0; i <= steps; ++i)
	{
		plan.emplace_back(static_cast<float>(px), static_cast<float>(py));
		auto const share{(i + 0.5) / steps};
		heading += curvature(share) * turn / steps;
		px += std::cos(heading) / steps;
		py -= std::sin(heading) / steps;
	}
	ImVec2 low{plan.front()}, high{plan.front()};
	for (auto const &point : plan)
	{
		low = ImVec2(std::min(low.x, point.x), std::min(low.y, point.y));
		high = ImVec2(std::max(high.x, point.x), std::max(high.y, point.y));
	}
	// the plan on the left, the curvature along the length on the right
	auto const half{std::floor((wrap - pad) * 0.5f)};
	auto const scale{std::min(half / std::max(1e-3f, high.x - low.x), sketch / std::max(1e-3f, high.y - low.y))};
	ImVec2 const origin{top.x + pad + (half - (high.x - low.x) * scale) * 0.5f, y + sketch - (high.y - low.y) * scale};
	for (auto &point : plan)
		point = ImVec2(origin.x + (point.x - low.x) * scale, origin.y + (point.y - low.y) * scale);
	auto const line{Chosen ? accent : ImGui::GetColorU32(ImGuiCol_Text, 0.75f)};
	list->AddPolyline(plan.data(), static_cast<int>(plan.size()), line, 0, 2.0f);
	auto const label{ImGui::CalcTextSize("1/R")};
	auto const graph_left{top.x + pad + half + pad * 0.5f + label.x + 2.0f};
	auto const graph_width{top.x + pad + wrap - graph_left};
	auto const graph_top{y};
	auto const graph{sketch};
	auto const axis{ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.6f)};
	list->AddLine(ImVec2(graph_left, graph_top + graph), ImVec2(graph_left + graph_width, graph_top + graph), axis);
	list->AddLine(ImVec2(graph_left, graph_top), ImVec2(graph_left, graph_top + graph), axis);
	list->AddText(ImVec2(graph_left - label.x - 2.0f, graph_top + graph * 0.2f - label.y * 0.5f), axis, "1/R");
	std::vector<ImVec2> curve;
	for (int i = 0; i <= steps; ++i)
	{
		auto const share{static_cast<double>(i) / steps};
		curve.emplace_back(graph_left + static_cast<float>(share) * graph_width, graph_top + graph - static_cast<float>(curvature(share)) * graph * 0.8f);
	}
	list->AddPolyline(curve.data(), static_cast<int>(curve.size()), line, 0, 1.5f);
	if (Railway)
		for (int i = 0; i < steps; i += 4)
		{
			auto const share{static_cast<double>(i) / steps};
			auto const next{static_cast<double>(i + 2) / steps};
			list->AddLine(ImVec2(graph_left + static_cast<float>(share) * graph_width, graph_top + graph - static_cast<float>(curvature(share)) * graph * 0.45f),
			              ImVec2(graph_left + static_cast<float>(next) * graph_width, graph_top + graph - static_cast<float>(curvature(next)) * graph * 0.45f), IM_COL32(240, 170, 60, 220), 1.0f);
		}
	auto const running{std::fmod(ImGui::GetTime() * 0.22, 1.0)};
	auto const index{std::clamp(static_cast<int>(running * steps), 0, steps)};
	list->AddCircleFilled(plan[index], 3.0f, IM_COL32(255, 255, 255, 230));
	list->AddLine(ImVec2(curve[index].x, graph_top), ImVec2(curve[index].x, graph_top + graph), IM_COL32(255, 255, 255, 70));
	list->AddCircleFilled(curve[index], 2.5f, IM_COL32(255, 255, 255, 230));
	y += sketch + pad * 0.5f;
	list->AddText(nullptr, 0.0f, ImVec2(top.x + pad, y), ImGui::GetColorU32(ImGuiCol_TextDisabled), Text, nullptr, wrap);
	return clicked;
}

} // namespace

bool editor_mode::render_norms_choice()
{
	auto picked{false};
	auto const gap{std::round(ImGui::GetFontSize() * 0.5f)};
	auto const width{std::floor((ImGui::GetContentRegionAvail().x - gap) * 0.5f)};
	char const *const titles[]{STR_C("Free laying"), STR_C("To the PKP PLK norms")};
	char const *const texts[]{STR_C("Plain arcs, no transition curves nor cant"), STR_C("Transition curves and cant for the speed, checked against the norms")};
	auto const height{std::max(geometry_card_height(titles[0], texts[0], width), geometry_card_height(titles[1], texts[1], width))};
	ImGui::PushID("geometry");
	for (int i = 0; i < 2; ++i)
	{
		if (i > 0)
			ImGui::SameLine(0.0f, gap);
		auto const chosen{m_norms_free == (i == 0)};
		auto const clicked{geometry_card(i == 0 ? "##free" : "##norms", titles[i], texts[i], i == 1, chosen, width, height)};
		item_tooltip(i == 0 ? "Plain arcs with no transition curves and no cant, no warnings about the norms: the track goes where it's clicked"
		                    : "Transition curves and cant for the design speed, warnings when the radius, the cant or the transitions don't meet the norms");
		picked |= clicked;
		if (clicked && false == chosen)
		{
			set_norms_free(i == 0);
			// a kept choice follows the one made later
			if (EditorSettings.norms() != editorSettings::track_norms::ask)
			{
				EditorSettings.norms(m_norms_free ? editorSettings::track_norms::free_laying : editorSettings::track_norms::plk);
				EditorSettings.save();
			}
		}
	}
	ImGui::PopID();
	return picked;
}

void editor_mode::render_norms_prompt()
{
	if (false == m_norms_prompt)
		return;
	char const *const title{STR_C("How to lay the tracks###normsprompt")};
	if (false == ImGui::IsPopupOpen(title))
		ImGui::OpenPopup(title);
	ImGui::SetNextWindowSize(ImVec2(520.0f * Global.ui_scale, 0.0f));
	if (false == ImGui::BeginPopupModal(title, nullptr, ImGuiWindowFlags_NoResize))
		return;
	ImGui::TextWrapped("%s", STR_C("Choose how the new tracks are laid. It can be changed any time at the top of the panels of laying and of the plan."));
	ImGui::Spacing();
	if (render_norms_choice())
	{
		if (m_norms_remember)
		{
			EditorSettings.norms(m_norms_free ? editorSettings::track_norms::free_laying : editorSettings::track_norms::plk);
			EditorSettings.save();
		}
		m_norms_prompt = false;
		ImGui::CloseCurrentPopup();
	}
	ImGui::Spacing();
	ImGui::Checkbox(STR_C("Remember the choice, don't ask again"), &m_norms_remember);
	item_tooltip("Asking again can be switched on in the settings of the editor (General)");
	ImGui::EndPopup();
}

void editor_mode::set_norms_free(bool const Free)
{
	m_norms_free = Free;
	m_lay.transitions = false == m_norms_free;
	if (false == m_route.chain.tracks.empty())
		route_update();
}

void editor_mode::render_lay_ui()
{
	auto &tool{m_lay};
	render_norms_choice();
	ImGui::Separator();
	ImGui::PushItemWidth(120.0f);
	ImGui::DragScalar(STR_C("Curve radius (m)"), ImGuiDataType_Double, &tool.radius, 5.0f, nullptr, nullptr, "%.0f");
	item_tooltip("Drag to change, Ctrl+click to type the value. While laying: Ctrl+wheel");
	value_chips("radii", tool.radius, {190.0, 300.0, 500.0, 800.0, 1200.0, 2000.0}, "%.0f");
	tool.radius = std::max(10.0, tool.radius);
	ImGui::DragScalar(STR_C("Speed (km/h)"), ImGuiDataType_Double, &tool.style.velocity, 1.0f, nullptr, nullptr, "%.0f");
	item_tooltip(m_norms_free ? "Speed limit of the new paths; the speed of the track joined comes first"
	                          : "Speed limit of the new paths, and the design speed of the transition curves and of the cant; the speed of the track joined comes first, and none higher than the radius allows");
	if (false == m_norms_free)
		value_chips("speeds", tool.style.velocity, {40.0, 60.0, 80.0, 100.0, 120.0, 160.0}, "%.0f");
	tool.style.velocity = std::clamp(tool.style.velocity, 0.0, 400.0);
	ImGui::PopItemWidth();
	ImGui::Checkbox(STR_C("This speed also on from a track with its own"), &tool.own_speed);
	item_tooltip(m_norms_free ? "Off: a track laid on from another one, or joined to it, takes the speed of that track.\nOn: the new tracks get the speed above"
	                          : "Off: a track laid on from another one, or joined to it, is designed for the speed of that track and takes it.\nOn: it's designed for the speed above and the new tracks get it.\nWhile laying: Ctrl+Shift+wheel");
	if (auto const [from, to]{laying_ends()}; false == m_norms_free && (from != nullptr || to != nullptr))
		ImGui::TextDisabled("%s", design_speed_of(from, to).c_str());
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
