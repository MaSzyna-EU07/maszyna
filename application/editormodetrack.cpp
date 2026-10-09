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
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <algorithm>
#include <cctype>
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
using geometry::signed_angle;
using geometry::turned;
using geometry::arc_pieces;

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
double const kLaySnapRadius{5.0};
float const kSnapPixels{16.0f};
float const kDragPixels{5.0f};
double const kSwitchReach{6.0}; // a press farther from the selected track selects another one instead of placing the switch
double const kLineReach{2000.0}; // how far each way the Curve tab follows the line
double const kJointRange{400.0}; // the ends of the paths are drawn for the tracks this close to the camera
ImU32 const kJointColor{IM_COL32(255, 255, 255, 200)};
int const kRailCategory{1};

void item_tooltip(char const *Text)
{
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C(Text));
}

// the curve a track ends with at Joint, as it goes on beyond the joint, and a transition curve leading out of it to the straight
struct easing
{
	double curvature{0.0}; // positive turning from x towards z
	double length{0.0};
	glm::dvec3 end{0.0};
	glm::dvec2 direction{0.0};
	std::vector<segment_data> pieces;
};

easing ease_out_of(TTrack const &Neighbour, glm::dvec3 const &Joint, double const Grade, double const Speed, alignment::limits const &Limits)
{
	easing result;
	for (auto const &path : Neighbour.m_paths)
		for (auto const atend : {false, true})
		{
			if (glm::distance(path.points[atend ? segment_data::point::end : segment_data::point::start], Joint) > 0.05)
				continue;
			if (path.points[segment_data::point::control1] == glm::dvec3{} && path.points[segment_data::point::control2] == glm::dvec3{})
				return result;
			bezier const curve{path};
			auto const first{curve.first(atend ? 1.0 : 0.0)};
			auto const second{curve.second(atend ? 1.0 : 0.0)};
			auto const speed{std::hypot(first.x, first.z)};
			if (speed < 1e-9)
				return result;
			auto curvature{(first.x * second.z - first.z * second.x) / (speed * speed * speed)};
			glm::dvec2 direction{first.x / speed, first.z / speed};
			double roll{path.rolls[1]};
			if (false == atend)
			{
				curvature = -curvature;
				direction = -direction;
				roll = -path.rolls[0];
			}
			result.curvature = curvature;
			if (std::abs(curvature) < 1e-4)
				return result;
			result.length = std::max(20.0, alignment::recommend(Speed, 1.0 / std::abs(curvature), Limits).transition);
			auto const count{static_cast<int>(std::ceil(result.length / 10.0))};
			auto const piece{result.length / count};
			glm::dvec3 point{Joint};
			for (int i = 0; i < count; ++i)
			{
				segment_data part;
				part.points[segment_data::point::start] = point;
				auto const from{direction};
				int const steps{static_cast<int>(std::ceil(piece / 0.05))};
				auto const step{piece / steps};
				for (int j = 0; j < steps; ++j)
				{
					auto const along{i * piece + (j + 0.5) * step};
					auto const angle{curvature * (1.0 - along / result.length) * step};
					auto const half{turned(direction, angle * 0.5)};
					point += glm::dvec3{half.x * step, Grade * step, half.y * step};
					direction = turned(direction, angle);
				}
				part.points[segment_data::point::end] = point;
				part.points[segment_data::point::control1] = glm::dvec3{from.x, Grade, from.y} * (piece / 3.0);
				part.points[segment_data::point::control2] = -glm::dvec3{direction.x, Grade, direction.y} * (piece / 3.0);
				part.rolls = {static_cast<float>(roll * (1.0 - i * piece / result.length)), static_cast<float>(roll * (1.0 - (i + 1) * piece / result.length))};
				result.pieces.push_back(part);
			}
			result.end = point;
			result.direction = direction;
			return result;
		}
	return result;
}

segment_data turned_around(segment_data const &Path)
{
	segment_data result{Path};
	result.points[segment_data::point::start] = Path.points[segment_data::point::end];
	result.points[segment_data::point::end] = Path.points[segment_data::point::start];
	result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
	result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
	result.rolls = {-Path.rolls[1], -Path.rolls[0]};
	return result;
}

// curvature of the shape at Chainage, positive turning from x towards z
double curvature_at(alignment::result const &Shape, double const Chainage)
{
	auto const a{std::clamp(Chainage - 0.5, 0.0, Shape.length)};
	auto const b{std::clamp(Chainage + 0.5, 0.0, Shape.length)};
	if (b - a < 1e-3)
		return 0.0;
	auto const from{plan_of(alignment::evaluate(Shape, a).direction)};
	auto const to{plan_of(alignment::evaluate(Shape, b).direction)};
	return signed_angle(glm::normalize(from), glm::normalize(to)) / (b - a);
}

// circular curve from a point along a direction, cut into pieces of 90 degrees at most. Side: 1 to the left, -1 to the right
void compound_from(editor_track::curve const &Curve, alignment::vertex &Vertex)
{
	if (false == Curve.compound || Curve.arcs.size() < 2)
		return;
	Vertex.compound = true;
	Vertex.radius = std::round(Curve.arcs.front().radius);
	Vertex.share = Curve.arcs.front().turn;
	Vertex.arcs.clear();
	for (std::size_t i = 1; i < Curve.arcs.size(); ++i)
		Vertex.arcs.push_back({std::round(Curve.arcs[i].radius), std::round(Curve.arcs[i].transition), Curve.arcs[i].turn});
}

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
	{
		m_track_point = {};
		m_track_mode_notice.clear();
	}
	m_node = track;
	ui()->set_node(m_node);
	if (ui()->mode() != nodebank_panel::TRACK)
		return;
	m_track_window_open = true;
	m_track_mode_notice.clear();
	// the mode stays as it is, the path is taken up by it if it can be
	switch (m_track_tab)
	{
	case track_tab::straights:
		if (track->eType == tt_Switch || false == editor_track::is_straight(*track, m_straights.tolerance))
			m_track_mode_notice = STR_C("The selected path isn't straight: C edits it as a curve, Esc goes back to Select");
		break;
	case track_tab::route:
		if (std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) != m_route.chain.tracks.end())
			break;
		if (track->eType != tt_Normal || false == editor_track::is_supported(*track))
			m_track_mode_notice = STR_C("Switches aren't a part of a line: T edits the switch, Esc goes back to Select");
		else if (false == route_load(*track))
			m_track_mode_notice = m_route.error.empty() ? std::string{STR_C("The curve can't be read from this path, edit it as a single path")} : m_route.error;
		break;
	default:
		break;
	}
}

editor_track::point_ref editor_mode::track_handle_hit() const
{
	auto *track = selected_track();
	if (track == nullptr || false == editor_track::is_supported(*track) || m_track_tab != track_tab::path)
		return {};

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
	return hit;
}

void editor_mode::draw_track_overlay() const
{
	auto const *track = selected_track();
	if (track == nullptr)
		return;

	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
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
	snap.removed = std::move(Removed);
	infra_attach(snap);
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

bool editor_mode::tracks_editable(std::vector<TTrack *> const &Tracks, std::string &Reason)
{
	for (auto const *track : Tracks)
	{
		std::string reason;
		if (track != nullptr && false == editor_track::can_edit_geometry(*track, reason))
		{
			Reason = (track->name().empty() ? std::string{"(noname)"} : track->name()) + ": " + reason;
			return false;
		}
	}
	Reason.clear();
	return true;
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
	if (Snapshot.profiles)
	{
		std::vector<scene::layer_handle> layers;
		for (auto const *library : {&std::as_const(m_profiles), &Snapshot.profile_library})
			for (auto const &entry : *library)
				layers.push_back(entry.layer);
		current.profile_library = std::move(m_profiles);
		m_profiles = Snapshot.profile_library;
		profile_library_mark(std::move(layers));
	}
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
		if (m_route.from->m_editorremoved || m_route.to->m_editorremoved || false == editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error, true))
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
	if (Snapshot.profiles && false == m_profile.route.spans.empty())
		profile_restore();
}

std::array<editor_mode::track_mode, 12> const &editor_mode::track_modes()
{
	static std::array<track_mode, 12> const modes{{
	    {"Select", "Esc", track_tab::path, "LMB on a path selects it: its points, control vectors and parameters"},
	    {"Lay track", "L", track_tab::lay, "Lays new track through the clicked points, on the ground or from a free end of a track"},
	    {"Switch", "T", track_tab::turnout, "Puts switches into the selected straight or curve, edits the geometry of the selected switch"},
	    {"Turntable", "U", track_tab::turntable, "Places a turntable, leads tracks out of it and fits the tracks around to it"},
	    {"Straight", "G", track_tab::straights, "The whole straight through the selected path: drag its ends or the middle, break it, shift it"},
	    {"Curve", "C", track_tab::route, "The line through the path, switch to switch: vertices with the radii, transitions and cant of the curves"},
	    {"Signals", "H", track_tab::signals, "Signals by the track, from the templates of the scenery, with the event the train reads them by"},
	    {"Objects", "B", track_tab::lineside, "Along the track: hectometre posts, fouling point markers, a parallel track, a vehicle to drive"},
	    {"Profile", "P", track_tab::profile, "Vertical profile (grade line) along the line"},
	    {"Speed", "V", track_tab::speed, "Speed limits of the paths against the speed their geometry allows"},
	    {"Joints", "J", track_tab::joints, "Ends of the paths which almost meet, steps, kinks, jumps of the cant and of the grade at the joints"},
	    {"Infra", "I", track_tab::infra, "Infrastructure along the track, bound to follow its changes"},
	}};
	return modes;
}

void editor_mode::show_track_tab(track_tab const Tab)
{
	m_track_window_open = true;
	m_roadtool.window = false;
	if (Tab == m_track_tab)
		return;
	m_track_mode_notice.clear();
	// what was going on in the mode left stays behind
	cancel_track_tools();
	m_straights.tool = 0;
	m_lay.points.clear();
	m_lay.start = {};
	m_lay.preview.clear();
	m_lay.status.clear();
	m_lay.active = Tab == track_tab::lay;
	m_switch.armed = -1;
	m_track_tab = Tab;
	auto *track{selected_track()};
	switch (Tab)
	{
	case track_tab::turnout:
		arm_switch();
		break;
	case track_tab::straights:
		if (track != nullptr && (track->eType == tt_Switch || false == editor_track::is_straight(*track, m_straights.tolerance)))
			m_track_mode_notice = STR_C("The selected path isn't straight: LMB on a straight");
		break;
	case track_tab::route:
		if (track != nullptr && std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) == m_route.chain.tracks.end())
		{
			if (track->eType != tt_Normal || false == editor_track::is_supported(*track))
				m_track_mode_notice = STR_C("Switches aren't a part of a line: LMB on a plain path");
			else if (false == route_load(*track))
				m_track_mode_notice = m_route.error.empty() ? std::string{STR_C("The curve can't be read from this path, edit it as a single path")} : m_route.error;
		}
		break;
	case track_tab::profile:
		if (track != nullptr && m_profile.route.spans.empty())
			profile_open_run(*track);
		break;
	case track_tab::turntable:
		scan_turntables();
		turntable_read(track);
		break;
	case track_tab::signals:
		signal_scan();
		signal_refresh(true);
		break;
	default:
		break;
	}
}

void editor_mode::arm_switch()
{
	auto &tool{m_switch};
	if (false == tool.collected)
	{
		tool.templates = editor_track::standard_switch_templates();
		auto const found{editor_track::find_switch_templates()};
		tool.templates.insert(tool.templates.end(), found.begin(), found.end());
		tool.collected = true;
	}
	tool.status.clear();
	tool.armed = tool.templates.empty() ? -1 : std::clamp(tool.last, 0, static_cast<int>(tool.templates.size()) - 1);
	if (tool.armed >= 0 && tool.templates[tool.armed].double_slip)
	{
		auto const ordinary{std::find_if(tool.templates.begin(), tool.templates.end(), [](editor_track::switch_template const &Template) { return false == Template.double_slip; })};
		tool.armed = ordinary != tool.templates.end() ? static_cast<int>(ordinary - tool.templates.begin()) : -1;
	}
}

bool editor_mode::track_shortcut(int const Key)
{
	for (auto const &mode : track_modes())
	{
		if (mode.key[1] != '\0' || Key != GLFW_KEY_A + (mode.key[0] - 'A'))
			continue;
		// the key of the mode on leaves it
		show_track_tab(m_track_window_open && m_track_tab == mode.tab ? track_tab::path : mode.tab);
		return true;
	}
	return false;
}

bool editor_mode::track_busy() const
{
	auto const &straights{m_straights};
	return (m_lay.active && false == m_lay.points.empty()) || m_turntable.placing || m_signal.placing || m_signal.moving || straights.dragging || m_extend.active || m_point_drag.active || m_handle_drag.active || m_switch.placing || m_track_point.valid() || m_route.vertex >= 0 || m_route.grip >= 0 || false == straights.detour.empty() || straights.tool != 0 || straights.handle >= 0;
}

std::string editor_mode::track_mode_name() const
{
	if (false == m_track_window_open)
		return STR_C("Select: LMB on a path");
	switch (m_track_tab)
	{
	case track_tab::lay: return m_extend.active ? STR_C("Laying track: new path from the free end") : STR_C("Laying track: LMB adds the points");
	case track_tab::turnout:
		if (m_switch.placing)
			return STR_C("Switch: drag to set its direction and side");
		if (m_switch.armed < 0)
			return selected_track() != nullptr && selected_track()->eType == tt_Switch ? STR_C("Switch: the selected one. Another: LMB on a track, then its type") : STR_C("Switch: choose its type from the list");
		return STR_C("Switch: press and drag on the selected track");
	case track_tab::straights:
		if (false == m_straights.detour.empty())
			return STR_C("Straight: shift around, Shift+click the points");
		if (m_straights.tool == 1)
			return STR_C("Straight: break, LMB on the straight");
		if (m_straights.tool == 2)
			return STR_C("Straight: S-curve, LMB on the straight");
		return STR_C("Straight: drag its ends or the middle");
	case track_tab::route: return STR_C("Curve: drag the vertices and the radius grips");
	case track_tab::profile: return STR_C("Vertical profile: edit the grade line in the profile window");
	case track_tab::speed: return STR_C("Speed check: LMB on a row shows the path");
	case track_tab::joints: return STR_C("Joints: LMB on a row shows the place");
	case track_tab::lineside:
		return STR_C("Objects along the track");
	case track_tab::infra: return STR_C("Infrastructure along the track");
	case track_tab::signals:
		if (m_signal.moving)
			return STR_C("Signal: drag along the track, release to put it there");
		if (m_signal.placing)
			return STR_C("Signal: drag along the track sets the direction of the trains, release to place it");
		return signal_armed() != nullptr ? STR_C("Signal: LMB by the track places it on that side") : STR_C("Signal: choose its type below");
	case track_tab::turntable:
		if (m_turntable.placing)
			return STR_C("Turntable: drag to turn the bridge, release to place it");
		return m_turntable.table != nullptr ? STR_C("Turntable: LMB around it leads a track out, on a free end fits that track") : STR_C("Turntable: LMB places a new one");
	default: return m_point_drag.moved ? STR_C("Select: moving the point") : selected_track() != nullptr ? STR_C("Select: drag the points of the path") : STR_C("Select: LMB on a path");
	}
}

void editor_mode::render_track_inspector()
{
	auto const &io{ImGui::GetIO()};
	m_profile.open = m_track_window_open && m_track_tab == track_tab::profile;
	m_speed.open = m_track_window_open && m_track_tab == track_tab::speed;
	m_infra.open = m_track_window_open && m_track_tab == track_tab::infra;
	m_joints.open = m_track_window_open && m_track_tab == track_tab::joints;
	if (false == m_track_window_open || m_track_tab != track_tab::lineside)
		m_sweep.open = m_hekto.open = m_fouling.open = m_parallel.open = m_vehicle.open = false;
	ui()->set_track(m_track_window_open);
	if (false == m_track_window_open)
	{
		// a closed window takes its mode with it
		if (m_track_tab != track_tab::path)
			show_track_tab(track_tab::path);
		m_track_window_open = false;
		return;
	}

	m_route_tab = m_track_tab != track_tab::path;

	// the tools of the mode are drawn in the tool options window and the selected path in the inspector, see
	// render_track_tool_options() and render_track_selection(); the checks list what they find in a window of their own
	if (m_track_tab == track_tab::speed || m_track_tab == track_tab::joints || m_track_tab == track_tab::infra)
	{
		auto const *title{m_track_tab == track_tab::speed ? STR_C("Speed check") : m_track_tab == track_tab::joints ? STR_C("Joints") : STR_C("Infrastructure")};
		ImGui::SetNextWindowSize(ImVec2(600.0f, std::min(320.0f, io.DisplaySize.y - 80.0f)), ImGuiCond_FirstUseEver);
		if (ImGui::Begin((std::string{title} + "###trackanalysis").c_str(), nullptr, ImGuiWindowFlags_NoCollapse))
		{
			switch (m_track_tab)
			{
			case track_tab::speed: render_speed_body(); break;
			case track_tab::joints: render_joints_body(); break;
			default: render_infra_body(); break;
			}
		}
		ImGui::End();
	}

	if (m_track_tab == track_tab::profile && m_track_window_open && false == m_profile.route.spans.empty())
		render_profile_strip();
}

void editor_mode::render_track_tool_options()
{
	auto *track{selected_track()};
	// the tools themselves are in the toolbar
	render_track_search();
	ImGui::Spacing();
	render_track_guide();
	switch (m_track_tab)
	{
	case track_tab::lay:
		render_lay_ui();
		break;
	case track_tab::turnout:
		if (track != nullptr && track->eType == tt_Switch)
			render_turnout_ui();
		else
			render_switch_ui();
		break;
	case track_tab::straights:
		render_straight_ui();
		if (ImGui::CollapsingHeader(STR_C("Straights in the scenery")))
			render_straights_ui();
		break;
	case track_tab::route:
		render_route_ui();
		break;
	case track_tab::profile: render_profile_body(); break;
	case track_tab::speed:
	case track_tab::joints:
	case track_tab::infra:
		ImGui::TextDisabled("%s", STR_C("What the check finds is listed in its window"));
		break;
	case track_tab::lineside: render_lineside_ui(); break;
	case track_tab::signals: render_signal_ui(); break;
	case track_tab::turntable: render_turntable_ui(); break;
	default:
		render_track_set_ui();
		render_track_spread_ui();
		break;
	}
}

void editor_mode::render_track_selection()
{
	auto *track{selected_track()};
	if (track == nullptr)
	{
		ImGui::TextDisabled("%s", STR_C("LMB on a path in the view selects it"));
		return;
	}
	char const *type = track->eType == tt_Normal ? "normal" : track->eType == tt_Switch ? "switch" : track->eType == tt_Cross ? "cross" : track->eType == tt_Table ? "turntable" : track->eType == tt_Tributary ? "tributary" : "unknown";
	ImGui::TextUnformatted((track->name().empty() ? std::string{"(noname)"} : track->name()).c_str());
	ImGui::TextDisabled("%s, %.2f m", STR_C(type), track->Length());
	// the parameters of the path belong to the select tool, the other tools show the path they work with
	if (m_track_tab == track_tab::path)
		render_path_ui();
}

void editor_mode::render_track_toolbar()
{
	auto const &modes{track_modes()};
	auto const vehicle{m_track_tab == track_tab::lineside && m_vehicle.open};
	// a tool chosen again gives way to the select one
	auto const tool = [&](track_tab const Tab) {
		auto const found{std::find_if(modes.begin(), modes.end(), [&](track_mode const &Mode) { return Mode.tab == Tab; })};
		if (found == modes.end())
			return;
		auto const chosen{m_track_tab == Tab && false == (Tab == track_tab::lineside && vehicle)};
		auto const label{std::string{STR_C(found->label)} + "###tracktool" + std::to_string(static_cast<int>(found - modes.begin()))};
		if (ImGui::MenuItem(label.c_str(), nullptr, chosen))
			show_track_tab(chosen ? track_tab::path : Tab);
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s  [%s]", STR_C(found->tooltip), found->key);
	};
	// building the track, what goes by it, the checks
	for (auto const tab : {track_tab::path, track_tab::lay, track_tab::turnout, track_tab::straights, track_tab::route, track_tab::turntable})
		tool(tab);
	ImGui::Separator();
	tool(track_tab::signals);
	tool(track_tab::lineside);
	// the vehicle to drive is one of the objects along the track
	if (ImGui::MenuItem(STR_C("Vehicle"), nullptr, vehicle))
	{
		show_track_tab(vehicle ? track_tab::path : track_tab::lineside);
		m_vehicle.expand = false == vehicle;
	}
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("A vehicle to drive, put on the selected track; it isn't written to the scenery files"));
	ImGui::Separator();
	for (auto const tab : {track_tab::profile, track_tab::speed, track_tab::joints, track_tab::infra})
		tool(tab);
	if (ImGui::MenuItem(STR_C("Structure gauge"), nullptr, m_gauge.open))
		m_gauge.open = !m_gauge.open;
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Checks which models enter the structure gauge of the tracks and the clearance over the roads (skrajnia budowli)"));
}

void editor_mode::render_track_guide()
{
	auto const hints{track_key_hints(false)};
	auto *drawlist{ImGui::GetWindowDrawList()};
	auto const &style{ImGui::GetStyle()};
	drawlist->ChannelsSplit(2);
	drawlist->ChannelsSetCurrent(1);
	auto const origin{ImGui::GetCursorScreenPos()};
	ImGui::SetCursorScreenPos(ImVec2(origin.x + 8.0f, origin.y + 6.0f));
	ImGui::BeginGroup();
	auto const mode{track_mode_name()};
	ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "%s", mode.c_str());
	ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - ImGui::CalcTextSize("(?)").x - 8.0f);
	ImGui::TextDisabled("(?)");
	if (ImGui::IsItemHovered())
	{
		std::string text;
		for (auto const &hint : track_key_hints(true))
			text += std::string{hint.key} + "   " + hint.action + "\n";
		ImGui::SetTooltip("%s", text.c_str());
	}
	float keywidth{0.0f};
	for (auto const &hint : hints)
		keywidth = std::max(keywidth, ImGui::CalcTextSize(hint.key).x);
	auto const left{ImGui::GetCursorPosX()};
	for (auto const &hint : hints)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.25f, 1.0f), "%s", hint.key);
		ImGui::SameLine(left + keywidth + 12.0f);
		ImGui::PushTextWrapPos(ImGui::GetWindowContentRegionMax().x - 8.0f);
		ImGui::TextUnformatted(hint.action.c_str());
		ImGui::PopTextWrapPos();
	}
	if (false == m_track_mode_notice.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", m_track_mode_notice.c_str());
	ImGui::EndGroup();
	auto const bottom{ImGui::GetItemRectMax().y + 6.0f};
	drawlist->ChannelsSetCurrent(0);
	drawlist->AddRectFilled(origin, ImVec2(ImGui::GetWindowPos().x + ImGui::GetWindowContentRegionMax().x, bottom), IM_COL32(40, 70, 110, 110), style.FrameRounding + 3.0f);
	drawlist->ChannelsMerge();
	ImGui::SetCursorScreenPos(ImVec2(origin.x, bottom + style.ItemSpacing.y));
	ImGui::Dummy(ImVec2(0.0f, 0.0f));
}

void editor_mode::render_track_name(TTrack &Track)
{
	auto &edit{m_track_name};
	auto const current{Track.name() == "none" ? std::string{} : Track.name()};
	auto const sync = [&]() {
		edit.track = &Track;
		edit.synced = current;
		std::snprintf(edit.text, sizeof(edit.text), "%s", current.c_str());
		edit.uses = editor_track::uses_of_name(Track);
	};
	if (edit.track != &Track)
	{
		sync();
		edit.status.clear();
	}
	else if (edit.synced != current && edit.synced == edit.text)
		sync();

	std::string why;
	if (false == scene::Layers.editable(&Track, &why))
	{
		ImGui::Text(STR_C("Name: %s"), current.empty() ? STR_C("(no name)") : current.c_str());
		ImGui::SameLine();
		ImGui::TextDisabled(STR_C("(can't be changed: %s)"), why.c_str());
		return;
	}
	auto const apply = [&]() {
		std::string name{edit.text};
		std::string reason;
		if (false == editor_track::name_valid(Track, name, reason))
			return;
		auto const before{editor_track::capture(Track)};
		editor_track::rename(Track, name);
		push_track_snapshot({{&Track, before}});
		edit.status = format(STR_C("Renamed from %s"), current.empty() ? STR_C("(no name)") : current.c_str());
		sync();
	};
	ImGui::SetNextItemWidth(220.f);
	if (ImGui::InputTextWithHint(STR_C("Name##trackname"), STR_C("(no name)"), edit.text, sizeof(edit.text), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsNoBlank))
		apply();
	if (ImGui::IsItemHovered())
		ImGui::SetTooltip("%s", STR_C("Enter renames the path, Esc while typing takes the text back.\nThe events, memory cells and the map of the game find the path by its name"));
	std::string typed{edit.text};
	if (typed == edit.synced)
	{
		if (false == edit.status.empty())
		{
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.5f, 0.9f, 0.5f, 1.f), "%s", edit.status.c_str());
		}
		return;
	}
	std::string reason;
	TTrack *owner{nullptr};
	auto const valid{editor_track::name_valid(Track, typed, reason, &owner)};
	ImGui::SameLine();
	if (valid)
	{
		auto const warned{false == edit.uses.events.empty() || false == edit.uses.loose.empty()};
		if (ImGui::SmallButton(warned ? STR_C("Rename anyway") : STR_C("Rename")))
			apply();
	}
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Keep the old")))
	{
		std::snprintf(edit.text, sizeof(edit.text), "%s", edit.synced.c_str());
		return;
	}
	if (false == valid)
	{
		ImGui::TextColored(ImVec4(1.f, 0.45f, 0.35f, 1.f), "%s", STR_C(reason.c_str()));
		if (owner != nullptr)
		{
			ImGui::SameLine();
			if (ImGui::SmallButton(STR_C("Look at it")))
				focus_track(*owner, 0, owner->location());
		}
		return;
	}
	if (typed != edit.text)
		ImGui::TextDisabled(STR_C("Saved as %s: the scenery keeps the names in lower case"), typed.c_str());
	if (edit.uses.cells > 0 || edit.uses.includes > 0)
		ImGui::TextDisabled(STR_C("The new name goes to %d memory cell(s) and %d include(s), the drives of a switch among them"), edit.uses.cells, edit.uses.includes);
	auto const listed = [](std::vector<std::string> const &Names) {
		std::string result;
		for (std::size_t i = 0; i < Names.size() && i < 4; ++i)
			result += (i > 0 ? ", " : "") + Names[i];
		if (Names.size() > 4)
			result += format(" (+%d)", static_cast<int>(Names.size() - 4));
		return result;
	};
	if (false == edit.uses.events.empty())
		ImGui::TextColored(ImVec4(1.f, 0.75f, 0.3f, 1.f), STR_C("These events aim at the path by the old name and the editor doesn't write them: %s"), listed(edit.uses.events).c_str());
	if (false == edit.uses.loose.empty())
		ImGui::TextColored(ImVec4(1.f, 0.75f, 0.3f, 1.f), STR_C("These events belong to the path by the old name: %s"), listed(edit.uses.loose).c_str());
	if (false == edit.uses.events.empty() || false == edit.uses.loose.empty())
		ImGui::TextDisabled("%s", STR_C("They work until the scenery is loaded again; then they need the new name in their files"));
}

void editor_mode::render_turnout_ui()
{
	auto *track{selected_track()};
	if (track == nullptr || track->eType != tt_Switch || track->m_paths.size() < 2)
		return;
	auto const &main{track->m_paths[0]};
	auto const &diverging{track->m_paths[1]};
	auto const origin{main.points[segment_data::point::start]};
	auto const chord{plan_of(main.points[segment_data::point::end] - origin)};
	auto const &control2{diverging.points[segment_data::point::control2]};
	auto const control{glm::length(plan_of(control2)) > 1e-6 ? control2 : diverging.points[segment_data::point::start] - diverging.points[segment_data::point::end]};
	if (glm::length(chord) < 1e-3 || glm::length(plan_of(control)) < 1e-6)
	{
		ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.3f, 1.0f), STR_C("The geometry of this switch is degenerate, it can't be edited here"));
		return;
	}
	glm::dvec2 const axis{glm::normalize(chord)};
	glm::dvec2 const normal{-axis.y, axis.x};
	auto const length{glm::dot(chord, axis)};
	glm::dvec2 const end{diverging.points[segment_data::point::end].x - origin.x, diverging.points[segment_data::point::end].z - origin.z};
	auto const offset{glm::dot(end, normal)};
	glm::dvec2 const tangent{glm::normalize(glm::dvec2{-control.x, -control.z})};
	auto const angle{std::atan2(std::abs(tangent.x * axis.y - tangent.y * axis.x), glm::dot(tangent, axis))};
	ImGui::Text(STR_C("Length %.3f m, diverging %s, end offset %.3f m"), length, offset > 0.0 ? STR_C("left") : STR_C("right"), std::abs(offset));
	render_track_name(*track);
	if ((simulation::Events.FindEvent(track->name() + "+") != nullptr && simulation::Events.FindEvent(track->name() + "-") != nullptr) || (m_track_name.track == track && m_track_name.uses.includes > 0))
		ImGui::TextDisabled(STR_C("Drive: events %s+ and %s- switch it, also in the map"), track->name().c_str(), track->name().c_str());
	else
	{
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", STR_C("No drive: the switch can't be switched during the game"));
		render_switch_drive_choice();
		if (m_switch.drive >= 0 && ImGui::Button(STR_C("Put the drive by the switch")))
			place_switch_drive(*track);
	}
	if (false == m_switch.status.empty())
		ImGui::TextWrapped("%s", m_switch.status.c_str());
	if (angle > 1e-6)
		ImGui::Text(STR_C("Angle 1:%.2f (%.4f deg), R in the entry %.0f m"), 1.0 / std::tan(angle), glm::degrees(angle), diverging.radius);

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
	if (tool.templates[m_turnout_template].double_slip)
	{
		auto const ordinary{std::find_if(tool.templates.begin(), tool.templates.end(), [](auto const &Template) { return false == Template.double_slip; })};
		if (ordinary == tool.templates.end())
			return;
		m_turnout_template = static_cast<int>(ordinary - tool.templates.begin());
	}
	ImGui::Separator();
	ImGui::PushItemWidth(-1.0f);
	if (ImGui::BeginCombo("##turnouttemplate", tool.templates[m_turnout_template].label.c_str()))
	{
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		{
			if (tool.templates[i].double_slip)
				continue;
			auto const label{(tool.templates[i].source != nullptr ? "Scenery: " : "PLK: ") + tool.templates[i].label};
			if (ImGui::Selectable(label.c_str(), i == m_turnout_template))
				m_turnout_template = i;
		}
		ImGui::EndCombo();
	}
	ImGui::PopItemWidth();
	auto const replace = [&](editor_track::switch_template const &Template, int const Side) {
		auto const grade{length > 0.0 ? (main.points[segment_data::point::end].y - origin.y) / length : 0.0};
		auto paths{editor_track::place_switch(Template, origin, axis, Side, grade)};
		if (paths.size() < 2)
			return;
		for (int i = 0; i < 2; ++i)
			if (paths[i].rolls[0] == 0.f && paths[i].rolls[1] == 0.f)
				paths[i].rolls = track->m_paths[i].rolls;
		auto const joints{editor_track::joints(*track)};
		std::vector<std::pair<TTrack *, editor_track::state>> states{{track, editor_track::capture(*track)}};
		track->m_paths.assign(paths.begin(), paths.begin() + 2);
		// the neighbours follow the ends which moved a little, those which went far are left loose
		std::vector<editor_track::joint> following;
		std::vector<TTrack *> changed{track};
		std::string loose;
		for (auto const &joint : joints)
		{
			auto const &theirs{joint.other->m_paths[joint.theirs.path]};
			auto const shift{glm::distance(editor_track::point_position(*track, joint.own), editor_track::point_position(*joint.other, joint.theirs))};
			if (shift > 0.3 * bezier{theirs}.plan_length())
			{
				loose += (loose.empty() ? "" : ", ") + (joint.other->name().empty() ? std::string{"(noname)"} : joint.other->name());
				continue;
			}
			following.push_back(joint);
			if (std::find(changed.begin(), changed.end(), joint.other) == changed.end())
			{
				changed.push_back(joint.other);
				states.emplace_back(joint.other, editor_track::capture(*joint.other));
			}
		}
		push_track_snapshot(std::move(states));
		editor_track::follow(*track, following);
		editor_track::commit(changed);
		if (false == loose.empty())
			ui()->set_status(STR_C("Too far from the new ends of the switch, left unjoined: ") + loose, true);
	};
	int const side{offset > 0.0 ? 1 : -1};
	if (ImGui::Button(STR_C("Replace the geometry with the template")))
		replace(tool.templates[m_turnout_template], side);
	item_tooltip("Keeps the start and the direction of the switch, takes the shape of the chosen template");
	ImGui::SameLine();
	if (ImGui::Button(STR_C("Flip the side")))
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
	item_tooltip("Mirrors the switch: the diverging track goes to the other side");
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

	ImGui::PushItemWidth(120.0f);
	double velocity = editor_track::velocity(*track);
	if (ImGui::InputDouble(STR_C("Velocity (km/h, -1: none)"), &velocity, 0.0, 0.0, "%.1f"))
		editor_track::velocity(*track, velocity);
	item_tooltip("Speed limit of the path for the AI drivers and the timetable, -1: no limit of its own");
	finish_edit(rebuild::none);
	char const *environments[] = {"flat", "mountains", "canyon", "tunnel", "bridge", "bank"};
	int environment = std::clamp(static_cast<int>(track->eEnvironment), 0, 5);
	if (ImGui::Combo("Environment", &environment, environments, IM_ARRAYSIZE(environments)))
	{
		track->eEnvironment = static_cast<TEnvironmentType>(environment);
		apply_now(rebuild::none);
	}
	item_tooltip("Surroundings of the path, they affect the sound of the passing vehicles");

	ImGui::PopItemWidth();

	if (ImGui::CollapsingHeader(STR_C("Ride and sound")))
	{
		ImGui::PushItemWidth(120.0f);
		if (ImGui::InputFloat(STR_C("Friction"), &track->fFriction, 0.0f, 0.0f, "%.3f"))
			track->fFriction = std::max(0.0f, track->fFriction);
		item_tooltip("Friction coefficient of the rails, 0.15 for the dry rails; lower values make the wheels slip");
		finish_edit(rebuild::none);
		ImGui::InputFloat(STR_C("Sound distance"), &track->fSoundDistance, 0.0f, 0.0f, "%.1f");
		item_tooltip("Distance between the rail joints in metres, sets the rhythm of the wheel clatter; -1: the default");
		finish_edit(rebuild::none);
		ImGui::InputInt(STR_C("Quality"), &track->iQualityFlag, 0);
		item_tooltip("Quality of the track, 20 is the nominal one; lower values give a rougher ride");
		finish_edit(rebuild::none);
		int damage = track->iDamageFlag;
		if (ImGui::InputInt(STR_C("Damage flags"), &damage, 0))
			editor_track::damage(*track, damage);
		item_tooltip("0: track in order; any value adds jolts, 128: the vehicles derail on the path");
		finish_edit(rebuild::none);

		if (ImGui::InputFloat(STR_C("Width"), &track->fTrackWidth, 0.0f, 0.0f, "%.3f"))
			track->fTrackWidth = std::max(0.01f, track->fTrackWidth);
		item_tooltip("Gauge of the track (1.435 m for the standard gauge), width of the surface for the roads");
		finish_edit(rebuild::parameters);
		ImGui::PopItemWidth();
	}
	if (track->m_visible && ImGui::CollapsingHeader(STR_C("Textures and trackbed")))
	{
		material_handle *materials[] = {&track->m_material1, &track->m_material2, track->SwitchExtension ? &track->SwitchExtension->m_material3 : nullptr};
		char const *pathlabels[] = {"Texture 1 (rails/surface)", "Texture 2 (trackbed/side)", "Trackbed (switch)"};
		char const *switchlabels[] = {"Rails P1-P2", "Rails P3-P4", "Trackbed"};
		auto const &labels{track->eType == tt_Switch ? switchlabels : pathlabels};
		if (false == ImGui::IsAnyItemActive())
		{
			for (int i = 0; i < 3; ++i)
			{
				if (m_track_material_edited[i] != nullptr)
					continue;
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
			if (ImGui::InputText(labels[i], m_track_materials[i].data(), m_track_materials[i].size()))
				m_track_material_edited[i] = track;
			if (ImGui::IsItemDeactivated() && m_track_material_edited[i] != nullptr)
			{
				auto const *edited{m_track_material_edited[i]};
				m_track_material_edited[i] = nullptr;
				if (edited == track && m_track_materials[i].data() != editor_track::material_name(*materials[i]))
				{
					*materials[i] = editor_track::fetch_material(m_track_materials[i].data());
					apply_now(rebuild::parameters);
				}
			}
			ImGui::PopItemWidth();
		}
		ImGui::TextDisabled(STR_C("The texture changes on Enter or leaving the field, \"none\" removes it"));

		ImGui::PushItemWidth(120.0f);
		if (ImGui::InputFloat(STR_C("Texture length"), &track->fTexLength, 0.0f, 0.0f, "%.2f"))
			track->fTexLength = std::max(0.01f, track->fTexLength);
		item_tooltip("Length in metres after which the texture of the rails and the trackbed repeats");
		finish_edit(rebuild::parameters);
		float height = editor_track::texture_height(*track);
		if (ImGui::InputFloat(STR_C("Trackbed height"), &height, 0.0f, 0.0f, "%.3f"))
			editor_track::texture_height(*track, height);
		item_tooltip("Height of the trackbed edge below the rail top, in metres");
		finish_edit(rebuild::parameters);
		ImGui::InputFloat(STR_C("Trackbed width"), &track->fTexWidth, 0.0f, 0.0f, "%.3f");
		item_tooltip("Width of the trackbed outside of the rail, in metres");
		finish_edit(rebuild::parameters);
		ImGui::InputFloat(STR_C("Trackbed slope"), &track->fTexSlope, 0.0f, 0.0f, "%.3f");
		item_tooltip("Horizontal reach of the slope at the outer edge of the trackbed, in metres");
		finish_edit(rebuild::parameters);
		ImGui::PopItemWidth();
	}
	render_path_circuits(*track);
}

void editor_mode::render_path_circuits(TTrack &Track)
{
	auto *track{&Track};
	auto const copy = [](std::array<char, 256> &Buffer, std::string const &Text) {
		std::strncpy(Buffer.data(), Text.c_str(), Buffer.size() - 1);
		Buffer.back() = '\0';
	};
	auto const isolated_text = [&]() {
		std::string text;
		for (auto const *isolated : track->Isolated)
			text += (text.empty() ? "" : " ") + isolated->asName;
		return text;
	};
	if (m_track_names_for != track && false == ImGui::IsAnyItemActive())
	{
		m_track_names_for = track;
		copy(m_track_isolated, isolated_text());
		copy(m_track_sleeper_model, track->m_sleeper_model_name);
		copy(m_track_sleeper_skin, track->m_sleeper_skin_name.empty() ? std::string{"none"} : track->m_sleeper_skin_name);
		m_track_event[0] = '\0';
		m_track_events_missing.clear();
	}
	// one undo step for the change made by the widget just used
	auto const changed = [&](editor_track::state const &Before, bool const Geometry) {
		push_track_snapshot({{track, Before}});
		if (Geometry)
			editor_track::commit_parameters(*track);
		else
			track->mark_dirty();
	};

	if (false == ImGui::CollapsingHeader(STR_C("Track circuit, overhead line, sleepers, events")))
		return;
	auto const before{editor_track::capture(*track)};

	ImGui::PushItemWidth(-160.0f);
	ImGui::InputText(STR_C("Track circuits"), m_track_isolated.data(), m_track_isolated.size());
	item_tooltip("Names of the track circuits (isolated) the path belongs to, separated by spaces.\n"
	             "A name which doesn't exist yet makes a new circuit. Changes on leaving the field");
	if (ImGui::IsItemDeactivated() && m_track_isolated.data() != isolated_text())
	{
		if (false == track->Dynamics.empty())
		{
			ui()->set_status(STR_C("The track circuits can't change while vehicles stand on the path"), true);
			copy(m_track_isolated, isolated_text());
		}
		else
		{
			std::istringstream names{m_track_isolated.data()};
			std::vector<std::string> list;
			for (std::string name; names >> name;)
				list.push_back(name);
			editor_track::isolated(*track, list);
			changed(before, false);
			copy(m_track_isolated, isolated_text());
		}
	}
	ImGui::PopItemWidth();

	int overheadmode{track->fOverhead < 0.f ? 0 : track->fOverhead == 0.f ? 1 : 2};
	ImGui::PushItemWidth(220.0f);
	if (ImGui::Combo("Overhead line", &overheadmode, "Normal\0No current, coasting\0Pantographs lowered, speed limit\0"))
	{
		editor_track::overhead(*track, overheadmode == 0 ? -1.f : overheadmode == 1 ? 0.f : std::max(1.f, track->fOverhead > 0.f ? track->fOverhead : 40.f));
		changed(before, false);
	}
	ImGui::PopItemWidth();
	item_tooltip("State of the overhead line over the path: the trains coast through a section with no current,\nor lower the pantographs and keep to the speed limit given");
	if (track->fOverhead > 0.f)
	{
		ImGui::PushItemWidth(120.0f);
		float limit{track->fOverhead};
		if (ImGui::InputFloat(STR_C("Speed with the pantographs lowered (km/h)"), &limit, 0.0f, 0.0f, "%.0f"))
			editor_track::overhead(*track, std::max(1.f, limit));
		if (ImGui::IsItemActivated())
			m_track_field_before = before;
		if (ImGui::IsItemDeactivatedAfterEdit())
			changed(m_track_field_before, false);
		ImGui::PopItemWidth();
	}

	ImGui::Separator();
	bool sleepers{track->m_sleeper_enabled};
	if (ImGui::Checkbox(STR_C("Sleeper models"), &sleepers))
	{
		if (sleepers && m_track_sleeper_model[0] == '\0')
		{
			ui()->set_status(STR_C("Give the sleeper model first"), true);
		}
		else
		{
			track->m_sleeper_enabled = sleepers;
			track->m_sleeper_model_name = m_track_sleeper_model.data();
			changed(before, true);
		}
	}
	item_tooltip("Repeats a model along the path, the sleepers in place of the drawn trackbed");
	ImGui::PushItemWidth(-160.0f);
	auto const name_field = [&](char const *Label, std::array<char, 256> &Buffer, std::string &Value, char const *Tooltip) {
		ImGui::InputText(Label, Buffer.data(), Buffer.size());
		item_tooltip(Tooltip);
		std::string typed{Buffer.data()};
		if (&Value == &track->m_sleeper_skin_name && typed == "none")
			typed.clear();
		if (ImGui::IsItemDeactivated() && typed != Value)
		{
			Value = typed;
			changed(before, track->m_sleeper_enabled);
		}
	};
	name_field(STR_C("Sleeper model"), m_track_sleeper_model, track->m_sleeper_model_name, STR_C("Path of the .e3d model, as in the scenery file"));
	name_field(STR_C("Sleeper skin"), m_track_sleeper_skin, track->m_sleeper_skin_name, STR_C("Replaceable skin of the model, \"none\": the one of the model"));
	ImGui::PopItemWidth();
	ImGui::PushItemWidth(120.0f);
	auto const number_field = [&](char const *Label, float &Value, char const *Format, char const *Tooltip) {
		ImGui::InputFloat(Label, &Value, 0.0f, 0.0f, Format);
		item_tooltip(Tooltip);
		if (ImGui::IsItemActivated())
			m_track_field_before = before;
		if (ImGui::IsItemDeactivatedAfterEdit())
		{
			track->m_sleeper_frequency = std::max(0.1f, track->m_sleeper_frequency);
			changed(m_track_field_before, track->m_sleeper_enabled);
		}
	};
	number_field(STR_C("Sleeper spacing (m)"), track->m_sleeper_frequency, "%.3f", STR_C("Distance between the sleepers along the path"));
	ImGui::PopItemWidth();
	ImGui::PushItemWidth(200.0f);
	ImGui::InputFloat3(STR_C("Sleeper offset (m)"), &track->m_sleeper_offset.x, "%.3f");
	item_tooltip("Shift of each model: sideways, along the path, up");
	if (ImGui::IsItemActivated())
		m_track_field_before = before;
	if (ImGui::IsItemDeactivatedAfterEdit())
		changed(m_track_field_before, track->m_sleeper_enabled);
	ImGui::PopItemWidth();
	ImGui::PushItemWidth(120.0f);
	number_field(STR_C("Trackbed shift (m)"), track->m_sleeper_ballast_z, "%.3f", STR_C("Vertical shift of the drawn trackbed under the sleepers, negative lowers it"));
	ImGui::PopItemWidth();

	ImGui::Separator();
	ImGui::TextUnformatted(STR_C("Events"));
	item_tooltip("event0: a vehicle with a crew stands on the path, event1: runs along it towards its start, event2: towards its end.\n"
	             "eventall0/1/2: the same for any vehicle");
	auto const editable_events{track->Dynamics.empty()};
	for (int list = 0; list < 6; ++list)
	{
		auto &events{editor_track::events(*track, list)};
		for (std::size_t i = 0; i < events.size(); ++i)
		{
			auto const &event{events[i]};
			// bound by the name of the path, not written in the file
			if (event.first.empty())
			{
				ImGui::TextDisabled(STR_C("%s  %s (by the name of the path)"), editor_track::event_keyword(list), event.second != nullptr ? STR_C("auto") : "?");
				continue;
			}
			ImGui::PushID(list * 1000 + static_cast<int>(i));
			if (ImGui::SmallButton("x") && editable_events)
			{
				events.erase(events.begin() + i);
				editor_track::bind_events(*track, m_track_events_missing);
				changed(before, false);
				ImGui::PopID();
				break;
			}
			ImGui::SameLine();
			if (event.second == nullptr)
				ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), STR_C("%s  %s (no such event)"), editor_track::event_keyword(list), event.first.c_str());
			else
				ImGui::Text("%s  %s", editor_track::event_keyword(list), event.first.c_str());
			ImGui::PopID();
		}
	}
	ImGui::PushItemWidth(100.0f);
	ImGui::Combo("##eventlist", &m_track_event_list, "event0\0event1\0event2\0eventall0\0eventall1\0eventall2\0");
	ImGui::PopItemWidth();
	ImGui::SameLine();
	ImGui::PushItemWidth(-60.0f);
	auto const entered{ImGui::InputText("##eventname", m_track_event.data(), m_track_event.size(), ImGuiInputTextFlags_EnterReturnsTrue)};
	ImGui::PopItemWidth();
	ImGui::SameLine();
	if ((ImGui::Button(STR_C("Add")) || entered) && m_track_event[0] != '\0')
	{
		if (false == editable_events)
		{
			ui()->set_status(STR_C("The events can't change while vehicles stand on the path"), true);
		}
		else
		{
			editor_track::events(*track, m_track_event_list).emplace_back(m_track_event.data(), nullptr);
			editor_track::bind_events(*track, m_track_events_missing);
			changed(before, false);
			m_track_event[0] = '\0';
		}
	}
	item_tooltip("Name of the event to add to the chosen list. An event which doesn't exist is kept, and reported");
	if (false == m_track_events_missing.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.3f, 1.0f), STR_C("No such events: %s"), m_track_events_missing.c_str());
}

void editor_mode::render_path_ui()
{
	auto *track = selected_track();
	if (track == nullptr)
	{
		ImGui::TextDisabled(STR_C("No path selected"));
		return;
	}

	if (false == editor_track::is_supported(*track))
	{
		ImGui::TextDisabled(STR_C("Editing of this path type isn't supported"));
		return;
	}
	render_track_name(*track);
	std::string reason;
	bool const geometry = editor_track::can_edit_geometry(*track, reason);
	if (false == geometry)
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), STR_C("Geometry locked: %s"), reason.c_str());

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

	if (ImGui::CollapsingHeader(STR_C("Geometry"), ImGuiTreeNodeFlags_DefaultOpen))
	{
		for (int i = 0; i < static_cast<int>(track->m_paths.size()); ++i)
		{
			ImGui::PushID(i);
			if (track->m_paths.size() > 1)
				ImGui::TextDisabled(STR_C("Path %d"), i + 1);
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
			item_tooltip("(+) joined with another track, (-) a free end");

			auto &path = track->m_paths[i];
			if (false == geometry)
			{
				auto const &start = path.points[segment_data::point::start];
				auto const &end = path.points[segment_data::point::end];
				ImGui::TextDisabled(STR_C("Start (%.2f, %.2f, %.2f)  End (%.2f, %.2f, %.2f)"), start.x, start.y, start.z, end.x, end.y, end.z);
				ImGui::PopID();
				continue;
			}
			ImGui::PushItemWidth(-110.0f);
			if (m_track_point.valid() && m_track_point.path == i)
			{
				auto const kind{m_track_point.kind};
				if (editor_track::is_end(kind))
				{
					auto position = path.points[kind == editor_track::point_kind::start ? segment_data::point::start : segment_data::point::end];
					if (ImGui::DragScalarN("x, y, z##point", ImGuiDataType_Double, glm::value_ptr(position), 3, 0.01f, nullptr, nullptr, "%.3f"))
						editor_track::move_point(*track, m_track_point, position);
					item_tooltip("Drag to move, Ctrl+click to type the value");
					finish_edit(rebuild::geometry);
				}
				else
				{
					auto &vector{path.points[kind == editor_track::point_kind::control1 ? segment_data::point::control1 : segment_data::point::control2]};
					ImGui::DragScalarN(STR_C("vector##point"), ImGuiDataType_Double, glm::value_ptr(vector), 3, 0.01f, nullptr, nullptr, "%.3f");
					item_tooltip("Control vector, relative to its end; zero for a straight");
					finish_edit(rebuild::geometry);
				}
			}
			if (ImGui::TreeNode(STR_C("Cant and radius")))
			{
				ImGui::DragFloat2(STR_C("Roll (deg)"), path.rolls.data(), 0.01f, -15.0f, 15.0f, "%.2f");
				item_tooltip("Cant at the start and at the end as the roll angle of the track");
				finish_edit(rebuild::geometry);
				ImGui::InputFloat(STR_C("Radius"), &path.radius, 0.0f, 0.0f, "%.1f");
				item_tooltip("Radius stored with the path in the scenery; the shape itself comes from the control vectors");
				finish_edit(rebuild::geometry);
				if (ImGui::Button(STR_C("Straighten")))
				{
					push_track_snapshot({{track, before}});
					path.points[segment_data::point::control1] = glm::dvec3{};
					path.points[segment_data::point::control2] = glm::dvec3{};
					path.radius = 0.0f;
					editor_track::commit({track});
				}
				item_tooltip("Removes the control vectors and the radius, the path becomes a straight between its ends");
				ImGui::TreePop();
			}
			ImGui::PopItemWidth();
			ImGui::PopID();
		}
		if (geometry && ImGui::TreeNode(STR_C("Snapping of the ends")))
		{
			ImGui::PushItemWidth(120.0f);
			ImGui::DragFloat(STR_C("Snap radius (m)"), &m_track_snap_radius, 0.05f, 0.0f, 50.0f, "%.2f");
			item_tooltip("A dragged free end within this distance of another free end joins it on release");
			ImGui::PopItemWidth();
			ImGui::Checkbox(STR_C("Align with snapped path"), &m_track_align_tangent);
			item_tooltip("On joining, the end also takes over the direction of the other path, so the joint has no kink");
			ImGui::Checkbox(STR_C("Drag connected ends"), &m_track_drag_connected);
			item_tooltip("Dragging a joined end, or moving and turning the whole path or switch,\nmoves the ends of the neighbouring paths with it");
			ImGui::TreePop();
		}
	}
	render_path_parameters(*track);
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
	if (false == editor_track::find_chain(route.from, route.to, route.chain, route.error, true))
		return;
	if (false == route.chain.switches.empty())
		route.status = format(STR_C("The fragment runs through %zu switches along their main tracks. They move as a whole with the line, "
		                      "so they have to fall on its straights"),
		                      route.chain.switches.size());

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
	{
		m_route.chain = {};
		m_route.result = {};
		m_route.vertex = m_route.grip = -1;
		m_route.error = STR_C("No curve here: LMB on a curve");
		return false;
	}
	m_route.from = curve.from;
	m_route.to = curve.to;
	route_reset();
	if (m_route.chain.tracks.empty())
		return false;
	auto &design{m_route.design};
	auto vertex{route_vertex_of(curve)};
	if (curve.reversals == 0)
	{
		design.vertices = {vertex};
	}
	else
	{
		auto const distance{plan_distance(design.start, design.end)};
		vertex.offset = distance / 3.0;
		vertex.compound = false;
		vertex.reverse_turn = false;
		design.vertices = {vertex, vertex};
	}
	route_update();
	m_route.status = STR_C("Curve of ") + std::to_string(m_route.chain.tracks.size()) + STR_C(" paths, turning ") + std::to_string(static_cast<int>(std::round(glm::degrees(curve.turn)))) + STR_C(" deg");
	return true;
}

alignment::vertex editor_mode::route_vertex_of(editor_track::curve const &Curve) const
{
	alignment::vertex vertex;
	vertex.radius = Curve.radius > 0.0 ? std::max(10.0, std::round(Curve.radius)) : 1000.0;
	route_recommend(vertex);
	if (Curve.transition_in > 0.0)
		vertex.transition_in = std::round(Curve.transition_in);
	if (Curve.transition_out > 0.0)
		vertex.transition_out = std::round(Curve.transition_out);
	if (Curve.cant > 0.0)
		vertex.cant = std::round(Curve.cant);
	vertex.reverse_turn = std::abs(Curve.turn) > glm::pi<double>();
	compound_from(Curve, vertex);
	return vertex;
}

bool editor_mode::route_from_line(TTrack &Track)
{
	auto &route{m_route};
	editor_track::chain run;
	if (false == editor_track::find_run(Track, kLineReach, run, route.error))
		return false;
	route.from = run.tracks.front();
	route.to = run.tracks.back();
	route_reset();
	if (route.chain.tracks.empty())
		return false;
	auto &design{route.design};
	auto const found{editor_track::recognize_line(route.chain, m_straights.tolerance, design.norms.gauge)};
	if (false == found.empty())
	{
		design.vertices.clear();
		for (auto const &entry : found)
		{
			alignment::vertex vertex;
			if (entry.kink)
			{
				vertex.radius = std::max(10.0, std::round(alignment::recommend(design.speed, 1000.0, design.norms).radius_min / 10.0) * 10.0);
				vertex.transition_in = vertex.transition_out = vertex.cant = 0.0;
			}
			else
			{
				vertex = route_vertex_of(entry.shape);
			}
			vertex.position = entry.position;
			design.vertices.push_back(vertex);
		}
		if (design.vertices.size() >= 2)
		{
			design.vertices.front().offset = std::max(0.0, glm::dot(design.vertices.front().position - plan_of(design.start), design.start_direction));
			design.vertices.back().offset = std::max(0.0, glm::dot(plan_of(design.end) - design.vertices.back().position, design.end_direction));
		}
		route_update();
	}
	route.status = format(STR_C("Line of %zu paths, %.0f m, %zu vertices"), route.chain.tracks.size(), route.chain.length, design.vertices.size());
	return true;
}

void editor_mode::route_apply()
{
	auto &route{m_route};
	route.status.clear();
	if (false == route.result.valid || route.chain.tracks.empty())
		return;
	editor_track::chain chain;
	if (false == editor_track::find_chain(route.from, route.to, chain, route.error, true))
		return;
	if (glm::distance(chain.start, route.chain.start) > 1e-3 || glm::distance(chain.end, route.chain.end) > 1e-3)
	{
		route.error = STR_C("The fragment changed since it was selected, select it again");
		return;
	}
	auto involved{chain.tracks};
	for (auto const &passage : chain.switches)
		involved.push_back(passage.track);
	if (false == tracks_editable(involved, route.error))
		return;

	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	// takes back what was done before the paths were re-laid
	auto const undo = [&]() {
		for (auto *track : created)
			editor_track::retire(*track);
		for (auto entry = states.rbegin(); entry != states.rend(); ++entry)
		{
			editor_track::apply(*entry->first, entry->second);
			editor_track::commit({entry->first});
		}
	};
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
		auto const found{editor_track::find_chain(from, to, chain, error, true)};
		auto const startoff{found ? plan_distance(chain.start, route.result.start) : 0.0};
		auto const endoff{found ? plan_distance(chain.end, route.result.end) : 0.0};
		if (false == found || startoff > 5.0 || endoff > 5.0)
		{
			auto const startrun{editor_track::straight_beyond(before, false, m_straights.tolerance)};
			auto const endrun{editor_track::straight_beyond(before, true, m_straights.tolerance)};
			WriteLog(format("editor: cutting the straights failed: found %d, start %.2f m off (extension %.2f, run %zu paths %.2f m), end %.2f m off (extension %.2f, run %zu paths %.2f m), chain %zu paths, from %s, to %s, error '%s'", found ? 1 : 0, startoff,
			                startextension, startrun.tracks.size(), startrun.length, endoff, endextension, endrun.tracks.size(), endrun.length, chain.tracks.size(), from != nullptr ? from->name().c_str() : "-", to != nullptr ? to->name().c_str() : "-", error.c_str()));
			undo();
			route.error = error.empty() ? format(STR_C("The adjoining straights can't be cut at the ends of the curve: the start misses by %.1f m, the end by %.1f m"), startoff, endoff) : error;
			return;
		}
		if (false == tracks_editable(chain.tracks, route.error))
		{
			undo();
			return;
		}
	}
	for (auto *track : chain.tracks)
		if (std::none_of(states.begin(), states.end(), [&](auto const &Entry) { return Entry.first == track; }))
			states.emplace_back(track, editor_track::capture(*track));
	std::vector<TTrack *> relaid;
	if (chain.switches.empty())
	{
		auto pieces{alignment::pieces(route.result, route.design, chain.tracks.size())};
		editor_track::keep_heights(chain, pieces);
		relaid = editor_track::relay(chain, pieces);
	}
	else if (false == route_relay_through_switches(chain, states, relaid))
	{
		undo();
		return;
	}
	// the straights were cut along their own direction, the curve ends where the design says: the ends are brought together
	auto const added{relaid.size()};
	created.insert(created.end(), relaid.begin(), relaid.end());
	push_track_snapshot(std::move(states), std::move(created));

	route.from = from;
	route.to = to;
	editor_track::find_chain(route.from, route.to, route.chain, route.error, true);
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
	route.status = STR_C("Re-laid ") + std::to_string(chain.tracks.size()) + STR_C(" paths") + (added > 0 ? STR_C(", added ") + std::to_string(added) : std::string{});
	if (startextension > 0.0 || endextension > 0.0)
		route.status += format(", the curve took over %.2f / %.2f m of the adjoining straights", startextension, endextension);
	WriteLog("Editor: route design - " + route.status, logtype::generic);
}

// the switches of the chain are moved as a whole onto the designed line, the regular paths between them re-laid
bool editor_mode::route_relay_through_switches(editor_track::chain const &Chain, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created)
{
	auto &route{m_route};
	auto const &result{route.result};
	auto const name = [](TTrack const *Track) { return Track->name().empty() ? std::string{"(noname)"} : Track->name(); };
	auto const plan = [](glm::dvec3 const &Vector) { return glm::normalize(plan_of(Vector)); };
	struct placement
	{
		TTrack *track;
		double from, length; // chainage along the new line
		glm::dvec3 entry; // of the main track, where the chain comes in
		double angle;
	};
	// chainages along the old line, the switches keep their distance from the nearer end of the chain
	double before{0.0};
	std::vector<double> starts;
	std::size_t next{0};
	for (std::size_t i = 0; i <= Chain.tracks.size(); ++i)
	{
		for (; next < Chain.switches.size() && Chain.switches[next].before == i; ++next)
		{
			starts.push_back(before);
			before += bezier{Chain.switches[next].track->m_paths.front()}.plan_length();
		}
		if (i < Chain.tracks.size())
			before += bezier{Chain.tracks[i]->m_paths.front()}.plan_length();
	}
	auto const oldlength{before};
	std::vector<placement> placements;
	std::vector<double> breaks;
	for (std::size_t k = 0; k < Chain.switches.size(); ++k)
	{
		auto const &passage{Chain.switches[k]};
		auto const &main{passage.track->m_paths.front()};
		auto const length{bezier{main}.plan_length()};
		auto const from{starts[k] + length * 0.5 < oldlength * 0.5 ? starts[k] : result.length - (oldlength - starts[k])};
		if (from < 0.0 || from + length > result.length || (false == breaks.empty() && from < breaks.back() - 0.01))
		{
			route.error = STR_C("The switch ") + name(passage.track) + STR_C(" doesn't fit on the designed line");
			return false;
		}
		auto const entry{main.points[passage.forward ? segment_data::point::start : segment_data::point::end]};
		auto const exit{main.points[passage.forward ? segment_data::point::end : segment_data::point::start]};
		bezier const curve{main};
		auto const direction{passage.forward ? plan(curve.first(0.0)) : -plan(curve.first(1.0))};
		auto const first{alignment::evaluate(result, from)};
		auto const last{alignment::evaluate(result, from + length)};
		auto const angle{signed_angle(direction, plan(first.direction))};
		// where the other end of the main track goes, set against the line
		auto const moved{plan_of(first.position) + turned(plan_of(exit - entry), angle)};
		auto const departure{glm::distance(moved, plan_of(last.position))};
		if (departure > 0.02)
		{
			route.error = format(STR_C("The switch %s would leave the designed line by %.2f m at its other end: it lies in a curve of the design. "
			                     "Move the vertices so that the switch falls on a straight"),
			                     name(passage.track).c_str(), departure);
			return false;
		}
		placements.push_back({passage.track, from, length, entry, angle});
		breaks.push_back(from);
		breaks.push_back(from + length);
	}
	// the regular parts between the switches, one interval each, the switches taking the ones between
	auto const parts{editor_track::chain_parts(Chain)};
	std::vector<std::size_t> counts;
	std::vector<int> partof;
	std::size_t part{0};
	for (std::size_t k = 0; k <= Chain.switches.size(); ++k)
	{
		auto const first{k == 0 ? std::size_t{0} : Chain.switches[k - 1].before};
		auto const last{k == Chain.switches.size() ? Chain.tracks.size() : Chain.switches[k].before};
		counts.push_back(last - first);
		partof.push_back(last > first ? static_cast<int>(part++) : -1);
		if (k < Chain.switches.size())
			counts.push_back(1);
	}
	if (part != parts.size())
	{
		route.error = STR_C("The paths between the switches can't be read as a chain");
		return false;
	}
	std::vector<std::size_t> intervals;
	auto const pieces{alignment::pieces(result, route.design, 0, breaks, counts, &intervals)};
	std::vector<std::vector<segment_data>> groups(parts.size());
	for (std::size_t i = 0; i < pieces.size(); ++i)
		if (intervals[i] % 2 == 0 && partof[intervals[i] / 2] >= 0)
			groups[partof[intervals[i] / 2]].push_back(pieces[i]);
	for (std::size_t i = 0; i < parts.size(); ++i)
		if (groups[i].size() < parts[i].tracks.size())
		{
			route.error = STR_C("The designed line between the switches is too short for the paths there");
			return false;
		}

	// all checked, the changes go in
	auto const remember = [&](TTrack *Track) {
		if (std::none_of(States.begin(), States.end(), [&](auto const &Entry) { return Entry.first == Track; }))
			States.emplace_back(Track, editor_track::capture(*Track));
	};
	for (auto const &place : placements)
	{
		auto joints{editor_track::joints(*place.track)};
		joints.erase(std::remove_if(joints.begin(), joints.end(), [&](editor_track::joint const &Joint) { return std::find(Chain.tracks.begin(), Chain.tracks.end(), Joint.other) != Chain.tracks.end(); }), joints.end());
		remember(place.track);
		std::vector<TTrack *> changed{place.track};
		for (auto const &joint : joints)
		{
			remember(joint.other);
			if (std::find(changed.begin(), changed.end(), joint.other) == changed.end())
				changed.push_back(joint.other);
		}
		// the heights stay, the paths between the switches keep theirs too
		auto target{alignment::evaluate(result, place.from).position};
		target.y = place.entry.y;
		editor_track::place(*place.track, place.entry, target, place.angle);
		editor_track::follow(*place.track, joints);
		editor_track::commit(changed);
	}
	for (std::size_t i = 0; i < parts.size(); ++i)
	{
		auto group{groups[i]};
		editor_track::keep_heights(parts[i], group);
		auto const added{editor_track::relay(parts[i], group)};
		Created.insert(Created.end(), added.begin(), added.end());
	}
	return true;
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

bool editor_mode::route_hit(int &Vertex, int &Grip) const
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
	Vertex = hit;
	Grip = griphit;
	return hit >= 0 || griphit >= 0;
}

void editor_mode::draw_route_overlay() const
{
	auto const &route{m_route};
	auto const &design{route.design};
	auto const &result{route.result};
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
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
			auto radius{format("R %.0f", report != nullptr ? report->radius : design.vertices[i].radius)};
			if (report != nullptr && report->radii.size() >= 2)
			{
				radius = "R";
				for (std::size_t k = 0; k < report->radii.size(); ++k)
					radius += format(k == 0 ? " %.0f" : " / %.0f", report->radii[k]);
			}
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
	glm::dvec3 moved{position};
	bool held{false};
	if (m_handle_drag.active)
	{
		held = handle_drag_step(moved);
		if (false == held && false == m_route_gizmo_using)
			return;
	}
	else
	{
		if (false == m_route_gizmo_using)
			m_route_gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(position - frame.camera));
		frame.manipulate(ImGuizmo::TRANSLATE, m_route_gizmo, m_gizmo_snap);
		held = ImGuizmo::IsUsing();
		moved = frame.camera + glm::dvec3(m_route_gizmo[3]);
	}

	if (false == held)
	{
		if (m_route_gizmo_using)
		{
			m_route_gizmo_using = false;
			route_apply();
		}
		return;
	}
	m_route_gizmo_using = true;
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
	if (false == route.error.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", route.error.c_str());
	if (route.chain.tracks.empty())
		return;

	ImGui::TextDisabled(STR_C("%zu paths, %.2f m"), route.chain.tracks.size(), route.chain.length);
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Reset design")))
	{
		auto *track{route.chain.tracks[route.chain.tracks.size() / 2]};
		route_load(*track);
		return;
	}
	item_tooltip("Reads the curves of the line from the scenery again, dropping the changes not applied");
	auto changed{render_route_vertices()};
	if (ImGui::CollapsingHeader(STR_C("Design speed and limits")))
		changed |= render_route_parameters();
	if (changed)
		route_update();

	ImGui::Separator();
	render_route_result();
}

bool editor_mode::render_route_parameters()
{
	auto &design{m_route.design};
	bool changed{false};
	ImGui::PushItemWidth(120.0f);
	changed |= ImGui::InputDouble(STR_C("Design speed (km/h)"), &design.speed, 0.0, 0.0, "%.0f");
	item_tooltip("Speed the cant, the transitions and the recommended radius are designed for");
	design.speed = std::max(1.0, design.speed);
	int shape = static_cast<int>(design.shape);
	char const *shapes[] = {"Cubic parabola", "Clothoid"};
	if (ImGui::Combo("Transition curve", &shape, shapes, IM_ARRAYSIZE(shapes)))
	{
		design.shape = static_cast<alignment::transition_shape>(shape);
		changed = true;
	}
	item_tooltip("Cubic parabola: the usual one on the Polish lines; clothoid: curvature growing linearly with the length");
	ImGui::PopItemWidth();

	if (ImGui::TreeNode(STR_C("Limits")))
	{
		auto &norms{design.norms};
		ImGui::PushItemWidth(100.0f);
		changed |= ImGui::InputDouble(STR_C("Rail axes distance (mm)"), &norms.gauge, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble(STR_C("Unbalanced acceleration (m/s2)"), &norms.unbalanced, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble(STR_C("Maximum cant (mm)"), &norms.cant_max, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble(STR_C("Cant change rate (mm/s)"), &norms.cant_rate, 0.0, 0.0, "%.0f");
		changed |= ImGui::InputDouble(STR_C("Cant ramp 1:(k*V), k"), &norms.ramp_factor, 0.0, 0.0, "%.1f");
		changed |= ImGui::InputDouble(STR_C("Unbalanced acc. change rate (m/s3)"), &norms.jerk, 0.0, 0.0, "%.2f");
		changed |= ImGui::InputDouble(STR_C("Straight between curves (s of travel)"), &norms.tangent_min_time, 0.0, 0.0, "%.1f");
		ImGui::PopItemWidth();
		norms.gauge = std::max(100.0, norms.gauge);
		norms.cant_rate = std::max(1.0, norms.cant_rate);
		norms.jerk = std::max(0.01, norms.jerk);
		ImGui::TreePop();
	}
	return changed;
}

bool editor_mode::render_route_vertices()
{
	bool changed{false};
	auto const count{static_cast<int>(m_route.design.vertices.size())};
	for (int i = 0; i < count; ++i)
	{
		ImGui::PushID(i);
		changed |= render_route_vertex(i);
		ImGui::PopID();
	}
	return changed;
}

bool editor_mode::render_route_vertex(int const Index)
{
	auto &route{m_route};
	auto &design{route.design};
	auto &vertex{design.vertices[Index]};
	auto const count{static_cast<int>(design.vertices.size())};
	bool changed{false};
	if (count > 1)
		ImGui::TextDisabled(STR_C("Curve %d of the S-curve"), Index + 1);
	ImGui::PushItemWidth(110.0f);
	changed |= ImGui::DragScalar(vertex.compound ? STR_C("Radius R1 (m)") : STR_C("Radius R (m)"), ImGuiDataType_Double, &vertex.radius, 5.0f, nullptr, nullptr, "%.0f");
	if (ImGui::Checkbox(STR_C("Compound curve (several radii)"), &vertex.compound))
	{
		changed = true;
		if (vertex.compound && vertex.arcs.empty())
			vertex.arcs.push_back({std::max(10.0, std::round(vertex.radius * 0.6 / 10.0) * 10.0), 0.0, 1.0});
	}
	item_tooltip("Curve made of arcs of different radii, each one its own path, joined with transitions");
	if (vertex.compound)
	{
		auto total{std::max(0.0, vertex.share)};
		for (auto const &arc : vertex.arcs)
			total += std::max(0.0, arc.share);
		auto const percent = [&](double const Share) { return total > 0.0 ? 100.0 * std::max(0.0, Share) / total : 0.0; };
		changed |= ImGui::DragScalar(format(STR_C("Share of R%d"), 1).c_str(), ImGuiDataType_Double, &vertex.share, 0.01f, nullptr, nullptr, "%.2f");
		ImGui::SameLine();
		ImGui::TextDisabled("%.0f%%", percent(vertex.share));
		int removed{-1};
		for (int k = 0; k < static_cast<int>(vertex.arcs.size()); ++k)
		{
			auto &arc{vertex.arcs[k]};
			ImGui::PushID(k);
			ImGui::Separator();
			changed |= ImGui::DragScalar(format(STR_C("Radius R%d (m)"), k + 2).c_str(), ImGuiDataType_Double, &arc.radius, 5.0f, nullptr, nullptr, "%.0f");
			ImGui::SameLine();
			if (ImGui::SmallButton("x"))
				removed = k;
			item_tooltip("Takes this radius out of the curve");
			changed |= ImGui::DragScalar(format(STR_C("Transition R%d-R%d (m)"), k + 1, k + 2).c_str(), ImGuiDataType_Double, &arc.transition, 1.0f, nullptr, nullptr, "%.0f");
			changed |= ImGui::DragScalar(format(STR_C("Share of R%d"), k + 2).c_str(), ImGuiDataType_Double, &arc.share, 0.01f, nullptr, nullptr, "%.2f");
			ImGui::SameLine();
			ImGui::TextDisabled("%.0f%%", percent(arc.share));
			arc.radius = std::max(1.0, arc.radius);
			arc.transition = std::max(0.0, arc.transition);
			arc.share = std::max(0.0, arc.share);
			ImGui::PopID();
		}
		vertex.share = std::max(0.0, vertex.share);
		if (removed >= 0)
		{
			vertex.arcs.erase(vertex.arcs.begin() + removed);
			if (vertex.arcs.empty())
				vertex.compound = false;
			changed = true;
		}
		if (ImGui::SmallButton(STR_C("Add a radius")))
		{
			auto const last{vertex.arcs.empty() ? vertex.radius : vertex.arcs.back().radius};
			vertex.arcs.push_back({std::max(10.0, std::round(last * 0.8 / 10.0) * 10.0), 0.0, 1.0});
			changed = true;
		}
		item_tooltip("Another arc after the last one, the angle of the curve is shared out by the shares");
		ImGui::Separator();
	}
	changed |= ImGui::DragScalar(STR_C("Transition in (m)"), ImGuiDataType_Double, &vertex.transition_in, 1.0f, nullptr, nullptr, "%.0f");
	changed |= ImGui::DragScalar(STR_C("Transition out (m)"), ImGuiDataType_Double, &vertex.transition_out, 1.0f, nullptr, nullptr, "%.0f");
	changed |= ImGui::DragScalar(STR_C("Cant (mm)"), ImGuiDataType_Double, &vertex.cant, 1.0f, nullptr, nullptr, "%.0f");
	item_tooltip("Height of the outer rail above the inner one");
	ImGui::PopItemWidth();
	vertex.radius = std::max(1.0, vertex.radius);
	vertex.transition_in = std::max(0.0, vertex.transition_in);
	vertex.transition_out = std::max(0.0, vertex.transition_out);
	vertex.cant = std::max(0.0, vertex.cant);
	if (ImGui::SmallButton(STR_C("Apply recommended for V")))
	{
		route_recommend(vertex);
		changed = true;
	}
	item_tooltip("Cant and transitions recommended for the design speed and the radius");

	auto const recommended{alignment::recommend(design.speed, vertex.radius, design.norms)};
	ImGui::TextDisabled(STR_C("For %.0f km/h: R min %.0f m, cant %.0f mm (equilibrium %.0f, min %.0f), transition %.0f m"), design.speed, recommended.radius_min, recommended.cant, recommended.cant_equilibrium, recommended.cant_min, recommended.transition);
	if (ImGui::TreeNode(STR_C("Details")))
	{
		for (auto const &curve : route.result.curves)
		{
			if (curve.vertex != Index)
				continue;
			ImGui::Text(STR_C("Deflection %.4f deg, tangents %.2f / %.2f m, arc %.2f m"), glm::degrees(curve.deflection), curve.tangent_in, curve.tangent_out, curve.arc_length);
			ImGui::Text(STR_C("Unbalanced acc. %.3f m/s2, cant rate %.1f / %.1f mm/s, acc. rate %.3f / %.3f m/s3"), curve.unbalanced, curve.cant_rate_in, curve.cant_rate_out, curve.jerk_in, curve.jerk_out);
		}
		changed |= ImGui::Checkbox(STR_C("Turn the longer way (over 180 deg)"), &vertex.reverse_turn);
		item_tooltip("For the loops: the curve turns by more than a half circle");
		ImGui::TreePop();
	}
	return changed;
}

void editor_mode::render_route_result()
{
	auto const &route{m_route};
	auto const &design{route.design};
	auto const &result{route.result};
	if (result.length > 0.0)
	{
		auto const pieces{alignment::minimum_pieces(result, design)};
		ImGui::Text(STR_C("Length %.2f m (now %.2f m), %zu elements"), result.length, route.chain.length, result.elements.size());
		ImGui::TextDisabled(STR_C("Paths: %zu in the fragment, %zu needed%s"), route.chain.tracks.size(), pieces, pieces > route.chain.tracks.size() ? STR_C(" (copies of the neighbours will be added)") : "");
		if (result.start_extension > 0.0 || result.end_extension > 0.0)
			ImGui::Text(STR_C("Takes over %.2f m of the straight at the start, %.2f m at the end"), result.start_extension, result.end_extension);
	}
	for (auto const &error : result.errors)
		ImGui::TextColored(ImVec4(1.0f, 0.35f, 0.3f, 1.0f), "%s", error.c_str());
	for (auto const &warning : result.warnings)
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", warning.c_str());

	if (ImGui::TreeNode(STR_C("Elements")))
	{
		ImGui::TextDisabled(STR_C("Straight available beyond the ends: %.2f / %.2f m"), design.start_reserve, design.end_reserve);
		char const *kinds[] = {"straight", "transition", "curve", "transition"};
		for (auto const &e : result.elements)
			ImGui::Text("%8.2f m  %-10s  L %8.2f m%s", e.chainage, kinds[static_cast<int>(e.kind)], e.length, e.kind == alignment::element_kind::straight ? "" : ("  R " + std::to_string(static_cast<int>(std::round(e.radius))) + " m").c_str());
		ImGui::TreePop();
	}

	if (result.valid)
	{
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.55f, 0.25f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.25f, 0.65f, 0.3f, 1.0f));
		if (ImGui::Button(STR_C("Apply to the scenery"), ImVec2(-1.0f, 28.0f)))
			route_apply();
		ImGui::PopStyleColor(2);
	}
	else
	{
		ImGui::TextDisabled(STR_C("Correct the errors to apply the design"));
	}
	if (false == route.status.empty())
		ImGui::TextUnformatted(route.status.c_str());
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
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
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
	ImGui::TextDisabled(STR_C("Straights recognized from chains of collinear straight paths.\nLMB: select a path to see the straight it belongs to"));

	ImGui::PushItemWidth(100.0f);
	double angle{glm::degrees(state.tolerance.angle)};
	if (ImGui::InputDouble(STR_C("Angle tolerance (deg)"), &angle, 0.0, 0.0, "%.4f"))
		state.tolerance.angle = glm::radians(std::max(0.0, angle));
	item_tooltip("Largest change of the direction between the paths of one straight");
	ImGui::InputDouble(STR_C("Offset tolerance (m)"), &state.tolerance.offset, 0.0, 0.0, "%.3f");
	item_tooltip("Largest distance of the paths from the common axis of the straight");
	ImGui::InputDouble(STR_C("Treat as straight above R (m)"), &state.tolerance.radius, 0.0, 0.0, "%.0f");
	item_tooltip("Curves of a larger radius count as straight");
	ImGui::InputDouble(STR_C("Minimum length (m)"), &state.minimum_length, 0.0, 0.0, "%.0f");
	item_tooltip("Shorter straights aren't listed");
	ImGui::PopItemWidth();
	if (ImGui::Button(STR_C("Recognize straights in the scenery")))
	{
		state.found = editor_track::find_straights(state.minimum_length, state.tolerance);
		state.listed = -1;
		state.current_for = nullptr;
	}
	if (false == state.found.empty())
	{
		ImGui::Text(STR_C("%zu straights"), state.found.size());
		ImGui::SameLine();
		ImGui::PushItemWidth(150.0f);
		ImGui::InputTextWithHint("##straightfilter", STR_C("path name"), state.filter, sizeof(state.filter));
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
		if (false == state.status.empty())
			ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", state.status.c_str());
		return;
	}
	ImGui::TextUnformatted(describe(line).c_str());

	auto const toolbutton = [&](char const *Label, int const Tool, char const *Tooltip) {
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
		item_tooltip(Tooltip);
	};
	toolbutton(STR_C("Break"), 1, STR_C("Turns the rest of the straight from a point, a curve goes at the break (Ctrl+drag in the view does the same)"));
	ImGui::SameLine();
	toolbutton(STR_C("S-curve"), 2, STR_C("Shifts the rest of the straight sideways through two curves"));
	ImGui::SameLine();
	bool const member{in_straight_set(line)};
	if (ImGui::Button(member ? STR_C("Leave the group") : STR_C("Move with others")))
	{
		if (member)
			state.set.erase(std::remove_if(state.set.begin(), state.set.end(), [&](editor_track::straight const &Member) { return std::any_of(line.tracks.begin(), line.tracks.end(), [&](TTrack const *Track) { return std::find(Member.tracks.begin(), Member.tracks.end(), Track) != Member.tracks.end(); }); }), state.set.end());
		else
			state.set.push_back(line);
	}
	item_tooltip("Straights of a group move together and stay parallel; Alt+click in the view adds another one");
	if (state.tool != 0)
	{
		ImGui::PushItemWidth(100.0f);
		ImGui::InputDouble(STR_C("Curve R (m, 0: automatic)"), &state.curve_radius, 0.0, 0.0, "%.0f");
		item_tooltip("Radius of the curves made by the break and the S-curve; automatic: 1.5 times the minimum radius for the speed of the line");
		ImGui::Checkbox(STR_C("Transitions from the line speed"), &state.auto_transitions);
		item_tooltip("Lengths of the transition curves from the speed of the paths and the limits of the curve design");
		if (false == state.auto_transitions)
		{
			ImGui::SameLine();
			ImGui::InputDouble(STR_C("Transition (m)"), &state.transition, 0.0, 0.0, "%.1f");
		}
		ImGui::PopItemWidth();
	}
	if (false == state.status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", state.status.c_str());
	if (false == state.set.empty())
	{
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.15f, 1.0f), STR_C("Group: %zu straights move together"), state.set.size());
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Clear")))
			state.set.clear();
	}

	if (false == ImGui::IsAnyItemActive())
	{
		state.edit_start = line.start;
		state.edit_end = line.end;
		state.edit_length = line.length;
		state.edit_azimuth = line.azimuth;
	}
	if (ImGui::CollapsingHeader(STR_C("Exact values")))
	{
		ImGui::PushItemWidth(-110.0f);
		ImGui::InputScalarN(STR_C("Start (x, y, z)"), ImGuiDataType_Double, glm::value_ptr(state.edit_start), 3, nullptr, nullptr, "%.3f");
		ImGui::InputScalarN(STR_C("End (x, y, z)"), ImGuiDataType_Double, glm::value_ptr(state.edit_end), 3, nullptr, nullptr, "%.3f");
		ImGui::PopItemWidth();
		if (ImGui::Button(STR_C("Apply ends")))
			straight_apply(line, state.edit_start, state.edit_end);
		ImGui::PushItemWidth(120.0f);
		ImGui::InputDouble(STR_C("Length (m)"), &state.edit_length, 0.0, 0.0, "%.3f");
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Set from the start##length")))
		{
			glm::dvec3 const direction{line.direction.x, line.grade, line.direction.y};
			straight_apply(line, line.start, line.start + direction * std::max(0.1, state.edit_length));
		}
		ImGui::InputDouble(STR_C("Azimuth (deg)"), &state.edit_azimuth, 0.0, 0.0, "%.5f");
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Turn around the start")))
		{
			auto const azimuth{glm::radians(state.edit_azimuth)};
			glm::dvec3 const direction{std::sin(azimuth), line.grade, std::cos(azimuth)};
			straight_apply(line, line.start, line.start + direction * line.length);
		}
		ImGui::PopItemWidth();
	}
}

bool editor_mode::straights_active()
{
	return ui()->mode() == nodebank_panel::TRACK && m_track_tab == track_tab::straights && false == current_straight().tracks.empty();
}

int editor_mode::straight_handle_hit()
{
	if (false == straights_active())
		return -1;
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
	return hit;
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
	if (false == tracks_editable(tracks, state.status))
		return;
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
		glm::dvec3 const ground{cursor_ground()};
		state.tool_handle = {ground.x, state.tool_handle.y, ground.z};
		typed_turn();
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
	glm::dvec3 handled{anchor};
	bool held{false};
	if (m_handle_drag.active)
	{
		held = handle_drag_step(handled);
		if (false == held && false == state.dragging)
			return;
	}
	else
	{
		if (false == state.dragging)
			state.gizmo = glm::translate(glm::mat4(1.0f), glm::vec3(anchor - frame.camera));
		frame.manipulate(ImGuizmo::TRANSLATE, state.gizmo, m_gizmo_snap);
		held = ImGuizmo::IsUsing();
		handled = frame.camera + glm::dvec3(state.gizmo[3]);
	}

	if (held)
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
			state.fitting = lines.size() == 1 && start_curve_fit(line, state.handle);
			auto const tracks{state.fitting ? std::vector<TTrack *>{} : editor_track::straight_affected(lines)};
			if (false == tracks_editable(tracks, state.status))
			{
				state.fitting = false;
				return;
			}
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
		glm::dvec3 moved{handled};
		std::function<glm::dvec3(glm::dvec3 const &)> transform;
		if (state.handle == 2)
		{
			auto offset{snap_straight_offset(grabbed, moved - (grabbed.start + grabbed.end) * 0.5)};
			if (auto const values{typed_values()}; false == values.empty())
				offset = glm::dvec3{-grabbed.direction.y, 0.0, grabbed.direction.x} * values[0];
			transform = [offset](glm::dvec3 const &Point) { return Point + offset; };
		}
		else
		{
			auto const pivot{state.handle == 0 ? grabbed.end : grabbed.start};
			auto const original{state.handle == 0 ? grabbed.start : grabbed.end};
			moved = snap_straight_direction(pivot, moved);
			if (auto const values{typed_values()}; false == values.empty() && values[0] > 0.1)
			{
				glm::dvec2 heading{moved.x - pivot.x, moved.z - pivot.z};
				if (glm::length(heading) < 1e-6)
					heading = {original.x - pivot.x, original.z - pivot.z};
				heading = glm::normalize(heading) * values[0];
				moved.x = pivot.x + heading.x;
				moved.z = pivot.z + heading.y;
			}
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

void editor_mode::cancel_track_tools()
{
	auto &state{m_straights};
	if (state.dragging)
	{
		// the geometry goes back to what it was before the drag
		if (false == state.fitting)
		{
			for (auto const &entry : state.drag_states)
				editor_track::apply(*entry.first, entry.second);
			m_track_dirty = true;
			commit_track_drag(true);
			m_track_drag.clear();
		}
		state.dragging = false;
		state.fitting = false;
		state.drag_states.clear();
		state.drag_lines.clear();
		straight_refresh();
	}
	m_extend.active = false;
	m_turntable.placing = false;
	m_signal.placing = false;
	m_signal.moving = false;
	m_handle_drag = {};
	if (m_point_drag.active)
	{
		m_track_snap = {};
		point_drag_finish();
	}
	if (m_switch.placing)
	{
		m_switch.placing = false;
		m_switch.status = STR_C("Placing of the switch cancelled");
	}
	else if (m_switch.armed >= 0)
	{
		m_switch.armed = -1;
		m_switch.status.clear();
	}
	m_track_point = {};
	m_route.vertex = -1;
	m_route.grip = -1;
	state.handle = -1;
	state.tool_placed = false;
	state.detour.clear();
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
	if (false == tracks_editable(tracks, m_straights.status))
	{
		undo_split();
		return;
	}
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	for (auto *track : tracks)
		states.emplace_back(track, editor_track::capture(*track));

	editor_track::move_straights({tail}, {{Tail(tail.start), Tail(tail.end)}});

	m_route.from = spans[first].track;
	m_route.to = spans[last].track;
	route_reset();
	if (m_route.chain.tracks.empty() || false == m_route.chain.switches.empty())
	{
		if (false == m_route.chain.switches.empty())
			m_route.error = STR_C("The curve would run through a switch");
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
		state.status = m_route.result.errors.empty() ? STR_C("The curve can't be fitted") : m_route.result.errors.front();
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
	editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error, true);
	straight_refresh();
	state.status = STR_C("Done, select a curve to adjust it in the Curve mode");
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
	glm::dvec3 const ground{cursor_ground()};
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
		typed_turn();
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
	if (track == nullptr)
		return;
	if (false == track->Dynamics.empty())
	{
		ui()->set_status(STR_C("The track can't be deleted while vehicles stand on it"), true);
		return;
	}
	// retired first, so the infrastructure it carried and moves elsewhere is in the same undo step
	editor_track::retire(*track);
	// a switch leaves its main track behind, so the line isn't broken, unless Shift asks for all of it
	std::vector<TTrack *> created;
	if (track->eType == tt_Switch && false == track->m_paths.empty() && false == Global.shiftState)
	{
		auto *main{editor_track::create_path(*track, track->m_paths.front())};
		editor_track::velocity(*main, speed_check::path_velocity(*track, 0));
		editor_track::commit({main});
		created.push_back(main);
		ui()->set_status(STR_C("The switch is replaced by its main track, Shift+Del removes all of it"), false);
	}
	push_track_snapshot({}, created, {track});
	// the model and the events of a turntable can't stay without its track
	if (track->eType == tt_Table)
		for (auto const instance : turntable_includes(*track))
		{
			scene::Layers.removed(instance, true);
			m_history.back().instances.push_back(instance);
		}
	if (std::find(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), track) != m_route.chain.tracks.end())
		m_route = {};
	m_node = nullptr;
	ui()->set_node(nullptr);
	m_track_point = {};
	straight_refresh();
}

std::vector<editor_mode::key_hint> editor_mode::track_key_hints(bool const All) const
{
	std::vector<key_hint> hints;
	bool gizmo{false};
	auto const *selected{selected_track()};
	if (m_lay.active)
	{
		if (m_lay.points.empty())
			hints = {{"LMB", "start of the new track, on the ground or at a free end"}, {"Drag from a free end", "straight along it, to another free end: a curve joining them"}, {"Esc", "stop laying"}};
		else
		{
			hints = {{"LMB", "next point, a curve goes at it"}, {"LMB on a free end", "join and lay"}, {"Enter", "lay up to the cursor"}, {"Backspace", "take back the point"}, {"Esc", "cancel"}};
			if (glm::dvec2 heading; lay_heading(heading))
				hints.insert(hints.begin() + 1, {"Shift", "straight on in the direction of the track"});
		}
	}
	else if (false == m_straights.detour.empty())
	{
		char const *const steps[] = {"leave the axis", "shifted", "start back", "back on the axis"};
		auto const step{std::min<std::size_t>(m_straights.detour.size(), 3)};
		hints = {{"Shift+click", format("point %zu of 4: %s", step + 1, steps[step])}, {"Esc", "cancel"}};
	}
	else if (m_extend.active && m_extend_freehand)
		hints = {{"Drag", "curve through the cursor, along the end: straight"}, {"Ctrl", "straight"}, {"Release", "build"}, {"Esc", "cancel"}};
	else if (m_extend.active)
		hints = {{"Drag", "straight along the end, to a free end: a curve joining it"}, {"Shift", "straight, joins nothing"}, {"Release", "build"}, {"Esc", "cancel"}};
	else if (m_switch.placing)
		hints = {{"Drag along", "direction of the switch"}, {"Drag sideways", "side of the diverging track"}, {"Esc", "cancel"}};
	else if (m_track_tab == track_tab::lay)
		hints = {{"LMB", "start of the new track, on the ground or at a free end"}, {"Esc", "back to Select"}};
	else if (m_track_tab == track_tab::turnout && (selected_track() == nullptr || selected_track()->eType == tt_Switch))
	{
		hints = {{"LMB on a track", "choose the straight or the curve to put the switch in"}};
		if (selected != nullptr)
		{
			hints.push_back({"Gizmo", "move the selected switch"});
			hints.push_back({"Del", "remove the switch, its main track stays"});
			gizmo = true;
		}
	}
	else if (m_track_tab == track_tab::turnout && m_switch.armed < 0)
		hints = {{"List below", "choose the type of the switch to put in"}, {"LMB on a track", "choose the straight or the curve"}, {"Esc", "back to Select"}};
	else if (m_track_tab == track_tab::turnout)
		hints = {{"1. List below", "choose the type of the switch"}, {"2. Press+drag", "on the selected track where the switch starts: along sets the direction, sideways the side"}, {"LMB elsewhere", "choose another track"}};
	else if (m_track_tab == track_tab::profile && m_profile.route.spans.empty())
		hints = {{"LMB on a track", "loads the grade line of its line"}};
	else if (m_track_tab == track_tab::profile)
		hints = {{"Drag a point", "in the profile window: changes the grade"}, {"Double click", "on the line: adds a point"}, {"Apply", "writes the heights into the tracks"}};
	else if (m_track_window_open && m_track_tab == track_tab::speed)
		hints = {{"LMB on a row", "show the path"}};
	else if (m_track_window_open && m_track_tab == track_tab::lineside)
		hints = {{"LMB on a track", "the line the objects go along"}};
	else if (m_track_window_open && m_track_tab == track_tab::signals && (m_signal.placing || m_signal.moving))
		hints = {{"Drag along", m_signal.moving ? "moves the signal along the track" : "the trains it faces run that way"}, {"Release", "done"}, {"Esc", "cancel"}};
	else if (m_track_window_open && m_track_tab == track_tab::signals)
	{
		hints = {{"LMB by a track", "places the signal on that side, facing the trains which have it on their right"}, {"Press+drag along", "the trains it faces run the way of the drag"}, {"LMB on a signal", "selects it"}, {"Drag a signal", "moves it along the track"}};
		if (m_signal.selected != 0)
			hints.push_back({"Del", "removes the selected signal with its event"});
	}
	else if (m_track_window_open && m_track_tab == track_tab::joints)
		hints = {{"LMB on a row", "show the place"}, {"Fix", "joins the ends, aligns them"}};
	else if (m_track_window_open && m_track_tab == track_tab::infra)
		hints = {{"LMB", "select a path"}};
	else if (m_track_tab == track_tab::straights && m_straights.tool == 1)
		hints = {{"LMB on the straight", "where it breaks, then drag the handle to turn the rest"}, {"Esc", "cancel"}};
	else if (m_track_tab == track_tab::straights && m_straights.tool == 2)
		hints = {{"LMB on the straight", "where the shift starts, then drag the handle: sideways the shift, along its length"}, {"Esc", "cancel"}};
	else if (m_track_tab == track_tab::straights && m_straights.current.tracks.empty())
		hints = {{"LMB on a straight track", "selects the whole straight"}};
	else if (m_track_tab == track_tab::straights)
	{
		hints = {{"Drag a square", "moves an end, the curves at it follow"}, {"Drag the diamond", "shifts the straight sideways"}, {"Ctrl+drag", "breaks the straight"}, {"Shift+click x4", "shifts a piece around an obstacle"}, {"Alt+click", "adds another straight to move together"}};
		gizmo = true;
	}
	else if (m_track_tab == track_tab::route && m_route.chain.tracks.empty())
		hints = {{"LMB on a curve", "loads it with the straights at its ends"}};
	else if (m_track_tab == track_tab::route)
		hints = {{"Drag yellow", "moves a vertex, the curve follows"}, {"Drag green", "changes the radius"}, {"Apply", "writes the design into the tracks"}, {"Esc", "release the vertex"}};
	else if (selected == nullptr)
		hints = {{"LMB on a track", "selects it"}, {"Shift+LMB", "adds a track to the selection, drag: a box"}, {"RMB on a track", "what can be done with it"}, {"L", "lay a new track"}};
	else if (m_point_drag.moved)
		hints = {{"Drag", "moves the point, level"}, {"Over a free end", "the end joins it"}, {"Release", "done"}, {"Esc", "done"}};
	else if (m_track_point.valid())
	{
		hints = {{"Drag a point", "moves it, an end dropped on a free end joins it"}, {"Gizmo", "moves the point along the axes"}, {"Esc", "releases the point"}};
		gizmo = true;
	}
	else
	{
		hints = {{"Drag a point", "moves it, an end dropped on a free end joins it"}, {"Gizmo", "moves the whole track"}, {"Shift+LMB", "adds a track to the selection, drag: a box"}, {"L", "new track, also from a free end"}, {"Esc", "deselects"}, {"Del", "deletes the track"}};
		gizmo = true;
	}
	if (auto const *meaning{typed_meaning()}; meaning != nullptr)
		hints.insert(hints.begin() + std::min<std::size_t>(1, hints.size()), {"0-9", meaning});
	if (All)
	{
		if (gizmo && m_gizmo_snap > 0.0f)
			hints.push_back({"Ctrl", format("gizmo snaps by %.2f m", m_gizmo_snap)});
		hints.push_back({"K", "split the path under the cursor"});
		hints.push_back({"O", "top view"});
		hints.push_back({"Ctrl+Z", "undo"});
		hints.push_back({"Ctrl+Y", "redo"});
		hints.push_back({"L T G C B P V J I", "modes: lay, switch, straight, curve, objects, profile, speed, joints, infra"});
	}
	for (auto &hint : hints)
	{
		hint.key = STR_C(hint.key);
		hint.action = STR(hint.action);
	}
	return hints;
}

// values of the change in progress, shown next to the cursor
std::string editor_mode::track_readout() const
{
	auto const name = [](TTrack const *Track) { return Track->name().empty() ? std::string{"(noname)"} : Track->name(); };
	auto const azimuth = [](glm::dvec3 const &From, glm::dvec3 const &To) {
		auto const degrees{glm::degrees(std::atan2(To.x - From.x, To.z - From.z))};
		return degrees < 0.0 ? degrees + 360.0 : degrees;
	};
	if (m_extend.active)
	{
		double length{0.0}, radius{0.0};
		auto const pieces{extend_pieces()};
		for (auto const &piece : pieces)
		{
			length += bezier{piece}.plan_length();
			if (piece.radius != 0.0f && (radius == 0.0 || std::abs(piece.radius) < radius))
				radius = std::abs(piece.radius);
		}
		editor_track::snap_target target;
		auto const snapped{extend_snap(target) && target.track != nullptr};
		if (snapped && pieces.empty())
			return "no track fits to the end of " + name(target.track);
		auto text{radius > 0.0 ? format("L %.2f m   R %.0f m", length, radius) : format("L %.2f m   straight", length)};
		if (snapped)
			text += "\njoins " + name(target.track);
		return text;
	}
	if (m_lay.active)
	{
		auto const &lay{m_lay};
		std::string text;
		if (lay.points.empty())
			text = lay.mouse_snap.track != nullptr ? "starts at the end of " + name(lay.mouse_snap.track) : "start";
		else if (false == lay.preview.empty())
			text = format("L %.1f m   R %.0f m   %zu paths", lay.preview_length, lay.radius, lay.preview.size());
		if (false == lay.points.empty() && lay.mouse_snap.track != nullptr)
			text += "\njoins " + name(lay.mouse_snap.track);
		if (false == lay.preview_error.empty())
			text += "\n" + lay.preview_error;
		return text;
	}
	auto const &state{m_straights};
	if (state.tool_placed && state.tool_dragging)
	{
		auto const &line{state.tool_line};
		auto const offset{plan_of(state.tool_handle - state.tool_point)};
		if (state.tool == 1)
		{
			if (glm::length(offset) < 1e-3)
				return {};
			auto const turn{glm::degrees(signed_angle(line.direction, glm::normalize(offset)))};
			return format("turn %+.3f deg   R %.0f m", turn, straight_tool_radius(line));
		}
		glm::dvec2 const normal{-line.direction.y, line.direction.x};
		return format("shift %+.3f m   over %.2f m", glm::dot(offset, normal), glm::dot(offset, line.direction));
	}
	if (state.dragging && state.fitting)
	{
		auto const &result{m_route.result};
		if (false == result.valid)
			return result.errors.empty() ? std::string{"no curve fits"} : result.errors.front();
		auto text{format("curve L %.2f m", result.length)};
		for (auto const &curve : result.curves)
			text += format("   deflection %.3f deg", glm::degrees(curve.deflection));
		return text;
	}
	if (state.dragging)
	{
		auto const &grabbed{state.drag_line};
		if (state.handle == 2)
		{
			glm::dvec2 const normal{-grabbed.direction.y, grabbed.direction.x};
			return format("shift %+.3f m", glm::dot(plan_of(state.preview_start - grabbed.start), normal));
		}
		return format("L %.3f m (%+.3f)   azimuth %.4f deg", plan_distance(state.preview_start, state.preview_end), plan_distance(state.preview_start, state.preview_end) - grabbed.length,
		              azimuth(state.preview_start, state.preview_end));
	}
	auto const *track{selected_track()};
	if (m_track_gizmo_using && track != nullptr)
	{
		if (m_track_point.valid() && m_track_point.path < static_cast<int>(track->m_paths.size()))
		{
			auto const &path{track->m_paths[m_track_point.path]};
			auto text{format("L %.3f m", bezier{path}.plan_length())};
			if (path.radius != 0.0f)
				text += format("   R %.1f m", std::abs(path.radius));
			if (m_track_snap.track != nullptr)
				text += "\njoins " + name(m_track_snap.track);
			return text;
		}
		return format("moved %.3f m", glm::distance(editor_track::pivot(*track), m_track_pivot));
	}
	return {};
}

void editor_mode::draw_track_hints()
{
	ImGuiIO const &io = ImGui::GetIO();
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	float const margin{12.0f};
	float const gap{14.0f};
	float const pad{4.0f};
	float const lineheight{ImGui::GetTextLineHeight() + 2.0f * pad + 4.0f};
	auto const chip = [&](ImVec2 const At, char const *Text, ImU32 const Background, ImU32 const Foreground) {
		auto const size{ImGui::CalcTextSize(Text)};
		drawlist->AddRectFilled(ImVec2(At.x, At.y - 2.0f), ImVec2(At.x + size.x + 2.0f * pad, At.y + size.y + 2.0f), Background, 3.0f);
		drawlist->AddText(ImVec2(At.x + pad, At.y), Foreground, Text);
		return size.x + 2.0f * pad;
	};
	auto const width_of = [&](key_hint const &Hint) { return ImGui::CalcTextSize(Hint.key).x + 2.0f * pad + 5.0f + ImGui::CalcTextSize(Hint.action.c_str()).x; };

	auto hints{track_key_hints(false)};
	if (hints.size() > 4)
		hints.resize(4);
	// in the part of the window the 3d view is seen in, between the docked windows
	auto const left{editor_ui::view_min().x + margin};
	auto const bottom{editor_ui::view_max().y};
	auto const available{(editor_ui::view_max().x - editor_ui::view_min().x) * 0.6f - 2.0f * margin};
	auto const mode{track_mode_name()};
	std::vector<std::vector<key_hint const *>> lines(1);
	float used{ImGui::CalcTextSize(mode.c_str()).x + 2.0f * pad + gap};
	for (auto const &hint : hints)
	{
		auto const width{width_of(hint)};
		if (false == lines.back().empty() && used + gap + width > available)
		{
			lines.emplace_back();
			used = 0.0f;
		}
		used += (lines.back().empty() ? 0.0f : gap) + width;
		lines.back().push_back(&hint);
	}
	auto y{bottom - margin - lineheight * static_cast<float>(lines.size()) + pad};
	for (std::size_t i = 0; i < lines.size(); ++i, y += lineheight)
	{
		float width{i == 0 ? ImGui::CalcTextSize(mode.c_str()).x + 2.0f * pad : 0.0f};
		for (auto const *hint : lines[i])
			width += (width > 0.0f ? gap : 0.0f) + width_of(*hint);
		drawlist->AddRectFilled(ImVec2(left - 6.0f, y - pad - 2.0f), ImVec2(left + width + 6.0f, y + lineheight - pad - 2.0f), IM_COL32(0, 0, 0, 160), 4.0f);
		auto x{left};
		if (i == 0)
			x += chip(ImVec2(x, y), mode.c_str(), IM_COL32(60, 140, 230, 235), IM_COL32(255, 255, 255, 255));
		for (auto const *hint : lines[i])
		{
			if (x > left)
				x += gap;
			x += chip(ImVec2(x, y), hint->key, IM_COL32(255, 210, 60, 220), IM_COL32(20, 20, 20, 255)) + 5.0f;
			drawlist->AddText(ImVec2(x, y), IM_COL32(255, 255, 255, 230), hint->action.c_str());
			x += ImGui::CalcTextSize(hint->action.c_str()).x;
		}
	}

	auto readout{track_readout()};
	if (auto const *meaning{typed_meaning()}; meaning != nullptr && false == m_typed.empty())
		readout += (readout.empty() ? "" : "\n") + std::string{"= "} + m_typed + "_   " + STR(meaning);
	if (readout.empty() || ImGui::GetIO().WantCaptureMouse)
		return;
	auto *foreground{ImGui::GetForegroundDrawList(ImGui::GetMainViewport())};
	auto const size{ImGui::CalcTextSize(readout.c_str())};
	ImVec2 at{io.MousePos.x + 20.0f, io.MousePos.y + 20.0f};
	at.x = std::min(at.x, io.DisplaySize.x - size.x - 10.0f);
	at.y = std::min(at.y, io.DisplaySize.y - size.y - 10.0f);
	foreground->AddRectFilled(ImVec2(at.x - 6.0f, at.y - 4.0f), ImVec2(at.x + size.x + 6.0f, at.y + size.y + 4.0f), IM_COL32(0, 0, 0, 190), 4.0f);
	foreground->AddRect(ImVec2(at.x - 6.0f, at.y - 4.0f), ImVec2(at.x + size.x + 6.0f, at.y + size.y + 4.0f), overlay_color::marked, 4.0f);
	foreground->AddText(at, IM_COL32(255, 255, 255, 255), readout.c_str());
}

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
	auto mouse{ground};
	glm::dvec2 heading;
	bool const straight{io.KeyShift && lay_heading(heading)};
	if (straight)
	{
		auto const &last{lay.points.back()};
		auto const along{std::max(0.0, glm::dot(glm::dvec2{ground.x - last.x, ground.z - last.z}, heading))};
		mouse = {last.x + heading.x * along, ground.y, last.z + heading.y * along};
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
	if (false == projection.on_level(ImGui::GetIO().MousePos, Height, level) || glm::distance(level, glm::dvec3{Global.pCamera.Pos}) > static_cast<double>(kMaxPlacementDistance))
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
	if (tool.armed < 0 || m_track_tab != track_tab::turnout)
		return false;
	// with no plain track selected, or a press away from it, the click selects
	auto *selected{selected_track()};
	if (selected == nullptr || selected->eType != tt_Normal || selected->m_paths.empty())
		return false;
	glm::dvec3 const ground{cursor_level(selected->m_paths.front().points[segment_data::point::start].y)};
	tool.curved = false;
	tool.status.clear();
	if (current_straight().tracks.empty())
	{
		auto *track{selected};
		if (tool.templates[tool.armed].double_slip)
		{
			tool.status = STR_C("A double slip goes only into a straight crossing another one");
			return false;
		}
		editor_track::curve curve;
		editor_track::chain chain;
		std::string error;
		if (false == editor_track::find_curve(*track, m_straights.tolerance, m_route.design.norms.gauge, curve))
		{
			tool.status = STR_C("The selected track is neither a recognized straight nor a curve");
			return false;
		}
		if (false == editor_track::find_chain(curve.from, curve.to, chain, error, true))
		{
			tool.status = error.empty() ? std::string{STR_C("The curve can't be followed")} : error;
			return false;
		}
		for (auto const &part : editor_track::chain_parts(chain))
			if (std::find(part.tracks.begin(), part.tracks.end(), track) != part.tracks.end())
			{
				chain = part;
				break;
			}
		tool.frame = sample_chain(chain);
		tool.frame_tracks = chain.tracks;
		if (tool.frame.size() < 2)
			return false;
		tool.curved = true;
		tool.along = nearest_station(tool.frame, ground);
		tool.point = sample_at(tool.frame, tool.along).position;
		if (plan_distance(tool.point, ground) > kSwitchReach)
			return false;
		tool.mouse = ground;
		tool.placing = true;
		return true;
	}
	auto const &line{current_straight()};
	auto const along{std::clamp(line.along(ground), 0.0, line.length)};
	auto const point{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * along};
	if (plan_distance(point, ground) > kSwitchReach)
		return false;
	if (tool.templates[tool.armed].double_slip)
	{
		insert_double_slip(line, ground, tool.templates[tool.armed]);
		return true;
	}
	tool.along = along;
	tool.point = point;
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
	if (false == place_switch_on_straight(Line, m_switch.templates[m_switch.armed], nullptr, Along, Direction, Side, true, states, created, removed, switch_name_for_new()))
		return;
	auto const placed{created};
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(placed);
}

void editor_mode::switch_placed(std::vector<TTrack *> const &Created)
{
	auto const found{std::find_if(Created.begin(), Created.end(), [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; })};
	if (found == Created.end())
		return;
	auto const slip{std::count_if(Created.begin(), Created.end(), [](TTrack const *Track) { return Track != nullptr && Track->eType == tt_Switch; }) > 1};
	m_switch.armed = -1;
	m_switch.placing = false;
	m_track_point = {};
	m_node = *found;
	ui()->set_node(m_node);
	if (slip)
		return;
	if (m_switch.drive >= 0)
		place_switch_drive(**found);
	std::string name{m_switch.name};
	auto digits{name.size()};
	while (digits > 0 && std::isdigit(static_cast<unsigned char>(name[digits - 1])))
		--digits;
	if (digits < name.size() && name.size() - digits < 9)
		name = name.substr(0, digits) + std::to_string(std::stoi(name.substr(digits)) + 1);
	else
		name.clear();
	std::snprintf(m_switch.name, sizeof(m_switch.name), "%s", name.c_str());
}

void editor_mode::scan_switch_drives()
{
	auto &tool{m_switch};
	if (tool.drives_scanned)
		return;
	tool.drives_scanned = true;
	tool.drives.clear();
	if (false == EditorIncludes.scanned())
		EditorIncludes.scan();
	for (auto const &entry : EditorIncludes.entries())
	{
		if (entry.category != "switch drive")
			continue;
		include_info info;
		std::string error;
		if (false == editor_includes::load(entry.file, info, error) || editor_includes::parameter_with_role(info, "track") == 0)
			continue;
		auto const side = [](std::string const &Side) { return Side == "left" ? 0 : Side == "right" ? 1 : -1; };
		auto const hand{side(info.switch_hand)};
		auto const drive{side(info.switch_drive)};
		if (hand < 0 || drive < 0)
			continue;
		auto const name{info.name.empty() ? entry.file : info.name};
		auto family{std::find_if(tool.drives.begin(), tool.drives.end(), [&](switch_tool::drive_family const &Family) { return Family.name == name; })};
		if (family == tool.drives.end())
		{
			tool.drives.push_back({name, {}});
			family = std::prev(tool.drives.end());
		}
		family->files[hand * 2 + drive] = entry.file;
	}
	std::sort(tool.drives.begin(), tool.drives.end(), [](switch_tool::drive_family const &A, switch_tool::drive_family const &B) { return A.name < B.name; });
	tool.drive = tool.drives.empty() ? -1 : 0;
}

std::string editor_mode::switch_name_for_new() const
{
	if (m_switch.name[0] != '\0')
		return m_switch.name;
	for (int i = 1;; ++i)
	{
		auto const name{"z" + std::to_string(i)};
		if (simulation::Paths.find(name) == nullptr && simulation::Events.FindEvent(name + "+") == nullptr)
			return name;
	}
}

bool editor_mode::place_switch_drive(TTrack &Switch)
{
	auto &tool{m_switch};
	if (tool.drive < 0 || tool.drive >= static_cast<int>(tool.drives.size()) || Switch.eType != tt_Switch || Switch.m_paths.size() < 2)
		return false;
	auto const &main{Switch.m_paths[0]};
	auto const &diverging{Switch.m_paths[1]};
	auto const start{main.points[segment_data::point::start]};
	auto const &control{main.points[segment_data::point::control1]};
	auto heading{plan_of(control != glm::dvec3{} ? control : main.points[segment_data::point::end] - start)};
	if (glm::length(heading) < 1e-9)
		return false;
	heading = glm::normalize(heading);
	glm::dvec2 const right{-heading.y, heading.x};
	bool const rightswitch{glm::dot(plan_of(diverging.points[segment_data::point::end] - start), right) > 0.0};
	bool const driveright{(tool.drive_side == 0) == rightswitch};
	auto const &family{tool.drives[tool.drive]};
	template_item item;
	item.file = family.files[(rightswitch ? 2 : 0) + (driveright ? 1 : 0)];
	if (item.file.empty())
	{
		tool.status = format(STR_C("The drive %s has no variant for a %s switch with the drive on the %s"), family.name.c_str(), rightswitch ? STR_C("right") : STR_C("left"), driveright ? STR_C("right") : STR_C("left"));
		return false;
	}
	item.location = start;
	auto yaw{glm::degrees(std::atan2(heading.x, heading.y))};
	item.yaw = yaw < 0.0 ? yaw + 360.0 : yaw;
	item.described = true;
	item.track = Switch.name();
	{
		auto const along{control != glm::dvec3{} ? control : main.points[segment_data::point::end] - start};
		auto const run{glm::length(plan_of(along))};
		auto const roll{static_cast<double>(main.rolls[0])};
		item.tilt = {run > 1e-9 ? -glm::degrees(std::atan(along.y / run)) : 0.0, roll};
		if (Global.bRollFix)
			item.location.y += 0.75 * std::abs(std::sin(glm::radians(roll)));
	}
	std::string error;
	if (place_templates({item}, error) == 0)
	{
		tool.status = STR_C("The drive couldn't be placed: ") + error;
		return false;
	}
	tool.status = format(STR_C("Switch %s with the drive %s: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), item.file.c_str(), Switch.name().c_str(), Switch.name().c_str());
	return true;
}

bool editor_mode::render_switch_drive_choice()
{
	auto &tool{m_switch};
	scan_switch_drives();
	if (tool.drives.empty())
	{
		ImGui::TextDisabled("%s", STR_C("No switch drives: no template is described with category: switch drive"));
		return false;
	}
	auto const label = [&](int const Index) { return Index < 0 ? std::string{STR_C("no drive")} : tool.drives[Index].name; };
	ImGui::PushItemWidth(200.0f);
	bool changed{false};
	if (ImGui::BeginCombo(STR_C("Drive"), label(tool.drive).c_str()))
	{
		for (int i = -1; i < static_cast<int>(tool.drives.size()); ++i)
			if (ImGui::Selectable(label(i).c_str(), tool.drive == i))
			{
				tool.drive = i;
				changed = true;
			}
		ImGui::EndCombo();
	}
	ImGui::PopItemWidth();
	item_tooltip("Templates described with category: switch drive, the side of the switch and of the drive, and a parameter with the role track for the name of the switch.\nIts events <name>+ and <name>- switch it with the keys by the switch and in the map during the game");
	if (tool.drive >= 0)
	{
		ImGui::SameLine();
		changed |= ImGui::RadioButton(STR_C("diverging side"), &tool.drive_side, 0);
		ImGui::SameLine();
		changed |= ImGui::RadioButton(STR_C("other side"), &tool.drive_side, 1);
	}
	return changed;
}

bool editor_mode::cut_straight(editor_track::straight const &Line, double const From, double const To, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack **Style)
{
	auto const from{From};
	auto const to{To};
	std::string reason;
	if (false == tracks_editable(Line.tracks, reason))
	{
		m_switch.status = reason;
		ui()->set_status(reason, true);
		return false;
	}
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

bool editor_mode::place_switch_on_straight(editor_track::straight const &Line, editor_track::switch_template const &Shape, TTrack const *Style, double const Along, int const Direction, int const Side, bool const Snap, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, std::string const &Name)
{
	auto const &shape{Shape};
	if (shape.length > Line.length - 0.02)
	{
		m_switch.status = format(STR_C("The straight is %.1f m long, too short for the %.1f m switch"), Line.length, shape.length);
		return false;
	}
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
	{
		m_switch.status = STR_C("The switch would overlap another switch, move it onto plain track");
		return false;
	}
	auto const point = [&](double const Distance) { return Line.start + glm::dvec3{Line.direction.x, Line.grade, Line.direction.y} * Distance; };
	glm::dvec2 const direction{Line.direction * static_cast<double>(Direction)};
	auto const paths{editor_track::place_switch(shape, point(origin), direction, Side, Line.grade * Direction)};
	auto *track{editor_track::create_switch(shape, paths, Style != nullptr ? *Style : *first, Name)};
	if (track != nullptr)
	{
		editor_track::commit({track});
		Created.push_back(track);
		m_switch.status = STR_C("Switch ") + track->name() + STR_C(" inserted");
	}
	else
		m_switch.status = STR_C("The switch couldn't be created from this template");
	return true;
}

void editor_mode::draw_build_overlay() const
{
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
	auto const drawpath = [&](segment_data const &Path, ImU32 const Color) {
		auto previous{bezier{Path}.point(0.0)};
		for (int i = 1; i <= 24; ++i)
		{
			auto const next{bezier{Path}.point(i / 24.0)};
			projection.line(drawlist, previous, next, Color, 3.0f);
			previous = next;
		}
	};
	glm::dvec3 const camera{Global.pCamera.Pos};
	auto const &display{ImGui::GetIO().DisplaySize};
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		auto const &first{track->m_paths.front().points};
		if (glm::distance(first[segment_data::point::start], camera) > kJointRange && glm::distance(first[segment_data::point::end], camera) > kJointRange)
			continue;
		for (auto const &path : track->m_paths)
			for (auto const point : {segment_data::point::start, segment_data::point::end})
			{
				ImVec2 screen;
				if (projection.project(path.points[point], screen) && screen.x >= 0.0f && screen.y >= 0.0f && screen.x <= display.x && screen.y <= display.y)
					drawlist->AddCircleFilled(screen, 3.5f, kJointColor);
			}
	}
	if (m_extend.active)
	{
		if (m_extend.track != nullptr)
			draw_free_ends(m_extend.track, m_extend.track->iCategoryFlag & 15, {m_extend.track});
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
	if (m_lay.active)
	{
		auto const &lay{m_lay};
		for (auto const &piece : lay.preview)
			drawpath(piece, lay.preview_error.empty() ? overlay_color::marked : overlay_color::grip);
		for (std::size_t i = 0; i + 1 < lay.points.size(); ++i)
			projection.line(drawlist, lay.points[i], lay.points[i + 1], IM_COL32(255, 255, 255, 90), 1.0f);
		for (auto const &point : lay.points)
		{
			ImVec2 screen;
			if (projection.project(point, screen))
				drawlist->AddCircleFilled(screen, 5.0f, overlay_color::grip);
		}
		ImVec2 screen;
		if (lay.mouse_snap.track != nullptr && projection.project(lay.mouse_snap.position, screen))
			drawlist->AddCircle(screen, 14.0f, IM_COL32(255, 60, 255, 255), 20, 3.0f);
		if (lay.start.track != nullptr && projection.project(lay.start.position, screen))
			drawlist->AddCircle(screen, 10.0f, IM_COL32(255, 60, 255, 255), 20, 2.0f);
		if (false == m_extend.active)
			draw_free_ends(nullptr, kRailCategory, {});
	}
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
	if ((m_point_drag.moved || (m_track_gizmo_using && m_track_point.valid())) && editor_track::is_end(m_track_point.kind) && m_track_drag_points.empty())
		if (auto const *track{selected_track()}; track != nullptr)
			draw_free_ends(track, track->iCategoryFlag & 15, std::vector<TTrack const *>(m_track_drag.begin(), m_track_drag.end()));
}

void editor_mode::render_switch_ui()
{
	auto &tool{m_switch};
	auto const list = [&](bool const Slips) {
		for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
		{
			auto const &entry{tool.templates[i]};
			if (entry.double_slip != Slips)
				continue;
			auto label{entry.source != nullptr ? STR("Scenery: ") + entry.label + "  (" + std::to_string(entry.count) + "x)" : "PLK: " + entry.label};
			label += "##switch" + std::to_string(i);
			if (ImGui::Selectable(label.c_str(), tool.armed == i))
				tool.armed = tool.last = i;
		}
	};
	ImGui::TextDisabled("%s", STR_C("Switch template"));
	ImGui::BeginChild("##switchtemplates", ImVec2(0.0f, 240.0f), true);
	list(false);
	ImGui::EndChild();
	if (std::any_of(tool.templates.begin(), tool.templates.end(), [](editor_track::switch_template const &Template) { return Template.double_slip; }))
	{
		if (tool.armed >= 0 && tool.armed < static_cast<int>(tool.templates.size()) && tool.templates[tool.armed].double_slip)
			ImGui::SetNextItemOpen(true);
		if (ImGui::CollapsingHeader(STR_C("Double slip switches")))
		{
			ImGui::TextDisabled("%s", STR_C("Only at a crossing of two straights"));
			list(true);
		}
	}
	ImGui::PushItemWidth(120.0f);
	ImGui::InputTextWithHint(STR_C("Name##switchname"), switch_name_for_new().c_str(), tool.name, sizeof(tool.name));
	ImGui::PopItemWidth();
	item_tooltip("Name of the next switch, its drive events are named after it; empty: the next free z1, z2...; a number at the end goes up with each switch");
	render_switch_drive_choice();
	if (false == tool.status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", tool.status.c_str());
	if (ImGui::SmallButton(STR_C("Collect again from the scenery")))
	{
		tool.collected = false;
		arm_switch();
	}
	item_tooltip("Looks for the switches of the scenery again, to use their shapes as the templates");
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
	state.detour.push_back(cursor_ground());
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

bool editor_mode::start_curve_fit(editor_track::straight const &Line, int const Handle)
{
	auto &state{m_straights};
	auto *before{editor_track::outside_neighbour(Line, false)};
	auto *after{editor_track::outside_neighbour(Line, true)};
	auto const curve_at = [&](TTrack *Neighbour, editor_track::curve &Curve) {
		Curve = {};
		return Neighbour != nullptr && Neighbour->eType == tt_Normal && editor_track::find_curve(*Neighbour, state.tolerance, m_route.design.norms.gauge, Curve) && Curve.reversals == 0 && std::abs(Curve.turn) > 1e-4;
	};
	state.fit_has_before = curve_at(before, state.fit_before);
	state.fit_has_after = curve_at(after, state.fit_after);
	if (false == state.fit_has_before && false == state.fit_has_after)
		return false;
	auto const side_follows = [&](bool const Curve, TTrack const *Neighbour, bool const Pivot) { return Curve || Neighbour == nullptr || Pivot; };
	if (false == side_follows(state.fit_has_before, before, Handle == 1) || false == side_follows(state.fit_has_after, after, Handle == 0))
		return false;
	auto const own_end = [&](glm::dvec3 const &Point) -> TTrack * {
		for (auto *track : Line.tracks)
			if (editor_track::touches(*track, Point))
				return track;
		return Line.tracks.empty() ? nullptr : Line.tracks.front();
	};
	auto const other_end = [](editor_track::curve const &Curve, glm::dvec3 const &Joint) { return editor_track::touches(*Curve.from, Joint) ? Curve.to : Curve.from; };
	m_route.from = state.fit_has_before ? other_end(state.fit_before, Line.start) : own_end(Line.start);
	m_route.to = state.fit_has_after ? other_end(state.fit_after, Line.end) : own_end(Line.end);
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
		compound_from(Curve, vertex);
		vertex.offset = std::max(0.1, Offset);
		return vertex;
	};
	design.vertices.clear();
	if (state.fit_has_before)
		design.vertices.push_back(make(state.fit_before, first));
	else
	{
		design.start = Start;
		design.start_direction = direction;
		design.start_reserve = 0.0;
	}
	if (state.fit_has_after)
		design.vertices.push_back(make(state.fit_after, last));
	else
	{
		design.end = End;
		design.end_direction = direction;
		design.end_reserve = 0.0;
	}
	route_update();
}

bool editor_mode::plan_crossover(crossover_plan &Plan) const
{
	auto const &tool{m_extend};
	if (false == tool.active || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() < 2 || tool.path != 1 || false == tool.atend || ImGui::GetIO().KeyShift)
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
	// the ends of the two diagonals at the crossing are kept apart further than the simulation joins the ends, 2 cm
	auto const halfsine{std::sqrt(std::max(1e-6, (1.0 - glm::dot(First, Second)) * 0.5))};
	auto const gap{std::max(0.25, 0.06 / halfsine)};
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
		state.status = STR_C("No straight crossing the selected one near the click");
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
		state.status = STR_C("The straights cross at an angle unsuitable for a double slip");
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
			state.status = STR_C("Not enough straight track around the crossing");
			return false;
		}
		state.status = STR_C("R reduced to ") + std::to_string(static_cast<int>(radius)) + STR_C(" m, the straights end ") + std::to_string(static_cast<int>(available)) + STR_C(" m from the crossing");
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
	switch_placed(parts);
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
			state.status = STR_C("The double slip isn't complete, it can't be replaced");
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
		state.status = STR_C("The double slip geometry isn't recognized");
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
				state.status = STR_C("R limited to ") + std::to_string(static_cast<int>(radius)) + STR_C(" m, a neighbouring path isn't straight");
				break;
			}
			auto const line{editor_track::find_straight(*e->outer, m_straights.tolerance)};
			auto const reach{std::abs(glm::dot(plan_of(line.end) - crossing, -e->inward))};
			auto const reachstart{std::abs(glm::dot(plan_of(line.start) - crossing, -e->inward))};
			if (std::max(reach, reachstart) - 0.5 < half)
			{
				half = std::min(half, std::max(reach, reachstart) - 0.5);
				radius = half / std::tan(angle * 0.5);
				state.status = STR_C("R reduced to ") + std::to_string(static_cast<int>(radius)) + STR_C(" m to fit the straights");
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
		{
			auto const cone = [&](double const T) {
				auto const point{bezier{path}.point(T)};
				auto const sample{sample_at(Frame, nearest_station(Frame, point))};
				auto const lateral{glm::dot(plan_of(point) - plan_of(sample.position), glm::dvec2{-sample.tangent.y, sample.tangent.x})};
				return sample.position.y - lateral * std::tan(glm::radians(static_cast<double>(sample.roll)));
			};
			auto const h1{cone(1.0 / 3.0)};
			auto const h2{cone(2.0 / 3.0)};
			auto const y1{3.0 * h1 - 1.5 * h2 - 5.0 / 6.0 * p0.y + p3.y / 3.0};
			auto const y2{3.0 * h2 - 1.5 * h1 + p0.y / 3.0 - 5.0 / 6.0 * p3.y};
			path.points[segment_data::point::control1].y = y1 - p0.y;
			path.points[segment_data::point::control2].y = y2 - p3.y;
		}
		auto const rollstart{sample_at(Frame, Station + Direction * start.z).roll * Direction};
		auto const rollend{sample_at(Frame, Station + Direction * end.z).roll * Direction};
		path.rolls = {static_cast<float>(rollstart), static_cast<float>(rollend)};
		path.radius = local.radius;
		{
			bezier const curve{path};
			auto const begin{plan_of(curve.first(0.0))};
			auto const finish{plan_of(curve.first(1.0))};
			auto const turn{glm::length(begin) > 1e-9 && glm::length(finish) > 1e-9 ? geometry::signed_angle(begin, finish) : 0.0};
			if (local.radius != 0.f && std::abs(turn) > 1e-6)
				path.radius = static_cast<float>(std::copysign(curve.plan_length() / std::abs(turn), static_cast<double>(local.radius)));
		}
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
	if (shape.length > total - 0.02)
	{
		tool.status = format(STR_C("The curve is %.1f m long, too short for the %.1f m switch"), total, shape.length);
		return;
	}
	auto const station{std::clamp(tool.along, Direction > 0 ? 0.0 : shape.length, Direction > 0 ? total - shape.length : total)};
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
	auto *track{editor_track::create_switch(shape, paths, *tool.frame_tracks.front(), switch_name_for_new())};
	if (track != nullptr)
	{
		editor_track::commit({track});
		created.push_back(track);
		tool.status = STR_C("Switch ") + track->name() + STR_C(" inserted into the curve");
	}
	else
		tool.status = STR_C("The switch couldn't be created from this template");
	auto const placed{created};
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(placed);
}
