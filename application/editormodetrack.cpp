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
#include "application/editormodeshared.h"
#include "editor/editorFormat.hpp"
#include "editor/editorAlignmentStore.hpp"
#include "editor/editorSwitchStore.hpp"
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
#include <iterator>
#include <set>

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
using geometry::arc_through;
using geometry::has_controls;
using geometry::level;
using geometry::reversed;
using geometry::straight_piece;

std::string template_label(editor_track::switch_template const &Template)
{
	if (Template.source != nullptr)
		return STR("Scenery: ") + Template.label + "  (" + std::to_string(Template.count) + "x)";
	return (Template.own ? STR("Own: ") : std::string{"PLK: "}) + Template.label;
}

double constexpr kParallelFindAngle{15.0}; // deg, neighbours counted as beside the straight
double constexpr kParallelSnapAngle{5.0}; // deg, close enough to snap parallel
double constexpr kParallelSnapWindow{0.45}; // m from 4.00 / 4.50 / 4.75 m
double constexpr kParallelHintWindow{1.25}; // m, hint before the snap takes

double piece_plan_radius(segment_data const &Path)
{
	if (std::abs(Path.radius) > 0.5f)
		return std::abs(Path.radius);
	bezier const curve{Path};
	auto const begin{plan_of(curve.first(0.0))};
	auto const finish{plan_of(curve.first(1.0))};
	if (glm::length(begin) < 1e-9 || glm::length(finish) < 1e-9)
		return 0.0;
	auto const turn{std::abs(signed_angle(begin, finish))};
	if (turn < 1e-4)
		return 0.0;
	return curve.plan_length() / turn;
}

std::string lines_of(std::vector<std::string> const &Texts)
{
	std::string text;
	for (auto const &line : Texts)
		text += (text.empty() ? "" : "\n") + line;
	return text;
}

std::string narrowed(std::vector<segment_data> const &Pieces, editor_track::layout const Layout, double const Radius)
{
	if (Layout != editor_track::layout::arc_straight && Layout != editor_track::layout::straight_arc)
		return {};
	double sharpest{0.0};
	for (auto const &piece : Pieces)
		if (auto const radius{piece_plan_radius(piece)}; radius >= 0.5 && (sharpest == 0.0 || radius < sharpest))
			sharpest = radius;
	if (sharpest == 0.0 || sharpest >= Radius * 0.99)
		return {};
	return format(STR_C("R %.0f m doesn't reach the cursor, the arc goes through it with R %.0f m"), Radius, sharpest);
}

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
double const kSwitchReach{6.0}; // a press farther from the selected track selects another one instead of placing the switch
double const kLineReach{2000.0}; // how far each way the Curve tab follows the line
double const kDigitizeFit{0.5};
double const kDrawnReach{100000.0};
double const kJointRange{400.0}; // the ends of the paths are drawn for the tracks this close to the camera
ImU32 const kJointColor{IM_COL32(255, 255, 255, 200)};
int const kRailCategory{1};

editor_track::branch_end branch_start(segment_data const &Diverging)
{
	auto const tangent{bezier{Diverging}.first(1.0)};
	return {Diverging.points[segment_data::point::end], plan_of(tangent), grade_of(tangent), static_cast<double>(Diverging.rolls[1])};
}

editor_track::branch_end branch_target(editor_track::snap_target const &Target)
{
	return {Target.position, plan_of(Target.direction), grade_of(Target.direction), Target.track != nullptr ? editor_track::joint_roll(*Target.track, Target.point) : 0.0};
}

double switch_from(editor_track::straight const &Line, double const Length, double const Along, int const Direction, bool const Snap)
{
	auto from{std::clamp(Direction > 0 ? Along : Along - Length, 0.0, Line.length - Length)};
	if (false == Snap)
		return from;
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
	auto const snappedend{nearest(from + Length)};
	if (snappedstart != from)
		from = snappedstart;
	else if (snappedend != from + Length)
		from = snappedend - Length;
	return std::clamp(from, 0.0, Line.length - Length);
}

std::string link_summary(editor_track::snap_target const &Target, editor_track::branch_link const &Link, double const Switchradius)
{
	auto const name{Target.track != nullptr && false == Target.track->name().empty() ? Target.track->name() : std::string{"(noname)"}};
	auto text{Link.reverse ? format(STR_C("The branch leads into %s with reverse curves R %.0f m"), name.c_str(), Link.radius) :
	          Link.radius > 0.0 ? format(STR_C("The branch leads into %s with an R %.0f m curve"), name.c_str(), Link.radius) : format(STR_C("The branch leads straight into %s"), name.c_str())};
	if (Link.insert > 0.0)
		text += format(STR_C(", a straight of %.1f m between them"), Link.insert);
	if (Link.reverse && Link.radius < Switchradius - 0.5)
		text += format(STR_C(", sharper than the switch (R %.0f m)"), Switchradius);
	if (Link.trim > 0.0)
		text += format(STR_C(", %s shortened %.2f m"), name.c_str(), Link.trim);
	else if (Link.trim < 0.0)
		text += format(STR_C(", %s lengthened %.2f m"), name.c_str(), -Link.trim);
	return text;
}

std::optional<alignment::join_result> eased_join(alignment::join_request const &Request, editor_track::easing const &In, editor_track::easing const &Out)
{
	if (In.pieces.empty() && Out.pieces.empty())
		return std::nullopt;
	auto middle{Request};
	if (false == In.pieces.empty())
	{
		middle.start.position = In.end;
		middle.start.direction = In.direction;
		middle.start.roll = In.pieces.back().rolls[1];
	}
	if (false == Out.pieces.empty())
	{
		middle.end.position = Out.end;
		middle.end.direction = -Out.direction;
		middle.end.roll = -Out.pieces.back().rolls[1];
	}
	if (false == editor_track::eases_room(In, Out, Request.start.position, Request.start.direction, Request.end.position, Request.end.direction))
		return std::nullopt;
	auto eased{alignment::join(middle)};
	if (eased.kind == alignment::join_kind::none || eased.kind == alignment::join_kind::bezier)
		return std::nullopt;
	eased.length = editor_track::eased_length(In, eased.length, Out);
	eased.pieces = editor_track::eased_pieces(In, eased.pieces, Out);
	return eased;
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
	if (track_plan_tab() && plan_tab_for(track) != m_track_tab)
	{
		show_track_tab(plan_tab_for(track));
		return;
	}
	// the mode stays as it is, the path is taken up by it if it can be
	switch (m_track_tab)
	{
	case track_tab::straights:
		if (track->eType == tt_Switch)
			m_track_mode_notice = STR_C("Switches aren't a part of a line: T edits the switch, Esc goes back to Select");
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
	snap.alignments = alignment_store::take_changes();
	snap.switch_records = switch_store::take_changes();
	snap.directives.insert(snap.directives.end(), m_drive_buffer.begin(), m_drive_buffer.end());
	m_drive_buffer.clear();
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
	alignment_store::settle();
	switch_store::settle();
	if (Snapshot.alignments.has_value())
	{
		current.alignments = alignment_store::records();
		alignment_store::restore(*Snapshot.alignments);
	}
	if (Snapshot.switch_records.has_value())
	{
		current.switch_records = switch_store::records();
		switch_store::restore(*Snapshot.switch_records);
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
	m_drive_buffer.clear();
	m_infra_suspended = false;
	if (false == m_route.chain.tracks.empty())
	{
		std::string error;
		if (m_route.from->m_editorremoved || m_route.to->m_editorremoved || false == editor_track::find_chain(m_route.from, m_route.to, m_route.chain, error, true))
		{
			m_route.chain = {};
			m_route.result = {};
			m_route.vertex = -1;
			m_route.grip = -1;
		}
		else
		{
			route_bind_ends();
			route_update();
		}
	}

	lay_after_history(Undo);
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
	    {"Plan", "G", track_tab::straights, "The plan of the line laid: LMB on a straight drags its ends or the middle, on a curve its vertex and radius. C works too"},
	    {"Switch", "T", track_tab::turnout, "Puts switches into the selected straight or curve, edits the geometry of the selected switch"},
	    {"Parallel", "N", track_tab::parallel, "A second track beside a straight: LMB on the straight, then the side and the spacing"},
	    {"Turntable", "U", track_tab::turntable, "Places a turntable, leads tracks out of it and fits the tracks around to it"},
	    {"Signals", "H", track_tab::signals, "Signals by the track, from the templates of the scenery, with the event the train reads them by"},
	    {"Objects", "B", track_tab::lineside, "Along the track: hectometre posts, fouling point markers, trainsets"},
	    {"Profile", "P", track_tab::profile, "Vertical profile (grade line) along the line"},
	    {"Speed", "V", track_tab::speed, "Speed limits of the paths against the speed their geometry allows"},
	    {"Joints", "J", track_tab::joints, "Ends of the paths which almost meet, steps, kinks, jumps of the cant and of the grade at the joints"},
	    {"Infra", "I", track_tab::infra, "Infrastructure along the track, bound to follow its changes"},
	}};
	return modes;
}

void editor_mode::show_track_tab(track_tab const Wanted)
{
	auto const Tab{Wanted == track_tab::straights || Wanted == track_tab::route ? plan_tab_for(selected_track()) : Wanted};
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
	m_lay.chain.clear();
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
		if (track != nullptr && track->eType == tt_Switch)
			m_track_mode_notice = STR_C("Switches aren't a part of a line: LMB on a plain path");
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
	auto const key{Key == GLFW_KEY_C ? GLFW_KEY_G : Key};
	for (auto const &mode : track_modes())
	{
		if (mode.key[1] != '\0' || key != GLFW_KEY_A + (mode.key[0] - 'A'))
			continue;
		// the key of the mode on leaves it
		auto const on{m_track_tab == mode.tab || (mode.tab == track_tab::straights && track_plan_tab())};
		show_track_tab(m_track_window_open && on ? track_tab::path : mode.tab);
		return true;
	}
	return false;
}

editor_mode::track_tab editor_mode::plan_tab_for(TTrack const *Track) const
{
	if (Track == nullptr)
		return track_plan_tab() ? m_track_tab : track_tab::straights;
	if (Track->eType == tt_Normal && editor_track::is_supported(*Track) && false == editor_track::is_straight(*Track, m_straights.tolerance))
		return track_tab::route;
	return track_tab::straights;
}

bool editor_mode::track_busy() const
{
	auto const &straights{m_straights};
	return (m_lay.active && false == m_lay.points.empty()) || m_turntable.placing || m_signal.placing || m_signal.moving || straights.dragging || m_extend.active || m_point_drag.active || m_handle_drag.active || m_switch.placing || m_track_point.valid() || m_route.vertex >= 0 || m_route.grip >= 0 || false == straights.detour.empty() || straights.tool != 0 || straights.handle >= 0;
}

std::string editor_mode::track_mode_name() const
{
	if (false == m_track_window_open)
		return STR_C("Select");
	switch (m_track_tab)
	{
	case track_tab::lay: return STR_C("Lay track");
	case track_tab::turnout: return STR_C("Switch");
	case track_tab::straights:
		if (false == m_straights.detour.empty() || m_straights.tool == 2)
			return STR_C("Plan, straight: S-curve");
		if (m_straights.tool == 1)
			return STR_C("Plan, straight: break");
		return STR_C("Plan, straight");
	case track_tab::route: return STR_C("Plan, curve");
	case track_tab::profile: return STR_C("Vertical profile");
	case track_tab::speed: return STR_C("Speed check");
	case track_tab::joints: return STR_C("Joints");
	case track_tab::lineside: return STR_C("Objects along the track");
	case track_tab::infra: return STR_C("Infrastructure");
	case track_tab::parallel: return STR_C("Parallel track");
	case track_tab::signals: return STR_C("Signal");
	case track_tab::turntable: return STR_C("Turntable");
	default: return STR_C("Select");
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
	{
		m_sweep.open = false;
		m_hekto.open = false;
		m_fouling.open = false;
		m_parallel.open = false;
		m_vehicle.open = false;
	}
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
	case track_tab::parallel:
		render_parallel_ui();
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
		ImGui::TextDisabled("%s", STR_C("Nothing selected"));
		return;
	}
	char const *type = track->eType == tt_Normal ? "normal" : track->eType == tt_Switch ? "switch" : track->eType == tt_Cross ? "cross" : track->eType == tt_Table ? "turntable" : track->eType == tt_Tributary ? "tributary" : "unknown";
	ImGui::TextUnformatted((track->name().empty() ? std::string{"(noname)"} : track->name()).c_str());
	ImGui::TextDisabled("%s, %.2f m", STR_C(type), track->Length());
	// the parameters of the path belong to the select tool, the other tools show the path they work with
	if (m_track_tab == track_tab::path)
		render_path_ui();
}

void editor_mode::render_track_guide()
{
	ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "%s", track_mode_name().c_str());
	ImGui::SameLine();
	ImGui::TextDisabled("(?)");
	if (ImGui::IsItemHovered())
	{
		std::string text;
		for (auto const &hint : track_key_hints(true))
			text += std::string{hint.key} + "   " + hint.action + "\n";
		ImGui::SetTooltip("%s", text.c_str());
	}
	if (false == m_track_mode_notice.empty())
	{
		ImGui::PushTextWrapPos(ImGui::GetWindowContentRegionMax().x - 8.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", m_track_mode_notice.c_str());
		ImGui::PopTextWrapPos();
	}
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
	auto const set{double_of(*track)};
	if (set.has_value())
		track = set->first;
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
	if (set.has_value())
	{
		auto const &second{set->second->m_paths.front()};
		ImGui::Text(STR_C("Double switch %s + %s: length %.3f m, the second switch starts %.3f m on"), set->first->name().c_str(), set->second->name().c_str(), glm::dot(plan_of(second.points[segment_data::point::end] - origin), axis), glm::dot(plan_of(second.points[segment_data::point::start] - origin), axis));
		ImGui::TextDisabled(STR_C("First switch: diverging %s, end offset %.3f m"), offset > 0.0 ? STR_C("right") : STR_C("left"), std::abs(offset));
	}
	else
		ImGui::Text(STR_C("Length %.3f m, diverging %s, end offset %.3f m"), length, offset > 0.0 ? STR_C("right") : STR_C("left"), std::abs(offset));
	render_track_name(*track);
	render_switch_record(*track);
	auto const drive_ui = [&](TTrack &Switch) {
		ImGui::PushID(&Switch);
		if (set.has_value())
			ImGui::TextUnformatted(Switch.name().c_str());
		if (auto const drive{find_switch_drive(Switch)})
		{
			auto const family{drive->family};
			auto const variant{drive->index};
			ImGui::TextDisabled(STR_C("Drive: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), Switch.name().c_str());
			ImGui::PushItemWidth(200.0f);
			if (ImGui::BeginCombo(STR_C("Drive##standing"), m_switch.drives[family].name.c_str()))
			{
				for (int i = 0; i < static_cast<int>(m_switch.drives.size()); ++i)
					if (ImGui::Selectable(m_switch.drives[i].name.c_str(), family == i) && i != family)
						replace_switch_drive(Switch, i, false);
				ImGui::EndCombo();
			}
			ImGui::PopItemWidth();
			item_tooltip("Puts another drive in place of the drive of this switch, on the same side");
			int side{((variant >> 1) == (variant & 1)) ? 0 : 1};
			ImGui::SameLine();
			auto const before{side};
			ImGui::RadioButton(STR_C("diverging side##standing"), &side, 0);
			ImGui::SameLine();
			ImGui::RadioButton(STR_C("other side##standing"), &side, 1);
			if (side != before)
				flip_switch_drive(Switch);
			item_tooltip("Puts the drive of the switch on the other side of the track: the same drive, its variant for the other side");
		}
		else if ((simulation::Events.FindEvent(Switch.name() + "+") != nullptr && simulation::Events.FindEvent(Switch.name() + "-") != nullptr) || (m_track_name.track == &Switch && m_track_name.uses.includes > 0))
			ImGui::TextDisabled(STR_C("Drive: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), Switch.name().c_str());
		else
		{
			ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", STR_C("No drive: the switch can't be switched during the game"));
			render_switch_drive_choice();
			if (m_switch.drive >= 0 && ImGui::Button(STR_C("Put the drive by the switch")))
				place_switch_drive(Switch);
		}
		ImGui::PopID();
	};
	drive_ui(*track);
	if (set.has_value())
		drive_ui(*set->second);
	if (false == m_switch.status.empty())
		ImGui::TextWrapped("%s", m_switch.status.c_str());
	if (angle > 1e-6)
		ImGui::Text(STR_C("Angle 1:%.2f (%.4f deg), R in the entry %.0f m"), 1.0 / std::tan(angle), glm::degrees(angle), diverging.radius);

	auto &tool{m_switch};
	if (false == tool.collected)
	{
		tool.templates = editor_track::standard_switch_templates();
		tool.collected = true;
	}
	if (tool.templates.empty())
		return;
	m_turnout_template = std::clamp(m_turnout_template, 0, static_cast<int>(tool.templates.size()) - 1);
	auto const fits = [&](editor_track::switch_template const &Template) { return false == Template.double_slip && Template.twofold() == set.has_value(); };
	if (false == fits(tool.templates[m_turnout_template]))
	{
		auto const ordinary{std::find_if(tool.templates.begin(), tool.templates.end(), fits)};
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
			if (false == fits(tool.templates[i]))
				continue;
			auto const label{template_label(tool.templates[i])};
			if (ImGui::Selectable(label.c_str(), i == m_turnout_template))
				m_turnout_template = i;
		}
		ImGui::EndCombo();
	}
	ImGui::PopItemWidth();
	auto const finish_set = [&](std::string const &Status) {
		switch_store::follow();
		records_to_history();
		straight_refresh();
		if (false == Status.empty())
			ui()->set_status(Status, true);
	};
	auto const replace = [&](editor_track::switch_template const &Template, int const Side) {
		auto const grade{length > 0.0 ? (main.points[segment_data::point::end].y - origin.y) / length : 0.0};
		auto paths{editor_track::place_switch(Template, origin, axis, Side, grade)};
		if (set.has_value())
		{
			if (paths.size() < 5)
				return;
			auto const before{double_paths(*set)};
			for (std::size_t i = 0; i < paths.size(); ++i)
				if (paths[i].rolls[0] == 0.f && paths[i].rolls[1] == 0.f)
					paths[i].rolls = before[i < 4 ? i & ~std::size_t{1} : 4].rolls;
			std::string status;
			if (false == move_double_paths(*set, paths, status))
			{
				ui()->set_status(status, true);
				return;
			}
			record_double(*set->first, *set->second, Template);
			finish_set(status);
			return;
		}
		if (paths.size() < 2)
			return;
		for (int i = 0; i < 2; ++i)
			if (paths[i].rolls[0] == 0.f && paths[i].rolls[1] == 0.f)
				paths[i].rolls = track->m_paths[i].rolls;
		std::string status;
		if (false == move_switch_paths(*track, paths, status))
		{
			ui()->set_status(status, true);
			return;
		}
		record_switch(*track, Template);
		records_to_history();
		straight_refresh();
		if (false == status.empty())
			ui()->set_status(status, true);
	};
	int const side{offset > 0.0 ? 1 : -1};
	if (ImGui::Button(STR_C("Replace the geometry with the template")))
		replace(tool.templates[m_turnout_template], side);
	item_tooltip("Keeps the start and the direction of the switch, takes the shape of the chosen template");
	ImGui::SameLine();
	if (set.has_value() && ImGui::Button(STR_C("Flip the side")))
	{
		auto const mirror = [&](glm::dvec3 const &Vector) {
			auto const across{glm::dot(plan_of(Vector), normal)};
			return glm::dvec3{Vector.x - 2.0 * across * normal.x, Vector.y, Vector.z - 2.0 * across * normal.y};
		};
		auto paths{double_paths(*set)};
		for (auto &path : paths)
			path.transform([&](glm::dvec3 const &Point) { return origin + mirror(Point - origin); }, mirror);
		std::string status;
		if (false == move_double_paths(*set, paths, status))
			ui()->set_status(status, true);
		else
		{
			if (auto entry{switch_store::named(*set->first)})
			{
				switch_store::place(*entry, *set->first);
				switch_store::classify(*entry, *set->first);
				switch_store::put(*entry, alignment_store::layer_of(*set->first));
			}
			finish_set(status);
		}
	}
	else if (false == set.has_value() && ImGui::Button(STR_C("Flip the side")))
	{
		editor_track::switch_template own;
		own.source = track;
		for (int i = 0; i < 2; ++i)
		{
			auto &local{own.local[i]};
			local = track->m_paths[i];
			auto const tolocal = [&](glm::dvec3 const &Vector) { return glm::dvec3{glm::dot(plan_of(Vector), normal) * side, Vector.y, glm::dot(plan_of(Vector), axis)}; };
			local.transform([&](glm::dvec3 const &Point) { return tolocal(Point - origin); }, tolocal);
			if (side < 0)
			{
				local.rolls = {-local.rolls[0], -local.rolls[1]};
				for (auto &piece : local.pieces)
					piece.rolls = {-piece.rolls[0], -piece.rolls[1]};
			}
		}
		replace(own, -side);
	}
	item_tooltip("Mirrors the switch: the diverging track goes to the other side");
	if (double low, high, reach{length}; switch_slides(*track) && (set.has_value() ? switch_slides(*set->second) && double_room(*set, low, high, reach, m_switch.status) : switch_room(*track, low, high, m_switch.status)))
	{
		ImGui::PushItemWidth(100.0f);
		ImGui::InputDouble("##switchslide", &tool.slide_typed, 0.0, 0.0, "%.2f m");
		ImGui::PopItemWidth();
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Slide along the straight")))
			slide_switch(*track, std::clamp(tool.slide_typed, low, high - reach));
		item_tooltip("Moves the switch along its straight by the distance typed, + towards the frog; the straight paths on both sides get shorter or longer. Dragging the switch with the mouse does the same");
		ImGui::TextDisabled(STR_C("Room on the straight: %.2f m back, %.2f m ahead"), -low, high - reach);
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

	ImGui::PushItemWidth(120.0f);
	double velocity = editor_track::velocity(*track);
	if (ImGui::InputDouble(STR_C("Velocity (km/h, -1: none)"), &velocity, 0.0, 0.0, "%.1f"))
		editor_track::velocity(*track, velocity);
	item_tooltip("Speed limit of the path for the AI drivers and the timetable, -1: no limit of its own");
	finish_edit(rebuild::none);
	char const *environments[] = {STR_C("flat"), STR_C("mountains"), STR_C("canyon"), STR_C("tunnel"), STR_C("bridge"), STR_C("bank")};
	int environment = std::clamp(static_cast<int>(track->eEnvironment), 0, 5);
	if (ImGui::Combo(STR_C("Environment"), &environment, environments, IM_ARRAYSIZE(environments)))
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
	char const *overheadmodes[] = {STR_C("Normal"), STR_C("No current, coasting"), STR_C("Pantographs lowered, speed limit")};
	if (ImGui::Combo(STR_C("Overhead line"), &overheadmode, overheadmodes, IM_ARRAYSIZE(overheadmodes)))
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
		m_route.vertex = -1;
		m_route.grip = -1;
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
	m_route.designed = curve.designed.has_value() ? 1 : 0;
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
	if (Curve.designed.has_value())
		return *Curve.designed;
	alignment::vertex vertex;
	vertex.radius = Curve.radius > 0.0 ? std::max(1.0, Curve.radius) : 1000.0;
	vertex.transition_in = std::max(0.0, Curve.transition_in);
	vertex.transition_out = std::max(0.0, Curve.transition_out);
	vertex.cant = std::max(0.0, Curve.cant);
	vertex.reverse_turn = std::abs(Curve.turn) > glm::pi<double>();
	compound_from(Curve, vertex);
	return vertex;
}


bool editor_mode::route_load(TTrack &Track)
{
	m_route.designed = 0;
	if (false == (m_route.single ? route_from_curve(Track) : route_from_line(Track)))
		return false;
	route_read_stored(Track);
	return true;
}

// the design kept in the scenery for the fragment takes the place of the one read from the paths; its chainage is carried on
void editor_mode::route_read_stored(TTrack &Track)
{
	auto &route{m_route};
	route.departed = -1;
	route.departure = 0.0;
	route.origin = 0.0;
	if (route.chain.tracks.empty())
		return;
	auto const chain{route.chain};
	bool reversed;
	auto index{alignment_store::find(chain.start, chain.end, reversed)};
	route.departed = alignment_store::departed(chain.tracks, route.departure);
	auto const loose{std::any_of(chain.tracks.begin(), chain.tracks.end(), [](TTrack const *Track) { return false == alignment_store::on_track(*Track).has_value(); })};
	bool digitized{false};
	int parts{0};
	int skipped{0};
	std::string failure;
	if (index < 0 && route.departed < 0 && loose && false == route.single)
	{
		parts = digitize_parts(chain, &failure, skipped);
		if (parts == 1 && skipped == 0)
		{
			alignment_store::settle();
			index = alignment_store::find(chain.start, chain.end, reversed);
			digitized = true;
		}
		else if (parts > 0)
		{
			alignment_store::settle();
			route_from_line(Track);
			route.departed = alignment_store::departed(route.chain.tracks, route.departure);
		}
	}
	if (index >= 0 && false == reversed && false == route.single)
	{
		route_take_stored(index);
	}
	else if (auto const km{alignment_store::chainage_at(chain.start)})
	{
		auto const ahead{chain.start + glm::dvec3{route.design.start_direction.x, 0.0, route.design.start_direction.y}};
		auto const further{alignment_store::chainage_at(ahead)};
		if (further.has_value() && *further > *km)
			route.origin = *km;
	}
	if (digitized && route.designed == 0)
		route.status += STR_C("\nThe line is digitized now, a straight");
	else if (digitized)
		route.status += format(STR_C("\nThe line is digitized now, curves kept in the scenery: %d"), route.designed);
	else if (parts > 0)
		route.status += format(STR_C("\nThe line is digitized in parts: %d, paths left out: %d"), parts, skipped) + (failure.empty() ? std::string{} : " - " + failure);
	else if (skipped > 0 && loose && skipped < static_cast<int>(chain.tracks.size()))
		route.status += format(STR_C("\nPaths left out of the digitized line: %d"), skipped) + (failure.empty() ? std::string{} : " - " + failure);
	else if (false == failure.empty())
		route.status += STR_C("\nThe line can't be digitized: ") + failure;
	else if (route.designed > 0)
		route.status += format(STR_C("\nCurves read from the design kept in the scenery: %d"), route.designed);
	else if (false == route.design.vertices.empty())
		route.status += STR_C("\nCurves recognized from the paths, the scenery keeps no design for them");
}

void editor_mode::route_take_stored(int const Index)
{
	auto &route{m_route};
	auto const &records{alignment_store::records()};
	if (Index < 0 || Index >= static_cast<int>(records.size()))
		return;
	auto const &stored{records[Index]};
	auto &design{route.design};
	design.vertices = stored.design.vertices;
	design.shape = stored.design.shape;
	design.speed = stored.design.speed;
	design.norms = stored.design.norms;
	design.transition_pieces = stored.design.transition_pieces;
	design.arc_piece_angle = stored.design.arc_piece_angle;
	route.designed = static_cast<int>(design.vertices.size());
	route.origin = stored.origin;
	route.vertex = -1;
	route.grip = -1;
	route_update();
}

void editor_mode::render_route_stored()
{
	auto &route{m_route};
	ImGui::PushItemWidth(120.0f);
	auto km{route.origin / 1000.0};
	if (ImGui::InputDouble(STR_C("Km of the start"), &km, 0.0, 0.0, "%.3f"))
		route.origin = km * 1000.0;
	ImGui::PopItemWidth();
	item_tooltip("Chainage of the start of the fragment, kept with the design in the scenery; the vertical profile of these paths counts from it");
	if (ImGui::SmallButton(STR_C("Digitize all the tracks")))
	{
		digitize_all();
		return;
	}
	item_tooltip("Every line of the scenery not digitized yet is kept in it with its straights, arcs and transition curves, the way the drawn tracks are kept; the paths stay as they are");
	auto const &records{alignment_store::records()};
	if (route.departed < 0 || route.departed >= static_cast<int>(records.size()))
		return;
	auto const &layer{scene::Layers.layer(records[route.departed].layer).name};
	ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), STR_C("The design kept in %s for this fragment departs from its paths by up to %.2f m"), layer.c_str(), route.departure);
	if (ImGui::SmallButton(STR_C("Take the kept design")))
	{
		route_take_stored(route.departed);
		route.status = STR_C("The kept design is loaded, applying it lays the paths by it again");
	}
	item_tooltip("Loads the design kept in the scenery in place of the one read from the paths; nothing changes until it's applied");
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Forget the kept design")))
	{
		alignment_store::forget(route.departed);
		alignment_store::settle();
		route.departed = -1;
		route.status = STR_C("The kept design will be removed from ") + layer + STR_C(" on save");
	}
	item_tooltip("The paths stay as they are, the scenery no longer keeps a design for them");
}

void editor_mode::route_store()
{
	auto &route{m_route};
	if (false == route.result.valid || route.chain.tracks.empty() || scene::Layers.empty())
		return;
	alignment_store::store(route.design, route.result, route.origin, alignment_store::layer_of(*route.chain.tracks.front()));
	route.departed = -1;
	route.departure = 0.0;
}

bool editor_mode::digitize_drawn(std::vector<TTrack *> const &Created, bool const Alone)
{
	std::vector<TTrack *> drawn;
	for (auto *track : Created)
		if (track != nullptr && track->eType == tt_Normal && false == track->m_editorremoved && false == track->m_paths.empty())
			drawn.push_back(track);
	if (drawn.empty() || scene::Layers.empty())
		return false;
	editor_track::chain chain;
	std::string error;
	if (editor_track::find_run(*drawn.front(), kDrawnReach, chain, error) && digitize_chain(chain))
		return true;
	if (Alone && editor_track::find_chain(drawn.front(), drawn.back(), chain, error) && chain.tracks.size() == drawn.size() && digitize_chain(chain))
		return true;
	WriteLog("Editor: the drawn paths don't make a line of straights, arcs and transition curves, no design kept for them", logtype::generic);
	return false;
}

void editor_mode::digitize_all()
{
	std::set<TTrack const *> visited;
	int kept{0};
	int failed{0};
	std::string first;
	for (auto *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->eType != tt_Normal || track->m_paths.empty() || false == editor_track::is_supported(*track) || visited.count(track) > 0)
			continue;
		if (alignment_store::on_track(*track).has_value())
			continue;
		editor_track::chain chain;
		std::string error;
		if (false == editor_track::find_run(*track, kDrawnReach, chain, error) || chain.tracks.empty())
		{
			visited.insert(track);
			continue;
		}
		visited.insert(chain.tracks.begin(), chain.tracks.end());
		std::string reason;
		kept += digitize_parts(chain, &reason, failed);
		if (first.empty())
			first = reason;
	}
	alignment_store::settle();
	auto text{format(STR_C("Lines digitized: %d, paths left out: %d"), kept, failed)};
	if (false == first.empty())
		text += STR_C(", the first: ") + first;
	WriteLog("Editor: " + text, logtype::generic);
	ui()->set_status(text, failed > 0);
	if (false == m_route.chain.tracks.empty())
		route_load(*m_route.chain.tracks[m_route.chain.tracks.size() / 2]);
}

int editor_mode::digitize_parts(editor_track::chain const &Chain, std::string *Reason, int &Skipped)
{
	auto const count{Chain.tracks.size()};
	if (count == 0 || std::all_of(Chain.tracks.begin(), Chain.tracks.end(), [](TTrack const *Track) { return alignment_store::on_track(*Track).has_value(); }))
		return 0;
	TTrack *culprit{nullptr};
	std::string reason;
	if (digitize_chain(Chain, &reason, &culprit))
		return 1;
	if (count == 1)
	{
		++Skipped;
		reason = (Chain.tracks.front()->name().empty() ? std::string{"(noname)"} : Chain.tracks.front()->name()) + ": " + reason;
		WriteLog("Editor: digitizing - " + reason, logtype::generic);
		if (Reason != nullptr && Reason->empty())
			*Reason = reason;
		return 0;
	}
	auto const found{static_cast<std::size_t>(std::find(Chain.tracks.begin(), Chain.tracks.end(), culprit) - Chain.tracks.begin())};
	auto const split{found < count ? std::max<std::size_t>(1, found) : count / 2};
	int parts{0};
	for (auto const &[first, last] : {std::pair<std::size_t, std::size_t>{0, split - 1}, std::pair<std::size_t, std::size_t>{split, count - 1}})
	{
		editor_track::chain part;
		std::string error;
		if (editor_track::find_chain(Chain.tracks[first], Chain.tracks[last], part, error) && part.tracks.size() == last - first + 1)
			parts += digitize_parts(part, Reason, Skipped);
		else
			Skipped += static_cast<int>(last - first + 1);
	}
	return parts;
}

bool editor_mode::digitize_chain(editor_track::chain const &Chain, std::string *Reason, TTrack **Culprit)
{
	if (Chain.tracks.empty())
		return false;
	auto const name = [](TTrack const *Track) { return Track == nullptr || Track->name().empty() ? std::string{"(noname)"} : Track->name(); };
	auto const nearest = [&](glm::dvec2 const &Point) {
		TTrack *found{nullptr};
		double distance{std::numeric_limits<double>::max()};
		for (auto *track : Chain.tracks)
			for (auto const end : {segment_data::point::start, segment_data::point::end})
				if (auto const d{glm::distance(plan_of(track->m_paths.front().points[end]), Point)}; d < distance)
				{
					distance = d;
					found = track;
				}
		return found;
	};
	alignment::design design;
	design.norms = m_route.design.norms;
	design.transition_pieces = m_route.design.transition_pieces;
	design.arc_piece_angle = m_route.design.arc_piece_angle;
	design.speed = Chain.velocity > 0.0 ? Chain.velocity : 100.0;
	design.start = Chain.start;
	design.end = Chain.end;
	if (glm::length(plan_of(Chain.start_direction)) < 1e-9 || glm::length(plan_of(Chain.end_direction)) < 1e-9)
		return false;
	design.start_direction = glm::normalize(plan_of(Chain.start_direction));
	design.end_direction = glm::normalize(plan_of(Chain.end_direction));
	design.start_grade = grade_of(Chain.start_direction);
	design.end_grade = grade_of(Chain.end_direction);
	design.start_radius = Chain.start_radius;
	design.end_radius = Chain.end_radius;
	auto const preferred{m_route.design.shape};
	auto const other{preferred == alignment::transition_shape::clothoid ? alignment::transition_shape::cubic_parabola : alignment::transition_shape::clothoid};
	alignment::result result;
	alignment::design best;
	alignment::result bestresult;
	double closest{std::numeric_limits<double>::max()};
	TTrack *farthest{nullptr};
	bool fits{false};
	for (auto const attempt : {0, 1, 2, 3})
	{
		auto const shape{attempt % 2 == 0 ? preferred : other};
		design.shape = shape;
		if (attempt == 2)
		{
			design.norms.arc_min = 0.0;
			design.norms.arc_min_factor = 0.0;
		}
		design.vertices.clear();
		for (auto const &entry : editor_track::recognize_line(Chain, m_straights.tolerance, design.norms.gauge, shape))
		{
			if (entry.kink)
			{
				if (Reason != nullptr)
					*Reason = format(STR_C("the straights meet with no curve at path %s"), name(nearest(entry.position)).c_str());
				if (Culprit != nullptr)
					*Culprit = nearest(entry.position);
				return false;
			}
			auto vertex{route_vertex_of(entry.shape)};
			vertex.position = entry.position;
			design.vertices.push_back(vertex);
		}
		if (design.vertices.size() >= 2)
		{
			design.vertices.front().offset = std::max(0.0, glm::dot(design.vertices.front().position - plan_of(design.start), design.start_direction));
			design.vertices.back().offset = std::max(0.0, glm::dot(plan_of(design.end) - design.vertices.back().position, design.end_direction));
		}
		result = alignment::compute(design);
		if (alignment_store::follows(result, Chain.tracks))
		{
			fits = true;
			break;
		}
		TTrack *worst{nullptr};
		if (auto const away{alignment_store::deviation(result, Chain.tracks, worst)}; away < closest)
		{
			closest = away;
			farthest = worst;
			best = design;
			bestresult = result;
		}
	}
	double fit{0.0};
	if (false == fits && closest <= kDigitizeFit)
	{
		design = best;
		result = bestresult;
		fit = closest;
		fits = true;
	}
	if (false == fits)
	{
		if (Culprit != nullptr)
			*Culprit = farthest;
		if (Reason != nullptr)
			*Reason = closest < std::numeric_limits<double>::max() ? format(STR_C("the curves read from the paths leave path %s by %.2f m"), name(farthest).c_str(), closest) : std::string{STR_C("the paths don't make a line of straights, arcs and transition curves")};
		return false;
	}
	auto const &records{alignment_store::records()};
	std::set<int> covered;
	for (auto *track : Chain.tracks)
		if (auto const found{alignment_store::on_track(*track)})
			covered.insert(found->record);
	double origin{0.0};
	for (auto const index : covered)
	{
		auto const &old{records[index]};
		auto const start{alignment_store::chainage_on(result, old.design.start)};
		auto const end{alignment_store::chainage_on(result, old.design.end)};
		if (false == start.has_value() || false == end.has_value())
			return false;
		if (index == *covered.begin() && *end > *start)
			origin = old.origin - *start;
	}
	auto const layer{covered.empty() ? alignment_store::layer_of(*Chain.tracks.front()) : records[*covered.begin()].layer};
	for (auto index{covered.rbegin()}; index != covered.rend(); ++index)
		alignment_store::forget(*index);
	alignment_store::store(design, result, origin, layer, "drawn", fit);
	return true;
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
	auto const found{editor_track::recognize_line(route.chain, m_straights.tolerance, design.norms.gauge, design.shape)};
	route.designed = 0;
	if (false == found.empty())
	{
		design.vertices.clear();
		for (auto const &entry : found)
		{
			alignment::vertex vertex;
			if (entry.kink)
			{
				vertex.radius = std::max(10.0, std::round(alignment::recommend(design.speed, 1000.0, design.norms).radius_min / 10.0) * 10.0);
				vertex.transition_in = 0.0;
				vertex.transition_out = 0.0;
				vertex.cant = 0.0;
			}
			else
			{
				vertex = route_vertex_of(entry.shape);
				if (entry.shape.designed.has_value())
					++route.designed;
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
		relaid = editor_track::relay(chain, pieces, &to);
	}
	else if (false == route_relay_through_switches(chain, states, relaid, to))
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
	route_store();
	records_to_history();
	route.status = STR_C("Re-laid ") + std::to_string(chain.tracks.size()) + STR_C(" paths") + (added > 0 ? STR_C(", added ") + std::to_string(added) : std::string{});
	if (startextension > 0.0 || endextension > 0.0)
		route.status += format(", the curve took over %.2f / %.2f m of the adjoining straights", startextension, endextension);
	WriteLog("Editor: route design - " + route.status, logtype::generic);
}

// the switches of the chain are moved as a whole onto the designed line, the regular paths between them re-laid
bool editor_mode::route_relay_through_switches(editor_track::chain const &Chain, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, TTrack *&Last)
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
		TTrack *last{nullptr};
		auto const added{editor_track::relay(parts[i], group, &last)};
		if (last != nullptr && parts[i].tracks.back() == Chain.tracks.back())
			Last = last;
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
	if (false == grip && (route.vertex < 0 || route.vertex >= count))
	{
		m_route_dragging = false;
		return;
	}

	auto const position{grip ? alignment::evaluate(route.result, route.result.vertex_chainages[route.grip]).position : route_vertex_position(route.vertex)};
	glm::dvec3 moved{position};
	if (false == handle_drag_step(moved))
	{
		if (m_route_dragging)
		{
			m_route_dragging = false;
			route_apply();
		}
		return;
	}
	m_route_dragging = true;
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
	render_norms_choice();
	ImGui::Separator();
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
	render_route_stored();
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
	char const *shapes[] = {STR_C("Cubic parabola"), STR_C("Clothoid")};
	if (ImGui::Combo(STR_C("Transition curve"), &shape, shapes, IM_ARRAYSIZE(shapes)))
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
	if (false == m_norms_free)
	{
		if (ImGui::SmallButton(STR_C("Apply recommended for V")))
		{
			route_recommend(vertex);
			changed = true;
		}
		item_tooltip("Cant and transitions recommended for the design speed and the radius");
		auto const recommended{alignment::recommend(design.speed, vertex.radius, design.norms)};
		ImGui::TextDisabled(STR_C("For %.0f km/h: R min %.0f m, cant %.0f mm (equilibrium %.0f, min %.0f), transition %.0f m"), design.speed, recommended.radius_min, recommended.cant, recommended.cant_equilibrium, recommended.cant_min, recommended.transition);
	}
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
		char const *kinds[] = {"straight", "transition", "curve", "transition", "spiral"};
		for (auto const &e : result.elements)
			ImGui::Text("km %s  %-10s  L %8.2f m%s", profile::format_chainage(m_route.origin + e.chainage).c_str(), kinds[static_cast<int>(e.kind)], e.length, e.kind == alignment::element_kind::straight ? "" : ("  R " + std::to_string(static_cast<int>(std::round(e.radius))) + " m").c_str());
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
	auto line{m_straights.dragging ? m_straights.drag_line : m_straights.current};
	if (line.tracks.empty())
		return;
	glm::dvec3 const offset{m_straights.dragging ? m_straights.preview_start - m_straights.drag_line.start : glm::dvec3{}};
	if (m_straights.dragging)
	{
		line.start = m_straights.preview_start;
		line.end = m_straights.preview_end;
		auto const planar{plan_of(line.end - line.start)};
		auto const length{glm::length(planar)};
		if (length > 1e-6)
		{
			line.direction = planar / length;
			line.length = length;
		}
	}
	auto const offer{straight_parallel_offer(m_straights.dragging ? m_straights.drag_line : m_straights.current, offset)};
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
		bool const highlighted{offer.nearby && offer.neighbour == &other};
		auto const tick{highlighted ? (offer.snaps ? IM_COL32(80, 230, 255, 255) : IM_COL32(255, 210, 60, 255)) : IM_COL32(255, 230, 120, 230)};
		if (highlighted)
			projection.line(drawlist, other.start, other.end, tick, 3.0f);
		projection.line(drawlist, foot, target, tick, highlighted ? 3.0f : 1.5f);
		ImVec2 screen;
		if (projection.project((foot + target) * 0.5, screen))
		{
			auto const angle{glm::degrees(std::abs(std::asin(std::clamp(line.direction.x * other.direction.y - line.direction.y * other.direction.x, -1.0, 1.0))))};
			auto const text{highlighted ? (offer.snaps ? format(STR_C("parallel  %.2f m"), offer.spacing) : format("%.2f → %.2f m  %.1f°", std::abs(lateral), offer.spacing, offer.angle)) :
			                             (angle > 0.05 ? format("%.2f m  %.1f°", std::abs(lateral), angle) : format("%.2f m", std::abs(lateral)))};
			auto const *label{text.c_str()};
			auto const size{ImGui::CalcTextSize(label)};
			drawlist->AddRectFilled(ImVec2(screen.x - 4.0f, screen.y - 2.0f), ImVec2(screen.x + size.x + 4.0f, screen.y + size.y + 2.0f), IM_COL32(0, 0, 0, 190), 3.0f);
			drawlist->AddText(screen, highlighted ? tick : IM_COL32(255, 230, 120, 255), label);
		}
	}
	if (offer.nearby && false == offer.snaps && std::abs(offer.correction) > 0.02)
	{
		auto const shift{glm::dvec3{normal.x, 0.0, normal.y} * offer.correction};
		projection.line(drawlist, line.start + shift, line.end + shift, IM_COL32(80, 230, 255, 210), 3.5f);
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
				if (other.tracks.empty() || std::abs(line.direction.x * other.direction.y - line.direction.y * other.direction.x) > std::sin(glm::radians(kParallelFindAngle)))
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

editor_mode::parallel_offer editor_mode::straight_parallel_offer(editor_track::straight const &Line, glm::dvec3 const &Offset) const
{
	parallel_offer offer;
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	auto const middle{(Line.start + Line.end) * 0.5 + Offset};
	double best{kParallelHintWindow};
	for (auto const &other : m_straights.neighbours)
	{
		auto const sine{std::abs(Line.direction.x * other.direction.y - Line.direction.y * other.direction.x)};
		if (sine > std::sin(glm::radians(kParallelFindAngle)))
			continue;
		auto const distance{glm::dot(plan_of(other.start - middle), normal)};
		auto const angle{glm::degrees(std::abs(std::asin(std::clamp(sine, 0.0, 1.0))))};
		for (auto const spacing : m_straights.spacings)
		{
			auto const error{std::abs(std::abs(distance) - spacing)};
			if (error < best)
			{
				best = error;
				offer.nearby = true;
				offer.snaps = error < kParallelSnapWindow && angle <= kParallelSnapAngle;
				offer.spacing = spacing;
				offer.distance = std::abs(distance);
				offer.angle = angle;
				offer.correction = distance > 0.0 ? std::abs(distance) - spacing : spacing - std::abs(distance);
				offer.neighbour = &other;
			}
		}
	}
	return offer;
}

glm::dvec3 editor_mode::snap_straight_offset(editor_track::straight const &Line, glm::dvec3 const &Offset) const
{
	auto const offer{straight_parallel_offer(Line, Offset)};
	if (false == offer.snaps)
		return Offset;
	glm::dvec2 const normal{-Line.direction.y, Line.direction.x};
	return Offset + glm::dvec3{normal.x, 0.0, normal.y} * offer.correction;
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
		if (std::abs(sine) > std::sin(glm::radians(kParallelSnapAngle)))
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
	if (false == m_straights.neighbours.empty())
		ImGui::TextDisabled("%s", STR_C("Nearby straight: the diamond snaps parallel at 4.00 / 4.50 / 4.75 m"));

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
	toolbutton(STR_C("S-curve"), 2, STR_C("Shifts a piece of the straight sideways around an obstacle and back: four clicks on the straight (Shift+click in the view does the same)"));
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
	auto const &line{state.dragging ? state.drag_line : current_straight()};
	if (state.handle < 0 || line.tracks.empty())
	{
		state.dragging = false;
		return;
	}

	glm::dvec3 const anchors[] = {line.start, line.end, (line.start + line.end) * 0.5};
	glm::dvec3 handled{anchors[state.handle]};
	bool const held{handle_drag_step(handled)};
	if (false == held && false == state.dragging)
		return;

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
		double from;
		double to;
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
	int first{-1};
	int last{-1};
	auto const select = [&](double const End) {
		first = -1;
		last = -1;
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
				first = -1;
				last = -1;
				return;
			}
			middle = best;
		}
		int low{middle};
		int high{middle};
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
	m_route.design.start_reserve = 0.0;
	m_route.design.end_reserve = 0.0;
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
	auto created{editor_track::relay(m_route.chain, pieces, &m_route.to)};
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
	{
		auto changed{created};
		for (auto const &entry : states)
			changed.push_back(entry.first);
		digitize_drawn(changed);
	}
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
	state.handle = -1;
	return true;
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
	state.tool_before = state.tool;
	state.tool = Tool;
	if (false == place_straight_tool())
	{
		state.tool = state.tool_before;
		return;
	}
	state.tool_mouse = true;
}

void editor_mode::finish_straight_gesture()
{
	auto &state{m_straights};
	state.tool_mouse = false;
	apply_straight_tool();
	state.tool = state.tool_before;
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
	if (auto const set{double_of(*track)})
	{
		for (auto *part : {set->first, set->link, set->second})
			if (false == part->Dynamics.empty())
			{
				ui()->set_status(STR_C("The track can't be deleted while vehicles stand on it"), true);
				return;
			}
		if (auto const known{switch_store::named(*set->first)})
			switch_store::forget(known->name);
		auto main{set->first->m_paths.front()};
		main.points[segment_data::point::end] = set->second->m_paths.front().points[segment_data::point::end];
		auto const velocity{speed_check::path_velocity(*set->first, 0)};
		for (auto *part : {set->first, set->link, set->second})
			editor_track::retire(*part);
		std::vector<TTrack *> created;
		if (false == Global.shiftState)
		{
			auto *straight{editor_track::create_path(*set->first, main)};
			editor_track::velocity(*straight, velocity);
			editor_track::commit({straight});
			created.push_back(straight);
			ui()->set_status(STR_C("The double switch is replaced by its main track, Shift+Del removes all of it"), false);
		}
		push_track_snapshot({}, created, {set->first, set->link, set->second});
		if (std::any_of(m_route.chain.tracks.begin(), m_route.chain.tracks.end(), [&](TTrack const *Each) { return Each == set->first || Each == set->link || Each == set->second; }))
			m_route = {};
		m_node = nullptr;
		ui()->set_node(nullptr);
		m_track_point = {};
		straight_refresh();
		return;
	}
	// retired first, so the infrastructure it carried and moves elsewhere is in the same undo step
	if (auto const known{switch_store::of(*track)})
		switch_store::forget(known->name);
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
			hints = {{"LMB", "start, on the ground or at a free end"}, {"Drag from a free end", "lays the shape to the cursor"}, {"Shift+click a straight", "a parallel track (N)"}, {"Alt", "the direction in steps of 15 degrees"}, {"Esc", "stop laying"}};
		else
		{
			bool const chained{m_lay.start.track != nullptr};
			hints = {{"LMB", chained ? "lay the shape up to here" : "lay the straight up to here"}, {"LMB on a free end", "join and lay"}};
			if (chained || m_lay.mouse_snap.track != nullptr)
				hints.insert(hints.end(), {{"Ctrl+wheel", "radius"}, {"Shift+wheel", "another shape"}});
			hints.insert(hints.end(), {{"Alt", "the direction in steps of 15 degrees"}, {"Enter", "lay up to the cursor"}, {"Backspace", m_lay.chain.empty() ? "take back the point" : "take back the last piece"}, {"Esc", "cancel"}});
		}
	}
	else if (false == m_straights.detour.empty())
	{
		char const *const steps[] = {"leave the axis", "shifted", "start back", "back on the axis"};
		auto const step{std::min<std::size_t>(m_straights.detour.size(), 3)};
		hints = {{m_straights.tool == 2 ? "LMB" : "Shift+click", format("point %zu of 4: %s", step + 1, steps[step])}, {"Esc", "cancel"}};
	}
	else if (m_extend.active)
		hints = {{"Drag", "lays the shape to the cursor, to a free end: joins it"}, {"Ctrl+wheel", "radius"}, {"Shift+wheel", "another shape"}, {"Release", "build"}, {"Esc", "cancel"}};
	else if (m_switch.placing)
		hints = m_switch.connection >= 0 && false == m_switch.curved ?
		            std::vector<key_hint>{{"Drag onto the other track", "the connection there"}, {"Shift+wheel", "another kind"}, {"Ctrl+wheel", "R of the aligning curve"}, {"Alt+wheel", "where a double crossover meets"}, {"Esc", "cancel"}} :
		            std::vector<key_hint>{{"Drag along", "direction of the switch"}, {"Drag sideways", "side of the diverging track"}, {"Esc", "cancel"}};
	else if (m_track_tab == track_tab::lay)
		hints = {{"LMB", "start, on the ground or at a free end"}};
	else if (m_track_tab == track_tab::turnout && (selected_track() == nullptr || selected_track()->eType == tt_Switch))
	{
		hints = {{"LMB on a track", "the straight or the curve for the switch"}};
		if (selected != nullptr)
		{
			hints.push_back({"Gizmo", "move the selected switch"});
			hints.push_back({"Del", "remove the switch, its main track stays"});
			gizmo = true;
		}
	}
	else if (m_track_tab == track_tab::turnout && m_switch.armed < 0)
		hints = {{"List in the panel", "the type of the switch"}, {"LMB on a track", "another track"}};
	else if (m_track_tab == track_tab::turnout)
		hints = {{"Press+drag on the track", "along: direction, sideways: side"}, {"List in the panel", "the type of the switch"}, {"LMB on another track", "choose it"}};
	else if (m_track_tab == track_tab::profile && m_profile.route.spans.empty())
		hints = {{"LMB on a track", "loads the grade line of its line"}};
	else if (m_track_tab == track_tab::profile)
		hints = {{"Drag a point", "changes the grade"}, {"Double click", "adds a point"}, {"Apply", "writes the heights into the tracks"}};
	else if (m_track_window_open && m_track_tab == track_tab::speed)
		hints = {{"LMB on a row", "show the path"}};
	else if (m_track_window_open && m_track_tab == track_tab::lineside)
		hints = {{"LMB on a track", "the line the objects go along"}};
	else if (m_track_window_open && m_track_tab == track_tab::parallel)
		hints = {{"LMB on a straight", "the new track goes beside it"}};
	else if (m_track_window_open && m_track_tab == track_tab::signals && (m_signal.placing || m_signal.moving))
		hints = {{"Drag along", m_signal.moving ? "moves the signal along the track" : "the trains it faces run that way"}, {"Release", "done"}, {"Esc", "cancel"}};
	else if (m_track_window_open && m_track_tab == track_tab::signals && signal_armed() == nullptr)
		hints = {{"List in the panel", "the type of the signal"}, {"Drag a signal", "moves it along the track"}, {"LMB on a signal", "selects it"}};
	else if (m_track_window_open && m_track_tab == track_tab::signals)
	{
		hints = {{"LMB by a track", "places the signal on that side"}, {"Press+drag along", "the trains it faces run that way"}, {"Drag a signal", "moves it along the track"}, {"LMB on a signal", "selects it"}};
		if (m_signal.selected != 0)
			hints.push_back({"Del", "removes the selected signal with its event"});
	}
	else if (m_track_window_open && m_track_tab == track_tab::turntable && m_turntable.placing)
		hints = {{"Drag", "turns the bridge"}, {"Release", "places the turntable"}, {"Esc", "cancel"}};
	else if (m_track_window_open && m_track_tab == track_tab::turntable && m_turntable.table != nullptr)
		hints = {{"LMB around it", "leads a track out"}, {"LMB on a free end", "fits that track to the turntable"}};
	else if (m_track_window_open && m_track_tab == track_tab::turntable)
		hints = {{"LMB", "places a new turntable"}};
	else if (m_track_window_open && m_track_tab == track_tab::joints)
		hints = {{"LMB on a row", "show the place"}, {"Fix", "joins the ends, aligns them"}};
	else if (m_track_window_open && m_track_tab == track_tab::infra)
		hints = {{"LMB", "select a path"}};
	else if (m_track_tab == track_tab::straights && m_straights.tool == 1)
		hints = {{"Press+drag on the straight", "breaks it there and turns the rest"}, {"Esc", "cancel"}};
	else if (m_track_tab == track_tab::straights && m_straights.tool == 2)
		hints = {{"LMB on the straight", "point 1 of 4: leave the axis"}, {"Esc", "cancel"}};
	else if (m_track_tab == track_tab::straights && m_straights.dragging && m_straights.handle == 2)
	{
		auto const offer{straight_parallel_offer(m_straights.drag_line, m_straights.preview_start - m_straights.drag_line.start)};
		if (offer.snaps)
			hints = {{"Release", format(STR_C("parallel at %.2f m"), offer.spacing)}, {"Esc", "cancel"}};
		else if (offer.nearby)
			hints = {{"Near another straight", format(STR_C("snap at %.2f m  (%.1f°)"), offer.spacing, offer.angle)}, {"Esc", "cancel"}};
		else
			hints = {{"Close to another straight", "snaps parallel at 4.00 / 4.50 / 4.75 m"}, {"Esc", "cancel"}};
	}
	else if (m_track_tab == track_tab::straights && m_straights.current.tracks.empty())
		hints = {{"LMB on a straight", "its ends and the middle to drag"}, {"LMB on a curve", "its vertex and radius to drag"}};
	else if (m_track_tab == track_tab::straights)
	{
		hints = {{"Drag an end", "the curves at it follow"}, {"Drag the middle", "shifts sideways, snaps parallel"}, {"Ctrl+drag", "breaks the straight"}, {"Shift+click x4", "shifts a piece around an obstacle"}, {"Alt+click", "adds another straight to move together"}};
	}
	else if (m_track_tab == track_tab::route && m_route.chain.tracks.empty())
		hints = {{"LMB on a straight", "its ends and the middle to drag"}, {"LMB on a curve", "its vertex and radius to drag"}};
	else if (m_track_tab == track_tab::route)
		hints = {{"Drag a vertex", "the curve follows"}, {"Drag the radius grip", "changes the radius"}, {"Apply", "writes the design into the tracks"}, {"Esc", "release the vertex"}};
	else if (selected == nullptr)
		hints = {{"LMB on a track", "selects it"}, {"Shift+LMB", "adds a track to the selection, drag: a box"}, {"RMB on a track", "what can be done with it"}, {"L", "lay a new track"}};
	else if (m_point_drag.moved)
		hints = {{"Drag", "moves the point, level"}, {"Over a free end", "the end joins it"}, {"Release", "done"}, {"Esc", "done"}};
	else if (m_track_point.valid())
	{
		char const *action{"moves the point along the axes"};
		if (false == editor_track::is_end(m_track_point.kind))
			action = m_gizmo_op == gizmo_operation::rotate ? "turns the control vector sideways or up, cant at its end, the joined track keeps the tangent" : m_gizmo_op == gizmo_operation::scale ? "length of the control vector" : "moves the control point, the joined track keeps the tangent";
		else if (m_gizmo_op == gizmo_operation::rotate)
			action = "rings about the point: direction, grade, cant; the joined tracks follow";
		else if (m_gizmo_op == gizmo_operation::scale && selected != nullptr && track_gizmo_operation(*selected, true) == gizmo_operation::scale)
			action = "lengthens the track at this end, the curve keeps its radius";
		hints = {{"Drag a point", "moves it, an end dropped on a free end joins it"}, {"Gizmo", action}, {"Q W E", "gizmo: move, turn, lengthen"}, {"Esc", "releases the point"}};
		gizmo = true;
	}
	else
	{
		char const *action{"moves the whole track"};
		if (m_gizmo_op == gizmo_operation::rotate)
			action = "rings: direction, grade, cant; the joined tracks follow";
		else if (m_gizmo_op == gizmo_operation::scale && selected != nullptr && track_gizmo_operation(*selected, false) == gizmo_operation::scale)
			action = "lengthens the track at its free end, the curve keeps its radius";
		hints = {{"Drag a point", "moves it, an end dropped on a free end joins it"}, {"Gizmo", action}, {"Q W E", "gizmo: move, turn, lengthen"}, {"Shift+LMB", "adds a track to the selection, drag: a box"}, {"L", "new track, also from a free end"}, {"Esc", "deselects"}, {"Del", "deletes the track"}};
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
		hints.push_back({"Esc", "back to Select"});
		if (Global.EditorOrtho)
			hints.push_back({"Hold the wheel", "pans the view"});
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
		double length{0.0};
		double radius{0.0};
		auto const pieces{extend_pieces()};
		for (auto const &piece : pieces)
		{
			length += bezier{piece}.plan_length();
			if (piece.radius != 0.0f && (radius == 0.0 || std::abs(piece.radius) < radius))
				radius = std::abs(piece.radius);
		}
		editor_track::snap_target target;
		auto const snapped{extend_snap(target) && target.track != nullptr};
		editor_track::branch_link link;
		std::string reason;
		if (plan_branch_link(link, reason))
			return link_summary(target, link, editor_track::branch_radius(m_extend.track->m_paths[1]));
		if (snapped && pieces.empty())
			return "no track fits to the end of " + name(target.track);
		auto text{radius > 0.0 ? format("L %.2f m   R %.0f m", length, radius) : format("L %.2f m   straight", length)};
		if (snapped)
			text += "\njoins " + name(target.track);
		if (false == reason.empty())
			text += "\n" + reason;
		return text + "\n" + track_shaping(snapped);
	}
	if (m_lay.active)
	{
		auto const &lay{m_lay};
		std::string text;
		if (lay.points.empty())
			text = lay.mouse_snap.track != nullptr ? "starts at the end of " + name(lay.mouse_snap.track) : "start";
		else if (false == lay.preview.empty())
		{
			double radius{0.0};
			bool straight{false};
			for (auto const &piece : lay.preview)
			{
				auto const r{piece_plan_radius(piece)};
				if (r < 0.5)
					straight = true;
				else if (radius == 0.0 || r < radius)
					radius = r;
			}
			if (radius > 0.0)
				text = straight ? format("L %.1f m   %s + R %.0f m", lay.preview_length, STR_C("straight"), radius) : format("L %.1f m   R %.0f m", lay.preview_length, radius);
			else
				text = format("L %.1f m   %s", lay.preview_length, STR_C("straight"));
		}
		if (false == lay.points.empty() && lay.mouse_snap.track != nullptr)
			text += "\njoins " + name(lay.mouse_snap.track);
		if (false == lay.preview_error.empty())
			text += "\n" + lay.preview_error;
		if (lay.points.size() == 1 && (lay.start.track != nullptr || lay.mouse_snap.track != nullptr))
			text += "\n" + track_shaping(lay.mouse_snap.track != nullptr && lay.start.track != nullptr);
		if (ImGui::GetIO().KeyAlt && false == lay.points.empty())
			text += format("\naz %.0f deg", azimuth(lay.points.back(), lay.mouse));
		if (false == lay.status.empty() && ImGui::GetTime() - lay.status_time < 4.0)
			text += "\n" + lay.status;
		return text;
	}
	auto const &state{m_straights};
	if (state.tool_placed && state.tool_mouse)
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
			auto const offset{state.preview_start - grabbed.start};
			auto text{format("shift %+.3f m", glm::dot(plan_of(offset), normal))};
			auto const offer{straight_parallel_offer(grabbed, offset)};
			if (offer.snaps)
				text += "\n" + format(STR_C("parallel  %.2f m"), offer.spacing);
			else if (offer.nearby)
				text += "\n" + format(STR_C("close: %.2f m  %.1f°"), offer.spacing, offer.angle);
			return text;
		}
		return format("L %.3f m (%+.3f)   azimuth %.4f deg", plan_distance(state.preview_start, state.preview_end), plan_distance(state.preview_start, state.preview_end) - grabbed.length,
		              azimuth(state.preview_start, state.preview_end));
	}
	if (m_track_set_using && m_track_gizmo_drag.operation == gizmo_operation::rotate)
		return format(STR_C("turned %+.2f deg"), glm::degrees(m_track_gizmo_drag.angle));
	auto const *track{selected_track()};
	if (m_track_gizmo_using && track != nullptr)
	{
		auto const &drag{m_track_gizmo_drag};
		if (drag.control)
		{
			auto const control{editor_track::point_position(*track, m_track_point) - editor_track::point_position(*track, editor_track::end_of(m_track_point))};
			auto text{format(STR_C("control %.2f m   azimuth %.2f deg"), glm::length(control), azimuth(glm::dvec3{0.0}, control))};
			if (drag.operation == gizmo_operation::rotate && drag.ring == 1)
				text += format(STR_C("   turned %+.2f deg"), glm::degrees(drag.angle));
			if (drag.operation == gizmo_operation::rotate && drag.ring == 0)
				text += format(STR_C("   grade %+.1f per mille"), 1000.0 * grade_of(control) * (glm::dot(level(control), drag.basis[2]) >= 0.0 ? 1.0 : -1.0));
			if (drag.operation == gizmo_operation::rotate && drag.ring == 2)
				text += "   " + format(STR_C("cant %.0f mm (%+.0f mm)"), 1500.0 * std::sin(glm::radians(drag.roll0 + drag.roll)), 1500.0 * (std::sin(glm::radians(drag.roll0 + drag.roll)) - std::sin(glm::radians(drag.roll0))));
			if (drag.fixed != glm::dvec3{})
				text += "\n" + STR("along the switch");
			return text;
		}
		if (drag.operation == gizmo_operation::scale && drag.path >= 0)
		{
			auto text{format("L %.3f m (%+.3f)", drag.stretch, drag.stretch - drag.length)};
			if (drag.path < static_cast<int>(drag.paths.size()) && std::abs(drag.paths[drag.path].radius) > 0.5f)
				text += format("   R %.1f m", std::abs(drag.paths[drag.path].radius));
			return text;
		}
		if (drag.operation == gizmo_operation::rotate && drag.ring == 0)
			return format(STR_C("grade changed by %+.1f per mille"), 1000.0 * drag.grade);
		if (drag.operation == gizmo_operation::rotate && drag.ring == 2)
			return format(STR_C("cant %.0f mm (%+.0f mm)"), 1500.0 * std::sin(glm::radians(drag.roll0 + drag.roll)), 1500.0 * (std::sin(glm::radians(drag.roll0 + drag.roll)) - std::sin(glm::radians(drag.roll0))));
		if (drag.operation == gizmo_operation::rotate)
			return format(STR_C("turned %+.2f deg"), glm::degrees(drag.angle));
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

std::string editor_mode::track_shaping(bool const Joined) const
{
	if (false == Joined)
	{
		char const *const names[]{"Arc, then straight", "Straight, then arc", "Arc through the cursor", "Straight"};
		auto text{std::string{STR_C(names[static_cast<int>(m_track_layout)])}};
		if (m_track_layout == track_layout::arc_straight || m_track_layout == track_layout::straight_arc)
			text += format("   R %.0f m", m_lay.radius);
		return text + "\n" + STR("Ctrl+wheel: radius, Shift+wheel: arc / straight");
	}
	auto const &join{m_join_last};
	char const *const kinds[]{"", "straight", "arc", "curve", "reverse curve", "free curve"};
	std::string text{STR_C(kinds[static_cast<int>(join.kind)])};
	if (join.radius > 0.0)
		text += format("   R %.0f m", join.radius) + (m_join_radius > 0.0 ? std::string{} : std::string{" "} + STR("(auto)"));
	if (join.cant > 0.0)
		text += format(STR_C("   cant %.0f mm"), join.cant);
	if (join.transition > 0.0)
		text += format(STR_C("   transitions %.0f m"), join.transition);
	if (join.speed > 0.0 && (join.cant > 0.0 || join.transition > 0.0))
		text += format("   V %.0f km/h", join.speed);
	if (join.variants > 1)
		text += format("   %d/%d", (m_join_variant % join.variants + join.variants) % join.variants + 1, join.variants);
	return text + "\n" + STR("Ctrl+wheel: radius, Shift+wheel: another shape");
}

void editor_mode::draw_track_hints()
{
	ImGuiIO const &io = ImGui::GetIO();
	auto *drawlist{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	float const margin{12.0f};
	float const gap{14.0f};
	float const pad{4.0f};
	auto const chip = [&](ImVec2 const At, char const *Text, ImU32 const Background, ImU32 const Foreground) {
		auto const size{ImGui::CalcTextSize(Text)};
		drawlist->AddRectFilled(ImVec2(At.x, At.y - 2.0f), ImVec2(At.x + size.x + 2.0f * pad, At.y + size.y + 2.0f), Background, 3.0f);
		drawlist->AddText(ImVec2(At.x + pad, At.y), Foreground, Text);
		return size.x + 2.0f * pad;
	};
	auto const width_of = [&](key_hint const &Hint) { return ImGui::CalcTextSize(Hint.key).x + 2.0f * pad + 5.0f + ImGui::CalcTextSize(Hint.action.c_str()).x; };

	auto hints{track_key_hints(false)};
	auto const mode{track_mode_name()};
	auto const left{editor_ui::view_min().x + margin};
	auto const available{editor_ui::view_max().x - editor_ui::view_min().x - 2.0f * margin};
	float width{ImGui::CalcTextSize(mode.c_str()).x + 2.0f * pad};
	std::size_t shown{0};
	for (; shown < std::min<std::size_t>(hints.size(), 3); ++shown)
	{
		auto const next{width + gap + width_of(hints[shown])};
		if (next > available)
			break;
		width = next;
	}
	auto const height{ImGui::GetTextLineHeight()};
	auto const y{editor_ui::view_max().y - margin - height};
	ImVec2 const from{left - 6.0f, y - pad - 2.0f};
	ImVec2 const to{left + width + 6.0f, y + height + pad};
	auto const hovered{io.MousePos.x > from.x - 40.0f && io.MousePos.x < to.x + 40.0f && io.MousePos.y > from.y - 40.0f && io.MousePos.y < to.y + 40.0f};
	auto const fade = [hovered](ImU32 const Colour) { return hovered ? (Colour & ~IM_COL32_A_MASK) | (((Colour >> IM_COL32_A_SHIFT) & 0xFF) / 4 << IM_COL32_A_SHIFT) : Colour; };
	drawlist->AddRectFilled(from, to, fade(IM_COL32(0, 0, 0, 160)), 4.0f);
	auto x{left};
	x += chip(ImVec2(x, y), mode.c_str(), fade(IM_COL32(60, 140, 230, 235)), fade(IM_COL32(255, 255, 255, 255)));
	for (std::size_t i = 0; i < shown; ++i)
	{
		x += gap;
		x += chip(ImVec2(x, y), hints[i].key, fade(IM_COL32(255, 210, 60, 220)), fade(IM_COL32(20, 20, 20, 255))) + 5.0f;
		drawlist->AddText(ImVec2(x, y), fade(IM_COL32(255, 255, 255, 230)), hints[i].action.c_str());
		x += ImGui::CalcTextSize(hints[i].action.c_str()).x;
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
	if (pointing && ImGui::GetIO().KeyAlt && m_typed.empty() && false == lay.points.empty())
		lay.mouse = lay_angle_snap(lay.mouse);
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

std::vector<segment_data> editor_mode::extend_pieces() const
{
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
	auto pieces{editor_track::layout_pieces(m_track_layout, tool.point, tool.direction, tool.grade, tool.mouse, radius)};
	follow_ground(pieces, tool.point.y, tool.grade, tool.mouse.y, std::numeric_limits<double>::quiet_NaN(), tool.track);
	return pieces;
}

double editor_mode::design_speed(TTrack const *From, TTrack const *To) const
{
	double speed{0.0};
	for (auto const *track : {From, To})
	{
		if (track == nullptr)
			continue;
		auto const velocity{editor_track::velocity(*track)};
		if (velocity > 0.0)
			speed = speed > 0.0 ? std::min(speed, velocity) : velocity;
	}
	if (speed <= 0.0)
		speed = m_lay.style.velocity > 0.0 ? m_lay.style.velocity : 100.0;
	return speed;
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
	request.kink = m_joints.angle * 0.5;
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
		if (m_join_radius > 0.0 || warning.find("R reduced") == std::string::npos)
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
}

void editor_mode::render_layout_combo()
{
	char const *const names[]{"Arc, then straight", "Straight, then arc", "Arc through the cursor", "Straight"};
	auto current{static_cast<int>(m_track_layout)};
	ImGui::PushItemWidth(200.0f);
	if (ImGui::BeginCombo(STR_C("Shape from a free end"), STR_C(names[current])))
	{
		for (int i = 0; i < 4; ++i)
			if (ImGui::Selectable(STR_C(names[i]), i == current))
				m_track_layout = static_cast<track_layout>(i);
		ImGui::EndCombo();
	}
	ImGui::PopItemWidth();
	item_tooltip("What a drag or a click from a free end lays towards the cursor; Shift+wheel changes it, Ctrl+wheel the radius");
}

bool editor_mode::track_wheel(double const Offset)
{
	auto const &io{ImGui::GetIO()};
	if (m_switch.placing && m_switch.connection >= 0 && Offset != 0.0 && (io.KeyCtrl || io.KeyShift || io.KeyAlt))
	{
		auto &tool{m_switch};
		int const step{Offset > 0.0 ? 1 : -1};
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
	int const step{Offset > 0.0 ? 1 : -1};
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
	first_step_done(first_step::wheel);
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
			pieces = editor_track::layout_pieces(m_track_layout, Points[0], glm::normalize(leaving), grade, Points[1], tool.radius);
			Error = narrowed(pieces, m_track_layout, tool.radius);
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
			auto const back{editor_track::layout_pieces(m_track_layout, End.position, glm::normalize(-plan_of(End.direction)), -grade_of(End.direction), Points[0], tool.radius)};
			for (auto piece{back.rbegin()}; piece != back.rend(); ++piece)
				pieces.push_back(reversed(*piece));
			Error = narrowed(pieces, m_track_layout, tool.radius);
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
		tool.points.push_back(snap.track != nullptr ? snap.position : tool.mouse);
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
	editor_track::commit(created);
	digitize_drawn(created);
	push_track_snapshot({}, created);
	first_step_done(first_step::laid);
	if (End.track != nullptr)
		first_step_done(first_step::join);
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
	tool.chain.clear();
	tool.preview.clear();
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

glm::dvec3 editor_mode::lay_angle_snap(glm::dvec3 const &Mouse) const
{
	auto const &last{m_lay.points.back()};
	glm::dvec2 const offset{Mouse.x - last.x, Mouse.z - last.z};
	auto const length{glm::length(offset)};
	if (length < 1.0)
		return Mouse;
	auto const leaving{m_lay.start.track != nullptr ? -plan_of(m_lay.start.direction) : glm::dvec2{0.0}};
	glm::dvec2 const reference{glm::length(leaving) > 1e-6 ? glm::normalize(leaving) : glm::dvec2{0.0, 1.0}};
	auto const step{glm::radians(15.0)};
	auto const heading{turned(reference, std::round(signed_angle(reference, offset / length) / step) * step) * length};
	return {last.x + heading.x, Mouse.y, last.z + heading.y};
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

bool editor_mode::render_norms_choice()
{
	ImGui::AlignTextToFramePadding();
	ImGui::TextUnformatted(STR_C("Norms:"));
	auto changed{false};
	char const *names[]{STR_C("free"), STR_C("PKP PLK")};
	for (int i = 0; i < 2; ++i)
	{
		ImGui::SameLine();
		auto const chosen{m_norms_free == (i == 0)};
		if (chosen)
			ImGui::PushStyleColor(ImGuiCol_Button, ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive));
		if (ImGui::Button(names[i], ImVec2(90.0f, 0.0f)) && false == chosen)
		{
			m_norms_free = i == 0;
			m_lay.transitions = false == m_norms_free;
			changed = true;
		}
		if (chosen)
			ImGui::PopStyleColor();
		item_tooltip(i == 0 ? "Plain arcs with no transition curves and no cant, no warnings about the norms: the track goes where it's clicked"
		                    : "Transition curves and cant for the design speed, warnings when the radius, the cant or the transitions don't meet the norms");
	}
	if (changed && false == m_route.chain.tracks.empty())
		route_update();
	return changed;
}

void editor_mode::render_lay_ui()
{
	auto &tool{m_lay};
	render_norms_choice();
	ImGui::Separator();
	ImGui::PushItemWidth(120.0f);
	ImGui::DragScalar(STR_C("Curve radius (m)"), ImGuiDataType_Double, &tool.radius, 5.0f, nullptr, nullptr, "%.0f");
	item_tooltip("Drag to change, Ctrl+click to type the value. While laying: Ctrl+wheel");
	tool.radius = std::max(10.0, tool.radius);
	ImGui::DragScalar(STR_C("Speed (km/h)"), ImGuiDataType_Double, &tool.style.velocity, 1.0f, nullptr, nullptr, "%.0f");
	tool.style.velocity = std::clamp(tool.style.velocity, 0.0, 400.0);
	item_tooltip(m_norms_free ? "Speed limit of the new paths; the speed of the track joined comes first"
	                          : "Speed limit of the new paths, and the design speed of the transition curves and of the cant; the speed of the track joined comes first, and none higher than the radius allows");
	ImGui::PopItemWidth();
	render_layout_combo();
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
	ImGui::PushItemWidth(120.0f);
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
	if (switch_link plan; plan_switch_link(plan))
	{
		plan.paths.insert(plan.paths.end(), plan.link.pieces.begin(), plan.link.pieces.end());
		return plan.paths;
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
	if (tool.connection >= 0)
	{
		if (false == tool.curved)
		{
			insert_connection();
			return;
		}
		tool.placing = false;
		tool.status = STR_C("A connection is laid between straights");
		ui()->set_status(tool.status, true);
		return;
	}
	switch_link plan;
	auto const linked{plan_switch_link(plan)};
	tool.placing = false;
	if (tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()))
		return;
	if (linked)
	{
		insert_linked_switch(plan);
		return;
	}
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
	if (false == plan.reason.empty())
		ui()->set_status(plan.reason, true);
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
	m_switch.placing = false;
	first_step_done(first_step::turnout);
	m_track_point = {};
	m_node = *found;
	ui()->set_node(m_node);
	if (slip)
	{
		auto const known{switch_store::named(**found)};
		if (false == known.has_value() || known->kind != "double")
			return;
		std::vector<std::string> driven;
		for (auto *track : Created)
			if (track != nullptr && track->eType == tt_Switch && place_switch_drive(*track))
			{
				join_last_steps();
				driven.push_back(track->name());
			}
		if (driven.size() == 2)
		{
			m_switch.status = format(STR_C("Double switch %s + %s with drives: events %s+, %s- and %s+, %s- switch them, also in the map"), driven[0].c_str(), driven[1].c_str(), driven[0].c_str(), driven[0].c_str(), driven[1].c_str(), driven[1].c_str());
			ui()->set_status(m_switch.status, false);
		}
		advance_switch_name();
		return;
	}
	if (place_switch_drive(**found))
		join_last_steps();
	advance_switch_name();
}

void editor_mode::advance_switch_name()
{
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

void editor_mode::join_last_steps()
{
	if (m_history.size() < 2)
		return;
	auto last{std::move(m_history.back())};
	m_history.pop_back();
	auto &previous{m_history.back()};
	previous.instances.insert(previous.instances.end(), last.instances.begin(), last.instances.end());
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
		double from;
		double to;
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
	int first{-1};
	int last{-1};
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
	auto const from{switch_from(Line, shape.length, Along, Direction, Snap)};
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
	auto const made{create_switch_set(shape, paths, Style != nullptr ? *Style : *first, Name)};
	Created.insert(Created.end(), made.begin(), made.end());
	return true;
}

std::vector<TTrack *> editor_mode::create_switch_set(editor_track::switch_template const &Shape, std::vector<segment_data> const &Paths, TTrack const &Style, std::string const &Name)
{
	if (Shape.twofold() && Paths.size() >= 5)
	{
		auto firstshape{Shape};
		firstshape.length = glm::distance(Paths[0].points[segment_data::point::start], Paths[0].points[segment_data::point::end]);
		auto secondshape{Shape};
		secondshape.length = Shape.length - Shape.stagger;
		auto *one{editor_track::create_switch(firstshape, Paths, Style, Name)};
		auto *two{one != nullptr ? editor_track::create_switch(secondshape, {Paths[2], Paths[3]}, Style, Name + "b") : nullptr};
		auto *link{two != nullptr ? editor_track::create_path(Style, Paths[4]) : nullptr};
		if (link == nullptr)
		{
			m_switch.status = STR_C("The switch couldn't be created from this template");
			return {};
		}
		editor_track::commit({one, two, link});
		record_double(*one, *two, Shape);
		m_switch.status = format(STR_C("Double switch %s with its second switch %s inserted"), one->name().c_str(), two->name().c_str());
		return {one, two, link};
	}
	auto *track{editor_track::create_switch(Shape, Paths, Style, Name)};
	if (track == nullptr)
	{
		m_switch.status = STR_C("The switch couldn't be created from this template");
		return {};
	}
	editor_track::commit({track});
	record_switch(*track, Shape);
	m_switch.status = STR_C("Switch ") + track->name() + STR_C(" inserted");
	return {track};
}

void editor_mode::draw_build_overlay() const
{
	screen_projection const projection;
	ImDrawList *drawlist = ImGui::GetBackgroundDrawList(ImGui::GetMainViewport());
	auto const drawpath = [&](segment_data const &Path, ImU32 const Color) {
		for (auto const &piece : Path.pieces.empty() ? std::vector<segment_data>{Path} : Path.pieces)
		{
			bezier const curve{piece};
			auto previous{curve.point(0.0)};
			for (int i = 1; i <= 24; ++i)
			{
				auto const next{curve.point(i / 24.0)};
				projection.line(drawlist, previous, next, Color, 3.0f);
				previous = next;
			}
		}
	};
	if (false == GfxRenderer->Mouse_Hit() && false == ImGui::GetIO().WantCaptureMouse)
	{
		auto const centre{m_lay.active ? m_lay.mouse : cursor_ground()};
		double const cell{10.0};
		int const reach{10};
		glm::dvec3 const origin{std::round(centre.x / cell) * cell, centre.y, std::round(centre.z / cell) * cell};
		for (int i = -reach; i <= reach; ++i)
		{
			auto const major{std::fmod(std::abs(origin.x / cell + i), 10.0) < 0.5};
			auto const colour{major ? IM_COL32(255, 255, 255, 70) : IM_COL32(255, 255, 255, 35)};
			for (int j = -reach; j < reach; ++j)
				projection.line(drawlist, origin + glm::dvec3{i * cell, 0.0, j * cell}, origin + glm::dvec3{i * cell, 0.0, (j + 1) * cell}, colour, 1.0f);
		}
		for (int j = -reach; j <= reach; ++j)
		{
			auto const major{std::fmod(std::abs(origin.z / cell + j), 10.0) < 0.5};
			auto const colour{major ? IM_COL32(255, 255, 255, 70) : IM_COL32(255, 255, 255, 35)};
			for (int i = -reach; i < reach; ++i)
				projection.line(drawlist, origin + glm::dvec3{i * cell, 0.0, j * cell}, origin + glm::dvec3{(i + 1) * cell, 0.0, j * cell}, colour, 1.0f);
		}
	}
	if (m_track_tab == track_tab::path && false == m_lay.active && false == m_track_gizmo_using)
		if (auto const spot{split_spot()}; spot.track != nullptr)
		{
			ImVec2 at;
			if (projection.project(spot.point, at))
			{
				drawlist->AddCircle(at, 6.0f, overlay_color::highlight, 12, 1.5f);
				drawlist->AddText({at.x + 9.0f, at.y - 18.0f}, overlay_color::highlight, STR_C("K: split here"));
			}
		}
	auto const &display{ImGui::GetIO().DisplaySize};
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
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
	if (m_switch.placing && m_switch.connection >= 0 && false == m_switch.curved)
	{
		connection::request request;
		connection::result plan;
		auto const valid{plan_connection(request, plan)};
		if (valid)
		{
			for (auto const &entry : plan.switches)
				for (auto const &path : entry.paths)
					drawpath(path, overlay_color::marked);
			for (auto const &insert : plan.inserts)
				drawpath(insert, overlay_color::grip);
			if (plan.crossing)
				for (int i = 0; i < 4; i += 2)
				{
					segment_data path;
					path.points[segment_data::point::start] = plan.ends[i];
					path.points[segment_data::point::end] = plan.ends[i + 1];
					drawpath(path, overlay_color::grip);
				}
		}
		else
			for (auto const &piece : switch_preview())
				drawpath(piece, overlay_color::invalid);
		ImVec2 screen;
		auto const text{valid ? connection_readout(plan) : plan.error.empty() ? std::string{STR_C("Drag onto the other track")} : plan.error};
		if (projection.project(m_switch.mouse, screen))
		{
			auto const size{ImGui::CalcTextSize(text.c_str())};
			drawlist->AddRectFilled(ImVec2(screen.x + 14.0f, screen.y + 10.0f), ImVec2(screen.x + 22.0f + size.x, screen.y + 14.0f + size.y), IM_COL32(0, 0, 0, 180), 3.0f);
			drawlist->AddText(ImVec2(screen.x + 18.0f, screen.y + 12.0f), valid ? (plan.warning.empty() ? overlay_color::highlight : overlay_color::warning) : overlay_color::invalid, text.c_str());
		}
	}
	else if (m_switch.placing)
	{
		for (auto const &piece : switch_preview())
			drawpath(piece, overlay_color::marked);
		switch_link plan;
		auto const linked{plan_switch_link(plan)};
		ImVec2 screen;
		if (plan.target.track != nullptr && projection.project(plan.target.position, screen))
		{
			drawlist->AddCircle(screen, 14.0f, linked ? IM_COL32(255, 60, 255, 255) : overlay_color::invalid, 20, 3.0f);
			if (false == linked && false == plan.reason.empty())
				drawlist->AddText({screen.x + 18.0f, screen.y - 8.0f}, overlay_color::invalid, plan.reason.c_str());
		}
	}
	if (m_lay.active)
	{
		auto const &lay{m_lay};
		auto const piece_colour = [&](segment_data const &Path) {
			if (false == lay.preview_error.empty())
				return overlay_color::warning;
			return piece_plan_radius(Path) < 0.5 ? IM_COL32(255, 255, 255, 230) : overlay_color::marked;
		};
		for (auto const &piece : lay.preview)
			drawpath(piece, piece_colour(piece));
		for (std::size_t i = 0, n = lay.preview.size(); i < n;)
		{
			auto const radius{piece_plan_radius(lay.preview[i])};
			bool const curved{radius >= 0.5};
			std::size_t j{i + 1};
			double length{bezier{lay.preview[i]}.plan_length()};
			double sharpest{radius};
			for (; j < n; ++j)
			{
				auto const next{piece_plan_radius(lay.preview[j])};
				if ((next >= 0.5) != curved)
					break;
				length += bezier{lay.preview[j]}.plan_length();
				if (curved && next < sharpest)
					sharpest = next;
			}
			double along{0.0};
			glm::dvec3 middle{bezier{lay.preview[i]}.point(0.5)};
			for (std::size_t k = i; k < j; ++k)
			{
				auto const piece{bezier{lay.preview[k]}.plan_length()};
				if (along + piece >= length * 0.5)
				{
					middle = bezier{lay.preview[k]}.point(piece > 1e-6 ? std::clamp((length * 0.5 - along) / piece, 0.0, 1.0) : 0.5);
					break;
				}
				along += piece;
			}
			ImVec2 screen;
			if (projection.project(middle, screen))
			{
				auto const label{curved ? format("R %.0f", sharpest) : std::string{STR_C("straight")}};
				auto const size{ImGui::CalcTextSize(label.c_str())};
				drawlist->AddRectFilled(ImVec2(screen.x + 6.0f, screen.y - 8.0f), ImVec2(screen.x + 14.0f + size.x, screen.y + 8.0f), IM_COL32(0, 0, 0, 180), 3.0f);
				drawlist->AddText(ImVec2(screen.x + 10.0f, screen.y - 8.0f), curved ? overlay_color::marked : overlay_color::highlight, label.c_str());
			}
			i = j;
		}
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
			auto label{template_label(entry)};
			label += "##switch" + std::to_string(i);
			if (ImGui::Selectable(label.c_str(), tool.armed == i))
			{
				tool.last = i;
				tool.armed = tool.last;
			}
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
		if (ImGui::CollapsingHeader(STR_C("Slip switches and crossings")))
		{
			ImGui::TextDisabled("%s", STR_C("Only at a crossing of two straights"));
			list(true);
		}
	}
	render_connection_ui();
	ImGui::PushItemWidth(120.0f);
	ImGui::InputTextWithHint(STR_C("Name##switchname"), switch_name_for_new().c_str(), tool.name, sizeof(tool.name));
	ImGui::PopItemWidth();
	item_tooltip("Name of the next switch, its drive events are named after it; empty: the next free z1, z2...; a number at the end goes up with each switch");
	if (render_switch_drive_choice())
		if (auto *track{selected_track()}; track != nullptr && track->eType == tt_Switch && false == track->m_editorremoved)
			apply_switch_drive_choice(*track);
	if (false == tool.status.empty())
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", tool.status.c_str());
	if (ImGui::SmallButton(STR_C("Collect from the scenery")))
	{
		tool.templates = editor_track::standard_switch_templates();
		auto const found{editor_track::find_switch_templates()};
		tool.templates.insert(tool.templates.end(), found.begin(), found.end());
		tool.collected = true;
		arm_switch();
	}
	item_tooltip("Looks for the switches of the scenery, to use their shapes as the templates");
	ImGui::SameLine();
	if (ImGui::SmallButton(STR_C("Template editor...")))
		m_switch_designer.open = true;
	item_tooltip("Own switch templates: diverging tracks of straights, arcs and transition curves, double switches");
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
		double from;
		double to;
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
	int first{-1};
	int last{-1};
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
			first = -1;
			last = -1;
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
	{
		vertex.transition_out = state.transition;
		vertex.transition_in = vertex.transition_out;
	}
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
	if (false == tool.active || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() < 2 || tool.path != 1 || false == tool.atend || m_track_layout == track_layout::straight)
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
	auto const &control{(diverging.pieces.empty() ? diverging : diverging.pieces.back()).points[segment_data::point::control2]};
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
	editor_track::switch_template shape;
	shape.source = tool.track;
	shape.length = glm::dot(plan_of(main.points[segment_data::point::end] - origin), axis);
	for (int i = 0; i < 2; ++i)
	{
		auto const &path{own.m_paths[i]};
		auto &local{shape.local[i]};
		local = path;
		auto const tolocal = [&](glm::dvec3 const &Vector) { return glm::dvec3{glm::dot(plan_of(Vector), perpendicular) * side, Vector.y, glm::dot(plan_of(Vector), axis)}; };
		local.transform([&](glm::dvec3 const &Point) { return tolocal(Point - origin); }, tolocal);
		if (side < 0.0)
		{
			local.rolls = {-local.rolls[0], -local.rolls[1]};
			for (auto &piece : local.pieces)
				piece.rolls = {-piece.rolls[0], -piece.rolls[1]};
		}
	}
	Plan.shape = shape;
	auto const faced{connection::face(frog, tangent, connection::of(line), 1, shape)};
	if (false == faced.valid)
		return false;
	Plan.along = faced.partner.along;
	Plan.direction = faced.partner.forward;
	Plan.side = faced.partner.side;
	Plan.paths = faced.partner.paths;
	Plan.insert = faced.insert;
	return true;
}

std::string editor_mode::link_problem(editor_track::snap_target const &Target, editor_track::branch_link const &Link) const
{
	auto const name{Target.track != nullptr && false == Target.track->name().empty() ? Target.track->name() : std::string{"(noname)"}};
	if (false == Link.valid)
		return Link.shortfall > 0.0 ? format(STR_C("The branch can't lead into %s: the R %.0f m curve needs %.2f m more room"), name.c_str(), Link.radius, Link.shortfall) :
		                              format(STR_C("The branch can't lead into %s"), name.c_str());
	if (Link.trim > 0.0 && (false == stretchable(Target) || geometry::chord_length(Target.track->m_paths[Target.point.path]) < Link.trim + 1.0))
		return format(STR_C("%s would have to be shortened by %.2f m, it's too short or not straight"), name.c_str(), Link.trim);
	return {};
}

bool editor_mode::stretchable(editor_track::snap_target const &Target) const
{
	return Target.track != nullptr && Target.track->eType == tt_Normal && Target.track->m_paths.size() == 1 && editor_track::is_straight(*Target.track, m_straights.tolerance);
}

bool editor_mode::plan_switch_link(switch_link &Plan) const
{
	auto const &tool{m_switch};
	if (false == tool.placing || tool.curved || tool.connection >= 0 || tool.armed < 0 || tool.armed >= static_cast<int>(tool.templates.size()) || tool.line.tracks.empty() || tool.templates[tool.armed].double_slip)
		return false;
	auto const &shape{tool.templates[tool.armed]};
	auto const &line{tool.line};
	std::vector<TTrack const *> const exclude(line.tracks.begin(), line.tracks.end());
	auto const category{line.tracks.front()->iCategoryFlag & 15};
	auto const orient = [&](glm::dvec3 const &Toward) {
		glm::dvec2 const offset{plan_of(Toward - tool.point)};
		Plan.forward = glm::dot(offset, line.direction) >= 0.0 ? 1 : -1;
		Plan.side = cross(line.direction * static_cast<double>(Plan.forward), offset) >= 0.0 ? 1 : -1;
	};
	auto const place = [&](double const Along) {
		auto const point{line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * Along};
		return editor_track::place_switch(shape, point, line.direction * static_cast<double>(Plan.forward), Plan.side, line.grade * Plan.forward);
	};
	auto const &io{ImGui::GetIO()};
	if (false == m_cursor_override.has_value())
		Plan.target = snap_free_end(tool.mouse, {io.MousePos.x, io.MousePos.y}, nullptr, category, exclude);
	if (Plan.target.track == nullptr)
		Plan.target = editor_track::find_free_end(nullptr, category, tool.mouse, kLaySnapRadius, exclude);
	if (Plan.target.track != nullptr)
		orient(Plan.target.position);
	else
	{
		orient(tool.mouse);
		auto const paths{place(tool.along)};
		if (paths.size() < 2)
			return false;
		Plan.target = editor_track::find_free_end(nullptr, category, paths[1].points[segment_data::point::end], kLaySnapRadius, exclude);
		if (Plan.target.track == nullptr)
			return false;
	}
	if (shape.length > line.length - 0.02)
		return false;
	auto const kink{glm::radians(m_joints.angle * 0.5)};
	auto const plan = [&](double const Along) {
		Plan.along = Along;
		Plan.paths = place(Along);
		if (Plan.paths.size() < 2)
			return false;
		auto const radius{editor_track::branch_radius(Plan.paths[1])};
		Plan.link = editor_track::link_branch(branch_start(Plan.paths[1]), radius, branch_target(Plan.target), kink,
		                                      editor_track::reverse_insert(std::min(editor_track::diverging_speed(radius), design_speed(nullptr, Plan.target.track))), stretchable(Plan.target));
		Plan.reason = link_problem(Plan.target, Plan.link);
		return Plan.reason.empty();
	};
	auto const from{switch_from(line, shape.length, tool.along, Plan.forward, true)};
	auto const snapped{Plan.forward > 0 ? from : from + shape.length};
	auto const paths{place(snapped)};
	if (paths.size() < 2)
		return false;
	if (auto const shift{editor_track::branch_shift(branch_start(paths[1]), line.direction * static_cast<double>(Plan.forward), editor_track::branch_radius(paths[1]), branch_target(Plan.target), kink)})
	{
		auto const along{snapped + *shift * Plan.forward};
		auto const begin{Plan.forward > 0 ? along : along - shape.length};
		Plan.moved = std::abs(*shift);
		if (begin >= -1e-6 && begin + shape.length <= line.length + 1e-6 && plan(along) && false == Plan.link.reverse)
			return true;
	}
	Plan.moved = 0.0;
	return plan(snapped);
}

void editor_mode::insert_linked_switch(switch_link const &Plan)
{
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	std::vector<TTrack *> removed;
	if (false == place_switch_on_straight(m_switch.line, m_switch.templates[m_switch.armed], nullptr, Plan.along, Plan.forward, Plan.side, false, states, created, removed, switch_name_for_new()))
		return;
	auto const placed{created};
	link_tracks(Plan.target, Plan.link, Plan.target.track->eType == tt_Normal ? *Plan.target.track : *m_switch.line.tracks.front(), states, created);
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	m_switch.status.clear();
	switch_placed(placed);
	auto summary{link_summary(Plan.target, Plan.link, editor_track::branch_radius(Plan.paths[1]))};
	if (Plan.moved >= 0.005)
		summary += format(STR_C(", the switch moved %.2f m"), Plan.moved);
	m_switch.status = summary + (m_switch.status.empty() ? std::string{} : "\n" + m_switch.status);
	ui()->set_status(m_switch.status, false);
}

bool editor_mode::plan_branch_link(editor_track::branch_link &Link, std::string &Reason) const
{
	auto const &tool{m_extend};
	editor_track::snap_target target;
	Reason.clear();
	if (false == extend_snap(target) || tool.track == nullptr || tool.track->eType != tt_Switch || tool.track->m_paths.size() != 2 || tool.path != 1 || false == tool.atend)
		return false;
	if (std::find(tool.line.begin(), tool.line.end(), target.track) != tool.line.end())
		return false;
	auto const &diverging{tool.track->m_paths[1]};
	auto const radius{editor_track::branch_radius(diverging)};
	Link = editor_track::link_branch(branch_start(diverging), radius, branch_target(target), glm::radians(m_joints.angle * 0.5),
	                                 editor_track::reverse_insert(std::min(editor_track::diverging_speed(radius), design_speed(tool.track, target.track))), stretchable(target));
	Reason = link_problem(target, Link);
	return Reason.empty();
}

void editor_mode::finish_branch_link(editor_track::branch_link const &Link)
{
	auto *own{m_extend.track};
	auto const target{m_extend.snap};
	m_extend.active = false;
	std::vector<std::pair<TTrack *, editor_track::state>> states;
	std::vector<TTrack *> created;
	auto const &style{target.track->eType == tt_Normal || m_extend.line.empty() ? *target.track : *m_extend.line.front()};
	auto const linked{link_tracks(target, Link, style, states, created)};
	push_track_snapshot(std::move(states), std::move(created));
	m_node = linked.empty() ? own : linked.back();
	ui()->set_node(m_node);
	straight_refresh();
	ui()->set_status(link_summary(target, Link, editor_track::branch_radius(own->m_paths[1])), false);
}

std::vector<TTrack *> editor_mode::link_tracks(editor_track::snap_target const &Target, editor_track::branch_link const &Link, TTrack const &Style, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created)
{
	auto *target{Target.track};
	if (Link.trim != 0.0)
	{
		States.emplace_back(target, editor_track::capture(*target));
		editor_track::move_point(*target, Target.point, Link.landing);
		auto &path{target->m_paths[Target.point.path]};
		path.points[segment_data::point::control1] = glm::dvec3{0.0};
		path.points[segment_data::point::control2] = glm::dvec3{0.0};
		editor_track::commit({target});
	}
	std::vector<TTrack *> pieces;
	for (auto const &piece : Link.pieces)
		pieces.push_back(editor_track::create_path(Style, piece));
	editor_track::commit(pieces);
	digitize_drawn(pieces);
	Created.insert(Created.end(), pieces.begin(), pieces.end());
	return pieces;
}

std::vector<TTrack *> editor_mode::build_double_slip(glm::dvec3 const &PointA, glm::dvec3 const &PointB, glm::dvec3 const &PointC, glm::dvec3 const &PointD, glm::dvec2 const &Crossing, double const Height, glm::dvec2 const &First, glm::dvec2 const &Second, double const Angle, double const Radius, TTrack &Style, std::string const &Type)
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
	auto const middle{Angle * 0.5};
	auto const split{gap / Radius};
	auto const centreA{at(Crossing - First * gap, Height)};
	auto const centreB{at(Crossing + First * gap, Height)};
	auto const centreC{at(Crossing - Second * gap, Height)};
	auto const centreD{at(Crossing + Second * gap, Height)};
	std::string base;
	auto const prefix{Type == "Skrz" ? "editor_skrz" : Type == "Rkp" ? "editor_dkp" : "editor_dks"};
	for (int i = 1; base.empty() || simulation::Paths.find(base + "_a") != nullptr || simulation::Paths.find(base + "_d") != nullptr; ++i)
		base = prefix + std::to_string(i);
	std::vector<TTrack *> parts;
	if (Type == "Skrz")
	{
		// the two tracks just cross, their ends far from the crossing
		parts.push_back(editor_track::create_path(Style, straight(PointA, PointB)));
		parts.push_back(editor_track::create_path(Style, straight(PointC, PointD)));
		parts.erase(std::remove(parts.begin(), parts.end(), nullptr), parts.end());
		record_slip(parts, "crossing", Type, 0.0, Angle, base);
		return parts;
	}
	editor_track::switch_template shape;
	shape.length = glm::distance(plan_of(PointA), Crossing);
	bool const single{Type == "Rkp"};
	parts.push_back(editor_track::create_switch(shape, {straight(PointA, centreA), arc(PointA, First, side, PointD, 0.0, middle - split)}, Style, base + "_a"));
	if (single)
		parts.push_back(editor_track::create_path(Style, straight(PointB, centreB)));
	else
		parts.push_back(editor_track::create_switch(shape, {straight(PointB, centreB), reversed(arc(PointC, Second, -side, PointB, middle + split, Angle))}, Style, base + "_b"));
	if (single)
		parts.push_back(editor_track::create_path(Style, straight(PointC, centreC)));
	else
		parts.push_back(editor_track::create_switch(shape, {straight(PointC, centreC), arc(PointC, Second, -side, PointB, 0.0, middle - split)}, Style, base + "_c"));
	parts.push_back(editor_track::create_switch(shape, {straight(PointD, centreD), reversed(arc(PointA, First, side, PointD, middle + split, Angle))}, Style, base + "_d"));
	parts.push_back(editor_track::create_path(Style, straight(centreA, centreB)));
	parts.push_back(editor_track::create_path(Style, straight(centreC, centreD)));
	parts.push_back(editor_track::create_path(Style, arc(PointA, First, side, PointD, middle - split, middle + split)));
	if (false == single)
		parts.push_back(editor_track::create_path(Style, arc(PointC, Second, -side, PointB, middle - split, middle + split)));
	parts.erase(std::remove(parts.begin(), parts.end(), nullptr), parts.end());
	record_slip(parts, "slip", Type, Radius, Angle, base);
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
					if (Shape.type == "Rkpd" && track->eType == tt_Switch && false == track->m_editorremoved && track->DoubleSlip() && glm::distance(plan_of(track->location()), plan_of(Point)) < 25.0)
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
	bool const plain{Shape.type == "Skrz"};
	if (angle < glm::radians(0.5) || angle > glm::radians(plain ? 89.0 : 30.0))
	{
		state.status = STR_C("The straights cross at an angle unsuitable for a double slip");
		return false;
	}
	auto const alongfirst{glm::dot(crossing - plan_of(Line.start), Line.direction)};
	auto const alongsecond{glm::dot(crossing - plan_of(second.start), second.direction)};
	auto const available{std::min({alongfirst, Line.length - alongfirst, alongsecond, second.length - alongsecond}) - 0.5};
	auto radius{Shape.radius};
	auto half{plain ? Shape.a : radius * std::tan(angle * 0.5)};
	if (plain && half > available)
	{
		half = available;
		if (half < 2.0)
		{
			state.status = STR_C("Not enough straight track around the crossing");
			return false;
		}
	}
	else if (half > available)
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
	auto &styled{style != nullptr ? *style : *Line.tracks.front()};
	// a single slip joins the pair of the tracks on the side of the click
	auto const nearer{glm::distance((plan_of(pointA) + plan_of(pointD)) * 0.5, plan_of(Point)) <= glm::distance((plan_of(pointC) + plan_of(pointB)) * 0.5, plan_of(Point))};
	auto const parts{Shape.type == "Rkp" && false == nearer ? build_double_slip(pointC, pointD, pointA, pointB, crossing, heightfirst(alongfirst), direction, first, angle, radius, styled, Shape.type)
	                                                        : build_double_slip(pointA, pointB, pointC, pointD, crossing, heightfirst(alongfirst), first, direction, angle, radius, styled, Shape.type)};
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
	auto const bend = [&](segment_data const &Local) {
		auto const &start{Local.points[segment_data::point::start]};
		auto const &end{Local.points[segment_data::point::end]};
		auto const &control1{Local.points[segment_data::point::control1]};
		auto const &control2{Local.points[segment_data::point::control2]};
		glm::dvec2 const chord{end.x - start.x, end.z - start.z};
		auto const localstart{glm::normalize(control1 != glm::dvec3{} ? plan_of(control1) : chord)};
		auto const localend{glm::normalize(control2 != glm::dvec3{} ? -plan_of(control2) : chord)};
		glm::dvec2 startdirection;
		glm::dvec2 enddirection;
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
		path.radius = Local.radius;
		{
			bezier const curve{path};
			auto const begin{plan_of(curve.first(0.0))};
			auto const finish{plan_of(curve.first(1.0))};
			auto const turn{glm::length(begin) > 1e-9 && glm::length(finish) > 1e-9 ? geometry::signed_angle(begin, finish) : 0.0};
			if (Local.radius != 0.f && std::abs(turn) > 1e-6)
				path.radius = static_cast<float>(std::copysign(curve.plan_length() / std::abs(turn), static_cast<double>(Local.radius)));
		}
		return path;
	};
	std::vector<segment_data> locals{Shape.local.begin(), Shape.local.end()};
	if (Shape.twofold())
	{
		locals.insert(locals.end(), Shape.second.begin(), Shape.second.end());
		locals.push_back(Shape.link);
	}
	std::vector<segment_data> result;
	for (std::size_t i = 0; i < locals.size(); ++i)
	{
		auto const &local{locals[i]};
		auto path{bend(local)};
		for (auto const &piece : local.pieces)
			path.pieces.push_back(bend(piece));
		// the main tracks follow the curve they are laid into
		if (i % 2 == 0)
			path.radius = sample_at(Frame, Station + Direction * 0.5 * (local.points[segment_data::point::start].z + local.points[segment_data::point::end].z)).radius;
		result.push_back(path);
	}
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
	auto const made{create_switch_set(shape, paths, *tool.frame_tracks.front(), switch_name_for_new())};
	created.insert(created.end(), made.begin(), made.end());
	if (made.size() == 1)
		tool.status = STR_C("Switch ") + made.front()->name() + STR_C(" inserted into the curve");
	auto const placed{created};
	push_track_snapshot(std::move(states), std::move(created), std::move(removed));
	straight_refresh();
	switch_placed(placed);
}
