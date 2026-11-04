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

void editor_mode::commit_track_drag(bool const Force)
{
	// on letting go the rest of what follows the dragged tracks is yet to come, even with all of the drag committed
	if ((false == m_track_dirty && false == (Force && m_track_drag_on_the_way)) || m_track_drag.empty())
		return;
	auto const now = std::chrono::steady_clock::now();
	if (false == Force && now - m_track_last_commit < std::chrono::milliseconds(100))
		return;
	editor_track::commit(m_track_drag, false == Force);
	m_track_drag_on_the_way = false == Force;
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
	    {"Objects", "B", track_tab::lineside, "Along the track: hectometre posts, fouling point markers, a vehicle to drive"},
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
		finish_edit(rebuild::parameters);
		if ((track->eType == tt_Normal || track->eType == tt_Switch) && (track->iCategoryFlag & 15) == 1)
		{
			if (ImGui::InputFloat(STR_C("Unevenness (mm)"), &track->m_uneven.value, 0.0f, 0.0f, "%.1f"))
				track->m_uneven.value = std::clamp(track->m_uneven.value, 0.f, track_uneven::most);
			item_tooltip("Peak deviation of a rail on a 10 m chord. Id-14 limits: 6 mm at 160 km/h, 10 at 120, 18 at 80, 35 at 40.\nWith a sound distance set, the rails also dip at the joints. Joined switches use the lowest of their values");
			finish_edit(rebuild::parameters);
			auto const component = [&](char const *Label, float &Value) {
				if (ImGui::InputFloat(STR_C(Label), &Value, 0.0f, 0.0f, "%.1f"))
					Value = Value < 0.f ? -1.f : std::min(Value, track_uneven::most);
				item_tooltip("Set apart from the unevenness, mm. Negative: the unevenness");
				finish_edit(rebuild::parameters);
			};
			component("Up and down (mm)", track->m_uneven.up);
			component("Sideways (mm)", track->m_uneven.side);
			if (track->eType == tt_Normal)
			{
				component("Cross level (mm)", track->m_uneven.cross);
			}
			else
			{
				component("Diverging path (mm)", track->m_uneven.branch);
				component("Diverging, up and down (mm)", track->m_uneven.branchup);
				component("Diverging, sideways (mm)", track->m_uneven.branchside);
			}
		}
		if (track->eType == tt_Normal && (track->iCategoryFlag & 15) == 1)
		{
			if (ImGui::InputFloat(STR_C("Dip at rail joints (mm)"), &track->m_uneven.dip, 0.0f, 0.0f, "%.1f"))
				track->m_uneven.dip = track->m_uneven.dip < 0.f ? -1.f : std::min(track->m_uneven.dip, track_uneven::most);
			item_tooltip("Depth of the dip at each rail joint, every sound distance. Works with unevenness at 0. Negative: 0.4 of the unevenness");
			finish_edit(rebuild::parameters);
		}
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
				auto &buffer{m_track_materials[i]};
				buffer[name.copy(buffer.data(), buffer.size() - 1)] = '\0';
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
		Buffer[Text.copy(Buffer.data(), Buffer.size() - 1)] = '\0';
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
			hints = {{"LMB", "start of the new track, on the ground or at a free end"}, {"Drag from a free end", "straight along it, to another free end: a curve joining them"}, {"Parallel", "first select a straight, then a second track beside it in the panel"}, {"Esc", "stop laying"}};
		else
		{
			hints = {{"LMB", m_lay.points.size() == 1 ? "end of the straight, or a vertex of a curve" : "next vertex of the curve"}, {"LMB on a free end", "join and lay"}, {"Enter", "lay up to the cursor"}, {"Backspace", "take back the point"}, {"Esc", "cancel"}};
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
	else if (m_track_tab == track_tab::straights && m_straights.dragging && m_straights.handle == 2)
	{
		auto const offer{straight_parallel_offer(m_straights.drag_line, m_straights.preview_start - m_straights.drag_line.start)};
		if (offer.snaps)
			hints = {{"Release", format(STR_C("parallel at %.2f m"), offer.spacing)}, {"Esc", "cancel"}};
		else if (offer.nearby)
			hints = {{"Near another straight", format(STR_C("snap at %.2f m  (%.1f°)"), offer.spacing, offer.angle)}, {"Esc", "cancel"}};
		else
			hints = {{"Drag the diamond", "snaps parallel at 4.00 / 4.50 / 4.75 m"}, {"Esc", "cancel"}};
	}
	else if (m_track_tab == track_tab::straights && m_straights.current.tracks.empty())
		hints = {{"LMB on a straight track", "selects the whole straight"}};
	else if (m_track_tab == track_tab::straights)
	{
		hints = {{"Drag a square", "moves an end, the curves at it follow"}, {"Drag the diamond", "shifts sideways; close: snaps parallel at 4.00 / 4.50 / 4.75 m"}, {"Ctrl+drag", "breaks the straight"}, {"Shift+click x4", "shifts a piece around an obstacle"}, {"Alt+click", "adds another straight to move together"}};
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
	if (m_switch.placing)
		for (auto const &piece : switch_preview())
			drawpath(piece, overlay_color::marked);
	if (m_lay.active)
	{
		auto const &lay{m_lay};
		auto const piece_colour = [&](segment_data const &Path) {
			if (false == lay.preview_error.empty())
				return overlay_color::invalid;
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
