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
#include "application/editorgestures.h"
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
#include <cstdlib>
#include <cstring>
#include <functional>
#include <iterator>
#include <set>

#include "application/editormodetrackshared.h"

using namespace track_mode;


TTrack *editor_mode::selected_track() const
{
	return dynamic_cast<TTrack *>(m_node);
}

bool editor_mode::track_editing() const
{
	return m_track_gizmo_using || m_dragging;
}

std::size_t editor_mode::history_size() const
{
	return m_history.size();
}

glm::dvec3 editor_mode::camera_position() const
{
	return Camera.Pos;
}

editor_mode::track_hover const &editor_mode::hovered_track() const
{
	return m_hover;
}

std::vector<TTrack *> const &editor_mode::track_set() const
{
	return m_track_set;
}

editor_track_tab editor_mode::current_track_tab() const
{
	return m_track_tab;
}

void editor_mode::set_current_node(scene::basic_node *Node)
{
	m_node = Node;
	ui()->set_node(m_node);
}

alignment::limits const &editor_mode::route_norms() const
{
	return m_route.design.norms;
}

void editor_mode::clear_track_point()
{
	m_track_point = {};
}

editor_nmt &editor_mode::terrain_model()
{
	return m_nmt;
}

double editor_mode::terrain_model_offset() const
{
	return m_nmt_state.offset;
}

bool editor_mode::terrain_model_fitted() const
{
	return m_nmt_state.fitted;
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
	// a track out of the set ends it, the set is the selection only together with its tracks
	if (false == in_track_set(track))
		m_track_set.clear();
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
	// the tracks of a set are moved by its gizmo, not by the points of one of them
	if (track == nullptr || false == editor_track::is_supported(*track) || m_track_tab != track_tab::path || (m_track_set.size() >= 2 && in_track_set(track)))
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
	// a track of the set is drawn with the set, see draw_track_set()
	if (trackmode && m_track_set.size() >= 2 && in_track_set(track))
		return;

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
	++m_track_edits;
}

bool editor_mode::tracks_editable(std::vector<TTrack *> const &Tracks, std::string &Reason)
{
	for (auto const *track : Tracks)
	{
		std::string reason;
		if (track != nullptr && false == editor_track::can_edit_geometry(*track, reason))
		{
			Reason = name_of(*track) + ": " + reason;
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

	++m_track_edits;
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
			editor_track::revive(*track);
			tracks.push_back(track);
		}
	}
	else
	{
		for (auto *track : Snapshot.removed)
			editor_track::retire(*track);
		for (auto *track : Snapshot.created)
			editor_track::revive(*track);
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
	    {"Select", "Esc", track_tab::path, "LMB on a path selects it: its points, control vectors and parameters", editor_icons::icon::select, 0},
	    {"Lay track", "L", track_tab::lay, "Lays new track through the clicked points, on the ground or from a free end of a track", editor_icons::icon::lay_track, 0},
	    {"Plan", "G", track_tab::straights, "The plan of the line laid: LMB on a straight drags its ends or the middle, on a curve its vertex and radius. C works too", editor_icons::icon::curve, 0},
	    {"Switch", "T", track_tab::turnout, "Puts switches into the selected straight or curve, edits the geometry of the selected switch", editor_icons::icon::turnout, 0},
	    {"Parallel", "N", track_tab::parallel, "A second track beside a track: LMB on it, then the side and the spacing", editor_icons::icon::parallel, 0},
	    {"Turntable", "U", track_tab::turntable, "Places a turntable, leads tracks out of it and fits the tracks around to it", editor_icons::icon::turntable, 1},
	    {"Signals", "H", track_tab::signals, "Signals by the track, from the templates of the scenery, with the event the train reads them by", editor_icons::icon::signal, 1},
	    {"Signalling", "I", track_tab::srk, "The signalling of a station: LMB on a switch lays its station out as a schematic", editor_icons::icon::station, 1},
	    {"Objects", "B", track_tab::lineside, "Along the track: hectometre posts, fouling point markers, trainsets", editor_icons::icon::objects, 1},
	    {"Profile", "P", track_tab::profile, "Vertical profile (grade line) along the line", editor_icons::icon::profile, 2},
	    {"Speed", "V", track_tab::speed, "Speed limits of the paths against the speed their geometry allows", editor_icons::icon::speed, 2},
	    {"Joints", "J", track_tab::joints, "Ends of the paths which almost meet, steps, kinks, jumps of the cant and of the grade at the joints", editor_icons::icon::joints, 2},
	    // the infrastructure tool (track_tab::infra) is left out: no button nor key
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
	if ((Tab == track_tab::lay || Tab == track_tab::straights || Tab == track_tab::route) && false == m_norms_asked && EditorSettings.norms() == editorSettings::track_norms::ask && std::getenv("EU07_EDITOR_SELFTEST") == nullptr)
	{
		m_norms_asked = true;
		m_norms_prompt = true;
	}
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
		scan_turntables(false);
		turntable_read(track);
		break;
	case track_tab::signals:
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
	if (Key == GLFW_KEY_M)
	{
		m_tape = {false == m_tape.active, {}, m_tape.mouse};
		return true;
	}
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
	if (m_tape.active)
		return STR_C("Measure");
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
	case track_tab::srk: return STR_C("Signalling");
	case track_tab::turntable: return STR_C("Turntable");
	default: return STR_C("Select");
	}
}

void editor_mode::render_track_inspector()
{
	auto const &io{ImGui::GetIO()};
	m_profile.open = m_track_window_open && m_track_tab == track_tab::profile;
	m_speed.set_open(m_track_window_open && m_track_tab == track_tab::speed);
	m_infra.open = m_track_window_open && m_track_tab == track_tab::infra;
	m_joints.set_open(m_track_window_open && m_track_tab == track_tab::joints);
	m_srk.set_open(m_track_window_open && m_track_tab == track_tab::srk);
	if (false == m_track_window_open || m_track_tab != track_tab::lineside)
	{
		m_sweep.open = false;
		m_hekto.tab().open = false;
		m_fouling.tab().open = false;
		m_parallel.tab().open = false;
		m_vehicle.tab().open = false;
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
			case track_tab::speed: m_speed.render_options(); break;
			case track_tab::joints: m_joints.render_options(); break;
			default: render_infra_body(); break;
			}
		}
		ImGui::End();
	}

	m_profile.strip_top = -1.0f;
	if (m_track_tab == track_tab::profile && m_track_window_open)
		render_profile_strip();
	m_srk.render_windows();
}

void editor_mode::render_track_tool_options()
{
	auto *track{selected_track()};
	// the tools themselves are in the toolbar
	render_track_search();
	ImGui::Spacing();
	render_track_guide();
	// the texts of the tools go on to the next line at the edge of the panel, none of them makes it scroll sideways
	ImGui::PushTextWrapPos(0.0f);
	switch (m_track_tab)
	{
	case track_tab::lay:
		render_lay_ui();
		break;
	case track_tab::parallel:
		m_parallel.render_options();
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
	case track_tab::srk: m_srk.render_options(); break;
	case track_tab::turntable: render_turntable_ui(); break;
	default:
		render_track_set_ui();
		render_track_spread_ui();
		break;
	}
	ImGui::PopTextWrapPos();
}

void editor_mode::render_track_selection()
{
	track_field_settle();
	auto const tracks{selected_tracks()};
	if (tracks.empty())
	{
		ImGui::TextDisabled("%s", STR_C("Nothing selected"));
		if (m_track_tab != track_tab::path)
			return;
		ImGui::Spacing();
		gesture_rows({{"LMB on a track", "its properties show here"}, {"Ctrl+F", "finds a track by its name"}});
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::Spacing();
		ImGui::TextDisabled("%s", STR_C("Or start with:"));
		auto const width{std::floor((ImGui::GetContentRegionAvail().x - ImGui::GetStyle().ItemSpacing.x) * 0.5f)};
		if (ImGui::Button(STR_C("Lay a track  (L)"), ImVec2(width, ImGui::GetFrameHeight() * 1.4f)))
			show_track_tab(track_tab::lay);
		ImGui::SameLine();
		if (ImGui::Button(STR_C("Tutorial"), ImVec2(width, ImGui::GetFrameHeight() * 1.4f)))
			m_first_steps_open = true;
		return;
	}
	// the properties belong to the select tool and to the switch tool, the other tools show the track they work with
	auto const sheet{m_track_tab == track_tab::path || m_track_tab == track_tab::turnout};
	if (tracks.size() >= 2)
	{
		render_track_set_header(tracks);
		if (false == sheet)
			return;
		render_track_select_tools(tracks);
		render_track_style(tracks);
		render_track_properties(tracks);
		ImGui::Spacing();
		ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.55f, 0.18f, 0.15f, 1.0f));
		if (ImGui::Button(format(STR_P("Delete %d track", "Delete %d tracks", static_cast<long long>(tracks.size())), static_cast<int>(tracks.size())).c_str()))
			track_set_delete();
		ImGui::PopStyleColor();
		item_tooltip("Deletes the selected tracks (Del); Ctrl+Z brings them back");
		return;
	}
	auto *track{tracks.front()};
	if (false == sheet)
	{
		ImGui::TextUnformatted(name_of(*track).c_str());
		ImGui::TextDisabled("%s, %.2f m", STR_C(type_name(*track)), track->Length());
		return;
	}
	render_path_ui();
}

void editor_mode::render_track_guide()
{
	// a card of the tool in use: what it is for, and how the mouse and the keys drive it, drawn
	auto const &modes{track_modes()};
	auto const tab{false == m_track_window_open ? track_tab::path : m_track_tab == track_tab::route ? track_tab::straights : m_track_tab};
	auto const found{std::find_if(modes.begin(), modes.end(), [&](track_mode const &Mode) { return Mode.tab == tab; })};
	auto const family{found != modes.end() ? found->family : 0};
	auto const accent{family_colour(family)};
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 0.6f)};
	auto *list{ImGui::GetWindowDrawList()};
	auto const top{ImGui::GetCursorScreenPos()};
	auto const width{ImGui::GetContentRegionAvail().x};
	auto const right{top.x + width};
	list->ChannelsSplit(2);
	list->ChannelsSetCurrent(1);
	ImGui::BeginGroup();
	ImGui::Dummy(ImVec2(width, pad * 0.6f));
	ImGui::Indent(pad + 3.0f);
	auto const row{ImGui::GetCursorScreenPos()};
	auto const iconside{std::round(font * 1.7f)};
	if (found != modes.end())
		editor_icons::draw(list, found->picture, ImVec2(row.x + iconside * 0.5f, row.y + iconside * 0.5f), accent);
	ImGui::Dummy(ImVec2(iconside, iconside));
	ImGui::SameLine(0.0f, pad);
	ImGui::BeginGroup();
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));
	ImGui::SetWindowFontScale(1.15f);
	ImGui::TextUnformatted(track_mode_name().c_str());
	ImGui::SetWindowFontScale(1.0f);
	ImGui::PopStyleColor();
	ImGui::PushStyleColor(ImGuiCol_Text, ImGui::ColorConvertU32ToFloat4(accent));
	ImGui::TextUnformatted(family_name(family));
	ImGui::PopStyleColor();
	ImGui::EndGroup();
	if (found != modes.end() && found->key != nullptr)
	{
		auto const height{std::round(font * 1.35f)};
		auto const cap{std::max(std::round(ImGui::CalcTextSize(found->key).x + height * 0.6f), height)};
		gesture_key(*list, ImVec2(right - pad - cap, row.y + std::round((iconside - height) * 0.5f)), height, found->key);
	}
	ImGui::PushTextWrapPos(right - pad - ImGui::GetWindowPos().x);
	if (found != modes.end())
	{
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("%s", STR_C(found->tooltip));
		ImGui::PopStyleColor();
	}
	// the gestures, each drawn: the mouse with the button lit, the keys as caps
	auto const hints{track_key_hints(false)};
	auto const gesture{std::round(font * 1.25f)};
	std::vector<std::vector<gesture_word>> drawn;
	float column{0.0f};
	for (auto const &hint : hints)
	{
		drawn.push_back(gesture_picture(hint.raw != nullptr ? hint.raw : hint.key));
		column = std::max(column, gesture_width(gesture, drawn.back()));
	}
	column = std::min(column, font * 6.0f);
	if (false == hints.empty())
		ImGui::Dummy(ImVec2(0.0f, pad * 0.3f));
	for (std::size_t i = 0; i < hints.size() && i < 7; ++i)
	{
		auto const at{ImGui::GetCursorScreenPos()};
		auto const line{ImGui::GetTextLineHeight()};
		auto const height{std::max(gesture, line)};
		list->PushClipRect(at, ImVec2(at.x + column, at.y + height), true);
		gesture_draw(*list, ImVec2(at.x, at.y + std::round((height - gesture) * 0.5f)), gesture, drawn[i], ImGui::GetColorU32(ImGuiCol_Text));
		list->PopClipRect();
		ImGui::Dummy(ImVec2(column, height));
		ImGui::SameLine(0.0f, pad);
		ImGui::SetCursorPosY(ImGui::GetCursorPosY() + std::round((height - line) * 0.5f));
		if (hints[i].raw == nullptr || false == gesture_keys_only(hints[i].raw))
		{
			ImGui::TextColored(ImVec4(1.0f, 0.83f, 0.3f, 1.0f), "%s", hints[i].key);
			ImGui::SameLine();
		}
		ImGui::TextWrapped("%s", hints[i].action.c_str());
	}
	ImGui::PopTextWrapPos();
	ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
	ImGui::TextUnformatted(STR_C("F1: all the shortcuts"));
	ImGui::PopStyleColor();
	if (ImGui::IsItemHovered())
		ImGui::SetMouseCursor(ImGuiMouseCursor_Hand);
	if (ImGui::IsItemClicked())
		m_shortcuts_open = true;
	ImGui::Unindent(pad + 3.0f);
	ImGui::Dummy(ImVec2(width, pad * 0.5f));
	ImGui::EndGroup();
	auto const bottom{ImGui::GetItemRectMax().y};
	list->ChannelsSetCurrent(0);
	auto const rounding{std::round(font * 0.35f)};
	list->AddRectFilled(top, ImVec2(right, bottom), (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 28), rounding);
	list->AddRect(top, ImVec2(right, bottom), (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 70), rounding);
	list->AddRectFilled(top, ImVec2(top.x + 4.0f, bottom), accent, rounding, ImDrawFlags_RoundCornersLeft);
	list->ChannelsMerge();
	if (false == m_track_mode_notice.empty())
	{
		ImGui::Spacing();
		ImGui::PushTextWrapPos(ImGui::GetWindowContentRegionMax().x - 8.0f);
		ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%s", m_track_mode_notice.c_str());
		ImGui::PopTextWrapPos();
	}
	ImGui::Spacing();
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
	if (begin_fields("##tracknamefields", 0.3f))
	{
		field_row("Name##trackname");
		if (ImGui::InputTextWithHint("##trackname", STR_C("(no name)"), edit.text, sizeof(edit.text), ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CharsNoBlank))
			apply();
		if (ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", STR_C("Enter renames the path, Esc while typing takes the text back.\nThe events, memory cells and the map of the game find the path by its name"));
		ImGui::EndTable();
	}
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
		std::vector<std::string> notes;
		if (typed != edit.text)
			notes.push_back(format(STR_C("Saved as %s: the scenery keeps the names in lower case"), typed.c_str()));
		if (edit.uses.cells > 0 || edit.uses.includes > 0)
			notes.push_back(format(STR_C("The new name goes to %d memory cell(s) and %d include(s), the drives of a switch among them"), edit.uses.cells, edit.uses.includes));
		if (warned)
			notes.push_back(STR_C("The events listed below work until the scenery is loaded again; then they need the new name in their files"));
		if (false == notes.empty() && ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", lines_of(notes).c_str());
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
	std::string about;
	if (set.has_value())
	{
		auto const &second{set->second->m_paths.front()};
		about = format(STR_C("Double switch %s + %s: length %.3f m, the second switch starts %.3f m on"), set->first->name().c_str(), set->second->name().c_str(), glm::dot(plan_of(second.points[segment_data::point::end] - origin), axis), glm::dot(plan_of(second.points[segment_data::point::start] - origin), axis)) + "\n" +
		        format(STR_C("First switch: diverging %s, end offset %.3f m"), offset > 0.0 ? STR_C("right") : STR_C("left"), std::abs(offset));
	}
	else
		ImGui::Text(STR_C("Length %.3f m, diverging %s, end offset %.3f m"), length, offset > 0.0 ? STR_C("right") : STR_C("left"), std::abs(offset));
	{
		// the switch in plan at one scale: the main track white, the diverging one yellow, the angle at its end, the length under
		auto const font{ImGui::GetFontSize()};
		auto const width{ImGui::GetContentRegionAvail().x};
		auto const height{std::round(font * 4.2f)};
		auto const top{ImGui::GetCursorScreenPos()};
		ImGui::Dummy(ImVec2(width, height));
		if (false == about.empty() && ImGui::IsItemHovered())
			ImGui::SetTooltip("%s", about.c_str());
		auto *list{ImGui::GetWindowDrawList()};
		list->AddRectFilled(top, ImVec2(top.x + width, top.y + height), ImGui::GetColorU32(ImGuiCol_FrameBg), std::round(font * 0.35f));
		auto const local = [&](glm::dvec3 const &Point) {
			glm::dvec2 const relative{Point.x - origin.x, Point.z - origin.z};
			return glm::dvec2{glm::dot(relative, axis), glm::dot(relative, normal)};
		};
		std::vector<std::vector<glm::dvec2>> lines;
		std::vector<TTrack *> switches{track};
		if (set.has_value())
			switches.push_back(set->second);
		double far_along{0.0}, low_across{0.0}, high_across{0.0};
		for (auto *each : switches)
			for (auto const &path : each->m_paths)
			{
				geometry::bezier const curve{path};
				std::vector<glm::dvec2> points;
				for (int i = 0; i <= 24; ++i)
				{
					auto const point{local(curve.point(i / 24.0))};
					far_along = std::max(far_along, point.x);
					low_across = std::min(low_across, point.y);
					high_across = std::max(high_across, point.y);
					points.push_back(point);
				}
				lines.push_back(std::move(points));
			}
		auto const pad{std::round(font * 0.7f)};
		auto const label{ImGui::GetTextLineHeight()};
		auto const room{height - 2.0f * pad - label};
		auto const scale{static_cast<float>(std::min((width - 2.0 * pad) / std::max(1.0, far_along), room / std::max(0.5, high_across - low_across)))};
		auto const base{top.y + pad + (room - static_cast<float>(high_across - low_across) * scale) * 0.5f - static_cast<float>(low_across) * scale};
		auto const screen = [&](glm::dvec2 const &Point) { return ImVec2(top.x + pad + static_cast<float>(Point.x) * scale, base + static_cast<float>(Point.y) * scale); };
		// the diverging tracks first, the main ones over them
		for (auto const main : {false, true})
			for (auto const &line : lines)
			{
				if ((std::abs(line.back().y - line.front().y) < 0.05) != main)
					continue;
				std::vector<ImVec2> drawn;
				for (auto const &point : line)
					drawn.push_back(screen(point));
				list->AddPolyline(drawn.data(), static_cast<int>(drawn.size()), main ? IM_COL32(235, 235, 235, 255) : IM_COL32(255, 210, 60, 255), 0, 2.0f);
			}
		// the point of the blades, and the angle at the end of the diverging track
		list->AddCircleFilled(screen(glm::dvec2{0.0, 0.0}), 3.0f, IM_COL32(230, 80, 230, 255));
		if (angle > 1e-6)
		{
			auto const text{format("1:%.1f", 1.0 / std::tan(angle))};
			auto const size{ImGui::CalcTextSize(text.c_str())};
			auto const at{screen(glm::dvec2{far_along, offset})};
			list->AddText(ImVec2(std::round(at.x - size.x - 2.0f), std::round(offset > 0.0 ? at.y - size.y - 1.0f : at.y + 1.0f)), IM_COL32(255, 210, 60, 255), text.c_str());
		}
		if (diverging.radius > 0.0f)
		{
			auto const text{format("R %.0f", static_cast<double>(diverging.radius))};
			auto const size{ImGui::CalcTextSize(text.c_str())};
			auto const at{screen(glm::dvec2{far_along * 0.45, offset * 0.25})};
			list->AddText(ImVec2(std::round(at.x - size.x * 0.5f), std::round(offset > 0.0 ? at.y + 3.0f : at.y - size.y - 3.0f)), IM_COL32(255, 210, 60, 200), text.c_str());
		}
		auto const text{format("%.2f m", far_along)};
		auto const size{ImGui::CalcTextSize(text.c_str())};
		auto const y{top.y + height - pad * 0.4f - label};
		list->AddLine(ImVec2(top.x + pad, y + label * 0.5f), ImVec2(top.x + width - pad, y + label * 0.5f), ImGui::GetColorU32(ImGuiCol_TextDisabled, 0.5f));
		list->AddRectFilled(ImVec2(top.x + (width - size.x) * 0.5f - 4.0f, y), ImVec2(top.x + (width + size.x) * 0.5f + 4.0f, y + label), ImGui::GetColorU32(ImGuiCol_FrameBg));
		list->AddText(ImVec2(std::round(top.x + (width - size.x) * 0.5f), y), ImGui::GetColorU32(ImGuiCol_TextDisabled), text.c_str());
	}
	if (auto const known{switch_store::of(*track)}; known.has_value())
		ImGui::TextUnformatted(switch_store::label(*known).c_str());
	// the name, the number, the rail and the kilometre of the switch, with what the scenery keeps of it
	if (ImGui::CollapsingHeader(STR_C("Details##switchdetails")))
	{
		render_track_name(*track);
		render_switch_record(*track);
		if (angle > 1e-6)
			ImGui::Text(STR_C("Angle 1:%.2f (%.4f deg), R in the entry %.0f m"), 1.0 / std::tan(angle), glm::degrees(angle), diverging.radius);
	}
	auto const drive_ui = [&](TTrack &Switch) {
		ImGui::PushID(&Switch);
		if (set.has_value())
			ImGui::TextUnformatted(Switch.name().c_str());
		if (false == EditorIncludes.scanned())
			ImGui::TextDisabled("%s", STR_C("Looking for the templates..."));
		else if (auto const drive{find_switch_drive(Switch, false)})
		{
			auto const family{drive->family};
			auto const variant{drive->index};
			if (begin_fields("##standingdrive", 0.3f))
			{
				field_row("Drive##standing");
				if (ImGui::BeginCombo("##standing", m_switch.drives[family].name.c_str()))
				{
					for (int i = 0; i < static_cast<int>(m_switch.drives.size()); ++i)
						if (ImGui::Selectable(m_switch.drives[i].name.c_str(), family == i) && i != family)
							replace_switch_drive(Switch, i, false);
					ImGui::EndCombo();
				}
				if (ImGui::IsItemHovered())
					ImGui::SetTooltip("%s\n%s", STR_C("Puts another drive in place of the drive of this switch, on the same side"), format(STR_C("Drive: events %s+ and %s- switch it, also in the map"), Switch.name().c_str(), Switch.name().c_str()).c_str());
				ImGui::EndTable();
			}
			int side{((variant >> 1) == (variant & 1)) ? 0 : 1};
			if (segmented("standingside", side, {{0, STR_C("diverging side"), "Puts the drive of the switch on the other side of the track: the same drive, its variant for the other side"},
			                                     {1, STR_C("other side"), "Puts the drive of the switch on the other side of the track: the same drive, its variant for the other side"}}))
				flip_switch_drive(Switch);
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
	{
		bool frogs{track->SwitchExtension->m_frogs};
		bool movable{track->SwitchExtension->m_movablenose};
		char const *const frogtip{"Cuts the rails of the switch at its frog and lays wing rails and check rails.\n"
		                          "Off for a switch whose include has its own model of them"};
		char const *const nosetip{"The nose of the frog moves with the blades over to the wing rail of the track set, leaving no gap; the frog has no check rails"};
		ImGui::BeginDisabled(false == track_features::crossings);
		if (frogs)
			toggle_chips("switchfrogs", {{STR_C("Frog and check rails made from the rails"), &frogs, frogtip}, {STR_C("Movable nose"), &movable, nosetip}});
		else
			toggle_chips("switchfrogs", {{STR_C("Frog and check rails made from the rails"), &frogs, frogtip}});
		ImGui::EndDisabled();
		if (false == track_features::crossings && ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
			ImGui::SetTooltip("%s", STR_C("Switched off in this build of the simulator"));
		if (track_features::crossings && (frogs != track->SwitchExtension->m_frogs || movable != track->SwitchExtension->m_movablenose))
		{
			auto const before{editor_track::capture(*track)};
			track->SwitchExtension->m_frogs = frogs;
			track->SwitchExtension->m_movablenose = movable;
			push_track_snapshot({{track, before}});
			editor_track::commit_parameters(*track);
		}
	}
	if (false == m_switch.status.empty())
		ImGui::TextWrapped("%s", m_switch.status.c_str());

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
	double low{0.0}, high{0.0}, reach{length};
	auto const slides{switch_slides(*track) && (set.has_value() ? switch_slides(*set->second) && double_room(*set, low, high, reach, m_switch.status) : switch_room(*track, low, high, m_switch.status))};
	ImGui::PushID("turnout");
	if (ImGui::TreeNode(STR_C("More settings")))
	{
		ImGui::PushItemWidth(-1.0f);
		if (ImGui::BeginCombo("##turnouttemplate", switch_template_label(tool.templates[m_turnout_template]).c_str()))
		{
			for (int i = 0; i < static_cast<int>(tool.templates.size()); ++i)
			{
				if (false == fits(tool.templates[i]))
					continue;
				auto const label{switch_template_label(tool.templates[i])};
				if (ImGui::Selectable(label.c_str(), i == m_turnout_template))
					m_turnout_template = i;
			}
			ImGui::EndCombo();
		}
		ImGui::PopItemWidth();
		if (ImGui::Button(STR_C("Replace the geometry with the template")))
			replace(tool.templates[m_turnout_template], side);
		item_tooltip("Keeps the start and the direction of the switch, takes the shape of the chosen template");
		if (slides)
		{
			ImGui::PushItemWidth(100.0f);
			ImGui::InputDouble("##switchslide", &tool.slide_typed, 0.0, 0.0, "%.2f m");
			ImGui::PopItemWidth();
			ImGui::SameLine();
			if (ImGui::Button(STR_C("Slide along the straight")))
				slide_switch(*track, std::clamp(tool.slide_typed, low, high - reach));
			if (ImGui::IsItemHovered())
				ImGui::SetTooltip("%s\n%s", STR_C("Moves the switch along its straight by the distance typed, + towards the frog; the straight paths on both sides get shorter or longer. Dragging the switch with the mouse does the same"),
				                  format(STR_C("Room on the straight: %.2f m back, %.2f m ahead"), -low, high - reach).c_str());
		}
		ImGui::TreePop();
	}
	ImGui::PopID();
	ImGui::Separator();
	ImGui::TextDisabled("%s", STR_C("The textures, the speed and the other properties of the switch are in the inspector"));
}

void editor_mode::render_path_events(TTrack &Track)
{
	auto *track{&Track};
	if (m_track_events_for != track && false == ImGui::IsAnyItemActive())
	{
		m_track_events_for = track;
		m_track_event[0] = '\0';
		m_track_events_missing.clear();
	}
	auto const count{[&]() {
		std::size_t count{0};
		for (int list = 0; list < 6; ++list)
			count += editor_track::events(*track, list).size();
		return count;
	}()};
	if (false == ImGui::CollapsingHeader(format("%s (%zu)###trackevents", STR_C("Events"), count).c_str()))
		return;
	auto const before{editor_track::capture(*track)};
	auto const changed = [&](editor_track::state const &Before) {
		push_track_snapshot({{track, Before}});
		track->mark_dirty();
	};
	ImGui::TextDisabled("%s", STR_C("What the track sets off as the vehicles run on it"));
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
				changed(before);
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
			changed(before);
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
	ImGui::TextDisabled("%s, %.2f m", STR_C(type_name(*track)), track->Length());
	if (track->eType == tt_Normal && false == track->m_paths.empty())
	{
		// the track at a glance: tiles of its length, radius, grade, cant and speed, going on to the next row at the edge
		auto const &path{track->m_paths.front()};
		geometry::bezier const curve{path};
		auto const length{curve.plan_length()};
		auto const first{curve.first(0.5)};
		auto const second{curve.second(0.5)};
		auto const speed{std::hypot(first.x, first.z)};
		auto const bend{speed > 1e-9 ? std::abs(first.x * second.z - first.z * second.x) / (speed * speed * speed) : 0.0};
		auto const straight{editor_track::is_straight(*track, m_straights.tolerance) || bend < 1e-5};
		auto const rise{path.points[segment_data::point::end].y - path.points[segment_data::point::start].y};
		auto const roll{std::max(std::abs(path.rolls[0]), std::abs(path.rolls[1]))};
		metric_tiles({{format("%.2f m", track->Length()), STR_C("length"), ImGui::GetColorU32(ImGuiCol_Text)},
		              {straight ? std::string{STR_C("straight")} : format("R %.0f m", 1.0 / bend), STR_C("plan"), straight ? IM_COL32(255, 255, 255, 230) : overlay_color::grip},
		              {format("%+.1f mm/m", length > 1e-6 ? rise / length * 1000.0 : 0.0), STR_C("grade"), ImGui::GetColorU32(ImGuiCol_Text)},
		              {format("%.0f mm", 1500.0 * std::sin(glm::radians(static_cast<double>(roll)))), STR_C("cant"), IM_COL32(240, 170, 60, 255)},
		              {editor_track::velocity(*track) > 0.0 ? format("%.0f km/h", editor_track::velocity(*track)) : std::string{"-"}, STR_C("speed"), ImGui::GetColorU32(ImGuiCol_Text)}});
	}
	std::string reason;
	bool const geometry = editor_track::can_edit_geometry(*track, reason);
	if (false == geometry)
		ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.2f, 1.0f), STR_C("Geometry locked: %s"), reason.c_str());
	if (track->eType == tt_Switch && m_track_tab == track_tab::path)
	{
		ImGui::SameLine();
		if (ImGui::SmallButton(STR_C("Edit the switch (T)")))
			show_track_tab(track_tab::turnout);
		item_tooltip("A switch: the turnout tool sets its geometry, side and drive");
	}
	std::vector<TTrack *> const tracks{track};
	render_track_select_tools(tracks);
	render_track_style(tracks);
	render_track_properties(tracks);
	render_path_events(*track);
	// the shape is given by dragging in the view, the numbers are for the exact values
	if (m_track_tab != track_tab::path)
		return;

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

	auto const finite = [](glm::dvec3 const &Point) { return std::isfinite(Point.x) && std::isfinite(Point.y) && std::isfinite(Point.z); };
	if (ImGui::CollapsingHeader(track->eType == tt_Switch ? STR_C("Geometry of the paths") : STR_C("Geometry")))
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
					if (ImGui::DragScalarN("x, y, z##point", ImGuiDataType_Double, glm::value_ptr(position), 3, 0.01f, nullptr, nullptr, "%.3f") && finite(position))
						editor_track::move_point(*track, m_track_point, position);
					item_tooltip("Drag to move, Ctrl+click to type the value");
					finish_edit(rebuild::geometry);
				}
				else
				{
					auto &vector{path.points[kind == editor_track::point_kind::control1 ? segment_data::point::control1 : segment_data::point::control2]};
					auto typed{vector};
					if (ImGui::DragScalarN(STR_C("vector##point"), ImGuiDataType_Double, glm::value_ptr(typed), 3, 0.01f, nullptr, nullptr, "%.3f") && finite(typed) && glm::length(typed) < 1e5)
						vector = typed;
					item_tooltip("Control vector, relative to its end; zero for a straight");
					finish_edit(rebuild::geometry);
				}
			}
			if (ImGui::TreeNode(STR_C("Cant and radius")))
			{
				if (begin_fields("##cantradius", 0.3f))
				{
					field_row("Cant (mm)");
					auto const lift = [](float const Roll) { return static_cast<float>(1500.0 * std::sin(glm::radians(static_cast<double>(Roll)))); };
					auto const roll = [](float const Lift) { return static_cast<float>(glm::degrees(std::asin(std::clamp(static_cast<double>(Lift) / 1500.0, -1.0, 1.0)))); };
					if (std::array<float, 2> cants{lift(path.rolls[0]), lift(path.rolls[1])}; ImGui::DragFloat2("##roll", cants.data(), 0.5f, -388.0f, 388.0f, "%.0f") && std::isfinite(cants[0]) && std::isfinite(cants[1]))
						path.rolls = {roll(cants[0]), roll(cants[1])};
					item_tooltip("Cant at the start and at the end: how much higher one rail is, + raises the left rail looking along the path");
					finish_edit(rebuild::geometry);
					field_row("Radius");
					if (auto radius{path.radius}; ImGui::InputFloat("##radius", &radius, 0.0f, 0.0f, "%.1f") && std::isfinite(radius))
						path.radius = radius;
					item_tooltip("Radius stored with the path in the scenery; the shape itself comes from the control vectors");
					finish_edit(rebuild::geometry);
					ImGui::EndTable();
				}
				if (ImGui::Button(STR_C("Straighten"), ImVec2(-FLT_MIN, 0.0f)))
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
			if (begin_fields("##snapfields", 0.45f))
			{
				field_row("Snap radius (m)");
				ImGui::DragFloat("##snapradius", &m_track_snap_radius, 0.05f, 0.0f, 50.0f, "%.2f");
				item_tooltip("A dragged free end within this distance of another free end joins it on release");
				ImGui::EndTable();
			}
			toggle_chips("snapflags", {{STR_C("Align with snapped path"), &m_track_align_tangent, "On joining, the end also takes over the direction of the other path, so the joint has no kink"},
			                           {STR_C("Drag connected ends"), &m_track_drag_connected, "Dragging a joined end, or moving and turning the whole path or switch,\nmoves the ends of the neighbouring paths with it"}});
			ImGui::TreePop();
		}
	}
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
			hints = {{"LMB", "start, on the ground or at a free end"}, {"Drag from a free end", "lays the shape to the cursor"}, {"Shift+click a straight", "a parallel track (N)"}, {"Alt", "the direction in steps of 15 degrees, or 1:9, 1:6.6... to a straight track"}, {"Esc", "stop laying"}};
		else
		{
			bool const chained{m_lay.start.track != nullptr};
			hints = {{"LMB", chained ? "lay the shape up to here" : "lay the straight up to here"}, {"LMB on a free end", "join and lay"}};
			if (chained || m_lay.mouse_snap.track != nullptr)
				hints.insert(hints.end(), {{"Ctrl+wheel", "radius"}, {"Shift+wheel", "another shape"}});
			hints.insert(hints.end(), {{"Alt", "the direction in steps of 15 degrees, or 1:9, 1:6.6... to a straight track"}, {"Enter", "lay up to the cursor"}, {"Backspace", m_lay.chain.empty() ? "take back the point" : "take back the last piece"}, {"Esc", "cancel"}});
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
		switch (current_switch_kind())
		{
		case switch_kind::connection: hints = {{"Press on a straight", "and drag onto the other track"}, {"Shift+wheel", "another kind"}, {"Ctrl+wheel", "R of the aligning curve"}}; break;
		case switch_kind::slip:
		case switch_kind::crossing: hints = {{"LMB", "at a crossing of two straights"}}; break;
		default: hints = {{"LMB on a track", "the straight or the curve for the switch"}}; break;
		}
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
		hints = {{"LMB on a track", "the grade line of its whole line"}, {"Shift+LMB on more tracks", "the route between the outermost ones"}, {"Shift+drag a box", "the route over the tracks in it"}};
	else if (m_track_tab == track_tab::profile)
		hints = {{"Drag a point", "changes the grade"}, {"Double click", "adds a point"}, {"LMB on another track", "its whole line"}, {"Shift+LMB on more tracks", "the route between the outermost ones"}, {"Shift+drag a box", "the route over the tracks in it"}, {"Apply", "writes the heights into the tracks"}};
	else if (m_track_window_open && m_track_tab == track_tab::speed)
		hints = {{"LMB on a row", "show the path"}};
	else if (m_track_window_open && m_track_tab == track_tab::lineside)
		hints = {{"LMB on a track", "the line the objects go along"}};
	else if (m_track_window_open && m_track_tab == track_tab::parallel)
		hints = {{"LMB on a track", "the new track goes beside it"}, {"Click on a side in the panel", "the side of the new track"}};
	else if (m_track_window_open && m_track_tab == track_tab::signals && m_signal.moving)
		hints = {{"Drag along", "moves the signal along the track"}, {"Drag over the track", "to the other side"}, {"Release", "done"}, {"Esc", "cancel"}};
	else if (m_track_window_open && m_track_tab == track_tab::signals && m_signal.placing)
		hints = {{"Drag along", "the trains it faces run that way"}, {"Release", "done"}, {"Esc", "cancel"}};
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
		hints = {{"Drag a vertex", "the curve follows"}, {"Drag the radius grip", "changes the radius"}, {"Drag in the curvature band", "R up and down, transitions sideways"}, {"Apply", "writes the design into the tracks"}, {"Esc", "release the vertex"}};
	else if (selected == nullptr)
		hints = {{"LMB on a track", "selects it"}, {"Drag on the ground", "selects the tracks in a box"}, {"Shift+LMB", "adds a track to the selection, drag: a box"}, {"RMB on a track", "what can be done with it"}, {"L", "lay a new track"}};
	else if (m_track_tab == track_tab::path && m_track_set.size() >= 2 && in_track_set(selected))
	{
		hints = {{"Inspector", "the properties of all of them at once"}, {"Gizmo", "moves them together"}, {"Shift+LMB", "adds or takes out a track"}, {"Ctrl+Shift+V", "pastes the copied properties"}, {"Esc", "deselects"}, {"Del", "deletes them"}};
		gizmo = true;
	}
	else if (m_point_drag.moved)
		hints = {{"Drag", "moves the point, level"}, {"Over a free end", "the end joins it"}, {"Release", "done"}, {"Esc", "cancel"}};
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
		hint.raw = hint.key;
		hint.key = STR_C(hint.key);
		hint.action = STR(hint.action);
	}
	return hints;
}

// values of the change in progress, shown next to the cursor
std::string editor_mode::track_readout() const
{
	if (m_tape.active)
		return tape_readout();
	auto const name = [](TTrack const *Track) { return name_of(*Track); };
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
		if (m_eased_last.has_value() && false == m_eased_last->pieces.empty())
			radius = m_eased_last->radius;
		editor_track::snap_target target;
		auto const snapped{extend_snap(target) && target.track != nullptr};
		editor_track::branch_link link;
		std::string reason;
		if (plan_branch_link(link, reason))
			return link_summary(target, link, editor_track::branch_radius(m_extend.track->m_paths[1]));
		if (snapped && pieces.empty())
			return format(STR_C("no track fits to the end of %s"), name(target.track).c_str());
		auto text{radius > 0.0 ? format("L %.2f m   R %.0f m", length, radius) + speed_of(radius, laying_norms()) : format("L %.2f m   %s", length, STR_C("straight"))};
		if (auto const eased{eased_of(m_eased_last)}; false == eased.empty())
			text += "\n" + eased;
		if (false == m_norms_free)
			text += "\n" + design_speed_of(m_extend.track, snapped ? target.track : nullptr);
		if (auto const vertical{vertical_of(pieces)}; false == vertical.empty())
			text += "\n" + vertical;
		if (snapped)
			text += "\n" + format(STR_C("joins %s"), name(target.track).c_str());
		if (false == reason.empty())
			text += "\n" + reason;
		return text + "\n" + track_shaping(snapped);
	}
	if (m_lay.active)
	{
		auto const &lay{m_lay};
		std::string text;
		if (lay.points.empty())
			text = lay.mouse_snap.track != nullptr ? format(STR_C("starts at the end of %s"), name(lay.mouse_snap.track).c_str()) : std::string{STR_C("start")};
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
			if (m_eased_last.has_value() && false == m_eased_last->pieces.empty())
				radius = m_eased_last->radius;
			if (radius > 0.0)
				text = (straight ? format("L %.1f m   %s + R %.0f m", lay.preview_length, STR_C("straight"), radius) : format("L %.1f m   R %.0f m", lay.preview_length, radius)) + speed_of(radius, laying_norms());
			else
				text = format("L %.1f m   %s", lay.preview_length, STR_C("straight"));
			if (auto const eased{eased_of(m_eased_last)}; false == eased.empty())
				text += "\n" + eased;
			if (false == m_norms_free)
			{
				auto const [from, to]{laying_ends()};
				text += "\n" + design_speed_of(from, to);
			}
			if (auto const vertical{vertical_of(lay.preview)}; false == vertical.empty())
				text += "\n" + vertical;
		}
		if (false == lay.points.empty() && lay.mouse_snap.track != nullptr)
			text += "\n" + format(STR_C("joins %s"), name(lay.mouse_snap.track).c_str());
		if (false == lay.preview_error.empty())
			text += "\n" + lay.preview_error;
		if (lay.points.size() == 1 && (lay.start.track != nullptr || lay.mouse_snap.track != nullptr))
			text += "\n" + track_shaping(lay.mouse_snap.track != nullptr && lay.start.track != nullptr);
		if (ImGui::GetIO().KeyAlt && false == lay.points.empty())
			text += "\n" + (lay.skew.empty() ? format(STR_C("azimuth %.0f deg"), azimuth(lay.points.back(), lay.mouse)) : lay.skew);
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
			return format(STR_C("turn %+.3f deg   R %.0f m"), turn, straight_tool_radius(line));
		}
		glm::dvec2 const normal{-line.direction.y, line.direction.x};
		return format(STR_C("shift %+.3f m   over %.2f m"), glm::dot(offset, normal), glm::dot(offset, line.direction));
	}
	if (state.dragging && state.fitting)
	{
		auto const &result{m_route.result};
		if (false == result.valid)
			return result.errors.empty() ? std::string{STR_C("no curve fits")} : result.errors.front();
		auto text{format(STR_C("curve L %.2f m"), result.length)};
		for (auto const &curve : result.curves)
			text += format(STR_C("   deflection %.3f deg"), glm::degrees(curve.deflection));
		return text;
	}
	if (state.dragging)
	{
		auto const &grabbed{state.drag_line};
		if (state.handle == 2)
		{
			glm::dvec2 const normal{-grabbed.direction.y, grabbed.direction.x};
			auto const offset{state.preview_start - grabbed.start};
			auto text{format(STR_C("shift %+.3f m"), glm::dot(plan_of(offset), normal))};
			auto const offer{straight_parallel_offer(grabbed, offset)};
			if (offer.snaps)
				text += "\n" + format(STR_C("parallel  %.2f m"), offer.spacing);
			else if (offer.nearby)
				text += "\n" + format(STR_C("close: %.2f m  %.1f°"), offer.spacing, offer.angle);
			return text;
		}
		return format(STR_C("L %.3f m (%+.3f)   azimuth %.4f deg"), plan_distance(state.preview_start, state.preview_end), plan_distance(state.preview_start, state.preview_end) - grabbed.length,
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
				text += "\n" + format(STR_C("joins %s"), name(m_track_snap.track).c_str());
			return text;
		}
		return format(STR_C("moved %.3f m"), glm::distance(editor_track::pivot(*track), m_track_pivot));
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
		return text + "\n" + STR("Ctrl+wheel: radius, Shift+wheel: arc / straight") + (m_norms_free ? std::string{} : STR(", Ctrl+Shift+wheel: speed"));
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
	return text + "\n" + STR("Ctrl+wheel: radius, Shift+wheel: another shape") + (m_norms_free ? std::string{} : STR(", Ctrl+Shift+wheel: speed"));
}

void editor_mode::draw_empty_scenery_hint()
{
	if (m_terrain_open || (m_track_tab != track_tab::path && m_track_tab != track_tab::lay) || (m_lay.active && false == m_lay.points.empty()) || m_first_steps_open || ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel))
		return;
	auto const &paths{simulation::Paths.sequence()};
	if (std::any_of(paths.begin(), paths.end(), [](TTrack const *Track) { return Track != nullptr && false == Track->m_editorremoved; }))
		return;
	auto *list{ImGui::GetBackgroundDrawList(ImGui::GetMainViewport())};
	auto const low{editor_ui::view_min()};
	auto const high{editor_ui::view_max()};
	auto const font{ImGui::GetFontSize()};
	auto const pad{std::round(font * 1.0f)};
	auto const picture{std::round(ImGui::GetTextLineHeight() * 1.3f)};
	auto const title{STR_C("The scenery has no tracks yet")};
	std::vector<std::pair<char const *, char const *>> rows;
	if (m_track_tab == track_tab::path)
		rows.emplace_back("L", "the lay tool");
	rows.emplace_back("LMB", "on the ground: the first point, then each next one lays a piece");
	rows.emplace_back("Ctrl+wheel", "the radius of the curve while laying");
	rows.emplace_back("F1", "all the shortcuts, and the tutorial");
	auto const width{std::min(high.x - low.x - 2.0f * pad, std::round(font * 30.0f))};
	auto const column{std::round(picture * 5.0f)};
	auto const wrap{width - 2.0f * pad - column};
	float height{pad + font * 2.4f + pad * 0.5f};
	for (auto const &row : rows)
		height += std::max(picture, ImGui::CalcTextSize(STR_C(row.second), nullptr, false, wrap).y) + pad * 0.45f;
	height += pad * 0.6f;
	ImVec2 const top{std::round((low.x + high.x - width) * 0.5f), std::round((low.y + high.y - height) * 0.5f)};
	ImVec2 const bottom{top.x + width, top.y + height};
	auto const accent{family_colour(0)};
	list->AddRectFilled(top, bottom, IM_COL32(16, 20, 26, 215), std::round(font * 0.6f));
	list->AddRect(top, bottom, (accent & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 160), std::round(font * 0.6f), 0, 1.5f);
	auto const icon{std::round(font * 2.4f)};
	editor_icons::draw(list, editor_icons::icon::lay_track, ImVec2(top.x + pad + icon * 0.5f, top.y + pad + icon * 0.5f), accent);
	auto const big{font * 1.25f};
	list->AddText(nullptr, big, ImVec2(top.x + pad + icon + pad * 0.6f, std::round(top.y + pad + (icon - big) * 0.5f)), ImGui::GetColorU32(ImGuiCol_Text), title);
	auto y{top.y + pad + icon + pad * 0.5f};
	for (auto const &row : rows)
	{
		auto const text{STR_C(row.second)};
		auto const size{ImGui::CalcTextSize(text, nullptr, false, wrap)};
		auto const line{std::max(picture, size.y)};
		gesture_draw(*list, ImVec2(top.x + pad, std::round(y + (line - picture) * 0.5f)), picture, gesture_picture(row.first), ImGui::GetColorU32(ImGuiCol_Text));
		list->AddText(nullptr, 0.0f, ImVec2(top.x + pad + column, std::round(y + (line - size.y) * 0.5f)), ImGui::GetColorU32(ImGuiCol_Text, 0.85f), text, nullptr, wrap);
		y += line + pad * 0.45f;
	}
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
	auto hints{track_key_hints(false)};
	auto const picture{std::round(ImGui::GetTextLineHeight() * 1.15f)};
	std::vector<std::vector<gesture_word>> pictures;
	for (auto const &hint : hints)
		pictures.push_back(gesture_picture(hint.raw != nullptr ? hint.raw : hint.key));
	auto const picture_width = [&](std::size_t const Index) { return pictures[Index].empty() ? 0.0f : gesture_width(picture, pictures[Index]) + 5.0f; };
	auto const width_of = [&](std::size_t const Index) {
		auto const chip_width{hints[Index].raw != nullptr && gesture_keys_only(hints[Index].raw) ? 0.0f : ImGui::CalcTextSize(hints[Index].key).x + 2.0f * pad + 5.0f};
		return picture_width(Index) + chip_width + ImGui::CalcTextSize(hints[Index].action.c_str()).x;
	};

	auto const mode{track_mode_name()};
	auto const left{editor_ui::view_min().x + margin};
	auto const available{editor_ui::view_max().x - editor_ui::view_min().x - 2.0f * margin};
	float width{ImGui::CalcTextSize(mode.c_str()).x + 2.0f * pad};
	std::size_t shown{0};
	for (; shown < std::min<std::size_t>(hints.size(), 3); ++shown)
	{
		auto const next{width + gap + width_of(shown)};
		if (next > available)
			break;
		width = next;
	}
	auto const height{ImGui::GetTextLineHeight()};
	auto const bottom{m_profile.strip_top > editor_ui::view_min().y + 4.0f * height ? std::min(editor_ui::view_max().y, m_profile.strip_top) : editor_ui::view_max().y};
	auto const y{bottom - margin - height};
	ImVec2 const from{left - 6.0f, y - pad - 2.0f};
	ImVec2 const to{left + width + 6.0f, y + height + pad};
	auto const hovered{io.MousePos.x > from.x - 40.0f && io.MousePos.x < to.x + 40.0f && io.MousePos.y > from.y - 40.0f && io.MousePos.y < to.y + 40.0f};
	auto const fade = [hovered](ImU32 const Colour) { return hovered ? (Colour & ~IM_COL32_A_MASK) | (((Colour >> IM_COL32_A_SHIFT) & 0xFF) / 4 << IM_COL32_A_SHIFT) : Colour; };
	drawlist->AddRectFilled(from, to, fade(IM_COL32(0, 0, 0, 160)), 4.0f);
	auto x{left};
	auto const &modes{track_modes()};
	auto const tab{false == m_track_window_open ? track_tab::path : m_track_tab == track_tab::route ? track_tab::straights : m_track_tab};
	auto const found{std::find_if(modes.begin(), modes.end(), [&](track_mode const &Mode) { return Mode.tab == tab; })};
	auto const family{(family_colour(found != modes.end() ? found->family : 0) & ~IM_COL32_A_MASK) | IM_COL32(0, 0, 0, 235)};
	x += chip(ImVec2(x, y), mode.c_str(), fade(family), fade(IM_COL32(255, 255, 255, 255)));
	for (std::size_t i = 0; i < shown; ++i)
	{
		x += gap;
		if (false == pictures[i].empty())
			x += gesture_draw(*drawlist, ImVec2(x, std::round(y + (height - picture) * 0.5f)), picture, pictures[i], fade(IM_COL32(255, 255, 255, 230))) + 5.0f;
		if (hints[i].raw == nullptr || false == gesture_keys_only(hints[i].raw))
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
	// in the 3D view only near the camera, where they can be told apart; from above all of them
	glm::dvec3 const eye{Global.pCamera.Pos};
	auto const in_reach = [&](glm::dvec3 const &Point) { return Global.EditorOrtho || glm::distance(Point, eye) <= kDotsRange; };
	for (auto const *track : simulation::Paths.sequence())
	{
		if (track == nullptr || track->m_editorremoved || track->m_paths.empty())
			continue;
		for (auto const &path : track->m_paths)
			for (auto const point : {segment_data::point::start, segment_data::point::end})
			{
				ImVec2 screen;
				if (in_reach(path.points[point]) && projection.project(path.points[point], screen) && screen.x >= 0.0f && screen.y >= 0.0f && screen.x <= display.x && screen.y <= display.y)
					drawlist->AddCircleFilled(screen, 3.5f, kJointColor);
			}
	}
	// the ends joined to nothing, where a train would run off
	for (auto const &point : free_end_points(eye, Global.EditorOrtho ? std::max(kDotsRange, static_cast<double>(Global.EditorOrthoExtent) * 2.0) : kDotsRange))
	{
		ImVec2 screen;
		if (in_reach(point) && projection.project(point, screen) && screen.x >= 0.0f && screen.y >= 0.0f && screen.x <= display.x && screen.y <= display.y)
		{
			drawlist->AddCircleFilled(screen, 5.0f, kFreeEndColor);
			drawlist->AddCircle(screen, 5.0f, IM_COL32(0, 0, 0, 160), 12, 1.0f);
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
	else if (auto const *selected{selected_track()}; false == m_switch.placing && m_track_tab == track_tab::turnout && m_switch.connection < 0 && m_switch.armed >= 0 && m_switch.armed < static_cast<int>(m_switch.templates.size()) &&
	                                                 m_switch.templates[m_switch.armed].double_slip && selected != nullptr && selected == m_straights.current_for && false == m_straights.current.tracks.empty() &&
	                                                 false == ImGui::GetIO().WantCaptureMouse)
	{
		// where the slip goes for a click there, before the click
		auto const &line{m_straights.current};
		auto const cursor{cursor_on_straight(line)};
		auto const along{std::clamp(line.along(cursor), 0.0, line.length)};
		if (plan_distance(line.start + glm::dvec3{line.direction.x, line.grade, line.direction.y} * along, cursor) <= kSwitchReach)
		{
			slip_plan plan;
			auto const valid{plan_double_slip(line, cursor, m_switch.templates[m_switch.armed], plan)};
			std::string text{plan.problem};
			glm::dvec3 marker{cursor};
			ImU32 colour{overlay_color::invalid};
			if (valid && plan.replaces != nullptr)
			{
				marker = plan.replaces->location();
				colour = overlay_color::highlight;
				text = STR_C("Click: this double slip is replaced by the one chosen");
			}
			else if (valid)
			{
				for (int i = 0; i < 4; i += 2)
				{
					segment_data path;
					path.points[segment_data::point::start] = plan.points[i];
					path.points[segment_data::point::end] = plan.points[i + 1];
					drawpath(path, overlay_color::marked);
				}
				marker = {plan.crossing.x, line.start.y + line.grade * plan.alongfirst, plan.crossing.y};
				colour = overlay_color::highlight;
				text = plan.note.empty() ? std::string{STR_C("Click: it goes here")} : plan.note;
			}
			ImVec2 screen;
			if (projection.project(marker, screen))
			{
				drawlist->AddCircle(screen, 12.0f, colour, 20, 2.5f);
				drawlist->AddText({screen.x + 16.0f, screen.y - 8.0f}, colour, text.c_str());
			}
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
