/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/editormodebuildtools.h"
#include "application/editoruilayer.h"
#include "model/AnimModel.h"
#include "scene/scenelayers.h"

namespace
{

// the tracks changed by the last Edits steps of the History which changed tracks, those of the Kind of action, each once
template <typename Steps, typename Action>
std::vector<TTrack *> tracks_of_edits(Steps const &History, std::size_t const Edits, Action const Kind)
{
	std::vector<TTrack *> tracks;
	std::size_t found{0};
	for (auto step{History.rbegin()}; step != History.rend() && found < Edits; ++step)
	{
		if (step->action != Kind)
			continue;
		++found;
		for (auto const &[track, step_state] : step->tracks)
			tracks.push_back(track);
		tracks.insert(tracks.end(), step->created.begin(), step->created.end());
		tracks.insert(tracks.end(), step->removed.begin(), step->removed.end());
	}
	std::erase(tracks, nullptr);
	std::ranges::sort(tracks);
	tracks.erase(std::ranges::unique(tracks).begin(), tracks.end());
	return tracks;
}

} // namespace

void editor_mode::make_build_tools()
{
	editor_services &services{*this};
	editor_build_services &build{*this};
	m_build_tools = std::make_unique<build_tools>(services, build);
	m_tools.push_back(&m_build_tools->sweep);
	m_tools.push_back(&m_build_tools->bend);
	// nothing selected in the road window at the start
	m_build_tools->road.road_select(nullptr);
}

road_tool &editor_mode::road_state()
{
	return m_roadtool;
}

std::vector<double> editor_mode::ground_heights_at(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found) const
{
	return ground_heights(Points, Fresh, Found);
}

sweep_tool &editor_mode::sweep_state()
{
	return m_sweep;
}

void editor_mode::push_sweep_toggled(sweep_node &Sweep)
{
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = Sweep.name();
	snap.sweeps_toggled.push_back(&Sweep);
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::push_sweep_change(sweep_node &Sweep, sweep_node::state Before)
{
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Other;
	snap.node_name = Sweep.name();
	snap.sweeps.emplace_back(&Sweep, std::move(Before));
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

void editor_mode::push_bend_made(sweep_node &Sweep, bend_source const &Source)
{
	EditorSnapshot snap;
	snap.action = EditorSnapshot::Action::Bend;
	snap.sweeps_toggled.push_back(&Sweep);
	snap.layer = Source.layer;
	if (Source.model != nullptr)
	{
		auto *model{Source.model};
		model->export_as_text(snap.serialized);
		snap.node_name = model->name();
		snap.position = model->location();
		snap.rotation = model->Angles();
		snap.scale = model->Scale();
		snap.uuid = model->uuid;
		m_node = nullptr;
		m_dragging = false;
		ui()->set_node(nullptr);
		erase_model(model);
	}
	else
	{
		snap.instance = Source.instance;
		snap.node_name = Source.file;
		scene::Layers.removed(Source.instance, true);
		select_include(0);
	}
	trim_history();
	m_history.push_back(std::move(snap));
	g_redo.clear();
}

editor_ui *editor_mode::build_ui() const
{
	return ui();
}

scene::basic_node *editor_mode::current_node() const
{
	return m_node;
}

scene::instance_handle editor_mode::current_instance() const
{
	return m_instance;
}

void editor_mode::drop_node_selection()
{
	m_node = nullptr;
	m_dragging = false;
	select_include(0);
	ui()->set_node(nullptr);
}

bool editor_mode::right_button_held() const
{
	return m_input.mouse.button(GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
}

profile_state &editor_mode::vertical_profile()
{
	return m_profile;
}

std::vector<stored_profile> &editor_mode::stored_profiles()
{
	return m_profiles;
}

bool &editor_mode::profiles_loaded()
{
	return m_profiles_loaded;
}

nmt_state &editor_mode::terrain_state()
{
	return m_nmt_state;
}

terrain_streamer &editor_mode::ground_streamer()
{
	return m_streamer;
}

float const &editor_mode::terrain_spacing() const
{
	return m_terrain_spacing;
}

int const &editor_mode::track_edits() const
{
	return m_track_edits;
}

void editor_mode::history_take_profiles(std::vector<stored_profile> Library)
{
	m_history.back().profiles = true;
	m_history.back().profile_library = std::move(Library);
}

std::vector<TTrack *> editor_mode::tracks_edited(std::size_t const Edits) const
{
	return tracks_of_edits(m_history, Edits, EditorSnapshot::Action::TrackEdit);
}

void editor_mode::terrain_step_joined()
{
	auto const before{m_history.empty() ? EditorSnapshot::Action::Other : m_history.back().action};
	terrain_history_step();
	if (before != EditorSnapshot::Action::Terrain && false == m_history.empty() && m_history.back().action == EditorSnapshot::Action::Terrain)
		m_history.back().joined = true;
}

TCamera &editor_mode::view_camera()
{
	return Camera;
}

void editor_mode::release_focus()
{
	m_focus_active = false;
}

void editor_mode::forget_ground_cache()
{
	forget_ground();
}

bool editor_mode::terrain_ready()
{
	return ensure_terrain();
}

void editor_mode::open_terrain_workspace(bool const Open)
{
	terrain_workspace(Open);
}

std::size_t editor_mode::shape_tracks_on_terrain(std::vector<TTrack *> const *Only)
{
	return nmt_shape_tracks(Only);
}

void editor_mode::join_last_history_steps()
{
	join_last_steps();
}

std::vector<TTrack *> editor_mode::selected_track_list()
{
	return selected_tracks();
}

TTrack *editor_mode::grade_line_pick_aimed(int &Index) const
{
	if (m_intent.what != track_intent::kind::grade_line_pick || m_intent.track == nullptr)
		return nullptr;
	Index = m_intent.index;
	return m_intent.track;
}

bool editor_mode::tracks_editable_for(std::vector<TTrack *> const &Tracks, std::string &Reason) const
{
	return tracks_editable(Tracks, Reason);
}

std::size_t editor_mode::follow_crossovers_of(std::vector<editor_track::route_sample> const &Before)
{
	return follow_crossovers(Before);
}

void editor_mode::render_ground_conversion_panel()
{
	render_ground_conversion();
}
