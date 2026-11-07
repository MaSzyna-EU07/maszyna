/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editormodeinfrastate.h"
#include "application/editormodelaystate.h"
#include "application/editormodenmtstate.h"
#include "application/editormodeprofilestate.h"
#include "application/editormoderoadstate.h"
#include "application/editormoderoutestate.h"
#include "application/editormodestraightsstate.h"
#include "application/editormodesweepstate.h"
#include "editor/editorTerrainStreamer.hpp"
#include "scene/scenenode.h"
#include "vehicle/Camera.h"

#include <cstddef>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class editor_ui;
class TTrack;

// what the road window needs from the editor: its state, the ground under the roads, its history and the material fields
class editor_road_services
{
  public:
	virtual road_tool &road_state() = 0;
	// height of the ground under each of the points, with the roads not taken for the ground. a point with nothing under it keeps
	// its height. Found: set for the points which have the ground under them, nullptr for none
	virtual std::vector<double> ground_heights_at(std::vector<glm::dvec3> const &Points, bool const Fresh, std::vector<char> *Found) const = 0;
	// the changes of the roads, the junctions and the points: one step of the history
	virtual void push_road_snapshot(editor_road::record Record) = 0;
	// a field of the material of a part of a road or of a junction. returns: true if the material was changed
	virtual bool render_road_material(char const *Label, std::string &Value) = 0;

  protected:
	~editor_road_services() = default;
};

// what the models laid and bent along the track need from the editor: the state of the one laid, its curve, the steps of the history
// and the model or include selected to be bent
class editor_sweep_services
{
  public:
	virtual sweep_tool &sweep_state() = 0;
	// the curve the model is laid along: the axis of the selected track, once a step of the history
	virtual void sweep_curve() = 0;
	// a model laid along the track made or taken away: one step of the history
	virtual void push_sweep_toggled(sweep_node &Sweep) = 0;
	// a model laid along the track changed from the state Before: one step of the history
	virtual void push_sweep_change(sweep_node &Sweep, sweep_node::state Before) = 0;
	// the model bent made of the source, which is taken out of the scenery: one step of the history
	virtual void push_bend_made(sweep_node &Sweep, bend_source const &Source) = 0;
	// the model or the include selected, as a model to be bent. returns: false if there's none to bend
	virtual bool bend_source_of_selection(bend_source &Source) const = 0;

  protected:
	~editor_sweep_services() = default;
};

// what the vertical profile and the grade lines need from the editor: their state and the terrain model's, the ground and its
// streamer, the camera, and the steps of the history they join or take the kept grade lines into
class editor_profile_services
{
  public:
	virtual profile_state &vertical_profile() = 0;
	// the grade lines kept in the scenery files, and whether they were read from them yet
	virtual std::vector<stored_profile> &stored_profiles() = 0;
	virtual bool &profiles_loaded() = 0;
	virtual nmt_state &terrain_state() = 0;
	virtual terrain_streamer &ground_streamer() = 0;
	// point spacing of the chunks of the terrain made
	virtual float const &terrain_spacing() const = 0;
	// the steps of the history which changed tracks, counted as they're made
	virtual int const &track_edits() const = 0;
	// the last step takes the kept grade lines back to the library given, as they were before it changed them
	virtual void profiles_to_history(std::vector<stored_profile> Library) = 0;
	// the same for the step just made, whatever it is
	virtual void history_take_profiles(std::vector<stored_profile> Library) = 0;
	// the tracks changed by the last Edits steps of the history which edited tracks, each once
	virtual std::vector<TTrack *> tracks_edited(std::size_t const Edits) const = 0;
	// the terrain changed as a step of the history, joined to the step before it unless that one changed the terrain too
	virtual void terrain_step_joined() = 0;
	virtual TCamera &view_camera() = 0;
	// the camera no longer flies to what it was focused on
	virtual void release_focus() = 0;
	// forgets what's known about the ground, after the terrain changed
	virtual void forget_ground_cache() = 0;
	// opens the heightmap terrain of the scenery for editing, made if the scenery has none yet. returns: true if it's open
	virtual bool terrain_ready() = 0;
	// opens or leaves the terrain
	virtual void open_terrain_workspace(bool const Open) = 0;
	// the paths of the tracks shaped anew on the terrain, Only those if given, all of them for nullptr; the number of the modifiers laid
	virtual std::size_t shape_tracks_on_terrain(std::vector<TTrack *> const *Only) = 0;
	// the last step of the history undone together with the one before, or its includes put into that one
	virtual void join_last_history_steps() = 0;
	// the set when there's one, the selected track otherwise
	virtual std::vector<TTrack *> selected_track_list() = 0;
	// the track the grade line pick aims at, Index its end; nullptr when the cursor aims at none for it
	virtual TTrack *grade_line_pick_aimed(int &Index) const = 0;
	// whether the Tracks may be changed; Reason: why not
	virtual bool tracks_editable_for(std::vector<TTrack *> const &Tracks, std::string &Reason) const = 0;
	// the crossovers between the tracks of the route sampled Before and their neighbours led after the route changed; how many
	virtual std::size_t follow_crossovers_of(std::vector<editor_track::route_sample> const &Before) = 0;
	// the window converting the ground files of the scenery to the heightmap terrain
	virtual void render_ground_conversion_panel() = 0;

  protected:
	~editor_profile_services() = default;
};

// what the build tools (laying track, routes, straights, roads, sweeps, vertical profiles and grade lines, the terrain model, connections and the infrastructure) need from the editor beyond the interfaces of editortool.h; editor_mode implements it
class editor_build_services : public editor_road_services, public editor_sweep_services, public editor_profile_services
{
  public:
	// the panels of the editor
	virtual editor_ui *build_ui() const = 0;
	// the node selected, nullptr for none, and the include selected, 0 for none
	virtual scene::basic_node *current_node() const = 0;
	virtual scene::instance_handle current_instance() const = 0;
	// nothing selected any more: no node, no include, no drag of the node
	virtual void drop_node_selection() = 0;
	virtual bool right_button_held() const = 0;

  protected:
	~editor_build_services() = default;
};
