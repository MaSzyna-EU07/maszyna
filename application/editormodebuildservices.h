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
#include "editor/editorGeoMap.hpp"
#include "editor/editorOrthophoto.hpp"
#include "editor/editorTerrainStreamer.hpp"
#include "scene/scenenode.h"
#include "vehicle/Camera.h"

#include <cstddef>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class editor_ui;
class TTrack;
class TAnimModel;

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
	// the window converting the ground files of the scenery to the heightmap terrain
	virtual void render_ground_conversion_panel() = 0;

  protected:
	~editor_profile_services() = default;
};

// what the terrain model and the download area need from the editor: the imagery, the map, the state of the terrain
// and of the laying, the camera and the steps of the history
class editor_nmt_services
{
  public:
	// the orthophoto of the scenery and the layers of the imagery over it
	virtual editor_orthophoto &orthophoto() = 0;
	virtual std::vector<std::unique_ptr<editor_orthophoto>> &imagery_layers() = 0;
	// the map the place of the terrain model is picked on
	virtual editor_map &terrain_map() = 0;
	virtual area_drawing &area_state() = 0;
	// the download of the area held by the button
	virtual bool &area_paused() = 0;
	// the track laid through the points clicked, and the free end lengthened
	virtual lay_tool &lay_state() = 0;
	virtual extend_tool &extend_state() = 0;
	// the heightmap terrain: chunks along a side added around the camera, whether it's the field of work, the height of the
	// // chunks made where there's no ground
	virtual int &terrain_add_count() = 0;
	virtual bool &terrain_open() = 0;
	virtual float &terrain_base_height() = 0;
	// the orthophoto chosen by hand, and its origin being typed in (northing, easting)
	virtual bool &orthophoto_chosen() = 0;
	virtual glm::dvec2 &orthophoto_origin_edit() = 0;
	// the terrain taken into the history before the tracks shape it; what the last step of the history keeps of it
	virtual terrain_streamer::change const *terrain_step_marked() = 0;
	// the terrain the tracks shaped taken into the history, undone with the change of the tracks: unless the last step still
	// // keeps the Top marked before
	virtual void terrain_step_join_tracks(terrain_streamer::change const *Top) = 0;
	// the change of the terrain made so far, a step of the history
	virtual void terrain_step() = 0;
	// the tracks joined to the Tracks added to them by the rules of the spread, all of them selected; the set
	virtual std::vector<TTrack *> const &select_grown_track_set(std::vector<TTrack *> const &Tracks) = 0;
	// the tracks the steps of the history from the one given on made, in the order they were made
	virtual std::vector<TTrack const *> tracks_made_since(std::size_t const From) const = 0;
	// the terrain is the field of work
	virtual bool terrain_is_work_area() const = 0;
	// the steps of the history; the last one, or each from the one given on, undone together with the one before
	virtual std::size_t history_size() const = 0;
	virtual void join_last_step() = 0;
	virtual void join_history_from(std::size_t const Step) = 0;
	// the model moved up by Rise, a step of the history undone with the one before
	virtual void raise_model_joined(TAnimModel &Model, double const Rise) = 0;
	// forgets what's known about the ground within the bounds, after the terrain there changed
	virtual void forget_ground_area(glm::dvec2 const &Min, glm::dvec2 const &Max) = 0;
	// the tracks selected as the set, or the one track selected
	virtual void select_track_list(std::vector<TTrack *> const &Tracks) = 0;
	virtual void select_track_node(scene::basic_node *Node) = 0;
	// the view switched between the perspective and the one from above
	virtual void toggle_orthographic() = 0;
	// the settings of the orthophoto and of the imagery: whether they're open, and kept
	virtual bool orthophoto_settings_shown() const = 0;
	virtual void save_imagery_settings() = 0;
	// the point of the ground under the cursor, or of the level of the sea where there's none
	virtual glm::dvec3 cursor_ground_point() const = 0;

  protected:
	~editor_nmt_services() = default;
};

// what the designed line, its crossovers and the digitization need from the editor: the state of the line and of the straights,
// the records, the crossovers and the objects along the tracks
class editor_route_services
{
  public:
	// the line designed, and the straights of the paths
	virtual route_design &route_state() = 0;
	virtual straights_state &straight_lines() = 0;
	// the gizmo of a vertex of the line is dragged
	virtual bool &route_dragging() = 0;
	// the objects along the tracks and their bindings
	virtual infra_window &infra_state() = 0;
	// the result of the line computed anew from its design
	virtual void route_recompute() = 0;
	// the norms the track is laid to
	virtual alignment::limits lay_norms() const = 0;
	// the last step of the history undone
	virtual void undo_last_step() = 0;
	// the reason a change was refused shown in the status; the text
	virtual std::string refusal(std::string Text) = 0;
	// the records of the switches changed taken into the last step of the history
	virtual void records_into_history() = 0;
	// the switch recorded with the shape given
	virtual void record_switch_shape(TTrack const &Track, editor_track::switch_template const &Shape) const = 0;
	// the single crossover the switch is a part of, along a curve or straights
	virtual std::optional<curved_crossover> crossover_at(TTrack const &Switch) const = 0;
	// a crossover of the shape laid between the points pressed and released
	virtual bool lay_crossover_at(int const Shape, glm::dvec3 const &Press, glm::dvec3 const &Release, double const Reach, std::vector<std::string> const &Names) = 0;
	// the objects along the tracks: how many the infra tool bound, the line of the status on those the tracks moved, the
	// // objects recognised and the chosen ones bound
	virtual int infra_bound_total() const = 0;
	virtual std::string infra_carried_status() = 0;
	virtual void infra_recognise_objects() = 0;
	virtual void infra_bind_chosen_objects() = 0;
	// the point under the cursor on the level of the height given
	virtual glm::dvec3 cursor_at_level(double const Height) const = 0;
	// a step of the drag of a handle. Ground: the handle goes onto the ground, at the height of the ballast of the track, unless
	// // Shift is held
	virtual bool handle_drag_moved(glm::dvec3 &Position, TTrack const *Ground) = 0;
	// the crossover taken out of the tracks
	virtual bool take_out_crossover_at(curved_crossover const &Crossover, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Created, std::vector<TTrack *> &Removed, TTrack *&First, TTrack *&Second) = 0;
	// the template a crossover was laid of, -1 if it's not among the templates
	virtual int crossover_template_of(switch_store::record const &Record) = 0;
	// the straights beside the one shown found anew
	virtual void neighbour_straights_found() = 0;
	// the states of the objects along the tracks before the change in progress, and the ones the last step of the history keeps
	// // when it is an edit of the tracks; nullptr when it isn't
	virtual std::vector<infra::object_state> &infra_states_before() = 0;
	virtual std::vector<infra::object_state> const *track_edit_infra() const = 0;
	// the path the template of a signal of the file reads it by, none for a file of no signal template
	virtual std::optional<std::string> signal_read_of(std::string const &File) const = 0;
	// the directive of the include written anew
	virtual void include_directive_set(scene::instance_handle Instance, std::string const &Directive) = 0;

  protected:
	~editor_route_services() = default;
};

// what the straights of the paths need from the editor: the camera brought to a track, the drag of the tracks and the tab
// of the track window
class editor_straights_services
{
  public:
	// the camera flies smoothly to the node
	virtual void focus_on_node(scene::basic_node *Node) = 0;
	// the tracks dragged now; marked moved, and the drag committed, Force: for good
	virtual std::vector<TTrack *> &tracks_dragged() = 0;
	virtual void track_drag_moved() = 0;
	virtual void track_drag_commit(bool const Force) = 0;
	// the tab of the track window set, nothing else done
	virtual void track_tab_set(editor_track_tab const Tab) = 0;
	// the numbers typed during the gesture, and the turn of the straight tool set from them
	virtual std::vector<double> typed_numbers() const = 0;
	virtual void typed_turn_taken() = 0;

  protected:
	~editor_straights_services() = default;
};

// what laying the track through the points clicked and lengthening a free end need from the editor: the ground the pieces
// follow, the free ends snapped to, the switches placed at the end of a line
class editor_lay_services
{
  public:
	// the pieces of the preview follow the ground meanwhile, on the thread, not to the end
	virtual bool following_preview() const = 0;
	virtual void follow_preview_set(bool const Following) = 0;
	// the speed the line is designed for between the tracks it goes from and to
	virtual double lay_design_speed(TTrack const *From, TTrack const *To) const = 0;
	// the radius and the variant of the curve joining a free end chosen by the wheel, 0 for the radius found by itself, and
	// // the join worked out last
	virtual double &join_radius_chosen() = 0;
	virtual int &join_variant_chosen() = 0;
	virtual alignment::join_result &join_result_last() = 0;
	// the line with its corners eased worked out last
	virtual std::optional<alignment::lay_result> &eased_result_last() = 0;
	// the value typed during a gesture, taken instead of the one the cursor gives
	virtual std::string &typed_text() = 0;
	// the shape dragged from a free end, as the mode of laying has it
	virtual editor_track::layout &laying_layout() = 0;
	// the angle of the kink the joints of the rails may have
	virtual double joint_kink() const = 0;
	// the track and the point the cursor joins, the radius and the variant chosen for another one starting over
	virtual void join_target_aimed(editor_track::snap_target const &Target) = 0;
	// the first steps opened at the lesson of the tutorial of this title
	virtual void tutorial_open_at(char const *Lesson) = 0;
	// the point of the selected track chosen
	virtual editor_track::point_ref &track_point_chosen() = 0;
	// the point under the cursor on the terrain, or on the level of the height given off it
	virtual glm::dvec3 cursor_on_terrain(double const Height) const = 0;
	// the free end the point near the cursor snaps to, of a track of the category and not of the ones excluded
	virtual editor_track::snap_target free_end_snapped(glm::dvec3 const &Near, glm::vec2 const &Screen, TTrack const *Self, int const Category, std::vector<TTrack const *> const &Exclude) const = 0;
	// the free end of the path as the start of a new track; false if it isn't free or has no direction
	virtual bool free_end_of(TTrack &Track, int const Path, bool const Atend, extend_tool &Tool) const = 0;
	// the height of the bed of a track of the style at the point: on the ballast over the ground, and the point taken there
	virtual double bed_height_at(glm::dvec3 const &Point, glm::dvec2 const &Direction, TTrack const *Style) const = 0;
	virtual glm::dvec3 lay_bed_point(glm::dvec3 const &Point) const = 0;
	// the point of the line laid the value typed gives instead of the cursor
	virtual glm::dvec3 typed_lay_point(glm::dvec3 const &Mouse) const = 0;
	// the heights of the pieces laid over the ground between the heights and the grades at their ends
	virtual void ground_followed(std::vector<segment_data> &Pieces, double const Height0, double const Grade0, double const Height1, double const Grade1, TTrack const *Style) const = 0;
	// the track dragged from a switch onto another switch as the link of the two, and the link laid
	virtual bool branch_link_planned(editor_track::branch_link &Link, std::string &Reason) const = 0;
	virtual void branch_link_finished(editor_track::branch_link const &Link) = 0;
	// the second switch of a crossover placed by the track dragged on from a switch, when it plans one; false when it doesn't
	virtual bool crossover_laid_from_extend() = 0;
	// the track under the cursor, only the one given if not null, within the pixels given
	virtual track_hover track_hovered(TTrack const *Only, float const Pixels) const = 0;
	// the preview of the texture of a track of the kind, and the texture picked from the previews
	virtual std::uint64_t texture_image_of(int const Kind, std::string const &Name) = 0;
	virtual bool texture_picked(int const Kind, std::string const &Current, bool const Mixed, std::string &Chosen) = 0;
	// the name of a material dropped onto the field just drawn
	virtual bool material_dropped(std::string &Name) const = 0;

  protected:
	~editor_lay_services() = default;
};

// what the build tools (laying track, routes, straights, roads, sweeps, vertical profiles and grade lines, the terrain model, connections and the infrastructure) need from the editor beyond the interfaces of editortool.h; editor_mode implements it
class editor_build_services : public editor_road_services, public editor_sweep_services, public editor_profile_services, public editor_nmt_services, public editor_route_services, public editor_straights_services, public editor_lay_services
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
