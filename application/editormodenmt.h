/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editortool.h"
#include "application/editormodebuildservices.h"
#include "imgui/imgui.h"

#include <cstddef>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class nmt_builder;
class nmt_panel;
class download_area_tool;
class nmt_auto_fit;

// the terrain model under the tracks: fitted to them, made into the heightmap terrain around them as they grow, and following their changes
class nmt_builder final : public editor_tool
{
  public:
	nmt_builder(editor_services &Services, editor_build_services &Build, nmt_panel &Nmt_window, download_area_tool &Download_area, nmt_auto_fit &Nmt_auto) : m_services{Services}, m_build{Build}, m_nmt_window{Nmt_window}, m_download_area{Download_area}, m_nmt_auto{Nmt_auto}, m_profile{Build.vertical_profile()}, m_nmt_state{Build.terrain_state()}, m_nmt{Services.terrain_model()}, m_streamer{Build.ground_streamer()}, m_terrain_spacing{Build.terrain_spacing()}, m_orthophoto{Build.orthophoto()}, m_wms{Build.imagery_layers()}, m_lay{Build.lay_state()}, m_extend{Build.extend_state()}, m_terrain_add_count{Build.terrain_add_count()}, m_orthophoto_chosen{Build.orthophoto_chosen()}, m_terrain_baseheight{Build.terrain_base_height()} {}
	// the download area the terrain model was given
	std::vector<glm::dvec2> const &nmt_prefetched() const { return m_nmt_prefetched; }
	std::vector<nmt_place> nmt_track_places(double const Step) const;
	void nmt_locate();
	void nmt_fit();
	// the terrain under the tracks, the tracks left as they are: the model drawn to each of them, its relief kept
	void nmt_under_start();
	std::vector<heightmap::chunk_key> nmt_keys_around() const;
	std::vector<heightmap::chunk_key> nmt_keys_along_tracks() const;
	// only the chunks there are none of yet: a chunk there is, made from the model or by hand, edited or not, is never made anew.
	// Reshape: the formations of the tracks laid anew as the tracks lie now, as when they moved
	void nmt_make(std::vector<heightmap::chunk_key> const &Keys, bool const Reshape = false);
	// the squares of the model under the rectangle are in, none of them waiting to be asked for again
	bool nmt_ready(glm::dvec2 const &Min, glm::dvec2 const &Max);
	// the terrain beside the formation of the track is the one made from the model; Waiting: its squares of the model aren't in yet
	// the tracks changed by hand are told to the terrain: their formations laid anew, of the tracks removed taken away, a
	// track laid onto the terrain of the model given its own, once the mouse lets go; in the step of the history of the change
	void nmt_track_changed(std::vector<TTrack *> const &Tracks);
	void nmt_track_gone(TTrack const &Track);
	// the same, else the height of the model with the offset of the scenery where its square is in: the ground where there's no
	// terrain made yet, as under the base map of the top view
	bool nmt_ground(double const X, double const Z, double &Height) const;
	// Only: the paths of these tracks are shaped anew, the modifiers of those removed taken away; null: all of them. the number
	// of the modifiers laid
	std::size_t nmt_shape_tracks(std::vector<TTrack *> const *Only = nullptr);
	void nmt_update();
	void nmt_grow();
	bool nmt_busy() const;
	// the terrain made so far stays, what's waiting is dropped, the grade lines not fitted yet are left
	void nmt_stop();
	// the formations of the tracks led into the terrain, down or up to it, or taken out of it; one step of the history each
	void nmt_pull_terrain(std::vector<TTrack *> const &Tracks);
	void nmt_restore_terrain(std::span<TTrack *const> const Tracks);

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	nmt_panel &m_nmt_window;
	download_area_tool &m_download_area;
	nmt_auto_fit &m_nmt_auto;
	profile_state &m_profile;
	nmt_state &m_nmt_state;
	editor_nmt &m_nmt;
	terrain_streamer &m_streamer;
	float const &m_terrain_spacing;
	editor_orthophoto &m_orthophoto;
	std::vector<std::unique_ptr<editor_orthophoto>> &m_wms;
	lay_tool &m_lay;
	extend_tool &m_extend;
	int &m_terrain_add_count;
	bool &m_orthophoto_chosen;
	float &m_terrain_baseheight;
	std::vector<glm::dvec2> m_nmt_prefetched; // the download area the terrain model was given
	void nmt_under_anchors();
	// added to the height of the model at the place: the offset of the scenery, drawn to the tracks near them
	double nmt_offset_at(double const X, double const Z) const;
	void nmt_finish_fit();
	// the chunk reaches within the band of the track at the place, in the download area
	bool nmt_key_in_band(heightmap::chunk_key const &Key, nmt_place const &Place) const;
	// how much the terrain lies over the model at the points: none where there's too little terrain at them, nor where it
	// doesn't follow the model (Foreign), as one made by hand
	std::optional<double> nmt_terrain_offset(std::vector<glm::dvec2> const &Points, bool &Foreign) const;
	// the same along the edges the chunk shares with the chunks beside it in memory
	std::optional<double> nmt_seam_offset(heightmap::chunk_key const &Key, bool &Foreign) const;
	void nmt_follow_tracks();
	// the height the ground will have where the terrain is made ahead of the track laid and isn't there yet
	bool nmt_ahead(double const X, double const Z, double &Height) const;
	// Offset: the heights of the model go up or down by it in the whole chunk, in place of the offsets of the fit
	nmt_chunk add_chunk_from_nmt(heightmap::chunk_key const &Key, std::optional<double> const Offset = std::nullopt);
	// a chunk there with its ground level all over, as made where there was no ground and no model: none of the shape of the land
	bool nmt_flat_chunk(heightmap::chunk_key const &Key) const;
	// a few of the chunks waiting made, those whose model is in: Made counts them and the ones given up, Apart those ahead no
	// terrain of the model touches, Changed takes the ones made
	void nmt_make_pending(int &Made, std::size_t &Apart, std::vector<heightmap::chunk_key> &Changed);
	// the chunks made told of, the ground gathered again under them; once none waits, the formations and the photos led onto them
	void nmt_made(std::vector<heightmap::chunk_key> const &Changed);
};

// the window of the terrain model: where it comes from, its settings, its progress, what the fit found, and the offers and banners of the view
class nmt_panel final : public editor_tool
{
  public:
	nmt_panel(editor_services &Services, editor_build_services &Build, nmt_builder &Nmt_build, nmt_auto_fit &Nmt_auto, profile_library &Library, profile_editor &Profile_edit) : m_services{Services}, m_build{Build}, m_nmt_build{Nmt_build}, m_nmt_auto{Nmt_auto}, m_library{Library}, m_profile_edit{Profile_edit}, m_nmt_state{Build.terrain_state()}, m_nmt{Services.terrain_model()}, m_streamer{Build.ground_streamer()}, m_orthophoto{Build.orthophoto()}, m_nmt_map{Build.terrain_map()}, m_lay{Build.lay_state()}, m_terrain_add_count{Build.terrain_add_count()}, m_terrain_open{Build.terrain_open()}, m_orthophoto_origin_edit{Build.orthophoto_origin_edit()} {}
	// the terrain of the selected tracks in the panel of the tracks
	void render_track_terrain(std::vector<TTrack *> const &Tracks);
	// a line from From to To being put onto the terrain kept to be looked at: the place of it to show and what's wrong there
	void nmt_note(nmt_state::finding::kind const What, TTrack *From, TTrack *To, double const Length, glm::dvec3 const &Where, std::string Text, double const Size = 0.0);
	// the chunks waiting for the terrain model and the tiles of the imagery asked for outlined where they will be, so it's seen
	// what's on its way
	void draw_nmt_pending() const;
	// over the 3d view while the terrain panel isn't seen
	void render_nmt_banner();
	// the tracks laid since the step of the history hidden in the ground, told over the 3d view with the way to fit the ground to them
	void nmt_check_buried(std::size_t const From);
	void render_nmt_buried();
	// what to do first, in the middle of the view of a scenery with no tracks nor terrain
	void render_empty_scenery_hint() const;
	// the 3d camera under the ground, as after the view from above or the terrain model made under it, lifted over it
	void camera_above_ground() const;
	void render_nmt_ui();
	// a scenery begun from nothing, with no tracks nor terrain, is asked as the first track is laid whether it wants the terrain
	void nmt_offer();
	void nmt_accept_offer();
	void render_nmt_prompt();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	nmt_builder &m_nmt_build;
	nmt_auto_fit &m_nmt_auto;
	profile_library &m_library;
	profile_editor &m_profile_edit;
	nmt_state &m_nmt_state;
	editor_nmt &m_nmt;
	terrain_streamer &m_streamer;
	editor_orthophoto &m_orthophoto;
	editor_map &m_nmt_map;
	lay_tool &m_lay;
	int &m_terrain_add_count;
	bool &m_terrain_open;
	glm::dvec2 &m_orthophoto_origin_edit;
	// the work under way: what's left of it and the button to stop it; Compact: one line, for the 3d view
	void render_nmt_progress(bool const Compact);
	// all the tracks of the scenery onto the terrain model, or it under them: step 2 of the terrain model (Terrain_panel), and in
	// the panel of the tracks
	void render_nmt_all_tracks(bool const Terrain_panel);
	// the part of it putting all the tracks onto the terrain anew, Hand of them set by hand, Unfitted not fitted yet
	void render_nmt_all_anew(std::size_t const All, std::size_t const Hand, std::size_t const Unfitted, bool const Ready);
	// the place found shown in the 3d view with the track there selected; Profile: the profile of its line opened too
	void nmt_show(nmt_state::finding const &Finding, bool const Profile);
	// what the last putting of the tracks onto the terrain did, and the places it found to be looked at, each to be shown
	void render_nmt_result();
	// the list of the places found, in the Order of them, the bad ones and the doubtful in their colours
	void render_nmt_findings(std::vector<std::size_t> const &Order, ImVec4 const &Bad, ImVec4 const &Doubt);
	// where the scenery lies: the map to click on, the place typed; true once it's placed
	bool render_nmt_place(bool const Moving, bool const Button);
	// step 1 of it, where the scenery lies, the place to change shown once it's there; false while it's nowhere yet
	bool render_nmt_location(bool const Busy);
	// step 2 of it with no tracks yet: the model as it is loaded around the camera
	void render_nmt_load(bool const Busy);
	// the settings of it under the steps; Tracks: the scenery has tracks, the band along them and the height offset shown
	void render_nmt_settings(bool const Tracks, bool const Busy);
	// the place picked in the terrain window: the terrain model and the orthophoto take it
	void nmt_place_scenery(glm::dvec2 const &Point);
};

// the download area of the terrain model and of the imagery, drawn on the map and kept with the scenery
class download_area_tool final : public editor_tool
{
  public:
	download_area_tool(editor_services &Services, editor_build_services &Build, nmt_builder &Nmt_build) : m_build{Build}, m_nmt_build{Nmt_build}, m_nmt{Services.terrain_model()}, m_streamer{Build.ground_streamer()}, m_orthophoto{Build.orthophoto()}, m_area_drawing{Build.area_state()}, m_area_paused{Build.area_paused()}, m_orthophoto_chosen{Build.orthophoto_chosen()} {}
	// Keep: the area drawn taken, else the one before stays
	void area_finish(bool const Keep);
	// a press of the left button while drawing: a point added, or the one under the cursor taken to drag, Shift: removed
	void area_click(bool const Shift);
	void area_update();
	void draw_download_area() const;
	// the chunk of the terrain (x, z) touches the area, or there's none
	bool in_download_area(glm::dvec2 const &Low, glm::dvec2 const &High) const;
	void render_download_area_ui(editor_orthophoto::statistics const &Stats);

  private:
	editor_build_services &m_build;
	nmt_builder &m_nmt_build;
	editor_nmt &m_nmt;
	terrain_streamer &m_streamer;
	editor_orthophoto &m_orthophoto;
	area_drawing &m_area_drawing;
	bool &m_area_paused;
	bool &m_orthophoto_chosen;
	// the download area: a polygon drawn on the map, the boundary of what the orthophoto and the terrain model download in the
	// background, none of them asking for anything outside it (application/editormodenmt.cpp)
	void area_start();
	// a point of the area (x, z) where it's drawn: over the map of the top view, on the ground in the 3d view
	glm::dvec3 area_point(glm::dvec2 const &Point) const;
};

// the grade lines of all the tracks fitted to the terrain model line after line, the turntables levelled and the grades shared where lines meet
class nmt_auto_fit final : public editor_tool
{
  public:
	nmt_auto_fit(editor_services &Services, editor_build_services &Build, nmt_builder &Nmt_build, grade_line_tool &Grade_lines, profile_library &Library, nmt_panel &Nmt_window, profile_editor &Profile_edit) : m_services{Services}, m_build{Build}, m_nmt_build{Nmt_build}, m_grade_lines{Grade_lines}, m_library{Library}, m_nmt_window{Nmt_window}, m_profile_edit{Profile_edit}, m_profile{Build.vertical_profile()}, m_nmt_state{Build.terrain_state()}, m_nmt{Services.terrain_model()}, m_profiles{Build.stored_profiles()} {}
	// the turntables as they lie, each with the ends of the tracks running up to it on its rim
	std::vector<nmt_state::turntable> nmt_turntables() const;
	// Unfitted: only the tracks whose grade line isn't fitted yet, the scenery left at its height
	void nmt_auto_start(bool const Unfitted = false);
	// the tracks laid in one go since the history step From put onto the terrain model as one line, as the tracks go onto the
	// terrain: the track ending at End then the one the laying goes on from
	bool nmt_fit_laid(std::size_t const From, glm::dvec3 const End);
	void nmt_auto_step();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	nmt_builder &m_nmt_build;
	grade_line_tool &m_grade_lines;
	profile_library &m_library;
	nmt_panel &m_nmt_window;
	profile_editor &m_profile_edit;
	profile_state &m_profile;
	nmt_state &m_nmt_state;
	editor_nmt &m_nmt;
	std::vector<stored_profile> &m_profiles;
	// the steps of fitting a line and of sharing the grades where lines meet, with what they share (application/editormodenmt.cpp)
	struct nmt_line_fit;
	struct nmt_grade_share;
	// the whole scenery: tracks, includes and models, moved up or down as one undo step
	std::size_t nmt_join_ends();
	// each turntable level at the height of the tracks running up to it, its include with it, before the grade lines are fitted:
	// at the height of the ones fitted already, else of the terrain model beside its rim; the number of them moved
	std::size_t nmt_level_turntables();
	bool nmt_stays(TTrack const &Track); // this putting of the tracks onto the terrain leaves it as it is
	// the grade line of the line of tracks through the track fitted to the terrain model and applied; false if it couldn't be.
	// The line runs over the tracks not Fitted, the ones fitted stay as they are and the line meets them. Where the model falls
	// away under the line by the abutments the tracks go over on a bridge; Laid: the tracks just laid, the formation fills it and
	// they are cut only where they can't follow the line otherwise
	bool nmt_fit_line(TTrack &Track, std::function<bool(TTrack const *)> const &Fitted, bool const Laid);
	// the points of the paths of the track about a metre apart, in squares of the plan
	static void nmt_sample_paths(TTrack const &Track, std::map<std::pair<int, int>, std::vector<nmt_state::laid_point>> &Points);
	// the tracks laid one after another on one straight or one arc, cut by the grade lines fitted before, made one again
	void nmt_merge_laid(std::unordered_set<TTrack const *> &Laid);
	// the grade lines fitted, the tracks nearly parallel under 5.1 m apart brought onto one, a track going away from them fading back onto its own
	void nmt_share_grades();
	bool nmt_auto_offset();
	// its steps: the model fetched and offset to the tracks, then one line fitted
	void nmt_auto_fetch();
	bool nmt_fit_next_line();
};
