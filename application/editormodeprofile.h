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

#include <cstdint>
#include <functional>
#include <map>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class profile_library;
class profile_editor;
class profile_panel;
class grade_line_tool;

// the grade lines kept in the scenery files and the marks of the tracks fitted to the terrain: loaded, stored, found and restored
class profile_library final : public editor_tool
{
  public:
	profile_library(editor_build_services &Build, profile_editor &Profile_edit) : m_profile_edit{Profile_edit}, m_profile{Build.vertical_profile()}, m_profiles{Build.stored_profiles()}, m_profiles_loaded{Build.profiles_loaded()} {}
	void profile_library_load();
	// the marks of the tracks fitted dropped, read again from the scenery files when next asked; how many are kept
	void fitted_forget();
	std::size_t fitted_count() const;
	void profile_library_mark(std::vector<scene::layer_handle> Layers);
	// the kept grade lines back to the library given, the step changing them given up before it went to the history
	void profiles_restore(std::vector<stored_profile> Library);
	void fitted_set(std::span<TTrack *const> const Tracks, bool const Hand = false);
	bool grade_hand(TTrack const &Track); // its grade line was edited with the profile tool, and it wasn't changed since
	bool grade_fitted(TTrack const &Track);
	int profile_find(bool &Reversed) const;
	bool profile_restore();
	void profile_store();
	void profile_forget();
	// the route a kept design was laid over, as it runs now; none when its paths moved or are gone
	std::optional<editor_track::route> profile_stored_route(stored_profile const &Entry, TTrack **From = nullptr, TTrack **To = nullptr) const;
	// the kept designs over the tracks moved, as by the grade lines shared, laid anew onto them where they part from them; returns how
	// many were, how many couldn't be without an error and are left as they were
	std::pair<std::size_t, std::size_t> profile_refit_stored(std::unordered_set<TTrack const *> const &Moved);
	void profile_open_stored(std::size_t const Index);

  private:
	profile_editor &m_profile_edit;
	profile_state &m_profile;
	std::vector<stored_profile> &m_profiles;
	bool &m_profiles_loaded;
	std::map<std::pair<std::string, std::uint64_t>, fitted_mark> m_fitted; // by the name and the shape: scenery files have names used twice
	bool m_fitted_loaded{false};
	static std::uint64_t fitted_shape(TTrack const &Track);
	void fitted_load();
	bool profile_restore_parts();
};

// the vertical profile of a route: opened, sampled, recognised, fitted to the ground, applied to the paths and followed through the edits
class profile_editor final : public editor_tool
{
  public:
	profile_editor(editor_services &Services, editor_build_services &Build, profile_library &Library, grade_line_tool &Grade_lines) : m_services{Services}, m_build{Build}, m_library{Library}, m_grade_lines{Grade_lines}, m_profile{Build.vertical_profile()}, m_profiles{Build.stored_profiles()}, m_nmt_state{Build.terrain_state()}, m_nmt{Services.terrain_model()}, m_streamer{Build.ground_streamer()}, m_terrain_spacing{Build.terrain_spacing()}, m_track_edits{Build.track_edits()} {}
	void profile_edited();
	// the radii of its neighbours were for the breaks of the grade it made, they follow the limits after it
	void profile_remove_point(int const Index);
	void profile_open(TTrack *From, TTrack *To);
	void profile_open_run(TTrack &Track);
	void profile_follow_selection();
	void profile_take(editor_track::route Route, TTrack *From, TTrack *To);
	void profile_resample();
	// the ground of the terrain model under the route, as it was measured before any earthworks
	void profile_sample_model();
	void profile_recognize();
	void profile_fit_ground();
	// depth of the foot of the ballast under the rail top, and its distance from the axis
	static std::pair<double, double> profile_ballast(TTrack const &Track);
	// false for a track with no trackbed of its own, as the switches of the older sceneries whose bed is an include
	static bool profile_own_bed(TTrack const &Track);
	void render_profile_earthworks();
	void profile_check();
	void profile_fit_view();
	void profile_to_camera();
	// the camera to the part of the route the plot shows; the plot to the part of the route the camera shows, its zoom kept
	void profile_frame_view();
	// Terrain: the heightmap terrain follows the tracks changed. Bridges: stretches of the route over a bridge or a viaduct, the
	// paths over them cut off at their ends and set to the bridge environment
	void profile_apply(bool const Terrain = false, std::vector<profile::departure> const &Bridges = {}, double const Fewest = 0.0);
	void profile_after_undo();
	void profile_follow_edits();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	profile_library &m_library;
	grade_line_tool &m_grade_lines;
	profile_state &m_profile;
	std::vector<stored_profile> &m_profiles;
	nmt_state &m_nmt_state;
	editor_nmt &m_nmt;
	terrain_streamer &m_streamer;
	float const &m_terrain_spacing;
	int const &m_track_edits;
	void profile_open_selection(std::vector<TTrack *> const &Tracks);
	std::pair<int, int> profile_generate_ground(std::vector<std::pair<double, double>> const &Beds, double const Reach);
	void profile_shape_ground();
	void profile_restore_ground();
	// the terrain under the tracks changed by the last Edits steps of the history which changed tracks
	void profile_follow_terrain(std::size_t const Edits);
};

// the panel of the vertical profile: its header, points, parameters and canvas, and the profile drawn over the view
class profile_panel final : public editor_tool
{
  public:
	profile_panel(editor_services &Services, editor_build_services &Build, profile_library &Library, profile_editor &Profile_edit, grade_line_tool &Grade_lines) : m_services{Services}, m_library{Library}, m_profile_edit{Profile_edit}, m_grade_lines{Grade_lines}, m_profile{Build.vertical_profile()}, m_profiles{Build.stored_profiles()}, m_nmt{Services.terrain_model()} {}
	void render_profile_body();
	void render_profile_strip();
	void draw_profile_overlay() const;

  private:
	// screen mapping of the profile canvas, chainage across and elevation upwards
	struct profile_view;
	editor_services &m_services;
	profile_library &m_library;
	profile_editor &m_profile_edit;
	grade_line_tool &m_grade_lines;
	profile_state &m_profile;
	std::vector<stored_profile> &m_profiles;
	editor_nmt &m_nmt;
	void render_profile_header();
	void render_profile_empty();
	void render_profile_source();
	bool render_profile_parameters();
	void render_profile_issues();
	void render_profile_tabs();
	// the fields of the point Index of the grade line: true where one was changed
	bool render_profile_point_fields(int const Index, bool const End, bool const Locked);
	void render_profile_point();
	void render_profile_points();
	void render_profile_toolbar();
	void render_profile_canvas(float const Height);
	void profile_fit_exaggeration(profile_view const &View);
	void profile_navigate(profile_view const &View, bool const Hovered);
	bool profile_canvas_edit(profile_view const &View, bool const Hovered);
	void draw_profile_canvas(profile_view const &View, ImDrawList &Draw) const;
	void profile_canvas_tooltip() const;
};
