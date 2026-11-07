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

#include <functional>
#include <string>
#include <vector>

#include <glm/glm.hpp>

class TTrack;
class profile_library;
class profile_editor;
class profile_panel;

// the grade lines of the terrain model along the tracks: kept by the name of their tracks, picked, put on a route and carried through the edits
class grade_line_tool final : public editor_tool
{
  public:
	grade_line_tool(editor_services &Services, editor_build_services &Build, profile_library &Library) : m_services{Services}, m_build{Build}, m_library{Library}, m_profile{Build.vertical_profile()}, m_nmt_state{Build.terrain_state()}, m_track_edits{Build.track_edits()} {}
	static void move_path_heights(segment_data &Path, double const First, double const Second, std::function<void(segment_data &)> const &After = {});
	static std::string grade_line_key(TTrack const &Track);
	static std::string grade_line_key(std::string const &Name, TTrack const &Track);
	void grade_lines_load();
	void grade_lines_store();
	bool on_route_grade_line(TTrack const &Track);
	std::vector<TTrack *> const &grade_line_beside();
	std::size_t grade_line_follow(std::vector<editor_track::route_sample> const &Before);
	void grade_line_take_off(bool const Off);
	// the grade line the route shares, by most of its paths; -1 when it shares none
	int route_grade_line();
	// the run of the track clicked onto the grade line of the route by hand, or off it if it's on it; a route sharing none makes one with it
	void grade_line_pick(TTrack &Track, bool const Alone);
	// a piece split off a track lies on the grade line the track does
	void grade_line_split(TTrack const &Original, TTrack const &Created);
	// a track changed keeps its grade line under the key it has after it; a new one between two tracks of a line, a switch put into one, lies on it
	void grade_line_captured(TTrack const &Track);
	void grade_lines_committed(std::vector<TTrack *> const &Tracks);
	void grade_line_renamed(TTrack const &Track, std::string const &Old);
	// all the tracks of the grade line, wherever they are, for the view while they're picked
	std::vector<TTrack const *> const &grade_line_members(int const Number);
	void draw_grade_line_pick();
	void render_profile_grade_line();

  private:
	editor_services &m_services;
	editor_build_services &m_build;
	profile_library &m_library;
	profile_state &m_profile;
	nmt_state &m_nmt_state;
	int const &m_track_edits;
	// the plain tracks from switch to switch the track is one of, a switch by itself; Alone: the track only
	static std::vector<TTrack *> grade_line_run(TTrack &Track, bool const Alone);
	// the run put on the grade line of the route of the number; below 0, the route sharing none, a new line is made of the route and the run
	void grade_line_put_on(std::vector<TTrack *> const &Run, std::unordered_set<TTrack const *> const &Route, int const Number);
	// of the new tracks committed, the ones lying where a track of a line lay before the change, and the ones between two tracks of a
	// line, take it; each one taken is set to nullptr. Whether any was
	bool grade_lines_take_replaced(std::vector<TTrack *> &Fresh);
	// the lines of the tracks joined to the track at the point, each with whether one of those is on it by hand
	std::map<int, bool> grade_lines_at(TTrack const &Track, glm::dvec3 const &Point) const;
	// the line of the tracks joined at both ends of the first path of the track, -1 if none, -2 if more than one; and whether they're
	// on it by hand at both ends
	std::pair<int, bool> grade_line_between(TTrack const &Track) const;
	bool grade_lines_take_joined(std::vector<TTrack *> &Fresh);
};
