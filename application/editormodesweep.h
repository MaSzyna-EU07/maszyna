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
#include "application/editormodesweepstate.h"

#include <sstream>
#include <string>
#include <vector>

// a model laid along the track: bent to follow it, or repeated along it, over the stretch marked in the view; its state is
// editor_mode's sweep_tool, the paths the models were laid on its own
class sweep_editor final : public editor_tool
{
  public:
	sweep_editor(editor_selection &Selection, editor_build_services &Build, bend_tool &Bend) : m_selection{Selection}, m_build{Build}, m_sweep{Build.sweep_state()}, m_bend{Bend} {}
	void draw_overlay() const override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	void sweep_outline();
	double sweep_station(glm::dvec3 const &Point, double &Lateral) const;
	void sweep_measure();
	void sweep_delete(sweep_node &Sweep);
	std::vector<sweep_node *> sweeps_near(TTrack const &Track) const;
	void sweep_drag();
	void sweep_release();
	// the same at a point of the world: Anywhere takes the press however far from the track it is
	bool sweep_press_at(glm::dvec3 const &Point, bool const Anywhere);
	void render_sweep_ui();
	void sweeps_captured(TTrack const &Track);
	void sweeps_committed(std::vector<TTrack *> const &Tracks);
	void sweeps_split(TTrack &Original, TTrack &Created);

  private:
	using selftest_result = editor_selftest_result;
	editor_selection &m_selection;
	editor_build_services &m_build;
	sweep_tool &m_sweep;
	bend_tool &m_bend;
	std::vector<sweep_link> m_sweep_links;
	bool sweep_extend(bool const Atend, glm::dvec3 const &Cursor);
	void sweep_scan_models();
	sweep_node::state sweep_definition() const;
	void sweep_create();
	void sweep_apply();
	void sweep_edit(sweep_node &Sweep);
	void sweep_drag_at(glm::dvec3 const &Point);
	// the sweep command of the self-test with more to it: a model laid along the selected track
	selftest_result selftest_sweep_lay(std::istringstream &words);
};

// a model bent along a track: chosen, placed by its reference point, dragged by its marker and edited; its state is bend_tool of
// the build tools, shared with the models laid along the track
class bend_editor final : public editor_tool
{
  public:
	bend_editor(editor_selection &Selection, editor_build_services &Build, bend_tool &Bend, sweep_editor &Sweeps) : m_selection{Selection}, m_build{Build}, m_bend{Bend}, m_sweep{Build.sweep_state()}, m_sweeps{Sweeps} {}
	void draw_overlay() const override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	bool bend_drag_start(int const Mods);
	void bend_drag_update();
	bool bend_click();
	bool bend_shortcut();
	void bend_pick(glm::dvec3 const &Point);
	void render_bend();

  private:
	using selftest_result = editor_selftest_result;
	editor_selection &m_selection;
	editor_build_services &m_build;
	bend_tool &m_bend;
	sweep_tool &m_sweep;
	sweep_editor &m_sweeps;
	sweep_node *sweep_under(glm::dvec3 const &Point) const;
	void bend_find_tracks(glm::dvec3 const &Point, void const *Key);
	bool bend_definition(bend_source const &Source, TTrack &Track, sweep_node::state &State, std::vector<TTrack *> &Tracks);
	void bend_selection();
	void bend_edit(sweep_node::state const &State);
	void bend_reanchor(sweep_node::state &State, int const Side, int const Height, glm::dvec3 const &Point) const;
	double bend_fixed(sweep_node const &Sweep, glm::dvec3 const &Point) const;
	void bend_stretch(sweep_node const &Sweep, double const Fixed, double const Length, glm::dvec3 const &Point, sweep_node::state &State) const;
	glm::dvec3 bend_marker(sweep_node const &Sweep) const;
	// the bent model edited: where it lies by the track, how long, bent or repeated, and the rest of its settings
	void render_bend_edited(sweep_node &Sweep);
	bool render_bend_place(sweep_node &Sweep, sweep_node::state &State, double const Stretch) const;
	bool render_bend_lengths(sweep_node &Sweep, sweep_node::state &State, double const Own, float const Half) const;
	bool render_bend_more(sweep_node::state &State);
	void render_bend_finish(sweep_node &Sweep, float const Half);
	// no bent model edited: the model selected to be bent, the bent ones by the track or near the camera to pick
	void render_bend_choice();
	void render_bend_listed(TTrack const *Track);
	// the reference point of the model bent: where along the track, across and up it lies
	void render_bend_anchors(bool &Changed);
	// a model chosen to be bent, the tracks by it found: the track to bend it along and how
	void render_bend_source(bend_source const &Source, void const *Key);
	// the bend commands of the self-test with more to them: a model bent along a track, set again, probed
	selftest_result selftest_bend_lay(std::istringstream &words);
	selftest_result selftest_bend_set(std::istringstream &words);
	selftest_result selftest_bend_probe() const;
};
