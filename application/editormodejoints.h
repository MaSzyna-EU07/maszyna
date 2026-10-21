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

#include <string>
#include <utility>
#include <vector>

class joints_tool final : public editor_tool
{
  public:
	// joint of the paths found wrong by the joints check
	struct joint_issue
	{
		enum class kind { gap, step, kink, cant, grade, model, open };
		kind type{kind::gap};
		TTrack *track{nullptr};
		editor_track::point_ref point;
		TTrack *other{nullptr};
		editor_track::point_ref theirs;
		glm::dvec3 position{0.0};
		double value{0.0}; // m, deg, mm, per mille; m of the track over the terrain model
		bool fixed{false};
	};
	joints_tool(editor_selection &Selection, editor_view &View, editor_history &History, editor_ground &Ground) : m_selection{Selection}, m_view{View}, m_history{History}, m_ground{Ground} {}
	void update() override;
	void draw_overlay() const override;
	void render_options() override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	void set_open(bool const Open) { m_joints.open = Open; }
	double kink_angle() const { return m_joints.angle; }

  private:
	editor_selection &m_selection;
	editor_view &m_view;
	editor_history &m_history;
	editor_ground &m_ground;
	// joints of the paths: ends which almost meet, steps, kinks, jumps of the cant and of the grade
	struct joints_state
	{
		bool open{false};
		int scope{0}; // 0: around the camera, 1: the whole scenery
		double reach{2000.0};
		double gap{1.0}; // m, free ends closer than this to another end
		double step{0.005}; // m
		double angle{0.2}; // deg
		double cant{5.0}; // mm
		double grade{10.0}; // per mille
		double model{0.5}; // m, the track over or under the terrain model (NMT)
		bool free_ends{false};
		bool model_check{true};
		std::vector<TTrack *> queue;
		std::size_t next{0};
		// the places of the tracks compared with the terrain model, in runs of one path each, once the model under them is in
		std::vector<nmt_place> places;
		std::vector<std::pair<std::size_t, std::size_t>> runs;
		std::size_t run_next{0};
		bool model_waiting{false};
		bool model_asked{false};
		double model_polled{0.0};
		bool model_unplaced{false};
		std::vector<joint_issue> issues;
		int filter{-1};
		int selected{-1};
		std::size_t history{0};
		std::string status;
	};
	joints_state m_joints;
	void joints_start();
	void joints_check(TTrack &Track);
	// the paths lying off the terrain model by more than the threshold. returns: false while the model under them is coming
	bool joints_model();
	void joints_focus(int const Index);
	bool joint_fix(joint_issue &Issue, std::vector<std::pair<TTrack *, editor_track::state>> &States, std::vector<TTrack *> &Changed);
	void joints_fix(std::vector<int> const &Indices);
};
