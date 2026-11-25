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
#include "editor/editorNormsReport.hpp"

#include <string>
#include <vector>

// the norms of the track geometry checked on the tracks laid, read only: a line or the whole scenery against a profile and a class
// of the line, what's beyond the limits listed in the track analysis window, a row clicked showing the place
class norms_tool final : public editor_tool
{
  public:
	norms_tool(editor_selection &Selection, editor_view &View, editor_history &History) : m_selection{Selection}, m_view{View}, m_history{History} {}
	void update() override;
	void draw_overlay() const override;
	void render_options() override;
	void forget(scene::basic_node const *Node) override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	void set_open(bool const Open) { m_state.open = Open; }
	// what the last check found, for the expectations of the self-test
	std::vector<norms_report::entry> const &results() const { return m_state.results; }
	// the profile and the class of the line the checks are made against, as the assumptions of the project have them
	void defaults(norms::profile_id const Profile, norms::line_class const &Line);
	// the lines of the whole scenery checked against them, from now on, frame after frame, the window open on the report
	void check_scenery();
	bool checking() const { return false == m_state.queue.empty(); }
	// the lines of the check done last with a finding of the tier or worse
	std::size_t lines_beyond(norms::tier const Tier) const;

  private:
	editor_selection &m_selection;
	editor_view &m_view;
	editor_history &m_history;
	// a line to check: its paths, and the one span whose findings alone are kept (the diverging track of a switch), -1 all
	struct pending
	{
		std::vector<norms_report::span> spans;
		int own{-1};
	};
	struct state
	{
		bool open{false};
		int scope{0}; // 0: the line through the selected path, 1: the whole scenery
		int profile{0};
		norms_report::options options;
		bool notes{false}; // the values above the recommended ones listed too
		std::vector<pending> queue;
		std::size_t next{0};
		std::vector<norms_report::checked_line> lines;
		std::vector<norms_report::entry> results;
		int selected{-1};
		std::size_t history{0}; // size of the undo history at the check
		std::string status;
	};
	state m_state;
	void start();
	void finish();
	void focus(int const Index);
	std::vector<int> listed() const;
	void render_class();
	void render_results();
};
