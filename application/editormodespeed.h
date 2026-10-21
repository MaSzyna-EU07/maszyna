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
#include "editor/editorSpeed.hpp"

#include <string>
#include <utility>
#include <vector>

class speed_tool final : public editor_tool
{
  public:
	explicit speed_tool(editor_services &Services) : m_services{Services} {}
	void update() override;
	void draw_overlay() const override;
	void render_options() override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	void set_open(bool const Open) { m_speed.open = Open; }
	void check_line();

  private:
	editor_services &m_services;
	// speed limits of the paths checked against what their geometry allows
	struct speed_state
	{
		bool open{false};
		int scope{0}; // 0: the line through the selected path, 1: the whole scenery
		speed_check::options options;
		std::vector<std::pair<TTrack *, int>> queue;
		std::size_t next{0};
		std::vector<speed_check::verdict> results;
		bool show_all{false};
		int limits{0}; // 0: all the paths, 1: only those with a speed limit set, 2: only those without
		int selected{-1};
		std::size_t history{0}; // size of the undo history at the check
		std::string status;
	};
	speed_state m_speed;
	void speed_start();
	void speed_focus(int const Index);
	// with Branches the limits of the switch branches count too, otherwise they're only warnings
	void speed_apply(std::vector<int> const &Indices, bool const Branches);
	std::vector<int> speed_listed() const;
};
