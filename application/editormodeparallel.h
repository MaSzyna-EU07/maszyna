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
#include <vector>

// second track alongside the selected path or the line through it, at a spacing, curves included
class parallel_tool final : public editor_tool
{
  public:
	explicit parallel_tool(editor_services &Services) : m_services{Services} {}
	void draw_overlay() const override;
	void render_options() override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	tool_tab &tab() { return m_tab; }
	void render_set_options();

  private:
	editor_services &m_services;
	tool_tab m_tab;
	struct parallel_state
	{
		double spacing{4.0};
		int side{1}; // 1: to the right, -1: to the left, of the direction of the line
		int scope{1}; // 0: the selected path, 1: the line through it, up to the switches
		TTrack *built_for{nullptr};
		double built_spacing{0.0};
		int built_side{0};
		int built_scope{-1};
		std::size_t built_history{0};
		TTrack *pair_src{nullptr};
		TTrack *pair_dst{nullptr};
		double measured{0.0};
		std::vector<segment_data> pieces;
		std::vector<TTrack *> styles;
		double length{0.0};
		std::string error;
		std::string status;
	};
	parallel_state m_parallel;
	TTrack *parallel_source() const;
	TTrack *parallel_target() const;
	void parallel_update();
	void render_parallel_controls(bool const Scope = true);
	void parallel_build();
	void parallel_apply();
	bool parallel_preview_visible() const;
};
