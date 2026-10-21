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
#include "editor/editorGauge.hpp"

#include <memory>
#include <string>
#include <vector>

class gauge_tool final : public editor_tool
{
  public:
	// model entering the structure gauge, at its deepest place
	struct gauge_hit
	{
		std::string model;
		glm::dvec3 point{0.0};
		double depth{0.0}; // m
		TTrack const *track{nullptr}; // nearest one, may be gone by now
		int path{0};
		glm::dvec2 direction{0.0, 1.0}; // of the track there, in the plan
	};
	gauge_tool(editor_selection &Selection, editor_view &View, editor_history &History, editor_track_design &Design) : m_selection{Selection}, m_view{View}, m_history{History}, m_design{Design} {}
	void update() override;
	void render_windows() override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	bool is_open() const { return m_gauge.open; }
	void toggle() { m_gauge.open = !m_gauge.open; }

  private:
	editor_selection &m_selection;
	editor_view &m_view;
	editor_history &m_history;
	editor_track_design &m_design;
	// structure gauge along the line of the selected path, shown in the 3d view
	struct gauge_state
	{
		bool open{false}; // window
		bool enabled{true}; // tunnel along the selected path
		std::vector<gauge::profile> profiles;
		int profile{0}; // railway
		int road{-1};
		TTrack const *track{nullptr}; // scanned
		std::size_t history{0};
		bool pending{false}; // rescan once the edit ends
		struct
		{
			std::vector<glm::dvec3> surface; // triangles of the tunnel
			std::vector<glm::dvec3> edges; // lines
			std::vector<gauge_hit> hits;
		} line; // along the selected path
		struct
		{
			bool scanned{false};
			std::size_t history{0};
			std::string profile;
			std::vector<gauge_hit> hits;
		} map; // whole scenery
		struct
		{
			std::vector<glm::dvec3> surface;
			std::vector<glm::dvec3> edges;
		} spot; // tunnel around the hit the camera went to
		int current{-1}; // hit the camera went to
		bool published{false}; // overlay up to date
		std::shared_ptr<struct gauge_scan> scan; // of the whole scenery, done in slices over the frames
	} m_gauge;
	void gauge_load();
	// the chosen railway and road outlines
	gauge::choice gauge_choice() const;
	std::vector<gauge_hit> const &gauge_hits() const;
	void scan_gauge(TTrack &Track);
	void scan_gauge_map();
	void step_gauge_map();
	void finish_gauge_map();
	void gauge_publish();
	void gauge_focus(int Index);
	void gauge_spot(gauge_hit const &Hit);
	void render_gauge_hits();
	bool render_gauge_choice(char const *Label, int &Index, bool Road);
	bool render_gauge_outline(int Index);
};
