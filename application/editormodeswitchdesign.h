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
#include "application/editormoderailservices.h"
#include "editor/editorSwitchDesign.hpp"

#include <glm/glm.hpp>

#include <sstream>
#include <string>
#include <vector>

// the window of the switch templates: the catalogue and the own ones, made of straights, arcs and transition curves, and a switch
// measured off the view
class switch_designer_tool final : public editor_tool
{
  public:
	switch_designer_tool(editor_view &View, editor_guide &Guide, editor_rail_services &Rail) : m_view{View}, m_guide{Guide}, m_rail{Rail} {}
	// the window, and the points clicked off the view drawn under it
	void render_windows() override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	bool is_open() const { return m_list.open; }
	void open_window() { m_list.open = true; }
	// the points of a switch are being clicked off the view: the window takes the mouse
	bool is_picking() const { return m_picked.picking; }
	void stop_picking() { m_picked.picking = false; }
	std::vector<glm::dvec3> const &clicks() const { return m_picked.clicks; }
	// a point of the switch clicked off the view, and the last one taken back
	void click(glm::dvec3 const &Point);
	void click_back();

  private:
	// the templates of the window: the catalogue, the own ones and the one chosen
	struct design_list
	{
		bool open{false};
		bool loaded{false};
		std::vector<switch_design::design> catalogue;
		std::vector<switch_design::design> own;
		bool from_catalogue{true};
		int selected{-1};
		bool dirty{false};
		std::string status;
	};
	// the diverging track filled in from its radius, its angle and its a and b
	struct quick_fill
	{
		double quick_radius{300.0};
		double quick_ratio{9.0};
		double quick_a{0.0};
		double quick_b{0.0};
		int fit_kind{3}; // of switch_design::tangent_fit
		double quick_first{0.0}; // the radius of the first arc of an arc, a straight and an arc, 0: the R
	};
	// the switch clicked off the view, the orthophoto under it, and what its points make
	struct picked_switch
	{
		bool picking{false};
		std::vector<glm::dvec3> clicks;
		bool on_view{true}; // the template shown in the view where its first points were clicked
		switch_design::design measured; // what the points clicked so far make
		bool measured_ready{false};
		std::string measured_text;
	};
	// how the template is drawn, and the blade of its switch
	struct drawing
	{
		bool stretched{true};
		int pointed{-1}; // the value under the mouse, its dimension shown on the drawing
		int pointed_test{-1};
		double blade_length{5.0};
		double blade_gap{0.15};
	};

	void load();
	void measure();
	void make_template();
	void draw_clicks() const;
	void render_list();
	void render_clicking();
	void render_edit();
	bool render_quick(switch_design::design &Entry, int &Pointing);
	void render_actions(switch_design::design const &Entry, bool const Editable, bool const Buildable);
	editor_selftest_result selftest_designer(std::istringstream &Words);
	editor_selftest_result selftest_designerpick(std::istringstream &Words);

	editor_view &m_view;
	editor_guide &m_guide;
	editor_rail_services &m_rail;
	design_list m_list;
	quick_fill m_quick;
	picked_switch m_picked;
	drawing m_drawing;
};
