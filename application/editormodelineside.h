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

#include <optional>
#include <string>
#include <utility>
#include <vector>

// hectometre posts along the line, at every 100 m of the kilometrage
class hekto_tool final : public editor_tool
{
  public:
	explicit hekto_tool(editor_services &Services) : m_services{Services} {}
	void draw_overlay() const override;
	void render_options() override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	tool_tab &tab() { return m_tab; }
	void start(TTrack &Track, glm::dvec3 const &Point);

  private:
	editor_services &m_services;
	tool_tab m_tab;
	struct hekto_post
	{
		int hectometres{0};
		double chainage{0.0};
		glm::dvec3 position{0.0};
		double yaw{0.0};
		int standing{-1}; // post already there with this number
	};
	struct hekto_found
	{
		standing_template post;
		int hectometres{0};
		double chainage{0.0}; // where it stands along the line
		double expected{0.0}; // where its number belongs
		double error{0.0}; // m along the line
		double distance{0.0}; // m from where it belongs, in the plan
		bool wrong{false};
		bool duplicate{false}; // another post with the same number stands nearer to its place
		bool outside{false}; // its number belongs past the ends of the line
	};
	struct hekto_state
	{
		TTrack *track{nullptr}; // the line runs through it
		glm::dvec3 point{0.0}; // where the kilometrage is given
		double km{0.0};
		bool km_read{false}; // the kilometrage was read from the posts already standing
		int growth{1}; // 1: grows along the line, -1: against
		int side{1}; // 1: to the right of the growing kilometrage, -1: to the left, 0: alternately
		double offset{3.0}; // m from the axis
		int style{0}; // nowy, stary, zgnity, mixed
		int turn{0}; // degrees added to the rotation
		double reach{10000.0}; // m each way
		double tolerance{5.0}; // m along the line, for the posts standing
		bool dirty{true};
		std::size_t history{0};
		editor_track::route route;
		std::vector<editor_track::route_sample> samples;
		double origin{0.0}; // chainage of the point
		std::vector<hekto_post> posts;
		std::vector<hekto_found> found;
		std::string status;
		std::string error;
	};
	hekto_state m_hekto;
	void hekto_update();
	void hekto_read_kilometrage();
	void hekto_place();
	void hekto_fix();
	void hekto_remove_duplicates();
};

// fouling point markers (W17) where the tracks meeting at a switch are the spacing apart
class fouling_tool final : public editor_tool
{
  public:
	explicit fouling_tool(editor_services &Services) : m_services{Services} {}
	void draw_overlay() const override;
	void render_options() override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	tool_tab &tab() { return m_tab; }
	// the markers put at the fouling points without one, or at the one only
	void place(int const Only = -1);
	// the markers standing wrong moved to their places, or the one found only
	void fix(int const Only = -1);
	// the fouling point without a marker or the marker standing wrong under the cursor: the index of the point or of the
	// marker found, and whether it's a marker
	std::optional<std::pair<int, bool>> hit() const;
	glm::dvec3 position_of(std::pair<int, bool> const &Hit) const;
	void check_selected_switch();

  private:
	editor_services &m_services;
	tool_tab m_tab;
	struct fouling_point
	{
		TTrack *track{nullptr}; // the switch
		glm::dvec3 position{0.0};
		double yaw{0.0};
		int standing{-1}; // marker already there
	};
	struct fouling_found
	{
		standing_template marker;
		int point{-1}; // the fouling point it belongs to
		double error{0.0}; // m from where it belongs
		double height_error{0.0}; // m over where it belongs
		bool wrong{false};
	};
	struct fouling_state
	{
		int scope{1}; // 0: the selected switch, 1: around the camera, 2: the whole scenery
		double reach{1500.0};
		double spacing{3.5}; // m between the axes
		double tolerance{2.0}; // m, a marker farther than this from its point is wrong
		int height_mode{1}; // 0: on the ground, 1: at the height given over the rail heads
		double height{0.0}; // m over the rail heads
		double height_tolerance{0.03}; // m, a marker standing higher or lower than this is wrong
		int model{0}; // w17, w17_new, w17_old, w17_dwuteo
		int turn{0};
		bool dirty{true};
		std::size_t history{0};
		TTrack *selected{nullptr};
		std::vector<fouling_point> points;
		std::vector<fouling_found> found;
		std::string status;
		std::string error;
	};
	fouling_state m_fouling;
	void fouling_update();
};
