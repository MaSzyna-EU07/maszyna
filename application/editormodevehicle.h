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

#include <memory>
#include <set>
#include <string>
#include <vector>

namespace ui
{
class vehicles_bank;
struct vehicle_desc;
struct skin_set;
}

// a trainset put on the selected path, written to the scenery on save, or only to be driven
class vehicle_tool final : public editor_tool
{
  public:
	explicit vehicle_tool(editor_services &Services) : m_services{Services} {}
	void draw_overlay() const override;
	void render_options() override;
	editor_selftest_result selftest(std::string const &Command, std::istringstream &Words) override;
	tool_tab &tab() { return m_tab; }
	void start(TTrack &Track, glm::dvec3 const &Point);
	bool take_leave();
	std::size_t trainset_lines(std::vector<std::string> &Lines);
	void trainsets_written();

  private:
	editor_services &m_services;
	tool_tab m_tab;
	struct vehicle_state
	{
		std::shared_ptr<ui::vehicles_bank> bank;
		char search[64]{};
		bool controllable_only{true};
		std::shared_ptr<ui::vehicle_desc> vehicle;
		std::shared_ptr<ui::skin_set> skin;
		int driver{0}; // 0: a driver, to be driven; 1: nobody, parked
		TTrack *track{nullptr};
		glm::dvec3 point{0.0};
		std::string placed;
		std::string status;
		bool leave{false}; // the editor gives way to the driving at the start of the next frame
		struct consist_vehicle
		{
			std::shared_ptr<ui::vehicle_desc> vehicle;
			std::shared_ptr<ui::skin_set> skin;
			bool turned{false};
		};
		std::vector<consist_vehicle> consist;
		int picked{-1}; // vehicle of the train whose buttons are shown
		std::set<ui::skin_set const *> minis; // skins whose side view was asked for
		int facing{1};
		bool write{true};
		char timetable[64]{"none"};
		double velocity{0.0};
		struct trainset
		{
			std::string head;
			std::string track;
			std::vector<std::string> lines;
		};
		std::vector<trainset> trainsets;
	};
	vehicle_state m_vehicle;
	bool place();
};
