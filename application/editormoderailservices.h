/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <glm/glm.hpp>

#include <string>

// what the rail tools (the track tab and its panels, the gizmo, track properties and bed, switches, signals, the turntable and the lineside) need from the editor beyond the interfaces of editortool.h; editor_mode implements it
class editor_rail_services
{
  public:
	// the list of the switch tool built again with the own templates, keeping the armed one or arming the one whose label starts with Arm
	virtual void switch_templates_refresh(std::string const &Arm) = 0;
	// the switch tool armed with its template chosen
	virtual void arm_switch_tool() = 0;
	// the point of the ground under the cursor, and the one at the height given
	virtual glm::dvec3 cursor_on_ground() const = 0;
	virtual glm::dvec3 cursor_on_level(double const Height) const = 0;
	// the ring of the cursor drawn at the point
	virtual void draw_ring(glm::dvec3 const &Point) const = 0;
	// the orthophoto is turned on, and its window opened
	virtual bool orthophoto_shown() const = 0;
	virtual void open_orthophoto_window() = 0;

  protected:
	~editor_rail_services() = default;
};
