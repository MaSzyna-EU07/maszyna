/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <memory>

// the hud of the driver made with RmlUi, from ui/driverhud.rml in the game folder or from the copy built into the program when
// the folder has none: the driving aid (F1), the timetable (F2) and the scenario (F3), shown in place of their panels of imgui
// while RmlUi runs (ui.rmlui); without it the panels stay as they were. the player places the parts of the hud from the menu
// of the pause (Edit HUD): they're dragged, snap to the edges of the screen and of each other, and stay where they're left
// (hud_layout.ini in the profile of the user)
class driver_hud
{
  public:
	// what of the hud is shown, as the panels of imgui are set
	struct panels
	{
		bool aid{false}; // F1
		bool aid_expanded{false};
		bool timetable{false}; // F2
		bool timetable_expanded{false};
		bool scenario{false}; // F3
		bool editing{false}; // the parts are placed by the player: all are shown, and dragged with the mouse
	};

	driver_hud();
	~driver_hud();
	driver_hud(driver_hud const &) = delete;
	driver_hud &operator=(driver_hud const &) = delete;

	// the hud brought up to date with the simulation. returns: whether the hud shows the driving aid, the timetable and the
	// scenario, in place of the panels of imgui
	bool update(panels const &Panels);
	// the parts back where the document puts them, the places the player gave them forgotten
	void reset_layout();

  private:
	struct state;
	// the document is loaded the first time RmlUi runs
	bool open();
	// the places the player gave the parts, of hud_layout.ini
	void load_layout();
	void save_layout();

	std::unique_ptr<state> m_state;
	bool m_failed{false}; // the document couldn't be loaded, it isn't tried again
};
