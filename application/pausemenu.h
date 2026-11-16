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

class keyboard_input;

// the menu of the pause made with RmlUi, from ui/pausemenu.rml in the game folder or the copy built into the program: back to
// the simulation, the settings of the graphics and of the sound (changed at once, written to eu07.ini on demand), the keys of
// the controls, and the end of the simulation. shown in place of the window of the pause of imgui while RmlUi runs (ui.rmlui);
// without it the window stays as it was
class pause_menu
{
  public:
	// Keyboard: the keys of the driver, which the page of the controls lists
	explicit pause_menu(keyboard_input const &Keyboard);
	~pause_menu();
	pause_menu(pause_menu const &) = delete;
	pause_menu &operator=(pause_menu const &) = delete;

	// the menu brought up to date. Paused: the simulation is paused (the window of imgui would be shown).
	// returns: whether the menu shows the pause, in place of the window of imgui
	bool update(bool Paused);
	// a key, before the simulation gets it. true: the menu took it (escape goes back from a page of the menu to the first one,
	// on the first one it goes on to the simulation and ends the pause)
	bool on_key(int Key, int Action);

  private:
	struct state;
	// the document is loaded the first time the simulation is paused with RmlUi running
	bool open();

	keyboard_input const *m_keyboard;
	std::unique_ptr<state> m_state;
	bool m_failed{false}; // the document couldn't be loaded, it isn't tried again
	bool m_escape{false}; // the menu took the press of escape, it takes its release too
};
