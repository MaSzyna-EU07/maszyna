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

namespace ui
{
class map_panel;
}

// the map over the whole screen made with RmlUi (Tab while RmlUi runs), from ui/map.rml in the game folder or the copy built
// into the program: the image of the map drawn by the map of the ui (ui::map_panel, which the opengl renderer draws), under the
// bar of the map (the zoom, the view panned by hand or following the camera or the vehicle), the legend of its colours and the
// names of the trains. dragged with the mouse, zoomed with the wheel; the signals, the switches and the tracks clicked with the
// right button open the windows of the map
class map_screen
{
  public:
	explicit map_screen(ui::map_panel &Map);
	~map_screen();
	map_screen(map_screen const &) = delete;
	map_screen &operator=(map_screen const &) = delete;

	// Shown: the map is wanted on screen. returns: whether RmlUi shows it
	bool update(bool Shown);
	// the image of the map under the document, and the windows of the objects clicked; in the frame of imgui, before the
	// documents of RmlUi are drawn
	void draw();
	// the player closed the map with its button, since the last call
	bool take_closed();

  private:
	struct state;
	bool open();

	ui::map_panel *m_map;
	std::unique_ptr<state> m_state;
	bool m_failed{false}; // the document couldn't be loaded, it isn't tried again
};
