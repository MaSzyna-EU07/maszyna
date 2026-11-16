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

// the hud of the driver made with RmlUi: the driving aid of F1 (the speed, the limits ahead, the controllers, the brakes, the
// alerter) drawn from ui/driverhud.rml in the game folder, or from the copy built into the program when the folder has none.
// shown in place of the panel of the driving aid of imgui while RmlUi runs (ui.rmlui); without it the panel stays as it was
class driver_hud
{
  public:
	driver_hud();
	~driver_hud();
	driver_hud(driver_hud const &) = delete;
	driver_hud &operator=(driver_hud const &) = delete;

	// the hud brought up to date with the simulation. Visible, Expanded: as the panel of the driving aid is set (F1).
	// returns: whether the hud shows the driving aid, in place of the panel of imgui
	bool update(bool Visible, bool Expanded);
	// draws it, in the frame of imgui of the user interface of the driver
	void render() const;

  private:
	struct document;
	// the document is loaded the first time RmlUi runs
	bool open();

	std::unique_ptr<document> m_document;
	bool m_failed{false}; // the document couldn't be loaded, it isn't tried again
};
