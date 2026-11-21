/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/uilayer.h"

namespace ui
{
// the weather in the physics of the vehicles (McZapkie/weatherphysics.h): the settings of physics.weather.* of eu07.ini changed while
// the simulation runs, what lies on the rails in the open (set by hand to try it out), and what the weather does to the vehicle
// driven now: the adhesion of the rails under it, its brakes, the wind and the air in its drag, the contact of its pantographs
class weatherphysics_panel : public ui_panel
{
	void render_settings();
	void render_rails();
	void render_vehicle();

  public:
	weatherphysics_panel();

	void render_contents() override;
};
} // namespace ui
