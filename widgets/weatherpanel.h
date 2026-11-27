/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

class world_weather;

namespace ui
{

// controls of the weather of the simulation (environment/weather.h): a report or weather entries to apply, the layers of clouds,
// the fog, the precipitation and the wind at the clouds. what is changed applies to this computer at once, without being sent
// over the network. returns true when the weather was changed
bool weather_controls(world_weather &Weather);

} // namespace ui
