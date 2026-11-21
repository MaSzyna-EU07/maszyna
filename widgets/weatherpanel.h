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

class world_weather;
class snow_cover;

namespace ui
{

// controls of the weather of the simulation (environment/weather.h): a report or weather entries to apply, the layers of clouds,
// the fog, the precipitation and the wind at the clouds. what is changed applies to this computer at once, without being sent
// over the network. returns true when the weather was changed
bool weather_controls(world_weather &Weather);
// controls of the snow lying (environment/snowcover.h): its depth, its drifts and how loose it is, set by hand (the drifts turned
// to the Wind at the ground blowing now), and how much faster than the simulation it goes, to watch it pile up, drift and melt
void snow_controls(snow_cover &Snow, glm::vec2 const &Wind);

} // namespace ui
