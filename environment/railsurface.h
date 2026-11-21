/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// what the weather lays on the heads of the rails and on the surfaces in the open: the water of the rain and the dew, the dirt of a
// dry spell the first water turns into a paste, the ice (freezing rain, the water freezing on them, the frost) and the leaves of the
// autumn. the simulation keeps it (world_weather::rails()) from the same weather the snow cover goes by (snow_weather), the physics
// takes the adhesion of the rails from it (McZapkie/weatherphysics.h) and the shaders the look of the wet and iced surfaces
// (gl::weather_ubs::surface_state). a tunnel keeps its rails dry and clean; the wheels going over a rail take a part of it off
// (TTrack::since_wheels())

#pragma once

#include "environment/snowcover.h"
#include "McZapkie/weatherphysics.h"

class rail_surface
{
  public:
	// nothing on the rails
	void reset();
	// a step of Deltatime seconds in the weather given: Daylight 0 at night .. 1 in the day, Leaves how many leaves fall now, 0..2
	// (the season of the year and physics.weather.leaves)
	void update(double const Deltatime, snow_weather const &Weather, float const Daylight, float const Leaves);
	// the rails as the weather made them over the hours before the start, without the time it takes
	void settle(snow_weather const &Weather, float const Daylight, float const Leaves);

	weather_physics::railhead const &state() const
	{
		return m_state;
	}
	// set by hand (the window Weather physics)
	weather_physics::railhead &state()
	{
		return m_state;
	}

	// how many leaves fall on a day of the year, 0..1: from the end of September, most from the middle of October to the end of
	// November, none past the middle of December
	static float leaf_season(int const Yearday);

	// the rails go this many times faster than the time of the simulation (to see them dry and freeze, from the window)
	float time_scale{1.f};

  private:
	void step(double const Deltatime, snow_weather const &Weather, float const Daylight, float const Leaves);

	weather_physics::railhead m_state;
};
