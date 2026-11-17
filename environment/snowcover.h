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

// what the weather gives the snow cover for a step of the simulation
struct snow_weather
{
	float temperature{0.f}; // air temperature, degrees C
	float snowfall{0.f}; // how heavily the snow falls, 0..1 (the intensity of the precipitation)
	float rainfall{0.f}; // how heavily the rain falls, 0..1
	glm::vec2 wind{0.f}; // the wind at the ground as it blows, gusts included, world x and z of where it blows to, m/s
	float fog{0.f}; // how thick the fog at the ground is, 0..1 (rime grows in it below zero)
	float humidity{0.85f}; // relative humidity of the air, 0..1
	float wet_bulb{0.f}; // its wet-bulb temperature, degrees C: what the falling snow is like
};

// the snow lying on the ground and on the scenery, and the ice on the wires, as the weather lays it, the wind blows it about and
// the warmth melts it. the simulation keeps it (world_weather::snow()), the renderers draw it: the depth over open flat ground,
// how far the wind has worked it into drifts and from which side, how loose and how wet it is, and the snow stuck to the faces
// turned to the wind. the drifts themselves (behind what stands in the wind, in the hollows and the cuttings, blown off the crests)
// are worked out by the renderer from the shape of the scenery around the camera, with the wind kept here.
// the numbers are of the usual empirical rules: the density of new snow by the temperature (Hedstrom and Pomeroy), melting by
// the degree-day, the threshold of the wind carrying snow along the ground (about 4 m/s for new dry snow, more for old or wet)
// and the carriage growing with the cube of the wind over it (Pomeroy and Gray)
class snow_cover
{
  public:
	// no snow
	void reset();
	// snow lying from the start, or set by hand: Depth over open flat ground, m; Drift: how far the wind has worked it into drifts,
	// 0..1; Wind: the wind that did it, world x and z of where it blew to, m/s; Fresh: share of it the wind can still carry, 0..1
	void set(float const Depth, float const Drift, glm::vec2 const &Wind, float const Fresh);
	// a step of Deltatime seconds of the simulation in the weather given
	void update(double const Deltatime, snow_weather const &Weather);

	// depth over open flat ground, m
	float depth() const
	{
		return m_depth;
	}
	// share of the snow loose enough for the wind to carry, 0..1: new dry snow 1, old or wet or crusted 0
	float fresh() const
	{
		return m_fresh;
	}
	// how wet the snow is, 0..1: dry and powdery 0, slush 1
	float wetness() const
	{
		return m_wetness;
	}
	// how far the wind has worked the snow into drifts, 0..1
	float drift() const
	{
		return m_drift;
	}
	// the wind which made the drifts: world x and z of where it blew to, its length its typical speed, m/s
	glm::vec2 const &drift_wind() const
	{
		return m_driftwind;
	}
	// the wind at the ground now, with its gusts, world x and z of where it blows to, m/s
	glm::vec2 const &wind() const
	{
		return m_wind;
	}
	// how much snow the wind carries along the ground now, 0..1 (drifting and blowing snow)
	float blowing() const
	{
		return m_blowing;
	}
	// how heavily the snow falls now, 0..1
	float falling() const
	{
		return m_falling;
	}
	// how much snow is stuck to the faces turned to the wind (poles, signals, the fronts of the vehicles), 0..1, and the wind
	// which stuck it there, as drift_wind()
	float plaster() const
	{
		return m_plaster;
	}
	glm::vec2 const &plaster_wind() const
	{
		return m_plasterwind;
	}
	// the ice on the wires of the catenary and of the lines, the radial thickness, m: the glaze of freezing rain and drizzle, the
	// rime of a freezing fog and a sleeve of wet snow; and its density, kg/m3 (glaze about 900, rime 200..600, wet snow 400..700),
	// for the weight on the wires and the contact of the pantograph to be made of later
	float wire_ice() const
	{
		return m_wireice;
	}
	float wire_ice_density() const
	{
		return m_wireicedensity;
	}
	// the snow goes this many times faster than the time of the simulation (to see it pile up and drift, from the debug window)
	float time_scale{1.f};

  private:
	void fall(double const Deltatime, snow_weather const &Weather);
	void melt(double const Deltatime, snow_weather const &Weather);
	void carry(double const Deltatime, snow_weather const &Weather);
	void stick(double const Deltatime, snow_weather const &Weather);
	void ice(double const Deltatime, snow_weather const &Weather);

	float m_depth{0.f};
	float m_density{100.f}; // kg/m3 of the snow lying
	float m_fresh{0.f};
	float m_wetness{0.f};
	float m_drift{0.f};
	glm::vec2 m_driftwind{0.f, 6.f};
	glm::vec2 m_wind{0.f};
	float m_blowing{0.f};
	float m_falling{0.f};
	float m_plaster{0.f};
	glm::vec2 m_plasterwind{0.f, 6.f};
	float m_wireice{0.f};
	float m_wireicedensity{900.f};
};
