/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

class TMoverParameters;

// the smoke of a vehicle's engine, let out by the smoke sources of its model (rendering/particles.h)
namespace smoke {

// what a smoke source spawns, before the engine of its vehicle is taken into account
struct spawn_rate {
	float pending{0.f}; // particles left to spawn from the previous updates
	float rate{0.f}; // particles per second
	float fidelity{1.f}; // multiplier of the smoke settings
	float limit{0.f}; // most particles the source can have
};

// number of particles the smoke source of a vehicle has to spawn after Timedelta seconds: the steady rate of the
// source, plus a share which grows with the work of its engine. a vehicle taken off the tracks (Ontracks false: gone through a portal, or waiting in the pool of the road
// traffic) puffs no more, the particles it already let out fade away as usual
float vehicle_spawn_count(TMoverParameters const &Mover, bool Ontracks, spawn_rate const &Rate, double Timedelta);

} // namespace smoke
