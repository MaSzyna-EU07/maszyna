/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string_view>

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
// an engine the starter turns but which hasn't fired yet (TMoverParameters::dizel_FiringCountdown) lets out no smoke either
float vehicle_spawn_count(TMoverParameters const &Mover, bool Ontracks, spawn_rate const &Rate, double Timedelta);

// the part of a steam locomotive a smoke source of its model shows, set in the template of the source (steam:): the blast
// of the exhaust up the chimney, the smoke of the fire, the safety valves blowing off, the cylinder cocks, the exhaust of
// the air pump, the overflow of the injectors, the whistle, the exhaust of the turbo-generator, the steam leaking past the
// glands of the cylinders. none leaves the source to the engine, as for any other vehicle
enum class steam_part { none, exhaust, smoke, safety_valves, cocks, air_pump, injectors, whistle, generator, leaks };
// the part named in a template: exhaust, smoke, safety_valves, cocks, air_pump, injectors, whistle, generator or leaks;
// none for any other name
steam_part steam_part_of(std::string_view Name);

// how the steam engine of the vehicle drives a source of the given part now: the share of the source's spawn rate, and
// the shares of the opacity and of the launch velocity of a particle born now. nothing for a vehicle with no steam engine
struct steam_output {
	float spawn{0.f};
	float opacity{0.f};
	float velocity{1.f};
};
steam_output steam_emission(TMoverParameters const &Mover, steam_part Part);

} // namespace smoke
