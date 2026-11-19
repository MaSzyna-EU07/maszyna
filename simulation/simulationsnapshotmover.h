/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "simulation/simulationsnapshot.h"

class TMoverParameters;

// the part of a simulation snapshot held by the physics of a vehicle (TMoverParameters): what it carries, how it moves,
// what its machinery is set to and the air in its brakes. the list of what's kept is in the .cpp; it's what a train
// needs to carry on as it was, not every variable of the physics
namespace simulation::snapshot {

// adds the state of the vehicle to its entry
void capture_mover(record &Entry, TMoverParameters const &Mover);
// sets the vehicle to the state of the entry; what the entry doesn't have stays as the vehicle has it.
// the couplings aren't touched, they belong to the pair of vehicles (see plan_couplings())
void apply_mover(TMoverParameters &Mover, record const &Entry);

} // namespace simulation::snapshot
