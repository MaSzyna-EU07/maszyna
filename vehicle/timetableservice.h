/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>

#include "world/timetableflags.h"

class TMoverParameters;

// what the AI driver does at a stop and on the way, as the service flags of the timetable ask (eu07/maszyna#95). with
// no flags each answer is the one the AI gave before the flags
namespace timetable {

// whether the driver opens the doors of the consist from the cab at a stop, Vehicle being the one he drives: the doors
// the conductor or the driver opens, and with the central door opening also the ones of mixed control, which are
// otherwise left to the passengers
bool opens_doors(TMoverParameters const &Vehicle, service_flags const &Flags);

// the state the driver sets the doorsteps to before he opens the doors at a stop, none when he leaves them as they are
std::optional<bool> doorstep(service_flags const &Flags);

// whether a conductor serves the train: gives the departure signal (by radio or by voice) and keeps the first door
// open for himself until the departure
bool has_conductor(service_flags const &Flags);

// whether the driver makes the running brake test after the departure
bool makes_brake_test(service_flags const &Flags);

} // namespace timetable
