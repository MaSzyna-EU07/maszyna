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
#include <string>

#include <glm/glm.hpp>

// an AI driver of an electric vehicle standing with its pantographs up and no voltage at them: past the end of the
// overhead wire, or under a wire which is switched off. no driver can start such a vehicle, it waits in
// TController::PrepareEngine() until the voltage comes, towed back under the wire or the power switched on. the
// driver tells of it in the log, once, instead of waiting without a word (eu07/maszyna#421)
namespace ai_power {

// what the driver knows of the power of the vehicle when it prepares it
struct state {
	bool collector{false}; // the vehicle takes its power from the overhead wire
	bool power{false}; // there is the voltage the vehicle needs, at its pantographs or from the rest of the train
	bool standing{false}; // the vehicle stands
	bool pantographs_up{false}; // a pantograph is raised, with the air to hold it up
	bool wire{false}; // a raised pantograph found a wire above it
};

enum class stall {
	none, // the voltage is there, or the vehicle doesn't take it from the wire
	waiting, // no voltage, but nothing to tell yet: the vehicle moves, or its pantographs aren't up
	no_wire, // standing with the pantographs up and no wire above them
	no_voltage // standing with a pantograph up at a wire which carries no voltage
};

stall check(state const &State);

// how long [s] the vehicle stands with its pantographs up and no voltage before the driver tells of it. the
// pantographs need a few seconds to rise and the voltage a moment to come, a gap under a section insulator is
// shorter still
inline constexpr double report_delay{30.0};

// tells of a stall once: one report while it lasts. the voltage coming back ends it, and the next one is reported
// again; moving, or the pantographs going down, only restarts the count of report_delay
class reporter {
public:
	// one look at the vehicle at the time Now [s]: the stall to report now, if any
	std::optional<stall> update(stall Stall, double Now);

private:
	std::optional<double> m_since; // when the vehicle began to stand with the pantographs up and no voltage
	bool m_reported{false}; // the stall has been reported
};

// the line of the log telling of the stall of the vehicle Name standing at Position
std::string message(stall Stall, std::string const &Name, glm::dvec3 const &Position);

} // namespace ai_power
