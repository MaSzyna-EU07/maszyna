/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/aipower.h"

#include <format>

namespace ai_power {

stall check(state const &State) {
	if (false == State.collector || true == State.power) {
		return stall::none;
	}
	if (false == State.standing || false == State.pantographs_up) {
		return stall::waiting;
	}
	return State.wire ? stall::no_voltage : stall::no_wire;
}

std::optional<stall> reporter::update(stall const Stall, double const Now) {
	if (Stall == stall::none) {
		m_since.reset();
		m_reported = false;
		return std::nullopt;
	}
	if (Stall == stall::waiting) {
		m_since.reset();
		return std::nullopt;
	}
	if (false == m_since.has_value()) {
		m_since = Now;
	}
	if (true == m_reported || Now - *m_since < report_delay) {
		return std::nullopt;
	}
	m_reported = true;
	return Stall;
}

std::string message(stall const Stall, std::string const &Name, glm::dvec3 const &Position) {
	auto const reason{
		Stall == stall::no_wire ?
			"no overhead wire above its raised pantographs" :
			"no voltage in the overhead wire at its raised pantographs"};
	return std::format("AI [{}]: can't start, {} (at {:.2f}, {:.2f}, {:.2f}); it waits for the voltage", Name, reason, Position.x, Position.y, Position.z);
}

} // namespace ai_power
