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
#include <glm/geometric.hpp>
#include <glm/vec3.hpp>

// the driver figures of the lowpoly interior, mechanik1 and mechanik2, seen from outside the vehicle (eu07/maszyna#535).
// A vehicle with one desk for both directions (401Da, SM31, TEM2...) has two virtual cabs set up on the same spot, so
// the AI changes the cab to drive the other way and its figure turns round. The player usually stays in the cab and
// only moves the reverser, so in such a vehicle the player's figure follows the selected direction of travel instead.
namespace driver_figure {

// what the .mmd says about one driver cab, as far as the figures go
struct cab_place {
	std::string model; // cabNmodel:, empty when not given
	std::optional<glm::vec3> position; // driverNpos:
	std::optional<glm::vec3> sitting_position; // driverNsitpos:

	// where the driver sits: the sitting position, or the position when the cab gives no sitting position, the way
	// the cab is loaded for the player
	std::optional<glm::vec3> seat() const {
		return sitting_position.has_value() ? sitting_position : position;
	}
};

// how close the seats of the two cabs have to be to count as one seat, in metres
inline constexpr float seat_tolerance{0.01f};

// the cabs 1 and 2 of the vehicle are the same desk: the .mmd says they share the place and the model (jointcabs:),
// both name the same cab model and both seat the driver on the same spot. A vehicle with two desks in one cab (SM42,
// SP42) seats the driver on the other side of the cab for the other direction, and a vehicle with two cabs has them at
// its two ends, so neither of them is a single desk.
inline bool single_desk(bool const JointCabs, cab_place const &Cab1, cab_place const &Cab2) {
	if (false == JointCabs || Cab1.model.empty() || Cab1.model == "none" || Cab1.model != Cab2.model) {
		return false;
	}
	auto const seat1{Cab1.seat()};
	auto const seat2{Cab2.seat()};
	if (false == seat1.has_value() || false == seat2.has_value()) {
		return false;
	}
	return glm::distance(*seat1, *seat2) <= seat_tolerance;
}

// which of the figures shows: 1 for mechanik1, -1 for mechanik2, 0 for neither.
// Cab: the occupied cab (CabOccupied: 1, -1 or 0).
// Direction: the selected direction of travel against the couplers (DirAbsolute: 1, -1 or 0).
// FollowsDirection: the figure follows the direction rather than the cab, that is a player drives a single desk vehicle.
// Without a direction selected the figure of the occupied cab shows, as it always did.
inline int shown(int const Cab, int const Direction, bool const FollowsDirection) {
	if (Cab == 0) {
		return 0;
	}
	if (false == FollowsDirection || Direction == 0) {
		return Cab > 0 ? 1 : -1;
	}
	return Direction > 0 ? 1 : -1;
}

} // namespace driver_figure
