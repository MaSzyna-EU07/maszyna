/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>

#include <glm/vec3.hpp>

// the poses of the animated door wings of a vehicle, for the door position of the mover. a wing is posed alike
// in the main model and, when the low poly interior has a submodel of the same name, in the interior as well
namespace door_wings {

// whether the animation with the given flags (TAnim::iFlags, the kind of animation in bits 4-7) moves a door wing
inline bool is_wing(int const Flags) {
	return (Flags & 0xF0) == 0x10;
}

// odd numbered wings (door1, door3...) are on the left side of the vehicle, even numbered on the right
inline bool on_right_side(int const Number) {
	return (Number & 1) == 0;
}

// sliding door (type 1): shifted along the vehicle
inline glm::vec3 slide(float const Position) {
	return {0.f, 0.f, Position};
}

// plug door (type 4): pushed out first, up to Rangeout, then slid along the vehicle
inline glm::vec3 plug(float const Position, float const Rangeout) {
	return {std::min(Position * 2.f, Rangeout), 0.f, std::max(0.f, Position - Rangeout * 0.5f)};
}

// folding door (type 3): the angles of the smaller wing, of the larger wing (its child) and of the step (the
// child of the larger wing)
struct fold_angles {
	float smaller;
	float larger;
	float step;
};

inline fold_angles fold(float const Position) {
	return {Position, -Position - Position, Position};
}

} // namespace door_wings
