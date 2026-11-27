/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "world/RoadSurfaceType.h"

#include <algorithm>
#include <numbers>

// how tight a road the road editor lays out, by what the road is made of
struct road_limits
{
	double tightest_radius; // smallest radius of a bend [m]
	double shortest_road; // shortest road, and shortest piece left of a road cut for a junction [m]
	double sharpest_branch; // smallest angle between two roads leaving a junction [rad]
	// a road with both ends tied to a direction: the least part of the distance between the ends each end has to lead towards the other
	double tied_lead;
	double widest_turn; // a road with its start tied to a direction: the largest turn of the arc leading to the target [rad]
};

namespace editor_road_limits
{

inline constexpr double degree{std::numbers::pi / 180.0};
// a paved road: the limits the editor had for every road before the types
inline constexpr road_limits paved{6.0, 2.0, 30.0 * degree, 0.1, 170.0 * degree};
// a dirt road follows the ground between the fields: a tractor's bend, short stretches between the bends, a fork at a sharp angle,
// a hairpin, an end leading off square to the way to the other end
inline constexpr road_limits dirt{3.0, 1.0, 15.0 * degree, 0.0, 240.0 * degree};
// room the inner edge of a bend keeps from the centre of the bend, as on a roundabout [m]
inline constexpr double inner_edge_room{1.0};

// smallest radius of a bend of a road with these limits, Halfwidth [m] wide on each side of its axis: the limit, and no less than
// leaves the inner edge of the road short of the centre of the bend, where the surface would fold over itself
inline double tightest_bend(road_limits const &Limits, double const Halfwidth)
{
	return std::max(Limits.tightest_radius, Halfwidth + inner_edge_room);
}

} // namespace editor_road_limits

inline constexpr road_limits road_limits_for(road_surface_type const Type)
{
	return Type == road_surface_type::dirt ? editor_road_limits::dirt : editor_road_limits::paved;
}
