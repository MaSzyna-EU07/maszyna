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
#include <cmath>
#include <glm/glm.hpp>

// the room a road leaving the side of a roundabout takes of the ring, seen from above. on a straight road the edge of the road
// leaving meets the edge of the road it leaves where the two straight lines cross; the ring bends away from the road leaving it
// to the outside, so there the edges meet sooner; and a road leaving at a slant meets the edge of the ring sooner on the side
// it leans away from than on the side it leans towards
namespace editor_ring
{

// how far along the axis of the ring, from the point a road leaves it, the edge of that road meets the outer edge of the ring.
// Centre: of the ring, Radius: of its axis, Point: where the axis of the road leaving meets the axis of the ring,
// Direction: the way the road leaves, Along: the way along the ring the room is measured, Halfwidth: of the ring,
// Branchhalfwidth: of the road leaving. returns: the distance along the axis of the ring, 0 if the edges cross before they get
// past the point the road leaves, or a negative value if there's no such point: the road doesn't leave towards the outside,
// or it leans so far that the edge on that side stays outside the ring all along
inline double edge_room(glm::dvec2 const &Centre, double const Radius, glm::dvec2 const &Point, glm::dvec2 const &Direction, glm::dvec2 const &Along, double const Halfwidth,
                        double const Branchhalfwidth)
{
	if (Radius <= 0.0 || glm::length(Direction) < 1e-9 || glm::length(Point - Centre) < 1e-9)
		return -1.0;
	auto const direction{glm::normalize(Direction)};
	auto const outwards{glm::normalize(Point - Centre)};
	if (glm::dot(direction, outwards) < 1e-6)
		return -1.0;
	// the edge of the road leaving on the side the room is measured for
	glm::dvec2 side{-direction.y, direction.x};
	if (glm::dot(side, Along) < 0.0)
		side = -side;
	auto const start{Point - Centre + side * Branchhalfwidth};
	auto const edge{Radius + Halfwidth};
	// start + direction * t on the circle of the outer edge, where the edge leaves it going out. a road wide enough, or leaning
	// far enough, has the start of that edge outside the circle already: the edges crossed before it
	auto const b{glm::dot(start, direction)};
	auto const discriminant{b * b + edge * edge - glm::dot(start, start)};
	if (discriminant < 0.0)
		return -1.0;
	auto const t{-b + std::sqrt(discriminant)};
	auto const meet{start + direction * t};
	// the angle from the point the road leaves to that point, around the centre and the way the room is measured, carried onto the axis
	auto const turn{outwards.x * Along.y - outwards.y * Along.x < 0.0 ? -1.0 : 1.0};
	auto const angle{std::atan2(turn * (outwards.x * meet.y - outwards.y * meet.x), glm::dot(outwards, meet))};
	return std::max(0.0, Radius * angle);
}

} // namespace editor_ring
