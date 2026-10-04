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

#include "world/Segment.h"

// plan (x, z) geometry and cubic Bezier curves of the paths, shared by the track tools
namespace geometry
{

// points of the paths closer than this are taken for the same point
double constexpr same_point{0.05};

inline glm::dvec2 plan_of(glm::dvec3 const &Point)
{
	return {Point.x, Point.z};
}

inline double plan_distance(glm::dvec3 const &A, glm::dvec3 const &B)
{
	return std::hypot(A.x - B.x, A.z - B.z);
}

// z component of the 3d cross product; for a unit Direction it's the offset of the Point to its left
inline double cross(glm::dvec2 const &Direction, glm::dvec2 const &Point)
{
	return Direction.x * Point.y - Direction.y * Point.x;
}

inline glm::dvec2 turned(glm::dvec2 const &Vector, double const Angle)
{
	auto const c{std::cos(Angle)};
	auto const s{std::sin(Angle)};
	return {Vector.x * c - Vector.y * s, Vector.x * s + Vector.y * c};
}

inline double signed_angle(glm::dvec2 const &From, glm::dvec2 const &To)
{
	return std::atan2(cross(From, To), glm::dot(From, To));
}

// slope of the direction, rise over the run in the plan
inline double grade_of(glm::dvec3 const &Direction)
{
	return Direction.y / std::max(1e-9, std::hypot(Direction.x, Direction.z));
}

// path as a cubic Bezier curve; a path without control points is a straight with evenly spread parameter
struct bezier
{
	glm::dvec3 p0, p1, p2, p3;

	explicit bezier(segment_data const &Path)
	{
		p0 = Path.points[segment_data::point::start];
		p3 = Path.points[segment_data::point::end];
		auto const &control1{Path.points[segment_data::point::control1]};
		auto const &control2{Path.points[segment_data::point::control2]};
		if (control1 == glm::dvec3{} && control2 == glm::dvec3{})
		{
			p1 = glm::mix(p0, p3, 1.0 / 3.0);
			p2 = glm::mix(p0, p3, 2.0 / 3.0);
		}
		else
		{
			p1 = p0 + control1;
			p2 = p3 + control2;
		}
	}
	glm::dvec3 point(double const T) const
	{
		auto const u{1.0 - T};
		return u * u * u * p0 + 3.0 * u * u * T * p1 + 3.0 * u * T * T * p2 + T * T * T * p3;
	}
	glm::dvec3 first(double const T) const
	{
		auto const u{1.0 - T};
		return 3.0 * u * u * (p1 - p0) + 6.0 * u * T * (p2 - p1) + 3.0 * T * T * (p3 - p2);
	}
	glm::dvec3 second(double const T) const
	{
		return 6.0 * (1.0 - T) * (p2 - 2.0 * p1 + p0) + 6.0 * T * (p3 - 2.0 * p2 + p1);
	}
	double plan_length(int const Steps = 32) const
	{
		double length{0.0};
		auto previous{plan_of(p0)};
		for (int i = 1; i <= Steps; ++i)
		{
			auto const next{plan_of(point(static_cast<double>(i) / Steps))};
			length += glm::distance(previous, next);
			previous = next;
		}
		return length;
	}
	// parameter at specified plan distance from the start
	double parameter(double const Distance, int const Steps = 256) const
	{
		double length{0.0};
		auto previous{plan_of(p0)};
		for (int i = 1; i <= Steps; ++i)
		{
			auto const next{plan_of(point(static_cast<double>(i) / Steps))};
			auto const step{glm::distance(previous, next)};
			if (length + step >= Distance)
				return (i - 1 + (step > 1e-12 ? (Distance - length) / step : 0.0)) / Steps;
			length += step;
			previous = next;
		}
		return 1.0;
	}
};

// distance between the ends of the path, in the plan
inline double chord_length(segment_data const &Path)
{
	return glm::length(plan_of(Path.points[segment_data::point::end] - Path.points[segment_data::point::start]));
}

} // namespace geometry
