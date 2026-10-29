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
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "world/Segment.h"

// plan (x, z) geometry and cubic Bezier curves of the paths, shared by the track tools
namespace geometry
{

// m, the longest path of a track the editor lays or leaves after a change: longer ones are cut into equal pieces
double constexpr longest_path{400.0};


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
inline double heading_of(glm::dvec2 const &Direction)
{
	auto const degrees{glm::degrees(std::atan2(Direction.x, Direction.y))};
	return degrees < 0.0 ? degrees + 360.0 : degrees;
}

inline double cross(glm::dvec2 const &Direction, glm::dvec2 const &Point)
{
	return Direction.x * Point.y - Direction.y * Point.x;
}

// where the line from A along AlongA crosses the one from B along AlongB, as the distances along each in the lengths of their
// directions; none where they run alike
inline bool lines_cross(glm::dvec2 const &A, glm::dvec2 const &AlongA, glm::dvec2 const &B, glm::dvec2 const &AlongB, double &First, double &Second)
{
	auto const determinant{cross(AlongA, AlongB)};
	if (std::abs(determinant) < 1e-9)
		return false;
	First = cross(B - A, AlongB) / determinant;
	Second = cross(B - A, AlongA) / determinant;
	return true;
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

inline std::vector<segment_data> arc_pieces(glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Grade, double const Radius, double const Angle, int const Side)
{
	std::vector<segment_data> result;
	double const side{static_cast<double>(Side)};
	glm::dvec2 const centre{plan_of(Point) + glm::dvec2{-Direction.y, Direction.x} * (side * Radius)};
	auto const count{std::max(1, static_cast<int>(std::ceil(Angle / glm::radians(90.0) - 1e-9)))};
	auto const step{Angle / count};
	auto const handle{4.0 / 3.0 * std::tan(step / 4.0) * Radius};
	glm::dvec2 const radial{plan_of(Point) - centre};
	for (int i = 0; i < count; ++i)
	{
		auto const a0{side * step * i};
		auto const a1{side * step * (i + 1)};
		auto const p0{centre + turned(radial, a0)};
		auto const p3{centre + turned(radial, a1)};
		auto const d0{turned(Direction, a0)};
		auto const d3{turned(Direction, a1)};
		auto const y0{Point.y + Grade * Radius * step * i};
		auto const y3{Point.y + Grade * Radius * step * (i + 1)};
		segment_data path;
		path.points[segment_data::point::start] = {p0.x, y0, p0.y};
		path.points[segment_data::point::end] = {p3.x, y3, p3.y};
		path.points[segment_data::point::control1] = {d0.x * handle, Grade * handle, d0.y * handle};
		path.points[segment_data::point::control2] = {-d3.x * handle, -Grade * handle, -d3.y * handle};
		path.radius = static_cast<float>(Radius);
		result.push_back(path);
	}
	return result;
}

// path as a cubic Bezier curve; a path without control points is a straight with evenly spread parameter
struct bezier
{
	glm::dvec3 p0;
	glm::dvec3 p1;
	glm::dvec3 p2;
	glm::dvec3 p3;

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
	// the curvature in the plan at T, 1/m, positive to the left as the curve runs; none where it barely moves
	double plan_curvature(double const T) const
	{
		auto const along{first(T)};
		auto const turning{second(T)};
		auto const speed{std::hypot(along.x, along.z)};
		return speed < 1e-9 ? 0.0 : (along.x * turning.z - along.z * turning.x) / (speed * speed * speed);
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

inline bool has_controls(segment_data const &Path)
{
	return Path.points[segment_data::point::control1] != glm::dvec3{} || Path.points[segment_data::point::control2] != glm::dvec3{};
}

inline segment_data reversed(segment_data const &Path)
{
	segment_data result{Path};
	result.points[segment_data::point::start] = Path.points[segment_data::point::end];
	result.points[segment_data::point::end] = Path.points[segment_data::point::start];
	result.points[segment_data::point::control1] = Path.points[segment_data::point::control2];
	result.points[segment_data::point::control2] = Path.points[segment_data::point::control1];
	result.rolls = {-Path.rolls[1], -Path.rolls[0]};
	return result;
}

inline std::vector<segment_data> straight_piece(glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Grade, double const Length)
{
	if (Length < 0.5)
		return {};
	segment_data path;
	path.points[segment_data::point::start] = Point;
	path.points[segment_data::point::end] = {Point.x + Direction.x * Length, Point.y + Grade * Length, Point.z + Direction.y * Length};
	return {path};
}

inline segment_data part_of(segment_data const &Piece, double const From, double const To)
{
	bezier const curve{Piece};
	segment_data result{Piece};
	result.points[segment_data::point::start] = curve.point(From);
	result.points[segment_data::point::end] = curve.point(To);
	if (has_controls(Piece))
	{
		result.points[segment_data::point::control1] = curve.first(From) * ((To - From) / 3.0);
		result.points[segment_data::point::control2] = -curve.first(To) * ((To - From) / 3.0);
	}
	result.rolls = {glm::mix(Piece.rolls[0], Piece.rolls[1], static_cast<float>(From)), glm::mix(Piece.rolls[0], Piece.rolls[1], static_cast<float>(To))};
	return result;
}

inline std::vector<segment_data> arc_through(glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Grade, glm::dvec2 const &Offset)
{
	auto const along{glm::dot(Offset, Direction)};
	auto const lateral{cross(Direction, Offset)};
	auto const chord{glm::length(Offset)};
	if (along <= 0.0 || chord < 0.5)
		return {};
	if (std::abs(lateral) < 0.005 * chord + 0.05)
		return straight_piece(Point, Direction, Grade, along);
	auto const angle{2.0 * std::atan2(std::abs(lateral), along)};
	return arc_pieces(Point, Direction, Grade, chord / (2.0 * std::sin(angle * 0.5)), angle, lateral > 0.0 ? 1 : -1);
}

inline double hermite(double const Length, double const Height0, double const Grade0, double const Height1, double const Grade1, double const T, double &Grade)
{
	auto const t2{T * T};
	auto const t3{t2 * T};
	Grade = (6.0 * t2 - 6.0 * T) * (Height0 - Height1) / Length + (3.0 * t2 - 4.0 * T + 1.0) * Grade0 + (3.0 * t2 - 2.0 * T) * Grade1;
	return (2.0 * t3 - 3.0 * t2 + 1.0) * Height0 + (t3 - 2.0 * t2 + T) * Length * Grade0 + (3.0 * t2 - 2.0 * t3) * Height1 + (t3 - t2) * Length * Grade1;
}

inline glm::dvec3 level(glm::dvec3 const &Vector)
{
	glm::dvec3 const flat{Vector.x, 0.0, Vector.z};
	return glm::length(flat) > 1e-9 ? glm::normalize(flat) : glm::dvec3{0.0, 0.0, 1.0};
}

// a point inside the polygon, by the number of its edges a ray along +x from it crosses
inline bool point_in_polygon(std::vector<glm::dvec2> const &Polygon, glm::dvec2 const &Point)
{
	bool result{false};
	for (std::size_t i = 0, j = Polygon.size() - 1; i < Polygon.size(); j = i++)
	{
		auto const &a{Polygon[i]};
		auto const &b{Polygon[j]};
		if ((a.y > Point.y) != (b.y > Point.y) && Point.x < a.x + (b.x - a.x) * (Point.y - a.y) / (b.y - a.y))
			result = !result;
	}
	return result;
}

// the square of the side from (West, South), the axes the polygon's, touches the polygon: a corner of it inside the polygon, a
// vertex of the polygon inside it, or an edge crossing an edge. A side of 0: the point inside the polygon
inline bool square_touches(std::vector<glm::dvec2> const &Polygon, double const West, double const South, double const Size)
{
	if (Polygon.size() < 3)
		return false;
	glm::dvec2 const corners[]{{West, South}, {West + Size, South}, {West + Size, South + Size}, {West, South + Size}};
	for (auto const &corner : corners)
		if (point_in_polygon(Polygon, corner))
			return true;
	if (Size <= 0.0)
		return false;
	for (auto const &point : Polygon)
		if (point.x >= West && point.x <= West + Size && point.y >= South && point.y <= South + Size)
			return true;
	// two segments crossing, by the sides each one's ends lie on of the other
	auto const side = [](glm::dvec2 const &A, glm::dvec2 const &B, glm::dvec2 const &P) { return (B.x - A.x) * (P.y - A.y) - (B.y - A.y) * (P.x - A.x); };
	auto const crossing = [&](glm::dvec2 const &A, glm::dvec2 const &B, glm::dvec2 const &C, glm::dvec2 const &D) {
		return ((side(A, B, C) > 0.0) != (side(A, B, D) > 0.0)) && ((side(C, D, A) > 0.0) != (side(C, D, B) > 0.0));
	};
	for (std::size_t i = 0, j = Polygon.size() - 1; i < Polygon.size(); j = i++)
		for (int k = 0; k < 4; ++k)
			if (crossing(Polygon[j], Polygon[i], corners[k], corners[(k + 1) % 4]))
				return true;
	return false;
}

// the squares of a grid of the size, from 0 along both axes, the polygon touches, by their indices along the axes
inline std::vector<std::pair<int, int>> squares_in(std::vector<glm::dvec2> const &Polygon, double const Size)
{
	std::vector<std::pair<int, int>> result;
	if (Polygon.size() < 3 || Size <= 0.0)
		return result;
	glm::dvec2 low{Polygon.front()};
	glm::dvec2 high{Polygon.front()};
	for (auto const &point : Polygon)
	{
		low = glm::min(low, point);
		high = glm::max(high, point);
	}
	auto const first{glm::ivec2{static_cast<int>(std::floor(low.x / Size)), static_cast<int>(std::floor(low.y / Size))}};
	auto const last{glm::ivec2{static_cast<int>(std::floor(high.x / Size)), static_cast<int>(std::floor(high.y / Size))}};
	for (int y = first.y; y <= last.y; ++y)
		for (int x = first.x; x <= last.x; ++x)
			if (square_touches(Polygon, x * Size, y * Size, Size))
				result.emplace_back(x, y);
	return result;
}

} // namespace geometry
