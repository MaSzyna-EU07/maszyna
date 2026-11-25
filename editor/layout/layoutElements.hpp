/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <glm/glm.hpp>

// The elements of a line of the track geometry (editor-tracks-domain.md §2, "Element"): the straight, the circular arc and the
// transition curves (clothoid, cubic parabola, Bloss), each with its length along it, its curvature along it, its position and heading
// at a distance from its start and the points offset from it. The plan is the scenery's x and z, a curvature above 0 turns left, to
// the side geometry::cross() measures positive. The local forms of the spirals are the ones the route design has always used
// (editorAlignment.cpp calls them), so the two agree to the bit.
namespace layout
{

// a point of a curve in the frame of its start: along the tangent, to its left, and the heading turned so far
struct local_point
{
	double x{0.0};
	double y{0.0};
	double angle{0.0};
};

// the cubic parabola y = x^3 / (6 R L) at the share Parameter of its abscissa Length, turning towards the radius
local_point parabola_point(double const Length, double const Radius, double const Parameter);
// the clothoid from the tangent to the radius over the length, at the share Parameter of the length
local_point clothoid_point(double const Length, double const Radius, double const Parameter);
// a curve whose curvature changes evenly from Start to End over the length (a clothoid, an arc where they're alike), at Distance along it
local_point spiral_point(double const Start, double const End, double const Length, double const Distance);
// the Bloss curve from the curvature Start to End over the length, at Distance along it: the curvature follows 3t^2 - 2t^3
local_point bloss_point(double const Start, double const End, double const Length, double const Distance);
// the point of the cubic Bezier curve of the control points at the parameters A, B and C (de Casteljau's blossom)
glm::dvec2 blossom(std::array<glm::dvec2, 4> const &Points, double const A, double const B, double const C);

// 3t^2 - 2t^3, the share of the change a Bloss curve or a Bloss cant ramp has made at the share t of its length
double bloss_share(double const T);

enum class element_kind
{
	straight,
	arc,
	clothoid,
	bloss,
	cubic_parabola
};

// what the element is in the curve of a vertex, for the names of the tracks made of it
enum class element_role
{
	straight,
	transition_in,
	arc,
	transition_out
};

struct element
{
	element_kind kind{element_kind::straight};
	element_role role{element_role::straight};
	glm::dvec2 origin{0.0}; // the start
	glm::dvec2 direction{0.0, 1.0}; // the heading at the start, unit
	double chainage{0.0}; // of the start, along the line
	double length{0.0}; // m along the curve
	double curvature_start{0.0}; // 1/m, signed, above 0 turning left
	double curvature_end{0.0};
	// a cubic parabola: the abscissa of all of it along its tangent and the radius it leads to (signed); from the tangent into the
	// curve, or out of the curve onto the tangent
	double parabola_x{0.0};
	double parabola_radius{0.0};
	bool entering{true};
	// the id of the vertex whose curve it's a part of; a straight, the one it leads to, 0 for the one leading to the end of the line
	unsigned vertex{0};

	bool operator==(element const &) const = default;

	double end_chainage() const { return chainage + length; }
	// the curvature at the distance from the start
	double curvature(double const Distance) const;
	// the point at the distance from the start, in the frame of the start
	local_point local(double const Distance) const;
	glm::dvec2 position(double const Distance) const;
	glm::dvec2 tangent(double const Distance) const;
	// the point Offset to the left of the axis at the distance, as a track alongside would run
	glm::dvec2 offset_point(double const Distance, double const Offset) const;
	// a transition from a straight to an arc or from an arc to a straight: how far the arc is moved in from the tangent, the shift p of
	// the arc carried on from its curved end; 0 for the other elements
	double shift() const;
};

// the left of a heading, a quarter turn towards where a curvature above 0 turns
inline glm::dvec2 left_of(glm::dvec2 const &Direction)
{
	return {-Direction.y, Direction.x};
}

element make_straight(glm::dvec2 const &Origin, glm::dvec2 const &Direction, double const Length);
element make_arc(glm::dvec2 const &Origin, glm::dvec2 const &Direction, double const Curvature, double const Length);
// a clothoid or a Bloss curve from the curvature Start to End
element make_spiral(element_kind const Kind, glm::dvec2 const &Origin, glm::dvec2 const &Direction, double const Start, double const End, double const Length);
// the cubic parabola of the abscissa X leading to the signed radius, from the tangent (Entering) or onto it
element make_parabola(glm::dvec2 const &Origin, glm::dvec2 const &Direction, double const X, double const Radius, bool const Entering);
// the length along a cubic parabola of the abscissa X and the radius, to the abscissa Upto
double parabola_length(double const X, double const Radius, double const Upto);

} // namespace layout
