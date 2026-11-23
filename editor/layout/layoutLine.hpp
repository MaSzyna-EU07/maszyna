/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/layout/layoutElements.hpp"
#include "editor/norms/norms.hpp"

#include <cstdint>
#include <string>
#include <vector>
#include <glm/glm.hpp>

// The line of the track geometry (editor-tracks-domain.md §2 and §4, "Linia"): the axis of a track between two nodes, given as a tangent
// polygon of vertices between them, each with its radius, its transition curves and its cant, and a grade line of breaks with vertical
// curves; solved into the elements one after another, the cant and the heights as functions of the chainage. The ends of the line are
// its nodes: it has no coordinates of its own there, the polygon starts and ends at the points of the nodes.
namespace layout
{

// the identifiers of the domain: given once, never given again, 0 for none
enum class node_id : std::uint32_t
{
	none = 0
};
enum class line_id : std::uint32_t
{
	none = 0
};
enum class vertex_id : std::uint32_t
{
	none = 0
};
enum class turnout_id : std::uint32_t
{
	none = 0
};

inline std::uint32_t number(node_id const Id) { return static_cast<std::uint32_t>(Id); }
inline std::uint32_t number(line_id const Id) { return static_cast<std::uint32_t>(Id); }
inline std::uint32_t number(vertex_id const Id) { return static_cast<std::uint32_t>(Id); }
inline std::uint32_t number(turnout_id const Id) { return static_cast<std::uint32_t>(Id); }

enum class transition_shape
{
	clothoid,
	cubic_parabola, // the abscissa along the tangent is the transition length given
	bloss
};

// how the cant changes along a transition curve
enum class ramp_shape
{
	linear,
	bloss
};

// a break of the tangents: where they meet, the radius of the arc between them, the transition curves into it and out of it, its cant
struct vertex
{
	vertex_id id{vertex_id::none};
	glm::dvec2 point{0.0}; // plan, the scenery's x and z
	double radius{600.0}; // m
	double transition_in{0.0}; // m
	double transition_out{0.0};
	double cant{0.0}; // mm, leaning into the turn

	bool operator==(vertex const &) const = default;
};

// a break of the grade line: at the chainage, the height there, the radius of the vertical curve rounding it
struct grade_break
{
	double chainage{0.0};
	double elevation{0.0};
	double vertical_radius{5000.0};

	bool operator==(grade_break const &) const = default;
};

// the cant, mm, signed: above 0 leaning left as the line runs, from the start of the stretch to its end, a ramp or a stretch held
struct cant_stretch
{
	double from{0.0};
	double to{0.0};
	double start{0.0};
	double end{0.0};
	ramp_shape shape{ramp_shape::linear};

	bool operator==(cant_stretch const &) const = default;
};

// the grade line: the breaks between the ends, each with the tangent of its vertical curve
struct profile_point
{
	double chainage{0.0};
	double elevation{0.0};
	double vertical_radius{0.0};
	double tangent{0.0}; // m from the break to either end of its vertical curve

	bool operator==(profile_point const &) const = default;
};

// the values of the line at an end, as the line runs from its start to its end
struct end_values
{
	glm::dvec3 position{0.0};
	glm::dvec2 heading{0.0, 1.0};
	double curvature{0.0};
	double cant{0.0}; // mm, signed
	double grade{0.0};

	bool operator==(end_values const &) const = default;
};

// how the curve next to an end of the line meets its node: as the node holds it (the end goes on from a curve, the line was split
// inside one), or from a straight
struct end_condition
{
	glm::dvec3 position{0.0};
	bool continues{false};
	double curvature{0.0}; // as the line runs, of the node where the end continues
	double cant{0.0};
};

struct line_shape
{
	std::vector<vertex> vertices;
	std::vector<grade_break> grades;
	transition_shape transition{transition_shape::clothoid};
	ramp_shape ramp{ramp_shape::linear};

	bool operator==(line_shape const &) const = default;
};

// the line solved: the elements, the cant and the grade line along it; errors where it can't be laid as given
struct solved_line
{
	std::vector<element> elements;
	std::vector<cant_stretch> cant;
	std::vector<profile_point> profile;
	double length{0.0};
	end_values start;
	end_values end;
	std::vector<std::string> errors;

	bool valid() const { return errors.empty() && false == elements.empty(); }
	bool operator==(solved_line const &) const = default;

	// the element at the chainage, the later one at a boundary
	std::size_t element_at(double const Chainage) const;
	glm::dvec2 position(double const Chainage) const;
	glm::dvec2 tangent(double const Chainage) const;
	double curvature(double const Chainage) const;
	// mm, signed; where a stretch begins, its own value
	double cant_at(double const Chainage) const;
	double elevation(double const Chainage) const;
	double grade(double const Chainage) const;
	glm::dvec3 point(double const Chainage) const;
};

// the line laid between the end conditions; the vertices and the breaks as given, the ends from the nodes
solved_line solve(line_shape const &Shape, end_condition const &Start, end_condition const &End);

// the shape of a curve of a vertex in the frame of the tangent point where it starts, heading along x: the elements, and how far the
// tangent points are from the vertex
struct curve_fit
{
	std::vector<element> elements;
	double tangent_in{0.0};
	double tangent_out{0.0};
	double arc{0.0};
	std::string error;
};
// Deflection: signed, above 0 turning left. From and To: the curvatures the transitions start at and end at, signed
curve_fit fit_curve(transition_shape const Shape, double const Deflection, double const Radius, double const In, double const Out, double const From = 0.0, double const To = 0.0);

// the line as the norms describe it (norms::line), the class of the line telling the speed of its elements
norms::line described(solved_line const &Line, norms::line_class const &Class, std::string Name = {});

// the cant, the transition length and the radius the norms prefer for a curve of the radius at the line class, by the profile:
// the regular cant within the preferred deficiency and greatest cant, the transitions long enough for the preferred ramp gradient,
// cant rate and deficiency rate, rounded up to whole metres
struct curve_norms
{
	double cant{0.0};
	double transition{0.0};
	double radius_prefer{0.0};
	double radius_block{0.0};
};
curve_norms preferred_curve(norms::profile_id const Profile, norms::line_class const &Class, double const Radius);

// mm of the cant as degrees of the roll of the track: on the base of the rail heads, 1500 mm
double roll_of(double const Cant);

// the chainage of the point of the line nearest the point of the plan, and how far it is from it (Distance); 0 for a line not laid
double chainage_of(solved_line const &Line, glm::dvec2 const &Point, double *Distance = nullptr);

} // namespace layout
