/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "world/Segment.h"

// Railway alignment design: horizontal alignment defined by a polygon of tangents, with a circular curve
// and transition curves fitted at each intersection point (vertex), and a vertical profile made of grades
// joined with parabolic vertical curves.
//
// Plan coordinates are (x, z) of the scene; heights are scene y. Design values follow the usual railway
// conventions: speed in km/h, cant in mm, radii and lengths in metres. All the limits are configurable.
namespace alignment
{

enum class transition_shape
{
	cubic_parabola, // y = x^3 / (6RL), represented exactly by a single cubic Bezier
	clothoid        // curvature linear along the length, approximated by cubic pieces
};

// permissible values used to recommend and verify the design
struct limits
{
	double gauge{1500.0};          // distance between rail axes, mm
	double unbalanced{0.6};        // permissible unbalanced lateral acceleration, m/s2
	double cant_max{150.0};        // maximum cant, mm
	double cant_rate{35.0};        // permissible rate of cant change along transition, mm/s
	double ramp_factor{10.0};      // cant ramp inclination limited to 1 / (factor * V)
	double jerk{0.3};              // permissible rate of change of unbalanced acceleration, m/s3
	double vertical_factor{0.4};   // minimum vertical curve radius = factor * V^2
	double vertical_min{2000.0};   // absolute minimum vertical curve radius, m
	double tangent_min_time{0.0};  // minimum length of a straight between curves expressed as travel time, s (0: not checked)
};

// intersection point of two tangents of the horizontal alignment, with the curve fitted there
struct vertex
{
	glm::dvec2 position{0.0}; // plan position of an unconstrained vertex
	double offset{100.0};     // first/last vertex: distance from the fixed end along its tangent
	double radius{1000.0};
	double transition_in{0.0};
	double transition_out{0.0};
	double cant{0.0};            // mm
	bool auto_elevation{true};   // elevation interpolated between the fixed ends
	double elevation{0.0};       // height of the profile at the vertex
	double vertical_radius{0.0}; // 0: minimum radius from the limits
};

struct design
{
	// fixed ends of the designed fragment, with plan directions pointing along the route
	glm::dvec3 start{0.0};
	glm::dvec3 end{0.0};
	glm::dvec2 start_direction{0.0, 1.0};
	glm::dvec2 end_direction{0.0, 1.0};
	double start_grade{0.0}; // grade of the adjoining track, for continuity check
	double end_grade{0.0};
	double start_radius{0.0}; // plan radius of the adjoining track at the joint, 0 for straight
	double end_radius{0.0};
	std::vector<vertex> vertices;
	transition_shape shape{transition_shape::cubic_parabola};
	double speed{100.0}; // design speed, km/h
	limits norms;
};

// values derived from the design speed for a curve of given radius
struct recommendation
{
	double cant_equilibrium{0.0}; // cant compensating the lateral acceleration fully, mm
	double cant_min{0.0};         // smallest cant keeping the unbalanced acceleration within the limit, mm
	double cant{0.0};             // proposed cant, mm
	double radius_min{0.0};       // smallest radius for the design speed with maximum cant, m
	double transition{0.0};       // minimum transition length for the proposed cant, m
	double vertical_radius{0.0};  // minimum vertical curve radius, m
};
recommendation recommend(double const Speed, double const Radius, limits const &Limits);
// minimum transition length for given speed, cant and radius
double transition_length(double const Speed, double const Radius, double const Cant, limits const &Limits);
// unbalanced lateral acceleration, m/s2
double unbalanced_acceleration(double const Speed, double const Radius, double const Cant, limits const &Limits);

enum class element_kind
{
	straight,
	transition_in,
	arc,
	transition_out
};

// single geometric element of the computed alignment
struct element
{
	element_kind kind{element_kind::straight};
	int vertex{-1};        // vertex the curve element belongs to
	double chainage{0.0};  // distance from the start of the fragment to the start of the element
	double length{0.0};
	double radius{0.0};    // curve radius (also the final radius of a transition)
	double transition{0.0}; // transition length
	int turn{0};           // +1/-1 direction of the curve, 0 for straights
	double cant{0.0};      // full cant of the curve, mm
	// geometric frame: straight - start point and direction; arc - centre, radius vector to its start point
	// and direction at the start; transition_in - its start point (TS) with the tangent direction and inner normal;
	// transition_out - its end point (ST) with the direction pointing backwards and inner normal
	glm::dvec2 origin{0.0};
	glm::dvec2 direction{0.0, 1.0};
	glm::dvec2 normal{0.0};
};

// verification of a single curve against the limits
struct curve_report
{
	int vertex{-1};
	double deflection{0.0};  // angle between the tangents, radians
	double arc_length{0.0};
	double tangent_in{0.0};  // distance from the vertex to the start of the curve
	double tangent_out{0.0}; // distance from the vertex to the end of the curve
	double unbalanced{0.0};  // m/s2
	double cant_rate_in{0.0}, cant_rate_out{0.0}; // mm/s
	double jerk_in{0.0}, jerk_out{0.0};           // m/s3
	recommendation recommended;
};

struct profile_point
{
	double chainage{0.0};
	double elevation{0.0};
	double radius{0.0};  // vertical curve radius at the grade break
	double tangent{0.0}; // vertical curve tangent length
};

struct result
{
	bool valid{false};
	transition_shape shape{transition_shape::cubic_parabola};
	double gauge{1500.0};
	std::vector<std::string> errors;
	std::vector<std::string> warnings;
	std::vector<glm::dvec2> vertices; // resolved plan positions of the vertices
	std::vector<double> vertex_chainages; // chainage of the middle of each vertex curve
	std::vector<double> vertex_elevations; // profile elevation at each vertex
	std::vector<element> elements;
	std::vector<curve_report> curves;
	std::vector<profile_point> profile;
	double length{0.0};
};

result compute(design const &Design);
// intersection of the tangents of the fixed ends, if it lies ahead of both ends
bool tangent_intersection(design const &Design, glm::dvec2 &Point);
// checks whether the fixed ends lie on a common straight line
bool collinear(design const &Design);

// position, direction of travel (unit, 3d), cant and curvature direction at specified chainage
struct station
{
	glm::dvec3 position{0.0};
	glm::dvec3 direction{0.0};
	double cant{0.0};
	int turn{0};
};
station evaluate(result const &Result, double const Chainage);
// profile height and grade at specified chainage
double elevation(result const &Result, double const Chainage);
double grade(result const &Result, double const Chainage);

// smallest number of cubic pieces representing the alignment with the required accuracy
std::size_t minimum_pieces(result const &Result, design const &Design);
// splits the alignment into at least Count cubic pieces (the longest pieces are divided further to reach the count),
// in the order of the route, as path source data with absolute end points and relative control vectors
std::vector<segment_data> pieces(result const &Result, design const &Design, std::size_t const Count);

} // namespace alignment
