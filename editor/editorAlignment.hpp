/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/

#pragma once

#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "world/Segment.h"

namespace alignment
{

enum class transition_shape
{
	cubic_parabola,
	clothoid
};

struct limits
{
	double gauge{1500.0};
	double unbalanced{0.6};
	double cant_max{150.0};
	double cant_rate{35.0};
	double ramp_factor{10.0};
	double jerk{0.3};
	double vertical_factor{0.4};
	double vertical_min{2000.0};
	double tangent_min_time{0.0};
};

struct vertex
{
	glm::dvec2 position{0.0};
	double offset{100.0};
	double radius{1000.0};
	double transition_in{0.0};
	double transition_out{0.0};
	double cant{0.0};
	bool auto_elevation{true};
	double elevation{0.0};
	double vertical_radius{0.0};
	bool reverse_turn{false};
};

struct design
{
	glm::dvec3 start{0.0};
	glm::dvec3 end{0.0};
	glm::dvec2 start_direction{0.0, 1.0};
	glm::dvec2 end_direction{0.0, 1.0};
	double start_grade{0.0};
	double end_grade{0.0};
	double start_radius{0.0};
	double end_radius{0.0};
	std::vector<vertex> vertices;
	transition_shape shape{transition_shape::cubic_parabola};
	double speed{100.0};
	limits norms;
};

struct recommendation
{
	double cant_equilibrium{0.0};
	double cant_min{0.0};
	double cant{0.0};
	double radius_min{0.0};
	double transition{0.0};
	double vertical_radius{0.0};
};
recommendation recommend(double const Speed, double const Radius, limits const &Limits);
double transition_length(double const Speed, double const Radius, double const Cant, limits const &Limits);
double unbalanced_acceleration(double const Speed, double const Radius, double const Cant, limits const &Limits);

enum class element_kind
{
	straight,
	transition_in,
	arc,
	transition_out
};

struct element
{
	element_kind kind{element_kind::straight};
	int vertex{-1};
	double chainage{0.0};
	double length{0.0};
	double radius{0.0};
	double transition{0.0};
	int turn{0};
	double cant{0.0};
	glm::dvec2 origin{0.0};
	glm::dvec2 direction{0.0, 1.0};
	glm::dvec2 normal{0.0};
};

struct curve_report
{
	int vertex{-1};
	double radius{0.0};
	double transition_in{0.0};
	double transition_out{0.0};
	double deflection{0.0};
	double arc_length{0.0};
	double tangent_in{0.0};
	double tangent_out{0.0};
	double unbalanced{0.0};
	double cant_rate_in{0.0}, cant_rate_out{0.0};
	double jerk_in{0.0}, jerk_out{0.0};
	recommendation recommended;
};

struct profile_point
{
	double chainage{0.0};
	double elevation{0.0};
	double radius{0.0};
	double tangent{0.0};
};

struct result
{
	bool valid{false};
	transition_shape shape{transition_shape::cubic_parabola};
	double gauge{1500.0};
	std::vector<std::string> errors;
	std::vector<std::string> warnings;
	std::vector<glm::dvec2> vertices;
	std::vector<double> vertex_chainages;
	std::vector<double> vertex_elevations;
	std::vector<element> elements;
	std::vector<curve_report> curves;
	std::vector<profile_point> profile;
	double length{0.0};
};

result compute(design const &Design);
bool tangent_intersection(design const &Design, glm::dvec2 &Point);
bool collinear(design const &Design);

struct station
{
	glm::dvec3 position{0.0};
	glm::dvec3 direction{0.0};
	double cant{0.0};
	int turn{0};
};
station evaluate(result const &Result, double const Chainage);
double elevation(result const &Result, double const Chainage);
double grade(result const &Result, double const Chainage);

std::size_t minimum_pieces(result const &Result, design const &Design);
std::vector<segment_data> pieces(result const &Result, design const &Design, std::size_t const Count);

}
