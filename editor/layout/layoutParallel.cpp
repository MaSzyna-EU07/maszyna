/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutParallel.hpp"
#include "editor/editorGauge.hpp"
#include "editor/editorGeometry.hpp"

#include <cmath>
#include <format>

namespace layout
{

namespace
{

// m the arc of a curve is moved in from its tangents by a transition of the length: the shift of the clothoid, L^2 / 24 R, as the cubic
// parabola has it too
double shift_of(double const Radius, double const Transition)
{
	return Radius > 0.0 ? Transition * Transition / (24.0 * Radius) : 0.0;
}

// the transition giving the shift at the radius
double transition_of(double const Radius, double const Shift)
{
	return Shift > 0.0 ? std::sqrt(24.0 * Radius * Shift) : 0.0;
}

} // namespace

parallel_plan parallel_line(network const &Network, line_id const Line, double const Offset, bool const Widen)
{
	parallel_plan result;
	auto const *line{Network.find(Line)};
	if (line == nullptr)
	{
		result.errors.push_back(std::format("there's no line {}", number(Line)));
		return result;
	}
	auto const *start{Network.find(line->start)};
	auto const *end{Network.find(line->end)};
	if (start == nullptr || end == nullptr || false == line->solved.valid())
	{
		result.errors.push_back(std::format("line {} isn't laid", number(Line)));
		return result;
	}
	if (std::abs(Offset) < 0.1)
	{
		result.errors.push_back(std::format("a spacing of {:.2f} m", std::abs(Offset)));
		return result;
	}
	// the polygon of the tangents and the directions of its legs
	std::vector<glm::dvec2> corners{{start->position.x, start->position.z}};
	for (auto const &vertex : line->shape.vertices)
		corners.push_back(vertex.point);
	corners.push_back({end->position.x, end->position.z});
	std::vector<glm::dvec2> legs;
	for (std::size_t i = 0; i + 1 < corners.size(); ++i)
	{
		auto const leg{corners[i + 1] - corners[i]};
		if (glm::length(leg) < 1e-9)
		{
			result.errors.push_back(std::format("line {} has a leg of no length", number(Line)));
			return result;
		}
		legs.push_back(glm::normalize(leg));
	}
	auto &command{result.command};
	auto const startleft{left_of(legs.front())};
	auto const endleft{left_of(legs.back())};
	command.start = {start->position.x + startleft.x * Offset, start->position.y, start->position.z + startleft.y * Offset};
	command.end = {end->position.x + endleft.x * Offset, end->position.y, end->position.z + endleft.y * Offset};
	auto const spacing{std::abs(Offset)};
	for (std::size_t i = 0; i < line->shape.vertices.size(); ++i)
	{
		auto const &source{line->shape.vertices[i]};
		auto const &before{legs[i]};
		auto const &after{legs[i + 1]};
		auto const deflection{geometry::signed_angle(before, after)};
		vertex made{source};
		made.id = vertex_id::none;
		// where the offset legs meet: the vertex moved along the bisector of its legs
		auto const turn{1.0 + glm::dot(before, after)};
		if (turn < 1e-6)
		{
			result.errors.push_back(std::format("line {} turns back at its vertex {}", number(Line), i + 1));
			return result;
		}
		made.point = source.point + (left_of(before) + left_of(after)) * (Offset / turn);
		if (std::abs(deflection) < 1e-12)
		{
			command.vertices.push_back(made);
			result.spacings.push_back(spacing);
			continue;
		}
		// the new track on the inside of the curve, or on the outside
		auto const inner{(deflection > 0.0) == (Offset > 0.0)};
		auto const radius{source.radius};
		// widened by what the gauge of each track widens by on the side of the other one: the inner track on its outer side, the outer
		// track on its inner side
		double widening{0.0};
		if (Widen)
		{
			auto const inside{inner ? radius - spacing : radius};
			auto const outside{inner ? radius : radius + spacing};
			widening = gauge::curve_widening(std::max(1.0, inside), false) + gauge::curve_widening(outside, true);
		}
		auto const shift_in{shift_of(radius, source.transition_in)};
		auto const shift_out{shift_of(radius, source.transition_out)};
		// the arcs concentric: the centre the same distance from the offset tangents as the arc of the line is from its own, the arc
		// nearer the other track by the widening; on the outside its shift less by it, which a curve with no transition to take it from
		// hasn't, nor one whose transitions are shorter than that
		if (widening > 0.0 && (source.transition_in <= 0.0 || source.transition_out <= 0.0 || (false == inner && std::min(shift_in, shift_out) < widening)))
		{
			result.notes.push_back(std::format("the spacing in the curve at vertex {} isn't widened by {:.0f} mm: its transitions can't take it", i + 1, widening * 1000.0));
			widening = 0.0;
		}
		made.radius = inner ? radius - spacing - widening : radius + spacing + widening;
		if (made.radius < 1.0)
		{
			result.errors.push_back(std::format("the curve at vertex {} would be of a radius of {:.1f} m alongside", i + 1, made.radius));
			return result;
		}
		auto const change{inner ? widening : -widening};
		made.transition_in = source.transition_in > 0.0 ? transition_of(made.radius, shift_in + change) : 0.0;
		made.transition_out = source.transition_out > 0.0 ? transition_of(made.radius, shift_out + change) : 0.0;
		command.vertices.push_back(made);
		result.spacings.push_back(spacing + widening);
	}
	// the grade line as the line's, its breaks where they lie across, the same share of the length along it
	double across{0.0};
	for (std::size_t i = 0; i + 1 < corners.size(); ++i)
		across += glm::distance(i == 0 ? glm::dvec2{command.start.x, command.start.z} : command.vertices[i - 1].point, i + 1 < corners.size() - 1 ? command.vertices[i].point : glm::dvec2{command.end.x, command.end.z});
	double polygon{0.0};
	for (std::size_t i = 0; i + 1 < corners.size(); ++i)
		polygon += glm::distance(corners[i], corners[i + 1]);
	for (auto grade : line->shape.grades)
	{
		grade.chainage *= polygon > 0.0 ? across / polygon : 1.0;
		command.grades.push_back(grade);
	}
	command.transition = line->shape.transition;
	command.ramp = line->shape.ramp;
	command.category = line->category;
	command.chainage_start = line->chainage_start;
	result.valid = true;
	return result;
}

} // namespace layout
