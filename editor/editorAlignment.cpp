/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http:
*/

#include "stdafx.h"
#include "editor/editorAlignment.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <functional>

namespace alignment
{

namespace
{

double const kGravity{9.81};
double const kPi{3.14159265358979323846};

double cross(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return A.x * B.y - A.y * B.x;
}

glm::dvec2 perpendicular(glm::dvec2 const &Vector)
{
	return {-Vector.y, Vector.x};
}

glm::dvec2 rotate(glm::dvec2 const &Vector, double const Angle)
{
	auto const c{std::cos(Angle)};
	auto const s{std::sin(Angle)};
	return {Vector.x * c - Vector.y * s, Vector.x * s + Vector.y * c};
}

glm::dvec2 plan(glm::dvec3 const &Point)
{
	return {Point.x, Point.z};
}

double ceil_to(double const Value, double const Step)
{
	return std::ceil(Value / Step - 1e-9) * Step;
}

std::string format(char const *Format, ...)
{
	char buffer[512];
	va_list arguments;
	va_start(arguments, Format);
	std::vsnprintf(buffer, sizeof(buffer), Format, arguments);
	va_end(arguments);
	return buffer;
}

struct local_point
{
	double x{0.0};
	double y{0.0};
	double angle{0.0};
};

local_point transition_point(transition_shape const Shape, double const Length, double const Radius, double const Parameter)
{
	if (Length <= 0.0 || Radius <= 0.0)
		return {};
	if (Shape == transition_shape::cubic_parabola)
	{
		auto const x{Length * Parameter};
		return {x, x * x * x / (6.0 * Radius * Length), std::atan(x * x / (2.0 * Radius * Length))};
	}
	auto const distance{Length * Parameter};
	auto const angle = [&](double const S) { return S * S / (2.0 * Radius * Length); };
	int const steps{std::max(8, static_cast<int>(std::ceil(distance / 0.5)) * 2)};
	double const h{distance / steps};
	double x{0.0}, y{0.0};
	for (int i = 0; i <= steps; ++i)
	{
		double const weight = (i == 0 || i == steps) ? 1.0 : (i % 2 ? 4.0 : 2.0);
		auto const a{angle(i * h)};
		x += weight * std::cos(a);
		y += weight * std::sin(a);
	}
	return {x * h / 3.0, y * h / 3.0, angle(distance)};
}

local_point spiral_point(double const Start, double const End, double const Length, double const Distance)
{
	if (Distance <= 0.0)
		return {};
	auto const heading = [&](double const S) { return Start * S + (End - Start) * S * S / (2.0 * std::max(Length, 1e-9)); };
	int const steps{std::max(8, static_cast<int>(std::ceil(Distance / 0.5)) * 2)};
	double const h{Distance / steps};
	double x{0.0}, y{0.0};
	for (int i = 0; i <= steps; ++i)
	{
		double const weight = (i == 0 || i == steps) ? 1.0 : (i % 2 ? 4.0 : 2.0);
		auto const a{heading(i * h)};
		x += weight * std::cos(a);
		y += weight * std::sin(a);
	}
	return {x * h / 3.0, y * h / 3.0, heading(Distance)};
}

struct compound_curve
{
	std::vector<std::array<double, 3>> segments;
	local_point end;
	double shortened{1.0};
};

compound_curve build_compound(double const Deflection, double const Radius1, double const Radius2, double const Lengthin, double const Lengthmiddle, double const Lengthout, double const Split)
{
	compound_curve result;
	auto const k1{1.0 / Radius1};
	auto const k2{1.0 / Radius2};
	auto const transitions{(k1 * Lengthin + (k1 + k2) * Lengthmiddle + k2 * Lengthout) * 0.5};
	if (transitions > Deflection)
		result.shortened = Deflection / transitions * (1.0 - 1e-9);
	auto const f{result.shortened};
	auto const arcs{std::max(0.0, Deflection - transitions * f)};
	auto const split{std::clamp(Split, 0.0, 1.0)};
	std::array<double, 3> const segments[] = {{0.0, k1, Lengthin * f}, {k1, k1, split * arcs / k1}, {k1, k2, Lengthmiddle * f}, {k2, k2, (1.0 - split) * arcs / k2}, {k2, 0.0, Lengthout * f}};
	double x{0.0}, y{0.0}, heading{0.0};
	for (auto const &segment : segments)
	{
		if (segment[2] < 1e-6)
			continue;
		result.segments.push_back(segment);
		auto const local{spiral_point(segment[0], segment[1], segment[2], segment[2])};
		x += local.x * std::cos(heading) - local.y * std::sin(heading);
		y += local.x * std::sin(heading) + local.y * std::cos(heading);
		heading += local.angle;
	}
	result.end = {x, y, heading};
	return result;
}

void element_point(element const &Element, transition_shape const Shape, double const Distance, glm::dvec2 &Position, glm::dvec2 &Direction)
{
	switch (Element.kind)
	{
	case element_kind::straight:
		Position = Element.origin + Element.direction * Distance;
		Direction = Element.direction;
		break;
	case element_kind::arc:
	{
		auto const angle{Element.turn * Distance / Element.radius};
		Position = Element.origin + rotate(Element.normal, angle);
		Direction = rotate(Element.direction, angle);
		break;
	}
	case element_kind::transition_in:
	{
		auto const local{transition_point(Shape, Element.transition, Element.radius, Distance / Element.transition)};
		Position = Element.origin + Element.direction * local.x + Element.normal * local.y;
		Direction = Element.direction * std::cos(local.angle) + Element.normal * std::sin(local.angle);
		break;
	}
	case element_kind::spiral:
	{
		auto const local{spiral_point(Element.curvature_start, Element.curvature_end, Element.length, Distance)};
		Position = Element.origin + Element.direction * local.x + Element.normal * local.y;
		Direction = Element.direction * std::cos(local.angle) + Element.normal * std::sin(local.angle);
		break;
	}
	case element_kind::transition_out:
	{
		auto const local{transition_point(Shape, Element.transition, Element.radius, 1.0 - Distance / Element.transition)};
		Position = Element.origin + Element.direction * local.x + Element.normal * local.y;
		Direction = -(Element.direction * std::cos(local.angle) + Element.normal * std::sin(local.angle));
		break;
	}
	}
}

double element_cant(element const &Element, double const Distance)
{
	switch (Element.kind)
	{
	case element_kind::transition_in: return Element.cant * Distance / Element.transition;
	case element_kind::transition_out: return Element.cant * (1.0 - Distance / Element.transition);
	case element_kind::arc: return Element.cant;
	case element_kind::spiral:
	{
		auto const curvature{Element.curvature_start + (Element.curvature_end - Element.curvature_start) * Distance / std::max(Element.length, 1e-9)};
		return Element.curvature_peak > 0.0 ? Element.cant * curvature / Element.curvature_peak : 0.0;
	}
	default: return 0.0;
	}
}

element const &element_at(result const &Result, double const Chainage)
{
	auto const lookup = std::upper_bound(Result.elements.begin(), Result.elements.end(), Chainage, [](double const Value, element const &Element) { return Value < Element.chainage; });
	return lookup == Result.elements.begin() ? Result.elements.front() : *std::prev(lookup);
}

glm::dvec2 blossom(std::array<glm::dvec2, 4> const &Points, double const A, double const B, double const C)
{
	glm::dvec2 r[3], s[2];
	for (int i = 0; i < 3; ++i)
		r[i] = glm::mix(Points[i], Points[i + 1], A);
	for (int i = 0; i < 2; ++i)
		s[i] = glm::mix(r[i], r[i + 1], B);
	return glm::mix(s[0], s[1], C);
}

std::size_t element_pieces(element const &Element, transition_shape const Shape, int const Transitions, double const Arcangle)
{
	if (Element.length < 1e-3)
		return 0;
	switch (Element.kind)
	{
	case element_kind::arc:
		return static_cast<std::size_t>(std::max(1.0, std::ceil(Element.length / Element.radius / (Arcangle * kPi / 180.0) - 1e-9)));
	case element_kind::transition_in:
	case element_kind::transition_out:
		return std::max<std::size_t>(std::max(1, Transitions), Shape == transition_shape::cubic_parabola ? 1 : static_cast<std::size_t>(std::max(1.0, std::ceil(Element.length / 20.0 - 1e-9))));
	case element_kind::spiral:
	{
		auto const turn{(Element.curvature_start + Element.curvature_end) * 0.5 * Element.length};
		auto const count{static_cast<std::size_t>(std::max({1.0, std::ceil(turn / (Arcangle * kPi / 180.0) - 1e-9), std::ceil(Element.length / 20.0 - 1e-9)}))};
		return Element.curvature_start != Element.curvature_end ? std::max<std::size_t>(count, std::max(1, Transitions)) : count;
	}
	default:
		return 1;
	}
}

}

double unbalanced_acceleration(double const Speed, double const Radius, double const Cant, limits const &Limits)
{
	if (Radius <= 0.0)
		return 0.0;
	return Speed * Speed / (12.96 * Radius) - kGravity * Cant / Limits.gauge;
}

double transition_length(double const Speed, double const Radius, double const Cant, limits const &Limits)
{
	double length{0.0};
	if (Cant > 0.0)
	{
		length = std::max(length, Speed * Cant / (3.6 * Limits.cant_rate));
		length = std::max(length, Cant / 1000.0 * Limits.ramp_factor * Speed);
	}
	auto const unbalanced{std::max(0.0, unbalanced_acceleration(Speed, Radius, Cant, Limits))};
	length = std::max(length, unbalanced * Speed / (3.6 * Limits.jerk));
	return length > 0.0 ? ceil_to(length, 5.0) : 0.0;
}

recommendation recommend(double const Speed, double const Radius, limits const &Limits)
{
	recommendation result;
	auto const factor{Limits.gauge / (kGravity * 12.96)};
	auto const allowance{Limits.gauge * Limits.unbalanced / kGravity};
	result.radius_min = factor * Speed * Speed / (Limits.cant_max + allowance);
	result.vertical_radius = std::max(Limits.vertical_min, Limits.vertical_factor * Speed * Speed);
	if (Radius <= 0.0)
		return result;
	result.cant_equilibrium = factor * Speed * Speed / Radius;
	result.cant_min = std::max(0.0, result.cant_equilibrium - allowance);
	result.cant = std::min(Limits.cant_max, ceil_to(std::max(result.cant_min, result.cant_equilibrium * 2.0 / 3.0), 5.0));
	result.transition = transition_length(Speed, Radius, result.cant, Limits);
	return result;
}

result compute(design const &Design)
{
	result r;
	r.shape = Design.shape;
	r.gauge = Design.norms.gauge;

	auto const start{plan(Design.start)};
	auto const end{plan(Design.end)};
	auto const startdirection{glm::normalize(Design.start_direction)};
	auto const enddirection{glm::normalize(Design.end_direction)};
	auto const count{Design.vertices.size()};

	r.vertices.resize(count);
	if (count == 0)
	{
		auto const offset{end - start};
		if (std::abs(cross(startdirection, offset)) > 0.01 || glm::dot(startdirection, enddirection) < std::cos(1e-4) || glm::dot(startdirection, offset) <= 0.0)
			r.errors.emplace_back("The ends don't lie on a common straight line, add a vertex");
	}
	else if (count == 1)
	{
		auto const offset{end - start};
		auto const determinant{cross(startdirection, enddirection)};
		if (std::abs(determinant) < 1e-9)
		{
			r.errors.emplace_back("Tangents of the ends are parallel, use two vertices");
		}
		else
		{
			auto const t{cross(offset, enddirection) / determinant};
			auto const u{cross(startdirection, offset) / determinant};
			if ((t <= 0.0 || u <= 0.0) && false == Design.vertices.front().reverse_turn)
				r.errors.emplace_back("Tangents of the ends intersect outside of the fragment, turn the longer way or use two vertices");
			r.vertices[0] = start + startdirection * t;
		}
	}
	else
	{
		for (std::size_t i = 1; i + 1 < count; ++i)
			r.vertices[i] = Design.vertices[i].position;
		r.vertices.front() = start + startdirection * Design.vertices.front().offset;
		r.vertices.back() = end - enddirection * Design.vertices.back().offset;
	}
	if (false == r.errors.empty())
		return r;

	std::vector<glm::dvec2> polygon{start};
	polygon.insert(polygon.end(), r.vertices.begin(), r.vertices.end());
	polygon.push_back(end);
	for (std::size_t i = 0; i + 1 < polygon.size(); ++i)
	{
		if (glm::distance(polygon[i], polygon[i + 1]) < 1e-3)
		{
			r.errors.emplace_back(format("Tangent %zu has zero length", i + 1));
			return r;
		}
	}

	double chainage{0.0};
	auto const push = [&](element Element) {
		Element.chainage = chainage;
		chainage += Element.length;
		r.elements.push_back(Element);
	};
	auto const push_straight = [&](glm::dvec2 const &From, glm::dvec2 const &Direction, double const Length) {
		if (Length <= 1e-6)
			return;
		element straight;
		straight.kind = element_kind::straight;
		straight.origin = From;
		straight.direction = Direction;
		straight.length = Length;
		push(straight);
	};

	struct fitted
	{
		double radius{0.0};
		double transition_in{0.0};
		double transition_out{0.0};
		double tangent_in{0.0};
		double tangent_out{0.0};
		double deflection{0.0};
		bool compound{false};
		double radius2{0.0};
		double transition_middle{0.0};
		std::function<void(double, double &, double &)> tangents;
	};
	std::vector<fitted> fits(count);
	for (std::size_t k = 0; k < count; ++k)
	{
		auto const &vertex{Design.vertices[k]};
		auto &fit{fits[k]};
		fit.radius = std::max(1.0, vertex.radius);
		fit.transition_in = std::max(0.0, vertex.transition_in);
		fit.transition_out = std::max(0.0, vertex.transition_out);
		auto const directionin{glm::normalize(polygon[k + 1] - polygon[k])};
		auto const directionout{glm::normalize(polygon[k + 2] - polygon[k + 1])};
		auto const turn{cross(directionin, directionout)};
		auto const geometric{std::atan2(std::abs(turn), glm::dot(directionin, directionout))};
		auto const reverse{Design.vertices[k].reverse_turn};
		auto const deflection{reverse ? 2.0 * kPi - geometric : geometric};
		int const side{(turn > 0.0) != reverse ? 1 : -1};
		if (deflection < 1e-7)
			continue;
		if (geometric > kPi - 1e-3 || (reverse && geometric < 1e-3))
		{
			r.errors.emplace_back(format("Vertex %zu: a turn of about 180 or 360 degrees needs one more vertex", k + 1));
			return r;
		}
		if (vertex.compound)
		{
			fit.compound = true;
			fit.radius2 = std::max(1.0, vertex.radius2);
			fit.transition_middle = std::max(0.0, vertex.transition_middle);
			fit.deflection = deflection;
			fit.tangents = [&, k, directionin, directionout, side](double const Radius, double &Tangentin, double &Tangentout) {
				auto const &own{fits[k]};
				auto const scale{Radius / own.radius};
				auto const shape{build_compound(own.deflection, Radius, own.radius2 * scale, own.transition_in * scale, own.transition_middle * scale, own.transition_out * scale, Design.vertices[k].split)};
				auto const normalin{perpendicular(directionin) * static_cast<double>(side)};
				auto const end{directionin * shape.end.x + normalin * shape.end.y};
				auto const along{cross(directionout, end) / cross(directionout, directionin)};
				Tangentin = std::max(0.0, along);
				Tangentout = std::max(0.0, glm::dot(end - directionin * along, directionout));
			};
			fit.tangents(fit.radius, fit.tangent_in, fit.tangent_out);
			continue;
		}
		auto const angles = [&](double const Factor) {
			return transition_point(Design.shape, fit.transition_in * Factor, fit.radius, 1.0).angle + transition_point(Design.shape, fit.transition_out * Factor, fit.radius, 1.0).angle;
		};
		if (angles(1.0) > deflection)
		{
			double low{0.0}, high{1.0};
			for (int i = 0; i < 50; ++i)
			{
				auto const middle{(low + high) * 0.5};
				(angles(middle) > deflection ? high : low) = middle;
			}
			r.warnings.emplace_back(format("Vertex %zu: transition curves shortened to %.1f / %.1f m to fit the deflection angle", k + 1, fit.transition_in * low, fit.transition_out * low));
			fit.transition_in *= low;
			fit.transition_out *= low;
		}
		fit.deflection = deflection;
		fit.tangents = [&, k, directionin, directionout, side](double const Radius, double &Tangentin, double &Tangentout) {
			auto const endin{transition_point(Design.shape, fits[k].transition_in, Radius, 1.0)};
			auto const endout{transition_point(Design.shape, fits[k].transition_out, Radius, 1.0)};
			auto const a{Radius + endin.y - Radius * (1.0 - std::cos(endin.angle))};
			auto const b{Radius + endout.y - Radius * (1.0 - std::cos(endout.angle))};
			auto const normalin{perpendicular(directionin) * static_cast<double>(side)};
			auto const normalout{perpendicular(directionout) * static_cast<double>(side)};
			auto const determinant{cross(normalin, normalout)};
			glm::dvec2 const offset{(a * normalout.y - normalin.y * b) / determinant, (normalin.x * b - a * normalout.x) / determinant};
			auto const curvestart{offset - normalin * a - directionin * (endin.x - Radius * std::sin(endin.angle))};
			auto const curveend{offset - normalout * b + directionout * (endout.x - Radius * std::sin(endout.angle))};
			Tangentin = std::max(0.0, -glm::dot(curvestart, directionin));
			Tangentout = std::max(0.0, glm::dot(curveend, directionout));
		};
		fit.tangents(fit.radius, fit.tangent_in, fit.tangent_out);
	}
	std::vector<double> scales(count, 1.0);
	for (std::size_t leg = 0; leg + 1 < polygon.size(); ++leg)
	{
		auto const available{glm::distance(polygon[leg], polygon[leg + 1])};
		auto const required{(leg > 0 ? fits[leg - 1].tangent_out : 0.0) + (leg < count ? fits[leg].tangent_in : 0.0)};
		if (required <= available)
			continue;
		auto const scale{available / required * (1.0 - 1e-9)};
		if (leg > 0)
			scales[leg - 1] = std::min(scales[leg - 1], scale);
		if (leg < count)
			scales[leg] = std::min(scales[leg], scale);
	}
	for (std::size_t k = 0; k < count; ++k)
	{
		if (scales[k] >= 1.0)
			continue;
		auto &fit{fits[k]};
		auto const budgetin{fit.tangent_in * scales[k]};
		auto const budgetout{fit.tangent_out * scales[k]};
		auto const fits_budget = [&](double const Radius) {
			if (transition_point(Design.shape, fit.transition_in, Radius, 1.0).angle + transition_point(Design.shape, fit.transition_out, Radius, 1.0).angle > fit.deflection)
				return false;
			double tangentin, tangentout;
			fit.tangents(Radius, tangentin, tangentout);
			return tangentin <= budgetin && tangentout <= budgetout;
		};
		double low{1e-3};
		double high{fit.radius};
		for (int i = 0; i < 60; ++i)
		{
			auto const middle{(low + high) * 0.5};
			(transition_point(Design.shape, fit.transition_in, middle, 1.0).angle + transition_point(Design.shape, fit.transition_out, middle, 1.0).angle > fit.deflection ? low : high) = middle;
		}
		low = high * (1.0 + 1e-9);
		high = fit.radius;
		if (false == fit.compound && fits_budget(low))
		{
			for (int i = 0; i < 60; ++i)
			{
				auto const middle{(low + high) * 0.5};
				(fits_budget(middle) ? low : high) = middle;
			}
			fit.radius = low;
		}
		else
		{
			fit.radius *= scales[k];
			fit.radius2 *= scales[k];
			fit.transition_in *= scales[k];
			fit.transition_middle *= scales[k];
			fit.transition_out *= scales[k];
		}
		r.warnings.emplace_back(format("Vertex %zu: R reduced from %.1f to %.1f m, transitions to %.1f / %.1f m, to fit between the ends", k + 1, Design.vertices[k].radius, fit.radius, fit.transition_in, fit.transition_out));
	}

	std::vector<double> vertexchainage(count, 0.0);
	glm::dvec2 cursor{start};
	for (std::size_t k = 0; k < count; ++k)
	{
		auto const &vertex{Design.vertices[k]};
		auto const &fit{fits[k]};
		auto const &position{polygon[k + 1]};
		auto const directionin{glm::normalize(polygon[k + 1] - polygon[k])};
		auto const directionout{glm::normalize(polygon[k + 2] - polygon[k + 1])};
		auto const turn{cross(directionin, directionout)};
		auto const geometric{std::atan2(std::abs(turn), glm::dot(directionin, directionout))};
		auto const reverse{Design.vertices[k].reverse_turn};
		auto const deflection{reverse ? 2.0 * kPi - geometric : geometric};
		int const side{(turn > 0.0) != reverse ? 1 : -1};

		auto const lead{glm::dot(position - cursor, directionin)};
		if (deflection < 1e-7)
		{
			if (lead < -1e-4)
				r.errors.emplace_back(format("Vertex %zu lies inside the curve of the previous vertex", k + 1));
			push_straight(cursor, directionin, lead);
			vertexchainage[k] = chainage;
			cursor = position;
			continue;
		}
		if (geometric > kPi - 1e-3 || (reverse && geometric < 1e-3))
		{
			r.errors.emplace_back(format("Vertex %zu: a turn of about 180 or 360 degrees needs one more vertex", k + 1));
			return r;
		}
		if (fit.compound)
		{
			auto const shape{build_compound(deflection, fit.radius, fit.radius2, fit.transition_in, fit.transition_middle, fit.transition_out, vertex.split)};
			if (shape.shortened < 1.0)
				r.warnings.emplace_back(format("Vertex %zu: transition curves shortened to fit the deflection angle", k + 1));
			auto const normalin{perpendicular(directionin) * static_cast<double>(side)};
			auto const endoffset{directionin * shape.end.x + normalin * shape.end.y};
			auto const along{cross(directionout, endoffset) / cross(directionout, directionin)};
			auto const curvestart{position - directionin * along};
			auto const straight{glm::dot(curvestart - cursor, directionin)};
			if (straight < -1e-4)
			{
				if (k == 0)
					r.errors.emplace_back(format("Vertex 1: the curve starts %.2f m before the fixed start", -straight));
				else
					r.errors.emplace_back(format("Curves of vertices %zu and %zu overlap by %.2f m", k, k + 1, -straight));
			}
			push_straight(cursor, directionin, straight);
			auto const startchainage{chainage};
			glm::dvec2 origin{curvestart};
			glm::dvec2 heading{directionin};
			auto const peak{std::max(1.0 / fit.radius, 1.0 / fit.radius2)};
			for (auto const &segment : shape.segments)
			{
				element piece;
				piece.kind = element_kind::spiral;
				piece.vertex = static_cast<int>(k);
				piece.turn = side;
				piece.cant = std::max(0.0, vertex.cant);
				piece.curvature_start = segment[0];
				piece.curvature_end = segment[1];
				piece.curvature_peak = peak;
				piece.radius = std::max(segment[0], segment[1]) > 0.0 ? 1.0 / std::max(segment[0], segment[1]) : 0.0;
				piece.length = segment[2];
				piece.origin = origin;
				piece.direction = heading;
				piece.normal = perpendicular(heading) * static_cast<double>(side);
				push(piece);
				auto const local{spiral_point(segment[0], segment[1], segment[2], segment[2])};
				origin += heading * local.x + piece.normal * local.y;
				heading = rotate(heading, side * local.angle);
			}
			vertexchainage[k] = (startchainage + chainage) * 0.5;
			cursor = origin;

			curve_report report;
			report.vertex = static_cast<int>(k);
			report.radius = fit.radius;
			report.radius2 = fit.radius2;
			report.transition_in = fit.transition_in * shape.shortened;
			report.transition_out = fit.transition_out * shape.shortened;
			report.deflection = deflection;
			report.arc_length = chainage - startchainage;
			report.tangent_in = glm::distance(position, curvestart);
			report.tangent_out = glm::distance(position, cursor);
			auto const sharpest{std::min(fit.radius, fit.radius2)};
			report.unbalanced = unbalanced_acceleration(Design.speed, sharpest, std::max(0.0, vertex.cant), Design.norms);
			report.recommended = recommend(Design.speed, sharpest, Design.norms);
			r.curves.push_back(report);
			if (sharpest < report.recommended.radius_min - 1e-6)
				r.warnings.emplace_back(format("Vertex %zu: R %.0f m is below the minimum %.0f m for %.0f km/h", k + 1, sharpest, report.recommended.radius_min, Design.speed));
			if (report.unbalanced > Design.norms.unbalanced + 1e-6)
				r.warnings.emplace_back(format("Vertex %zu: unbalanced acceleration %.2f m/s2 exceeds %.2f m/s2", k + 1, report.unbalanced, Design.norms.unbalanced));
			continue;
		}
		auto const radius{fit.radius};
		auto const lengthin{fit.transition_in};
		auto const lengthout{fit.transition_out};
		auto const endin{transition_point(Design.shape, lengthin, radius, 1.0)};
		auto const endout{transition_point(Design.shape, lengthout, radius, 1.0)};
		auto const shiftin{endin.y - radius * (1.0 - std::cos(endin.angle))};
		auto const shiftout{endout.y - radius * (1.0 - std::cos(endout.angle))};
		auto const middlein{endin.x - radius * std::sin(endin.angle)};
		auto const middleout{endout.x - radius * std::sin(endout.angle)};

		auto const normalin{perpendicular(directionin) * static_cast<double>(side)};
		auto const normalout{perpendicular(directionout) * static_cast<double>(side)};
		auto const a{radius + shiftin};
		auto const b{radius + shiftout};
		auto const determinant{cross(normalin, normalout)};
		glm::dvec2 const centre{position + glm::dvec2{(a * normalout.y - normalin.y * b) / determinant, (normalin.x * b - a * normalout.x) / determinant}};
		auto const curvestart{centre - normalin * a - directionin * middlein};
		auto const curveend{centre - normalout * b + directionout * middleout};
		auto const arcangle{deflection - endin.angle - endout.angle};
		if (arcangle < -1e-9)
		{
			r.errors.emplace_back(format("Vertex %zu: transition curves are too long for the deflection angle", k + 1));
			return r;
		}

		auto const straight{glm::dot(curvestart - cursor, directionin)};
		if (straight < -1e-4)
		{
			if (k == 0)
				r.errors.emplace_back(format("Vertex 1: the curve starts %.2f m before the fixed start", -straight));
			else
				r.errors.emplace_back(format("Curves of vertices %zu and %zu overlap by %.2f m", k, k + 1, -straight));
		}
		push_straight(cursor, directionin, straight);

		element curve;
		curve.vertex = static_cast<int>(k);
		curve.radius = radius;
		curve.turn = side;
		curve.cant = std::max(0.0, vertex.cant);
		if (lengthin > 0.0)
		{
			curve.kind = element_kind::transition_in;
			curve.transition = curve.length = lengthin;
			curve.origin = curvestart;
			curve.direction = directionin;
			curve.normal = normalin;
			push(curve);
		}
		auto const arcstart{curvestart + directionin * endin.x + normalin * endin.y};
		curve.kind = element_kind::arc;
		curve.transition = 0.0;
		curve.length = radius * std::max(0.0, arcangle);
		curve.origin = centre;
		curve.normal = arcstart - centre;
		curve.direction = rotate(directionin, side * endin.angle);
		vertexchainage[k] = chainage + curve.length * 0.5;
		if (curve.length > 1e-6)
			push(curve);
		if (lengthout > 0.0)
		{
			curve.kind = element_kind::transition_out;
			curve.transition = curve.length = lengthout;
			curve.origin = curveend;
			curve.direction = -directionout;
			curve.normal = normalout;
			push(curve);
		}
		cursor = curveend;

		curve_report report;
		report.vertex = static_cast<int>(k);
		report.radius = radius;
		report.transition_in = lengthin;
		report.transition_out = lengthout;
		report.deflection = deflection;
		report.arc_length = radius * std::max(0.0, arcangle);
		report.tangent_in = glm::distance(position, curvestart);
		report.tangent_out = glm::distance(position, curveend);
		report.unbalanced = unbalanced_acceleration(Design.speed, radius, curve.cant, Design.norms);
		auto const speed{Design.speed / 3.6};
		if (lengthin > 0.0)
		{
			report.cant_rate_in = curve.cant * speed / lengthin;
			report.jerk_in = std::max(0.0, report.unbalanced) * speed / lengthin;
		}
		if (lengthout > 0.0)
		{
			report.cant_rate_out = curve.cant * speed / lengthout;
			report.jerk_out = std::max(0.0, report.unbalanced) * speed / lengthout;
		}
		report.recommended = recommend(Design.speed, radius, Design.norms);
		r.curves.push_back(report);

		auto const &norms{Design.norms};
		if (radius < report.recommended.radius_min - 1e-6)
			r.warnings.emplace_back(format("Vertex %zu: R %.0f m is below the minimum %.0f m for %.0f km/h", k + 1, radius, report.recommended.radius_min, Design.speed));
		if (curve.cant > norms.cant_max + 1e-6)
			r.warnings.emplace_back(format("Vertex %zu: cant %.0f mm exceeds %.0f mm", k + 1, curve.cant, norms.cant_max));
		if (report.unbalanced > norms.unbalanced + 1e-6)
			r.warnings.emplace_back(format("Vertex %zu: unbalanced acceleration %.2f m/s2 exceeds %.2f m/s2 (cant at least %.0f mm)", k + 1, report.unbalanced, norms.unbalanced, report.recommended.cant_min));
		auto const required{transition_length(Design.speed, radius, curve.cant, norms)};
		if (lengthin + 1e-6 < required || lengthout + 1e-6 < required)
			r.warnings.emplace_back(format("Vertex %zu: transition curves shorter than the required %.0f m", k + 1, required));
		if (curve.cant > 0.0 && (lengthin <= 0.0 || lengthout <= 0.0))
			r.warnings.emplace_back(format("Vertex %zu: cant without a transition curve to ramp it", k + 1));
	}
	auto const lastdirection{glm::normalize(polygon.back() - polygon[polygon.size() - 2])};
	auto const tail{glm::dot(end - cursor, lastdirection)};
	if (tail < -1e-4)
		r.errors.emplace_back(format("Vertex %zu: the curve ends %.2f m past the fixed end", count, -tail));
	push_straight(cursor, lastdirection, tail);
	r.length = chainage;

	if (Design.norms.tangent_min_time > 0.0)
	{
		auto const minimum{Design.norms.tangent_min_time * Design.speed / 3.6};
		for (std::size_t i = 1; i + 1 < r.elements.size(); ++i)
		{
			auto const &e{r.elements[i]};
			if (e.kind == element_kind::straight && e.length < minimum && r.elements[i - 1].kind != element_kind::straight && r.elements[i + 1].kind != element_kind::straight)
				r.warnings.emplace_back(format("Straight between curves is %.1f m long, less than %.0f m", e.length, minimum));
		}
	}

	r.vertex_chainages = vertexchainage;
	r.profile.push_back({0.0, Design.start.y, 0.0, 0.0});
	for (std::size_t k = 0; k < count; ++k)
	{
		auto const &vertex{Design.vertices[k]};
		auto const elevation{vertex.auto_elevation ? Design.start.y + (Design.end.y - Design.start.y) * vertexchainage[k] / std::max(r.length, 1e-6) : vertex.elevation};
		r.vertex_elevations.push_back(elevation);
		auto const radius{vertex.vertical_radius > 0.0 ? vertex.vertical_radius : std::max(Design.norms.vertical_min, Design.norms.vertical_factor * Design.speed * Design.speed)};
		if (vertexchainage[k] <= r.profile.back().chainage + 1e-3)
			continue;
		r.profile.push_back({vertexchainage[k], elevation, radius, 0.0});
	}
	r.profile.push_back({r.length, Design.end.y, 0.0, 0.0});
	for (std::size_t i = 1; i + 1 < r.profile.size(); ++i)
	{
		auto &point{r.profile[i]};
		auto const gradebefore{(point.elevation - r.profile[i - 1].elevation) / (point.chainage - r.profile[i - 1].chainage)};
		auto const gradeafter{(r.profile[i + 1].elevation - point.elevation) / (r.profile[i + 1].chainage - point.chainage)};
		point.tangent = point.radius * std::abs(gradeafter - gradebefore) / 2.0;
	}
	for (std::size_t i = 0; i + 1 < r.profile.size(); ++i)
	{
		if (r.profile[i].tangent + r.profile[i + 1].tangent > r.profile[i + 1].chainage - r.profile[i].chainage + 1e-6)
			r.warnings.emplace_back(format("Vertical curves overlap between chainage %.0f m and %.0f m", r.profile[i].chainage, r.profile[i + 1].chainage));
	}
	if (r.profile.size() >= 2)
	{
		auto const first{(r.profile[1].elevation - r.profile[0].elevation) / (r.profile[1].chainage - r.profile[0].chainage)};
		auto const last{(r.profile.back().elevation - r.profile[r.profile.size() - 2].elevation) / (r.profile.back().chainage - r.profile[r.profile.size() - 2].chainage)};
		if (std::abs(first - Design.start_grade) > 0.0005)
			r.warnings.emplace_back(format("Grade break at the start: %.1f per mille against %.1f of the adjoining track", first * 1000.0, Design.start_grade * 1000.0));
		if (std::abs(last - Design.end_grade) > 0.0005)
			r.warnings.emplace_back(format("Grade break at the end: %.1f per mille against %.1f of the adjoining track", last * 1000.0, Design.end_grade * 1000.0));
	}
	if (Design.start_radius > 0.0 && Design.start_radius < 10000.0)
		r.warnings.emplace_back(format("The adjoining track at the start is curved (R %.0f m), the curvature changes abruptly", Design.start_radius));
	if (Design.end_radius > 0.0 && Design.end_radius < 10000.0)
		r.warnings.emplace_back(format("The adjoining track at the end is curved (R %.0f m), the curvature changes abruptly", Design.end_radius));

	r.valid = r.errors.empty() && r.length > 1e-3;
	return r;
}

bool tangent_intersection(design const &Design, glm::dvec2 &Point)
{
	auto const start{plan(Design.start)};
	auto const offset{plan(Design.end) - start};
	auto const startdirection{glm::normalize(Design.start_direction)};
	auto const enddirection{glm::normalize(Design.end_direction)};
	auto const determinant{cross(startdirection, enddirection)};
	if (std::abs(determinant) < 1e-9)
		return false;
	auto const t{cross(offset, enddirection) / determinant};
	auto const u{cross(startdirection, offset) / determinant};
	if (t <= 0.0 || u <= 0.0)
		return false;
	Point = start + startdirection * t;
	return true;
}

bool collinear(design const &Design)
{
	auto const offset{plan(Design.end) - plan(Design.start)};
	auto const startdirection{glm::normalize(Design.start_direction)};
	auto const enddirection{glm::normalize(Design.end_direction)};
	return std::abs(cross(startdirection, offset)) <= 0.01 && glm::dot(startdirection, enddirection) >= std::cos(1e-4) && glm::dot(startdirection, offset) > 0.0;
}

double elevation(result const &Result, double const Chainage)
{
	auto const &profile{Result.profile};
	if (profile.size() < 2)
		return 0.0;
	auto const s{std::clamp(Chainage, 0.0, Result.length)};
	std::size_t segment{0};
	while (segment + 2 < profile.size() && s > profile[segment + 1].chainage)
		++segment;
	auto const &from{profile[segment]};
	auto const &to{profile[segment + 1]};
	double height{from.elevation + (to.elevation - from.elevation) * (s - from.chainage) / (to.chainage - from.chainage)};
	for (std::size_t i = 1; i + 1 < profile.size(); ++i)
	{
		auto const &point{profile[i]};
		auto const distance{std::abs(s - point.chainage)};
		if (point.tangent <= 0.0 || distance >= point.tangent)
			continue;
		auto const gradebefore{(point.elevation - profile[i - 1].elevation) / (point.chainage - profile[i - 1].chainage)};
		auto const gradeafter{(profile[i + 1].elevation - point.elevation) / (profile[i + 1].chainage - point.chainage)};
		auto const offset{(point.tangent - distance) * (point.tangent - distance) / (2.0 * point.radius)};
		height += gradeafter > gradebefore ? offset : -offset;
	}
	return height;
}

double grade(result const &Result, double const Chainage)
{
	auto const &profile{Result.profile};
	if (profile.size() < 2)
		return 0.0;
	auto const s{std::clamp(Chainage, 0.0, Result.length)};
	std::size_t segment{0};
	while (segment + 2 < profile.size() && s > profile[segment + 1].chainage)
		++segment;
	auto const &from{profile[segment]};
	auto const &to{profile[segment + 1]};
	double slope{(to.elevation - from.elevation) / (to.chainage - from.chainage)};
	for (std::size_t i = 1; i + 1 < profile.size(); ++i)
	{
		auto const &point{profile[i]};
		auto const distance{s - point.chainage};
		if (point.tangent <= 0.0 || std::abs(distance) >= point.tangent)
			continue;
		auto const gradebefore{(point.elevation - profile[i - 1].elevation) / (point.chainage - profile[i - 1].chainage)};
		auto const gradeafter{(profile[i + 1].elevation - point.elevation) / (profile[i + 1].chainage - point.chainage)};
		auto const change{(point.tangent - std::abs(distance)) / point.radius * (distance < 0.0 ? 1.0 : -1.0)};
		slope += gradeafter > gradebefore ? change : -change;
	}
	return slope;
}

station evaluate(result const &Result, double const Chainage)
{
	station result;
	if (Result.elements.empty())
		return result;
	auto const s{std::clamp(Chainage, 0.0, Result.length)};
	auto const &e{element_at(Result, s)};
	auto const distance{std::clamp(s - e.chainage, 0.0, e.length)};
	glm::dvec2 position, direction;
	element_point(e, Result.shape, distance, position, direction);
	result.position = {position.x, elevation(Result, s), position.y};
	result.direction = glm::normalize(glm::dvec3{direction.x, grade(Result, s), direction.y});
	result.cant = element_cant(e, distance);
	result.turn = e.turn;
	return result;
}

std::size_t minimum_pieces(result const &Result, design const &Design)
{
	std::size_t count{0};
	for (auto const &e : Result.elements)
		count += element_pieces(e, Design.shape, Design.transition_pieces, Design.arc_piece_angle);
	return count;
}

std::vector<segment_data> pieces(result const &Result, design const &Design, std::size_t const Count)
{
	struct span
	{
		std::size_t element;
		double from, to;
		bool generic{false};
		double start{0.0};
		double end{0.0};
	};
	auto const extent = [&](span const &Span) { return Span.generic ? Span.end - Span.start : Span.to - Span.from; };
	auto const chainage_from = [&](span const &Span) { return Span.generic ? Span.start : Result.elements[Span.element].chainage + Span.from; };
	auto const chainage_to = [&](span const &Span) { return Span.generic ? Span.end : Result.elements[Span.element].chainage + Span.to; };
	std::vector<span> spans;
	for (std::size_t i = 0; i < Result.elements.size(); ++i)
	{
		auto const &e{Result.elements[i]};
		auto const count{element_pieces(e, Design.shape, Design.transition_pieces, Design.arc_piece_angle)};
		for (std::size_t j = 0; j < count; ++j)
			spans.push_back({i, e.length * j / count, e.length * (j + 1) / count});
	}
	double const shortest{1.0};
	for (std::size_t i = 0; i < spans.size() && spans.size() > 1;)
	{
		auto const &e{Result.elements[spans[i].element]};
		if (e.kind != element_kind::straight || extent(spans[i]) >= shortest)
		{
			++i;
			continue;
		}
		auto const start{chainage_from(spans[i])};
		auto const end{chainage_to(spans[i])};
		if (i > 0)
		{
			auto &previous{spans[i - 1]};
			previous.start = chainage_from(previous);
			previous.end = end;
			previous.generic = true;
		}
		else
		{
			auto &next{spans[i + 1]};
			next.end = chainage_to(next);
			next.start = start;
			next.generic = true;
		}
		spans.erase(spans.begin() + i);
	}
	while (false == spans.empty() && spans.size() < Count)
	{
		auto longest = std::max_element(spans.begin(), spans.end(), [&](span const &A, span const &B) { return extent(A) < extent(B); });
		span second{*longest};
		if (longest->generic)
		{
			auto const middle{(longest->start + longest->end) * 0.5};
			second.start = middle;
			longest->end = middle;
		}
		else
		{
			auto const middle{(longest->from + longest->to) * 0.5};
			second.from = middle;
			longest->to = middle;
		}
		spans.insert(std::next(longest), second);
	}

	std::vector<segment_data> result;
	for (auto const &piece : spans)
	{
		if (piece.generic)
		{
			auto const first{evaluate(Result, piece.start)};
			auto const last{evaluate(Result, piece.end)};
			glm::dvec2 const p0{first.position.x, first.position.z};
			glm::dvec2 const p3{last.position.x, last.position.z};
			auto const d0{glm::normalize(glm::dvec2{first.direction.x, first.direction.z})};
			auto const d3{glm::normalize(glm::dvec2{last.direction.x, last.direction.z})};
			auto const handle{glm::distance(p0, p3) / 3.0};
			auto const p1{p0 + d0 * handle};
			auto const p2{p3 - d3 * handle};
			auto const y0{first.position.y};
			auto const y3{last.position.y};
			auto const y1{y0 + grade(Result, piece.start) * handle};
			auto const y2{y3 - grade(Result, piece.end) * handle};
			auto const roll = [&](station const &Station) { return static_cast<float>(Station.turn * glm::degrees(std::asin(std::clamp(Station.cant / Design.norms.gauge, -1.0, 1.0)))); };
			segment_data path;
			path.points[segment_data::point::start] = first.position;
			path.points[segment_data::point::end] = last.position;
			path.points[segment_data::point::control1] = glm::dvec3{p1.x, y1, p1.y} - first.position;
			path.points[segment_data::point::control2] = glm::dvec3{p2.x, y2, p2.y} - last.position;
			path.rolls[0] = roll(first);
			path.rolls[1] = roll(last);
			auto const &e{Result.elements[piece.element]};
			path.radius = e.kind == element_kind::straight ? 0.f : static_cast<float>(e.radius);
			result.push_back(path);
			continue;
		}
		auto const &e{Result.elements[piece.element]};
		glm::dvec2 p0, p3, d0, d3;
		element_point(e, Design.shape, piece.from, p0, d0);
		element_point(e, Design.shape, piece.to, p3, d3);
		glm::dvec2 p1, p2;
		switch (e.kind)
		{
		case element_kind::straight:
			p1 = p0 + (p3 - p0) / 3.0;
			p2 = p0 + (p3 - p0) * (2.0 / 3.0);
			break;
		case element_kind::arc:
		{
			auto const angle{(piece.to - piece.from) / e.radius};
			auto const handle{4.0 / 3.0 * std::tan(angle / 4.0) * e.radius};
			p1 = p0 + d0 * handle;
			p2 = p3 - d3 * handle;
			break;
		}
		default:
		{
			if (Design.shape == transition_shape::cubic_parabola && e.kind != element_kind::spiral)
			{
				std::array<glm::dvec2, 4> const local{glm::dvec2{0.0, 0.0}, glm::dvec2{e.transition / 3.0, 0.0}, glm::dvec2{e.transition * 2.0 / 3.0, 0.0}, glm::dvec2{e.transition, e.transition * e.transition / (6.0 * e.radius)}};
				auto const a{e.kind == element_kind::transition_in ? piece.from / e.transition : 1.0 - piece.from / e.transition};
				auto const b{e.kind == element_kind::transition_in ? piece.to / e.transition : 1.0 - piece.to / e.transition};
				auto const world = [&](glm::dvec2 const &Point) { return e.origin + e.direction * Point.x + e.normal * Point.y; };
				p0 = world(blossom(local, a, a, a));
				p1 = world(blossom(local, a, a, b));
				p2 = world(blossom(local, a, b, b));
				p3 = world(blossom(local, b, b, b));
			}
			else
			{
				auto const chord{glm::distance(p0, p3) / 3.0};
				p1 = p0 + d0 * chord;
				p2 = p3 - d3 * chord;
			}
			break;
		}
		}
		auto const s0{e.chainage + piece.from};
		auto const s3{e.chainage + piece.to};
		auto const y0{elevation(Result, s0)};
		auto const y3{elevation(Result, s3)};
		auto const y1{y0 + grade(Result, s0) * glm::distance(p0, p1)};
		auto const y2{y3 - grade(Result, s3) * glm::distance(p2, p3)};

		auto const roll = [&](double const Cant) { return static_cast<float>(e.turn * glm::degrees(std::asin(std::clamp(Cant / Design.norms.gauge, -1.0, 1.0)))); };

		segment_data path;
		path.points[segment_data::point::start] = {p0.x, y0, p0.y};
		path.points[segment_data::point::end] = {p3.x, y3, p3.y};
		path.points[segment_data::point::control1] = glm::dvec3{p1.x, y1, p1.y} - path.points[segment_data::point::start];
		path.points[segment_data::point::control2] = glm::dvec3{p2.x, y2, p2.y} - path.points[segment_data::point::end];
		path.rolls[0] = roll(element_cant(e, piece.from));
		path.rolls[1] = roll(element_cant(e, piece.to));
		path.radius = e.kind == element_kind::straight ? 0.f : static_cast<float>(e.radius);
		if (e.kind == element_kind::spiral)
		{
			auto const curvature{e.curvature_start + (e.curvature_end - e.curvature_start) * (piece.from + piece.to) * 0.5 / std::max(e.length, 1e-9)};
			path.radius = curvature > 1e-9 ? static_cast<float>(1.0 / curvature) : 0.f;
		}
		result.push_back(path);
	}
	return result;
}

}
