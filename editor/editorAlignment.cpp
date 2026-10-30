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
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "utilities/translation.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <optional>

namespace alignment
{

namespace
{

double const kGravity{9.81};
double const kPi{3.14159265358979323846};

using geometry::cross;
using geometry::plan_of;
using geometry::turned;

glm::dvec2 perpendicular(glm::dvec2 const &Vector)
{
	return {-Vector.y, Vector.x};
}

double ceil_to(double const Value, double const Step)
{
	return std::ceil(Value / Step - 1e-9) * Step;
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
	// Simpson over 64 steps at most keeps a transition of a radian within a millimetre, however long
	int const steps{std::clamp(static_cast<int>(std::ceil(distance / 0.5)) * 2, 8, 64)};
	double const h{distance / steps};
	double x{0.0};
	double y{0.0};
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
	// an arc, or a straight, has its point at once
	if (Start == End)
	{
		auto const angle{Start * Distance};
		if (std::abs(angle) < 1e-9)
			return {Distance, 0.0, 0.0};
		return {std::sin(angle) / Start, (1.0 - std::cos(angle)) / Start, angle};
	}
	auto const heading = [&](double const S) { return Start * S + (End - Start) * S * S / (2.0 * std::max(Length, 1e-9)); };
	int const steps{std::clamp(static_cast<int>(std::ceil(Distance / 0.5)) * 2, 8, 64)};
	double const h{Distance / steps};
	double x{0.0};
	double y{0.0};
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
	// the curvature at the start and at the end, the length, the cant at the start and at the end
	std::vector<std::array<double, 5>> segments;
	local_point end;
	double shortened{1.0};
};

// the curvature and the cant a curve goes on from at its start and into at its end, those of the curve of the adjoining track turning
// its way; 0 on a straight
struct curve_ends
{
	double from{0.0};
	double to{0.0};
	double from_cant{0.0};
	double to_cant{0.0};
};

// Cants: of the arcs, the cants of the segments stay 0 without them
compound_curve build_compound(double const Deflection, std::vector<compound_arc> const &Arcs, double const Lengthin, double const Lengthout, std::vector<double> const &Cants = {}, curve_ends const &Ends = {})
{
	compound_curve result;
	if (Arcs.empty())
		return result;
	std::vector<double> curvatures;
	for (auto const &arc : Arcs)
		curvatures.push_back(1.0 / std::max(1.0, arc.radius));
	auto const cant = [&](std::size_t const Arc) { return Arc < Cants.size() ? Cants[Arc] : 0.0; };
	auto transitions{(Ends.from + curvatures.front()) * Lengthin + (curvatures.back() + Ends.to) * Lengthout};
	for (std::size_t i = 1; i < Arcs.size(); ++i)
		transitions += (curvatures[i - 1] + curvatures[i]) * std::max(0.0, Arcs[i].transition);
	transitions *= 0.5;
	if (transitions > Deflection)
		result.shortened = Deflection / transitions * (1.0 - 1e-9);
	auto const f{result.shortened};
	auto const arcs{std::max(0.0, Deflection - transitions * f)};
	double total{0.0};
	for (auto const &arc : Arcs)
		total += std::max(0.0, arc.share);
	std::vector<std::array<double, 5>> segments{{Ends.from, curvatures.front(), Lengthin * f, Ends.from_cant, cant(0)}};
	for (std::size_t i = 0; i < Arcs.size(); ++i)
	{
		if (i > 0)
			segments.push_back({curvatures[i - 1], curvatures[i], std::max(0.0, Arcs[i].transition) * f, cant(i - 1), cant(i)});
		auto const share{total > 0.0 ? std::max(0.0, Arcs[i].share) / total : 1.0 / static_cast<double>(Arcs.size())};
		segments.push_back({curvatures[i], curvatures[i], share * arcs / curvatures[i], cant(i), cant(i)});
	}
	segments.push_back({curvatures.back(), Ends.to, Lengthout * f, cant(Arcs.size() - 1), Ends.to_cant});
	double x{0.0};
	double y{0.0};
	double heading{0.0};
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

// the cants of the arcs, each its own or Cant at the sharpest of them in proportion to the curvature
std::vector<double> arc_cants(std::vector<compound_arc> const &Arcs, double const Cant)
{
	auto sharpest{std::numeric_limits<double>::max()};
	for (auto const &arc : Arcs)
		sharpest = std::min(sharpest, std::max(1.0, arc.radius));
	std::vector<double> cants;
	for (auto const &arc : Arcs)
		cants.push_back(arc.cant >= 0.0 ? arc.cant : std::max(0.0, Cant) * sharpest / std::max(1.0, arc.radius));
	return cants;
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
		Position = Element.origin + turned(Element.normal, angle);
		Direction = turned(Element.direction, angle);
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
	case element_kind::spiral: return Element.cant_start + (Element.cant_end - Element.cant_start) * std::clamp(Distance / std::max(Element.length, 1e-9), 0.0, 1.0);
	default: return 0.0;
	}
}

element const &element_at(result const &Result, double const Chainage)
{
	auto const lookup = std::upper_bound(Result.elements.begin(), Result.elements.end(), Chainage, [](double const Value, element const &Element) { return Value < Element.chainage; });
	return lookup == Result.elements.begin() ? Result.elements.front() : *std::prev(lookup);
}

// the cant of the line at the chainage, signed with the turn of its curve; where it changes all at once there, the one before the
// chainage or the one after it
double signed_cant(result const &Result, element const &Element, double const Distance, bool const Before)
{
	auto const &cants{Result.cants};
	if (cants.empty())
		return Element.turn * element_cant(Element, Distance);
	auto const chainage{Element.chainage + Distance};
	auto const next{Before ? std::lower_bound(cants.begin(), cants.end(), chainage, [](glm::dvec2 const &Point, double const Value) { return Point.x < Value; })
	                       : std::upper_bound(cants.begin(), cants.end(), chainage, [](double const Value, glm::dvec2 const &Point) { return Value < Point.x; })};
	if (next == cants.begin())
		return cants.front().y;
	if (next == cants.end())
		return cants.back().y;
	auto const previous{std::prev(next)};
	auto const span{next->x - previous->x};
	return span < 1e-9 ? next->y : previous->y + (next->y - previous->y) * (chainage - previous->x) / span;
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
		auto const arcs{std::max(1.0, std::ceil(turn / (Arcangle * kPi / 180.0) - 1e-9))};
		if (Element.curvature_start == Element.curvature_end)
			return static_cast<std::size_t>(arcs);
		auto const count{static_cast<std::size_t>(std::max(arcs, std::ceil(Element.length / 20.0 - 1e-9)))};
		return std::max<std::size_t>(count, std::max(1, Transitions));
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

std::vector<double> compound_cants(vertex const &Vertex)
{
	if (false == Vertex.compound || Vertex.arcs.empty())
		return {std::max(0.0, Vertex.cant)};
	std::vector<compound_arc> arcs{{Vertex.radius, 0.0, Vertex.share, Vertex.first_cant}};
	arcs.insert(arcs.end(), Vertex.arcs.begin(), Vertex.arcs.end());
	return arc_cants(arcs, Vertex.cant);
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

double transition_between(double const Speed, double const Curvature0, double const Cant0, double const Curvature1, double const Cant1, limits const &Limits)
{
	auto const cant{std::abs(Cant1 - Cant0)};
	double length{0.0};
	if (cant > 0.0)
	{
		length = std::max(length, Speed * cant / (3.6 * Limits.cant_rate));
		length = std::max(length, cant / 1000.0 * Limits.ramp_factor * Speed);
	}
	auto const unbalanced = [&](double const Curvature, double const Cant) { return Speed * Speed / 12.96 * Curvature - kGravity * Cant / Limits.gauge; };
	length = std::max(length, std::abs(unbalanced(Curvature1, Cant1) - unbalanced(Curvature0, Cant0)) * Speed / (3.6 * Limits.jerk));
	return length > 0.0 ? ceil_to(length, 5.0) : 0.0;
}

recommendation recommend(double const Speed, double const Radius, limits const &Limits)
{
	recommendation result;
	auto const factor{Limits.gauge / (kGravity * 12.96)};
	auto const allowance{Limits.gauge * Limits.unbalanced / kGravity};
	result.radius_min = factor * Speed * Speed / (Limits.cant_max + allowance);
	if (Radius <= 0.0)
		return result;
	result.cant_equilibrium = factor * Speed * Speed / Radius;
	result.cant_min = std::max(0.0, result.cant_equilibrium - allowance);
	if (false == Limits.checked)
		return result;
	result.cant = std::min(Limits.cant_max, ceil_to(std::max(result.cant_min, result.cant_equilibrium * 2.0 / 3.0), 5.0));
	result.transition = std::min(5000.0, transition_length(Speed, Radius, result.cant, Limits));
	return result;
}

namespace
{

// the tangents of a curve of the radius with transitions of the lengths, from the vertex back along the straight in and on along the
// one out
void curve_tangents(transition_shape const Shape, double const Lengthin, double const Lengthout, double const Radius, glm::dvec2 const &Directionin, glm::dvec2 const &Directionout, int const Side,
                    double &Tangentin, double &Tangentout)
{
	auto const endin{transition_point(Shape, Lengthin, Radius, 1.0)};
	auto const endout{transition_point(Shape, Lengthout, Radius, 1.0)};
	auto const a{Radius + endin.y - Radius * (1.0 - std::cos(endin.angle))};
	auto const b{Radius + endout.y - Radius * (1.0 - std::cos(endout.angle))};
	auto const normalin{perpendicular(Directionin) * static_cast<double>(Side)};
	auto const normalout{perpendicular(Directionout) * static_cast<double>(Side)};
	auto const determinant{cross(normalin, normalout)};
	glm::dvec2 const offset{(a * normalout.y - normalin.y * b) / determinant, (normalin.x * b - a * normalout.x) / determinant};
	auto const curvestart{offset - normalin * a - Directionin * (endin.x - Radius * std::sin(endin.angle))};
	auto const curveend{offset - normalout * b + Directionout * (endout.x - Radius * std::sin(endout.angle))};
	Tangentin = std::max(0.0, -glm::dot(curvestart, Directionin));
	Tangentout = std::max(0.0, glm::dot(curveend, Directionout));
}

}

// each of the two transitions turns half of the deflection, ending at the curvature of the radius
double spirals_length(transition_shape const Shape, double const Deflection, double const Radius)
{
	auto const half{std::clamp(Deflection * 0.5, 0.0, 0.5 * kPi - 1e-6)};
	return Shape == transition_shape::clothoid ? 2.0 * Radius * half : 2.0 * Radius * std::tan(half);
}

namespace
{

// the transition between the curve of an adjoining track and the arc next to it, as between two arcs; none where the curvature differs
// by 5 % at most and the cant by less than 5 mm, the curve going on at the radius of the adjoining one
double between_curves(double const Speed, double const Curvature0, double const Cant0, double const Curvature1, double const Cant1, limits const &Limits)
{
	if (std::abs(Curvature1 - Curvature0) <= 0.05 * std::max(std::abs(Curvature0), std::abs(Curvature1)) && std::abs(Cant1 - Cant0) < 5.0)
		return 0.0;
	return transition_between(Speed, Curvature0, Cant0, Curvature1, Cant1, Limits);
}

} // namespace

bool meets_norms(curve_report const &Curve, design const &Design)
{
	auto const &norms{Design.norms};
	auto sharpest{Curve.radius};
	for (auto const radius : Curve.radii)
		sharpest = std::min(sharpest, radius);
	if (Curve.unbalanced > norms.unbalanced + 1e-6 || Curve.cant > norms.cant_max + 1e-6)
		return false;
	if (false == Curve.spirals && sharpest < Curve.recommended.radius_min - 1e-6)
		return false;
	// going on from the curve of an adjoining track, or into it: the transition there between its curvature and cant and those of the
	// arc next to it
	if ((Curve.curvature_in > 0.0 || Curve.curvature_out > 0.0) && false == Curve.radii.empty() && Curve.cants.size() == Curve.radii.size())
	{
		auto const required = [&](double const Curvature, double const Cant, double const Radius, double const Arccant) {
			return Curvature > 0.0 ? between_curves(Design.speed, Curvature, Cant, 1.0 / Radius, Arccant, norms) : transition_length(Design.speed, Radius, Arccant, norms);
		};
		return Curve.transition_in + 1e-6 >= required(Curve.curvature_in, Curve.cant_in, Curve.radii.front(), Curve.cants.front()) &&
		       Curve.transition_out + 1e-6 >= required(Curve.curvature_out, Curve.cant_out, Curve.radii.back(), Curve.cants.back());
	}
	if (Curve.radii.size() > 1 && Curve.cants.size() == Curve.radii.size())
		// a compound curve ramps the cant of its first arc in and the one of its last arc out
		return Curve.transition_in + 1e-6 >= transition_length(Design.speed, Curve.radii.front(), Curve.cants.front(), norms) &&
		       Curve.transition_out + 1e-6 >= transition_length(Design.speed, Curve.radii.back(), Curve.cants.back(), norms);
	auto const required{transition_length(Design.speed, sharpest, Curve.cant, norms)};
	return Curve.transition_in + 1e-6 >= required && Curve.transition_out + 1e-6 >= required;
}

namespace
{

// the length of a straight cant ramp the norms ask for to raise the cant at the speed: slow enough in time and along the track
double ramp_length(double const Speed, double const Cant, limits const &Limits)
{
	if (Cant <= 0.0)
		return 0.0;
	return ceil_to(std::max(Speed * Cant / (3.6 * Limits.cant_rate), Cant / 1000.0 * Limits.ramp_factor * Speed), 5.0);
}

// the cant along the line: in each curve as its elements carry it, raised from 0 and lowered back to it on its ramps, which go along
// its transition curves unless the vertex gives them a place and a length of their own
void make_cants(design const &Design, result &R)
{
	R.cants.clear();
	R.ramps.clear();
	auto const push = [&](double const Chainage, double const Cant) {
		if (false == R.cants.empty() && std::abs(R.cants.back().x - Chainage) < 1e-6 && std::abs(R.cants.back().y - Cant) < 1e-6)
			return;
		R.cants.push_back({Chainage, Cant});
	};
	double reached{0.0}; // where the cant of the curve before is down to 0 again
	for (std::size_t k = 0; k < Design.vertices.size(); ++k)
	{
		auto const &vertex{Design.vertices[k]};
		int first{-1};
		int last{-1};
		for (std::size_t i = 0; i < R.elements.size(); ++i)
			if (R.elements[i].vertex == static_cast<int>(k) && R.elements[i].length > 1e-6)
			{
				first = first < 0 ? static_cast<int>(i) : first;
				last = static_cast<int>(i);
			}
		if (first < 0)
			continue;
		auto const &head{R.elements[first]};
		auto const &tail{R.elements[last]};
		auto const start{head.chainage};
		auto const end{tail.chainage + tail.length};
		auto const turn{static_cast<double>(head.turn)};
		// the transitions at the ends: the first and the last element, where the curvature grows from 0 and goes back to it
		auto const entry{head.kind == element_kind::transition_in || (head.kind == element_kind::spiral && head.curvature_start == 0.0 && head.curvature_end > 0.0) ? head.length : 0.0};
		auto const exit{tail.kind == element_kind::transition_out || (tail.kind == element_kind::spiral && tail.curvature_end == 0.0 && tail.curvature_start > 0.0) ? tail.length : 0.0};
		auto const cantin{element_cant(head, entry)};
		auto const cantout{element_cant(tail, tail.length - exit)};
		// a curve going on from the curve of the adjoining track, or into it, has its cant there from the first: no ramp at that end
		auto const continuedin{head.kind == element_kind::spiral && head.curvature_start > 0.0};
		auto const continuedout{tail.kind == element_kind::spiral && tail.curvature_end > 0.0};
		if (continuedin || continuedout)
		{
			push(start, continuedin ? turn * cantin : 0.0);
			for (auto i{first}; i <= last; ++i)
			{
				auto const &e{R.elements[i]};
				for (auto const distance : {0.0, e.length})
					push(e.chainage + distance, turn * element_cant(e, distance));
			}
			push(end, continuedout ? turn * element_cant(tail, tail.length) : 0.0);
			if (false == continuedin && cantin >= 0.5)
				R.ramps.push_back({static_cast<int>(k), start, start + entry, cantin, false});
			if (false == continuedout && cantout >= 0.5)
				R.ramps.push_back({static_cast<int>(k), end - exit, end, cantout, false});
			reached = end;
			continue;
		}
		if (std::max(cantin, cantout) < 0.5)
		{
			reached = end;
			continue;
		}
		result::cant_ramp in{static_cast<int>(k), start, start + entry, cantin, vertex.ramp_in_length >= 0.0};
		result::cant_ramp out{static_cast<int>(k), end - exit, end, cantout, vertex.ramp_out_length >= 0.0};
		if (in.own)
		{
			in.from = start + vertex.ramp_in_offset;
			in.to = in.from + vertex.ramp_in_length;
		}
		if (out.own)
		{
			out.to = end - vertex.ramp_out_offset;
			out.from = out.to - vertex.ramp_out_length;
		}
		// the ramps kept off the curve before, within the line and off each other
		if (in.from < reached - 1e-6)
		{
			R.warnings.emplace_back(R.ramps.empty() ? format(STR_C("Vertex %zu: the cant ramp in reaches back past the start of the line, cut %.1f m shorter"), k + 1, reached - in.from) :
			                                          format(STR_C("Vertex %zu: the cant ramp in reaches back into the curve before it, cut %.1f m shorter"), k + 1, reached - in.from));
			in.from = std::min(reached, in.to);
		}
		if (out.to > R.length + 1e-6)
		{
			R.warnings.emplace_back(format(STR_C("Vertex %zu: the cant ramp out reaches past the end of the line, cut %.1f m shorter"), k + 1, out.to - R.length));
			out.to = std::max(R.length, out.from);
		}
		if (in.to > out.from + 1e-6)
		{
			R.warnings.emplace_back(format(STR_C("Vertex %zu: the cant ramps in and out of the curve overlap by %.1f m"), k + 1, in.to - out.from));
			auto const middle{(in.to + out.from) * 0.5};
			in.to = std::max(in.from, middle);
			out.from = std::min(out.to, middle);
		}
		auto const speed{std::max(1.0, Design.speed)};
		for (auto const *ramp : {&in, &out})
			if (ramp->own && Design.norms.checked)
				if (auto const required{ramp_length(speed, ramp->cant, Design.norms)}; ramp->to - ramp->from + 1e-6 < required)
					R.warnings.emplace_back(format(STR_C("Vertex %zu: the cant ramp of %.1f m is shorter than the %.0f m a cant of %.0f mm asks for"), k + 1, ramp->to - ramp->from, required, ramp->cant));
		// the curve between its ramps as its elements carry the cant, the transitions at its ends with the full cant of the curve where
		// the ramps went off them
		push(in.from, 0.0);
		push(in.to, turn * cantin);
		auto const low{std::max(in.to, start + entry)};
		auto const high{std::min(out.from, end - exit)};
		if (high > low + 1e-6)
		{
			push(low, turn * cantin);
			for (auto i{first}; i <= last; ++i)
			{
				auto const &e{R.elements[i]};
				for (auto const distance : {0.0, e.length})
					if (auto const chainage{e.chainage + distance}; chainage > low + 1e-6 && chainage < high - 1e-6)
						push(chainage, turn * element_cant(e, distance));
			}
			push(high, turn * cantout);
		}
		push(out.from, turn * cantout);
		push(out.to, 0.0);
		R.ramps.push_back(in);
		R.ramps.push_back(out);
		reached = out.to;
		// the time the cant takes to rise along the ramps of their own
		for (auto &curve : R.curves)
			if (curve.vertex == static_cast<int>(k))
			{
				if (in.own && in.to > in.from)
					curve.cant_rate_in = cantin * speed / 3.6 / (in.to - in.from);
				if (out.own && out.to > out.from)
					curve.cant_rate_out = cantout * speed / 3.6 / (out.to - out.from);
			}
	}
	std::stable_sort(R.cants.begin(), R.cants.end(), [](glm::dvec2 const &A, glm::dvec2 const &B) { return A.x < B.x; });
}

result fit_between(design const &Design)
{
	result r;
	r.shape = Design.shape;
	r.gauge = Design.norms.gauge;
	r.start = Design.start;
	r.end = Design.end;
	auto const startreserve{std::max(0.0, Design.start_reserve)};
	auto const endreserve{std::max(0.0, Design.end_reserve)};

	auto const start{plan_of(Design.start)};
	auto const end{plan_of(Design.end)};
	if (glm::length(Design.start_direction) < 1e-9 || glm::length(Design.end_direction) < 1e-9)
	{
		r.errors.emplace_back(STR_C("The direction of an end of the fragment is undefined, its end path has zero length"));
		return r;
	}
	auto const startdirection{glm::normalize(Design.start_direction)};
	auto const enddirection{glm::normalize(Design.end_direction)};
	auto const count{Design.vertices.size()};

	r.vertices.resize(count);
	if (count == 0)
	{
		auto const offset{end - start};
		if (std::abs(cross(startdirection, offset)) > 0.01 || glm::dot(startdirection, enddirection) < std::cos(1e-4) || glm::dot(startdirection, offset) <= 0.0)
			r.errors.emplace_back(STR_C("The ends don't lie on a common straight line, add a vertex"));
	}
	else if (count == 1)
	{
		auto const offset{end - start};
		auto const determinant{cross(startdirection, enddirection)};
		if (std::abs(determinant) < 1e-9)
		{
			r.errors.emplace_back(STR_C("Tangents of the ends are parallel, use two vertices"));
		}
		else
		{
			auto const t{cross(offset, enddirection) / determinant};
			auto const u{cross(startdirection, offset) / determinant};
			if ((t <= 0.0 || u <= 0.0) && false == Design.vertices.front().reverse_turn)
				r.errors.emplace_back(STR_C("Tangents of the ends intersect outside of the fragment, turn the longer way or use two vertices"));
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
			r.errors.emplace_back(format(STR_C("Tangent %zu has zero length"), i + 1));
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
	auto const starts_early = [&](double const Straight) {
		if (Straight < 0.0)
			r.start_extension = -Straight;
		if (Straight >= -1e-4 || -Straight <= startreserve + 1e-4)
			return;
		if (startreserve > 0.0)
			r.errors.emplace_back(format(STR_C("Vertex 1: the curve starts %.2f m before the adjoining straight"), -Straight - startreserve));
		else
			r.errors.emplace_back(format(STR_C("Vertex 1: the curve starts %.2f m before the fixed start"), -Straight));
	};

	struct fitted
	{
		double radius{0.0};
		double transition_in{0.0};
		double transition_out{0.0};
		double tangent_in{0.0};
		double tangent_out{0.0};
		double deflection{0.0};
		double cant{0.0};
		bool compound{false};
		bool spirals{false};
		// going on from the curve of the adjoining track: its size is fixed by the start or by the end, the curves next to it make room
		bool curved{false};
		curve_ends ends;
		std::vector<compound_arc> arcs;
		std::function<void(double, double &, double &)> tangents;
	};
	auto const scaled = [](std::vector<compound_arc> Arcs, double const Scale) {
		for (auto &arc : Arcs)
		{
			arc.radius *= Scale;
			arc.transition *= Scale;
		}
		return Arcs;
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
			r.errors.emplace_back(format(STR_C("Vertex %zu: a turn of about 180 or 360 degrees needs one more vertex"), k + 1));
			return r;
		}
		// the curve going on from the curve of the adjoining track turning its way, at the start or at the end: a compound curve of the
		// arcs it has, one or more, as large as begin it right at the start, or end it at the end
		auto const from{k == 0 && Design.start_curved && Design.start_radius > 0.0 && Design.start_turn == side};
		auto const to{k + 1 == count && Design.end_curved && Design.end_radius > 0.0 && Design.end_turn == side && false == from};
		if ((from || to) && false == reverse && false == vertex.spirals)
		{
			fit.compound = true;
			fit.curved = true;
			fit.deflection = deflection;
			// going on from the diverging track of a switch: at its radius, no transition between them
			auto const held{from ? Design.start_held : Design.end_held};
			if (held)
				(from ? fit.transition_in : fit.transition_out) = 0.0;
			fit.arcs = {{fit.radius, 0.0, 1.0, -1.0}};
			if (vertex.compound && false == vertex.arcs.empty())
			{
				fit.arcs.front() = {fit.radius, 0.0, vertex.share, vertex.first_cant};
				for (auto const &arc : vertex.arcs)
					fit.arcs.push_back({std::max(1.0, arc.radius), std::max(0.0, arc.transition), arc.share, arc.cant});
			}
			if (held)
				(from ? fit.arcs.front() : fit.arcs.back()).radius = from ? Design.start_radius : Design.end_radius;
			if (from)
			{
				fit.ends.from = 1.0 / Design.start_radius;
				fit.ends.from_cant = std::max(0.0, Design.start_cant);
				// with no transition between them the cant of the adjoining track goes on into the arc
				if (fit.transition_in < 1.0)
					fit.arcs.front().cant = fit.ends.from_cant;
			}
			else
			{
				fit.ends.to = 1.0 / Design.end_radius;
				fit.ends.to_cant = std::max(0.0, Design.end_cant);
				if (fit.transition_out < 1.0)
					fit.arcs.back().cant = fit.ends.to_cant;
			}
			auto const normalin{perpendicular(directionin) * static_cast<double>(side)};
			// the size of the curve which begins it right at the start, or ends it at the end: the scale of its radii, the arc held at the
			// radius of a switch kept; of a single arc held so the length of the transition on its other side
			auto const lengthened{held && fit.arcs.size() == 1};
			auto &beyond{from ? fit.transition_out : fit.transition_in};
			auto const sized = [&](double const Value, std::vector<compound_arc> &Arcs) {
				for (std::size_t i = 0; i < Arcs.size(); ++i)
					if (false == held || i != (from ? 0 : Arcs.size() - 1))
						Arcs[i].radius *= Value;
			};
			auto const tangents_at = [&](double const Value, double &Tangentin, double &Tangentout) {
				auto arcs{fit.arcs};
				auto in{fit.transition_in};
				auto out{fit.transition_out};
				if (lengthened)
					(from ? out : in) = Value;
				else
					sized(Value, arcs);
				auto const shape{build_compound(deflection, arcs, in, out, {}, fit.ends)};
				auto const end{directionin * shape.end.x + normalin * shape.end.y};
				Tangentin = cross(directionout, end) / cross(directionout, directionin);
				Tangentout = glm::dot(end - directionin * Tangentin, directionout);
			};
			// the straight it begins at, or ends at, all along the tangent: no straight between
			auto const wanted{from ? glm::distance(polygon[0], polygon[1]) : glm::distance(polygon[count], polygon[count + 1])};
			auto const miss = [&](double const Scale) {
				double in;
				double out;
				tangents_at(Scale, in, out);
				return (from ? in : out) - wanted;
			};
			// the radii grow with the tangent: from those given the way the miss says, doubled or halved until it changes its sign, then
			// the root between, by the regula falsi of Illinois. The transition out of an arc held moves it in, the tangent growing with it:
			// from none, doubled up to one turning by nearly all the deflection
			auto low{lengthened ? 0.0 : 1.0};
			auto lowmiss{miss(low)};
			auto high{low};
			auto highmiss{lowmiss};
			auto const smallest{5.0 / fit.radius};
			auto const largest{1e6 / fit.radius};
			auto const longest{1.96 * fit.arcs.front().radius * deflection};
			while (lengthened && (lowmiss < 0.0) == (highmiss < 0.0) && lowmiss < -1e-5 && high < longest)
			{
				low = high;
				lowmiss = highmiss;
				high = std::min(longest, std::max(1.0, 2.0 * high));
				highmiss = miss(high);
			}
			while (false == lengthened && (lowmiss < 0.0) == (highmiss < 0.0) && high < largest && low > smallest)
			{
				if (lowmiss < 0.0)
				{
					low = high;
					lowmiss = highmiss;
					high *= 2.0;
					highmiss = miss(high);
				}
				else
				{
					high = low;
					highmiss = lowmiss;
					low *= 0.5;
					lowmiss = miss(low);
				}
			}
			// an arc held which begins right there with no transition out of it
			auto const met{lengthened && std::abs(lowmiss) <= 1e-5};
			if (false == met && (lowmiss < 0.0) == (highmiss < 0.0))
			{
				r.errors.emplace_back(from ? format(STR_C("Vertex %zu: no curve goes on from the curve of the adjoining track (R %.0f m) to the straight after it"), k + 1, Design.start_radius) :
				                             format(STR_C("Vertex %zu: no curve goes on from the straight before it into the curve of the adjoining track (R %.0f m)"), k + 1, Design.end_radius));
				return r;
			}
			auto scale{low};
			for (int i = 0, kept = 0; i < 60 && false == met; ++i)
			{
				scale = (low * highmiss - high * lowmiss) / (highmiss - lowmiss);
				auto const value{miss(scale)};
				if (std::abs(value) < 1e-7 || high - low < 1e-12 * high)
					break;
				if ((value < 0.0) == (lowmiss < 0.0))
				{
					low = scale;
					lowmiss = value;
					if (kept == -1)
						highmiss *= 0.5;
					kept = -1;
				}
				else
				{
					high = scale;
					highmiss = value;
					if (kept == 1)
						lowmiss *= 0.5;
					kept = 1;
				}
			}
			if (lengthened)
				beyond = scale;
			else
				sized(scale, fit.arcs);
			fit.radius = fit.arcs.front().radius;
			tangents_at(lengthened ? beyond : 1.0, fit.tangent_in, fit.tangent_out);
			continue;
		}
		if (vertex.compound && false == vertex.arcs.empty())
		{
			fit.compound = true;
			fit.arcs = {{fit.radius, 0.0, vertex.share, vertex.first_cant}};
			for (auto const &arc : vertex.arcs)
				fit.arcs.push_back({std::max(1.0, arc.radius), std::max(0.0, arc.transition), arc.share, arc.cant});
			fit.deflection = deflection;
			fit.tangents = [&, k, directionin, directionout, side, scaled](double const Radius, double &Tangentin, double &Tangentout) {
				auto const &own{fits[k]};
				auto const scale{Radius / own.radius};
				auto const shape{build_compound(own.deflection, scaled(own.arcs, scale), own.transition_in * scale, own.transition_out * scale)};
				auto const normalin{perpendicular(directionin) * static_cast<double>(side)};
				auto const end{directionin * shape.end.x + normalin * shape.end.y};
				auto const along{cross(directionout, end) / cross(directionout, directionin)};
				Tangentin = std::max(0.0, along);
				Tangentout = std::max(0.0, glm::dot(end - directionin * along, directionout));
			};
			fit.tangents(fit.radius, fit.tangent_in, fit.tangent_out);
			continue;
		}
		if (vertex.spirals)
		{
			// the lengths follow from the radius, so a smaller one makes the whole curve smaller in proportion
			fit.spirals = true;
			fit.deflection = deflection;
			fit.transition_in = fit.transition_out = spirals_length(Design.shape, deflection, fit.radius);
			fit.tangents = [&, directionin, directionout, side](double const Radius, double &Tangentin, double &Tangentout) {
				auto const length{spirals_length(Design.shape, deflection, Radius)};
				curve_tangents(Design.shape, length, length, Radius, directionin, directionout, side, Tangentin, Tangentout);
			};
			fit.tangents(fit.radius, fit.tangent_in, fit.tangent_out);
			continue;
		}
		auto const angles = [&](double const Factor) {
			return transition_point(Design.shape, fit.transition_in * Factor, fit.radius, 1.0).angle + transition_point(Design.shape, fit.transition_out * Factor, fit.radius, 1.0).angle;
		};
		if (angles(1.0) > deflection)
		{
			double low{0.0};
			double high{1.0};
			for (int i = 0; i < 50; ++i)
			{
				auto const middle{(low + high) * 0.5};
				(angles(middle) > deflection ? high : low) = middle;
			}
			r.warnings.emplace_back(format(STR_C("Vertex %zu: transition curves shortened to %.1f / %.1f m to fit the deflection angle"), k + 1, fit.transition_in * low, fit.transition_out * low));
			fit.transition_in *= low;
			fit.transition_out *= low;
		}
		fit.deflection = deflection;
		fit.tangents = [&, k, directionin, directionout, side](double const Radius, double &Tangentin, double &Tangentout) {
			curve_tangents(Design.shape, fits[k].transition_in, fits[k].transition_out, Radius, directionin, directionout, side, Tangentin, Tangentout);
		};
		fit.tangents(fit.radius, fit.tangent_in, fit.tangent_out);
	}
	std::vector<double> scales(count, 1.0);
	for (std::size_t leg = 0; leg + 1 < polygon.size(); ++leg)
	{
		auto const available{glm::distance(polygon[leg], polygon[leg + 1]) + (leg == 0 ? startreserve : 0.0) + (leg == count ? endreserve : 0.0)};
		auto const before{leg > 0 ? fits[leg - 1].tangent_out : 0.0};
		auto const after{leg < count ? fits[leg].tangent_in : 0.0};
		if (before + after <= available + 1e-6)
			continue;
		// a curve going on from the adjoining track keeps its size, the one on the other side of the straight makes room for it
		auto const fixedbefore{leg > 0 && fits[leg - 1].curved};
		auto const fixedafter{leg < count && fits[leg].curved};
		if (fixedbefore != fixedafter)
		{
			auto const other{fixedbefore ? after : before};
			if (other > 1e-9)
			{
				auto &kept{scales[fixedbefore ? leg : leg - 1]};
				kept = std::min(kept, std::clamp((available - (fixedbefore ? before : after)) / other, 1e-3, 1.0) * (1.0 - 1e-9));
			}
			continue;
		}
		auto const scale{available / (before + after) * (1.0 - 1e-9)};
		if (leg > 0 && false == fits[leg - 1].curved)
			scales[leg - 1] = std::min(scales[leg - 1], scale);
		if (leg < count && false == fits[leg].curved)
			scales[leg] = std::min(scales[leg], scale);
	}
	for (std::size_t k = 0; k < count; ++k)
	{
		if (scales[k] >= 1.0 || fits[k].tangents == nullptr)
			continue;
		auto &fit{fits[k]};
		auto const budgetin{fit.tangent_in * scales[k]};
		auto const budgetout{fit.tangent_out * scales[k]};
		auto const fits_budget = [&](double const Radius) {
			if (transition_point(Design.shape, fit.transition_in, Radius, 1.0).angle + transition_point(Design.shape, fit.transition_out, Radius, 1.0).angle > fit.deflection)
				return false;
			double tangentin;
			double tangentout;
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
		if (false == fit.compound && false == fit.spirals && fits_budget(low))
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
			fit.arcs = scaled(fit.arcs, scales[k]);
			fit.transition_in *= scales[k];
			fit.transition_out *= scales[k];
		}
		r.warnings.emplace_back(format(STR_C("Vertex %zu: R reduced from %.1f to %.1f m, transitions to %.1f / %.1f m, to fit between the ends"), k + 1, Design.vertices[k].radius, fit.radius, fit.transition_in, fit.transition_out));
		r.radius_warnings.push_back(r.warnings.back());
	}
	for (std::size_t k = 0; k < count; ++k)
	{
		auto &fit{fits[k]};
		auto const &vertex{Design.vertices[k]};
		fit.cant = fit.spirals ? 0.0 : std::max(0.0, vertex.cant);
		if (fit.compound || fit.spirals || fit.deflection < 1e-7 || fit.transition_in + fit.transition_out <= 0.0)
			continue;
		auto const angles = [&](double const Factor) {
			return transition_point(Design.shape, fit.transition_in * Factor, fit.radius, 1.0).angle + transition_point(Design.shape, fit.transition_out * Factor, fit.radius, 1.0).angle;
		};
		// the arc of the norms too short is told, with the radius all of it fits at: the transitions aren't given up for it, that would
		// cost them their own length and the cant with it
		auto const minimum{std::max(Design.norms.arc_min, Design.norms.arc_min_factor * Design.speed)};
		if (auto const arc{fit.radius * (fit.deflection - angles(1.0))}; arc < minimum - 1e-6)
			r.warnings.emplace_back(format(STR_C("Vertex %zu: the circular arc is %.0f m, the norms ask for %.0f m; all of it fits from R %.0f m"), k + 1, arc, minimum, ((vertex.transition_in + vertex.transition_out) * 0.5 + minimum) / fit.deflection));
		if (fit.transition_in < 1.0)
			fit.transition_in = 0.0;
		if (fit.transition_out < 1.0)
			fit.transition_out = 0.0;
		auto const shortest{std::min(fit.transition_in, fit.transition_out)};
		if (fit.cant <= 0.0 || (fit.transition_in >= vertex.transition_in - 1e-6 && fit.transition_out >= vertex.transition_out - 1e-6))
			continue;
		auto const speed{std::max(1.0, Design.speed)};
		auto const ramp{std::min(shortest * 1000.0 / (Design.norms.ramp_factor * speed), Design.norms.cant_rate * 3.6 * shortest / speed)};
		if (ramp >= fit.cant)
			continue;
		fit.cant = ramp < 5.0 ? 0.0 : std::floor(ramp / 5.0) * 5.0;
		r.warnings.emplace_back(format(STR_C("Vertex %zu: cant lowered to %.0f mm, the shortened transition curves can't ramp more"), k + 1, fit.cant));
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
				r.errors.emplace_back(format(STR_C("Vertex %zu lies inside the curve of the previous vertex"), k + 1));
			push_straight(cursor, directionin, lead);
			vertexchainage[k] = chainage;
			cursor = position;
			continue;
		}
		if (geometric > kPi - 1e-3 || (reverse && geometric < 1e-3))
		{
			r.errors.emplace_back(format(STR_C("Vertex %zu: a turn of about 180 or 360 degrees needs one more vertex"), k + 1));
			return r;
		}
		if (fit.compound)
		{
			auto const cants{arc_cants(fit.arcs, fit.cant)};
			auto const shape{build_compound(deflection, fit.arcs, fit.transition_in, fit.transition_out, cants, fit.ends)};
			if (shape.shortened < 1.0)
				r.warnings.emplace_back(format(STR_C("Vertex %zu: transition curves shortened to fit the deflection angle"), k + 1));
			auto const normalin{perpendicular(directionin) * static_cast<double>(side)};
			auto const endoffset{directionin * shape.end.x + normalin * shape.end.y};
			auto const along{cross(directionout, endoffset) / cross(directionout, directionin)};
			auto const curvestart{position - directionin * along};
			auto const straight{glm::dot(curvestart - cursor, directionin)};
			if (k == 0)
				starts_early(straight);
			else if (straight < -1e-4)
				r.errors.emplace_back(format(STR_C("Curves of vertices %zu and %zu overlap by %.2f m"), k, k + 1, -straight));
			push_straight(cursor, directionin, straight);
			auto const startchainage{chainage};
			glm::dvec2 origin{curvestart};
			glm::dvec2 heading{directionin};
			auto sharpest{fit.radius};
			for (auto const &arc : fit.arcs)
				sharpest = std::min(sharpest, arc.radius);
			for (auto const &segment : shape.segments)
			{
				element piece;
				piece.kind = element_kind::spiral;
				piece.vertex = static_cast<int>(k);
				piece.turn = side;
				piece.cant = std::max(segment[3], segment[4]);
				piece.cant_start = segment[3];
				piece.cant_end = segment[4];
				piece.curvature_start = segment[0];
				piece.curvature_end = segment[1];
				piece.radius = std::max(segment[0], segment[1]) > 0.0 ? 1.0 / std::max(segment[0], segment[1]) : 0.0;
				piece.length = segment[2];
				piece.origin = origin;
				piece.direction = heading;
				piece.normal = perpendicular(heading) * static_cast<double>(side);
				push(piece);
				auto const local{spiral_point(segment[0], segment[1], segment[2], segment[2])};
				origin += heading * local.x + piece.normal * local.y;
				heading = turned(heading, side * local.angle);
			}
			vertexchainage[k] = (startchainage + chainage) * 0.5;
			cursor = origin;

			curve_report report;
			report.vertex = static_cast<int>(k);
			report.radius = fit.radius;
			for (auto const &arc : fit.arcs)
				report.radii.push_back(arc.radius);
			report.transition_in = fit.transition_in * shape.shortened;
			report.transition_out = fit.transition_out * shape.shortened;
			report.deflection = deflection;
			report.arc_length = chainage - startchainage;
			report.tangent_in = glm::distance(position, curvestart);
			report.tangent_out = glm::distance(position, cursor);
			report.cants = cants;
			report.curvature_in = fit.ends.from;
			report.cant_in = fit.ends.from_cant;
			report.curvature_out = fit.ends.to;
			report.cant_out = fit.ends.to_cant;
			report.unbalanced = -std::numeric_limits<double>::max();
			for (std::size_t i = 0; i < cants.size(); ++i)
			{
				report.unbalanced = std::max(report.unbalanced, unbalanced_acceleration(Design.speed, fit.arcs[i].radius, cants[i], Design.norms));
				report.cant = std::max(report.cant, cants[i]);
			}
			report.recommended = recommend(Design.speed, sharpest, Design.norms);
			r.curves.push_back(report);
			auto const &norms{Design.norms};
			if (false == norms.checked)
				continue;
			if (sharpest < report.recommended.radius_min - 1e-6)
				r.warnings.emplace_back(format(STR_C("Vertex %zu: R %.0f m is below the minimum %.0f m for %.0f km/h"), k + 1, sharpest, report.recommended.radius_min, Design.speed));
			if (report.unbalanced > norms.unbalanced + 1e-6)
				r.warnings.emplace_back(format(STR_C("Vertex %zu: unbalanced acceleration %.2f m/s2 exceeds %.2f m/s2"), k + 1, report.unbalanced, norms.unbalanced));
			if (report.cant > norms.cant_max + 1e-6)
				r.warnings.emplace_back(format(STR_C("Vertex %zu: cant %.0f mm exceeds %.0f mm"), k + 1, report.cant, norms.cant_max));
			// each transition long enough for the change of the curvature and of the cant along it, at the speed the sharpest arc
			// allows, as the transitions of the norms are set between the arcs
			auto const allowed{std::max(1.0, std::min(Design.speed, speed_limit(sharpest, norms)))};
			// from the curve of an adjoining track, or into it, as between two arcs, at the speed the sharper of them allows
			auto const slower = [&](double const Curvature) { return Curvature > 0.0 ? std::max(1.0, std::min(allowed, speed_limit(1.0 / Curvature, norms))) : allowed; };
			auto const requiredin{fit.ends.from > 0.0 ? between_curves(slower(fit.ends.from), fit.ends.from, fit.ends.from_cant, 1.0 / fit.arcs.front().radius, cants.front(), norms) :
			                                            transition_length(allowed, fit.arcs.front().radius, cants.front(), norms)};
			auto const requiredout{fit.ends.to > 0.0 ? between_curves(slower(fit.ends.to), 1.0 / fit.arcs.back().radius, cants.back(), fit.ends.to, fit.ends.to_cant, norms) :
			                                           transition_length(allowed, fit.arcs.back().radius, cants.back(), norms)};
			// the transition between the curve of the adjoining track and the arc told apart from the one to the straight
			auto const shortin{report.transition_in + 1e-6 < requiredin};
			auto const shortout{report.transition_out + 1e-6 < requiredout};
			auto const betweenin{fit.ends.from > 0.0};
			auto const betweenout{fit.ends.to > 0.0};
			if ((shortin && betweenin) || (shortout && betweenout))
				r.warnings.emplace_back(format(STR_C("Vertex %zu: the transition curve between the curve of the adjoining track and R %.0f m is shorter than the required %.0f m"), k + 1,
				                               shortin && betweenin ? fit.arcs.front().radius : fit.arcs.back().radius, shortin && betweenin ? requiredin : requiredout));
			if ((shortin && false == betweenin) || (shortout && false == betweenout))
				r.warnings.emplace_back(format(STR_C("Vertex %zu: transition curves shorter than the required %.0f m"), k + 1, std::max(shortin && false == betweenin ? requiredin : 0.0, shortout && false == betweenout ? requiredout : 0.0)));
			for (std::size_t i = 1; i < fit.arcs.size(); ++i)
			{
				auto const required{transition_between(allowed, 1.0 / fit.arcs[i - 1].radius, cants[i - 1], 1.0 / fit.arcs[i].radius, cants[i], norms)};
				if (fit.arcs[i].transition * shape.shortened + 1e-6 < required)
					r.warnings.emplace_back(format(STR_C("Vertex %zu: the transition curve between R %.0f and R %.0f m shorter than the required %.0f m"), k + 1, fit.arcs[i - 1].radius, fit.arcs[i].radius, required));
			}
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
		if (arcangle < -1e-9 && false == fit.spirals)
		{
			r.errors.emplace_back(format(STR_C("Vertex %zu: transition curves are too long for the deflection angle"), k + 1));
			return r;
		}

		auto const straight{glm::dot(curvestart - cursor, directionin)};
		if (k == 0)
			starts_early(straight);
		else if (straight < -1e-4)
			r.errors.emplace_back(format(STR_C("Curves of vertices %zu and %zu overlap by %.2f m"), k, k + 1, -straight));
		push_straight(cursor, directionin, straight);

		element curve;
		curve.vertex = static_cast<int>(k);
		curve.radius = radius;
		curve.turn = side;
		curve.cant = fit.cant;
		if (lengthin > 0.0)
		{
			curve.kind = element_kind::transition_in;
			curve.length = lengthin;
			curve.transition = curve.length;
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
		curve.direction = turned(directionin, side * endin.angle);
		vertexchainage[k] = chainage + curve.length * 0.5;
		if (curve.length > 1e-6)
			push(curve);
		if (lengthout > 0.0)
		{
			curve.kind = element_kind::transition_out;
			curve.length = lengthout;
			curve.transition = curve.length;
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
		report.cant = curve.cant;
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
		report.spirals = fit.spirals;
		r.curves.push_back(report);

		auto const &norms{Design.norms};
		if (false == norms.checked)
			continue;
		if (fit.spirals)
		{
			// with no cant all the acceleration across is unbalanced
			if (report.unbalanced > norms.unbalanced + 1e-6)
				r.warnings.emplace_back(format(STR_C("Vertex %zu: the transition curves meet at R %.0f m, with no cant at least %.0f m for %.0f km/h"), k + 1, radius, Design.speed * Design.speed / (12.96 * norms.unbalanced), Design.speed));
		}
		else
		{
			if (radius < report.recommended.radius_min - 1e-6)
				r.warnings.emplace_back(format(STR_C("Vertex %zu: R %.0f m is below the minimum %.0f m for %.0f km/h"), k + 1, radius, report.recommended.radius_min, Design.speed));
			if (report.unbalanced > norms.unbalanced + 1e-6)
				r.warnings.emplace_back(format(STR_C("Vertex %zu: unbalanced acceleration %.2f m/s2 exceeds %.2f m/s2 (cant at least %.0f mm)"), k + 1, report.unbalanced, norms.unbalanced, report.recommended.cant_min));
		}
		if (curve.cant > norms.cant_max + 1e-6)
			r.warnings.emplace_back(format(STR_C("Vertex %zu: cant %.0f mm exceeds %.0f mm"), k + 1, curve.cant, norms.cant_max));
		auto const required{transition_length(Design.speed, radius, curve.cant, norms)};
		if (lengthin + 1e-6 < required || lengthout + 1e-6 < required)
			r.warnings.emplace_back(format(STR_C("Vertex %zu: transition curves shorter than the required %.0f m"), k + 1, required));
		if (curve.cant > 0.0 && (lengthin <= 0.0 || lengthout <= 0.0))
			r.warnings.emplace_back(format(STR_C("Vertex %zu: cant without a transition curve to ramp it"), k + 1));
	}
	auto const lastdirection{glm::normalize(polygon.back() - polygon[polygon.size() - 2])};
	auto const tail{glm::dot(end - cursor, lastdirection)};
	if (tail < 0.0)
		r.end_extension = -tail;
	if (tail < -1e-4 && -tail > endreserve + 1e-4)
	{
		if (endreserve > 0.0)
			r.errors.emplace_back(format(STR_C("Vertex %zu: the curve ends %.2f m past the adjoining straight"), count, -tail - endreserve));
		else
			r.errors.emplace_back(format(STR_C("Vertex %zu: the curve ends %.2f m past the fixed end"), count, -tail));
	}
	push_straight(cursor, lastdirection, tail);
	r.length = chainage;

	if (Design.norms.checked && Design.norms.tangent_min_time > 0.0)
	{
		auto const minimum{Design.norms.tangent_min_time * Design.speed / 3.6};
		for (std::size_t i = 1; i + 1 < r.elements.size(); ++i)
		{
			auto const &e{r.elements[i]};
			if (e.kind == element_kind::straight && e.length < minimum && r.elements[i - 1].kind != element_kind::straight && r.elements[i + 1].kind != element_kind::straight)
				r.warnings.emplace_back(format(STR_C("Straight between curves is %.1f m long, less than %.0f m"), e.length, minimum));
		}
	}

	r.vertex_chainages = vertexchainage;
	r.profile.push_back({0.0, Design.start.y, 0.0, 0.0});
	auto const radius{std::max(Design.norms.vertical_min, Design.norms.vertical_factor * Design.speed * Design.speed)};
	for (std::size_t k = 0; k < count; ++k)
	{
		auto const elevation{Design.start.y + (Design.end.y - Design.start.y) * vertexchainage[k] / std::max(r.length, 1e-6)};
		r.vertex_elevations.push_back(elevation);
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
	// a curved track adjoining is a jump of the curvature only where the design meets it at another one: a curve going on from
	// the diverging track of a switch at its radius isn't. The curvature of the design there is signed by the side it turns to, a
	// piece shorter than 1 cm, as a straight the fit leaves before the curve, passed over
	auto const curvature_at = [&](bool const End) {
		for (std::size_t n = 0; n < r.elements.size(); ++n)
		{
			auto const &e{r.elements[End ? r.elements.size() - 1 - n : n]};
			if (e.length < 0.01)
				continue;
			auto const [start, end]{end_curvatures(e)};
			return (End ? end : start) * e.turn;
		}
		return 0.0;
	};
	auto const abrupt = [](double const Radius, int const Turn, double const Own) {
		if (Radius <= 0.0 || Radius >= 10000.0)
			return false;
		// the side not known, as of a design kept in the scenery before it was: taken for the one the design turns to
		auto const theirs{(Turn != 0 ? Turn : Own < 0.0 ? -1 : 1) / Radius};
		return std::abs(theirs - Own) > 0.05 * std::max(1.0 / Radius, std::abs(Own));
	};
	// a design asked to go on from the curve where its curve turns its way leaves the end where it doesn't to the one laying it: it eases
	// out of the curve there, or keeps the jump at the end of the diverging track of a switch, as the switch has it at its start
	if (false == Design.start_curved && abrupt(Design.start_radius, Design.start_turn, curvature_at(false)))
		r.warnings.emplace_back(format(STR_C("The adjoining track at the start is curved (R %.0f m), the curvature changes abruptly"), Design.start_radius));
	if (false == Design.end_curved && abrupt(Design.end_radius, Design.end_turn, curvature_at(true)))
		r.warnings.emplace_back(format(STR_C("The adjoining track at the end is curved (R %.0f m), the curvature changes abruptly"), Design.end_radius));
	make_cants(Design, r);

	r.valid = r.errors.empty() && r.length > 1e-3;
	return r;
}

glm::dvec3 along_end(glm::dvec3 const &Point, glm::dvec2 const &Direction, double const Grade, double const Distance)
{
	auto const direction{glm::normalize(Direction)};
	return {Point.x + direction.x * Distance, Point.y + Grade * Distance, Point.z + direction.y * Distance};
}

} // namespace

std::pair<double, double> end_curvatures(element const &Element)
{
	auto const own{Element.radius > 0.0 ? 1.0 / Element.radius : 0.0};
	switch (Element.kind)
	{
	case element_kind::transition_in: return {0.0, own};
	case element_kind::arc: return {own, own};
	case element_kind::transition_out: return {own, 0.0};
	case element_kind::spiral: return {Element.curvature_start, Element.curvature_end};
	default: return {0.0, 0.0};
	}
}

result compute(design const &Design)
{
	auto first{fit_between(Design)};
	auto const startextension{first.start_extension > 1e-4 ? first.start_extension : 0.0};
	auto const endextension{first.end_extension > 1e-4 ? first.end_extension : 0.0};
	if (false == first.errors.empty() || (startextension == 0.0 && endextension == 0.0))
	{
		first.start_extension = 0.0;
		first.end_extension = 0.0;
		return first;
	}
	auto extended{Design};
	extended.start = along_end(Design.start, Design.start_direction, Design.start_grade, -startextension);
	extended.end = along_end(Design.end, Design.end_direction, Design.end_grade, endextension);
	extended.start_reserve = 0.0;
	extended.end_reserve = 0.0;
	// a curve going on from the adjoining track begins right at the end, which isn't moved; an end moved is on a straight
	extended.start_curved = Design.start_curved && startextension == 0.0;
	extended.end_curved = Design.end_curved && endextension == 0.0;
	if (extended.vertices.size() >= 2)
	{
		extended.vertices.front().offset += startextension;
		extended.vertices.back().offset += endextension;
	}
	auto r{fit_between(extended)};
	r.start_extension = startextension;
	r.end_extension = endextension;
	return r;
}

double speed_limit(double const Radius, limits const &Limits)
{
	if (Radius <= 0.0)
		return 0.0;
	auto const factor{Limits.gauge / (kGravity * 12.96)};
	auto const allowance{Limits.gauge * Limits.unbalanced / kGravity};
	return std::sqrt(Radius * (Limits.cant_max + allowance) / factor);
}

result compute_recommended(design &Design)
{
	for (auto &vertex : Design.vertices)
	{
		vertex.cant = 0.0;
		vertex.transition_in = 0.0;
		vertex.transition_out = 0.0;
	}
	auto r{compute(Design)};
	if (false == r.valid)
		return r;
	auto const speed{Design.speed};
	for (int i = 0; i < 3; ++i)
	{
		bool accepted{false};
		for (auto const share : {1.0, 0.75, 0.5, 0.35, 0.25})
		{
			auto attempt{Design};
			auto slowest{speed};
			for (auto const &curve : r.curves)
			{
				if (curve.vertex < 0 || curve.vertex >= static_cast<int>(attempt.vertices.size()) || attempt.vertices[curve.vertex].spirals)
					continue;
				auto const allowed{std::max(1.0, std::min(speed, speed_limit(curve.radius, attempt.norms)))};
				auto const recommended{recommend(allowed, curve.radius, attempt.norms)};
				auto const length{recommended.transition * share};
				auto &vertex{attempt.vertices[curve.vertex]};
				vertex.transition_out = length;
				vertex.transition_in = vertex.transition_out;
				vertex.cant = std::min({recommended.cant, length * 1000.0 / (attempt.norms.ramp_factor * allowed), attempt.norms.cant_rate * 3.6 * length / allowed});
				vertex.cant = vertex.cant < 5.0 ? 0.0 : std::floor(vertex.cant / 5.0) * 5.0;
				slowest = std::min(slowest, allowed);
			}
			attempt.speed = slowest;
			auto next{compute(attempt)};
			if (false == next.valid)
				continue;
			auto const kept{std::all_of(next.curves.begin(), next.curves.end(), [&](curve_report const &Curve) {
				auto const &vertex{attempt.vertices[Curve.vertex]};
				return Curve.transition_in >= 0.95 * vertex.transition_in && Curve.transition_out >= 0.95 * vertex.transition_out;
			})};
			if (false == kept && share > 0.25)
				continue;
			Design = attempt;
			r = next;
			accepted = true;
			break;
		}
		if (false == accepted)
			break;
	}
	return r;
}

namespace
{

double turning(result const &Result)
{
	double total{0.0};
	for (auto const &e : Result.elements)
	{
		if (e.radius <= 0.0)
			continue;
		if (e.kind == element_kind::arc)
			total += e.length / e.radius;
		else if (e.kind == element_kind::transition_in || e.kind == element_kind::transition_out)
			total += e.length / (2.0 * e.radius);
	}
	return total;
}

void spread_rolls(std::vector<segment_data> &Pieces, double const Start, double const End)
{
	auto const count{static_cast<double>(Pieces.size())};
	for (std::size_t i = 0; i < Pieces.size(); ++i)
		Pieces[i].rolls = {static_cast<float>(Start + (End - Start) * i / count), static_cast<float>(Start + (End - Start) * (i + 1) / count)};
}

} // namespace

namespace
{

struct join_frame
{
	join_request const &request;
	glm::dvec2 start_direction;
	glm::dvec2 end_direction;
	glm::dvec2 chord;
	double distance;
	double tolerance;
	double heading;
	double rise;
};

std::optional<join_result> straight_join(join_frame const &Frame)
{
	auto const &a{Frame.request.start};
	auto const &b{Frame.request.end};
	auto const kink{std::max(std::abs(geometry::signed_angle(Frame.start_direction, Frame.chord)), std::abs(geometry::signed_angle(Frame.chord, Frame.end_direction)))};
	if (kink > Frame.tolerance)
		return std::nullopt;
	join_result straight;
	straight.kind = join_kind::straight;
	straight.kink = kink;
	segment_data path;
	path.points[segment_data::point::start] = a.position;
	path.points[segment_data::point::end] = b.position;
	straight.pieces = {path};
	spread_rolls(straight.pieces, a.roll, b.roll);
	straight.length = glm::distance(a.position, b.position);
	return straight;
}

std::optional<join_result> arc_join(join_frame const &Frame)
{
	auto const &a{Frame.request.start};
	auto const &b{Frame.request.end};
	auto const lateral{cross(Frame.start_direction, Frame.chord)};
	if (std::abs(lateral) <= 1e-6)
		return std::nullopt;
	auto const angle{2.0 * std::atan2(std::abs(lateral), glm::dot(Frame.start_direction, Frame.chord))};
	int const side{lateral > 0.0 ? 1 : -1};
	auto const radius{Frame.distance * Frame.distance / (2.0 * std::abs(lateral))};
	auto const kink{std::abs(geometry::signed_angle(turned(Frame.start_direction, side * angle), Frame.end_direction))};
	if (angle >= kPi - 1e-3 || kink > Frame.tolerance)
		return std::nullopt;
	join_result arc;
	arc.kind = join_kind::arc;
	arc.kink = kink;
	arc.pieces = geometry::arc_pieces(a.position, Frame.start_direction, Frame.rise / (radius * angle), radius, angle, side);
	arc.pieces.back().points[segment_data::point::end] = b.position;
	spread_rolls(arc.pieces, a.roll, b.roll);
	arc.radius = radius;
	arc.length = radius * angle;
	return arc;
}

design join_design(join_frame const &Frame, join_kind const Kind)
{
	auto const &request{Frame.request};
	design shape;
	shape.norms = request.norms;
	shape.shape = request.shape;
	shape.speed = request.speed;
	shape.arc_piece_angle = 90.0;
	shape.start = request.start.position;
	shape.end = request.end.position;
	shape.start_direction = Frame.start_direction;
	shape.end_direction = Frame.end_direction;
	shape.start_grade = Frame.rise / Frame.distance;
	shape.end_grade = shape.start_grade;
	vertex corner;
	corner.radius = request.radius > 0.0 ? request.radius : 100000.0;
	if (Kind == join_kind::curve)
		shape.vertices = {corner};
	else
	{
		corner.offset = Frame.distance / 4.0;
		shape.vertices = {corner, corner};
	}
	return shape;
}

std::optional<join_result> fitted_join(join_frame const &Frame, join_kind const Kind)
{
	auto shape{join_design(Frame, Kind)};
	auto const r{compute_recommended(shape)};
	auto const sharp{std::any_of(r.curves.begin(), r.curves.end(), [](curve_report const &Report) { return Report.radius < 10.0; })};
	if (false == r.valid || sharp || turning(r) > Frame.heading + kPi + 1e-6 || r.length > 3.0 * Frame.distance + 1.0)
		return std::nullopt;
	join_result curve;
	curve.kind = Kind;
	curve.pieces = pieces(r, shape, 0);
	if (curve.pieces.empty())
		return curve;
	curve.pieces.front().rolls[0] = static_cast<float>(Frame.request.start.roll);
	curve.pieces.back().rolls[1] = static_cast<float>(Frame.request.end.roll);
	curve.length = r.length;
	curve.speed = shape.speed;
	curve.warnings = r.warnings;
	curve.radius_warnings = r.radius_warnings;
	curve.radius = r.curves.empty() ? 0.0 : std::numeric_limits<double>::max();
	for (auto const &report : r.curves)
	{
		curve.radius = std::min(curve.radius, report.radius);
		curve.transition = std::max({curve.transition, report.transition_in, report.transition_out});
	}
	for (auto const &e : r.elements)
		curve.cant = std::max(curve.cant, e.cant);
	return curve;
}

std::optional<join_result> curve_join(join_frame const &Frame)
{
	design probe;
	probe.start = Frame.request.start.position;
	probe.end = Frame.request.end.position;
	probe.start_direction = Frame.start_direction;
	probe.end_direction = Frame.end_direction;
	glm::dvec2 intersection;
	return fitted_join(Frame, tangent_intersection(probe, intersection) ? join_kind::curve : join_kind::reverse_curve);
}

join_result bezier_join(join_frame const &Frame)
{
	auto const &a{Frame.request.start};
	auto const &b{Frame.request.end};
	join_result fallback;
	fallback.kind = join_kind::bezier;
	auto const grade{Frame.rise / Frame.distance};
	auto const handle{Frame.distance / 3.0};
	segment_data path;
	path.points[segment_data::point::start] = a.position;
	path.points[segment_data::point::end] = b.position;
	path.points[segment_data::point::control1] = glm::dvec3{Frame.start_direction.x, grade, Frame.start_direction.y} * handle;
	path.points[segment_data::point::control2] = -glm::dvec3{Frame.end_direction.x, grade, Frame.end_direction.y} * handle;
	fallback.pieces = {path};
	spread_rolls(fallback.pieces, a.roll, b.roll);
	fallback.length = Frame.distance;
	return fallback;
}

} // namespace

join_result join(join_request const &Request)
{
	auto const &a{Request.start};
	auto const &b{Request.end};
	auto const chord{plan_of(b.position) - plan_of(a.position)};
	auto const distance{glm::length(chord)};
	if (distance < 0.05 || glm::length(a.direction) < 1e-9 || glm::length(b.direction) < 1e-9)
		return {};
	auto const startdirection{glm::normalize(a.direction)};
	auto const enddirection{glm::normalize(b.direction)};
	join_frame const frame{Request, startdirection, enddirection, chord, distance, glm::radians(Request.kink), std::abs(geometry::signed_angle(startdirection, enddirection)), b.position.y - a.position.y};
	std::vector<join_result> candidates;
	std::vector<join_result> kinked;
	for (auto const &candidate : {straight_join(frame), arc_join(frame), curve_join(frame)})
		if (candidate.has_value())
			(candidate->kink > 0.1 * frame.tolerance ? kinked : candidates).push_back(*candidate);
	candidates.insert(candidates.end(), kinked.begin(), kinked.end());
	if (candidates.empty())
		candidates.push_back(bezier_join(frame));
	auto const count{static_cast<int>(candidates.size())};
	auto chosen{candidates[((Request.variant % count) + count) % count]};
	chosen.variants = count;
	if (chosen.speed <= 0.0)
		chosen.speed = Request.speed;
	return chosen;
}

namespace
{

// elements laid one after another from a point in a direction, put together the way the alignment computes them; the cant along
// them is kept apart, a transition between two canted arcs ramps it from the one to the other
struct element_chain
{
	result shape;
	glm::dvec2 position{0.0};
	glm::dvec2 direction{0.0, 1.0};
	std::vector<std::array<double, 2>> cants; // chainage, cant
};

void chain_push(element_chain &Chain, element Element, double const Cant0, double const Cant1)
{
	Element.chainage = Chain.shape.length;
	Chain.shape.length += Element.length;
	Chain.shape.elements.push_back(Element);
	Chain.cants.push_back({Element.chainage, Cant0});
	Chain.cants.push_back({Chain.shape.length, Cant1});
}

void chain_straight(element_chain &Chain, double const Length)
{
	if (Length < 1e-6)
		return;
	element straight;
	straight.kind = element_kind::straight;
	straight.origin = Chain.position;
	straight.direction = Chain.direction;
	straight.length = Length;
	Chain.position += Chain.direction * Length;
	chain_push(Chain, straight, 0.0, 0.0);
}

void chain_arc(element_chain &Chain, double const Curvature, double const Length, double const Cant)
{
	if (Length < 1e-6)
		return;
	if (std::abs(Curvature) < 1e-9)
	{
		chain_straight(Chain, Length);
		return;
	}
	element arc;
	arc.kind = element_kind::arc;
	arc.turn = Curvature > 0.0 ? 1 : -1;
	arc.radius = 1.0 / std::abs(Curvature);
	arc.length = Length;
	arc.origin = Chain.position + perpendicular(Chain.direction) / Curvature;
	arc.normal = Chain.position - arc.origin;
	arc.direction = Chain.direction;
	auto const angle{arc.turn * Length / arc.radius};
	Chain.position = arc.origin + turned(arc.normal, angle);
	Chain.direction = turned(Chain.direction, angle);
	chain_push(Chain, arc, Cant, Cant);
}

// a curvature going on through 0 turns the curve the other way: it's kept signed, the element turning to the left
void chain_spiral(element_chain &Chain, double const From, double const To, double const Length, double const Cant0, double const Cant1)
{
	if (Length < 1e-6)
		return;
	int const turn{(From * To < 0.0 || From + To >= 0.0) ? 1 : -1};
	element spiral;
	spiral.kind = element_kind::spiral;
	spiral.turn = turn;
	spiral.curvature_start = From * turn;
	spiral.curvature_end = To * turn;
	auto const peak{std::max(std::abs(From), std::abs(To))};
	spiral.radius = peak > 1e-12 ? 1.0 / peak : 0.0;
	spiral.length = Length;
	spiral.origin = Chain.position;
	spiral.direction = Chain.direction;
	spiral.normal = perpendicular(Chain.direction) * static_cast<double>(turn);
	auto const local{spiral_point(spiral.curvature_start, spiral.curvature_end, Length, Length)};
	Chain.position = spiral.origin + spiral.direction * local.x + spiral.normal * local.y;
	Chain.direction = glm::normalize(spiral.direction * std::cos(local.angle) + spiral.normal * std::sin(local.angle));
	chain_push(Chain, spiral, Cant0, Cant1);
}

lay_result chain_pieces(element_chain &Chain, lay_request const &Request)
{
	auto &shape{Chain.shape};
	shape.gauge = Request.norms.gauge;
	shape.start = Request.start.position;
	shape.end = {Chain.position.x, Request.target.y, Chain.position.y};
	shape.profile = {{0.0, Request.start.position.y, 0.0, 0.0}, {shape.length, Request.target.y, 0.0, 0.0}};
	shape.valid = shape.length > 1e-3;
	design style;
	style.norms = Request.norms;
	lay_result result;
	result.length = shape.length;
	auto const count{std::max(minimum_pieces(shape, style), static_cast<std::size_t>(std::ceil(shape.length / std::max(5.0, Request.piece_length) - 1e-9)))};
	result.pieces = pieces(shape, style, count);
	auto const cant_at = [&](double const Chainage) {
		auto const &cants{Chain.cants};
		for (std::size_t i = 0; i + 1 < cants.size(); i += 2)
			if (Chainage <= cants[i + 1][0] + 1e-6)
			{
				auto const span{cants[i + 1][0] - cants[i][0]};
				auto const t{span > 1e-9 ? std::clamp((Chainage - cants[i][0]) / span, 0.0, 1.0) : 1.0};
				return cants[i][1] + (cants[i + 1][1] - cants[i][1]) * t;
			}
		return cants.empty() ? 0.0 : cants.back()[1];
	};
	auto const roll = [&](double const Cant) { return static_cast<float>(glm::degrees(std::asin(std::clamp(Cant / Request.norms.gauge, -1.0, 1.0)))); };
	double chainage{0.0};
	for (auto &piece : result.pieces)
	{
		piece.rolls[0] = roll(cant_at(chainage));
		chainage += geometry::bezier{piece}.plan_length();
		piece.rolls[1] = roll(cant_at(chainage));
	}
	if (false == result.pieces.empty())
		result.pieces.front().rolls[0] = static_cast<float>(Request.start.roll);
	return result;
}

// a transition of the length given from the curvature of the start, as the position and the direction it ends with
struct spiral_end
{
	double x{0.0};
	double y{0.0};
	double angle{0.0};
	int turn{1};

	spiral_end(double const From, double const To, double const Length)
	{
		turn = (From * To < 0.0 || From + To >= 0.0) ? 1 : -1;
		auto const local{spiral_point(From * turn, To * turn, Length, Length)};
		x = local.x;
		y = local.y;
		angle = local.angle;
	}
	void apply(glm::dvec2 &Position, glm::dvec2 &Direction) const
	{
		auto const normal{perpendicular(Direction) * static_cast<double>(turn)};
		Position += Direction * x + normal * y;
		Direction = glm::normalize(Direction * std::cos(angle) + normal * std::sin(angle));
	}
};

void arc_end(glm::dvec2 &Position, glm::dvec2 &Direction, double const Curvature, double const Length)
{
	auto const centre{Position + perpendicular(Direction) / Curvature};
	auto const angle{Curvature * Length};
	Position = centre + turned(Position - centre, angle);
	Direction = turned(Direction, angle);
}

} // namespace

std::optional<lay_result> lay_on(lay_request const &Request)
{
	auto const &norms{Request.norms};
	if (glm::length(Request.start.direction) < 1e-9)
		return std::nullopt;
	glm::dvec2 const start{Request.start.position.x, Request.start.position.z};
	auto const direction{glm::normalize(Request.start.direction)};
	glm::dvec2 const target{Request.target.x, Request.target.z};
	auto const offset{target - start};
	auto const chord{glm::length(offset)};
	auto const lateral{cross(direction, offset)};
	if (chord < 0.5 || glm::dot(offset, direction) <= 0.0)
		return std::nullopt;
	auto const k0{Request.curvature};
	auto const c0{norms.gauge * std::sin(glm::radians(Request.start.roll))};
	if (Request.kind == lay_kind::straight && std::abs(k0) < 1e-9 && std::abs(c0) < 5.0)
		return std::nullopt;
	if (Request.kind == lay_kind::arc && std::abs(lateral) < 0.005 * chord + 0.05 && std::abs(k0) < 1e-9 && std::abs(c0) < 5.0)
		return std::nullopt;
	int const side{lateral > 0.0 ? 1 : -1};
	// the speed the arc allows and its cant; a transition is ramped for the lower speed of its two ends
	auto const allowed = [&](double const Curvature) { return std::abs(Curvature) < 1e-9 ? Request.speed : std::max(1.0, std::min(Request.speed, speed_limit(1.0 / std::abs(Curvature), norms))); };
	auto const cant_of = [&](double const Curvature) { return std::abs(Curvature) < 1e-9 ? 0.0 : std::copysign(recommend(allowed(Curvature), 1.0 / std::abs(Curvature), norms).cant, Curvature); };
	auto const slower = [&](double const From, double const To) { return std::min(allowed(From), allowed(To)); };
	auto const between = [&](double const From, double const Fromcant, double const To, double const Tocant) {
		if (std::abs(To - From) <= 0.05 * std::max(std::abs(From), std::abs(To)) && std::abs(Tocant - Fromcant) < 5.0)
			return 0.0;
		return transition_between(slower(From, To), From, Fromcant, To, Tocant, norms);
	};
	// the change of the cant a transition of the length ramps at the speed
	auto const ramped = [&](double const Length, double const Speed) { return std::min(Length * 1000.0 / (norms.ramp_factor * Speed), norms.cant_rate * 3.6 * Length / Speed); };

	// the transitions of the norms cut to the share, the cant of the arc lowered to what they can ramp; none where the cant of the
	// end can't be ramped to one leaning into the arc
	struct layout_plan
	{
		double out{0.0}; // out of the curvature of the end to the straight, before the straight of the layout
		double entry{0.0};
		double exit{0.0};
		double cant{0.0};
		double from{0.0}; // the curvature and the cant the entry starts from
		double fromcant{0.0};
		bool lowered{false};
	};
	auto const planned = [&](double const Curvature, bool const Lead, bool const Exit, double const Share) -> std::optional<layout_plan> {
		layout_plan result;
		auto const wanted{cant_of(Curvature)};
		if (Lead)
		{
			result.out = between(k0, c0, 0.0, 0.0) * Share;
			if (std::abs(c0) > ramped(result.out, slower(k0, 0.0)) + 1e-6)
				return std::nullopt;
		}
		else
		{
			result.from = k0;
			result.fromcant = c0;
		}
		result.entry = between(result.from, result.fromcant, Curvature, wanted) * Share;
		result.exit = Exit ? between(Curvature, wanted, 0.0, 0.0) * Share : 0.0;
		auto cant{wanted};
		if (result.entry > 0.0)
		{
			auto const reach{ramped(result.entry, slower(result.from, Curvature))};
			cant = std::clamp(cant, result.fromcant - reach, result.fromcant + reach);
		}
		else
			cant = result.fromcant;
		if (Exit)
		{
			auto const reach{result.exit > 0.0 ? ramped(result.exit, allowed(Curvature)) : 0.0};
			cant = std::clamp(cant, -reach, reach);
		}
		if (cant * Curvature < -1e-9 || (result.entry > 0.0 && std::abs(cant - result.fromcant) > ramped(result.entry, slower(result.from, Curvature)) + 1e-6) || (result.entry <= 0.0 && std::abs(cant - result.fromcant) > 1e-6))
			return std::nullopt;
		result.cant = cant;
		result.lowered = std::abs(cant) < std::abs(wanted) - 0.5;
		return result;
	};
	auto const begin = [&]() {
		element_chain chain;
		chain.position = start;
		chain.direction = direction;
		return chain;
	};
	auto const made = [&](element_chain &Chain, double const Curvature, layout_plan const &Plan, double const Share) {
		auto result{chain_pieces(Chain, Request)};
		result.radius = std::abs(Curvature) > 1e-9 ? 1.0 / std::abs(Curvature) : 0.0;
		result.cant = std::abs(Plan.cant);
		result.entry = Plan.out > 0.0 ? Plan.out : Plan.entry;
		result.exit = Plan.out > 0.0 ? Plan.entry : Plan.exit;
		result.from_radius = std::abs(k0) > 1e-9 ? 1.0 / std::abs(k0) : 0.0;
		result.compound = k0 * Curvature > 0.0 && Plan.out <= 0.0 && Plan.entry > 0.0;
		if (Share < 1.0)
			result.note = format(STR_C("The transition curves are cut to %.0f%% of what the norms ask for, there's no more room"), Share * 100.0);
		if (Plan.lowered)
			result.note += (result.note.empty() ? "" : "\n") + format(STR_C("The cant is lowered to %.0f mm, what the transition curves ramp"), std::abs(Plan.cant));
		return result;
	};

	auto const arc_through = [&](double const Share) -> std::optional<lay_result> {
		auto const reference{2.0 * lateral / (chord * chord)};
		auto curvature{reference};
		std::optional<layout_plan> plan;
		// the transition and the cant follow the curvature found, which moves with them
		for (int pass = 0; pass < 4; ++pass)
		{
			plan = planned(curvature, false, false, Share);
			if (false == plan.has_value())
				return std::nullopt;
			// over the curvatures turning to the target, from flat to sharp, it goes from inside of the circle of the arc to outside
			auto const outside = [&](double const Curvature) {
				auto position{start};
				auto heading{direction};
				spiral_end{k0, Curvature, plan->entry}.apply(position, heading);
				return glm::distance(target, position + perpendicular(heading) / Curvature) - 1.0 / std::abs(Curvature);
			};
			auto low{std::log(1e-6)};
			auto high{std::log(0.1)};
			if (outside(side * std::exp(low)) >= 0.0 || outside(side * std::exp(high)) <= 0.0)
				return std::nullopt;
			for (int i = 0; i < 60; ++i)
			{
				auto const middle{(low + high) * 0.5};
				(outside(side * std::exp(middle)) < 0.0 ? low : high) = middle;
			}
			curvature = side * std::exp((low + high) * 0.5);
		}
		// a transition too long for the way to the target squeezes the arc after it, a shorter one is tried then
		if (std::abs(curvature) > 1.3 * std::abs(reference) + 1e-5)
			return std::nullopt;
		auto chain{begin()};
		chain_spiral(chain, k0, curvature, plan->entry, c0, plan->cant);
		auto const centre{chain.position + perpendicular(chain.direction) / curvature};
		auto const angle{geometry::signed_angle(chain.position - centre, target - centre) * side};
		if (angle <= 1e-4 || angle >= kPi)
			return std::nullopt;
		chain_arc(chain, curvature, angle / std::abs(curvature), plan->cant);
		return made(chain, curvature, *plan, Share);
	};

	auto const arc_straight = [&](double const Share) -> std::optional<lay_result> {
		auto const curvature{side / std::max(1.0, Request.radius)};
		auto const plan{planned(curvature, false, true, Share)};
		if (false == plan.has_value())
			return std::nullopt;
		auto position{start};
		auto heading{direction};
		spiral_end{k0, curvature, plan->entry}.apply(position, heading);
		spiral_end const out{curvature, 0.0, plan->exit};
		// the arc grows until the straight after the transition out of it runs through the target
		auto const miss = [&](double const Arc) {
			auto at{position};
			auto towards{heading};
			arc_end(at, towards, curvature, Arc);
			out.apply(at, towards);
			return std::array<double, 2>{cross(towards, target - at) * side, glm::dot(towards, target - at)};
		};
		if (miss(0.0)[0] <= 0.0)
			return std::nullopt;
		auto const radius{1.0 / std::abs(curvature)};
		auto const step{std::max(0.5, radius * glm::radians(2.0))};
		double low{0.0};
		double high{-1.0};
		for (auto arc{step}; arc < 2.0 * kPi * radius; arc += step)
		{
			if (miss(arc)[0] <= 0.0)
			{
				high = arc;
				break;
			}
			low = arc;
		}
		if (high < 0.0)
			return std::nullopt;
		for (int i = 0; i < 60; ++i)
		{
			auto const middle{(low + high) * 0.5};
			(miss(middle)[0] > 0.0 ? low : high) = middle;
		}
		auto const arc{(low + high) * 0.5};
		auto const straight{miss(arc)[1]};
		if (straight < 0.0)
			return std::nullopt;
		auto chain{begin()};
		chain_spiral(chain, k0, curvature, plan->entry, c0, plan->cant);
		chain_arc(chain, curvature, arc, plan->cant);
		chain_spiral(chain, curvature, 0.0, plan->exit, plan->cant, 0.0);
		chain_straight(chain, straight);
		return made(chain, curvature, *plan, Share);
	};

	auto const straight_arc = [&](double const Share) -> std::optional<lay_result> {
		auto const curvature{side / std::max(1.0, Request.radius)};
		auto const plan{planned(curvature, true, false, Share)};
		if (false == plan.has_value())
			return std::nullopt;
		auto lead{begin()};
		chain_spiral(lead, k0, 0.0, plan->out, c0, 0.0);
		spiral_end const into{0.0, curvature, plan->entry};
		// the end of the arc with no straight before it; the straight moves it along the lead, the arc sideways
		auto const end = [&](double const Arc) {
			auto at{lead.position};
			auto towards{lead.direction};
			into.apply(at, towards);
			arc_end(at, towards, curvature, Arc);
			return at;
		};
		auto const miss = [&](double const Arc) { return cross(lead.direction, target - end(Arc)) * side; };
		auto const most{kPi / std::abs(curvature)};
		if (miss(0.0) <= 0.0 || miss(most) > 0.0)
			return std::nullopt;
		double low{0.0};
		double high{most};
		for (int i = 0; i < 60; ++i)
		{
			auto const middle{(low + high) * 0.5};
			(miss(middle) > 0.0 ? low : high) = middle;
		}
		auto const arc{(low + high) * 0.5};
		auto const straight{glm::dot(lead.direction, target - end(arc))};
		if (straight < 0.5)
			return std::nullopt;
		auto chain{lead};
		chain_straight(chain, straight);
		chain_spiral(chain, 0.0, curvature, plan->entry, 0.0, plan->cant);
		chain_arc(chain, curvature, arc, plan->cant);
		return made(chain, curvature, *plan, Share);
	};

	auto const straight = [&](double const Share) -> std::optional<lay_result> {
		auto const plan{planned(0.0, true, false, Share)};
		if (false == plan.has_value())
			return std::nullopt;
		auto chain{begin()};
		chain_spiral(chain, k0, 0.0, plan->out, c0, 0.0);
		auto const along{glm::dot(chain.direction, target - chain.position)};
		if (along < 0.5)
			return std::nullopt;
		chain_straight(chain, along);
		return made(chain, 0.0, *plan, Share);
	};

	auto const first = [](auto const &Layout) -> std::optional<lay_result> {
		for (auto const share : {1.0, 0.75, 0.5, 0.35, 0.25})
			if (auto laid{Layout(share)}; laid.has_value())
				return laid;
		return std::nullopt;
	};
	std::optional<lay_result> laid;
	switch (Request.kind)
	{
	case lay_kind::straight: laid = first(straight); break;
	case lay_kind::arc: laid = first(arc_through); break;
	case lay_kind::straight_arc:
		laid = first(straight_arc);
		if (false == laid.has_value())
			laid = first(arc_through);
		break;
	default:
		laid = first(arc_straight);
		if (false == laid.has_value())
			laid = first(arc_through);
		break;
	}
	if (laid.has_value())
		return laid;
	lay_result refused;
	refused.note = std::abs(c0) >= 5.0 ? format(STR_C("No room to ramp the cant of %.0f mm the track ends with, the track is laid without transition curves"), std::abs(c0))
	                                   : std::string{STR_C("No room for the transition curves of the norms, the track is laid without them")};
	return refused;
}

void recommend_between(vertex &Vertex, double const Speed, limits const &Limits)
{
	if (false == Vertex.compound || Vertex.arcs.empty())
		return;
	auto sharpest{std::max(1.0, Vertex.radius)};
	for (auto const &arc : Vertex.arcs)
		sharpest = std::min(sharpest, std::max(1.0, arc.radius));
	auto const speed{std::max(1.0, std::min(Speed, speed_limit(sharpest, Limits)))};
	auto const cants{compound_cants(Vertex)};
	auto previous{std::max(1.0, Vertex.radius)};
	for (std::size_t i = 0; i < Vertex.arcs.size(); ++i)
	{
		auto &arc{Vertex.arcs[i]};
		auto const radius{std::max(1.0, arc.radius)};
		arc.transition = transition_between(speed, 1.0 / previous, cants[i], 1.0 / radius, cants[i + 1], Limits);
		previous = radius;
	}
}

void norm_transitions(vertex &Vertex, double const Speed, limits const &Limits)
{
	if (Vertex.spirals)
		return;
	if (false == Vertex.compound || Vertex.arcs.empty())
	{
		Vertex.transition_in = Vertex.transition_out = transition_length(std::max(1.0, Speed), std::max(1.0, Vertex.radius), std::max(0.0, Vertex.cant), Limits);
		return;
	}
	// at the speed the sharpest arc allows, as the warnings of compute check them
	auto sharpest{std::max(1.0, Vertex.radius)};
	for (auto const &arc : Vertex.arcs)
		sharpest = std::min(sharpest, std::max(1.0, arc.radius));
	auto const allowed{std::max(1.0, std::min(Speed, speed_limit(sharpest, Limits)))};
	auto const cants{compound_cants(Vertex)};
	Vertex.transition_in = transition_length(allowed, std::max(1.0, Vertex.radius), cants.front(), Limits);
	Vertex.transition_out = transition_length(allowed, std::max(1.0, Vertex.arcs.back().radius), cants.back(), Limits);
	recommend_between(Vertex, Speed, Limits);
}

bool tangent_intersection(design const &Design, glm::dvec2 &Point)
{
	auto const start{plan_of(Design.start)};
	auto const offset{plan_of(Design.end) - start};
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

bool spiral_tangents(transition_shape const Shape, double const TurnIn, double const TurnOut, double &TangentIn, double &TangentOut)
{
	auto const length = [&](double const Turn) { return Shape == transition_shape::clothoid ? 2.0 * Turn : 2.0 * std::tan(Turn); };
	if (TurnIn <= 0.0 || TurnOut <= 0.0 || TurnIn >= 0.5 * kPi || TurnOut >= 0.5 * kPi)
		return false;
	auto const in{transition_point(Shape, length(TurnIn), 1.0, 1.0)};
	auto const out{transition_point(Shape, length(TurnOut), 1.0, 1.0)};
	auto const deflection{in.angle + out.angle};
	if (deflection >= kPi - 1e-3)
		return false;
	glm::dvec2 const direction{std::cos(deflection), std::sin(deflection)};
	glm::dvec2 const normal{-direction.y, direction.x};
	auto const end{glm::dvec2{in.x, in.y} + direction * out.x - normal * out.y};
	TangentOut = end.y / std::sin(deflection);
	TangentIn = end.x - TangentOut * std::cos(deflection);
	return TangentIn > 0.0 && TangentOut > 0.0;
}

bool collinear(design const &Design)
{
	auto const offset{plan_of(Design.end) - plan_of(Design.start)};
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
	glm::dvec2 position;
	glm::dvec2 direction;
	element_point(e, Result.shape, distance, position, direction);
	result.position = {position.x, elevation(Result, s), position.y};
	result.direction = glm::normalize(glm::dvec3{direction.x, grade(Result, s), direction.y});
	// a cant ramp of its own may lie on the straight before the curve, its cant turned the way the curve turns
	auto const cant{signed_cant(Result, e, distance, false)};
	result.cant = std::abs(cant);
	result.turn = e.turn != 0 ? e.turn : (cant > 0.0) - (cant < 0.0);
	return result;
}

namespace
{

// the plan from From to To in chords short enough to keep to the curves, the directions of its ends carried on far, so that a point
// past them is measured across the track
struct plan_line
{
	std::vector<glm::dvec2> points;
	std::vector<double> chainages;
};

plan_line sample_plan(result const &Result, double const From, double const To, double const Step)
{
	constexpr double kFar{5000.0};
	plan_line line;
	if (Result.elements.empty())
		return line;
	auto const from{std::clamp(From, 0.0, Result.length)};
	auto const to{std::clamp(To, from, Result.length)};
	auto const push = [&](glm::dvec2 const &Position, double const Chainage) {
		line.points.push_back(Position);
		line.chainages.push_back(Chainage);
	};
	for (auto const &e : Result.elements)
	{
		auto const low{std::max(from, e.chainage)};
		auto const high{std::min(to, e.chainage + e.length)};
		if (high < low)
			continue;
		auto const count{std::max(1, static_cast<int>(std::ceil((high - low) / Step)))};
		auto const at = [&](int const Index) { return low + (high - low) * Index / count; };
		if (e.kind != element_kind::spiral)
		{
			glm::dvec2 position;
			glm::dvec2 direction;
			for (int i = 0; i <= count; ++i)
			{
				element_point(e, Result.shape, at(i) - e.chainage, position, direction);
				push(position, at(i));
			}
			continue;
		}
		// spiral_point integrates from the start of the element for every point, here it goes on from the previous one
		auto const heading = [&](double const S) { return e.curvature_start * S + (e.curvature_end - e.curvature_start) * S * S / (2.0 * std::max(e.length, 1e-9)); };
		double x{0.0};
		double y{0.0};
		double s{0.0};
		auto const advance = [&](double const Distance) {
			auto const pieces{std::max(1, static_cast<int>(std::ceil((Distance - s) / 2.0)))};
			auto const h{(Distance - s) / pieces};
			for (int i = 0; i < pieces; ++i)
			{
				auto const a{heading(s + h * i)};
				auto const m{heading(s + h * (i + 0.5))};
				auto const b{heading(s + h * (i + 1))};
				x += h / 6.0 * (std::cos(a) + 4.0 * std::cos(m) + std::cos(b));
				y += h / 6.0 * (std::sin(a) + 4.0 * std::sin(m) + std::sin(b));
			}
			s = Distance;
		};
		for (int i = 0; i <= count; ++i)
		{
			advance(at(i) - e.chainage);
			push(e.origin + e.direction * x + e.normal * y, at(i));
		}
	}
	if (line.points.size() < 2)
		return line;
	auto const away = [](glm::dvec2 const &End, glm::dvec2 const &Inner) {
		auto const offset{End - Inner};
		return glm::length(offset) > 1e-9 ? offset / glm::length(offset) : glm::dvec2{0.0};
	};
	std::size_t first{1};
	while (first + 1 < line.points.size() && glm::distance(line.points[first], line.points.front()) < 1e-6)
		++first;
	auto last{line.points.size() - 2};
	while (last > 0 && glm::distance(line.points[last], line.points.back()) < 1e-6)
		--last;
	auto const before{away(line.points.front(), line.points[first])};
	auto const after{away(line.points.back(), line.points[last])};
	line.points.insert(line.points.begin(), line.points.front() + before * kFar);
	line.chainages.insert(line.chainages.begin(), line.chainages.front() - kFar);
	push(line.points.back() + after * kFar, line.chainages.back() + kFar);
	return line;
}

plan_projection project_on(plan_line const &Line, glm::dvec2 const &Point)
{
	plan_projection best;
	auto nearest{std::numeric_limits<double>::max()};
	for (std::size_t i = 0; i + 1 < Line.points.size(); ++i)
	{
		auto const segment{Line.points[i + 1] - Line.points[i]};
		auto const length2{glm::dot(segment, segment)};
		if (length2 < 1e-18)
			continue;
		auto const t{std::clamp(glm::dot(Point - Line.points[i], segment) / length2, 0.0, 1.0)};
		auto const offset{Point - (Line.points[i] + segment * t)};
		auto const distance2{glm::dot(offset, offset)};
		if (distance2 >= nearest)
			continue;
		nearest = distance2;
		best.chainage = Line.chainages[i] + (Line.chainages[i + 1] - Line.chainages[i]) * t;
		best.offset = std::copysign(std::sqrt(distance2), cross(segment, offset));
	}
	return best;
}

double squares(std::vector<double> const &Values)
{
	double sum{0.0};
	for (auto const value : Values)
		sum += value * value;
	return sum;
}

using residuals_of = std::function<bool(std::vector<double> const &, std::vector<double> &)>;

// Levenberg-Marquardt: the parameters of the least sum of the squares of the residuals; a step to where Residuals fails isn't taken
double least_squares(std::vector<double> &Parameters, residuals_of const &Residuals)
{
	constexpr double kDifference{1e-4};
	std::vector<double> residuals;
	if (false == Residuals(Parameters, residuals))
		return std::numeric_limits<double>::max();
	auto cost{squares(residuals)};
	auto const n{Parameters.size()};
	auto const m{residuals.size()};
	double damping{1e-3};
	std::vector<double> trial;
	for (int iteration = 0; iteration < 40 && cost > 1e-12; ++iteration)
	{
		std::vector<std::vector<double>> jacobian(n, std::vector<double>(m, 0.0));
		for (std::size_t j = 0; j < n; ++j)
		{
			auto parameters{Parameters};
			parameters[j] += kDifference;
			auto sign{1.0};
			if (false == Residuals(parameters, trial) || trial.size() != m)
			{
				parameters[j] = Parameters[j] - kDifference;
				sign = -1.0;
				if (false == Residuals(parameters, trial) || trial.size() != m)
					continue;
			}
			for (std::size_t i = 0; i < m; ++i)
				jacobian[j][i] = sign * (trial[i] - residuals[i]) / kDifference;
		}
		std::vector<double> normal(n * n, 0.0);
		std::vector<double> gradient(n, 0.0);
		for (std::size_t j = 0; j < n; ++j)
		{
			for (std::size_t i = 0; i < m; ++i)
				gradient[j] += jacobian[j][i] * residuals[i];
			for (std::size_t k = 0; k < n; ++k)
				for (std::size_t i = 0; i < m; ++i)
					normal[j * n + k] += jacobian[j][i] * jacobian[k][i];
		}
		auto better{false};
		auto settled{false};
		for (int attempt = 0; attempt < 12 && false == better; ++attempt)
		{
			auto system{normal};
			std::vector<double> step(n);
			for (std::size_t j = 0; j < n; ++j)
			{
				system[j * n + j] += damping * std::max(normal[j * n + j], 1e-9);
				step[j] = -gradient[j];
			}
			for (std::size_t c = 0; c < n; ++c)
			{
				auto pivot{c};
				for (std::size_t r = c + 1; r < n; ++r)
					if (std::abs(system[r * n + c]) > std::abs(system[pivot * n + c]))
						pivot = r;
				if (std::abs(system[pivot * n + c]) < 1e-300)
					break;
				for (std::size_t k = 0; k < n; ++k)
					std::swap(system[c * n + k], system[pivot * n + k]);
				std::swap(step[c], step[pivot]);
				for (std::size_t r = c + 1; r < n; ++r)
				{
					auto const factor{system[r * n + c] / system[c * n + c]};
					for (std::size_t k = c; k < n; ++k)
						system[r * n + k] -= factor * system[c * n + k];
					step[r] -= factor * step[c];
				}
			}
			for (std::size_t c = n; c-- > 0;)
			{
				for (std::size_t k = c + 1; k < n; ++k)
					step[c] -= system[c * n + k] * step[k];
				step[c] = std::abs(system[c * n + c]) > 1e-300 ? step[c] / system[c * n + c] : 0.0;
			}
			auto candidate{Parameters};
			for (std::size_t j = 0; j < n; ++j)
				candidate[j] += step[j];
			if (Residuals(candidate, trial) && trial.size() == m && squares(trial) < cost)
			{
				auto const lower{squares(trial)};
				settled = cost - lower < 1e-6 * cost;
				Parameters = candidate;
				residuals = trial;
				cost = lower;
				damping = std::max(1e-9, damping * 0.3);
				better = true;
			}
			else
			{
				damping *= 8.0;
			}
		}
		if (false == better || settled)
			break;
	}
	return cost;
}

bool compound_residuals(design const &Trial, std::vector<glm::dvec2> const &Points, std::vector<double> &Residuals);

constexpr double kTraceRadiusMin{15.0};
constexpr double kTraceRadiusMax{50000.0};
constexpr double kTraceRobust{0.25};
// m, how far the points clicked along a curve should be from the curve laid through them: a shape keeping to them no closer is
// given more radii
constexpr double kPointsWithin{0.3};
// m, how far they may be from it, clicked by hand far out: a row of bends making a curve no closer is cut in parts
constexpr double kPointsLoose{1.0};
// m, the shortest arc of a compound curve fitted through points: a shorter one is a kink the points made, no arc
constexpr double kShortestArc{10.0};

// the parameters: the logarithms of the radii, then the weights of the shares of all but the first radius, as logarithms too.
// Between: the transitions between the arcs those of the norms, else the ones of the base
vertex traced_shape(design const &Design, vertex const &Base, std::vector<double> const &Parameters, int const Radii, bool const Between = true)
{
	auto shape{Base};
	auto const radius = [&](int const Index) { return std::exp(std::clamp(Parameters[Index], std::log(kTraceRadiusMin), std::log(kTraceRadiusMax))); };
	shape.radius = radius(0);
	shape.share = 1.0;
	if (Radii == 1)
	{
		shape.compound = false;
		return shape;
	}
	std::vector<double> weights{1.0};
	for (int i = 1; i < Radii; ++i)
		weights.push_back(std::exp(std::clamp(Parameters[Radii + i - 1], -5.0, 5.0)));
	double total{0.0};
	for (auto const weight : weights)
		total += weight;
	shape.compound = true;
	shape.share = weights[0] / total;
	shape.arcs.clear();
	for (int i = 1; i < Radii; ++i)
	{
		compound_arc arc;
		arc.radius = radius(i);
		arc.transition = i - 1 < static_cast<int>(Base.arcs.size()) ? Base.arcs[i - 1].transition : 0.0;
		arc.cant = i - 1 < static_cast<int>(Base.arcs.size()) ? Base.arcs[i - 1].cant : -1.0;
		arc.share = weights[i] / total;
		shape.arcs.push_back(arc);
	}
	if (Between && Design.norms.checked)
		recommend_between(shape, Design.speed, Design.norms);
	return shape;
}

// the departures of the points from the curve of the vertex shaped by the parameters, as traced_shape takes them
residuals_of trace_residuals(design const &Design, std::size_t const Vertex, std::vector<glm::dvec2> const &Points, int const Count, bool const Between)
{
	return [&Design, &Points, Vertex, Count, Between](std::vector<double> const &Parameters, std::vector<double> &Residuals) {
		auto trial{Design};
		trial.norms.checked = false;
		trial.vertices[Vertex] = traced_shape(Design, Design.vertices[Vertex], Parameters, Count, Between);
		// a compound curve alone made straight away
		if (Count >= 2 && trial.vertices.size() == 1)
			return compound_residuals(trial, Points, Residuals);
		auto const result{compute(trial)};
		if (false == result.valid)
			return false;
		auto from{std::numeric_limits<double>::max()};
		auto to{std::numeric_limits<double>::lowest()};
		for (auto const &e : result.elements)
		{
			if (e.vertex != static_cast<int>(Vertex))
				continue;
			from = std::min(from, e.chainage);
			to = std::max(to, e.chainage + e.length);
		}
		if (from > to)
			return false;
		auto const line{sample_plan(result, from - 1.0, to + 1.0, std::clamp((to - from) / 300.0, 0.25, 2.0))};
		Residuals.clear();
		for (auto const &point : Points)
		{
			// a point clicked by mistake far off pulls the curve less than the square of its departure would
			auto const offset{project_on(line, point).offset};
			auto const departure{std::abs(offset)};
			Residuals.push_back(departure <= kTraceRobust ? offset : std::copysign(std::sqrt(2.0 * kTraceRobust * departure - kTraceRobust * kTraceRobust), offset));
		}
		return true;
	};
}

// the curve of the vertex given the shape, and how far the points are from it
trace_fit trace_of(design const &Design, std::size_t const Vertex, vertex const &Shape, std::vector<glm::dvec2> const &Points)
{
	trace_fit fit;
	auto trial{Design};
	trial.vertices[Vertex] = Shape;
	auto const result{compute(trial)};
	if (false == result.valid)
		return fit;
	fit.valid = true;
	fit.fitted = Shape;
	double sum{0.0};
	for (auto const &projection : project(result, Points))
	{
		sum += projection.offset * projection.offset;
		fit.worst = std::max(fit.worst, std::abs(projection.offset));
	}
	fit.rms = std::sqrt(sum / static_cast<double>(Points.size()));
	return fit;
}

} // namespace

std::vector<plan_projection> project(result const &Result, std::vector<glm::dvec2> const &Points)
{
	std::vector<plan_projection> projections;
	if (Result.elements.empty())
		return projections;
	auto const line{sample_plan(Result, 0.0, Result.length, std::max(0.5, Result.length / 4000.0))};
	for (auto const &point : Points)
		projections.push_back(project_on(line, point));
	return projections;
}

trace_fit fit_trace(design const &Design, std::size_t const Vertex, std::vector<glm::dvec2> const &Points, int const Radii)
{
	trace_fit best;
	if (Vertex >= Design.vertices.size() || Points.empty())
		return best;
	auto const &base{Design.vertices[Vertex]};
	auto const residuals = [&](int const Count) { return trace_residuals(Design, Vertex, Points, Count, true); };
	auto const cost_of = [](residuals_of const &Residuals, std::vector<double> const &Parameters) {
		std::vector<double> values;
		return Residuals(Parameters, values) ? squares(values) : std::numeric_limits<double>::max();
	};

	// one radius: a scan over the whole range, then the golden section around the best of it
	auto const one{residuals(1)};
	auto const low{std::log(kTraceRadiusMin)};
	auto const high{std::log(kTraceRadiusMax)};
	constexpr int kScan{60};
	auto const spacing{(high - low) / kScan};
	auto bestlog{std::log(std::max(kTraceRadiusMin, base.radius))};
	auto bestcost{cost_of(one, {bestlog})};
	for (int i = 0; i <= kScan; ++i)
		if (auto const cost{cost_of(one, {low + spacing * i})}; cost < bestcost)
		{
			bestcost = cost;
			bestlog = low + spacing * i;
		}
	// two points for each radius; a curve going on from the one of the adjoining track has its start, its curvature there given too, and
	// where one radius doesn't reach the end from there two are tried: a sharper one after the one it goes on from
	auto const going{Design.start_curved || Design.end_curved};
	auto const alone{bestcost < std::numeric_limits<double>::max()};
	auto const most{std::clamp(Radii > 0 ? Radii : 3, 1, std::max(going && false == alone ? 2 : 1, static_cast<int>(Points.size()) / 2))};
	if (false == alone && most < 2)
		return best;
	if (alone)
	{
		constexpr double kGolden{0.6180339887498949};
		auto a{bestlog - spacing};
		auto b{bestlog + spacing};
		auto c{b - (b - a) * kGolden};
		auto d{a + (b - a) * kGolden};
		auto fc{cost_of(one, {c})};
		auto fd{cost_of(one, {d})};
		for (int i = 0; i < 50; ++i)
		{
			if (fc < fd)
			{
				b = d;
				d = c;
				fd = fc;
				c = b - (b - a) * kGolden;
				fc = cost_of(one, {c});
			}
			else
			{
				a = c;
				c = d;
				fc = fd;
				d = a + (b - a) * kGolden;
				fd = cost_of(one, {d});
			}
		}
		if (auto const middle{(a + b) * 0.5}; cost_of(one, {middle}) < bestcost)
			bestlog = middle;
	}

	struct candidate
	{
		std::vector<double> parameters;
		double cost{std::numeric_limits<double>::max()};
	};
	std::array<candidate, 4> fits;
	fits[1] = {{bestlog}, cost_of(one, {bestlog})};
	auto const shift{std::log(1.3)};
	auto const share = [](double const Second, double const First) { return std::log(Second / First); };
	auto const rms = [&](int const Count) { return std::sqrt(fits[Count].cost / static_cast<double>(Points.size())); };
	// the next radius has to halve the departure about, a curve which keeps to the points already gets none
	constexpr double kClose{0.08};
	auto chosen{1};
	auto const wanted = [&](int const Count) { return Count <= most && (Radii > 0 || rms(chosen) > kClose); };
	auto const take = [&](int const Count) {
		if (fits[Count].cost == std::numeric_limits<double>::max())
			return;
		if (Radii > 0 || rms(Count) < 0.6 * rms(chosen))
			chosen = Count;
	};
	if (wanted(2))
	{
		auto const two{residuals(2)};
		// radii far apart too where the curve goes on from another one
		std::vector<double> shifts{shift};
		if (going)
			shifts.insert(shifts.end(), {std::log(3.0), std::log(8.0)});
		for (auto const apart : shifts)
			for (auto const fraction : {0.3, 0.5, 0.7})
				for (auto const sign : {-1.0, 1.0})
				{
					std::vector<double> parameters{bestlog + sign * apart, bestlog - sign * apart, share(1.0 - fraction, fraction)};
					if (auto const cost{least_squares(parameters, two)}; cost < fits[2].cost)
						fits[2] = {parameters, cost};
				}
		take(2);
	}
	if (wanted(3))
	{
		auto const three{residuals(3)};
		std::vector<std::vector<double>> starts;
		for (auto const sign : {-1.0, 1.0})
			starts.push_back({bestlog + sign * shift, bestlog - sign * shift, bestlog + sign * shift, 0.0, 0.0});
		if (false == fits[2].parameters.empty())
		{
			auto const &two{fits[2].parameters};
			auto const second{std::exp(two[2])};
			starts.push_back({two[0], two[0], two[1], 0.0, std::log(2.0 * second)});
			starts.push_back({two[0], two[1], two[1], std::log(0.5 * second), std::log(0.5 * second)});
		}
		for (auto parameters : starts)
			if (auto const cost{least_squares(parameters, three)}; cost < fits[3].cost)
				fits[3] = {parameters, cost};
		take(3);
	}

	auto shape{traced_shape(Design, base, fits[chosen].parameters, chosen)};
	shape.radius = std::max(kTraceRadiusMin, std::round(shape.radius));
	for (auto &arc : shape.arcs)
		arc.radius = std::max(kTraceRadiusMin, std::round(arc.radius));
	if (Design.norms.checked)
		recommend_between(shape, Design.speed, Design.norms);
	return trace_of(Design, Vertex, shape, Points);
}

trace_fit refit_radii(design const &Design, std::size_t const Vertex, std::vector<glm::dvec2> const &Points)
{
	if (Vertex >= Design.vertices.size() || Points.empty())
		return {};
	auto const &base{Design.vertices[Vertex]};
	auto const compound{base.compound && false == base.arcs.empty()};
	auto const count{compound ? static_cast<int>(base.arcs.size()) + 1 : 1};
	// started from the curve as it is: the radii, then the shares weighed against the one of the first arc
	std::vector<double> parameters{std::log(std::max(kTraceRadiusMin, base.radius))};
	if (compound)
	{
		for (auto const &arc : base.arcs)
			parameters.push_back(std::log(std::max(kTraceRadiusMin, arc.radius)));
		for (auto const &arc : base.arcs)
			parameters.push_back(std::log(std::max(1e-3, arc.share) / std::max(1e-3, base.share)));
	}
	least_squares(parameters, trace_residuals(Design, Vertex, Points, count, false));
	auto shape{traced_shape(Design, base, parameters, count, false)};
	shape.radius = std::max(kTraceRadiusMin, std::round(shape.radius));
	for (auto &arc : shape.arcs)
		arc.radius = std::max(kTraceRadiusMin, std::round(arc.radius));
	return trace_of(Design, Vertex, shape, Points);
}

std::vector<double> breaks_at(result const &Result, std::vector<glm::dvec2> const &Points, double const From, double const To)
{
	double constexpr apart{0.2};
	std::vector<double> breaks;
	auto previous{From};
	for (auto const &at : project(Result, Points))
	{
		if (at.chainage < previous + apart || at.chainage > To - apart)
			return {};
		breaks.push_back(previous = at.chainage);
	}
	return breaks;
}

std::vector<segment_data> pieces_at(result const &Result, design const &Design, std::size_t const Count, std::vector<glm::dvec2> const &Joints, std::vector<std::size_t> &Intervals)
{
	Intervals.clear();
	auto const breaks{breaks_at(Result, Joints, 0.0, Result.length)};
	if (breaks.empty() || breaks.size() != Joints.size())
		return pieces(Result, Design, Count);
	return pieces(Result, Design, Count, breaks, std::vector<std::size_t>(breaks.size() + 1, 1), &Intervals);
}

std::size_t minimum_pieces(result const &Result, design const &Design)
{
	std::size_t count{0};
	for (auto const &e : Result.elements)
		count += element_pieces(e, Design.shape, Design.transition_pieces, Design.arc_piece_angle);
	return count;
}

std::vector<segment_data> pieces(result const &Result, design const &Design, std::size_t const Count, std::vector<double> const &Breaks, std::vector<std::size_t> const &Counts,
                                 std::vector<std::size_t> *Intervals)
{
	struct span
	{
		std::size_t element;
		double from;
		double to;
		bool generic{false};
		double start{0.0};
		double end{0.0};
	};
	auto const extent = [&](span const &Span) { return Span.generic ? Span.end - Span.start : Span.to - Span.from; };
	auto const chainage_from = [&](span const &Span) { return Span.generic ? Span.start : Result.elements[Span.element].chainage + Span.from; };
	auto const chainage_to = [&](span const &Span) { return Span.generic ? Span.end : Result.elements[Span.element].chainage + Span.to; };
	std::vector<span> spans;
	std::optional<double> lead;
	for (std::size_t i = 0; i < Result.elements.size(); ++i)
	{
		auto const &e{Result.elements[i]};
		auto const count{element_pieces(e, Design.shape, Design.transition_pieces, Design.arc_piece_angle)};
		if (count == 0)
		{
			if (e.length <= 0.0)
				continue;
			if (spans.empty())
			{
				lead = lead.value_or(e.chainage);
				continue;
			}
			auto &previous{spans.back()};
			previous.start = chainage_from(previous);
			previous.end = e.chainage + e.length;
			previous.generic = true;
			continue;
		}
		for (std::size_t j = 0; j < count; ++j)
			spans.push_back({i, e.length * j / count, e.length * (j + 1) / count});
		if (lead.has_value())
		{
			auto &first{spans[spans.size() - count]};
			first.end = chainage_to(first);
			first.start = *lead;
			first.generic = true;
			lead.reset();
		}
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
	// splits the span at the chainage
	auto const split = [&](std::vector<span>::iterator Span, double const Chainage) {
		span second{*Span};
		if (Span->generic)
		{
			second.start = Chainage;
			Span->end = Chainage;
		}
		else
		{
			auto const local{Chainage - Result.elements[Span->element].chainage};
			second.from = local;
			Span->to = local;
		}
		spans.insert(std::next(Span), second);
	};
	double constexpr margin{0.01};
	// the ends of the cant ramps of their own, where the cant starts or stops changing along a straight, a transition or an arc
	for (auto const &ramp : Result.ramps)
		for (auto const chainage : {ramp.from, ramp.to})
			if (auto inside{std::find_if(spans.begin(), spans.end(), [&](span const &Span) { return chainage_from(Span) < chainage - margin && chainage_to(Span) > chainage + margin; })};
			    ramp.own && inside != spans.end())
				split(inside, chainage);
	for (auto const chainage : Breaks)
	{
		auto inside{std::find_if(spans.begin(), spans.end(), [&](span const &Span) { return chainage_from(Span) < chainage - margin && chainage_to(Span) > chainage + margin; })};
		if (inside != spans.end())
			split(inside, chainage);
	}
	auto const interval = [&](span const &Span) {
		return static_cast<std::size_t>(std::upper_bound(Breaks.begin(), Breaks.end(), (chainage_from(Span) + chainage_to(Span)) * 0.5) - Breaks.begin());
	};
	// no path of a few millimetres, as an arc of a compound curve squeezed to almost nothing or a joint by the end of an element
	// leaves: the simulation takes a path under 5 cm for one of no length. Such a stretch goes to the path next to it, the one
	// between the same joints if there's one
	double constexpr shortest_piece{0.1};
	for (std::size_t i = 0; i < spans.size() && spans.size() > 1;)
	{
		if (extent(spans[i]) >= shortest_piece)
		{
			++i;
			continue;
		}
		auto const own{interval(spans[i])};
		auto const backward{i > 0 && (i + 1 == spans.size() || interval(spans[i - 1]) == own || interval(spans[i + 1]) != own)};
		if (backward)
		{
			auto &previous{spans[i - 1]};
			previous.start = chainage_from(previous);
			previous.end = chainage_to(spans[i]);
			previous.generic = true;
		}
		else
		{
			auto &next{spans[i + 1]};
			next.end = chainage_to(next);
			next.start = chainage_from(spans[i]);
			next.generic = true;
		}
		spans.erase(spans.begin() + static_cast<std::ptrdiff_t>(i));
		if (i > 0)
			--i;
	}
	for (std::size_t k = 0; k < Counts.size(); ++k)
	{
		while (true)
		{
			auto const inside{static_cast<std::size_t>(std::count_if(spans.begin(), spans.end(), [&](span const &Span) { return interval(Span) == k; }))};
			if (inside == 0 || inside >= Counts[k])
				break;
			auto longest{spans.end()};
			for (auto candidate = spans.begin(); candidate != spans.end(); ++candidate)
				if (interval(*candidate) == k && (longest == spans.end() || extent(*candidate) > extent(*longest)))
					longest = candidate;
			split(longest, (chainage_from(*longest) + chainage_to(*longest)) * 0.5);
		}
	}
	while (false == spans.empty() && spans.size() < Count)
	{
		auto longest = std::max_element(spans.begin(), spans.end(), [&](span const &A, span const &B) { return extent(A) < extent(B); });
		split(longest, (chainage_from(*longest) + chainage_to(*longest)) * 0.5);
	}
	// no path longer than a track may be
	for (std::size_t i = 0; i < spans.size();)
	{
		auto const from{chainage_from(spans[i])};
		auto const length{chainage_to(spans[i]) - from};
		// a length not finite would be split for ever
		if (length <= geometry::longest_path || false == std::isfinite(length))
		{
			++i;
			continue;
		}
		auto const count{static_cast<int>(std::ceil(length / geometry::longest_path))};
		split(spans.begin() + static_cast<std::ptrdiff_t>(i), from + length / count);
	}
	if (Intervals != nullptr)
	{
		Intervals->clear();
		for (auto const &piece : spans)
			Intervals->push_back(interval(piece));
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
		glm::dvec2 p0;
		glm::dvec2 p3;
		glm::dvec2 d0;
		glm::dvec2 d3;
		element_point(e, Design.shape, piece.from, p0, d0);
		element_point(e, Design.shape, piece.to, p3, d3);
		glm::dvec2 p1;
		glm::dvec2 p2;
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

		// the cant signed with the turn, a ramp of its own on a straight too; where it changes all at once, the one of the piece itself
		auto const roll = [&](double const Cant) { return static_cast<float>(glm::degrees(std::asin(std::clamp(Cant / Design.norms.gauge, -1.0, 1.0)))); };

		segment_data path;
		path.points[segment_data::point::start] = {p0.x, y0, p0.y};
		path.points[segment_data::point::end] = {p3.x, y3, p3.y};
		path.points[segment_data::point::control1] = glm::dvec3{p1.x, y1, p1.y} - path.points[segment_data::point::start];
		path.points[segment_data::point::control2] = glm::dvec3{p2.x, y2, p2.y} - path.points[segment_data::point::end];
		path.rolls[0] = roll(signed_cant(Result, e, piece.from, false));
		path.rolls[1] = roll(signed_cant(Result, e, piece.to, true));
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

namespace
{

// the vertices where fit_between puts them, the start before them and the end after; empty where it can't
std::vector<glm::dvec2> polygon_of(design const &Design)
{
	if (glm::length(Design.start_direction) < 1e-9 || glm::length(Design.end_direction) < 1e-9)
		return {};
	auto const count{Design.vertices.size()};
	auto const start{plan_of(Design.start)};
	auto const end{plan_of(Design.end)};
	std::vector<glm::dvec2> polygon{start};
	if (count == 1)
	{
		glm::dvec2 corner;
		if (false == tangent_intersection(Design, corner))
			return {};
		polygon.push_back(corner);
	}
	else if (count >= 2)
	{
		polygon.push_back(start + glm::normalize(Design.start_direction) * Design.vertices.front().offset);
		for (std::size_t i = 1; i + 1 < count; ++i)
			polygon.push_back(Design.vertices[i].position);
		polygon.push_back(end - glm::normalize(Design.end_direction) * Design.vertices.back().offset);
	}
	polygon.push_back(end);
	return polygon;
}

// the angle the line turns by at the vertex, nought where it can't be told
double vertex_deflection(design const &Design, std::size_t const Vertex)
{
	auto const polygon{polygon_of(Design)};
	if (Vertex + 2 >= polygon.size())
		return 0.0;
	auto const in{polygon[Vertex + 1] - polygon[Vertex]};
	auto const out{polygon[Vertex + 2] - polygon[Vertex + 1]};
	if (glm::length(in) < 1e-9 || glm::length(out) < 1e-9)
		return 0.0;
	return std::atan2(std::abs(cross(in, out)), glm::dot(in, out));
}

// the cant the transitions of the length can ramp at the speed, in steps of 5 mm
double ramped_cant(double const Cant, double const Length, double const Speed, limits const &Norms)
{
	auto const cant{std::min({Cant, Length * 1000.0 / (Norms.ramp_factor * Speed), Norms.cant_rate * 3.6 * Length / Speed})};
	return cant < 5.0 ? 0.0 : std::floor(cant / 5.0) * 5.0;
}

// the transitions and the cant of the norms for the radii of the vertex, at the speed its sharpest arc allows
void recommended_curve(vertex &Vertex, double const Speed, limits const &Norms)
{
	auto sharpest{std::max(1.0, Vertex.radius)};
	if (Vertex.compound)
		for (auto const &arc : Vertex.arcs)
			sharpest = std::min(sharpest, std::max(1.0, arc.radius));
	auto const allowed{std::max(1.0, std::min(Speed, speed_limit(sharpest, Norms)))};
	auto const recommended{recommend(allowed, sharpest, Norms)};
	Vertex.transition_in = Vertex.transition_out = recommended.transition;
	Vertex.cant = ramped_cant(recommended.cant, recommended.transition, allowed, Norms);
	recommend_between(Vertex, Speed, Norms);
}

// a curve a bend can be eased with, and how far it reaches along the straights either side of it
struct corner_shape
{
	bool spirals{false};
	double radius{0.0};
	double transition{0.0};
	double cant{0.0};
	double reach{0.0};
};

// the bends eased one by one, each with the curve as large as fits between its neighbours
result ease_bends(design &Design, std::vector<char> const &Corners, corner_radii const &Radii)
{
	auto const count{Design.vertices.size()};
	auto const corner = [&](std::size_t const K) { return K < Corners.size() && Corners[K] != 0; };
	// the curves of the vertices left as they are reach along the straights as far as they do with the bends kept out of their way
	std::vector<std::array<double, 2>> kept(count, {0.0, 0.0});
	{
		auto bare{Design};
		for (std::size_t k = 0; k < count; ++k)
			if (corner(k))
				bare.vertices[k] = {bare.vertices[k].position, bare.vertices[k].offset, 1.0};
		for (auto const &curve : compute(bare).curves)
			if (curve.vertex >= 0 && curve.vertex < static_cast<int>(count))
				kept[curve.vertex] = {curve.tangent_in, curve.tangent_out};
	}
	auto const polygon{polygon_of(Design)};
	if (polygon.size() != count + 2)
		return compute(Design);
	auto const &norms{Design.norms};
	auto const speed{std::max(1.0, Design.speed)};
	auto const low{std::max(10.0, std::min(Radii.low, Radii.high))};
	auto const high{std::max(low, Radii.high)};

	// a bend at the start, or at the end, turning the way of the curve of the adjoining track goes on from it: its curve begins right
	// there, as large as the bend makes it, with the transitions and the cant of the norms between it and the adjoining curve and out of
	// it to the straight. Its size fixed so, the curves next to it share the straight with it as with a curve left as it is
	std::vector<char> fixed(count, 0);
	for (std::size_t k = 0; k < count; ++k)
	{
		if (false == corner(k) || (k > 0 && k + 1 < count))
			continue;
		auto const in{polygon[k + 1] - polygon[k]};
		auto const out{polygon[k + 2] - polygon[k + 1]};
		if (glm::length(in) < 1e-6 || glm::length(out) < 1e-6)
			continue;
		int const side{cross(in, out) > 0.0 ? 1 : -1};
		auto const from{k == 0 && Design.start_curved && Design.start_radius > 0.0 && Design.start_turn == side};
		auto const to{k + 1 == count && Design.end_curved && Design.end_radius > 0.0 && Design.end_turn == side && false == from};
		if (false == from && false == to)
			continue;
		// the bend alone between the end and the point after it, or before it, the straight there as long as the curve wants
		auto single{Design};
		single.vertices.assign(1, vertex{});
		if (from)
		{
			single.end = {polygon[k + 2].x, Design.end.y, polygon[k + 2].y};
			single.end_direction = glm::normalize(out);
			single.end_curved = false;
			single.end_reserve = 1e4;
		}
		else
		{
			single.start = {polygon[k].x, Design.start.y, polygon[k].y};
			single.start_direction = glm::normalize(in);
			single.start_curved = false;
			single.start_reserve = 1e4;
		}
		auto const adjoining{from ? 1.0 / Design.start_radius : 1.0 / Design.end_radius};
		auto const adjoiningcant{std::max(0.0, from ? Design.start_cant : Design.end_cant)};
		auto &shape{single.vertices.front()};
		shape.radius = std::max(1.0, Design.vertices[k].radius);
		std::optional<curve_report> made;
		// the transitions and the cant follow the radius, which the transitions move a little
		for (int pass = 0; pass < 6; ++pass)
		{
			auto const allowed{std::max(1.0, std::min(speed, speed_limit(shape.radius, norms)))};
			auto const recommended{recommend(allowed, shape.radius, norms)};
			auto const slower{std::max(1.0, std::min(allowed, speed_limit(1.0 / adjoining, norms)))};
			auto const along{recommended.transition};
			auto const cant{ramped_cant(recommended.cant, along, allowed, norms)};
			auto const between{between_curves(slower, adjoining, adjoiningcant, 1.0 / shape.radius, cant, norms)};
			(from ? shape.transition_in : shape.transition_out) = between;
			(from ? shape.transition_out : shape.transition_in) = along;
			shape.cant = cant;
			auto const laid{compute(single)};
			if (false == laid.valid || laid.curves.size() != 1)
			{
				made.reset();
				break;
			}
			auto const previous{shape.radius};
			made = laid.curves.front();
			shape.radius = made->radius;
			if (std::abs(shape.radius - previous) < 0.01)
				break;
		}
		if (false == made.has_value())
			continue;
		auto &vertex{Design.vertices[k]};
		vertex.spirals = false;
		vertex.radius = shape.radius;
		vertex.transition_in = shape.transition_in;
		vertex.transition_out = shape.transition_out;
		vertex.cant = shape.cant;
		vertex.reverse_turn = false;
		vertex.compound = false;
		vertex.share = 1.0;
		vertex.arcs.clear();
		fixed[k] = 1;
		kept[k] = {made->tangent_in, made->tangent_out};
	}

	// the shapes each bend can take, the best first
	std::vector<std::vector<corner_shape>> ladders(count);
	for (std::size_t k = 0; k < count; ++k)
	{
		if (false == corner(k) || fixed[k] != 0)
			continue;
		auto const in{polygon[k + 1] - polygon[k]};
		auto const out{polygon[k + 2] - polygon[k + 1]};
		if (glm::length(in) < 1e-6 || glm::length(out) < 1e-6)
			continue;
		auto const directionin{glm::normalize(in)};
		auto const directionout{glm::normalize(out)};
		auto const turn{cross(directionin, directionout)};
		auto const deflection{std::atan2(std::abs(turn), glm::dot(directionin, directionout))};
		int const side{turn > 0.0 ? 1 : -1};
		if (deflection < 1e-7 || deflection > kPi - 1e-3)
			continue;
		auto const reach = [&](double const Length, double const Radius) {
			double tangentin;
			double tangentout;
			curve_tangents(Design.shape, Length, Length, Radius, directionin, directionout, side, tangentin, tangentout);
			return std::max(tangentin, tangentout);
		};
		auto &ladder{ladders[k]};
		// two transitions meeting with no cant, long enough for the acceleration across to change gently at the speed: a bend too
		// slight for them at the highest radius takes a larger one
		auto const long_enough = [&](double const Radius) { return spirals_length(Design.shape, deflection, Radius) >= transition_length(speed, Radius, 0.0, norms) - 1e-6; };
		auto top{high};
		if (false == long_enough(top))
		{
			auto below{top};
			auto above{top * 2.0};
			while (false == long_enough(above) && above < 1e7)
			{
				below = above;
				above *= 2.0;
			}
			for (int i = 0; i < 40; ++i)
			{
				auto const middle{(below + above) * 0.5};
				(long_enough(middle) ? above : below) = middle;
			}
			top = std::ceil(above / 10.0) * 10.0;
		}
		constexpr int kSpirals{30};
		for (int i = 0; i <= kSpirals; ++i)
		{
			auto const radius{std::round(top * std::pow(low / top, static_cast<double>(i) / kSpirals))};
			if (false == long_enough(radius))
				break;
			auto const length{spirals_length(Design.shape, deflection, radius)};
			ladder.push_back({true, radius, length, 0.0, reach(length, radius)});
			if (top <= low)
				break;
		}
		// then arcs with the transitions and the cant of the norms, as large as fit, to any radius: the transitions shortened the way
		// fit_between does it, to leave the arc its least length
		auto const keep_arc{std::max(norms.arc_min, norms.arc_min_factor * speed)};
		constexpr int kArcs{60};
		for (int i = 0; i <= kArcs; ++i)
		{
			auto const radius{std::round(high * std::pow(10.0 / high, static_cast<double>(i) / kArcs))};
			auto const allowed{std::max(1.0, std::min(speed, speed_limit(radius, norms)))};
			auto const recommended{recommend(allowed, radius, norms)};
			auto const turn_of = [&](double const Length) { return 2.0 * transition_point(Design.shape, Length, radius, 1.0).angle; };
			auto const least{std::min(radius * deflection, keep_arc)};
			auto length{recommended.transition};
			if (length > 0.0 && radius * (deflection - turn_of(length)) < least - 1e-6)
			{
				double shorter{0.0};
				double longer{length};
				for (int j = 0; j < 50; ++j)
				{
					auto const middle{(shorter + longer) * 0.5};
					(radius * (deflection - turn_of(middle)) < least ? longer : shorter) = middle;
				}
				length = shorter < 1.0 ? 0.0 : shorter;
			}
			auto const cant{length > 0.0 ? ramped_cant(recommended.cant, length, allowed, norms) : 0.0};
			ladder.push_back({false, radius, length, cant, reach(length, radius)});
		}
	}

	// the straight between two curves is shared: each of them gets what it wants of it where they both fit, else the one wanting less
	// than a half gets it, the other the rest, or each a half. Then each bend takes the best shape the room its neighbours leave fits
	auto const between{norms.tangent_min_time * speed / 3.6};
	auto const leg = [&](std::size_t const Leg) {
		return glm::distance(polygon[Leg], polygon[Leg + 1]) + (Leg == 0 ? std::max(0.0, Design.start_reserve) : 0.0) + (Leg == count ? std::max(0.0, Design.end_reserve) : 0.0) -
		       (Leg > 0 && Leg < count ? between : 0.0);
	};
	std::vector<std::size_t> picks(count, 0);
	auto const reach_of = [&](std::size_t const K, bool const Out) {
		if (false == ladders[K].empty())
			return ladders[K][picks[K]].reach;
		return corner(K) && fixed[K] == 0 ? 0.0 : kept[K][Out ? 1 : 0];
	};
	auto const best_within = [&](std::size_t const K, double const Room) {
		auto const &ladder{ladders[K]};
		for (std::size_t i = 0; i < ladder.size(); ++i)
			if (ladder[i].reach <= Room + 1e-6)
				return i;
		return ladder.size() - 1;
	};
	std::vector<double> roomin(count, std::numeric_limits<double>::max());
	std::vector<double> roomout(count, std::numeric_limits<double>::max());
	for (std::size_t j = 0; j <= count; ++j)
	{
		auto const room{std::max(0.0, leg(j))};
		auto const before{j > 0 ? reach_of(j - 1, true) : 0.0};
		auto const after{j < count ? reach_of(j, false) : 0.0};
		auto give_before{room - after};
		auto give_after{room - before};
		if (before + after > room)
		{
			auto const half{room * 0.5};
			give_before = after <= half ? room - after : before <= half ? before : half;
			give_after = before <= half ? room - before : after <= half ? after : half;
		}
		if (j > 0)
			roomout[j - 1] = give_before;
		if (j < count)
			roomin[j] = give_after;
	}
	for (std::size_t k = 0; k < count; ++k)
		if (false == ladders[k].empty())
			picks[k] = best_within(k, std::min(roomin[k], roomout[k]));
	for (int pass = 0; pass < 3; ++pass)
		for (std::size_t k = 0; k < count; ++k)
		{
			if (ladders[k].empty())
				continue;
			auto const in{leg(k) - (k > 0 ? reach_of(k - 1, true) : 0.0)};
			auto const out{leg(k + 1) - (k + 1 < count ? reach_of(k + 1, false) : 0.0)};
			picks[k] = best_within(k, std::min(in, out));
		}

	for (std::size_t k = 0; k < count; ++k)
	{
		if (ladders[k].empty())
			continue;
		auto const &shape{ladders[k][picks[k]]};
		auto &vertex{Design.vertices[k]};
		vertex.spirals = shape.spirals;
		vertex.radius = shape.radius;
		vertex.transition_in = vertex.transition_out = shape.transition;
		vertex.cant = shape.cant;
		vertex.reverse_turn = false;
		vertex.compound = false;
		vertex.share = 1.0;
		vertex.arcs.clear();
	}
	return compute(Design);
}

// the curves the line lays at its vertices: the side each turns to, where each begins and ends
struct laid_curve
{
	int vertex{-1};
	int side{0};
	double from{0.0};
	double to{0.0};
};

std::vector<laid_curve> laid_curves(result const &Line)
{
	std::vector<laid_curve> curves;
	for (auto const &e : Line.elements)
	{
		if (e.vertex < 0 || e.kind == element_kind::straight)
			continue;
		if (curves.empty() || curves.back().vertex != e.vertex)
			curves.push_back({e.vertex, e.turn, e.chainage, e.chainage + e.length});
		else
			curves.back().to = e.chainage + e.length;
	}
	return curves;
}

// the shortest straight the line keeps between two curves turning the same way, as many metres as km/h of the speed, 30 m at least:
// two curves with a shorter one between them are one compound curve
double shortest_between(design const &Design)
{
	return std::max(30.0, Design.speed);
}

} // namespace

void settle_laid(std::vector<vertex> &Vertices, result const &Laid)
{
	for (auto const &curve : Laid.curves)
	{
		// a curve going on from an adjoining one is begun by that one's radius, not made smaller
		if (curve.vertex < 0 || curve.vertex >= static_cast<int>(Vertices.size()) || curve.curvature_in > 0.0 || curve.curvature_out > 0.0)
			continue;
		auto &vertex{Vertices[curve.vertex]};
		// two transitions meeting have their lengths from the radius they meet at
		auto const transitions{vertex.spirals || (curve.transition_in >= vertex.transition_in - 1e-6 && curve.transition_out >= vertex.transition_out - 1e-6)};
		if (curve.radius >= vertex.radius * (1.0 - 1e-6) && transitions)
			continue;
		auto const scale{curve.radius / std::max(1.0, vertex.radius)};
		vertex.radius = curve.radius;
		for (auto &arc : vertex.arcs)
		{
			arc.radius *= scale;
			arc.transition *= scale;
		}
		if (false == vertex.spirals)
		{
			vertex.transition_in = curve.transition_in;
			vertex.transition_out = curve.transition_out;
		}
	}
}

result ease_corners(design &Design, std::vector<char> &Corners, corner_radii const &Radii, stretch_fit const &Fit)
{
	// a point following a slight bend clicked closer than the curves leave a straight of the shortest length was clicked on a curve
	// seen on the photo: the bends of a row of such, turning the same way, are the points of a curve traced through them, between
	// the straights before and after them, one compound curve rather than short straights between curves, or several where one
	// doesn't keep to them
	auto line{ease_bends(Design, Corners, Radii)};
	auto const shortest{shortest_between(Design)};
	auto const bend = [&](int const K) { return K >= 0 && K < static_cast<int>(Corners.size()) && Corners[K] == 1; };
	auto pending{false};
	for (int pass = 0; pass < 32 && line.valid && false == pending; ++pass)
	{
		auto const curves{laid_curves(line)};
		auto const polygon{polygon_of(Design)};
		if (polygon.size() != Design.vertices.size() + 2)
			break;
		// the rows of bends: each as a pair of the first and the last curve, the longest first
		std::vector<std::pair<std::size_t, std::size_t>> rows;
		for (std::size_t i = 0; i < curves.size();)
		{
			auto j{i};
			while (j + 1 < curves.size() && bend(curves[i].vertex) && bend(curves[j + 1].vertex) && curves[j + 1].vertex == curves[j].vertex + 1 && curves[j + 1].side == curves[i].side &&
			       curves[j + 1].from - curves[j].to < shortest)
				++j;
			if (j > i)
				rows.emplace_back(i, j);
			i = j + 1;
		}
		auto merged{false};
		for (auto row{rows.begin()}; row != rows.end() && false == merged && false == pending; ++row)
		{
			// a row which doesn't make one curve is tried in parts, cut at its longest straight
			std::vector<std::pair<std::size_t, std::size_t>> parts{*row};
			while (false == parts.empty() && false == merged && false == pending)
			{
				auto const [first, last]{parts.back()};
				parts.pop_back();
				if (first == last)
					continue;
				auto const k{curves[first].vertex};
				auto const m{curves[last].vertex};
				design base{Design};
				base.vertices.clear();
				base.start = {polygon[k].x, Design.start.y, polygon[k].y};
				base.start_direction = glm::normalize(polygon[k + 1] - polygon[k]);
				base.end = {polygon[m + 2].x, Design.end.y, polygon[m + 2].y};
				base.end_direction = glm::normalize(polygon[m + 2] - polygon[m + 1]);
				base.start_reserve = 0.0;
				base.end_reserve = 0.0;
				// the curve of the row goes on from the curve of the adjoining track where the row begins at the start, or ends at the end
				base.start_curved = Design.start_curved && k == 0;
				base.end_curved = Design.end_curved && m + 1 == static_cast<int>(Design.vertices.size());
				std::vector<glm::dvec2> points(polygon.begin() + k + 1, polygon.begin() + m + 2);
				// two bends don't show the curve between them: the curve goes through the line their two curves laid, with no
				// straight between them, from where the first begins to where the second ends, the straights either side its room
				auto within{kPointsLoose};
				if (last == first + 1)
				{
					auto const from{curves[first].from};
					auto const to{curves[last].to};
					points.clear();
					auto const count{std::max(4, static_cast<int>(std::ceil((to - from) / 5.0)))};
					for (int i = 0; i <= count; ++i)
					{
						auto const station{evaluate(line, from + (to - from) * i / count)};
						points.push_back(plan_of(station.position));
					}
					auto const before{evaluate(line, from)};
					auto const after{evaluate(line, to)};
					base.start = before.position;
					base.start_direction = glm::normalize(plan_of(before.direction));
					base.end = after.position;
					base.end_direction = glm::normalize(plan_of(after.direction));
					base.start_reserve = from - (first > 0 ? curves[first - 1].to : 0.0);
					base.end_reserve = (last + 1 < curves.size() ? curves[last + 1].from : line.length) - to;
					base.start_curved = base.start_curved && from < 1e-6;
					base.end_curved = base.end_curved && to > line.length - 1e-6;
					within = kPointsWithin;
				}
				auto const fit{Fit ? Fit(base, points) : std::optional<trace_stretch>{fit_trace_stretch(base, points, 0, false)}};
				// still being fitted on the thread: the bends stay apart until it's done
				if (false == fit.has_value())
				{
					pending = true;
					break;
				}
				if (fit->valid && fit->vertices.size() == 1 && fit->worst <= within)
				{
					// the row of bends one vertex, kept as it is
					std::vector<vertex> vertices(Design.vertices.begin(), Design.vertices.begin() + k);
					vertices.push_back(fit->vertices.front());
					vertices.insert(vertices.end(), Design.vertices.begin() + m + 1, Design.vertices.end());
					std::vector<char> corners(Corners.begin(), Corners.begin() + k);
					corners.push_back(0);
					corners.insert(corners.end(), Corners.begin() + m + 1, Corners.end());
					for (std::size_t i = 0; i < vertices.size(); ++i)
						if (i < static_cast<std::size_t>(k) || i > static_cast<std::size_t>(k))
							vertices[i].position = polygon[(i < static_cast<std::size_t>(k) ? i : i + (m - k)) + 1];
					if (vertices.size() >= 2)
					{
						vertices.front().offset = glm::dot(vertices.front().position - plan_of(Design.start), glm::normalize(Design.start_direction));
						vertices.back().offset = glm::dot(plan_of(Design.end) - vertices.back().position, glm::normalize(Design.end_direction));
					}
					Design.vertices = std::move(vertices);
					Corners = std::move(corners);
					merged = true;
					break;
				}
				if (last - first < 2)
					continue;
				auto cut{first};
				for (auto i{first}; i < last; ++i)
					if (curves[i + 1].from - curves[i].to > curves[cut + 1].from - curves[cut].to)
						cut = i;
				parts.emplace_back(first, cut);
				parts.emplace_back(cut + 1, last);
			}
		}
		if (false == merged || pending)
			break;
		line = ease_bends(Design, Corners, Radii);
	}
	if (line.valid)
		settle_laid(Design.vertices, line);
	return line;
}

namespace
{

// a curve a single radius follows the points about as closely as this needs no more of them
constexpr double kStretchClose{0.08};

// the stretch as the design lays it, how far the points are from it
std::optional<trace_stretch> stretch_of(design const &Design, std::vector<glm::dvec2> const &Points)
{
	auto const laid{compute(Design)};
	if (false == laid.valid)
		return std::nullopt;
	trace_stretch stretch;
	stretch.valid = true;
	stretch.vertices = Design.vertices;
	for (std::size_t k = 0; k < stretch.vertices.size() && k < laid.vertices.size(); ++k)
		stretch.vertices[k].position = laid.vertices[k];
	// a curve going on from an adjoining one with the radii it was laid with, which begin it at the end
	for (auto const &curve : laid.curves)
		if ((curve.curvature_in > 0.0 || curve.curvature_out > 0.0) && curve.vertex >= 0 && curve.vertex < static_cast<int>(stretch.vertices.size()))
		{
			auto &vertex{stretch.vertices[curve.vertex]};
			auto const scale{curve.radius / std::max(1.0, vertex.radius)};
			vertex.radius = curve.radius;
			for (auto &arc : vertex.arcs)
				arc.radius *= scale;
		}
	settle_laid(stretch.vertices, laid);
	double sum{0.0};
	for (auto const &projection : project(laid, Points))
	{
		stretch.offsets.push_back(projection.offset);
		sum += projection.offset * projection.offset;
		stretch.worst = std::max(stretch.worst, std::abs(projection.offset));
	}
	stretch.rms = Points.empty() ? 0.0 : std::sqrt(sum / static_cast<double>(Points.size()));
	return stretch;
}

// how far the points are from the line laid, a point clicked by mistake far off pulling it less than the square of its departure would
bool laid_residuals(result const &Laid, std::vector<glm::dvec2> const &Points, std::vector<double> &Residuals)
{
	if (false == Laid.valid)
		return false;
	auto const line{sample_plan(Laid, 0.0, Laid.length, std::clamp(Laid.length / 300.0, 0.5, 4.0))};
	Residuals.clear();
	for (auto const &point : Points)
	{
		auto const offset{project_on(line, point).offset};
		auto const departure{std::abs(offset)};
		Residuals.push_back(departure <= kTraceRobust ? offset : std::copysign(std::sqrt(2.0 * kTraceRobust * departure - kTraceRobust * kTraceRobust), offset));
	}
	return true;
}

// the same for the line the design lays
bool stretch_residuals(design Trial, std::vector<glm::dvec2> const &Points, std::vector<double> &Residuals)
{
	Trial.norms.checked = false;
	return laid_residuals(compute(Trial), Points, Residuals);
}

// the same for a design of one compound curve, its shape made straight away as fit_between makes it, many times quicker than the whole
// design laid: the fits of compound curves try it some thousand times
bool compound_residuals(design const &Trial, std::vector<glm::dvec2> const &Points, std::vector<double> &Residuals)
{
	// a curve going on from an adjoining one is placed by its end, not by the vertex: laid as a whole
	if (Trial.vertices.size() != 1 || false == Trial.vertices.front().compound || Trial.vertices.front().arcs.empty() || Trial.start_curved || Trial.end_curved)
		return stretch_residuals(Trial, Points, Residuals);
	glm::dvec2 corner;
	if (false == tangent_intersection(Trial, corner))
		return false;
	auto const start{plan_of(Trial.start)};
	auto const end{plan_of(Trial.end)};
	auto const in{glm::normalize(Trial.start_direction)};
	auto const out{glm::normalize(Trial.end_direction)};
	auto const turn{cross(in, out)};
	auto const deflection{std::atan2(std::abs(turn), glm::dot(in, out))};
	if (deflection < 1e-7 || deflection > kPi - 1e-3)
		return false;
	int const side{turn > 0.0 ? 1 : -1};
	auto const &vertex{Trial.vertices.front()};
	std::vector<compound_arc> arcs{{std::max(1.0, vertex.radius), 0.0, vertex.share}};
	for (auto const &arc : vertex.arcs)
		arcs.push_back({std::max(1.0, arc.radius), std::max(0.0, arc.transition), arc.share});
	auto const shape{build_compound(deflection, arcs, std::max(0.0, vertex.transition_in), std::max(0.0, vertex.transition_out))};
	auto const normalin{perpendicular(in) * static_cast<double>(side)};
	auto const offset{in * shape.end.x + normalin * shape.end.y};
	auto const along{cross(out, offset) / cross(out, in)};
	auto const after{glm::dot(offset - in * along, out)};
	if (along <= 0.0 || after <= 0.0)
		return false;
	// a curve too large for the straights is made smaller as a whole, its radii and its transitions alike, as fit_between does it
	auto const scale{std::min({1.0, (glm::distance(corner, start) + std::max(0.0, Trial.start_reserve)) / along, (glm::distance(end, corner) + std::max(0.0, Trial.end_reserve)) / after})};
	// the curve in chords of 4 m at most, the straights either side carried on far
	constexpr double kFar{5000.0};
	auto const begin{corner - in * along * scale};
	auto origin{glm::dvec2{0.0}};
	auto heading{in};
	std::vector<glm::dvec2> line{begin - in * kFar, begin};
	for (auto const &segment : shape.segments)
	{
		auto const normal{perpendicular(heading) * static_cast<double>(side)};
		auto const steps{std::max(1, static_cast<int>(std::ceil(segment[2] * scale / 4.0)))};
		local_point local;
		for (int i = 1; i <= steps; ++i)
		{
			local = spiral_point(segment[0], segment[1], segment[2], segment[2] * i / steps);
			line.push_back(begin + (origin + heading * local.x + normal * local.y) * scale);
		}
		origin += heading * local.x + normal * local.y;
		heading = turned(heading, side * local.angle);
	}
	line.push_back(line.back() + out * kFar);
	plan_line plan;
	plan.points = std::move(line);
	plan.chainages.assign(plan.points.size(), 0.0);
	Residuals.clear();
	for (auto const &point : Points)
	{
		auto const offset{project_on(plan, point).offset};
		auto const departure{std::abs(offset)};
		Residuals.push_back(departure <= kTraceRobust ? offset : std::copysign(std::sqrt(2.0 * kTraceRobust * departure - kTraceRobust * kTraceRobust), offset));
	}
	return true;
}

// the longest transition a fit gives an arc of the radius at the speed: half as long again as the norms ask for, 30 m at least; one
// longer only keeps to points clicked a little off better, not to the track
double longest_transition(double const Speed, double const Radius, limits const &Norms)
{
	auto const allowed{std::max(1.0, std::min(std::max(1.0, Speed), speed_limit(Radius, Norms)))};
	return std::max(30.0, 1.5 * recommend(allowed, Radius, Norms).transition);
}

// a curve of one radius with transitions of the length: the cant of the norms for it, as much as the transitions ramp
void plain_curve(vertex &Vertex, double const Radius, double const Transition, double const Speed, limits const &Norms)
{
	Vertex.radius = Radius;
	Vertex.compound = false;
	Vertex.share = 1.0;
	Vertex.arcs.clear();
	Vertex.spirals = false;
	Vertex.transition_in = Vertex.transition_out = Transition;
	auto const allowed{std::max(1.0, std::min(Speed, speed_limit(Radius, Norms)))};
	Vertex.cant = Transition > 0.0 ? ramped_cant(recommend(allowed, Radius, Norms).cant, Transition, allowed, Norms) : 0.0;
}

// the cant of the norms for a curve going on from the curve of an adjoining track, or into it, as much as its transitions ramp: the one
// to the straight from 0, the one between it and the adjoining curve from the cant of that one, which goes on into the arc with none
double curved_cant(double const Radius, double const Between, double const Out, double const Adjoining, double const Speed, limits const &Norms)
{
	if (Between < 1.0)
		return Adjoining;
	auto const allowed{std::max(1.0, std::min(Speed, speed_limit(Radius, Norms)))};
	auto const cant{ramped_cant(recommend(allowed, Radius, Norms).cant, Out, allowed, Norms)};
	auto const reach{std::min(Between * 1000.0 / (Norms.ramp_factor * allowed), Norms.cant_rate * 3.6 * Between / allowed)};
	if (std::abs(cant - Adjoining) <= reach)
		return cant;
	auto const ramped{cant > Adjoining ? Adjoining + std::floor(reach / 5.0) * 5.0 : Adjoining - std::floor(reach / 5.0) * 5.0};
	return std::max(0.0, ramped);
}

// the transitions of the curve of the vertex going on from the curve of the adjoining track at the start, or into the one at the end,
// following the points best: its radius the one which begins it there, its cant that of the norms as far as the transitions ramp it
void fit_curved(design &Design, std::size_t const Vertex, std::vector<glm::dvec2> const &Points, bool const Fromstart)
{
	auto const speed{std::max(1.0, Design.speed)};
	auto const fromstart{Fromstart};
	auto const adjoining{std::max(kTraceRadiusMin, fromstart ? Design.start_radius : Design.end_radius)};
	auto const adjoiningcant{std::max(0.0, fromstart ? Design.start_cant : Design.end_cant)};
	auto const longest{longest_transition(speed, adjoining, Design.norms)};
	// the radius is sought from the one laid last, a step of the fit moving it little
	auto radius{adjoining};
	// the square roots of the lengths of the transitions, between the curves and to the straight, a 30th of them: the step the fit takes
	// its derivatives with moving the curve more than the chords the points are measured against do; one parameter: none between them,
	// the curve going on at the radius of the adjoining one
	constexpr double kScale{30.0};
	auto const length_of = [](double const Parameter) { return kScale * kScale * Parameter * Parameter; };
	auto const parameter_of = [](double const Length) { return std::sqrt(Length) / kScale; };
	auto const shaped = [&](std::vector<double> const &Parameters, design &Trial) {
		Trial = Design;
		auto &curve{Trial.vertices[Vertex]};
		curve = {curve.position, curve.offset, radius};
		auto const between{Parameters.size() > 1 ? std::min(length_of(Parameters[0]), longest) : 0.0};
		auto const out{std::min(length_of(Parameters.back()), longest)};
		curve.transition_in = fromstart ? between : out;
		curve.transition_out = fromstart ? out : between;
		curve.cant = 0.0;
	};
	auto const laid_with = [&](result const &Laid) {
		for (auto const &curve : Laid.curves)
			if (curve.vertex == static_cast<int>(Vertex))
				radius = curve.radius;
	};
	residuals_of const residuals{[&](std::vector<double> const &Parameters, std::vector<double> &Residuals) {
		design trial;
		shaped(Parameters, trial);
		trial.norms.checked = false;
		auto const laid{compute(trial)};
		laid_with(laid);
		return laid_residuals(laid, Points, Residuals);
	}};
	// the transition between the curves short or none, as where the curve goes on at the radius of the adjoining one, or as long as the
	// one to the straight; this one as long as the norms ask for at the radius of the adjoining curve. The square roots of the lengths,
	// a length of nought holding the step at nought
	auto const allowed{std::max(1.0, std::min(speed, speed_limit(adjoining, Design.norms)))};
	auto const out{std::clamp(recommend(allowed, adjoining, Design.norms).transition, 10.0, 0.9 * longest)};
	std::vector<double> best;
	auto lowest{std::numeric_limits<double>::max()};
	for (auto const between : {2.0, 0.5 * out})
	{
		radius = adjoining;
		std::vector<double> parameters{parameter_of(between), parameter_of(out)};
		if (auto const cost{least_squares(parameters, residuals)}; cost < lowest)
		{
			lowest = cost;
			best = parameters;
		}
	}
	// the curve going on at the radius of the adjoining one, with no transition between them, where it keeps to the points about as well:
	// a short transition the fit leaves only follows points clicked a little off
	{
		radius = adjoining;
		std::vector<double> parameters{parameter_of(out)};
		auto const cost{least_squares(parameters, residuals)};
		auto const points{static_cast<double>(std::max<std::size_t>(1, Points.size()))};
		// the radius it begins with the one of the adjoining curve, within 5 %, else the curvature would jump between them
		design trial;
		shaped(parameters, trial);
		trial.norms.checked = false;
		laid_with(compute(trial));
		auto const along{std::abs(radius - adjoining) <= 0.05 * adjoining};
		if (along && cost < std::numeric_limits<double>::max() && (best.empty() || std::sqrt(cost / points) <= std::sqrt(lowest / points) + 0.01))
		{
			lowest = cost;
			best = parameters;
		}
	}
	// a curve of another radius than the adjoining one, from too few points to show the transition between them: one as long as the
	// norms ask for at the speed the sharper of them allows, rather than the curvature jumping at the joint
	if (best.size() == 2 && Points.size() <= 3)
		for (int pass = 0; pass < 2; ++pass)
		{
			design trial;
			shaped(best, trial);
			trial.norms.checked = false;
			laid_with(compute(trial));
			auto const allowed{std::max(1.0, std::min(speed, speed_limit(std::min(radius, adjoining), Design.norms)))};
			auto const needed{std::min(between_curves(allowed, 1.0 / adjoining, adjoiningcant, 1.0 / radius, 0.0, Design.norms), 0.9 * longest)};
			if (length_of(best[0]) >= needed - 0.5)
				break;
			auto const between{parameter_of(needed)};
			residuals_of const rest{[&](std::vector<double> const &Parameters, std::vector<double> &Residuals) { return residuals({between, Parameters[0]}, Residuals); }};
			std::vector<double> out{best[1]};
			if (least_squares(out, rest) == std::numeric_limits<double>::max())
				break;
			best = {between, out[0]};
		}
	if (best.empty())
		return;
	design fitted;
	shaped(best, fitted);
	auto &curve{fitted.vertices[Vertex]};
	curve.transition_in = std::round(curve.transition_in);
	curve.transition_out = std::round(curve.transition_out);
	auto const laid{compute(fitted)};
	if (false == laid.valid)
		return;
	laid_with(laid);
	curve.radius = radius;
	curve.cant = fromstart ? curved_cant(radius, curve.transition_in, curve.transition_out, adjoiningcant, speed, fitted.norms) : curved_cant(radius, curve.transition_out, curve.transition_in, adjoiningcant, speed, fitted.norms);
	Design = fitted;
}

// the radius and the length of the transitions of the curve of the vertex following the points best, the rest of the design kept: the
// transitions of a line on the photo are what they are, the norms may have asked for others
void fit_curve(design &Design, std::size_t const Vertex, std::vector<glm::dvec2> const &Points)
{
	auto const speed{std::max(1.0, Design.speed)};
	// the transitions together turn no more than the curve: longer ones would only be cut to it
	auto const deflection{vertex_deflection(Design, Vertex)};
	// the transition into the curve and the one out of it each as long as the points show it, where there are points enough for both: a
	// curve begun short after a free end and running out long into the straight after it is one curve, not a sharp arc with a flat
	// one after it
	auto const apart{Points.size() >= 4};
	auto const shaped = [&](std::vector<double> const &Parameters, design &Trial) {
		Trial = Design;
		auto const radius{std::exp(std::clamp(Parameters[0], std::log(kTraceRadiusMin), std::log(kTraceRadiusMax)))};
		auto const longest{longest_transition(speed, radius, Trial.norms)};
		auto in{std::min(Parameters[1] * Parameters[1], longest)};
		auto out{std::min(Parameters.back() * Parameters.back(), longest)};
		if (auto const room{2.0 * radius * deflection}; in + out > room)
		{
			auto const scale{room / (in + out)};
			in *= scale;
			out *= scale;
		}
		auto &curve{Trial.vertices[Vertex]};
		plain_curve(curve, radius, std::min(in, out), speed, Trial.norms);
		curve.transition_in = in;
		curve.transition_out = out;
	};
	residuals_of const residuals{[&](std::vector<double> const &Parameters, std::vector<double> &Residuals) {
		design trial;
		shaped(Parameters, trial);
		return stretch_residuals(trial, Points, Residuals);
	}};
	// from the transitions it has and from those of the norms; the square root of the length, as a length of nought would hold the step
	// at nought
	auto const radius{std::max(kTraceRadiusMin, Design.vertices[Vertex].radius)};
	auto const allowed{std::max(1.0, std::min(speed, speed_limit(radius, Design.norms)))};
	// started below the longest transition: above it the length is held, the fit no longer moving it
	auto const started = [&](double const Length) { return std::sqrt(std::max(std::min(Length, 0.9 * std::min(radius * deflection, longest_transition(speed, radius, Design.norms))), 1.0)); };
	auto const norm{std::max(recommend(allowed, radius, Design.norms).transition, 10.0)};
	auto const fitted_from = [&](std::vector<std::vector<double>> const &Starts, std::vector<double> &Best) {
		auto lowest{std::numeric_limits<double>::max()};
		for (auto parameters : Starts)
			if (auto const cost{least_squares(parameters, residuals)}; cost < lowest)
			{
				lowest = cost;
				Best = parameters;
			}
		return lowest;
	};
	std::vector<double> best;
	auto const alike{fitted_from({{std::log(radius), started(std::max(Design.vertices[Vertex].transition_in, 10.0))}, {std::log(radius), started(norm)}}, best)};
	// the two apart, from the ones alike and from one short and the other long: taken where they keep to the points clearly closer, not
	// only to points clicked a little off
	constexpr double kApart{0.8};
	if (apart && false == best.empty())
	{
		std::vector<double> two;
		if (fitted_from({{best[0], best[1], best[1]}, {std::log(radius), started(10.0), started(norm)}, {std::log(radius), started(norm), started(10.0)}}, two) < kApart * alike)
			best = two;
	}
	if (best.empty())
		return;
	design fitted;
	shaped(best, fitted);
	auto &curve{fitted.vertices[Vertex]};
	auto const in{std::round(curve.transition_in)};
	auto const out{std::round(curve.transition_out)};
	plain_curve(curve, std::max(kTraceRadiusMin, std::round(curve.radius)), std::min(in, out), speed, fitted.norms);
	curve.transition_in = in;
	curve.transition_out = out;
	Design = fitted;
}

// the radii and the shares of the compound curve of the vertex together with the length of its transitions, following the points
// best, from the shape it has; Between: the transitions of the norms between the arcs, else the arcs meet with none
void fit_compound(design &Design, std::size_t const Vertex, std::vector<glm::dvec2> const &Points, bool const Between = true)
{
	auto const base{Design.vertices[Vertex]};
	if (false == base.compound || base.arcs.empty())
		return;
	auto const speed{std::max(1.0, Design.speed)};
	int const radii{static_cast<int>(base.arcs.size()) + 1};
	// the transitions into the first arc and out of the last one turn no more than those arcs
	auto const deflection{vertex_deflection(Design, Vertex)};
	auto const reach = [&](vertex const &Curve) {
		auto total{std::max(0.0, Curve.share)};
		for (auto const &arc : Curve.arcs)
			total += std::max(0.0, arc.share);
		auto const first{total > 0.0 ? std::max(0.0, Curve.share) / total : 1.0};
		auto const last{total > 0.0 && false == Curve.arcs.empty() ? std::max(0.0, Curve.arcs.back().share) / total : 1.0};
		return std::min(Curve.radius * first, Curve.arcs.empty() ? Curve.radius : Curve.arcs.back().radius * last) * deflection;
	};
	// as traced_shape takes them, the square root of the length of the transitions last
	std::vector<double> start{std::log(std::max(kTraceRadiusMin, base.radius))};
	for (auto const &arc : base.arcs)
		start.push_back(std::log(std::max(kTraceRadiusMin, arc.radius)));
	for (auto const &arc : base.arcs)
		start.push_back(std::log(std::max(1e-3, arc.share) / std::max(1e-3, base.share)));
	auto const shaped = [&](std::vector<double> const &Parameters, design &Trial) {
		Trial = Design;
		auto &curve{Trial.vertices[Vertex]};
		auto shape{Design};
		shape.norms.checked = Between;
		curve = traced_shape(shape, base, Parameters, radii);
		if (false == Between)
			for (auto &arc : curve.arcs)
				arc.transition = 0.0;
		curve.transition_in = curve.transition_out = std::min({Parameters.back() * Parameters.back(), reach(curve), longest_transition(speed, curve.radius, Trial.norms), longest_transition(speed, curve.arcs.empty() ? curve.radius : curve.arcs.back().radius, Trial.norms)});
	};
	residuals_of const residuals{[&](std::vector<double> const &Parameters, std::vector<double> &Residuals) {
		design trial;
		shaped(Parameters, trial);
		return compound_residuals(trial, Points, Residuals);
	}};
	std::vector<double> best;
	auto lowest{std::numeric_limits<double>::max()};
	// from the transitions it has, from half of them and from those of the norms for its sharpest arc
	auto sharpest{base.radius};
	for (auto const &arc : base.arcs)
		sharpest = std::min(sharpest, arc.radius);
	auto const allowed{std::max(1.0, std::min(speed, speed_limit(sharpest, Design.norms)))};
	for (auto const length : {base.transition_in, 0.5 * base.transition_in, recommend(allowed, sharpest, Design.norms).transition})
	{
		auto parameters{start};
		parameters.push_back(std::sqrt(std::max(std::min(length, 0.9 * std::min({reach(base), longest_transition(speed, base.radius, Design.norms), longest_transition(speed, base.arcs.back().radius, Design.norms)})), 10.0)));
		if (auto const cost{least_squares(parameters, residuals)}; cost < lowest)
		{
			lowest = cost;
			best = parameters;
		}
	}
	if (best.empty())
		return;
	design fitted;
	shaped(best, fitted);
	auto &curve{fitted.vertices[Vertex]};
	curve.radius = std::max(kTraceRadiusMin, std::round(curve.radius));
	auto smallest{curve.radius};
	for (auto &arc : curve.arcs)
		smallest = std::min(smallest, arc.radius = std::max(kTraceRadiusMin, std::round(arc.radius)));
	curve.transition_in = curve.transition_out = std::round(curve.transition_in);
	auto const limit{std::max(1.0, std::min(speed, speed_limit(smallest, fitted.norms)))};
	curve.cant = curve.transition_in > 0.0 ? ramped_cant(recommend(limit, smallest, fitted.norms).cant, curve.transition_in, limit, fitted.norms) : 0.0;
	if (Between)
		recommend_between(curve, speed, fitted.norms);
	Design = fitted;
}

// a compound curve of an arc this many times flatter than its sharpest one follows a straight the points show between two curves turning
// the same way, or off the straight at its end: curves with a straight between them are taken over it where they keep to the points
// about as closely
constexpr double kCompoundSpread{4.0};

bool flattened(vertex const &Curve)
{
	if (false == Curve.compound || Curve.arcs.empty())
		return false;
	auto sharpest{Curve.radius};
	auto flattest{Curve.radius};
	for (auto const &arc : Curve.arcs)
	{
		sharpest = std::min(sharpest, arc.radius);
		flattest = std::max(flattest, arc.radius);
	}
	return flattest > kCompoundSpread * sharpest;
}

// the one curve of the design as a compound curve of more arcs than it has, up to Most, as few as keep within kPointsWithin of the
// points, all along with the transitions of the norms between the arcs: each count started from the curve fitted with an arc less,
// its longest arc cut in two, and from the curvature the points show, the better fit taken
std::optional<trace_stretch> fit_arcs(design const &Single, std::vector<glm::dvec2> const &Points, int const Most)
{
	auto const laid{compute(Single)};
	if (false == laid.valid || Points.size() < 3)
		return std::nullopt;
	auto from{laid.length};
	auto to{0.0};
	for (auto const &e : laid.elements)
		if (e.vertex == 0)
		{
			from = std::min(from, e.chainage);
			to = std::max(to, e.chainage + e.length);
		}
	if (from >= to)
		return std::nullopt;
	// the curvature at each point of the circle through it and the points either side, at the chainage of the point on the curve
	auto const projections{project(laid, Points)};
	std::vector<std::pair<double, double>> curvatures;
	for (std::size_t i = 1; i + 1 < Points.size(); ++i)
	{
		auto const a{glm::distance(Points[i - 1], Points[i])};
		auto const b{glm::distance(Points[i], Points[i + 1])};
		auto const c{glm::distance(Points[i - 1], Points[i + 1])};
		if (a * b * c < 1e-9)
			continue;
		curvatures.emplace_back(projections[i].chainage, 2.0 * std::abs(cross(Points[i] - Points[i - 1], Points[i + 1] - Points[i - 1])) / (a * b * c));
	}
	auto const &single{Single.vertices.front()};
	// the arcs of a curve as a list, the first one too, and back
	auto const arcs_of = [](vertex const &Curve) {
		std::vector<compound_arc> arcs{{Curve.radius, 0.0, Curve.compound ? Curve.share : 1.0}};
		if (Curve.compound)
			arcs.insert(arcs.end(), Curve.arcs.begin(), Curve.arcs.end());
		return arcs;
	};
	auto const shaped_of = [&](std::vector<compound_arc> const &Arcs) {
		auto shaped{Single};
		auto &curve{shaped.vertices.front()};
		curve.compound = true;
		curve.spirals = false;
		curve.radius = Arcs.front().radius;
		curve.share = Arcs.front().share;
		curve.arcs.assign(Arcs.begin() + 1, Arcs.end());
		return shaped;
	};
	// a curve with the transitions of the norms between its arcs fitted from the shape given, all along with them; none where an
	// arc came out too short to be one, a kink the points made
	auto const fitted = [&](design Shaped) -> std::optional<std::pair<design, trace_stretch>> {
		recommend_between(Shaped.vertices.front(), Single.speed, Single.norms);
		fit_compound(Shaped, 0, Points);
		auto const laid{compute(Shaped)};
		if (std::any_of(laid.elements.begin(), laid.elements.end(), [](element const &Element) {
			    return Element.vertex == 0 && Element.curvature_start == Element.curvature_end && Element.curvature_start > 0.0 && Element.length < kShortestArc;
		    }))
			return std::nullopt;
		auto const stretch{stretch_of(Shaped, Points)};
		if (false == stretch.has_value())
			return std::nullopt;
		return std::make_pair(Shaped, *stretch);
	};
	std::vector<trace_stretch> counts; // the fit of each count of arcs, the fewest first
	auto previous{Single};
	for (auto count{arcs_of(single).size() + 1}; count <= static_cast<std::size_t>(Most); ++count)
	{
		// from the curve fitted with one arc less, its longest arc cut in two, one a little sharper than the other
		auto arcs{arcs_of(previous.vertices.front())};
		auto longest{arcs.begin()};
		for (auto arc{arcs.begin()}; arc != arcs.end(); ++arc)
			if (arc->share * arc->radius > longest->share * longest->radius)
				longest = arc;
		auto const cut{*longest};
		*longest = {cut.radius * 0.9, cut.transition, cut.share * 0.5};
		arcs.insert(std::next(longest), {cut.radius * 1.1, 0.0, cut.share * 0.5});
		auto choice{fitted(shaped_of(arcs))};
		// and from the curvature the points show, each arc along its share of the curve
		std::vector<compound_arc> shown;
		auto const span{(to - from) / count};
		for (std::size_t j = 0; j < count; ++j)
		{
			double sum{0.0};
			int points{0};
			for (auto const &[chainage, curvature] : curvatures)
				if (chainage >= from + span * j && chainage < from + span * (j + 1))
				{
					sum += curvature;
					++points;
				}
			auto const curvature{points > 0 ? sum / points : 1.0 / std::max(kTraceRadiusMin, single.radius)};
			shown.push_back({std::clamp(1.0 / std::max(curvature, 1.0 / kTraceRadiusMax), kTraceRadiusMin, kTraceRadiusMax), 0.0, span * std::max(curvature, 1e-6)});
		}
		if (auto other{fitted(shaped_of(shown))}; other.has_value() && (false == choice.has_value() || other->second.worst < choice->second.worst))
			choice = other;
		if (false == choice.has_value())
			continue;
		previous = choice->first;
		counts.push_back(choice->second);
		if (choice->second.worst <= kPointsWithin)
			break;
	}
	if (counts.empty())
		return std::nullopt;
	// the fewest arcs within kPointsWithin; where none keeps within it, the fewest no more than 0.1 m further off than the closest:
	// more arcs only to follow points clicked a little off aren't worth it
	auto closest{counts.front().worst};
	for (auto const &fit : counts)
		closest = std::min(closest, fit.worst);
	for (auto const &fit : counts)
		if (fit.worst <= std::max(kPointsWithin, closest + 0.1))
			return fit;
	return counts.back();
}

} // namespace

namespace
{

// the curves of the design with the radii and the transitions they were laid with, to the metre, or, Exact, as they were laid, a hair
// under: where a curve fills the room up to an end of the stretch or to the next curve, a metre more doesn't fit there, the design laid
// again warning of radii made smaller
design settled_curves(design Design, result const &Laid, bool const Exact, double const Speed)
{
	for (auto const &curve : Laid.curves)
	{
		if (curve.vertex < 0 || curve.vertex >= static_cast<int>(Design.vertices.size()))
			continue;
		auto &curved{Design.vertices[curve.vertex]};
		auto const radius{Exact ? curve.radius * (1.0 - 1e-6) : std::max(kTraceRadiusMin, std::round(curve.radius))};
		auto const in{Exact ? curve.transition_in * (1.0 - 1e-6) : std::round(curve.transition_in)};
		auto const out{Exact ? curve.transition_out * (1.0 - 1e-6) : std::round(curve.transition_out)};
		plain_curve(curved, radius, std::min(in, out), Speed, Design.norms);
		curved.transition_in = in;
		curved.transition_out = out;
	}
	return Design;
}

// the same laid again: to the metre where that gives no warning, else as laid
design settled_curves(design const &Design, double const Speed)
{
	auto const laid{compute(Design)};
	auto rounded{settled_curves(Design, laid, false, Speed)};
	if (auto const again{compute(rounded)}; again.valid && false == again.radius_warnings.empty())
		return settled_curves(Design, laid, true, Speed);
	return rounded;
}

// a chain of curves the points show, two to four: a curve where they turn one way, the next where they turn the other way or after a
// straight. The lines between the curves, the common tangents of two curves of a reverse curve, are fitted with the radii and the
// transitions; two curves turning the other way may meet with no straight between them, their tangents filling the room between their
// vertices. Two turning the same way are two curves only with a straight of shortest_between between them, else one compound curve, and
// only with Straights
std::optional<trace_stretch> fit_chain(design const &Base, std::vector<glm::dvec2> const &Points, double const Speed, bool const Straights)
{
	auto const n{Points.size()};
	if (n < 6)
		return std::nullopt;
	auto const start{plan_of(Base.start)};
	auto const startdirection{glm::normalize(Base.start_direction)};
	auto const end{plan_of(Base.end)};
	auto const enddirection{glm::normalize(Base.end_direction)};
	auto const travel{end - start};

	// the curvature the points show, of the circle through each point and its neighbours, a little smoothed
	std::vector<double> bends(n, 0.0);
	for (std::size_t i = 1; i + 1 < n; ++i)
	{
		auto const a{Points[i] - Points[i - 1]};
		auto const b{Points[i + 1] - Points[i]};
		auto const c{Points[i + 1] - Points[i - 1]};
		auto const lengths{glm::length(a) * glm::length(b) * glm::length(c)};
		if (lengths > 1e-9)
			bends[i] = 2.0 * cross(a, b) / lengths;
	}
	std::vector<double> curvature(n, 0.0);
	double peak{0.0};
	for (std::size_t i = 1; i + 1 < n; ++i)
	{
		curvature[i] = (bends[i - 1] + 2.0 * bends[i] + bends[i + 1]) / ((i > 1 ? 1.0 : 0.0) + 2.0 + (i + 2 < n ? 1.0 : 0.0));
		peak = std::max(peak, std::abs(curvature[i]));
	}
	if (peak < 1.0 / kTraceRadiusMax)
		return std::nullopt;
	// the parts turning one way, of two points at least, the ones turning hardly at all between them
	struct part
	{
		int side;
		std::size_t first;
		std::size_t last;
	};
	std::vector<part> parts;
	for (std::size_t i = 1; i + 1 < n; ++i)
	{
		int const side{curvature[i] > 0.2 * peak ? 1 : curvature[i] < -0.2 * peak ? -1 : 0};
		if (side == 0)
			continue;
		if (false == parts.empty() && parts.back().side == side && parts.back().last + 1 == i)
			parts.back().last = i;
		else
			parts.push_back({side, i, i});
	}
	parts.erase(std::remove_if(parts.begin(), parts.end(), [](part const &Part) { return Part.last == Part.first; }), parts.end());
	// the same way again right after, with nothing straight between, is the same curve
	for (std::size_t i = 1; i < parts.size();)
		if (parts[i].side == parts[i - 1].side && parts[i].first <= parts[i - 1].last + 1)
		{
			parts[i - 1].last = parts[i].last;
			parts.erase(parts.begin() + static_cast<std::ptrdiff_t>(i));
		}
		else
			++i;
	auto const count{parts.size()};
	if (count < 2 || count > 4 || (false == Straights && std::any_of(parts.begin() + 1, parts.end(), [&](part const &Part) { return Part.side == (&Part - 1)->side; })))
		return std::nullopt;

	// the parameters: of each line between two curves its direction and its distance from the start across it, then the logarithms of the
	// radii, then the square roots of the lengths of the transitions
	auto const lines{count - 1};
	auto const shaped = [&](std::vector<double> const &Parameters, design &Trial) {
		std::vector<glm::dvec2> origins;
		std::vector<glm::dvec2> directions;
		for (std::size_t j = 0; j < lines; ++j)
		{
			glm::dvec2 const direction{std::cos(Parameters[2 * j]), std::sin(Parameters[2 * j])};
			glm::dvec2 const across{-direction.y, direction.x};
			directions.push_back(direction);
			origins.push_back(start + across * Parameters[2 * j + 1]);
		}
		// where two lines cross: along the first one, along the second
		std::vector<glm::dvec2> vertices;
		double first;
		double along;
		if (false == geometry::lines_cross(start, startdirection, origins.front(), directions.front(), first, along) || first <= 0.0)
			return false;
		vertices.push_back(start + startdirection * first);
		for (std::size_t j = 0; j + 1 < lines; ++j)
		{
			double a;
			double b;
			if (false == geometry::lines_cross(origins[j], directions[j], origins[j + 1], directions[j + 1], a, b))
				return false;
			vertices.push_back(origins[j] + directions[j] * a);
		}
		double last;
		if (false == geometry::lines_cross(end, -enddirection, origins.back(), directions.back(), last, along) || last <= 0.0)
			return false;
		vertices.push_back(end - enddirection * last);
		// each vertex on from the one before along the line between them
		for (std::size_t j = 0; j < lines; ++j)
			if (glm::dot(vertices[j + 1] - vertices[j], directions[j]) <= 0.0)
				return false;
		Trial = Base;
		Trial.vertices.assign(count, vertex{});
		Trial.vertices.front().offset = first;
		Trial.vertices.back().offset = last;
		for (std::size_t i = 1; i + 1 < count; ++i)
			Trial.vertices[i].position = vertices[i];
		for (std::size_t i = 0; i < count; ++i)
		{
			auto const radius{std::exp(std::clamp(Parameters[2 * lines + i], std::log(kTraceRadiusMin), std::log(kTraceRadiusMax)))};
			auto const length{Parameters.size() > 2 * lines + count ? Parameters[2 * lines + count + i] * Parameters[2 * lines + count + i] : 0.0};
			plain_curve(Trial.vertices[i], radius, std::min({length, radius * vertex_deflection(Trial, i), longest_transition(Speed, radius, Trial.norms)}), Speed, Trial.norms);
		}
		return true;
	};
	residuals_of const residuals{[&](std::vector<double> const &Parameters, std::vector<double> &Residuals) {
		design trial;
		return shaped(Parameters, trial) && stretch_residuals(trial, Points, Residuals);
	}};
	// started from the line through the points where the curvature changes, the ones hardly turning there or the two either side
	std::vector<double> parameters;
	for (std::size_t j = 0; j < lines; ++j)
	{
		auto const from{parts[j].last};
		auto const to{parts[j + 1].first};
		auto direction{Points[to] - Points[from]};
		if (glm::length(direction) < 1e-3)
			return std::nullopt;
		if (glm::dot(direction, travel) < 0.0)
			direction = -direction;
		auto const angle{std::atan2(direction.y, direction.x)};
		glm::dvec2 const across{-std::sin(angle), std::cos(angle)};
		parameters.push_back(angle);
		parameters.push_back(glm::dot(across, 0.5 * (Points[from] + Points[to]) - start));
	}
	for (auto const &each : parts)
	{
		double sum{0.0};
		for (auto i{each.first}; i <= each.last; ++i)
			sum += std::abs(curvature[i]);
		parameters.push_back(std::log(std::clamp(static_cast<double>(each.last - each.first + 1) / std::max(sum, 1e-9), kTraceRadiusMin, kTraceRadiusMax)));
	}
	auto best{parameters};
	auto lowest{least_squares(best, residuals)};
	if (lowest == std::numeric_limits<double>::max())
		return std::nullopt;
	auto longer{best};
	for (std::size_t i = 0; i < count; ++i)
		longer.push_back(std::sqrt(20.0));
	if (least_squares(longer, residuals) < lowest)
		best = longer;
	design chain;
	if (false == shaped(best, chain))
		return std::nullopt;
	if (auto const laid{compute(chain)}; false == laid.valid || laid.curves.size() != count)
		return std::nullopt;
	// where two curves of a reverse curve meet, filling the room between their vertices, as they were laid
	chain = settled_curves(chain, Speed);
	auto const again{compute(chain)};
	if (false == again.valid || again.curves.size() != count || false == again.radius_warnings.empty())
		return std::nullopt;
	for (auto const &curve : again.curves)
		if (curve.radius < 1.05 * kTraceRadiusMin || curve.radius > 0.95 * kTraceRadiusMax || curve.deflection < 0.25 * kPi / 180.0 || curve.curvature_in > 0.0 || curve.curvature_out > 0.0)
			return std::nullopt;
	auto const curves{laid_curves(again)};
	for (std::size_t i = 1; i < curves.size(); ++i)
		if (curves[i].side == curves[i - 1].side && curves[i].from - curves[i - 1].to < shortest_between(Base))
			return std::nullopt;
	return stretch_of(chain, Points);
}

} // namespace

trace_stretch fit_going_on(design const &Base, std::vector<glm::dvec2> const &Points)
{
	trace_stretch fit;
	if (Points.empty() || false == Base.start_curved || Base.start_radius <= 0.0 || glm::length(Base.start_direction) < 1e-9)
		return fit;
	auto const speed{std::max(1.0, Base.speed)};
	auto const start{plan_of(Base.start)};
	auto const heading{glm::normalize(Base.start_direction)};
	auto const side{Base.start_turn > 0 ? 1.0 : -1.0};
	auto const from{1.0 / Base.start_radius};
	auto const fromcant{std::max(0.0, Base.start_cant)};
	// going on from the diverging track of a switch: its first arc of the radius of the switch, with no transition before it
	auto const held{Base.start_held};
	auto const target{Points.back()};
	// none where the last point lies on the other side of the heading
	auto const offset{target - start};
	auto const lateral{glm::dot(offset, perpendicular(heading) * side)};
	auto const chord{glm::length(offset)};
	if (lateral < 1e-3 || chord < 1.0)
		return fit;
	// the shape: the transition from the curvature of the adjoining track to the first arc, the arcs with the transitions between them,
	// the transition from the last one to the straight, which goes on to the last point where the points clicked after the curve show it
	struct shape
	{
		double in{0.0};
		std::vector<double> curvatures;
		std::vector<double> arcs;
		std::vector<double> between;
		double out{0.0};
		double straight{0.0};
	};
	auto const curvature_of = [](double const Logarithm) { return 1.0 / std::exp(std::clamp(Logarithm, std::log(kTraceRadiusMin), std::log(kTraceRadiusMax))); };
	auto const segments_of = [&](shape const &Shape) {
		std::vector<std::array<double, 3>> segments{{from, Shape.curvatures.front(), Shape.in}};
		for (std::size_t i = 0; i < Shape.curvatures.size(); ++i)
		{
			if (i > 0)
				segments.push_back({Shape.curvatures[i - 1], Shape.curvatures[i], Shape.between[i - 1]});
			segments.push_back({Shape.curvatures[i], Shape.curvatures[i], Shape.arcs[i]});
		}
		segments.push_back({Shape.curvatures.back(), 0.0, Shape.out});
		segments.push_back({0.0, 0.0, Shape.straight});
		return segments;
	};
	auto const turn_of = [&](shape const &Shape) {
		double turn{0.0};
		for (auto const &[first, last, length] : segments_of(Shape))
			turn += 0.5 * (first + last) * length;
		return turn;
	};
	// where the shape ends, heading which way
	auto const end_of = [&](shape const &Shape, glm::dvec2 &End, glm::dvec2 &Leaving) {
		glm::dvec2 origin{start};
		glm::dvec2 direction{heading};
		for (auto const &[first, last, length] : segments_of(Shape))
		{
			if (length < 1e-9)
				continue;
			auto const local{spiral_point(first, last, length, length)};
			origin += direction * local.x + perpendicular(direction) * side * local.y;
			direction = turned(direction, side * local.angle);
		}
		End = origin;
		Leaving = direction;
	};
	// the line of the shape in the plan, a point every half a metre about
	auto const line_of = [&](shape const &Shape) {
		plan_line line;
		glm::dvec2 origin{start};
		glm::dvec2 direction{heading};
		double chainage{0.0};
		line.points.push_back(origin);
		line.chainages.push_back(0.0);
		for (auto const &[first, last, length] : segments_of(Shape))
		{
			if (length < 1e-6)
				continue;
			glm::dvec2 const across{perpendicular(direction) * side};
			int const count{std::clamp(static_cast<int>(std::ceil(length / 0.5)), 1, 400)};
			for (int i = 1; i <= count; ++i)
			{
				auto const local{spiral_point(first, last, length, length * i / count)};
				line.points.push_back(origin + direction * local.x + across * local.y);
				line.chainages.push_back(chainage + length * i / count);
			}
			auto const local{spiral_point(first, last, length, length)};
			origin += direction * local.x + across * local.y;
			direction = turned(direction, side * local.angle);
			chainage += length;
		}
		return line;
	};
	// the shape of the parameters: the square roots of the lengths of the transitions and of the straight, then for each arc but the
	// last the logarithm of its radius, the square root of its length and of the transition after it; the last arc, its radius and
	// length, Last. Held, the transition in is none and the first arc of the radius of the switch; a single arc so is given the square
	// root of the straight after it in Last in place of its radius
	auto const single_held = [&](std::vector<double> const &Parameters) { return held && Parameters.size() == 3; };
	auto const shaped = [&](std::vector<double> const &Parameters, std::array<double, 2> const &Last) {
		shape result;
		result.in = held ? 0.0 : Parameters[0] * Parameters[0];
		result.out = Parameters[1] * Parameters[1];
		result.straight = Parameters[2] * Parameters[2];
		for (std::size_t i = 3; i + 2 < Parameters.size(); i += 3)
		{
			result.curvatures.push_back(held && i == 3 ? from : curvature_of(Parameters[i]));
			result.arcs.push_back(Parameters[i + 1] * Parameters[i + 1]);
			result.between.push_back(Parameters[i + 2] * Parameters[i + 2]);
		}
		if (single_held(Parameters))
		{
			result.curvatures.push_back(from);
			result.straight = Last[0] * Last[0];
		}
		else
			result.curvatures.push_back(curvature_of(Last[0]));
		result.arcs.push_back(Last[1] * Last[1]);
		return result;
	};
	// the radius and the length of the last arc bringing the end to the last point, the rest given: by Newton from the ones found last
	auto const reach = [&](std::vector<double> const &Parameters, std::array<double, 2> &Solution) {
		auto const miss = [&](std::array<double, 2> const &At, glm::dvec2 &Miss) {
			auto const trial{shaped(Parameters, At)};
			if (turn_of(trial) > kPi - 1e-2)
				return false;
			glm::dvec2 end;
			glm::dvec2 leaving;
			end_of(trial, end, leaving);
			Miss = end - target;
			return true;
		};
		constexpr double kStep{1e-6};
		auto at{Solution};
		for (int i = 0; i < 40; ++i)
		{
			glm::dvec2 here;
			if (false == miss(at, here))
				return false;
			if (glm::length(here) < 1e-5)
			{
				Solution = at;
				return true;
			}
			glm::dvec2 radial;
			glm::dvec2 along;
			auto byradius{at};
			byradius[0] += kStep;
			auto byarc{at};
			byarc[1] += kStep;
			if (false == miss(byradius, radial) || false == miss(byarc, along))
				return false;
			radial = (radial - here) / kStep;
			along = (along - here) / kStep;
			auto const determinant{radial.x * along.y - radial.y * along.x};
			if (std::abs(determinant) < 1e-12)
				return false;
			auto const stepradius{(along.x * here.y - here.x * along.y) / determinant};
			auto const steparc{(here.x * radial.y - radial.x * here.y) / determinant};
			// half a step of the logarithm of the radius at most, the curve not swinging round; of the root of the straight after an arc
			// held two
			auto const damped{std::min(1.0, (single_held(Parameters) ? 2.0 : 0.5) / std::max(std::abs(stepradius), 1e-12))};
			at[0] += stepradius * damped;
			if (single_held(Parameters))
				at[0] = std::abs(at[0]);
			at[1] = std::abs(at[1] + steparc * damped);
		}
		return false;
	};
	// the transitions as long as the norms ask for where the points don't show them shorter: a metre short of it weighed as a millimetre
	// off a point, which only chooses among the shapes too few points leave open
	constexpr double kShort{0.001};
	std::array<double, 2> solution{};
	residuals_of const residuals{[&](std::vector<double> const &Parameters, std::vector<double> &Residuals) {
		auto at{solution};
		if (false == reach(Parameters, at))
			return false;
		solution = at;
		auto const trial{shaped(Parameters, at)};
		auto const line{line_of(trial)};
		Residuals.clear();
		for (std::size_t i = 0; i + 1 < Points.size(); ++i)
			Residuals.push_back(project_on(line, Points[i]).offset);
		auto const need_between = [&](double const First, double const Second) {
			auto const sharper{1.0 / std::max({First, Second, 1e-9})};
			auto const allowed{std::max(1.0, std::min(speed, speed_limit(sharper, Base.norms)))};
			return std::min(between_curves(allowed, First, 0.0, Second, 0.0, Base.norms), 0.9 * longest_transition(speed, sharper, Base.norms));
		};
		auto const last{1.0 / trial.curvatures.back()};
		auto const allowed{std::max(1.0, std::min(speed, speed_limit(last, Base.norms)))};
		Residuals.push_back(kShort * std::max(0.0, need_between(from, trial.curvatures.front()) - trial.in));
		for (std::size_t i = 1; i < trial.curvatures.size(); ++i)
			Residuals.push_back(kShort * std::max(0.0, need_between(trial.curvatures[i - 1], trial.curvatures[i]) - trial.between[i - 1]));
		Residuals.push_back(kShort * std::max(0.0, std::min(transition_length(allowed, last, 0.0, Base.norms), 0.9 * longest_transition(speed, last, Base.norms)) - trial.out));
		// the curve ending at the last point where the points don't show a straight after it
		Residuals.push_back(kShort * trial.straight);
		return true;
	}};
	std::vector<double> best;
	std::array<double, 2> bestsolution{};
	auto lowest{std::numeric_limits<double>::max()};
	// the square root of the straight to start from: off nought, where the fit couldn't move it
	constexpr double kSeed{0.5};
	// an arc of a compound curve shorter than the shortest one, a sharp one squeezed between two long transitions to bring the curve to
	// the points (R 21 m for a millimetre after R 229 m): no curve to lay, as the fit of the arcs through the points has it. The first arc
	// going on from a switch at its radius may be as short as it is
	auto const kinked = [&](shape const &Shape) {
		if (Shape.curvatures.size() < 2)
			return false;
		for (std::size_t i = held ? 1 : 0; i < Shape.arcs.size(); ++i)
			if (Shape.arcs[i] < kShortestArc)
				return true;
		return false;
	};
	// the parameters fitted: all but those of a curve going on from a switch which the shape doesn't take, the transition in and the
	// radius of the first arc, or the straight after a single arc, which the reach finds then
	auto const attempt = [&](std::vector<double> Parameters, std::array<double, 2> const &Start) {
		solution = Start;
		std::vector<std::size_t> loose;
		for (std::size_t i = 0; i < Parameters.size(); ++i)
			if (false == held || (i != 0 && i != (single_held(Parameters) ? 2u : 3u)))
				loose.push_back(i);
		auto const whole = [&](std::vector<double> const &Loose) {
			auto result{Parameters};
			for (std::size_t i = 0; i < loose.size(); ++i)
				result[loose[i]] = Loose[i];
			return result;
		};
		std::vector<double> fitted;
		for (auto const i : loose)
			fitted.push_back(Parameters[i]);
		residuals_of const some{[&](std::vector<double> const &Loose, std::vector<double> &Residuals) { return residuals(whole(Loose), Residuals); }};
		if (auto const cost{least_squares(fitted, some)}; cost < lowest && false == kinked(shaped(whole(fitted), solution)))
		{
			lowest = cost;
			best = whole(fitted);
			bestsolution = solution;
		}
	};
	// one arc, from two guesses of how far it turns: as the points head at the end, Base ending that way, and as the circle going on
	// from the start through the last point does; held at the radius of a switch, as far round as that, the straight after it the rest
	// of the way
	std::vector<double> turns;
	if (glm::length(Base.end_direction) > 1e-9)
	{
		auto const ending{glm::normalize(Base.end_direction)};
		auto turn{std::atan2(side * cross(heading, ending), glm::dot(heading, ending))};
		if (turn < 0.0)
			turn += 2.0 * kPi;
		if (turn > 1e-3 && turn < kPi - 2e-2)
			turns.push_back(turn);
	}
	turns.push_back(2.0 * std::asin(std::clamp(lateral / chord, 0.0, 1.0)));
	for (auto const turn : turns)
		for (auto const length : {10.0, 25.0})
		{
			if (held)
			{
				auto const arc{std::max(1.0, Base.start_radius * turn - 0.5 * length)};
				attempt({0.0, std::sqrt(length), 0.0}, {std::sqrt(std::max(1.0, chord - arc)), std::sqrt(arc)});
				continue;
			}
			auto const radius{std::clamp(chord / std::max(1e-3, 2.0 * std::sin(0.5 * turn)), kTraceRadiusMin, kTraceRadiusMax)};
			attempt({std::sqrt(length), std::sqrt(length), kSeed}, {std::log(radius), std::sqrt(std::max(1.0, radius * turn - length))});
		}
	// Abreast: the shape ends abreast of the last point, not on it
	auto const settle = [&](shape const &Found, bool const Abreast) -> std::optional<trace_stretch> {
		glm::dvec2 end;
		glm::dvec2 leaving;
		end_of(Found, end, leaving);
		if (false == Abreast && glm::distance(end, target) > 0.01)
			return std::nullopt;
		// laid as the design lays a curve going on from an adjoining one: its vertex where the straights cross, its radii scaled to make
		// the tangent before it reach the start, which they do
		design laid{Base};
		laid.end = {end.x, Base.end.y, end.y};
		laid.end_direction = leaving;
		laid.end_curved = false;
		laid.end_reserve = 0.0;
		laid.vertices.assign(1, vertex{});
		glm::dvec2 meeting;
		if (false == tangent_intersection(laid, meeting))
			return std::nullopt;
		auto &curve{laid.vertices.front()};
		curve.position = meeting;
		curve.radius = 1.0 / Found.curvatures.front();
		curve.transition_in = Found.in;
		curve.transition_out = Found.out;
		if (Found.curvatures.size() > 1)
		{
			// the shares of the arcs by how far they turn
			curve.compound = true;
			curve.share = std::max(1e-6, Found.arcs.front() * Found.curvatures.front());
			for (std::size_t i = 1; i < Found.curvatures.size(); ++i)
			{
				compound_arc arc;
				arc.radius = 1.0 / Found.curvatures[i];
				arc.transition = Found.between[i - 1];
				arc.share = std::max(1e-6, Found.arcs[i] * Found.curvatures[i]);
				curve.arcs.push_back(arc);
			}
		}
		curve.cant = Found.curvatures.size() > 1 ? 0.0 : curved_cant(curve.radius, Found.in, Found.out, fromcant, speed, Base.norms);
		auto stretch{stretch_of(laid, Points)};
		if (stretch.has_value())
		{
			stretch->leaving = leaving;
			stretch->ending = end;
		}
		return stretch;
	};
	std::optional<trace_stretch> one;
	if (false == best.empty())
		one = settle(shaped(best, bestsolution), false);
	// held, the arc of the switch alone, as far round as abreast of the last point seen from its centre and ending there: as the points
	// clicked along it show it, no transition out of it bringing it to a point on it
	if (held)
	{
		auto const centre{start + perpendicular(heading) * side * Base.start_radius};
		auto const outward{target - centre};
		auto sweep{std::fmod(side * (std::atan2(outward.y, outward.x) - std::atan2(start.y - centre.y, start.x - centre.x)), 2.0 * kPi)};
		if (sweep < 0.0)
			sweep += 2.0 * kPi;
		if (glm::length(outward) > 0.5 && sweep > 1e-3 && sweep < kPi - 2e-2)
		{
			shape arc;
			arc.curvatures = {from};
			arc.arcs = {Base.start_radius * sweep};
			if (auto const alone{settle(arc, true)}; alone.has_value() && (false == one.has_value() || alone->rms < one->rms))
				one = alone;
		}
	}
	// two arcs where one doesn't keep to the points and there are enough of them to tell: a flatter one going on from the end, even a
	// straight about, then a sharper one, as where the line turns hard at the last points. Going on from a switch, the first one of its
	// radius, then a flatter or a sharper one
	if (Points.size() >= 3 && (false == one.has_value() || one->worst > kPointsWithin))
	{
		auto const single{best.empty() || held ? std::array<double, 2>{std::log(std::clamp(chord, kTraceRadiusMin, kTraceRadiusMax)), std::sqrt(chord)} : bestsolution};
		auto const radius{std::exp(single[0])};
		lowest = std::numeric_limits<double>::max();
		best.clear();
		std::vector<double> const firsts{held ? std::vector<double>{Base.start_radius} : std::vector<double>{Base.start_radius, 2.0 * Base.start_radius, 3000.0}};
		std::vector<double> const seconds{held ? std::vector<double>{0.5 * Base.start_radius, 2.0 * Base.start_radius} : std::vector<double>{0.5 * radius}};
		auto const shares{held ? std::array<double, 2>{0.15, 0.5} : std::array<double, 2>{0.4, 0.7}};
		for (auto const first : firsts)
			for (auto const second : seconds)
				for (auto const share : shares)
				{
					auto const along{std::max(1.0, share * chord)};
					attempt({std::sqrt(10.0), std::sqrt(10.0), kSeed, std::log(first), std::sqrt(along), std::sqrt(10.0)}, {std::log(std::max(kTraceRadiusMin, second)), std::sqrt(std::max(1.0, (1.0 - share) * chord))});
				}
		if (false == best.empty())
			if (auto const two{settle(shaped(best, bestsolution), false)}; two.has_value() && (false == one.has_value() || two->rms < 0.6 * one->rms))
				one = two;
	}
	if (one.has_value())
		fit = *one;
	return fit;
}

trace_stretch fit_trace_stretch(design const &Base, std::vector<glm::dvec2> const &Points, int const Radii, bool const Straights)
{
	trace_stretch chosen;
	if (Points.empty() || glm::length(Base.start_direction) < 1e-9 || glm::length(Base.end_direction) < 1e-9)
		return chosen;
	auto const speed{std::max(1.0, Base.speed)};
	auto const start{plan_of(Base.start)};
	auto const startdirection{glm::normalize(Base.start_direction)};
	auto const end{plan_of(Base.end)};
	auto const enddirection{glm::normalize(Base.end_direction)};
	// one compound curve with an arc flattened to a straight between its curves, where straights between curves may be laid
	auto const flat = [&]() { return Straights && chosen.valid && chosen.vertices.size() == 1 && flattened(chosen.vertices.front()); };
	// a shape with more curves is taken where it follows the points much closer than the simpler one, which doesn't keep to them; curves
	// with straights between them over such a compound curve where they keep to the points about as closely
	auto const take = [&](std::optional<trace_stretch> const &Candidate) {
		if (false == Candidate.has_value())
			return;
		if (false == chosen.valid || (chosen.rms > kStretchClose && Candidate->rms < 0.6 * chosen.rms) ||
		    (flat() && Candidate->vertices.size() >= 2 && Candidate->rms < std::max(kStretchClose, 2.0 * chosen.rms)))
			chosen = *Candidate;
	};

	// one straight, where the two are one
	if (auto const offset{end - start}; glm::dot(startdirection, enddirection) > std::cos(2e-3) && std::abs(cross(startdirection, offset)) < 0.25 && glm::dot(startdirection, offset) > 0.0)
	{
		auto straight{Base};
		straight.vertices.clear();
		auto const along{glm::dot(startdirection, offset)};
		straight.end = {Base.start.x + startdirection.x * along, Base.end.y, Base.start.z + startdirection.y * along};
		straight.end_direction = startdirection;
		take(stretch_of(straight, Points));
	}

	// one curve where the straights meet: the radius of a plain arc first, then the radius and the transitions together; the radii of
	// a compound curve where one radius doesn't keep to the points, or where they're asked for
	std::optional<trace_stretch> one;
	if (glm::dvec2 meeting; tangent_intersection(Base, meeting))
	{
		auto single{Base};
		single.vertices.assign(1, vertex{});
		auto &curve{single.vertices.front()};
		curve.transition_in = curve.transition_out = curve.cant = 0.0;
		// a curve going on from the curve of the adjoining track at the start, or into the one at the end, turning its way: the
		// transitions the points show, the radius the one which begins it there; the radii of a compound curve where they're asked for
		// or one doesn't keep to the points
		int const side{cross(startdirection, enddirection) > 0.0 ? 1 : -1};
		auto const fromstart{Base.start_curved && Base.start_radius > 0.0 && Base.start_turn == side};
		auto const curved{fromstart || (Base.end_curved && Base.end_radius > 0.0 && Base.end_turn == side)};
		// going on from the diverging track of a switch: at its radius, the transition on the other side of the arc making it begin there
		auto const held{fromstart ? Base.start_held : Base.end_held};
		if (curved && held)
		{
			curve.radius = fromstart ? Base.start_radius : Base.end_radius;
			one = stretch_of(single, Points);
		}
		else if (curved)
		{
			curve.radius = fromstart ? Base.start_radius : Base.end_radius;
			fit_curved(single, 0, Points, fromstart);
			one = stretch_of(single, Points);
			// a compound curve where one radius doesn't keep to points enough for two; where it doesn't reach the end at all, as where a
			// curve sharper than the one it goes on from ends the stretch, of two radii from fewer points too. Its transition from the curve
			// of the adjoining track as long as the norms ask for at least, the curvature not jumping at the joint
			auto const loose{one.has_value() && (one->rms > kStretchClose || one->worst > kPointsWithin)};
			auto const missing{false == one.has_value()};
			if (Radii != 1 && (missing || ((Radii > 1 || loose) && Points.size() >= 4)))
				if (auto const fit{fit_trace(single, 0, Points, Radii > 1 ? Radii : 2)}; fit.valid && fit.fitted.compound)
				{
					auto several{single};
					several.vertices.front() = fit.fitted;
					fit_compound(several, 0, Points);
					auto &shape{several.vertices.front()};
					auto const joining{fromstart ? shape.radius : shape.arcs.empty() ? shape.radius : shape.arcs.back().radius};
					auto const adjoining{fromstart ? Base.start_radius : Base.end_radius};
					auto const allowed{std::max(1.0, std::min(speed, speed_limit(std::min(joining, adjoining), Base.norms)))};
					auto const needed{std::min(between_curves(allowed, 1.0 / adjoining, std::max(0.0, fromstart ? Base.start_cant : Base.end_cant), 1.0 / std::max(1.0, joining), 0.0, Base.norms),
					                           0.9 * longest_transition(speed, adjoining, Base.norms))};
					auto &between{fromstart ? shape.transition_in : shape.transition_out};
					between = std::max(between, std::round(needed));
					if (auto const compound{stretch_of(several, Points)}; compound.has_value() && (Radii > 1 || false == one.has_value() || compound->rms < 0.75 * one->rms))
						one = compound;
				}
		}
		else if (auto const arc{fit_trace(single, 0, Points, 1)}; arc.valid)
		{
			curve = arc.fitted;
			fit_curve(single, 0, Points);
			// a fit running off to the largest radius, its transitions eating the arc, is two transitions meeting
			auto const &fitted{single.vertices.front()};
			auto const runaway{fitted.radius > 0.95 * kTraceRadiusMax};
			if (false == runaway)
				one = stretch_of(single, Points);
			auto spirals{single};
			spirals.vertices.front() = {};
			spirals.vertices.front().spirals = true;
			residuals_of const peak{[&](std::vector<double> const &Parameters, std::vector<double> &Residuals) {
				auto trial{spirals};
				trial.vertices.front().radius = std::exp(std::clamp(Parameters[0], std::log(kTraceRadiusMin), std::log(kTraceRadiusMax)));
				return stretch_residuals(trial, Points, Residuals);
			}};
			std::vector<double> radius{std::log(std::clamp(arc.fitted.radius, kTraceRadiusMin, kTraceRadiusMax))};
			least_squares(radius, peak);
			spirals.vertices.front().radius = std::round(std::exp(std::clamp(radius[0], std::log(kTraceRadiusMin), std::log(kTraceRadiusMax))));
			if (auto const candidate{stretch_of(spirals, Points)}; candidate.has_value() && (false == one.has_value() || candidate->rms < one->rms))
				one = candidate;
			// a compound curve is the one curve which keeps to the points the closest: of as few radii as keep within kPointsWithin of
			// them, up to three, or up to eight where three don't; fitted with the transitions between the arcs from several starts,
			// the problem having more than one bottom
			auto const loose = [&]() { return one->rms > kStretchClose || one->worst > kPointsWithin; };
			if (one.has_value() && Radii != 1 && (Radii > 1 || loose()))
			{
				std::optional<trace_stretch> compound;
				if (auto const fit{fit_trace(single, 0, Points, Radii)}; fit.valid && fit.fitted.compound)
				{
					auto several{single};
					several.vertices.front() = fit.fitted;
					fit_compound(several, 0, Points);
					compound = stretch_of(several, Points);
				}
				// no more radii than half the points: a few points clicked a little off don't show a curve of many radii
				if (auto const more{fit_arcs(single, Points, Radii > 1 ? Radii : std::clamp(static_cast<int>(Points.size()) / 2, 2, 8))}; more.has_value() && (false == compound.has_value() || more->worst < compound->worst))
					compound = more;
				// one with an arc as flat as a straight between its curves only where it keeps to all the points much closer: not to come
				// nearer to a point clicked a little off, sharp curves either side of a straight
				if (compound.has_value() && (Radii > 1 || compound->rms < 0.75 * one->rms || (one->worst > kPointsWithin && compound->worst < one->worst && (compound->vertices.empty() || false == flattened(compound->vertices.front())))))
					one = compound;
			}
		}
		take(one);
	}

	// two curves with a straight between them, turning the same way or not: the straight is where the points keep to a line between
	// the curves. The parameters: the direction of the straight, its distance from the start across it, the logarithms of the radii,
	// first with the transitions of the single curve, then with the square roots of the lengths of the transitions of each curve too
	constexpr std::size_t kParameters{4};
	if (Straights && Points.size() > kParameters && (false == chosen.valid || chosen.rms > kStretchClose || flat()))
	{
		auto const transition{one.has_value() && false == one->vertices.empty() ? one->vertices.front().transition_in : 0.0};
		auto const shaped = [&](std::vector<double> const &Parameters, design &Trial) {
			glm::dvec2 const direction{std::cos(Parameters[0]), std::sin(Parameters[0])};
			glm::dvec2 const across{-direction.y, direction.x};
			auto const meet = [&](glm::dvec2 const &From, glm::dvec2 const &Along, double &Distance) {
				auto const denominator{glm::dot(across, Along)};
				if (std::abs(denominator) < 1e-9)
					return false;
				Distance = (Parameters[1] - glm::dot(across, From - start)) / denominator;
				return Distance > 0.0;
			};
			double first;
			double second;
			if (false == meet(start, startdirection, first) || false == meet(end, -enddirection, second))
				return false;
			if (glm::dot((end - enddirection * second) - (start + startdirection * first), direction) <= 0.0)
				return false;
			Trial = Base;
			Trial.vertices.assign(2, vertex{});
			Trial.vertices[0].offset = first;
			Trial.vertices[1].offset = second;
			for (std::size_t i = 0; i < 2; ++i)
			{
				auto const radius{std::exp(std::clamp(Parameters[2 + i], std::log(kTraceRadiusMin), std::log(kTraceRadiusMax)))};
				auto const length{Parameters.size() > kParameters ? Parameters[kParameters + i] * Parameters[kParameters + i] : transition};
				plain_curve(Trial.vertices[i], radius, std::min({length, radius * vertex_deflection(Trial, i), longest_transition(speed, radius, Trial.norms)}), speed, Trial.norms);
			}
			return true;
		};
		residuals_of const residuals{[&](std::vector<double> const &Parameters, std::vector<double> &Residuals) {
			design trial;
			return shaped(Parameters, trial) && stretch_residuals(trial, Points, Residuals);
		}};
		// started from the lines through neighbouring points spread over the stretch, short of the two on the straight at its end
		auto const guess{std::log(one.has_value() && false == one->vertices.empty() ? std::max(kTraceRadiusMin, one->vertices.front().radius) : 1000.0)};
		auto const travel{end - start};
		std::vector<double> best;
		auto lowest{std::numeric_limits<double>::max()};
		auto const pairs{Points.size() - 2};
		auto const starts{std::min<std::size_t>(4, pairs)};
		for (std::size_t s = 0; s < starts; ++s)
		{
			auto const i{starts > 1 ? s * (pairs - 1) / (starts - 1) : 0};
			auto along{Points[i + 1] - Points[i]};
			if (glm::length(along) < 1e-3)
				continue;
			if (glm::dot(along, travel) < 0.0)
				along = -along;
			auto const angle{std::atan2(along.y, along.x)};
			glm::dvec2 const across{-std::sin(angle), std::cos(angle)};
			std::vector<double> parameters{angle, glm::dot(across, Points[i] - start), guess, guess};
			if (auto const cost{least_squares(parameters, residuals)}; cost < lowest)
			{
				lowest = cost;
				best = parameters;
			}
		}
		if (false == best.empty())
		{
			auto longer{best};
			longer.push_back(std::sqrt(std::max(transition, 10.0)));
			longer.push_back(longer.back());
			if (least_squares(longer, residuals) < lowest)
				best = longer;
		}
		if (design two; false == best.empty() && shaped(best, two))
		{
			// a curve ending at the last point, filling the room up to the end, as it was laid
			two = settled_curves(two, speed);
			// none where a curve ran to the end of the radii tried, turns hardly at all, or doesn't fit between the straights: the
			// points made the fit find two curves where they show one
			auto const laid{compute(two)};
			// a curve of the two going on from the curve of an adjoining track has its transitions tied together here, to the straight
			// and between the curves alike: none such, the one curve going on from it is fitted on its own
			auto const sound{laid.valid && laid.radius_warnings.empty() && laid.curves.size() == 2 && std::all_of(laid.curves.begin(), laid.curves.end(), [](curve_report const &Curve) {
				return Curve.radius > 1.05 * kTraceRadiusMin && Curve.radius < 0.95 * kTraceRadiusMax && Curve.deflection > 0.25 * kPi / 180.0 && Curve.curvature_in <= 0.0 && Curve.curvature_out <= 0.0;
			})};
			// one compound curve rather than two turning the same way with a short straight between them; two such only where the
			// straight is long and they keep to the points much closer
			auto const candidate{sound ? stretch_of(two, Points) : std::nullopt};
			auto const curves{laid_curves(laid)};
			auto const same{curves.size() == 2 && curves[0].side == curves[1].side};
			if (candidate.has_value() && (false == same || (curves[1].from - curves[0].to >= shortest_between(Base) && (false == chosen.valid || candidate->rms < 0.5 * chosen.rms || flat()))))
				take(candidate);
		}
	}

	// a reverse curve, or more curves turning to and fro or the same way with straights between them; of no more curves than the two
	// fitted above, the closer one
	if (false == chosen.valid || chosen.rms > kStretchClose || chosen.vertices.size() >= 2 || flat())
		if (auto const chain{fit_chain(Base, Points, speed, Straights)}; chain.has_value())
		{
			if (chosen.valid && chosen.vertices.size() >= 2 && chain->vertices.size() <= chosen.vertices.size() && chain->rms < chosen.rms)
				chosen = *chain;
			else
				take(chain);
		}
	return chosen;
}

stretch_fitter::~stretch_fitter()
{
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		m_stop = true;
	}
	m_wake.notify_all();
	if (m_thread.joinable())
		m_thread.join();
}

void stretch_fitter::ask(request Request)
{
	{
		std::lock_guard<std::mutex> lock{m_mutex};
		auto const same = [&](std::vector<double> const &Key) { return Key == Request.key; };
		if (std::any_of(m_asked.begin(), m_asked.end(), [&](request const &Asked) { return same(Asked.key); }) || (m_fitting.has_value() && same(m_fitting->key)) ||
		    std::any_of(m_results.begin(), m_results.end(), [&](auto const &Result) { return same(Result.first); }))
			return;
		m_asked.push_back(std::move(Request));
		// the cursor moves on: what was asked for long ago waits no more
		constexpr std::size_t kWaiting{8};
		if (m_asked.size() > kWaiting)
			m_asked.erase(m_asked.begin());
		if (false == m_thread.joinable())
			m_thread = std::thread{[this]() { work(); }};
	}
	m_wake.notify_one();
}

bool stretch_fitter::done(std::vector<double> const &Key, trace_stretch &Fit) const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	for (auto const &result : m_results)
		if (result.first == Key)
		{
			Fit = result.second;
			return true;
		}
	return false;
}

bool stretch_fitter::latest(std::vector<double> &Key, trace_stretch &Fit) const
{
	std::lock_guard<std::mutex> lock{m_mutex};
	if (m_results.empty())
		return false;
	Key = m_results.front().first;
	Fit = m_results.front().second;
	return true;
}

void stretch_fitter::work()
{
	std::unique_lock<std::mutex> lock{m_mutex};
	for (;;)
	{
		m_wake.wait(lock, [this]() { return m_stop || false == m_asked.empty(); });
		if (m_stop)
			return;
		// the one asked for last first: it's what the cursor shows now
		m_fitting = std::move(m_asked.back());
		m_asked.pop_back();
		auto const asked{*m_fitting};
		lock.unlock();
		auto fit{asked.going ? fit_going_on(asked.base, asked.points) : fit_trace_stretch(asked.base, asked.points, asked.radii, asked.straights)};
		lock.lock();
		constexpr std::size_t kKept{16};
		m_results.insert(m_results.begin(), {asked.key, std::move(fit)});
		if (m_results.size() > kKept)
			m_results.pop_back();
		m_fitting.reset();
	}
}

}
