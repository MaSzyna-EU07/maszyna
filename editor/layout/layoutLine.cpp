/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutLine.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <optional>

namespace layout
{

namespace
{

double constexpr kPi{3.14159265358979323846};
double constexpr kSame{1e-6}; // m: a straight shorter than this is none, a curve within this of its node begins at it
double constexpr kCantStep{0.5}; // mm: a smaller change isn't a step of the cant (the norms engine takes it alike)
double constexpr kRailBase{1500.0}; // mm between the middles of the rail heads, the base of the roll

double sign_of(double const Value)
{
	return Value < 0.0 ? -1.0 : 1.0;
}

// the elements of the curve after one another from the frame of the tangent point: the next starts where the last ended
void chain(std::vector<element> &Elements, element Element, glm::dvec2 &Origin, glm::dvec2 &Direction)
{
	Element.origin = Origin;
	Element.direction = Direction;
	Origin = Element.position(Element.length);
	Direction = glm::normalize(Element.tangent(Element.length));
	Elements.push_back(Element);
}

element in_frame(element Element, glm::dvec2 const &Origin, glm::dvec2 const &Direction)
{
	auto const left{left_of(Direction)};
	Element.origin = Origin + Direction * Element.origin.x + left * Element.origin.y;
	Element.direction = glm::normalize(Direction * Element.direction.x + left * Element.direction.y);
	return Element;
}

std::optional<double> tier_value(norms::thresholds const &Limits)
{
	for (auto const *bound : {&Limits.prefer, &Limits.warn, &Limits.block})
		if (bound->has_value())
			return (*bound)->value;
	return std::nullopt;
}

} // namespace

curve_fit fit_curve(transition_shape const Shape, double const Deflection, double const Radius, double const In, double const Out, double const From, double const To)
{
	curve_fit result;
	auto const in{std::max(0.0, In)};
	auto const out{std::max(0.0, Out)};
	if (std::abs(Deflection) < 1e-9 && in == 0.0 && out == 0.0)
		return result; // the tangents go on alike: no curve
	if (Radius < 1.0)
	{
		result.error = std::format("the radius {:.1f} m is too small", Radius);
		return result;
	}
	auto const side{sign_of(Deflection)};
	auto const curvature{side / Radius};
	auto const kind{Shape == transition_shape::bloss ? element_kind::bloss : element_kind::clothoid};
	auto const parabola{Shape == transition_shape::cubic_parabola};
	if (parabola && ((in > 0.0 && From != 0.0) || (out > 0.0 && To != 0.0)))
	{
		result.error = "a cubic parabola runs between a straight and an arc only";
		return result;
	}
	// how far the transitions turn: a clothoid and a Bloss curve by the mean of their curvatures, a cubic parabola by its end slope
	auto const turning = [&](double const Length, double const Other) {
		if (Length <= 0.0)
			return 0.0;
		return parabola ? side * std::atan(Length / (2.0 * Radius)) : 0.5 * (Other + curvature) * Length;
	};
	auto const arcangle{Deflection - turning(in, From) - turning(out, To)};
	auto arc{arcangle / curvature};
	if (arc < -1e-9)
	{
		result.error = std::format("the transition curves turn {:.4f} rad, more than the {:.4f} rad the tangents do", std::abs(Deflection - arcangle), std::abs(Deflection));
		return result;
	}
	arc = std::max(0.0, arc);
	result.arc = arc;
	glm::dvec2 origin{0.0};
	glm::dvec2 direction{1.0, 0.0};
	if (in > 0.0)
		chain(result.elements, parabola ? make_parabola({}, {1.0, 0.0}, in, side * Radius, true) : make_spiral(kind, {}, {1.0, 0.0}, From, curvature, in), origin, direction);
	if (arc > kSame)
		chain(result.elements, make_arc({}, {1.0, 0.0}, curvature, arc), origin, direction);
	if (out > 0.0)
		chain(result.elements, parabola ? make_parabola({}, {1.0, 0.0}, out, side * Radius, false) : make_spiral(kind, {}, {1.0, 0.0}, curvature, To, out), origin, direction);
	for (auto &element : result.elements)
	{
		element.role = &element == &result.elements.front() && in > 0.0 ? element_role::transition_in : &element == &result.elements.back() && out > 0.0 ? element_role::transition_out : element_role::arc;
	}
	// the tangent points: where the tangent at the start and the one at the end cross
	glm::dvec2 const leaving{std::cos(Deflection), std::sin(Deflection)};
	double first{0.0};
	double second{0.0};
	if (false == geometry::lines_cross({0.0, 0.0}, {1.0, 0.0}, origin, leaving, first, second))
	{
		result.error = "the curve ends heading as it starts";
		return result;
	}
	result.tangent_in = first;
	result.tangent_out = -second;
	return result;
}

std::size_t solved_line::element_at(double const Chainage) const
{
	if (elements.empty())
		return 0;
	auto const next{std::upper_bound(elements.begin(), elements.end(), Chainage, [](double const Value, element const &Element) { return Value < Element.chainage; })};
	return next == elements.begin() ? 0 : static_cast<std::size_t>(std::prev(next) - elements.begin());
}

glm::dvec2 solved_line::position(double const Chainage) const
{
	auto const &e{elements[element_at(Chainage)]};
	return e.position(Chainage - e.chainage);
}

glm::dvec2 solved_line::tangent(double const Chainage) const
{
	auto const &e{elements[element_at(Chainage)]};
	return e.tangent(Chainage - e.chainage);
}

double solved_line::curvature(double const Chainage) const
{
	auto const &e{elements[element_at(Chainage)]};
	return e.curvature(Chainage - e.chainage);
}

double solved_line::cant_at(double const Chainage) const
{
	if (cant.empty())
		return 0.0;
	auto const next{std::upper_bound(cant.begin(), cant.end(), Chainage, [](double const Value, cant_stretch const &Stretch) { return Value < Stretch.from; })};
	auto const &stretch{next == cant.begin() ? cant.front() : *std::prev(next)};
	auto const span{stretch.to - stretch.from};
	auto const t{span > 0.0 ? std::clamp((Chainage - stretch.from) / span, 0.0, 1.0) : 1.0};
	auto const share{stretch.shape == ramp_shape::bloss ? bloss_share(t) : t};
	return stretch.start + (stretch.end - stretch.start) * share;
}

double solved_line::elevation(double const Chainage) const
{
	if (profile.size() < 2)
		return profile.empty() ? 0.0 : profile.front().elevation;
	auto const s{std::clamp(Chainage, 0.0, length)};
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
		auto const before{(point.elevation - profile[i - 1].elevation) / (point.chainage - profile[i - 1].chainage)};
		auto const after{(profile[i + 1].elevation - point.elevation) / (profile[i + 1].chainage - point.chainage)};
		auto const offset{(point.tangent - distance) * (point.tangent - distance) / (2.0 * point.vertical_radius)};
		height += after > before ? offset : -offset;
	}
	return height;
}

double solved_line::grade(double const Chainage) const
{
	if (profile.size() < 2)
		return 0.0;
	auto const s{std::clamp(Chainage, 0.0, length)};
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
		auto const before{(point.elevation - profile[i - 1].elevation) / (point.chainage - profile[i - 1].chainage)};
		auto const after{(profile[i + 1].elevation - point.elevation) / (profile[i + 1].chainage - point.chainage)};
		auto const change{(point.tangent - std::abs(distance)) / point.vertical_radius * (distance < 0.0 ? 1.0 : -1.0)};
		slope += after > before ? change : -change;
	}
	return slope;
}

glm::dvec3 solved_line::point(double const Chainage) const
{
	auto const plan{position(Chainage)};
	return {plan.x, elevation(Chainage), plan.y};
}

solved_line solve(line_shape const &Shape, end_condition const &Start, end_condition const &End)
{
	solved_line result;
	auto const &vertices{Shape.vertices};
	auto const count{vertices.size()};
	std::vector<glm::dvec2> polygon{geometry::plan_of(Start.position)};
	for (auto const &vertex : vertices)
		polygon.push_back(vertex.point);
	polygon.push_back(geometry::plan_of(End.position));
	auto const vertex_name = [&](std::size_t const Index) { return std::format("vertex {}", number(vertices[Index].id)); };

	std::vector<glm::dvec2> directions;
	std::vector<double> lengths;
	for (std::size_t i = 0; i + 1 < polygon.size(); ++i)
	{
		auto const length{glm::distance(polygon[i], polygon[i + 1])};
		if (length < 1e-3)
		{
			result.errors.push_back(i == 0 ? std::format("{} lies on the start of the line", count > 0 ? vertex_name(0) : "the end")
			                                : std::format("{} lies on {}", vertex_name(i - 1), i < count ? vertex_name(i) : "the end of the line"));
			return result;
		}
		lengths.push_back(length);
		directions.push_back((polygon[i + 1] - polygon[i]) / length);
	}
	if (count == 0 && (Start.continues || End.continues) && (Start.curvature != 0.0 || End.curvature != 0.0 || Start.cant != 0.0 || End.cant != 0.0))
	{
		result.errors.emplace_back("a line going on from a curve at its end needs a vertex for the curve");
		return result;
	}

	// the curves of the vertices, each in the frame of its first tangent point
	std::vector<curve_fit> curves(count);
	std::vector<double> cants(count);
	for (std::size_t k = 0; k < count; ++k)
	{
		auto const &vertex{vertices[k]};
		auto const deflection{geometry::signed_angle(directions[k], directions[k + 1])};
		if (std::abs(deflection) > kPi - 1e-6)
		{
			result.errors.push_back(std::format("the line turns back at {}", vertex_name(k)));
			return result;
		}
		if (vertex.cant < 0.0)
		{
			result.errors.push_back(std::format("{}: a cant leaning out of the turn", vertex_name(k)));
			return result;
		}
		auto const from{k == 0 && Start.continues ? Start.curvature : 0.0};
		auto const to{k + 1 == count && End.continues ? End.curvature : 0.0};
		curves[k] = fit_curve(Shape.transition, deflection, vertex.radius, vertex.transition_in, vertex.transition_out, from, to);
		if (false == curves[k].error.empty())
		{
			result.errors.push_back(std::format("{}: {}", vertex_name(k), curves[k].error));
			return result;
		}
		if (curves[k].elements.empty() && vertex.cant > 0.0)
		{
			result.errors.push_back(std::format("{}: a cant where the line doesn't turn", vertex_name(k)));
			return result;
		}
		cants[k] = sign_of(deflection) * vertex.cant;
	}

	// the straights between the curves: what the tangents leave of the sides of the polygon
	std::vector<double> straights(count + 1);
	for (std::size_t i = 0; i <= count; ++i)
	{
		auto const taken{(i > 0 ? curves[i - 1].tangent_out : 0.0) + (i < count ? curves[i].tangent_in : 0.0)};
		straights[i] = lengths[i] - taken;
		if (straights[i] < -kSame)
		{
			if (i == 0)
				result.errors.push_back(std::format("{}: the curve starts {:.3f} m before the start of the line", vertex_name(0), -straights[i]));
			else if (i == count)
				result.errors.push_back(std::format("{}: the curve ends {:.3f} m past the end of the line", vertex_name(count - 1), -straights[i]));
			else
				result.errors.push_back(std::format("the curves of {} and {} overlap by {:.3f} m", vertex_name(i - 1), vertex_name(i), -straights[i]));
			return result;
		}
		auto const held{(i == 0 && Start.continues) || (i == count && End.continues)};
		if (held && straights[i] > kSame)
		{
			result.errors.push_back(std::format("{}: the curve has to {} right at the node, where the track goes on from a curve; it's {:.3f} m off", vertex_name(i == 0 ? 0 : count - 1),
			                                    i == 0 ? "begin" : "end", straights[i]));
			return result;
		}
	}

	// the elements one after another, each placed from the points of the polygon: the straights along its sides, the curves from their
	// tangent points, so nothing drifts along the line
	double chainage{0.0};
	auto const push = [&](element Element) {
		Element.chainage = chainage;
		chainage += Element.length;
		result.elements.push_back(Element);
	};
	for (std::size_t i = 0; i <= count; ++i)
	{
		if (straights[i] > kSame)
		{
			auto const from{i == 0 ? polygon[0] : polygon[i] + directions[i] * curves[i - 1].tangent_out};
			auto straight{make_straight(from, directions[i], straights[i])};
			straight.vertex = i < count ? number(vertices[i].id) : 0;
			push(straight);
		}
		if (i == count)
			break;
		// a curve beginning at the start node begins there to the bit
		auto const tangent_point{i == 0 && straights[0] <= kSame ? polygon[0] : polygon[i + 1] - directions[i] * curves[i].tangent_in};
		for (auto const &local : curves[i].elements)
		{
			auto placed{in_frame(local, tangent_point, directions[i])};
			placed.vertex = number(vertices[i].id);
			push(placed);
		}
	}
	result.length = chainage;
	if (result.elements.empty())
	{
		result.errors.emplace_back("the line has no length");
		return result;
	}

	// the cant: held along the arcs, ramped along the transitions, 0 along the straights; a change with no transition to ramp it along
	// is a step, which isn't allowed
	auto cant{Start.continues ? Start.cant : 0.0};
	auto const stretch = [&](double const From, double const To, double const Begin, double const Finish, ramp_shape const Ramp) {
		if (To - From > 0.0)
			result.cant.push_back({From, To, Begin, Finish, Ramp});
	};
	std::size_t curve{0};
	for (std::size_t i = 0; i < result.elements.size(); ++i)
	{
		auto const &e{result.elements[i]};
		if (e.role == element_role::straight)
		{
			if (std::abs(cant) > kCantStep)
			{
				result.errors.push_back(std::format("a cant of {:.0f} mm on the straight at {:.1f} m", std::abs(cant), e.chainage));
				return result;
			}
			stretch(e.chainage, e.end_chainage(), 0.0, 0.0, ramp_shape::linear);
			cant = 0.0;
			continue;
		}
		while (curve + 1 < count && number(vertices[curve].id) != e.vertex)
			++curve;
		auto const target{cants[curve]};
		auto const after{curve + 1 == count && End.continues ? End.cant : 0.0};
		switch (e.role)
		{
		case element_role::transition_in:
			stretch(e.chainage, e.end_chainage(), cant, target, Shape.ramp);
			cant = target;
			break;
		case element_role::arc:
			if (std::abs(cant - target) > kCantStep)
			{
				result.errors.push_back(std::format("{}: the cant steps by {:.0f} mm where the arc begins, with no transition curve to ramp it along", vertex_name(curve), std::abs(cant - target)));
				return result;
			}
			stretch(e.chainage, e.end_chainage(), target, target, ramp_shape::linear);
			cant = target;
			break;
		case element_role::transition_out:
			if (std::abs(cant - target) > kCantStep)
			{
				result.errors.push_back(std::format("{}: the cant steps by {:.0f} mm in the curve", vertex_name(curve), std::abs(cant - target)));
				return result;
			}
			stretch(e.chainage, e.end_chainage(), target, after, Shape.ramp);
			cant = after;
			break;
		default: break;
		}
		// the last element of a curve with no transition out: what follows starts at the cant the curve leaves
		auto const last_of_curve{i + 1 == result.elements.size() || result.elements[i + 1].vertex != e.vertex || result.elements[i + 1].role == element_role::straight};
		if (last_of_curve && std::abs(cant - after) > kCantStep)
		{
			result.errors.push_back(std::format("{}: the cant steps by {:.0f} mm where the curve ends, with no transition curve to ramp it along", vertex_name(curve), std::abs(cant - after)));
			return result;
		}
		if (last_of_curve)
			cant = after;
	}
	auto const end_cant{End.continues ? End.cant : 0.0};
	if (std::abs(cant - end_cant) > kCantStep)
	{
		result.errors.push_back(std::format("the cant steps by {:.0f} mm at the end of the line", std::abs(cant - end_cant)));
		return result;
	}

	// the grade line: the heights of the nodes at the ends, the breaks between them rounded with their vertical curves
	result.profile.push_back({0.0, Start.position.y, 0.0, 0.0});
	for (auto const &point : Shape.grades)
	{
		if (point.chainage <= result.profile.back().chainage + 1e-3 || point.chainage >= result.length - 1e-3)
		{
			result.errors.push_back(std::format("the grade break at {:.1f} m isn't between the one before it and the end of the line, {:.1f} m long", point.chainage, result.length));
			return result;
		}
		if (point.vertical_radius <= 0.0)
		{
			result.errors.push_back(std::format("the grade break at {:.1f} m has no vertical radius", point.chainage));
			return result;
		}
		result.profile.push_back({point.chainage, point.elevation, point.vertical_radius, 0.0});
	}
	result.profile.push_back({result.length, End.position.y, 0.0, 0.0});
	for (std::size_t i = 1; i + 1 < result.profile.size(); ++i)
	{
		auto &point{result.profile[i]};
		auto const before{(point.elevation - result.profile[i - 1].elevation) / (point.chainage - result.profile[i - 1].chainage)};
		auto const after{(result.profile[i + 1].elevation - point.elevation) / (result.profile[i + 1].chainage - point.chainage)};
		point.tangent = point.vertical_radius * std::abs(after - before) * 0.5;
	}
	for (std::size_t i = 0; i + 1 < result.profile.size(); ++i)
	{
		auto const &a{result.profile[i]};
		auto const &b{result.profile[i + 1]};
		if (a.tangent + b.tangent > b.chainage - a.chainage + 1e-9)
		{
			result.errors.push_back(std::format("the vertical curves at {:.1f} m and {:.1f} m overlap by {:.2f} m", a.chainage, b.chainage, a.tangent + b.tangent - (b.chainage - a.chainage)));
			return result;
		}
	}

	auto const &first{result.elements.front()};
	auto const &last{result.elements.back()};
	result.start = {Start.position, directions.front(), first.curvature(0.0), result.cant_at(0.0), result.grade(0.0)};
	result.end = {End.position, directions.back(), last.curvature(last.length), result.cant.empty() ? 0.0 : result.cant.back().end, result.grade(result.length)};
	return result;
}

norms::line described(solved_line const &Line, norms::line_class const &Class, std::string Name)
{
	norms::line result;
	result.name = std::move(Name);
	for (auto const &e : Line.elements)
	{
		norms::element element;
		element.kind = e.kind == element_kind::straight ? norms::element_kind::straight : e.kind == element_kind::arc ? norms::element_kind::arc : norms::element_kind::transition;
		element.start = e.chainage;
		element.length = e.length;
		element.curvature_start = e.curvature(0.0);
		element.curvature_end = e.curvature(e.length);
		element.speed = Class.vmax;
		auto const from{e.position(0.0)};
		auto const to{e.position(e.length)};
		element.from = {from.x, from.y};
		element.to = {to.x, to.y};
		result.elements.push_back(element);
	}
	auto const add = [&](double const Chainage, double const Cant) {
		if (false == result.cant.empty() && std::abs(result.cant.back().chainage - Chainage) < 1e-9 && std::abs(result.cant.back().cant - Cant) < 1e-9)
			return;
		result.cant.push_back({Chainage, Cant});
	};
	for (auto const &stretch : Line.cant)
	{
		// a Bloss ramp as eight even pieces, so its steeper middle is seen
		auto const pieces{stretch.shape == ramp_shape::bloss && stretch.start != stretch.end ? 8 : 1};
		for (int i = 0; i <= pieces; ++i)
		{
			auto const t{static_cast<double>(i) / pieces};
			auto const share{stretch.shape == ramp_shape::bloss ? bloss_share(t) : t};
			add(stretch.from + (stretch.to - stretch.from) * t, stretch.start + (stretch.end - stretch.start) * share);
		}
	}
	auto const &profile{Line.profile};
	for (std::size_t i = 0; i + 1 < profile.size(); ++i)
	{
		auto const &a{profile[i]};
		auto const &b{profile[i + 1]};
		auto const grade{(b.elevation - a.elevation) / (b.chainage - a.chainage)};
		if (i > 0 && a.tangent > 0.0)
		{
			auto const before{(a.elevation - profile[i - 1].elevation) / (a.chainage - profile[i - 1].chainage)};
			result.grades.push_back({a.chainage - a.tangent, 2.0 * a.tangent, before, grade});
		}
		auto const from{a.chainage + a.tangent};
		auto const to{b.chainage - b.tangent};
		if (to - from > 1e-9)
			result.grades.push_back({from, to - from, grade, grade});
	}
	return result;
}

curve_norms preferred_curve(norms::profile_id const Profile, norms::line_class const &Class, double const Radius)
{
	curve_norms result;
	norms::query where;
	where.speed = Class.vmax;
	where.radius = Radius;
	where.load = Class.load;
	where.kind = Class.kind;
	where.track = Class.track;
	where.features = Class.features;
	auto const limit = [&](norms::parameter const What) { return tier_value(norms::limits(Profile, What, where)); };
	auto const speed{Class.vmax};
	auto const slow{Class.vmin > 0.0 ? Class.vmin : norms::slow_speed(Class.kind, Class.vmax)};
	// the cant within the deficiency, the excess and the greatest cant of the preferred values, else within the ones which only warn,
	// else the most the cant may be: the regular cant where it's among them, rounded to 5 mm
	auto const deficiency_limits{norms::limits(Profile, norms::parameter::cant_deficiency, where)};
	auto const excess_limits{norms::limits(Profile, norms::parameter::cant_excess, where)};
	auto const cant_limits{norms::limits(Profile, norms::parameter::cant, where)};
	auto const regular{norms::regular_cant(speed, Radius)};
	double cant{0.0};
	bool found{false};
	for (auto const tier : {&norms::thresholds::prefer, &norms::thresholds::warn, &norms::thresholds::block})
	{
		auto low{0.0};
		auto high{1e9};
		if (auto const &bound{deficiency_limits.*tier})
			low = std::max(low, norms::least_cant(speed, Radius, bound->value));
		if (auto const &bound{excess_limits.*tier})
			high = std::min(high, norms::greatest_cant(slow, Radius, bound->value));
		if (auto const &bound{cant_limits.*tier})
			high = std::min(high, bound->value);
		if (low > high)
		{
			cant = high;
			continue;
		}
		cant = std::clamp(regular, low, high);
		auto const rounded{std::round(cant / 5.0) * 5.0};
		cant = rounded > high ? std::floor(high / 5.0) * 5.0 : rounded < low ? std::ceil(low / 5.0) * 5.0 : rounded;
		cant = std::clamp(cant, low, high);
		found = true;
		break;
	}
	cant = std::max(0.0, found ? cant : std::floor(cant / 5.0) * 5.0);
	// a cant given at all is at least the least one: that, or none where the curve does as well without
	if (auto const least{limit(norms::parameter::cant_min)}; least && cant > 0.0 && cant < *least)
	{
		auto const needed{deficiency_limits.prefer ? norms::least_cant(speed, Radius, deficiency_limits.prefer->value) : 0.0};
		cant = needed > 0.0 || cant >= 0.5 * *least ? std::ceil(*least / 5.0) * 5.0 : 0.0;
	}
	result.cant = cant;
	auto const metres{speed / 3.6};
	auto const deficiency{std::max(0.0, norms::equilibrium_cant(speed, Radius) - cant)};
	double transition{0.0};
	if (auto const gradient{limit(norms::parameter::cant_gradient)}; gradient && *gradient > 0.0)
		transition = std::max(transition, cant / *gradient);
	if (auto const rate{limit(norms::parameter::cant_rate)}; rate && *rate > 0.0)
		transition = std::max(transition, metres * cant / *rate);
	if (auto const rate{limit(norms::parameter::deficiency_rate)}; rate && *rate > 0.0)
		transition = std::max(transition, metres * deficiency / *rate);
	if (auto const shortest{limit(norms::parameter::transition_length)}; shortest && cant > 0.0)
		transition = std::max(transition, *shortest);
	result.transition = std::ceil(transition - 1e-9);
	result.radius_prefer = limit(norms::parameter::radius).value_or(0.0);
	auto const block{norms::limits(Profile, norms::parameter::radius, where).block};
	result.radius_block = block ? block->value : 0.0;
	return result;
}

double roll_of(double const Cant)
{
	return glm::degrees(std::asin(std::clamp(Cant / kRailBase, -1.0, 1.0)));
}

double chainage_of(solved_line const &Line, glm::dvec2 const &Point, double *Distance)
{
	if (Distance != nullptr)
		*Distance = std::numeric_limits<double>::infinity();
	if (Line.elements.empty() || Line.length <= 0.0)
		return 0.0;
	// the nearest of points a metre apart, then narrowed down within the metres either side of it: the line has no bend sharper than that
	auto const off = [&](double const Chainage) { return glm::distance(Line.position(Chainage), Point); };
	auto const steps{static_cast<int>(std::ceil(Line.length))};
	double best{0.0};
	double nearest{off(0.0)};
	for (int i = 1; i <= steps; ++i)
	{
		auto const at{std::min(Line.length, static_cast<double>(i))};
		if (auto const d{off(at)}; d < nearest)
		{
			nearest = d;
			best = at;
		}
	}
	auto low{std::max(0.0, best - 1.0)};
	auto high{std::min(Line.length, best + 1.0)};
	// golden section
	double constexpr kRatio{0.6180339887498949};
	auto a{high - kRatio * (high - low)};
	auto b{low + kRatio * (high - low)};
	auto fa{off(a)};
	auto fb{off(b)};
	for (int i = 0; i < 80 && high - low > 1e-9; ++i)
	{
		if (fa < fb)
		{
			high = b;
			b = a;
			fb = fa;
			a = high - kRatio * (high - low);
			fa = off(a);
		}
		else
		{
			low = a;
			a = b;
			fa = fb;
			b = low + kRatio * (high - low);
			fb = off(b);
		}
	}
	auto const found{(low + high) * 0.5};
	if (off(found) > nearest)
	{
		if (Distance != nullptr)
			*Distance = nearest;
		return best;
	}
	if (Distance != nullptr)
		*Distance = off(found);
	return found;
}

} // namespace layout
