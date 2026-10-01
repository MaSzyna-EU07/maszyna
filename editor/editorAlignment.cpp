/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorAlignment.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>

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

// vector rotated by 90 degrees counter-clockwise
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

// point of a transition curve in its local frame: x along the tangent at its start, y towards the curve centre
struct local_point
{
	double x{0.0};
	double y{0.0};
	double angle{0.0}; // tangent angle relative to the initial tangent
};

// Parameter is the fraction of the transition length, from the straight (0) to the circular curve (1)
local_point transition_point(transition_shape const Shape, double const Length, double const Radius, double const Parameter)
{
	if (Length <= 0.0 || Radius <= 0.0)
		return {};
	if (Shape == transition_shape::cubic_parabola)
	{
		auto const x{Length * Parameter};
		return {x, x * x * x / (6.0 * Radius * Length), std::atan(x * x / (2.0 * Radius * Length))};
	}
	// clothoid, tangent angle grows with the square of the travelled distance
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

// plan position and direction of travel at given distance from the start of the element
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
	default: return 0.0;
	}
}

element const &element_at(result const &Result, double const Chainage)
{
	auto const lookup = std::upper_bound(Result.elements.begin(), Result.elements.end(), Chainage, [](double const Value, element const &Element) { return Value < Element.chainage; });
	return lookup == Result.elements.begin() ? Result.elements.front() : *std::prev(lookup);
}

// polar form of a cubic Bezier curve; control points of the part between parameters a and b are f(a,a,a), f(a,a,b), f(a,b,b), f(b,b,b)
glm::dvec2 blossom(std::array<glm::dvec2, 4> const &Points, double const A, double const B, double const C)
{
	glm::dvec2 r[3], s[2];
	for (int i = 0; i < 3; ++i)
		r[i] = glm::mix(Points[i], Points[i + 1], A);
	for (int i = 0; i < 2; ++i)
		s[i] = glm::mix(r[i], r[i + 1], B);
	return glm::mix(s[0], s[1], C);
}

// number of cubic pieces needed to represent the element accurately
std::size_t element_pieces(element const &Element, transition_shape const Shape)
{
	if (Element.length < 1e-3)
		return 0;
	switch (Element.kind)
	{
	case element_kind::arc:
		// cubic approximation of a 10 degree arc deviates from the circle by less than 1e-7 of its radius
		return static_cast<std::size_t>(std::max(1.0, std::ceil(Element.length / Element.radius / (10.0 * kPi / 180.0) - 1e-9)));
	case element_kind::transition_in:
	case element_kind::transition_out:
		return Shape == transition_shape::cubic_parabola ? 1 : static_cast<std::size_t>(std::max(1.0, std::ceil(Element.length / 20.0 - 1e-9)));
	default:
		return 1;
	}
}

} // namespace

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
		// rate of cant change in time, and the inclination of the cant ramp
		length = std::max(length, Speed * Cant / (3.6 * Limits.cant_rate));
		length = std::max(length, Cant / 1000.0 * Limits.ramp_factor * Speed);
	}
	// rate of change of the unbalanced acceleration
	auto const unbalanced{std::max(0.0, unbalanced_acceleration(Speed, Radius, Cant, Limits))};
	length = std::max(length, unbalanced * Speed / (3.6 * Limits.jerk));
	return length > 0.0 ? ceil_to(length, 5.0) : 0.0;
}

recommendation recommend(double const Speed, double const Radius, limits const &Limits)
{
	recommendation result;
	auto const factor{Limits.gauge / (kGravity * 12.96)}; // ~11.8 for standard gauge
	auto const allowance{Limits.gauge * Limits.unbalanced / kGravity};
	result.radius_min = factor * Speed * Speed / (Limits.cant_max + allowance);
	result.vertical_radius = std::max(Limits.vertical_min, Limits.vertical_factor * Speed * Speed);
	if (Radius <= 0.0)
		return result;
	result.cant_equilibrium = factor * Speed * Speed / Radius;
	result.cant_min = std::max(0.0, result.cant_equilibrium - allowance);
	// keep a part of the permissible unbalanced acceleration in reserve, but never go below the required minimum
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

	// resolve vertex positions; the first and the last vertex lie on the tangents of the fixed ends
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
			if (t <= 0.0 || u <= 0.0)
				r.errors.emplace_back("Tangents of the ends intersect outside of the fragment, use two vertices");
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

	std::vector<double> vertexchainage(count, 0.0);
	glm::dvec2 cursor{start};
	for (std::size_t k = 0; k < count; ++k)
	{
		auto const &vertex{Design.vertices[k]};
		auto const &position{polygon[k + 1]};
		auto const directionin{glm::normalize(polygon[k + 1] - polygon[k])};
		auto const directionout{glm::normalize(polygon[k + 2] - polygon[k + 1])};
		auto const turn{cross(directionin, directionout)};
		auto const deflection{std::atan2(std::abs(turn), glm::dot(directionin, directionout))};

		auto const lead{glm::dot(position - cursor, directionin)};
		if (deflection < 1e-7)
		{
			// tangents are collinear, there's no curve to fit
			if (lead < -1e-4)
				r.errors.emplace_back(format("Vertex %zu lies inside the curve of the previous vertex", k + 1));
			push_straight(cursor, directionin, lead);
			vertexchainage[k] = chainage;
			cursor = position;
			continue;
		}
		if (deflection > kPi - 1e-3)
		{
			r.errors.emplace_back(format("Vertex %zu: the route turns back on itself", k + 1));
			return r;
		}
		if (vertex.radius < 1.0)
		{
			r.errors.emplace_back(format("Vertex %zu: radius is too small", k + 1));
			return r;
		}

		int const side{turn > 0.0 ? 1 : -1};
		auto const radius{vertex.radius};
		auto const lengthin{std::max(0.0, vertex.transition_in)};
		auto const lengthout{std::max(0.0, vertex.transition_out)};
		auto const endin{transition_point(Design.shape, lengthin, radius, 1.0)};
		auto const endout{transition_point(Design.shape, lengthout, radius, 1.0)};
		// shift of the circular curve towards its centre, and position of the shifted curve start along the tangent
		auto const shiftin{endin.y - radius * (1.0 - std::cos(endin.angle))};
		auto const shiftout{endout.y - radius * (1.0 - std::cos(endout.angle))};
		auto const middlein{endin.x - radius * std::sin(endin.angle)};
		auto const middleout{endout.x - radius * std::sin(endout.angle)};

		// the centre lies at distances R + shift from both tangents
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

		// verification against the limits
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

	// straights between curves, measured as travel time
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

	// vertical profile: grades between the fixed ends and the vertex elevations, joined with vertical curves
	r.vertex_chainages = vertexchainage;
	r.profile.push_back({0.0, Design.start.y, 0.0, 0.0});
	for (std::size_t k = 0; k < count; ++k)
	{
		auto const &vertex{Design.vertices[k]};
		auto const elevation{vertex.auto_elevation ? Design.start.y + (Design.end.y - Design.start.y) * vertexchainage[k] / std::max(r.length, 1e-6) : vertex.elevation};
		r.vertex_elevations.push_back(elevation);
		auto const radius{vertex.vertical_radius > 0.0 ? vertex.vertical_radius : std::max(Design.norms.vertical_min, Design.norms.vertical_factor * Design.speed * Design.speed)};
		if (vertexchainage[k] <= r.profile.back().chainage + 1e-3)
			continue; // vertex without its own place in the profile
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
	// curvature continuity with the adjoining tracks; the designed fragment starts and ends on a tangent
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
		count += element_pieces(e, Design.shape);
	return count;
}

std::vector<segment_data> pieces(result const &Result, design const &Design, std::size_t const Count)
{
	struct span
	{
		std::size_t element;
		double from, to;
	};
	std::vector<span> spans;
	for (std::size_t i = 0; i < Result.elements.size(); ++i)
	{
		auto const &e{Result.elements[i]};
		auto const count{element_pieces(e, Design.shape)};
		for (std::size_t j = 0; j < count; ++j)
			spans.push_back({i, e.length * j / count, e.length * (j + 1) / count});
	}
	// use up the requested number of pieces by dividing the longest ones
	while (false == spans.empty() && spans.size() < Count)
	{
		auto longest = std::max_element(spans.begin(), spans.end(), [](span const &A, span const &B) { return A.to - A.from < B.to - B.from; });
		auto const middle{(longest->from + longest->to) * 0.5};
		span const second{longest->element, middle, longest->to};
		longest->to = middle;
		spans.insert(std::next(longest), second);
	}

	std::vector<segment_data> result;
	for (auto const &piece : spans)
	{
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
			if (Design.shape == transition_shape::cubic_parabola)
			{
				// exact: part of the cubic which describes the whole transition
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
		// heights follow the profile, with end slopes matching its grade
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
		result.push_back(path);
	}
	return result;
}

} // namespace alignment
