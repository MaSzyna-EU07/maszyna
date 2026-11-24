/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutElements.hpp"

#include <algorithm>
#include <cmath>

namespace layout
{

namespace
{

double constexpr kPi{3.14159265358979323846};

// the heading turned along a curve integrated into the point of its frame, by Simpson's rule over steps of about a quarter of a metre,
// 64 at most: a transition of a radian stays within a millimetre however long, a usual one within a micrometre
template <typename Heading> local_point integrated(double const Distance, Heading const &Angle)
{
	int const steps{std::clamp(static_cast<int>(std::ceil(Distance / 0.5)) * 2, 8, 64)};
	double const h{Distance / steps};
	double x{0.0};
	double y{0.0};
	for (int i = 0; i <= steps; ++i)
	{
		double const weight = (i == 0 || i == steps) ? 1.0 : (i % 2 ? 4.0 : 2.0);
		auto const a{Angle(i * h)};
		x += weight * std::cos(a);
		y += weight * std::sin(a);
	}
	return {x * h / 3.0, y * h / 3.0, Angle(Distance)};
}

double sign_of(double const Value)
{
	return Value < 0.0 ? -1.0 : 1.0;
}

glm::dvec2 rotated(glm::dvec2 const &Vector, double const Angle)
{
	auto const c{std::cos(Angle)};
	auto const s{std::sin(Angle)};
	return {Vector.x * c - Vector.y * s, Vector.x * s + Vector.y * c};
}

// the cubic parabola of the abscissa X and the radius: the slope and the curvature at the abscissa, the radius taken as it is
double parabola_slope(double const X, double const Radius, double const At)
{
	return At * At / (2.0 * Radius * X);
}

double parabola_curvature(double const X, double const Radius, double const At)
{
	auto const slope{parabola_slope(X, Radius, At)};
	return At / (Radius * X) / std::pow(1.0 + slope * slope, 1.5);
}

// the abscissa of the cubic parabola at the length along it
double parabola_abscissa(double const X, double const Radius, double const Length)
{
	auto x{std::min(Length, X)};
	for (int i = 0; i < 40; ++i)
	{
		auto const slope{parabola_slope(X, Radius, x)};
		auto const step{(parabola_length(X, Radius, x) - Length) / std::sqrt(1.0 + slope * slope)};
		x = std::clamp(x - step, 0.0, X);
		if (std::abs(step) < 1e-12)
			break;
	}
	return x;
}

// the cubic parabola from the tangent, turning to the side of the radius, at the length along it
local_point parabola_entering(double const X, double const Radius, double const Length)
{
	auto const side{sign_of(Radius)};
	auto const x{parabola_abscissa(X, std::abs(Radius), Length)};
	auto const point{parabola_point(X, std::abs(Radius), X > 0.0 ? x / X : 0.0)};
	return {point.x, side * point.y, side * point.angle};
}

} // namespace

local_point parabola_point(double const Length, double const Radius, double const Parameter)
{
	if (Length <= 0.0 || Radius <= 0.0)
		return {};
	auto const x{Length * Parameter};
	return {x, x * x * x / (6.0 * Radius * Length), std::atan(x * x / (2.0 * Radius * Length))};
}

local_point clothoid_point(double const Length, double const Radius, double const Parameter)
{
	if (Length <= 0.0 || Radius <= 0.0)
		return {};
	auto const distance{Length * Parameter};
	auto const angle = [&](double const S) { return S * S / (2.0 * Radius * Length); };
	return integrated(distance, angle);
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
	return integrated(Distance, heading);
}

double bloss_share(double const T)
{
	return T * T * (3.0 - 2.0 * T);
}

local_point bloss_point(double const Start, double const End, double const Length, double const Distance)
{
	if (Distance <= 0.0)
		return {};
	if (Start == End)
		return spiral_point(Start, End, Length, Distance);
	auto const length{std::max(Length, 1e-9)};
	// the heading is the integral of the curvature: Start s + (End - Start) L (t^3 - t^4 / 2)
	auto const heading = [&](double const S) {
		auto const t{S / length};
		return Start * S + (End - Start) * length * (t * t * t - 0.5 * t * t * t * t);
	};
	return integrated(Distance, heading);
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

double parabola_length(double const X, double const Radius, double const Upto)
{
	if (Upto <= 0.0 || X <= 0.0 || Radius == 0.0)
		return std::max(0.0, Upto);
	auto const radius{std::abs(Radius)};
	int constexpr steps{64};
	double const h{Upto / steps};
	double length{0.0};
	for (int i = 0; i <= steps; ++i)
	{
		double const weight = (i == 0 || i == steps) ? 1.0 : (i % 2 ? 4.0 : 2.0);
		auto const slope{parabola_slope(X, radius, i * h)};
		length += weight * std::sqrt(1.0 + slope * slope);
	}
	return length * h / 3.0;
}

double element::curvature(double const Distance) const
{
	auto const t{length > 0.0 ? std::clamp(Distance / length, 0.0, 1.0) : 1.0};
	switch (kind)
	{
	case element_kind::straight: return 0.0;
	case element_kind::arc: return curvature_start;
	case element_kind::clothoid: return curvature_start + (curvature_end - curvature_start) * t;
	case element_kind::bloss: return curvature_start + (curvature_end - curvature_start) * bloss_share(t);
	case element_kind::cubic_parabola:
	{
		auto const along{entering ? std::clamp(Distance, 0.0, length) : length - std::clamp(Distance, 0.0, length)};
		auto const radius{std::abs(parabola_radius)};
		return sign_of(parabola_radius) * parabola_curvature(parabola_x, radius, parabola_abscissa(parabola_x, radius, along));
	}
	}
	return 0.0;
}

local_point element::local(double const Distance) const
{
	auto const d{std::clamp(Distance, 0.0, length)};
	switch (kind)
	{
	case element_kind::straight: return {d, 0.0, 0.0};
	case element_kind::arc: return spiral_point(curvature_start, curvature_start, length, d);
	case element_kind::clothoid: return spiral_point(curvature_start, curvature_end, length, d);
	case element_kind::bloss: return bloss_point(curvature_start, curvature_end, length, d);
	case element_kind::cubic_parabola:
	{
		if (entering)
			return parabola_entering(parabola_x, parabola_radius, d);
		// out of the curve: the parabola turning the other way, run backwards from its curved end. The frame of the start looks back
		// along it, half a turn from its heading there
		auto const whole{parabola_entering(parabola_x, -parabola_radius, length)};
		auto const here{parabola_entering(parabola_x, -parabola_radius, length - d)};
		auto const turned{rotated(glm::dvec2{here.x - whole.x, here.y - whole.y}, -(whole.angle + kPi))};
		return {turned.x, turned.y, here.angle - whole.angle};
	}
	}
	return {};
}

glm::dvec2 element::position(double const Distance) const
{
	auto const point{local(Distance)};
	return origin + direction * point.x + left_of(direction) * point.y;
}

glm::dvec2 element::tangent(double const Distance) const
{
	return rotated(direction, local(Distance).angle);
}

glm::dvec2 element::offset_point(double const Distance, double const Offset) const
{
	return position(Distance) + left_of(tangent(Distance)) * Offset;
}

double element::shift() const
{
	if (kind == element_kind::cubic_parabola)
	{
		auto const end{parabola_entering(parabola_x, std::abs(parabola_radius), length)};
		auto const radius{std::abs(parabola_radius)};
		return end.y - radius * (1.0 - std::cos(end.angle));
	}
	if (kind != element_kind::clothoid && kind != element_kind::bloss)
		return 0.0;
	if ((curvature_start == 0.0) == (curvature_end == 0.0))
		return 0.0;
	// the one from the tangent of the same curve: the shift is the same at either end
	auto const curved{std::abs(curvature_start == 0.0 ? curvature_end : curvature_start)};
	auto const end{kind == element_kind::clothoid ? spiral_point(0.0, curved, length, length) : bloss_point(0.0, curved, length, length)};
	return end.y - (1.0 - std::cos(end.angle)) / curved;
}

element make_straight(glm::dvec2 const &Origin, glm::dvec2 const &Direction, double const Length)
{
	element result;
	result.kind = element_kind::straight;
	result.origin = Origin;
	result.direction = Direction;
	result.length = Length;
	return result;
}

element make_arc(glm::dvec2 const &Origin, glm::dvec2 const &Direction, double const Curvature, double const Length)
{
	element result;
	result.kind = element_kind::arc;
	result.role = element_role::arc;
	result.origin = Origin;
	result.direction = Direction;
	result.length = Length;
	result.curvature_start = Curvature;
	result.curvature_end = Curvature;
	return result;
}

element make_spiral(element_kind const Kind, glm::dvec2 const &Origin, glm::dvec2 const &Direction, double const Start, double const End, double const Length)
{
	element result;
	result.kind = Kind;
	result.role = std::abs(End) >= std::abs(Start) ? element_role::transition_in : element_role::transition_out;
	result.origin = Origin;
	result.direction = Direction;
	result.length = Length;
	result.curvature_start = Start;
	result.curvature_end = End;
	return result;
}

element make_parabola(glm::dvec2 const &Origin, glm::dvec2 const &Direction, double const X, double const Radius, bool const Entering)
{
	element result;
	result.kind = element_kind::cubic_parabola;
	result.role = Entering ? element_role::transition_in : element_role::transition_out;
	result.origin = Origin;
	result.direction = Direction;
	result.parabola_x = X;
	result.parabola_radius = Radius;
	result.entering = Entering;
	result.length = parabola_length(X, Radius, X);
	auto const end{sign_of(Radius) * parabola_curvature(X, std::abs(Radius), X)};
	result.curvature_start = Entering ? 0.0 : end;
	result.curvature_end = Entering ? end : 0.0;
	return result;
}

} // namespace layout
