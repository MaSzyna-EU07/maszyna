/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorFouling.hpp"
#include "editor/editorGauge.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fouling
{

namespace
{

// m of the polyline on each side of a point its radius is taken over
double constexpr kRadiusSpan{5.0};
// above this radius, m, the polyline is straight
double constexpr kStraight{20000.0};

glm::dvec2 plan(glm::dvec3 const &Point)
{
	return {Point.x, Point.z};
}

double cross(glm::dvec2 const &A, glm::dvec2 const &B)
{
	return A.x * B.y - A.y * B.x;
}

} // namespace

glm::dvec3 nearest_on(std::vector<glm::dvec3> const &Line, glm::dvec3 const &Point, double &Distance, std::size_t &Segment)
{
	Distance = std::numeric_limits<double>::max();
	Segment = 0;
	glm::dvec3 result{Point};
	auto const point{plan(Point)};
	for (std::size_t i = 1; i < Line.size(); ++i)
	{
		auto const a{plan(Line[i - 1])};
		auto const ab{plan(Line[i]) - a};
		auto const length2{glm::dot(ab, ab)};
		auto const t{length2 > 1e-12 ? std::clamp(glm::dot(point - a, ab) / length2, 0.0, 1.0) : 0.0};
		auto const distance{glm::length(point - (a + ab * t))};
		if (distance < Distance)
		{
			Distance = distance;
			Segment = i - 1;
			result = glm::mix(Line[i - 1], Line[i], t);
		}
	}
	return result;
}

double radius_at(std::vector<glm::dvec3> const &Line, std::size_t const Index, glm::dvec3 const &Other, bool &Inner)
{
	Inner = false;
	if (Line.size() < 3 || Index >= Line.size())
		return 0.0;
	// the points kRadiusSpan before and after it, twice that on one side at an end of the line
	auto const reach = [&](bool const Forward, double const Span, std::size_t &To, double &Length) {
		To = Index;
		Length = 0.0;
		while (Length < Span && (Forward ? To + 1 < Line.size() : To > 0))
		{
			auto const next{Forward ? To + 1 : To - 1};
			Length += glm::length(plan(Line[next]) - plan(Line[To]));
			To = next;
		}
	};
	std::size_t low;
	std::size_t high;
	double behind;
	double ahead;
	reach(false, kRadiusSpan, low, behind);
	reach(true, kRadiusSpan, high, ahead);
	if (behind < kRadiusSpan)
		reach(true, 2.0 * kRadiusSpan - behind, high, ahead);
	if (ahead < kRadiusSpan)
		reach(false, 2.0 * kRadiusSpan - ahead, low, behind);
	if (high < low + 2)
		return 0.0;
	// the middle one between them, by the length
	auto const half{0.5 * (behind + ahead)};
	auto middle{low};
	for (double length{0.0}; middle + 1 < high && length < half; ++middle)
		length += glm::length(plan(Line[middle + 1]) - plan(Line[middle]));
	auto const a{plan(Line[low])};
	auto const b{plan(Line[middle])};
	auto const c{plan(Line[high])};
	auto const bulge{cross(c - a, b - a)};
	if (std::abs(bulge) < 1e-9)
		return 0.0;
	auto const radius{glm::length(b - a) * glm::length(c - b) * glm::length(c - a) / (2.0 * std::abs(bulge))};
	if (radius > kStraight)
		return 0.0;
	// the middle point bulges away from the centre: the other point lies on its inner side when it lies across the chord
	Inner = cross(c - a, plan(Other) - a) * bulge < 0.0;
	return radius;
}

double widening_at(std::vector<glm::dvec3> const &Line, std::size_t const Index, glm::dvec3 const &Other)
{
	bool inner{false};
	auto const radius{radius_at(Line, Index, Other, inner)};
	return radius > 0.0 ? gauge::curve_widening(radius, inner) : 0.0;
}

bool find(std::vector<glm::dvec3> const &Main, std::vector<glm::dvec3> const &Branch, double const Nominal, place &Place)
{
	if (Main.size() < 2 || Branch.size() < 2)
		return false;
	// how far the axes are apart at a point of the branch over what they have to be there, below 0 short of the fouling point
	auto const excess = [&](std::size_t const I, double &Wanted) {
		double distance;
		std::size_t segment;
		auto const foot{nearest_on(Main, Branch[I], distance, segment)};
		Wanted = Nominal + widening_at(Branch, I, foot) + widening_at(Main, segment, Branch[I]);
		return distance - Wanted;
	};
	double previouswanted;
	auto previous{excess(0, previouswanted)};
	for (std::size_t i = 1; i < Branch.size(); ++i)
	{
		double wanted;
		auto const now{excess(i, wanted)};
		if (now < 0.0)
		{
			previous = now;
			previouswanted = wanted;
			continue;
		}
		auto const f{previous < 0.0 ? previous / (previous - now) : 1.0};
		Place.branch = glm::mix(Branch[i - 1], Branch[i], f);
		Place.spacing = previouswanted + (wanted - previouswanted) * f;
		double distance;
		std::size_t segment;
		Place.main = nearest_on(Main, Place.branch, distance, segment);
		auto const back{plan(Branch[i - 1] - Branch[i])};
		Place.back = glm::length(back) > 1e-9 ? glm::normalize(back) : glm::dvec2{0.0, 1.0};
		return true;
	}
	return false;
}

} // namespace fouling
