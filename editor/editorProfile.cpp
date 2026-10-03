/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorProfile.hpp"
#include "editor/editorFormat.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>

namespace profile
{

namespace
{

double polyline(line const &Line, double const Chainage)
{
	auto const &points{Line.points};
	if (points.empty())
		return 0.0;
	if (points.size() == 1 || Chainage <= points.front().chainage)
		return points.front().elevation + (points.size() > 1 ? grade_after(Line, 0) * (Chainage - points.front().chainage) : 0.0);
	if (Chainage >= points.back().chainage)
		return points.back().elevation + grade_after(Line, points.size() - 2) * (Chainage - points.back().chainage);
	auto const next{std::upper_bound(points.begin(), points.end(), Chainage, [](double const Value, pvi const &Point) { return Value < Point.chainage; })};
	auto const index{static_cast<std::size_t>(std::distance(points.begin(), next)) - 1};
	return points[index].elevation + grade_after(Line, index) * (Chainage - points[index].chainage);
}

bool overlaps(double const From, double const To, context::zone const &Zone)
{
	return From < Zone.to - 1e-6 && To > Zone.from + 1e-6;
}

} // namespace

double grade_after(line const &Line, std::size_t const Index)
{
	auto const &points{Line.points};
	if (Index + 1 >= points.size())
		return 0.0;
	auto const run{points[Index + 1].chainage - points[Index].chainage};
	return run > 1e-9 ? (points[Index + 1].elevation - points[Index].elevation) / run : 0.0;
}

double required_radius(line const &Line)
{
	return std::max(Line.norms.radius_min, Line.norms.radius_factor * Line.speed * Line.speed);
}

void assign_radii(line &Line)
{
	auto const required{std::ceil(required_radius(Line) / 10.0) * 10.0};
	for (std::size_t k = 1; k + 1 < Line.points.size(); ++k)
	{
		auto &point{Line.points[k]};
		if (false == point.automatic || point.joint)
			continue;
		auto const change{std::abs(grade_after(Line, k) - grade_after(Line, k - 1)) * 1000.0};
		point.radius = change > Line.norms.curve_threshold + 1e-6 ? required : 0.0;
	}
}

void complete(line &Line, context const &Context)
{
	auto &points{Line.points};
	points.erase(std::remove_if(points.begin(), points.end(), [](pvi const &Point) { return Point.joint; }), points.end());
	if (points.size() < 2)
		return;
	bool const start{Line.join_start && Context.start_joined};
	bool const end{Line.join_end && Context.end_joined};
	if (start)
		points.front().elevation = Context.start_elevation;
	if (end)
		points.back().elevation = Context.end_elevation;

	// the vertical curve of the joint starts right at the end of the route, off the adjoining track
	auto const joint = [&](pvi const &End, pvi const &Neighbour, double const Grade, double const Direction) -> std::optional<pvi> {
		auto const run{(Neighbour.chainage - End.chainage) * Direction};
		if (run < 1.0)
			return std::nullopt;
		auto const offset{std::abs(Neighbour.elevation - (End.elevation + Grade * run * Direction))};
		if (offset / run * 1000.0 <= Line.norms.curve_threshold + 1e-6)
			return std::nullopt;
		auto radius{std::ceil(required_radius(Line) / 10.0) * 10.0};
		double tangent;
		if (run * run >= 2.0 * radius * offset)
			tangent = (run - std::sqrt(run * run - 2.0 * radius * offset)) * 0.5;
		else
		{
			tangent = run * 0.5;
			radius = run * run / (2.0 * offset);
		}
		pvi result;
		result.chainage = End.chainage + tangent * Direction;
		result.elevation = End.elevation + Grade * tangent * Direction;
		result.radius = radius;
		result.automatic = false;
		result.joint = true;
		return result;
	};
	if (start)
		if (auto const point{joint(points.front(), points[1], Context.start_grade, 1.0)})
			points.insert(points.begin() + 1, *point);
	if (end)
		if (auto const point{joint(points.back(), points[points.size() - 2], Context.end_grade, -1.0)})
			points.insert(points.end() - 1, *point);
	assign_radii(Line);
}

std::vector<curve> curves(line const &Line)
{
	std::vector<curve> result;
	auto const &points{Line.points};
	for (std::size_t k = 1; k + 1 < points.size(); ++k)
	{
		auto const radius{points[k].radius};
		if (radius <= 0.0)
			continue;
		curve c;
		c.point = k;
		c.change = grade_after(Line, k) - grade_after(Line, k - 1);
		c.tangent = radius * std::abs(c.change) * 0.5;
		if (c.tangent < 1e-6)
			continue;
		c.sagitta = c.tangent * c.tangent / (2.0 * radius);
		c.start = points[k].chainage - c.tangent;
		c.end = points[k].chainage + c.tangent;
		result.push_back(c);
	}
	return result;
}

double elevation(line const &Line, double const Chainage)
{
	auto height{polyline(Line, Chainage)};
	for (auto const &c : curves(Line))
	{
		auto const distance{std::abs(Chainage - Line.points[c.point].chainage)};
		if (distance >= c.tangent)
			continue;
		height += c.change * (c.tangent - distance) * (c.tangent - distance) / (4.0 * c.tangent);
	}
	return height;
}

double grade(line const &Line, double const Chainage)
{
	auto const &points{Line.points};
	if (points.size() < 2)
		return 0.0;
	std::size_t index{0};
	while (index + 2 < points.size() && Chainage >= points[index + 1].chainage)
		++index;
	auto slope{grade_after(Line, index)};
	for (auto const &c : curves(Line))
	{
		auto const offset{Chainage - points[c.point].chainage};
		if (std::abs(offset) >= c.tangent)
			continue;
		slope -= c.change * (c.tangent - std::abs(offset)) / (2.0 * c.tangent) * (offset < 0.0 ? -1.0 : 1.0);
	}
	return slope;
}

double radius_for_sagitta(line const &Line, std::size_t const Point, double const Sagitta)
{
	if (Point == 0 || Point + 1 >= Line.points.size())
		return 0.0;
	auto const change{std::abs(grade_after(Line, Point) - grade_after(Line, Point - 1))};
	if (change < 1e-9)
		return Line.points[Point].radius;
	return std::max(0.0, 8.0 * std::abs(Sagitta) / (change * change));
}

std::vector<issue> check(line const &Line, context const &Context)
{
	std::vector<issue> result;
	auto const &points{Line.points};
	auto const &norms{Line.norms};
	auto const add = [&](bool const Error, int const Point, double const Chainage, std::string Text) {
		result.push_back({Error, Point, Chainage, std::move(Text)});
	};
	if (points.size() < 2)
	{
		add(true, -1, 0.0, "The grade line needs at least the two ends");
		return result;
	}
	for (std::size_t k = 0; k + 1 < points.size(); ++k)
	{
		if (points[k + 1].chainage - points[k].chainage < 0.5)
			add(true, static_cast<int>(k + 1), points[k + 1].chainage, format("Points %zu and %zu are closer than 0.5 m", k + 1, k + 2));
		auto const slope{grade_after(Line, k) * 1000.0};
		if (std::abs(slope) > norms.grade_max + 1e-6)
			add(false, static_cast<int>(k), points[k].chainage, format("Grade %.2f per mille between points %zu and %zu exceeds %.2f", slope, k + 1, k + 2, norms.grade_max));
	}

	auto const list{curves(Line)};
	auto const required{required_radius(Line)};
	for (std::size_t k = 1; k + 1 < points.size(); ++k)
	{
		auto const change{std::abs(grade_after(Line, k) - grade_after(Line, k - 1)) * 1000.0};
		auto const radius{points[k].radius};
		if (radius <= 0.0 && change > norms.curve_threshold + 1e-6)
			add(false, static_cast<int>(k), points[k].chainage, format("Point %zu: change of grade %.2f per mille needs a vertical curve", k + 1, change));
		else if (radius > 0.0 && radius < required - 1e-6)
			add(false, static_cast<int>(k), points[k].chainage, format("Point %zu: R %.0f m is below the required %.0f m for %.0f km/h", k + 1, radius, required, Line.speed));
		if (radius <= 0.0 && change > 1e-3)
			for (auto const &zone : Context.switches)
				if (points[k].chainage > zone.from + 1e-3 && points[k].chainage < zone.to - 1e-3)
					add(true, static_cast<int>(k), points[k].chainage, format("Point %zu: the grade changes inside switch %s", k + 1, zone.name.c_str()));
	}
	for (std::size_t i = 0; i < list.size(); ++i)
	{
		auto const &c{list[i]};
		auto const point{static_cast<int>(c.point)};
		if (c.start < points.front().chainage - 1e-6 || c.end > points.back().chainage + 1e-6)
			add(true, point, points[c.point].chainage, format("Point %zu: the vertical curve reaches past the end of the route", c.point + 1));
		if (i + 1 < list.size() && c.end > list[i + 1].start + 1e-6)
			add(true, point, c.end, format("Vertical curves of points %zu and %zu overlap by %.2f m", c.point + 1, list[i + 1].point + 1, c.end - list[i + 1].start));
		for (auto const &zone : Context.switches)
			if (overlaps(c.start, c.end, zone))
				add(true, point, points[c.point].chainage, format("Point %zu: the vertical curve lies on switch %s", c.point + 1, zone.name.c_str()));
	}
	if (norms.element_min > 0.0)
	{
		for (std::size_t k = 1; k + 2 < points.size(); ++k)
		{
			auto const tangent = [&](std::size_t const Point) {
				for (auto const &c : list)
					if (c.point == Point)
						return c.tangent;
				return 0.0;
			};
			auto const free{(points[k + 1].chainage - tangent(k + 1)) - (points[k].chainage + tangent(k))};
			if (free < norms.element_min - 1e-6)
				add(false, static_cast<int>(k), points[k].chainage, format("Constant grade between points %zu and %zu is %.1f m long, less than %.0f m", k + 1, k + 2, std::max(0.0, free), norms.element_min));
		}
	}
	for (auto const &zone : Context.switches)
	{
		auto const slope{std::abs(grade(Line, (zone.from + zone.to) * 0.5)) * 1000.0};
		if (slope > norms.grade_switch_max + 1e-6)
			add(false, -1, zone.from, format("Grade %.2f per mille over switch %s exceeds %.2f", slope, zone.name.c_str(), norms.grade_switch_max));
	}
	for (auto const &fixed : Context.fixed)
	{
		auto const departure{elevation(Line, fixed.first) - fixed.second};
		if (std::abs(departure) > norms.fixed_tolerance)
		{
			add(true, -1, fixed.first, format("At %s the grade line departs %.3f m from a fixed point", format_chainage(fixed.first).c_str(), departure));
			break;
		}
	}
	auto const ends = [&](bool const Atend) {
		auto const joined{Atend ? Context.end_joined : Context.start_joined};
		if (false == joined)
			return;
		auto const chainage{Atend ? points.back().chainage : points.front().chainage};
		auto const departure{elevation(Line, chainage) - (Atend ? Context.end_elevation : Context.start_elevation)};
		if (std::abs(departure) > norms.fixed_tolerance)
			add(false, Atend ? static_cast<int>(points.size()) - 1 : 0, chainage, format("The %s of the grade line is %.3f m off the adjoining track", Atend ? "end" : "start", departure));
		auto const slope{grade(Line, chainage)};
		auto const change{std::abs(slope - (Atend ? Context.end_grade : Context.start_grade)) * 1000.0};
		if (change > norms.curve_threshold + 1e-6)
			add(false, Atend ? static_cast<int>(points.size()) - 1 : 0, chainage, format("Change of grade %.2f per mille against the adjoining track at the %s", change, Atend ? "end" : "start"));
	};
	ends(false);
	ends(true);
	std::stable_sort(result.begin(), result.end(), [](issue const &A, issue const &B) { return A.error && false == B.error; });
	return result;
}

line recognize(std::vector<double> const &Chainages, std::vector<double> const &Elevations, std::vector<double> const &Grades, recognition const &Options)
{
	line result;
	auto const count{std::min({Chainages.size(), Elevations.size(), Grades.size()})};
	if (count < 2)
		return result;

	struct part
	{
		std::size_t first, last;
		double slope{0.0}, offset{0.0};
	};
	auto const fit = [&](part &Part) {
		double sx{0.0}, sy{0.0}, sxx{0.0}, sxy{0.0};
		auto const origin{Chainages[Part.first]};
		auto const n{static_cast<double>(Part.last - Part.first + 1)};
		for (auto i = Part.first; i <= Part.last; ++i)
		{
			auto const x{Chainages[i] - origin};
			sx += x;
			sy += Elevations[i];
			sxx += x * x;
			sxy += x * Elevations[i];
		}
		auto const denominator{n * sxx - sx * sx};
		Part.slope = std::abs(denominator) > 1e-12 ? (n * sxy - sx * sy) / denominator : 0.0;
		Part.offset = (sy - Part.slope * sx) / n - Part.slope * origin;
	};

	std::vector<part> parts;
	for (std::size_t i = 0; i < count;)
	{
		auto j{i};
		auto low{Grades[i]}, high{Grades[i]};
		while (j + 1 < count && std::max(high, Grades[j + 1]) - std::min(low, Grades[j + 1]) <= Options.grade_tolerance)
		{
			++j;
			low = std::min(low, Grades[j]);
			high = std::max(high, Grades[j]);
		}
		if (Chainages[j] - Chainages[i] >= Options.minimum_length)
		{
			part p{i, j};
			fit(p);
			parts.push_back(p);
			i = j + 1;
		}
		else
			++i;
	}
	for (std::size_t i = 0; i + 1 < parts.size();)
	{
		if (std::abs(parts[i].slope - parts[i + 1].slope) < Options.grade_tolerance * 0.5)
		{
			parts[i].last = parts[i + 1].last;
			fit(parts[i]);
			parts.erase(parts.begin() + i + 1);
		}
		else
			++i;
	}

	result.points.push_back({Chainages.front(), Elevations.front(), 0.0});
	for (std::size_t i = 0; i + 1 < parts.size(); ++i)
	{
		auto const &a{parts[i]};
		auto const &b{parts[i + 1]};
		auto const from{Chainages[a.last]};
		auto const to{Chainages[b.first]};
		auto const change{b.slope - a.slope};
		auto chainage{std::abs(change) > 1e-9 ? (a.offset - b.offset) / change : (from + to) * 0.5};
		chainage = std::clamp(chainage, from, std::max(from, to));
		pvi point;
		point.automatic = false;
		point.chainage = chainage;
		point.elevation = a.slope * chainage + a.offset;
		auto const gap{to - from};
		if (gap >= Options.sharp_gap && std::abs(change) > 1e-9)
			point.radius = std::round(gap / std::abs(change));
		if (point.chainage - result.points.back().chainage >= 0.5)
			result.points.push_back(point);
	}
	if (Chainages[count - 1] - result.points.back().chainage < 0.5 && result.points.size() > 1)
		result.points.pop_back();
	result.points.push_back({Chainages[count - 1], Elevations[count - 1], 0.0});

	// the gap between constant grades is shorter than the curve, the sagitta gives the radius
	for (std::size_t i = 1; i + 1 < result.points.size(); ++i)
	{
		auto &point{result.points[i]};
		if (point.radius <= 0.0)
			continue;
		auto const next{std::lower_bound(Chainages.begin(), Chainages.begin() + count, point.chainage)};
		if (next == Chainages.begin() || next == Chainages.begin() + count)
			continue;
		auto const index{static_cast<std::size_t>(std::distance(Chainages.begin(), next))};
		auto const run{Chainages[index] - Chainages[index - 1]};
		auto const measured{run > 1e-9 ? Elevations[index - 1] + (Elevations[index] - Elevations[index - 1]) * (point.chainage - Chainages[index - 1]) / run : Elevations[index]};
		auto const radius{radius_for_sagitta(result, i, measured - point.elevation)};
		if (radius > 0.0)
			point.radius = std::round(radius);
	}
	return result;
}

std::vector<double> breaks(line const &Line)
{
	std::vector<double> result;
	auto const &points{Line.points};
	for (std::size_t k = 1; k + 1 < points.size(); ++k)
	{
		if (points[k].radius > 0.0)
			continue;
		if (std::abs(grade_after(Line, k) - grade_after(Line, k - 1)) > 1e-7)
			result.push_back(points[k].chainage);
	}
	for (auto const &c : curves(Line))
	{
		result.push_back(c.start);
		result.push_back(c.end);
	}
	std::sort(result.begin(), result.end());
	return result;
}

std::string format_chainage(double const Chainage)
{
	auto const value{std::abs(Chainage)};
	auto const kilometres{static_cast<long>(value / 1000.0)};
	return format("%s%ld+%06.2f", Chainage < 0.0 ? "-" : "", kilometres, value - kilometres * 1000.0);
}

}
