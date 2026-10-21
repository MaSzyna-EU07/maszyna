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
#include "utilities/translation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <tuple>

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
	return std::max({10.0, Line.norms.radius_min, Line.norms.radius_factor * Line.speed * Line.speed});
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

	// the vertical curve of the joint starts at the end of the route, off the adjoining track, or past the switches and the shared
	// timbers the end lies on: over them the grade of the adjoining track goes on unchanged
	auto const joint = [&](pvi const &End, pvi const &Neighbour, double const Grade, double const Direction) -> std::optional<pvi> {
		auto lead{0.0};
		for (bool grown{true}; grown;)
		{
			grown = false;
			for (auto const &zone : Context.switches)
			{
				auto const to_start{Direction > 0.0 ? zone.from - End.chainage : End.chainage - zone.to};
				auto const to_end{Direction > 0.0 ? zone.to - End.chainage : End.chainage - zone.from};
				if (to_start <= lead + 0.5 && to_end > lead)
				{
					lead = to_end;
					grown = true;
				}
			}
		}
		auto const run{(Neighbour.chainage - End.chainage) * Direction - lead};
		if (run < 1.0)
			return std::nullopt;
		auto const offset{std::abs(Neighbour.elevation - (End.elevation + Grade * (lead + run) * Direction))};
		if (offset / run * 1000.0 <= Line.norms.curve_threshold + 1e-6)
		{
			// no curve needed, but the grade breaks only past the timbers the end lies on
			if (lead < 0.5)
				return std::nullopt;
			pvi result;
			result.chainage = End.chainage + lead * Direction;
			result.elevation = End.elevation + Grade * lead * Direction;
			result.automatic = false;
			result.joint = true;
			return result;
		}
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
		result.chainage = End.chainage + (lead + tangent) * Direction;
		result.elevation = End.elevation + Grade * (lead + tangent) * Direction;
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
		add(true, -1, 0.0, STR_C("The grade line needs at least the two ends"));
		return result;
	}
	for (std::size_t k = 0; k + 1 < points.size(); ++k)
	{
		if (points[k + 1].chainage - points[k].chainage < 0.5)
			add(true, static_cast<int>(k + 1), points[k + 1].chainage, format(STR_C("Points %zu and %zu are closer than 0.5 m"), k + 1, k + 2));
		auto const slope{grade_after(Line, k) * 1000.0};
		if (std::abs(slope) > norms.grade_max + 1e-6)
			add(false, static_cast<int>(k), points[k].chainage, format(STR_C("Grade %.2f per mille between points %zu and %zu exceeds %.2f"), slope, k + 1, k + 2, norms.grade_max));
	}

	auto const list{curves(Line)};
	auto const required{required_radius(Line)};
	for (std::size_t k = 1; k + 1 < points.size(); ++k)
	{
		auto const change{std::abs(grade_after(Line, k) - grade_after(Line, k - 1)) * 1000.0};
		auto const radius{points[k].radius};
		if (radius <= 0.0 && change > norms.curve_threshold + 1e-6)
			add(false, static_cast<int>(k), points[k].chainage, format(STR_C("Point %zu: change of grade %.2f per mille needs a vertical curve"), k + 1, change));
		else if (radius > 0.0 && radius < required - 1e-6)
			add(false, static_cast<int>(k), points[k].chainage, format(STR_C("Point %zu: R %.0f m is below the required %.0f m for %.0f km/h"), k + 1, radius, required, Line.speed));
		if (radius <= 0.0 && change > 1e-3)
			for (auto const &zone : Context.switches)
				// a break on the bound of a switch is let be, within what the chainages of routes laid over each other differ by
				if (points[k].chainage > zone.from + 0.05 && points[k].chainage < zone.to - 0.05)
					add(true, static_cast<int>(k), points[k].chainage, format(STR_C("Point %zu: the grade changes inside switch %s"), k + 1, zone.name.c_str()));
	}
	for (std::size_t i = 0; i < list.size(); ++i)
	{
		auto const &c{list[i]};
		auto const point{static_cast<int>(c.point)};
		if (c.start < points.front().chainage - 1e-6 || c.end > points.back().chainage + 1e-6)
			add(true, point, points[c.point].chainage, format(STR_C("Point %zu: the vertical curve reaches past the end of the route"), c.point + 1));
		if (i + 1 < list.size() && c.end > list[i + 1].start + 1e-6)
			add(true, point, c.end, format(STR_C("Vertical curves of points %zu and %zu overlap by %.2f m"), c.point + 1, list[i + 1].point + 1, c.end - list[i + 1].start));
		for (auto const &zone : Context.switches)
			if (overlaps(c.start, c.end, zone))
				add(true, point, points[c.point].chainage, format(STR_C("Point %zu: the vertical curve lies on switch %s"), c.point + 1, zone.name.c_str()));
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
				add(false, static_cast<int>(k), points[k].chainage, format(STR_C("Constant grade between points %zu and %zu is %.1f m long, less than %.0f m"), k + 1, k + 2, std::max(0.0, free), norms.element_min));
		}
	}
	for (auto const &zone : Context.switches)
	{
		auto const slope{std::abs(grade(Line, (zone.from + zone.to) * 0.5)) * 1000.0};
		if (norms.grade_switch_max > 0.0 && slope > norms.grade_switch_max + 1e-6)
			add(false, -1, zone.from, format(STR_C("Grade %.2f per mille over switch %s exceeds %.2f"), slope, zone.name.c_str(), norms.grade_switch_max));
	}
	for (auto const &fixed : Context.fixed)
	{
		auto const departure{elevation(Line, fixed.first) - fixed.second};
		if (std::abs(departure) > norms.fixed_tolerance)
		{
			add(true, -1, fixed.first, format(STR_C("At %s the grade line departs %.3f m from a fixed point"), format_chainage(fixed.first).c_str(), departure));
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
			add(false, Atend ? static_cast<int>(points.size()) - 1 : 0, chainage, format(Atend ? STR_C("The end of the grade line is %.3f m off the adjoining track") : STR_C("The start of the grade line is %.3f m off the adjoining track"), departure));
		auto const slope{grade(Line, chainage)};
		auto const change{std::abs(slope - (Atend ? Context.end_grade : Context.start_grade)) * 1000.0};
		if (change > norms.curve_threshold + 1e-6)
			add(false, Atend ? static_cast<int>(points.size()) - 1 : 0, chainage, format(Atend ? STR_C("Change of grade %.2f per mille against the adjoining track at the end") : STR_C("Change of grade %.2f per mille against the adjoining track at the start"), change));
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
	auto const apart = [&](part const &A, part const &B, double const Chainage) { return std::abs((A.slope - B.slope) * Chainage + A.offset - B.offset); };
	for (std::size_t i = 0; i + 1 < parts.size();)
	{
		if (std::abs(parts[i].slope - parts[i + 1].slope) < Options.grade_tolerance * 0.5 && apart(parts[i], parts[i + 1], (Chainages[parts[i].last] + Chainages[parts[i + 1].first]) * 0.5) <= Options.step)
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
		// the grades don't meet between the parts, as at a stretch lifted with the grade it had: a break where each ends
		if (apart(a, b, chainage) > Options.step)
		{
			for (auto const &[at, slope, offset] : {std::tuple{from, a.slope, a.offset}, std::tuple{to, b.slope, b.offset}})
				if (at - result.points.back().chainage >= 0.5)
				{
					pvi point;
					point.automatic = false;
					point.chainage = at;
					point.elevation = slope * at + offset;
					result.points.push_back(point);
				}
			continue;
		}
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
	if (Options.detail <= 0.0)
		return result;
	std::vector<char> off(count, 0);
	for (std::size_t i = 0; i < count; ++i)
		off[i] = std::abs(elevation(result, Chainages[i]) - Elevations[i]) > Options.detail ? 1 : 0;
	std::vector<pvi> bends;
	for (std::size_t i = 0; i < count;)
	{
		if (off[i] == 0)
		{
			++i;
			continue;
		}
		auto const first{i > 0 ? i - 1 : i};
		while (i < count && off[i] != 0)
			++i;
		auto const last{std::min(i, count - 1)};
		std::vector<char> keep(count, 0);
		keep[first] = keep[last] = 1;
		std::vector<std::pair<std::size_t, std::size_t>> stack{{first, last}};
		while (false == stack.empty())
		{
			auto const [a, b]{stack.back()};
			stack.pop_back();
			if (b <= a + 1)
				continue;
			auto const run{std::max(1e-9, Chainages[b] - Chainages[a])};
			double worst{0.0};
			std::size_t index{a};
			for (auto k = a + 1; k < b; ++k)
			{
				auto const departure{std::abs(Elevations[k] - (Elevations[a] + (Elevations[b] - Elevations[a]) * (Chainages[k] - Chainages[a]) / run))};
				if (departure > worst)
				{
					worst = departure;
					index = k;
				}
			}
			if (worst > Options.detail * 0.5)
			{
				keep[index] = 1;
				stack.push_back({a, index});
				stack.push_back({index, b});
			}
		}
		auto const low{Chainages[first]}, high{Chainages[last]};
		result.points.erase(std::remove_if(result.points.begin() + 1, result.points.end() - 1, [&](pvi const &Point) { return Point.chainage >= low - 0.5 && Point.chainage <= high + 0.5; }), result.points.end() - 1);
		for (auto k = first; k <= last; ++k)
			if (keep[k] != 0 && k > 0 && k + 1 < count)
			{
				pvi point;
				point.automatic = false;
				point.chainage = Chainages[k];
				point.elevation = Elevations[k];
				bends.push_back(point);
			}
	}
	if (bends.empty())
		return result;
	result.points.insert(result.points.end() - 1, bends.begin(), bends.end());
	std::sort(result.points.begin(), result.points.end(), [](pvi const &Left, pvi const &Right) { return Left.chainage < Right.chainage; });
	std::vector<pvi> points;
	for (auto const &point : result.points)
		if (points.empty() || point.chainage - points.back().chainage >= 0.5 || &point == &result.points.back())
		{
			if (false == points.empty() && point.chainage - points.back().chainage < 0.5)
				points.pop_back();
			points.push_back(point);
		}
	result.points = std::move(points);
	return result;
}

line fit_ground(std::vector<double> const &Chainages, std::vector<double> const &Ground, context const &Context, line const &Settings, ground_fit const &Options, std::vector<double> const &Tolerances,
                std::vector<double> const &Reaches)
{
	line result;
	result.speed = Settings.speed;
	result.norms = Settings.norms;
	result.join_start = Settings.join_start;
	result.join_end = Settings.join_end;
	auto const count{std::min(Chainages.size(), Ground.size())};
	if (count < 2 || std::none_of(Ground.begin(), Ground.begin() + count, [](double const Height) { return std::isfinite(Height); }))
		return result;
	auto const origin{Chainages.front()};
	auto const length{Chainages[count - 1] - origin};
	if (length < 1.0)
		return result;

	// the line is sought on nodes a few metres apart, straight between them: the grade can change only at a node
	auto const nodes{std::max<std::size_t>(3, static_cast<std::size_t>(std::ceil(length / 5.0)) + 1)};
	auto const step{length / static_cast<double>(nodes - 1)};
	std::vector<std::size_t> cell(count);
	std::vector<double> part(count);
	for (std::size_t i = 0; i < count; ++i)
	{
		auto const at{std::clamp((Chainages[i] - origin) / step, 0.0, static_cast<double>(nodes - 1))};
		cell[i] = std::min(nodes - 2, static_cast<std::size_t>(at));
		part[i] = at - static_cast<double>(cell[i]);
	}
	auto const node = [&](std::size_t const K) { return origin + step * static_cast<double>(K); };
	// a change of grade at a node is the curve of the required radius around it, which keeps off the switches: none on a
	// switch, and beside one no more than a curve reaching up to it
	auto const required{std::ceil(required_radius(result) / 10.0) * 10.0};
	auto const sharpest{step / (required * 1.001)};
	std::vector<double> bend_max(nodes, sharpest);
	for (std::size_t k = 1; k + 1 < nodes; ++k)
		for (auto const &zone : Context.switches)
			bend_max[k] = std::min(bend_max[k], std::max(0.0, 2.0 * std::max(zone.from - node(k), node(k) - zone.to) / (required * 1.001)));
	struct held
	{
		std::size_t cell;
		double part;
		double low;
		double high;
	};
	std::vector<held> bounds;
	for (auto const &bound : Options.bounds)
	{
		auto const at{std::clamp((bound.chainage - origin) / step, 0.0, static_cast<double>(nodes - 1))};
		auto const cell{std::min(nodes - 2, static_cast<std::size_t>(at))};
		bounds.push_back({cell, at - static_cast<double>(cell), std::min(bound.low, bound.high), std::max(bound.low, bound.high)});
	}
	bool const start_fixed{Settings.join_start && Context.start_joined};
	bool const end_fixed{Settings.join_end && Context.end_joined};

	// a change of grade costs the price times its size, so the line breaks where the ground keeps away from it long enough to
	// be worth it. The ground costs the square of its distance over its tolerance; past its reach ever less, up to a bound:
	// what parts from the line, a bridge, a viaduct, a wall, a track on another level, is gone past rather than followed. The
	// bends are no sharper than the radius of the norms and the grades no steeper than the limit
	auto const tolerance{std::max(0.01, Options.tolerance)};
	auto const own = [&](std::size_t const I) { return I < Tolerances.size() && Tolerances[I] > 0.0 ? std::max(0.005, Tolerances[I]) : tolerance; };
	auto const reach = [&](std::size_t const I) { return I < Reaches.size() && Reaches[I] > 0.0 ? std::max(0.005, Reaches[I]) : own(I); };
	auto const price{24000.0 * tolerance * tolerance};
	auto const grade_max{(Options.grade_max > 0.0 ? Options.grade_max : result.norms.grade_max) / 1000.0};
	// what has to hold, the heights of the joined ends and the switches in one plane, holds before the grades of the ends, and
	// these before the limits of the norms
	double constexpr stiff{1e11};
	double constexpr firm{1e9};

	std::vector<double> x(nodes);
	{
		std::size_t i{0};
		for (std::size_t k = 0; k < nodes; ++k)
		{
			while (i + 1 < count && Chainages[i] < node(k))
				++i;
			auto j{i};
			while (j > 0 && false == std::isfinite(Ground[j]))
				--j;
			while (j + 1 < count && false == std::isfinite(Ground[j]))
				++j;
			x[k] = Ground[j];
		}
	}
	// the cost of a line, with its slope and the band of its curvature when asked. The distance of the ground counts up to
	// its tolerance times the root of the spread, the size of a change of grade is rounded off within the flat
	auto spread{256.0};
	auto flat{sharpest};
	std::vector<double> gradient(nodes), diagonal(nodes), first(nodes), second(nodes);
	auto const cost = [&](std::vector<double> const &Line, bool const Derivatives) {
		if (Derivatives)
		{
			std::fill(gradient.begin(), gradient.end(), 0.0);
			std::fill(diagonal.begin(), diagonal.end(), 1e-9);
			std::fill(first.begin(), first.end(), 0.0);
			std::fill(second.begin(), second.end(), 0.0);
		}
		auto total{0.0};
		auto const add = [&](std::initializer_list<std::pair<std::size_t, double>> Terms, double const Value, double const Slope, double const Curvature) {
			total += Value;
			if (false == Derivatives)
				return;
			for (auto const &[a, ca] : Terms)
			{
				gradient[a] += Slope * ca;
				for (auto const &[b, cb] : Terms)
				{
					if (b == a)
						diagonal[a] += Curvature * ca * cb;
					else if (b == a + 1)
						first[a] += Curvature * ca * cb;
					else if (b == a + 2)
						second[a] += Curvature * ca * cb;
				}
			}
		};
		auto const square = [&](std::initializer_list<std::pair<std::size_t, double>> Terms, double const Off, double const Weight) { add(Terms, Weight * Off * Off, 2.0 * Weight * Off, 2.0 * Weight); };
		for (std::size_t i = 0; i < count; ++i)
		{
			if (false == std::isfinite(Ground[i]))
				continue;
			auto const off{Line[cell[i]] * (1.0 - part[i]) + Line[cell[i] + 1] * part[i] - Ground[i]};
			auto const agreeing{spread * reach(i) * reach(i)};
			auto const share{agreeing / (agreeing + off * off)};
			auto const weight{tolerance * tolerance / (own(i) * own(i))};
			add({{cell[i], 1.0 - part[i]}, {cell[i] + 1, part[i]}}, weight * off * off * share, weight * 2.0 * off * share * share, weight * 2.0 * share * share);
		}
		for (std::size_t k = 1; k + 1 < nodes; ++k)
		{
			auto const terms = {std::pair{k - 1, 1.0 / step}, std::pair{k, -2.0 / step}, std::pair{k + 1, 1.0 / step}};
			auto const bend{(Line[k - 1] - 2.0 * Line[k] + Line[k + 1]) / step};
			if (bend_max[k] <= 0.0)
			{
				square(terms, bend, stiff);
				continue;
			}
			auto const root{std::sqrt(bend * bend + flat * flat)};
			add(terms, price * root, price * bend / root, price * flat * flat / (root * root * root));
			if (std::abs(bend) > bend_max[k])
				square(terms, bend - std::copysign(bend_max[k], bend), firm);
		}
		for (std::size_t k = 0; k + 1 < nodes; ++k)
		{
			auto const grade{(Line[k + 1] - Line[k]) / step};
			if (std::abs(grade) > grade_max)
				square({{k, -1.0 / step}, {k + 1, 1.0 / step}}, grade - std::copysign(grade_max, grade), firm);
		}
		if (start_fixed)
		{
			square({{0, 1.0}}, Line[0] - Context.start_elevation, stiff);
			square({{0, -1.0 / step}, {1, 1.0 / step}}, (Line[1] - Line[0]) / step - Context.start_grade, firm * 10.0);
		}
		if (end_fixed)
		{
			square({{nodes - 1, 1.0}}, Line[nodes - 1] - Context.end_elevation, stiff);
			square({{nodes - 2, -1.0 / step}, {nodes - 1, 1.0 / step}}, (Line[nodes - 1] - Line[nodes - 2]) / step - Context.end_grade, firm * 10.0);
		}
		for (auto const &bound : bounds)
		{
			auto const height{Line[bound.cell] * (1.0 - bound.part) + Line[bound.cell + 1] * bound.part};
			if (height < bound.low || height > bound.high)
				square({{bound.cell, 1.0 - bound.part}, {bound.cell + 1, bound.part}}, height - std::clamp(height, bound.low, bound.high), firm * 10.0);
		}
		return total;
	};
	// the step of Newton, the band factored as L D L^T with two diagonals below the main one
	auto const newton = [&]() {
		std::vector<double> d(nodes), l1(nodes, 0.0), l2(nodes, 0.0), z(nodes);
		for (std::size_t i = 0; i < nodes; ++i)
		{
			d[i] = diagonal[i] - (i >= 1 ? l1[i - 1] * l1[i - 1] * d[i - 1] : 0.0) - (i >= 2 ? l2[i - 2] * l2[i - 2] * d[i - 2] : 0.0);
			if (i + 1 < nodes)
				l1[i] = (first[i] - (i >= 1 ? l2[i - 1] * l1[i - 1] * d[i - 1] : 0.0)) / d[i];
			if (i + 2 < nodes)
				l2[i] = second[i] / d[i];
		}
		for (std::size_t i = 0; i < nodes; ++i)
			z[i] = -gradient[i] - (i >= 1 ? l1[i - 1] * z[i - 1] : 0.0) - (i >= 2 ? l2[i - 2] * z[i - 2] : 0.0);
		for (std::size_t i = 0; i < nodes; ++i)
			z[i] /= d[i];
		for (auto i = nodes - 1; i-- > 0;)
			z[i] -= l1[i] * z[i + 1] + (i + 2 < nodes ? l2[i] * z[i + 2] : 0.0);
		return z;
	};
	// the spread and the flat narrow down level by level, each level going on from the line of the one before: from nearly
	// all the ground agreeing and changes of grade costing their square, to the tolerance and their size
	for (;;)
	{
		auto current{cost(x, true)};
		for (int pass = 0; pass < 100; ++pass)
		{
			auto const direction{newton()};
			auto descent{0.0};
			for (std::size_t k = 0; k < nodes; ++k)
				descent += gradient[k] * direction[k];
			std::vector<double> trial(nodes);
			auto length{1.0};
			for (;; length *= 0.5)
			{
				for (std::size_t k = 0; k < nodes; ++k)
					trial[k] = x[k] + length * direction[k];
				if (cost(trial, false) <= current + 1e-4 * length * descent || length < 1e-6)
					break;
			}
			auto moved{0.0};
			for (std::size_t k = 0; k < nodes; ++k)
				moved = std::max(moved, std::abs(length * direction[k]));
			x = std::move(trial);
			current = cost(x, true);
			if (moved < 1e-5)
				break;
		}
		if (spread <= 1.0 && flat <= 1e-6)
			break;
		spread = std::max(1.0, spread * 0.5);
		flat = std::max(1e-6, flat * 0.5);
	}

	// the design: the changes of grade along a bend, one after another the same way, are one point of intersection where
	// the grades before and after them meet. No change at a node is sharper than the required radius, so the curve of that
	// radius stays within the nodes of its bend; where the ends held the line sharper, the curve is as sharp as the line
	auto const change = [&](std::size_t const K) { return (x[K - 1] - 2.0 * x[K] + x[K + 1]) / step; };
	auto const significant{sharpest * 0.01};
	auto const between = [&](std::size_t const K, std::size_t const L) {
		return std::any_of(Context.switches.begin(), Context.switches.end(), [&](auto const &Zone) { return Zone.to > node(K) && Zone.from < node(L); });
	};
	auto const bends = [&](std::size_t const K) { return bend_max[K] > 0.0 && std::abs(change(K)) >= significant; };
	// an end whose grade the line couldn't take over keeps only its height, the grade breaks at the joint
	if (start_fixed && std::abs((x[1] - x[0]) / step - Context.start_grade) > significant)
		result.join_start = false;
	if (end_fixed && std::abs((x[nodes - 1] - x[nodes - 2]) / step - Context.end_grade) > significant)
		result.join_end = false;
	result.points.push_back({origin, x[0]});
	std::vector<double> rooms{0.0};
	for (std::size_t k = 1; k + 1 < nodes;)
	{
		if (false == bends(k))
		{
			++k;
			continue;
		}
		auto const first{k};
		auto const sign{change(k) > 0.0};
		double sum{0.0}, moment{0.0};
		for (; k + 1 < nodes && bends(k) && (change(k) > 0.0) == sign && (k == first || false == between(k - 1, k)); ++k)
		{
			sum += change(k);
			moment += change(k) * node(k);
		}
		auto const at{moment / sum};
		pvi point{at, x[first] + (x[first] - x[first - 1]) / step * (at - node(first))};
		auto room{std::min(at - node(first) + step * 0.5, node(k - 1) + step * 0.5 - at)};
		for (auto const &zone : Context.switches)
			room = std::min(room, std::max(0.0, std::max(zone.from - at, at - zone.to)));
		result.points.push_back(point);
		rooms.push_back(room);
	}
	result.points.push_back({node(nodes - 1), x[nodes - 1]});
	rooms.push_back(0.0);
	// the curve of the required radius where it has room, off the switches and the curves beside; where it hasn't, as sharp as
	// the room lets it be
	for (std::size_t i = 1; i + 1 < result.points.size(); ++i)
	{
		auto const turn{std::abs(grade_after(result, i) - grade_after(result, i - 1))};
		if (turn * 1000.0 > result.norms.curve_threshold + 1e-6 && required * turn * 0.5 > rooms[i] - 1e-4)
		{
			result.points[i].radius = 2.0 * std::max(0.0, rooms[i] - 1e-4) / turn;
			result.points[i].automatic = false;
		}
	}
	assign_radii(result);

	// then the simplest design which keeps within half the tolerance of the line found, the one of the ground agreeing with
	// it: a point is dropped, or two bending the same way become one, as long as that departs the least and adds no fault
	std::vector<double> limit(nodes, tolerance);
	for (std::size_t i = 0; i < count; ++i)
		if (std::isfinite(Ground[i]) && std::abs(Ground[i] - (x[cell[i]] * (1.0 - part[i]) + x[cell[i] + 1] * part[i])) <= reach(i))
			for (auto const k : {cell[i], cell[i] + 1})
				limit[k] = std::min(limit[k], own(i));
	for (auto const &bound : bounds)
		for (auto const k : {bound.cell, bound.cell + 1})
			limit[k] = std::min(limit[k], std::max(0.001, (bound.high - bound.low) * 0.5));
	auto const errors = [&](line const &Line) {
		auto completed{Line};
		complete(completed, Context);
		auto const found{check(completed, Context)};
		return std::count_if(found.begin(), found.end(), [](issue const &Issue) { return Issue.error; });
	};
	auto const departure = [&](line const &Line, double const From, double const To) {
		auto worst{0.0};
		for (std::size_t k = 0; k < nodes; ++k)
			if (node(k) >= From && node(k) <= To)
				worst = std::max(worst, std::abs(elevation(Line, node(k)) - x[k]) / limit[k]);
		return worst;
	};
	auto faults{errors(result)};
	// an end whose grade the curve of the joint can't take over without a fault breaks at the joint
	for (auto *join : {&result.join_start, &result.join_end})
		if (faults > 0 && *join)
		{
			*join = false;
			if (auto const fewer{errors(result)}; fewer < faults)
				faults = fewer;
			else
				*join = true;
		}
	for (;;)
	{
		auto const &points{result.points};
		auto const last{points.size() - 1};
		std::vector<std::pair<double, line>> candidates;
		for (std::size_t i = 1; i < last; ++i)
		{
			auto const from{points[i >= 2 ? i - 2 : 0].chainage};
			auto dropped{result};
			dropped.points.erase(dropped.points.begin() + i);
			assign_radii(dropped);
			candidates.emplace_back(departure(dropped, from, points[std::min(last, i + 2)].chainage), std::move(dropped));
			if (i + 1 >= last)
				continue;
			auto const before{grade_after(result, i - 1)};
			auto const between{grade_after(result, i)};
			auto const after{grade_after(result, i + 1)};
			if ((between - before) * (after - between) <= 0.0)
				continue;
			auto const at{(points[i + 1].elevation - points[i].elevation + before * points[i].chainage - after * points[i + 1].chainage) / (before - after)};
			auto merged{result};
			merged.points[i] = {at, points[i].elevation + before * (at - points[i].chainage)};
			merged.points.erase(merged.points.begin() + i + 1);
			assign_radii(merged);
			candidates.emplace_back(departure(merged, from, points[std::min(last, i + 3)].chainage), std::move(merged));
		}
		std::sort(candidates.begin(), candidates.end(), [](auto const &A, auto const &B) { return A.first < B.first; });
		auto const chosen{std::find_if(candidates.begin(), candidates.end(), [&](auto const &Candidate) { return Candidate.first <= 0.5 && errors(Candidate.second) <= faults; })};
		if (chosen == candidates.end())
			break;
		result = std::move(chosen->second);
		faults = errors(result);
	}
	return result;
}

std::vector<departure> departures(line const &Line, std::vector<double> const &Chainages, std::vector<double> const &Ground, double const Tolerance)
{
	std::vector<departure> result;
	auto const count{std::min(Chainages.size(), Ground.size())};
	auto const off = [&](std::size_t const I) { return std::isfinite(Ground[I]) ? Ground[I] - elevation(Line, Chainages[I]) : 0.0; };
	for (std::size_t i = 0; i < count;)
	{
		if (std::abs(off(i)) <= Tolerance)
		{
			++i;
			continue;
		}
		departure stretch;
		stretch.from = Chainages[i];
		auto const below{off(i) < 0.0};
		for (; i < count && std::abs(off(i)) > Tolerance && (off(i) < 0.0) == below; ++i)
		{
			if (std::abs(off(i)) > std::abs(stretch.depth))
				stretch.depth = off(i);
			stretch.to = Chainages[i];
		}
		result.push_back(stretch);
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
