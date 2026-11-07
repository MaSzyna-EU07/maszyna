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
#include <initializer_list>
#include <limits>
#include <optional>
#include <set>
#include <span>
#include <utility>
#include <vector>

namespace profile
{

namespace
{

// a switch, and the timbers it shares, may lie on a vertical curve: a concave one, or a convex one no sharper than this
double constexpr switch_crest{2000.0}; // m
// per mille: the change of the grade on a switch taken for a break of it, 3 mm over 30 m of it; below it the switch lies in one plane
double constexpr switch_break{0.1};

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

// a break of the grade line where two parts of it don't meet, half a metre past the point before it at least
void add_break(line &Result, double const At, double const Slope, double const Offset)
{
	if (At - Result.points.back().chainage >= 0.5)
	{
		pvi point;
		point.automatic = false;
		point.chainage = At;
		point.elevation = Slope * At + Offset;
		Result.points.push_back(point);
	}
}

// the point between A and B the elevations lie farthest off the chord between them at, and how far: A itself and 0 when none does
std::pair<double, std::size_t> farthest_off(std::vector<double> const &Chainages, std::vector<double> const &Elevations, std::size_t const A, std::size_t const B, double const Run)
{
	double worst{0.0};
	std::size_t index{A};
	for (auto k = A + 1; k < B; ++k)
	{
		auto const departure{std::abs(Elevations[k] - (Elevations[A] + (Elevations[B] - Elevations[A]) * (Chainages[k] - Chainages[A]) / Run))};
		if (departure > worst)
		{
			worst = departure;
			index = k;
		}
	}
	return {worst, index};
}

// the points of a run of the line parted from the ground followed at the height, those followed already kept as they are
void follow_run(std::vector<double> &Followed, std::vector<std::size_t> const &Run, double const Height)
{
	for (auto const i : Run)
		if (Followed[i] <= 0.0)
			Followed[i] = Height;
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
		// a change as small as the grade lines laid one after another meet with, a few millimetres over the length of a switch, is no
		// break of it: the tracks of a slip met so lie in one plane all the same (pila.scn: 0.05 per mille inside shared timbers)
		if (false == (radius <= 0.0 && change > switch_break))
			continue;
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
		// a switch may lie on a concave curve, and on a convex one no sharper than switch_crest, though better off them; within
		// what the fit rounds the radius to its room by
		for (auto const &zone : Context.switches)
			if (overlaps(c.start, c.end, zone))
			{
				if (auto const on_hump{std::ranges::any_of(Context.humps, [&](context::zone const &Hump) { return points[c.point].chainage >= Hump.from && points[c.point].chainage <= Hump.to; })}; c.change < 0.0 && points[c.point].radius < switch_crest * 0.99 && false == on_hump)
					add(true, point, points[c.point].chainage, format(STR_C("Point %zu: the convex vertical curve on switch %s is sharper than %.0f m"), c.point + 1, zone.name.c_str(), switch_crest));
				else
					add(false, point, points[c.point].chainage, format(STR_C("Point %zu: the vertical curve lies on switch %s"), c.point + 1, zone.name.c_str()));
				break;
			}
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
				add_break(result, at, slope, offset);
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
		auto radius{radius_for_sagitta(result, i, measured - point.elevation)};
		// of a change of grade too small to tell its curve by, the sagitta is the millimetres of the track: the curve is no longer
		// than half the grades on either side of it, off the curves beside and the ends
		auto room{std::numeric_limits<double>::max()};
		if (auto const change{std::abs(grade_after(result, i) - grade_after(result, i - 1))}; change > 1e-9)
			room = std::min(point.chainage - result.points[i - 1].chainage, result.points[i + 1].chainage - point.chainage) / change;
		if (radius > room)
			point.radius = std::floor(room);
		else if (radius > 0.0)
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
		keep[last] = 1;
		keep[first] = 1;
		std::vector<std::pair<std::size_t, std::size_t>> stack{{first, last}};
		while (false == stack.empty())
		{
			auto const [a, b]{stack.back()};
			stack.pop_back();
			if (b <= a + 1)
				continue;
			auto const run{std::max(1e-9, Chainages[b] - Chainages[a])};
			auto const [worst, index]{farthest_off(Chainages, Elevations, a, b, run)};
			if (worst > Options.detail * 0.5)
			{
				keep[index] = 1;
				stack.push_back({a, index});
				stack.push_back({index, b});
			}
		}
		auto const low{Chainages[first]};
		auto const high{Chainages[last]};
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
	std::ranges::sort(result.points, [](pvi const &Left, pvi const &Right) { return Left.chainage < Right.chainage; });
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

namespace
{

// the ground of a line every few metres, the mean of the model about each, past its noise, and the humps of a marshalling yard on it
struct hump_ground
{
	static constexpr double spacing{5.0};
	// the crest is the highest of the ground about it, the feet where the ground has fallen nearly as far as it falls within
	// the reach: some metres lower than the crest, not as many as of a hill or a bridge
	static constexpr double around{60.0}; // m
	static constexpr double farthest{300.0}; // m
	static constexpr double lowest{1.0}; // m
	static constexpr double highest{6.0}; // m
	// the grades just past the rounding of the crest fall away from it both ways, one of them more steeply than lines and
	// stations are laid, the two together a change of grade over some 50 m which no curve of 1250 m or more makes
	static constexpr double inner{10.0}; // m
	static constexpr double outer{40.0}; // m
	static constexpr double falling{5.0}; // per mille
	static constexpr double steep{25.0}; // per mille
	static constexpr double sharp{40.0}; // per mille
	// nowhere about the crest as steep as the banks of a bridge, the walls of a cutting
	static constexpr double wall{80.0}; // per mille
	// no switch on the crest; past it the fan, switches of their own length, not the shared timbers of the ones beside
	static constexpr double clear{15.0}; // m
	static constexpr double fan{150.0}; // m
	static constexpr double body{15.0}; // m
	static constexpr std::size_t fanning{2};

	// the grades of the two sides of a crest, summed up
	struct falls
	{
		double grades{0.0};
		double steepest{0.0};
		bool fanned{false};
	};

	std::vector<double> level;
	std::ptrdiff_t cells{0};
	double origin{0.0};
	double end{0.0};
	std::vector<context::zone> const &switches;

	double at(std::ptrdiff_t const K) const
	{
		return K >= 0 && K < cells ? level[K] : std::numeric_limits<double>::quiet_NaN();
	}
	static std::ptrdiff_t cells_of(double const Distance)
	{
		return static_cast<std::ptrdiff_t>(std::lround(Distance / spacing));
	}
	// whether the ground of the cell C, Top high, is the highest about it, the first of the highest
	bool highest_about(std::ptrdiff_t const C, double const Top) const
	{
		for (auto k = C - cells_of(around); k <= C + cells_of(around); ++k)
			if (auto const other{at(k)}; std::isfinite(other) && (other > Top || (other == Top && k < C)))
				return false;
		return true;
	}
	// the lowest of the ground within the reach to the Side, and in how many cells the model has it
	std::pair<double, std::ptrdiff_t> lowest_beside(std::ptrdiff_t const C, int const Side, double const Top) const
	{
		auto low{Top};
		std::ptrdiff_t seen{0};
		auto const reach{cells_of(farthest)};
		for (std::ptrdiff_t d = 1; d <= reach; ++d)
			if (auto const height{at(C + Side * d)}; std::isfinite(height))
			{
				low = std::min(low, height);
				++seen;
			}
		return {low, seen};
	}
	// the cells from the crest to the foot to the Side, where the ground has fallen nearly as far as to Low
	std::ptrdiff_t foot_of(std::ptrdiff_t const C, int const Side, double const Top, double const Low) const
	{
		auto const reach{cells_of(farthest)};
		for (std::ptrdiff_t d = 1; d <= reach; ++d)
			if (auto const height{at(C + Side * d)}; std::isfinite(height) && Top - height >= 0.9 * (Top - Low))
				return d;
		return reach;
	}
	// whether the ground about the crest to the Side is anywhere as steep as a wall
	bool walled(std::ptrdiff_t const C, int const Side) const
	{
		for (std::ptrdiff_t d = 0; d + 2 <= cells_of(around); ++d)
			if (auto const a{at(C + Side * d)}, b{at(C + Side * (d + 2))}; std::isfinite(a) && std::isfinite(b) && std::abs(a - b) / (2.0 * spacing) * 1000.0 > wall)
				return true;
		return false;
	}
	// the switches of the fan past the Crest to the Side
	std::size_t fan_past(double const Crest, int const Side) const
	{
		std::size_t past{0};
		for (auto const &zone : switches)
			if (auto const distance{Side > 0 ? zone.from - Crest : Crest - zone.to}; zone.to - zone.from >= body && distance >= clear && distance <= fan)
				++past;
		return past;
	}
	// the side of the crest in the cell C, Top high, to the Side: false where it isn't one of a hump
	bool side_of(std::ptrdiff_t const C, int const Side, double const Top, hump &Found, falls &Falls) const
	{
		auto const [low, seen]{lowest_beside(C, Side, Top)};
		if (seen < cells_of(around) || Top - low < lowest || Top - low > highest)
			return false;
		Found.height = std::max(Found.height, Top - low);
		auto const foot{foot_of(C, Side, Top, low)};
		// and on past the foot as far as the curve into the grade beyond
		(Side < 0 ? Found.from : Found.to) = std::clamp(Found.crest + Side * spacing * static_cast<double>(foot + cells_of(25.0)), origin, end);
		auto const off{at(C + Side * cells_of(inner))};
		auto const on{at(C + Side * cells_of(outer))};
		if (false == std::isfinite(off) || false == std::isfinite(on))
			return false;
		auto const grade{(off - on) / (outer - inner) * 1000.0};
		if (grade < falling)
			return false;
		Falls.grades += grade;
		Falls.steepest = std::max(Falls.steepest, grade);
		if (walled(C, Side))
			return false;
		Falls.fanned = Falls.fanned || fan_past(Found.crest, Side) >= fanning;
		return true;
	}
	// the hump whose crest is in the cell C, none where it isn't one
	std::optional<hump> crest_of(std::ptrdiff_t const C) const
	{
		auto const top{at(C)};
		if (false == std::isfinite(top) || false == highest_about(C, top))
			return std::nullopt;
		hump found;
		found.crest = origin + spacing * static_cast<double>(C);
		if (std::ranges::any_of(switches, [&found](context::zone const &Zone) { return Zone.from < found.crest + clear && Zone.to > found.crest - clear; }))
			return std::nullopt;
		falls sides;
		for (int const side : {-1, 1})
			if (false == side_of(C, side, top, found, sides))
				return std::nullopt;
		if (sides.steepest < steep || sides.grades < sharp || false == sides.fanned)
			return std::nullopt;
		return found;
	}
};

} // namespace

std::vector<hump> humps(std::vector<double> const &Chainages, std::vector<double> const &Ground, context const &Context, double const Tolerance, std::vector<double> const &Tolerances)
{
	std::vector<hump> result;
	auto const count{std::min(Chainages.size(), Ground.size())};
	if (count < 2)
		return result;
	// the ground every few metres, the mean of the model about each, past its noise
	double constexpr spacing{hump_ground::spacing};
	auto const origin{Chainages.front()};
	auto const end{Chainages[count - 1]};
	auto const cells{static_cast<std::ptrdiff_t>(std::lround((end - origin) / spacing)) + 1};
	std::vector level(cells, 0.0);
	std::vector taken(cells, 0);
	for (std::size_t i = 0; i < count; ++i)
	{
		if (false == std::isfinite(Ground[i]) || (i < Tolerances.size() && Tolerances[i] > 0.0 && Tolerances[i] < Tolerance))
			continue;
		auto const k{std::clamp<std::ptrdiff_t>(std::lround((Chainages[i] - origin) / spacing), 0, cells - 1)};
		level[k] += Ground[i];
		++taken[k];
	}
	for (std::ptrdiff_t k = 0; k < cells; ++k)
		level[k] = taken[k] > 0 ? level[k] / taken[k] : std::numeric_limits<double>::quiet_NaN();
	hump_ground const ground{std::move(level), cells, origin, end, Context.switches};
	for (std::ptrdiff_t c = 0; c < cells; ++c)
		if (auto const found{ground.crest_of(c)}; found && (result.empty() || found->crest > result.back().to))
			result.push_back(*found);
	return result;
}

namespace
{

// the faults of a line completed as it is laid
std::ptrdiff_t errors_of(line const &Line, context const &Context)
{
	auto completed{Line};
	complete(completed, Context);
	auto const found{check(completed, Context)};
	return std::ranges::count_if(found, [](issue const &Issue) { return Issue.error; });
}

// an end whose grade the curve of the joint can't take over without a fault breaks at the joint
void joins_broken(line &Result, std::ptrdiff_t &Faults, context const &Context)
{
	for (auto *join : {&Result.join_start, &Result.join_end})
		if (Faults > 0 && *join)
		{
			*join = false;
			if (auto const fewer{errors_of(Result, Context)}; fewer < Faults)
				Faults = fewer;
			else
				*join = true;
		}
}

// the samples of the ground a line is fitted to
struct ground_samples
{
	std::vector<double> const &chainages;
	std::vector<double> const &ground;
	std::vector<double> const &tolerances;
	std::vector<double> const &reaches;
	std::size_t count;
};

// the line fitted to the ground on nodes a few metres apart, straight between them, and the design made of it
struct ground_fitter
{
	using term_list = std::initializer_list<std::pair<std::size_t, double>>;
	// a bound of the height of the line at a place between two nodes
	struct held
	{
		std::size_t cell;
		double part;
		double low;
		double high;
		bool yielding;
	};
	// the cost of a line summed up, with its derivatives when asked
	struct cost_sum
	{
		double total{0.0};
		bool derivatives{false};
	};
	// a run of the ground parted from the line one way
	struct parting
	{
		std::vector<std::size_t> run;
		std::size_t back{0};
		std::size_t previous{0};
		int side{0};
		double farthest{0.0};
		double steepest{0.0};
		bool more{false};
	};
	// the points of a design, with the room and the length of the bend of each and its first node and the one past its last
	struct bend_runs
	{
		std::vector<double> rooms{0.0};
		std::vector<double> lengths{0.0};
		std::vector<std::pair<std::size_t, std::size_t>> runs{{0, 0}};
	};

	// what has to hold, the heights of the joined ends, holds before the grades of the ends, and these before the limits of the
	// norms. A yielding bound counts as much as a kilometre of the ground off as far, and gives
	// way to the norms
	static constexpr double stiff{1e11};
	static constexpr double firm{1e9};
	static constexpr double leaning{1e3};
	// the steepness of the ground over a few metres, past the noise of the model
	static constexpr double chord{5.0};

	std::vector<double> const &chainages;
	std::vector<double> const &ground;
	std::vector<double> const &tolerances;
	std::vector<double> const &reaches;
	std::size_t count;
	context const &line_context;
	ground_fit const &options;
	double origin{0.0};
	std::size_t nodes{0};
	double step{0.0};
	std::vector<std::size_t> cell;
	std::vector<double> part;
	double required{0.0};
	double sharpest{0.0};
	std::vector<double> sag_max;
	std::vector<double> crest_max;
	std::vector<held> bounds;
	bool start_fixed{false};
	bool end_fixed{false};
	double tolerance{0.0};
	// the ground the line follows however far it parts, and as closely as asked, where it parted longer than a structure would
	std::vector<double> followed;
	double close_to{0.0};
	double price{0.0};
	double grade_max{0.0};
	std::vector<double> x;
	double spread{256.0};
	double flat{0.0};
	std::vector<double> gradient;
	std::vector<double> diagonal;
	std::vector<double> first;
	std::vector<double> second;
	// how far the design may keep from the line found at each node
	std::vector<double> limit;

	ground_fitter(ground_samples const &Samples, context const &Context, line const &Settings, ground_fit const &Options, line const &Result) :
		chainages{Samples.chainages}, ground{Samples.ground}, tolerances{Samples.tolerances}, reaches{Samples.reaches}, count{Samples.count}, line_context{Context}, options{Options}
	{
		origin = chainages.front();
		auto const length{chainages[count - 1] - origin};
		// the line is sought on nodes a few metres apart, straight between them: the grade can change only at a node
		nodes = std::max<std::size_t>(3, static_cast<std::size_t>(std::ceil(length / 5.0)) + 1);
		step = length / static_cast<double>(nodes - 1);
		cell.resize(count);
		part.resize(count);
		for (std::size_t i = 0; i < count; ++i)
		{
			auto const at{std::clamp((chainages[i] - origin) / step, 0.0, static_cast<double>(nodes - 1))};
			cell[i] = std::min(nodes - 2, static_cast<std::size_t>(at));
			part[i] = at - static_cast<double>(cell[i]);
		}
		// a change of grade at a node is the curve of the required radius around it. On a hump the least radius is the one of the
		// hump, though a convex curve over a switch is no sharper than switch_crest
		required = std::ceil(required_radius(Result) / 10.0) * 10.0;
		sharpest = step / (required * 1.001);
		curvature_limits();
		bounds_held();
		start_fixed = Settings.join_start && Context.start_joined;
		end_fixed = Settings.join_end && Context.end_joined;
		// a change of grade costs the price times its size, so the line breaks where the ground keeps away from it long enough to
		// be worth it. The ground costs the square of its distance over its tolerance; past its reach ever less, up to a bound:
		// what parts from the line, a bridge, a viaduct, a wall, a track on another level, is gone past rather than followed. The
		// bends are no sharper than the radius of the norms and the grades no steeper than the limit
		tolerance = std::max(0.01, Options.tolerance);
		followed.assign(count, 0.0);
		close_to = Options.follow_tolerance > 0.0 ? std::max(0.005, Options.follow_tolerance) : tolerance;
		humps_followed();
		price = 24000.0 * tolerance * tolerance;
		grade_max = (Options.grade_max > 0.0 ? Options.grade_max : Result.norms.grade_max) / 1000.0;
		first_line();
		// the cost of a line, with its slope and the band of its curvature when asked. The distance of the ground counts up to
		// its tolerance times the root of the spread, the size of a change of grade is rounded off within the flat
		flat = sharpest;
		gradient.resize(nodes);
		diagonal.resize(nodes);
		first.resize(nodes);
		second.resize(nodes);
	}

	double node(std::size_t const K) const
	{
		return origin + step * static_cast<double>(K);
	}
	bool on_hump(double const Chainage) const
	{
		return std::ranges::any_of(options.humps, [Chainage](hump const &Hump) { return Chainage >= Hump.from && Chainage <= Hump.to; });
	}
	double least(double const Chainage) const
	{
		return options.hump_radius > 0.0 && on_hump(Chainage) ? std::min(required, std::ceil(options.hump_radius / 10.0) * 10.0) : required;
	}
	bool on_switch(double const From, double const To) const
	{
		return std::ranges::any_of(line_context.switches, [From, To](auto const &Zone) { return Zone.to > From && Zone.from < To; });
	}
	// the sharpest bends of the line at the nodes, sagging and cresting
	void curvature_limits()
	{
		sag_max.resize(nodes);
		crest_max.resize(nodes);
		for (std::size_t k = 0; k < nodes; ++k)
		{
			auto const radius{least(node(k))};
			crest_max[k] = step / (radius * 1.001);
			sag_max[k] = crest_max[k];
			// a sharper one keeps its curve off the switches
			if (radius < switch_crest)
				for (auto const &zone : line_context.switches)
					crest_max[k] = std::min(crest_max[k], std::max(step / (switch_crest * 1.001), 2.0 * std::max(zone.from - node(k), node(k) - zone.to) / (radius * 1.001)));
		}
	}
	// the bounds of the options between the nodes. Those which contradict each other about one place, as of two crossovers into
	// lines fitted apart in height, hold as far as both can: the line keeps between them. Held both, it would tear itself apart
	// over them
	void bounds_held()
	{
		for (auto const &bound : options.bounds)
		{
			auto const at{std::clamp((bound.chainage - origin) / step, 0.0, static_cast<double>(nodes - 1))};
			auto const cell_index{std::min(nodes - 2, static_cast<std::size_t>(at))};
			bounds.emplace_back(cell_index, at - static_cast<double>(cell_index), std::min(bound.low, bound.high), std::max(bound.low, bound.high), bound.yielding);
		}
		std::ranges::sort(bounds, [](held const &A, held const &B) { return A.cell + A.part < B.cell + B.part; });
		for (std::size_t group = 0; group < bounds.size();)
		{
			auto last{group + 1};
			while (last < bounds.size() && static_cast<double>(bounds[last].cell) + bounds[last].part - static_cast<double>(bounds[group].cell) - bounds[group].part <= 1.0)
				++last;
			reconciled(group, last);
			group = last;
		}
	}
	// the bounds from First up to Last, about one place, between the lowest of their highs and the highest of their lows where
	// these cross
	void reconciled(std::size_t const First, std::size_t const Last)
	{
		auto low{-std::numeric_limits<double>::max()};
		auto high{std::numeric_limits<double>::max()};
		for (auto i = First; i < Last; ++i)
		{
			low = std::max(low, bounds[i].low);
			high = std::min(high, bounds[i].high);
		}
		if (low > high)
			for (auto i = First; i < Last; ++i)
			{
				bounds[i].low = high;
				bounds[i].high = low;
			}
	}
	double told(std::size_t const I) const
	{
		return I < tolerances.size() && tolerances[I] > 0.0 ? std::max(0.005, tolerances[I]) : tolerance;
	}
	double own(std::size_t const I) const
	{
		return followed[I] > 0.0 ? close_to : told(I);
	}
	double reach(std::size_t const I) const
	{
		if (followed[I] > 0.0)
			return followed[I];
		return I < reaches.size() && reaches[I] > 0.0 ? std::max(0.005, reaches[I]) : told(I);
	}
	// which side of the ground the line is at a point, by how far it's off: 1 above, -1 below, 0 within its reach or followed
	int side_of(std::size_t const I, double const Off) const
	{
		if (followed[I] > 0.0)
			return 0;
		if (Off > reach(I))
			return 1;
		return Off < -reach(I) ? -1 : 0;
	}
	// how far the Line is off the ground at the sample I
	double off_of(std::vector<double> const &Line, std::size_t const I) const
	{
		return Line[cell[I]] * (1.0 - part[I]) + Line[cell[I] + 1] * part[I] - ground[I];
	}
	void humps_followed()
	{
		for (auto const &hump : options.humps)
			for (std::size_t i = 0; i < count; ++i)
				if (std::isfinite(ground[i]) && told(i) >= tolerance && chainages[i] >= hump.from && chainages[i] <= hump.to)
					followed[i] = hump.height + tolerance;
	}
	// the line to start from: the ground at the nodes, the nearest of the model where it has none
	void first_line()
	{
		x.resize(nodes);
		std::size_t i{0};
		for (std::size_t k = 0; k < nodes; ++k)
		{
			while (i + 1 < count && chainages[i] < node(k))
				++i;
			auto j{i};
			while (j > 0 && false == std::isfinite(ground[j]))
				--j;
			while (j + 1 < count && false == std::isfinite(ground[j]))
				++j;
			x[k] = ground[j];
		}
	}

	// a term of the cost: its Value, and when the derivatives are asked its Slope and Curvature along the Terms of the line
	void add(term_list const Terms, double const Value, double const Slope, double const Curvature, cost_sum &Sum)
	{
		Sum.total += Value;
		if (false == Sum.derivatives)
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
	}
	void square(term_list const Terms, double const Off, double const Weight, cost_sum &Sum)
	{
		add(Terms, Weight * Off * Off, 2.0 * Weight * Off, 2.0 * Weight, Sum);
	}
	void ground_cost(std::vector<double> const &Line, cost_sum &Sum)
	{
		for (std::size_t i = 0; i < count; ++i)
		{
			if (false == std::isfinite(ground[i]))
				continue;
			auto const off{off_of(Line, i)};
			auto const agreeing{spread * reach(i) * reach(i)};
			auto const share{agreeing / (agreeing + off * off)};
			auto const weight{tolerance * tolerance / (own(i) * own(i))};
			add({{cell[i], 1.0 - part[i]}, {cell[i] + 1, part[i]}}, weight * off * off * share, weight * 2.0 * off * share * share, weight * 2.0 * share * share, Sum);
		}
	}
	void bend_cost(std::vector<double> const &Line, cost_sum &Sum)
	{
		for (std::size_t k = 1; k + 1 < nodes; ++k)
		{
			auto const terms = {std::pair{k - 1, 1.0 / step}, std::pair{k, -2.0 / step}, std::pair{k + 1, 1.0 / step}};
			auto const bend{(Line[k - 1] - 2.0 * Line[k] + Line[k + 1]) / step};
			auto const root{std::sqrt(bend * bend + flat * flat)};
			add(terms, price * root, price * bend / root, price * flat * flat / (root * root * root), Sum);
			if (auto const most{bend > 0.0 ? sag_max[k] : crest_max[k]}; std::abs(bend) > most)
				square(terms, bend - std::copysign(most, bend), firm, Sum);
		}
	}
	void grade_cost(std::vector<double> const &Line, cost_sum &Sum)
	{
		for (std::size_t k = 0; k + 1 < nodes; ++k)
		{
			auto const grade{(Line[k + 1] - Line[k]) / step};
			if (std::abs(grade) > grade_max)
				square({{k, -1.0 / step}, {k + 1, 1.0 / step}}, grade - std::copysign(grade_max, grade), firm, Sum);
		}
	}
	void ends_cost(std::vector<double> const &Line, cost_sum &Sum)
	{
		if (start_fixed)
		{
			square({{0, 1.0}}, Line[0] - line_context.start_elevation, stiff, Sum);
			if (options.hold_grades)
				square({{0, -1.0 / step}, {1, 1.0 / step}}, (Line[1] - Line[0]) / step - line_context.start_grade, firm * 10.0, Sum);
		}
		if (end_fixed)
		{
			square({{nodes - 1, 1.0}}, Line[nodes - 1] - line_context.end_elevation, stiff, Sum);
			if (options.hold_grades)
				square({{nodes - 2, -1.0 / step}, {nodes - 1, 1.0 / step}}, (Line[nodes - 1] - Line[nodes - 2]) / step - line_context.end_grade, firm * 10.0, Sum);
		}
	}
	void bounds_cost(std::vector<double> const &Line, cost_sum &Sum)
	{
		for (auto const &bound : bounds)
		{
			auto const height{Line[bound.cell] * (1.0 - bound.part) + Line[bound.cell + 1] * bound.part};
			if (height < bound.low || height > bound.high)
				square({{bound.cell, 1.0 - bound.part}, {bound.cell + 1, bound.part}}, height - std::clamp(height, bound.low, bound.high), bound.yielding ? leaning : firm * 10.0, Sum);
		}
	}
	// the cost of a Line, with its slope and the band of its curvature when the Derivatives are asked
	double cost(std::vector<double> const &Line, bool const Derivatives)
	{
		if (Derivatives)
		{
			std::ranges::fill(gradient, 0.0);
			std::ranges::fill(diagonal, 1e-9);
			std::ranges::fill(first, 0.0);
			std::ranges::fill(second, 0.0);
		}
		cost_sum sum{0.0, Derivatives};
		ground_cost(Line, sum);
		bend_cost(Line, sum);
		grade_cost(Line, sum);
		ends_cost(Line, sum);
		bounds_cost(Line, sum);
		return sum.total;
	}
	// the band factored as L D L^T: its diagonal D and the two diagonals of L below the main one, L1 and L2
	void factored(std::vector<double> &D, std::vector<double> &L1, std::vector<double> &L2) const
	{
		for (std::size_t i = 0; i < nodes; ++i)
		{
			D[i] = diagonal[i] - (i >= 1 ? L1[i - 1] * L1[i - 1] * D[i - 1] : 0.0) - (i >= 2 ? L2[i - 2] * L2[i - 2] * D[i - 2] : 0.0);
			if (i + 1 < nodes)
				L1[i] = (first[i] - (i >= 1 ? L2[i - 1] * L1[i - 1] * D[i - 1] : 0.0)) / D[i];
			if (i + 2 < nodes)
				L2[i] = second[i] / D[i];
		}
	}
	// the step of Newton, the band factored as L D L^T with two diagonals below the main one
	std::vector<double> newton() const
	{
		std::vector<double> d(nodes);
		std::vector l1(nodes, 0.0);
		std::vector l2(nodes, 0.0);
		std::vector<double> z(nodes);
		factored(d, l1, l2);
		for (std::size_t i = 0; i < nodes; ++i)
			z[i] = -gradient[i] - (i >= 1 ? l1[i - 1] * z[i - 1] : 0.0) - (i >= 2 ? l2[i - 2] * z[i - 2] : 0.0);
		for (std::size_t i = 0; i < nodes; ++i)
			z[i] /= d[i];
		for (auto i = nodes - 1; i-- > 0;)
			z[i] -= l1[i] * z[i + 1] + (i + 2 < nodes ? l2[i] * z[i + 2] : 0.0);
		return z;
	}
	// the length of the step along the Direction which lowers the cost from Current enough, halved until it does; the line it
	// leads to in Trial
	double step_along(std::vector<double> const &Direction, double const Current, double const Descent, std::vector<double> &Trial)
	{
		auto step_length{1.0};
		for (;; step_length *= 0.5)
		{
			for (std::size_t k = 0; k < nodes; ++k)
				Trial[k] = x[k] + step_length * Direction[k];
			if (cost(Trial, false) <= Current + 1e-4 * step_length * Descent || step_length < 1e-6)
				break;
		}
		return step_length;
	}
	// the line brought down to the least cost at the spread and the flat of the level, step by step of Newton
	void descend()
	{
		auto current{cost(x, true)};
		for (int pass = 0; pass < 100; ++pass)
		{
			auto const direction{newton()};
			auto descent{0.0};
			for (std::size_t k = 0; k < nodes; ++k)
				descent += gradient[k] * direction[k];
			std::vector<double> trial(nodes);
			auto const step_length{step_along(direction, current, descent, trial)};
			auto moved{0.0};
			for (std::size_t k = 0; k < nodes; ++k)
				moved = std::max(moved, std::abs(step_length * direction[k]));
			x = std::move(trial);
			current = cost(x, true);
			if (moved < 1e-5)
				break;
		}
	}
	// the spread and the flat narrow down level by level, each level going on from the line of the one before: from nearly
	// all the ground agreeing and changes of grade costing their square, to the tolerance and their size
	void solve()
	{
		spread = 256.0;
		flat = sharpest;
		for (;;)
		{
			descend();
			if (spread <= 1.0 && flat <= 1e-6)
				break;
			spread = std::max(1.0, spread * 0.5);
			flat = std::max(1e-6, flat * 0.5);
		}
	}

	// the run of the ground parted from the line comes to an end: followed where it was longer than a structure and not as far
	// nor as steep as a viaduct or a hill over a tunnel
	void close(parting &Parting)
	{
		if (Parting.side != 0 && chainages[Parting.run.back()] - chainages[Parting.run.front()] > options.follow_length && Parting.farthest <= options.follow_height && Parting.steepest <= options.follow_steepest / 1000.0)
		{
			follow_run(followed, Parting.run, Parting.farthest + tolerance);
			Parting.more = true;
		}
		Parting.side = 0;
		Parting.run.clear();
	}
	void begin_run(parting &Parting, int const Side) const
	{
		Parting.side = Side;
		Parting.back = 0;
		Parting.farthest = 0.0;
		Parting.steepest = 0.0;
		if (Parting.previous < count)
			Parting.run.push_back(Parting.previous);
	}
	void extend(parting &Parting, std::size_t const I) const
	{
		auto &run{Parting.run};
		while (Parting.back + 1 < run.size() && chainages[I] - chainages[run[Parting.back + 1]] >= chord)
			++Parting.back;
		if (false == run.empty() && chainages[I] - chainages[run[Parting.back]] >= chord)
			Parting.steepest = std::max(Parting.steepest, std::abs(ground[I] - ground[run[Parting.back]]) / (chainages[I] - chainages[run[Parting.back]]));
		run.push_back(I);
	}
	// the ground parted from the line farther along than a structure is long, not as far as a viaduct or a hill over a tunnel
	// keep it, and nowhere as steep as the banks and the walls under a bridge, is none: a hump, a rise, a hollow the line goes
	// over. It agrees with the line however far it is. The ground which tells the height no more closely than the tolerance
	// counts, the model, not the rail tops beside. True where any is followed anew
	bool parted_runs_followed()
	{
		parting parted;
		parted.previous = count;
		for (std::size_t i = 0; i < count; ++i)
		{
			if (false == std::isfinite(ground[i]) || told(i) < tolerance)
				continue;
			auto const off{off_of(x, i)};
			auto const now{side_of(i, off)};
			auto const changed{now != parted.side};
			if (changed)
				close(parted);
			if (changed && now != 0)
				begin_run(parted, now);
			if (now != 0)
			{
				parted.farthest = std::max(parted.farthest, std::abs(off));
				extend(parted, i);
			}
			parted.previous = i;
		}
		close(parted);
		return parted.more;
	}
	// the ground rising from the point a chord behind I along the Stretch too steeply for a bank; the point Behind brought up first
	bool steep_from(std::size_t const I, std::vector<std::size_t> const &Stretch, std::size_t &Behind) const
	{
		while (Behind + 1 < Stretch.size() && std::abs(chainages[I] - chainages[Stretch[Behind + 1]]) >= chord)
			++Behind;
		if (Stretch.empty())
			return false;
		auto const along{std::abs(chainages[I] - chainages[Stretch[Behind]])};
		return along >= chord && std::abs(ground[I] - ground[Stretch[Behind]]) / along > options.follow_steepest / 1000.0;
	}
	// a structure comes to an end. The ground parted from the line up to a loose end of it, however far, back to the first bank or
	// wall, is only the line going on at the grade it left the ground with, as six metres under a level field past a viaduct it
	// had to keep to. True where it is followed
	bool loose_end_followed(bool const Atend)
	{
		std::vector<std::size_t> stretch;
		std::size_t behind{0};
		int way{0};
		double most{0.0};
		for (std::size_t index = 0; index < count; ++index)
		{
			auto const i{Atend ? count - 1 - index : index};
			if (false == std::isfinite(ground[i]) || told(i) < tolerance)
				continue;
			auto const off{off_of(x, i)};
			auto const now{side_of(i, off)};
			// the ground coming to an end short of the end of the line: what's past it isn't ground, the line ends on a structure;
			// the ground parting the other way, or rising too steeply for a bank
			if ((stretch.empty() && std::abs(chainages[i] - chainages[Atend ? count - 1 : 0]) > chord) || now == 0 || (way != 0 && now != way) || steep_from(i, stretch, behind))
				break;
			way = now;
			most = std::max(most, std::abs(off));
			stretch.push_back(i);
		}
		if (stretch.empty() || std::abs(chainages[stretch.back()] - chainages[stretch.front()]) <= options.follow_length)
			return false;
		for (auto const i : stretch)
			followed[i] = most + tolerance;
		return true;
	}
	// the ground parted from the line as no structure would followed, and the line sought again, a few rounds
	void parted_followed()
	{
		for (int round = 0; round < 3 && options.follow_length > 0.0; ++round)
		{
			bool more{parted_runs_followed()};
			if (false == start_fixed && loose_end_followed(false))
				more = true;
			if (false == end_fixed && loose_end_followed(true))
				more = true;
			if (false == more)
				break;
			solve();
		}
	}

	// the change of grade at the node K
	double change(std::size_t const K) const
	{
		return (x[K - 1] - 2.0 * x[K] + x[K + 1]) / step;
	}
	double significant() const
	{
		return sharpest * 0.01;
	}
	bool bends(std::size_t const K) const
	{
		return std::abs(change(K)) >= significant();
	}
	// an end whose grade the line couldn't take over keeps only its height, the grade breaks at the joint
	void joins_kept(line &Result) const
	{
		if (start_fixed && std::abs((x[1] - x[0]) / step - line_context.start_grade) > significant())
			Result.join_start = false;
		if (end_fixed && std::abs((x[nodes - 1] - x[nodes - 2]) / step - line_context.end_grade) > significant())
			Result.join_end = false;
	}
	// the design keeps within half the tolerance of the line found, the one of the ground agreeing with it
	void limits_set()
	{
		limit.assign(nodes, tolerance);
		for (std::size_t i = 0; i < count; ++i)
			if (std::isfinite(ground[i]) && std::abs(ground[i] - (x[cell[i]] * (1.0 - part[i]) + x[cell[i] + 1] * part[i])) <= reach(i))
				for (auto const k : {cell[i], cell[i] + 1})
					limit[k] = std::min(limit[k], own(i));
		for (auto const &bound : bounds)
			for (auto const k : {bound.cell, bound.cell + 1})
				limit[k] = std::min(limit[k], std::max(0.001, (bound.high - bound.low) * 0.5));
	}
	double departure_of(line const &Line, double const From, double const To) const
	{
		auto worst{0.0};
		for (std::size_t k = 0; k < nodes; ++k)
			if (node(k) >= From && node(k) <= To)
				worst = std::max(worst, std::abs(elevation(Line, node(k)) - x[k]) / limit[k]);
		return worst;
	}
	// the points of the design: the changes of grade along a bend, one after another the same way, are one point of intersection
	// where the grades before and after them meet; a run of them is split before the nodes of the Splits
	bend_runs bend_points(line &Result, std::set<std::size_t> const &Splits) const
	{
		bend_runs found;
		Result.points.emplace_back(origin, x[0]);
		for (std::size_t k = 1; k + 1 < nodes;)
		{
			if (false == bends(k))
			{
				++k;
				continue;
			}
			auto const run_first{k};
			auto const sign{change(k) > 0.0};
			double sum{0.0};
			double moment{0.0};
			for (; k + 1 < nodes && bends(k) && (change(k) > 0.0) == sign && (k == run_first || false == Splits.contains(k)); ++k)
			{
				sum += change(k);
				moment += change(k) * node(k);
			}
			auto const at{moment / sum};
			pvi const point{at, x[run_first] + (x[run_first] - x[run_first - 1]) / step * (at - node(run_first))};
			auto const room{std::min(at - node(run_first) + step * 0.5, node(k - 1) + step * 0.5 - at)};
			Result.points.push_back(point);
			found.rooms.push_back(room);
			found.lengths.push_back(node(k - 1) - node(run_first) + step);
			found.runs.emplace_back(run_first, k);
		}
		Result.points.emplace_back(node(nodes - 1), x[nodes - 1]);
		found.rooms.push_back(0.0);
		found.lengths.push_back(0.0);
		found.runs.emplace_back(nodes, nodes);
		return found;
	}
	// the curve of the point I of the design, of the required radius where it has the Room, off the curves beside; where it hasn't,
	// as sharp as the room lets it be. On a hump as long as the bend of the line, Length, no sharper than the radius of the hump,
	// nor than a switch lets a convex one
	void curve_of(line &Result, std::size_t const I, double const Room, double const Length) const
	{
		auto const turn{std::abs(grade_after(Result, I) - grade_after(Result, I - 1))};
		auto const bare{turn * 1000.0 <= Result.norms.curve_threshold + 1e-6};
		// a change no curve needs breaks bare, but not on a switch
		if (auto const at{Result.points[I].chainage}; bare && turn > 1e-9 && on_switch(at, at))
		{
			Result.points[I].radius = std::min(required, 2.0 * std::max(0.0, Room - 1e-4) / turn);
			Result.points[I].automatic = false;
		}
		if (bare)
			return;
		auto const radius{least(Result.points[I].chainage)};
		auto bent{radius < required ? std::clamp(std::ceil(Length / turn / 10.0) * 10.0, radius, required) : required};
		if (auto const at{Result.points[I].chainage}; bent < switch_crest && grade_after(Result, I) < grade_after(Result, I - 1) && on_switch(at - bent * turn * 0.5, at + bent * turn * 0.5))
			bent = switch_crest;
		if (bent * turn * 0.5 > Room - 1e-4)
		{
			Result.points[I].radius = 2.0 * std::max(0.0, Room - 1e-4) / turn;
			Result.points[I].automatic = false;
		}
		else if (bent < required)
		{
			Result.points[I].radius = bent;
			Result.points[I].automatic = false;
		}
	}
	// a bend which one point can't keep to, as a long gentle one the curve of the required radius cuts across, is two, split by
	// its middle: true where one of the Runs of the design is split anew
	bool runs_split(line const &Result, std::vector<std::pair<std::size_t, std::size_t>> const &Runs, std::set<std::size_t> &Splits) const
	{
		auto split{false};
		for (std::size_t i = 1; i + 1 < Result.points.size(); ++i)
		{
			auto const [run_start, past]{Runs[i]};
			if (past - run_start < 2 || departure_of(Result, node(run_start - 1), node(past)) <= 0.5)
				continue;
			auto middle{run_start + 1};
			while (middle + 1 < past && node(middle) < Result.points[i].chainage)
				++middle;
			split = Splits.insert(middle).second || split;
		}
		return split;
	}
	// the design of the line found, Unset with its points: no change at a node is sharper than the required radius, so the curve
	// of that radius stays within the nodes of its bend; where the ends held the line sharper, the curve is as sharp as the line.
	// The bends split each tried again
	line designed(line const &Unset) const
	{
		line result;
		std::set<std::size_t> splits;
		for (;;)
		{
			result = Unset;
			auto const found{bend_points(result, splits)};
			for (std::size_t i = 1; i + 1 < result.points.size(); ++i)
				curve_of(result, i, found.rooms[i], found.lengths[i]);
			assign_radii(result);
			if (false == runs_split(result, found.runs, splits))
				break;
		}
		return result;
	}
	// a convex curve over a switch which the room of its bend cut sharper than a switch may lie on takes the radius it may have,
	// as far as the curves beside leave it room
	void switch_crests_widened(line &Result, std::ptrdiff_t &Faults) const
	{
		for (std::size_t i = 1; Faults > 0 && i + 1 < Result.points.size(); ++i)
		{
			auto const turn{grade_after(Result, i) - grade_after(Result, i - 1)};
			auto const radius{Result.points[i].radius};
			if (auto const at{Result.points[i].chainage}; radius <= 0.0 || radius >= switch_crest || turn >= 0.0 || false == on_switch(at + radius * turn * 0.5, at - radius * turn * 0.5))
				continue;
			auto wider{Result};
			wider.points[i].radius = switch_crest;
			wider.points[i].automatic = false;
			if (auto const fewer{errors_of(wider, line_context)}; fewer < Faults)
			{
				Result = std::move(wider);
				Faults = fewer;
			}
		}
	}
	// the designs one point simpler than the Result, a point dropped or two bending the same way made one, each with how far it
	// departs from the line found
	std::vector<std::pair<double, line>> simpler(line const &Result) const
	{
		auto const &points{Result.points};
		auto const last{points.size() - 1};
		std::vector<std::pair<double, line>> candidates;
		for (std::size_t i = 1; i < last; ++i)
		{
			auto const from{points[i >= 2 ? i - 2 : 0].chainage};
			auto dropped{Result};
			dropped.points.erase(dropped.points.begin() + i);
			assign_radii(dropped);
			candidates.emplace_back(departure_of(dropped, from, points[std::min(last, i + 2)].chainage), std::move(dropped));
			if (i + 1 >= last)
				continue;
			auto const before{grade_after(Result, i - 1)};
			auto const between{grade_after(Result, i)};
			auto const after{grade_after(Result, i + 1)};
			if ((between - before) * (after - between) <= 0.0)
				continue;
			auto const at{(points[i + 1].elevation - points[i].elevation + before * points[i].chainage - after * points[i + 1].chainage) / (before - after)};
			auto merged{Result};
			merged.points[i] = {at, points[i].elevation + before * (at - points[i].chainage)};
			merged.points.erase(merged.points.begin() + i + 1);
			assign_radii(merged);
			candidates.emplace_back(departure_of(merged, from, points[std::min(last, i + 3)].chainage), std::move(merged));
		}
		return candidates;
	}
	// then the simplest design which keeps within half the tolerance of the line found: a point is dropped, or two bending the
	// same way become one, as long as that departs the least and adds no fault
	void simplified(line &Result, std::ptrdiff_t &Faults) const
	{
		for (;;)
		{
			auto candidates{simpler(Result)};
			std::ranges::sort(candidates, [](auto const &A, auto const &B) { return A.first < B.first; });
			auto const chosen{std::ranges::find_if(candidates, [this, &Faults](auto const &Candidate) { return Candidate.first <= 0.5 && errors_of(Candidate.second, line_context) <= Faults; })};
			if (chosen == candidates.end())
				break;
			Result = std::move(chosen->second);
			Faults = errors_of(Result, line_context);
		}
	}
};

} // namespace

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
	if (Chainages[count - 1] - Chainages.front() < 1.0)
		return result;

	ground_fitter fitter{{Chainages, Ground, Tolerances, Reaches, count}, Context, Settings, Options, result};
	fitter.solve();
	fitter.parted_followed();
	fitter.joins_kept(result);
	fitter.limits_set();
	result = fitter.designed(result);
	auto faults{errors_of(result, Context)};
	fitter.switch_crests_widened(result, faults);
	joins_broken(result, faults, Context);
	// where the line can't bend from the grades of its ends into the ground within the norms, as a short piece between two
	// lines, the grades break at the joints: the line keeps only the heights of the ends
	if (faults > 0 && Options.hold_grades && (fitter.start_fixed || fitter.end_fixed))
	{
		auto loose{Options};
		loose.hold_grades = false;
		if (auto again{fit_ground(Chainages, Ground, Context, Settings, loose, Tolerances, Reaches)}; errors_of(again, Context) < faults)
			return again;
	}
	fitter.simplified(result, faults);
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

namespace
{

// the steepness of the Ground over a few metres, past the noise of the model, in and about the stretch From To, per mille
double steepest_about(std::vector<double> const &Chainages, std::vector<double> const &Ground, std::size_t const Count, double const From, double const To)
{
	double constexpr chord{5.0};
	double constexpr about{10.0};
	double most{0.0};
	for (std::size_t i = 0, j = 0; i < Count; ++i)
	{
		if (Chainages[i] < From - about || Chainages[i] > To + about || false == std::isfinite(Ground[i]))
			continue;
		j = std::max(j, i);
		while (j + 1 < Count && Chainages[j] - Chainages[i] < chord)
			++j;
		if (auto const run{Chainages[j] - Chainages[i]}; run >= chord && run <= 2.0 * chord && std::isfinite(Ground[j]))
			most = std::max(most, std::abs(Ground[j] - Ground[i]) / run);
	}
	return most * 1000.0;
}

} // namespace

std::vector<departure> bridged(std::vector<double> const &Chainages, std::vector<double> const &Heights, std::vector<double> const &Ground, double const Depth, double const Steepest)
{
	std::vector<departure> result;
	auto const count{std::min({Chainages.size(), Heights.size(), Ground.size()})};
	auto const under = [&](std::size_t const I) { return std::isfinite(Ground[I]) && Heights[I] - Ground[I] > Depth; };
	for (std::size_t i = 0; i < count;)
	{
		if (false == under(i))
		{
			++i;
			continue;
		}
		departure stretch;
		stretch.from = Chainages[i];
		for (; i < count && under(i); ++i)
		{
			stretch.depth = std::min(stretch.depth, Ground[i] - Heights[i]);
			stretch.to = Chainages[i];
		}
		if (steepest_about(Chainages, Ground, count, stretch.from, stretch.to) > Steepest)
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

std::vector<double> fewest_breaks(line const &Line, std::vector<double> Joints, double const Tolerance)
{
	return fewest_breaks(breaks(Line), std::move(Joints), Tolerance, [&Line](double const Chainage) { return elevation(Line, Chainage); }, [&Line](double const Chainage) { return grade(Line, Chainage); });
}

namespace
{

// whether the line of the Elevation keeps within the Tolerance from From to To of the cubic with its heights and its grades,
// taken a little inwards, at the ends
template <typename Heights, typename Grades>
bool cubic_follows(Heights const &Elevation, Grades const &Grade, double const Tolerance, double const From, double const To)
{
	auto const inward = [&Grade](double const Chainage, double const Other) { return Grade(Chainage + std::min(1e-3, std::abs(Other - Chainage) * 0.5) * (Other > Chainage ? 1.0 : -1.0)); };
	auto const length{To - From};
	auto const h0{Elevation(From)};
	auto const h1{Elevation(To)};
	auto const g0{inward(From, To) * length};
	auto const g1{inward(To, From) * length};
	auto const count{std::max(4, static_cast<int>(std::ceil(length)))};
	for (int i = 1; i < count; ++i)
	{
		auto const u{static_cast<double>(i) / count};
		auto const u2{u * u};
		auto const u3{u2 * u};
		auto const height{(2.0 * u3 - 3.0 * u2 + 1.0) * h0 + (u3 - 2.0 * u2 + u) * g0 + (-2.0 * u3 + 3.0 * u2) * h1 + (u3 - u2) * g1};
		if (std::abs(height - Elevation(From + u * length)) > Tolerance)
			return false;
	}
	return true;
}

// the fewest of the Stops past From, the last of them the end, at which the line breaks: each where no cubic from the break before
// follows it farther, Follows_cubic(from, to) telling, added to the Result
template <typename Follows>
void fewest_stops(std::vector<double> const &Stops, double From, Follows const &Follows_cubic, std::vector<double> &Result)
{
	std::size_t next{0};
	while (next + 1 < Stops.size())
	{
		auto reach{next};
		for (auto i{next + 1}; i < Stops.size() && Follows_cubic(From, Stops[i]); ++i)
			reach = i;
		if (reach + 1 == Stops.size())
			break;
		Result.push_back(Stops[reach]);
		From = Stops[reach];
		next = reach + 1;
	}
}

} // namespace

std::vector<double> fewest_breaks(std::vector<double> const &Breaks, std::vector<double> Joints, double const Tolerance, std::function<double(double)> const &Elevation, std::function<double(double)> const &Grade)
{
	std::ranges::sort(Joints);
	Joints.erase(std::ranges::unique(Joints, [](double const A, double const B) { return std::abs(A - B) < 1e-6; }).begin(), Joints.end());
	auto all{Breaks};
	std::ranges::sort(all);
	auto const follows = [&Elevation, &Grade, Tolerance](double const From, double const To) { return cubic_follows(Elevation, Grade, Tolerance, From, To); };
	std::vector<double> result;
	for (std::size_t k = 0; k + 1 < Joints.size(); ++k)
	{
		std::vector<double> stops;
		for (auto const chainage : all)
			if (chainage > Joints[k] + 1e-6 && chainage < Joints[k + 1] - 1e-6)
				stops.push_back(chainage);
		stops.push_back(Joints[k + 1]);
		fewest_stops(stops, Joints[k], follows, result);
	}
	return result;
}

std::string format_chainage(double const Chainage)
{
	auto const value{std::abs(Chainage)};
	auto const kilometres{static_cast<long>(value / 1000.0)};
	return format("%s%ld+%06.2f", Chainage < 0.0 ? "-" : "", kilometres, value - kilometres * 1000.0);
}

}
