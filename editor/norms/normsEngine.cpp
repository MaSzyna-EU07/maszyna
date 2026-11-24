/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/norms/norms.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>

// the engine: one for all the profiles (editor-tracks-domain.md §6.3). Along the line it finds the cant deficiency at the highest speed
// and the cant excess at the speed of the slow trains, the abrupt changes of the cant deficiency at every jump of the curvature or of
// the cant, taken together over elements shorter than the profile lets them be taken apart, the gradient and the rate of change of
// the cant along the ramps, the rate of change of the cant deficiency along the transitions and the ramps, the lengths of the
// elements, the radii, the cant, the vertical curves and the grades
namespace norms
{

namespace
{

double constexpr kC{11.8};
double constexpr kSame{1e-6}; // m: points closer along the line are one
int constexpr kBlossPieces{8}; // a Bloss transition seen as this many pieces, as the cant ramps of the same shape are
double constexpr kCurve{1.0 / 50000.0}; // 1/m: a curve, not a straight
double constexpr kJumpCurvature{1e-6}; // 1/m
double constexpr kStep{0.5}; // mm: a smaller step of the cant isn't one
double constexpr kTwistBase{5.0}; // m: a step of the cant felt over the base of the twist, ST-T1-A6 tab. 9.12
double constexpr kBreak{0.002}; // a change of the grade up to 2 per mille needn't be rounded (ST-T1-A6 10.2 pkt 2, AB 17 7.1.3)

double sign(double const Value)
{
	if (Value > 0.0)
		return 1.0;
	return Value < 0.0 ? -1.0 : 0.0;
}

// the lateral acceleration in the track plane as mm of cant, signed with the curvature: 11.8 V^2 K - D
double lateral(double const Speed, double const Curvature, double const Cant)
{
	return kC * Speed * Speed * Curvature - Cant;
}

// I: above 0 the cant falls short of the curve
double deficiency(double const Speed, double const Curvature, double const Cant)
{
	if (std::abs(Curvature) < kCurve)
		return -std::abs(Cant);
	return sign(Curvature) * lateral(Speed, Curvature, Cant);
}

// E: above 0 the cant is more than the curve asks for at the speed; on a straight all of the cant
double excess(double const Speed, double const Curvature, double const Cant)
{
	if (std::abs(Curvature) < kCurve)
		return std::abs(Cant);
	return sign(Curvature) * Cant - kC * Speed * Speed * std::abs(Curvature);
}

// the cant at the chainage, before a step there or after it
double cant_at(std::vector<cant_point> const &Points, double const Chainage, bool const After)
{
	if (Points.empty())
		return 0.0;
	auto const first{std::ranges::lower_bound(Points, Chainage - kSame, {}, &cant_point::chainage)};
	auto const last{std::ranges::upper_bound(Points, Chainage + kSame, {}, &cant_point::chainage)};
	if (first != last)
		return After ? std::prev(last)->cant : first->cant;
	if (last == Points.begin())
		return Points.front().cant;
	if (first == Points.end())
		return Points.back().cant;
	auto const &before{*std::prev(first)};
	auto const &next{*first};
	auto const run{next.chainage - before.chainage};
	return run > kSame ? std::lerp(before.cant, next.cant, (Chainage - before.chainage) / run) : next.cant;
}

double curvature_at(element const &Element, double const Chainage)
{
	if (Element.length <= kSame)
		return Element.curvature_end;
	auto const t{std::clamp((Chainage - Element.start) / Element.length, 0.0, 1.0)};
	return std::lerp(Element.curvature_start, Element.curvature_end, Element.bloss ? t * t * (3.0 - 2.0 * t) : t);
}

// a point inside an element where the curvature and the cant are known from inside it
struct station
{
	double chainage;
	double curvature;
	double cant;
};

class checker
{
  public:
	checker(line const &Line, line_class const &Class, area_map const &Areas) : m_line{Line}, m_class{Class}, m_areas{Areas} {}

	std::vector<finding> run()
	{
		for (std::size_t i = 0; i < m_line.elements.size(); ++i)
			check_element(i);
		check_jumps();
		check_ramps();
		check_grades();
		return ordered();
	}

  private:
	line const &m_line;
	line_class const &m_class;
	area_map const &m_areas;
	std::vector<finding> m_found;

	double speed(element const &Element) const { return Element.speed > 0.0 ? Element.speed : m_class.vmax; }
	double slow(double const Speed) const
	{
		auto const vmin{m_class.vmin > 0.0 ? m_class.vmin : slow_speed(m_class.kind, m_class.vmax)};
		return std::min(vmin, Speed);
	}

	std::vector<profile_id> profiles(element const &Element) const
	{
		std::vector<profile_id> result{m_areas.at(Element.from, m_line.name)};
		if (auto const other{m_areas.at(Element.to, m_line.name)}; other != result.front())
			result.push_back(other);
		return result;
	}

	query where(element const &Element, double const Speed, double const Radius, unsigned const Extra = 0) const
	{
		query result;
		result.speed = Speed;
		result.radius = Radius;
		result.load = m_class.load;
		result.kind = m_class.kind;
		result.track = m_class.track;
		result.features = m_class.features | Element.features | Extra;
		return result;
	}

	// the element the chainage lies on, the one starting there at a joint
	std::size_t element_at(double const Chainage) const
	{
		auto const next{std::ranges::upper_bound(m_line.elements, Chainage + kSame, {}, &element::start)};
		return next == m_line.elements.begin() ? 0 : static_cast<std::size_t>(std::distance(m_line.elements.begin(), next) - 1);
	}

	// the value checked against each of the profiles, the worst kept
	void check(parameter const What, double const Value, std::size_t const Index, query const &Where, double const Chainage, double const Length = 0.0)
	{
		auto const &element{m_line.elements[Index]};
		std::optional<finding> worst;
		for (auto const profile : profiles(element))
		{
			auto const verdict{assess(What, Value, limits(profile, What, Where), hard_limits(What, Where), m_line.precision)};
			if (verdict.level == tier::none || (worst.has_value() && verdict.level <= worst->level))
				continue;
			finding found;
			found.what = What;
			found.level = verdict.level;
			found.value = Value;
			found.limit = verdict.passed.value;
			found.profile = profile;
			found.from = verdict.passed.from;
			found.confirmed = verdict.passed.confirmed;
			found.hard = verdict.hard;
			found.chainage = Chainage;
			found.length = Length;
			found.element = Index;
			found.speed = Where.speed;
			worst = found;
		}
		if (worst.has_value())
			m_found.push_back(*worst);
	}

	std::vector<station> stations(element const &Element) const
	{
		std::vector<station> result;
		auto const end{Element.end()};
		result.push_back({Element.start, Element.curvature_start, cant_at(m_line.cant, Element.start, true)});
		for (auto const &point : m_line.cant)
		{
			if (point.chainage <= Element.start + kSame || point.chainage >= end - kSame)
				continue;
			result.push_back({point.chainage, curvature_at(Element, point.chainage), point.cant});
		}
		// the curvature of a Bloss curve doesn't change evenly: eight pieces of it, for its steeper middle to be seen
		if (Element.bloss)
		{
			for (int i = 1; i < kBlossPieces; ++i)
			{
				auto const chainage{Element.start + Element.length * i / kBlossPieces};
				result.push_back({chainage, curvature_at(Element, chainage), cant_at(m_line.cant, chainage, true)});
			}
			std::ranges::sort(result, {}, &station::chainage);
		}
		result.push_back({end, Element.curvature_end, cant_at(m_line.cant, end, false)});
		return result;
	}

	// an arc meeting another curvature with no transition at one of its ends
	bool abrupt(std::size_t const Index) const
	{
		auto const &element{m_line.elements[Index]};
		if (element.kind != element_kind::arc)
			return false;
		auto const jump_before{Index > 0 && std::abs(m_line.elements[Index - 1].curvature_end - element.curvature_start) > kCurve};
		auto const jump_after{Index + 1 < m_line.elements.size() && std::abs(m_line.elements[Index + 1].curvature_start - element.curvature_end) > kCurve};
		return jump_before || jump_after;
	}

	// the greatest of a value along an element, and where
	struct peak
	{
		double value{0.0};
		double chainage{0.0};
		double length{0.0};

		void take(double const Value, double const Chainage, double const Length = 0.0)
		{
			if (Value <= value)
				return;
			value = Value;
			chainage = Chainage;
			length = Length;
		}
	};

	// a transition next to an arc reaches its radius and its cant deficiency only where the arc begins: the arc has them checked
	bool beside_arc(std::size_t const Index) const
	{
		auto const arc_at = [this](std::size_t const At) { return At < m_line.elements.size() && m_line.elements[At].kind == element_kind::arc; };
		return m_line.elements[Index].kind == element_kind::transition && ((Index > 0 && arc_at(Index - 1)) || arc_at(Index + 1));
	}

	void check_element(std::size_t const Index)
	{
		auto const &element{m_line.elements[Index]};
		auto const fast{speed(element)};
		auto const sharpest{std::max(std::abs(element.curvature_start), std::abs(element.curvature_end))};
		auto const radius{sharpest >= kCurve ? 1.0 / sharpest : 0.0};
		auto const here{where(element, fast, radius, abrupt(Index) ? feature::abrupt : 0u)};
		auto const points{stations(element)};
		auto const vmin{slow(fast)};
		peak cant;
		peak most_deficiency;
		peak most_excess;
		peak fastest_change;
		for (std::size_t i = 0; i < points.size(); ++i)
		{
			auto const &point{points[i]};
			cant.take(std::abs(point.cant), point.chainage);
			most_deficiency.take(deficiency(fast, point.curvature, point.cant), point.chainage);
			most_excess.take(excess(vmin, point.curvature, point.cant), point.chainage);
			if (i == 0)
				continue;
			// along the piece between two points the curvature and the cant change evenly, and so does the cant deficiency
			auto const &previous{points[i - 1]};
			auto const run{point.chainage - previous.chainage};
			if (run < 0.5)
				continue;
			auto const change{std::abs(lateral(fast, point.curvature, point.cant) - lateral(fast, previous.curvature, previous.cant))};
			fastest_change.take(change * fast / (3.6 * run), previous.chainage, run);
		}
		auto const own_curve{false == beside_arc(Index)};
		if (radius > 0.0 && own_curve)
			check(parameter::radius, radius, Index, here, element.start, element.length);
		if (cant.value > kStep)
			check(parameter::cant, cant.value, Index, here, cant.chainage);
		if (element.kind == element_kind::arc && cant.value > kStep)
			check(parameter::cant_min, cant.value, Index, here, element.start, element.length);
		if (most_deficiency.value > 0.0 && own_curve)
			check(parameter::cant_deficiency, most_deficiency.value, Index, here, most_deficiency.chainage);
		if (most_excess.value > 0.0 && own_curve)
		{
			// the limits by the speed band of the element, the excess at the speed of the slow trains
			auto const before{m_found.size()};
			check(parameter::cant_excess, most_excess.value, Index, here, most_excess.chainage);
			if (m_found.size() > before)
				m_found.back().speed = vmin;
		}
		if (fastest_change.value > 0.0)
			check(parameter::deficiency_rate, fastest_change.value, Index, here, fastest_change.chainage, fastest_change.length);
		check_lengths(Index, here);
		check_boundary(Index);
	}

	void check_lengths(std::size_t const Index, query const &Here)
	{
		auto const &element{m_line.elements[Index]};
		if (element.open_start || element.open_end)
			return;
		if (element.kind == element_kind::transition)
		{
			check(parameter::transition_length, element.length, Index, Here, element.start, element.length);
			// the arc moved in by a cubic parabola or a clothoid: L^2 / 24 times the change of the curvature (ST-T1-A6 tab. 9.11), by a
			// Bloss curve L^2 / 40
			auto const shift{element.length * element.length * std::abs(element.curvature_end - element.curvature_start) / (element.bloss ? 40.0 : 24.0)};
			check(parameter::transition_shift, shift, Index, Here, element.start, element.length);
			return;
		}
		check(parameter::element_length, element.length, Index, Here, element.start, element.length);
	}

	void check_boundary(std::size_t const Index)
	{
		auto const &element{m_line.elements[Index]};
		auto const both{profiles(element)};
		if (element.kind != element_kind::arc || both.size() < 2)
			return;
		finding found;
		found.what = parameter::profile_boundary;
		found.level = tier::warn;
		found.profile = both[0];
		found.other = both[1];
		found.chainage = element.start;
		found.length = element.length;
		found.element = Index;
		found.speed = speed(element);
		m_found.push_back(found);
	}

	// an abrupt change of the curvature or the cant at a chainage
	struct jump
	{
		double chainage{0.0};
		std::size_t element{0}; // the one after it
		double curvature_before{0.0};
		double curvature_after{0.0};
		double cant_before{0.0};
		double cant_after{0.0};
		double speed{0.0};
		unsigned features{0};
	};

	// the jump at a joint of two elements or at a step of the cant, none where the curvature and the cant go on there
	std::optional<jump> jump_at(double const Chainage) const
	{
		auto const index{element_at(Chainage)};
		auto const &after{m_line.elements[index]};
		jump step;
		step.chainage = Chainage;
		step.element = index;
		step.curvature_after = curvature_at(after, Chainage);
		step.curvature_before = step.curvature_after;
		step.speed = speed(after);
		step.features = after.features;
		if (index > 0 && std::abs(after.start - Chainage) < kSame)
		{
			// a train passes the joint no faster than either of the limits there
			auto const &before{m_line.elements[index - 1]};
			step.curvature_before = before.curvature_end;
			step.speed = std::min(step.speed, speed(before));
			step.features |= before.features & feature::switch_track;
		}
		step.cant_before = cant_at(m_line.cant, Chainage, false);
		step.cant_after = cant_at(m_line.cant, Chainage, true);
		if (std::abs(step.curvature_after - step.curvature_before) <= kJumpCurvature && std::abs(step.cant_after - step.cant_before) <= kStep)
			return std::nullopt;
		return step;
	}

	std::vector<jump> jumps() const
	{
		std::vector<double> places;
		for (std::size_t i = 1; i < m_line.elements.size(); ++i)
			places.push_back(m_line.elements[i].start);
		for (std::size_t i = 1; i < m_line.cant.size(); ++i)
			if (m_line.cant[i].chainage - m_line.cant[i - 1].chainage < kSame)
				places.push_back(m_line.cant[i].chainage);
		std::ranges::sort(places);
		auto const repeated{std::ranges::unique(places, [](double const A, double const B) { return B - A < kSame; })};
		places.erase(repeated.begin(), repeated.end());
		std::vector<jump> result;
		for (auto const place : places)
			if (auto const step{jump_at(place)})
				result.push_back(*step);
		return result;
	}

	// the shortest element between two jumps for them to be taken apart: the longest the profiles of the element between them ask for.
	// TSI INF and EN 13803 limit the jump but say nothing of jumps close together, so with only them there the longest any profile
	// asks for is taken: otherwise two jumps a metre apart would each pass a limit their sum breaks
	double intermediate(std::size_t const Index, double const Speed) const
	{
		auto const &element{m_line.elements[Index]};
		auto const longest = [this, &element, Speed](auto const &Profiles) {
			double result{0.0};
			for (auto const profile : Profiles)
				if (auto const bound{limits(profile, parameter::intermediate_length, where(element, Speed, 0.0)).block})
					result = std::max(result, bound->value);
			return result;
		};
		if (auto const own{longest(profiles(element))}; own > 0.0)
			return own;
		return longest(std::array{profile_id::plk, profile_id::tsi, profile_id::en13803, profile_id::ch});
	}

	// ST-T1-A6 9.11 pkt 8 - 13: the jumps closer than the intermediate length, changing the cant deficiency the same way, are taken
	// together, from the deficiency before the first to the one after the last; otherwise one by one
	void check_jumps()
	{
		auto const all{jumps()};
		std::size_t first{0};
		while (first < all.size())
		{
			auto last{first};
			auto fast{all[first].speed};
			auto features{all[first].features};
			auto const way{sign(lateral(fast, all[first].curvature_after, all[first].cant_after) - lateral(fast, all[first].curvature_before, all[first].cant_before))};
			while (last + 1 < all.size())
			{
				auto const &next{all[last + 1]};
				auto const speed_there{std::max(fast, next.speed)};
				auto const gap{next.chainage - all[last].chainage};
				auto const step{lateral(speed_there, next.curvature_after, next.cant_after) - lateral(speed_there, next.curvature_before, next.cant_before)};
				auto const between{lateral(speed_there, next.curvature_before, next.cant_before) - lateral(speed_there, all[last].curvature_after, all[last].cant_after)};
				auto const along{sign(between) == 0.0 || sign(between) == way};
				if (gap >= intermediate(element_at(all[last].chainage + gap * 0.5), speed_there) || sign(step) != way || false == along)
					break;
				fast = speed_there;
				features |= next.features;
				++last;
			}
			auto const before{lateral(fast, all[first].curvature_before, all[first].cant_before)};
			auto const after{lateral(fast, all[last].curvature_after, all[last].cant_after)};
			auto const extra{std::abs(before) <= 75.0 && std::abs(after) <= 75.0 ? feature::low_deficiency : 0u};
			auto const &element{m_line.elements[all[first].element]};
			query here{where(element, fast, 0.0, extra)};
			here.features = (here.features & ~feature::switch_track) | (features & feature::switch_track);
			check(parameter::deficiency_jump, std::abs(after - before), all[first].element, here, all[first].chainage, all[last].chainage - all[first].chainage);
			first = last + 1;
		}
	}

	// the speed of the fastest element over the stretch, and the element its middle lies on
	std::pair<double, std::size_t> over(double const From, double const To) const
	{
		double fastest{0.0};
		for (auto const &element : m_line.elements)
			if (element.end() > From + kSame && element.start < To - kSame)
				fastest = std::max(fastest, speed(element));
		auto const index{element_at((From + To) * 0.5)};
		if (fastest <= 0.0)
			fastest = speed(m_line.elements[index]);
		return {fastest, index};
	}

	// the ramps: between two points of the cant the gradient and the rate of change (ST-T1-A6 9.16, 9.18); a step of the cant as the
	// twist it makes over the base of 5 m
	void check_ramps()
	{
		if (m_line.elements.empty())
			return;
		for (std::size_t i = 1; i < m_line.cant.size(); ++i)
		{
			auto const &from{m_line.cant[i - 1]};
			auto const &to{m_line.cant[i]};
			auto const change{std::abs(to.cant - from.cant)};
			auto const run{to.chainage - from.chainage};
			if (change <= 1e-6)
				continue;
			auto const [fast, index]{over(from.chainage, std::max(to.chainage, from.chainage + kTwistBase))};
			auto const here{where(m_line.elements[index], fast, 0.0)};
			if (run < kSame)
			{
				if (change > kStep)
					check(parameter::cant_gradient, change / kTwistBase, index, here, from.chainage);
				continue;
			}
			auto const gradient{change / run};
			check(parameter::cant_gradient, gradient, index, here, from.chainage, run);
			check(parameter::cant_rate, gradient * fast / 3.6, index, here, from.chainage, run);
		}
	}

	// the grade line: the vertical curves, the breaks of the grade over 2 per mille with no curve, the grades
	void check_grades()
	{
		if (m_line.elements.empty())
			return;
		auto const &grades{m_line.grades};
		std::size_t i{0};
		while (i < grades.size())
		{
			auto const &piece{grades[i]};
			auto const [fast, index]{over(piece.start, piece.start + std::max(piece.length, kSame))};
			auto const held{piece.grade_start == piece.grade_end};
			// the grades held; a vertical curve goes between the ones on its sides
			if (held && piece.grade_start != 0.0)
				check(parameter::grade, std::abs(piece.grade_start) * 1000.0, index, where(m_line.elements[index], fast, 0.0, piece.length < 500.0 ? feature::short_stretch : 0u), piece.start, piece.length);
			if (i > 0)
			{
				auto const &previous{grades[i - 1]};
				auto const change{piece.grade_start - previous.grade_end};
				if (std::abs(change) > kBreak)
				{
					auto const at{element_at(piece.start)};
					check(parameter::vertical_radius, 0.0, at, where(m_line.elements[at], fast, 0.0, change < 0.0 ? feature::crest : feature::sag), piece.start);
				}
			}
			if (held)
			{
				++i;
				continue;
			}
			// a vertical curve, of the pieces in a row bending the same way with no break between them
			auto last{i};
			auto const way{sign(piece.grade_end - piece.grade_start)};
			auto sharpest{piece.length / std::abs(piece.grade_end - piece.grade_start)};
			while (last + 1 < grades.size())
			{
				auto const &next{grades[last + 1]};
				if (next.grade_start == next.grade_end || sign(next.grade_end - next.grade_start) != way || std::abs(next.grade_start - grades[last].grade_end) > 1e-9)
					break;
				sharpest = std::min(sharpest, next.length / std::abs(next.grade_end - next.grade_start));
				++last;
			}
			auto const total{grades[last].grade_end - piece.grade_start};
			auto const end{grades[last].start + grades[last].length};
			if (std::abs(total) > kBreak)
			{
				auto const [curve_speed, curve_index]{over(piece.start, end)};
				check(parameter::vertical_radius, sharpest, curve_index, where(m_line.elements[curve_index], curve_speed, 0.0, way < 0.0 ? feature::crest : feature::sag), piece.start, end - piece.start);
			}
			i = last + 1;
		}
	}

	// in the order of the line; each element has one finding of a parameter checked over it, the jumps, the ramps and the breaks of the
	// grade one each
	std::vector<finding> ordered() const
	{
		auto result{m_found};
		std::ranges::stable_sort(result, [](finding const &A, finding const &B) { return std::tie(A.chainage, A.what) < std::tie(B.chainage, B.what); });
		return result;
	}
};

// the point within the outline: a ray from it crosses its sides an odd number of times
bool inside(std::vector<plan_point> const &Outline, plan_point const &Point)
{
	bool result{false};
	auto previous{Outline.size() - 1};
	for (std::size_t i = 0; i < Outline.size(); ++i)
	{
		auto const &a{Outline[i]};
		auto const &b{Outline[previous]};
		if ((a.z > Point.z) != (b.z > Point.z) && Point.x < (b.x - a.x) * (Point.z - a.z) / (b.z - a.z) + a.x)
			result = !result;
		previous = i;
	}
	return result;
}

} // namespace

profile_id area_map::at(plan_point const &Point, std::string_view const Line) const
{
	if (false == Line.empty())
		for (auto const &[name, profile] : lines)
			if (name == Line)
				return profile;
	for (auto const &area : areas)
		if (area.outline.size() >= 3 && inside(area.outline, Point))
			return area.profile;
	return scenery;
}

std::vector<finding> evaluate(line const &Line, line_class const &Class, area_map const &Areas)
{
	if (Line.elements.empty())
		return {};
	return checker{Line, Class, Areas}.run();
}

} // namespace norms
