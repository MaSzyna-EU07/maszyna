/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorConnections.hpp"
#include "editor/editorFormat.hpp"
#include "editor/editorGeometry.hpp"
#include "utilities/translation.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace connection
{

namespace
{

using geometry::bezier;
using geometry::cross;
using geometry::grade_of;
using geometry::has_controls;
using geometry::plan_of;
using geometry::signed_angle;

double constexpr kStraightTolerance{2e-3};
double constexpr kCrossingHalf{2.0};

struct branch
{
	glm::dvec3 end{0.0};
	glm::dvec2 heading{0.0, 1.0};
	double grade{0.0};
	double tail{0.0}; // straight at the end of the diverging track
};

bool plain(segment_data const &Piece)
{
	if (false == has_controls(Piece))
		return true;
	bezier const curve{Piece};
	auto const first{plan_of(curve.first(0.0))};
	auto const last{plan_of(curve.first(1.0))};
	if (glm::length(first) < 1e-9 || glm::length(last) < 1e-9)
		return true;
	return std::abs(signed_angle(first, last)) < 1e-6;
}

branch branch_of(std::vector<segment_data> const &Paths)
{
	branch result;
	if (Paths.size() < 2)
		return result;
	auto const &path{Paths[1]};
	result.end = path.points[segment_data::point::end];
	auto const &last{path.pieces.empty() ? path : path.pieces.back()};
	auto tangent{bezier{last}.first(1.0)};
	if (glm::length(plan_of(tangent)) < 1e-9)
		tangent = last.points[segment_data::point::end] - last.points[segment_data::point::start];
	result.heading = glm::normalize(plan_of(tangent));
	result.grade = grade_of(tangent);
	for (auto piece{path.pieces.rbegin()}; piece != path.pieces.rend() && plain(*piece); ++piece)
		result.tail += bezier{*piece}.plan_length();
	return result;
}

glm::dvec2 middle_point(placed const &Switch)
{
	auto const &main{Switch.paths[0]};
	auto const origin{plan_of(main.points[segment_data::point::start])};
	auto const axis{glm::normalize(plan_of(main.points[segment_data::point::end] - main.points[segment_data::point::start]))};
	auto const diverging{branch_of(Switch.paths)};
	auto const determinant{cross(diverging.heading, axis)};
	if (std::abs(determinant) < 1e-9)
		return origin;
	return origin + axis * (cross(diverging.heading, plan_of(diverging.end) - origin) / determinant);
}

glm::dvec3 start_of(placed const &Switch)
{
	return Switch.paths[0].points[segment_data::point::start];
}

bool fits(line const &Line, double const Along, int const Forward, double const Length)
{
	auto const from{Forward > 0 ? Along : Along - Length};
	return from >= -1e-3 && from + Length <= Line.length + 1e-3;
}

placed put(line const &Line, int const Index, double const Along, int const Forward, int const Side, editor_track::switch_template const &Shape, bool const Second)
{
	placed result;
	result.line = Index;
	result.along = Along;
	result.forward = Forward;
	result.side = Side;
	result.second = Second;
	result.paths = editor_track::place_switch(Shape, Line.point(Along), Line.direction * static_cast<double>(Forward), Side, Line.grade * Forward);
	return result;
}

segment_data straight(glm::dvec3 const &From, glm::dvec3 const &To)
{
	segment_data path;
	path.points[segment_data::point::start] = From;
	path.points[segment_data::point::end] = To;
	return path;
}

double sharper(editor_track::switch_template const &One, editor_track::switch_template const &Two)
{
	double result{0.0};
	for (auto const radius : {One.radius, Two.radius})
		if (radius > 1.0 && (result == 0.0 || radius < result))
			result = radius;
	return result > 0.0 ? result : 190.0;
}

struct crossing_over
{
	bool valid{false};
	bool straight{false};
	std::string error;
	std::string warning;
	placed one;
	placed two;
	std::vector<segment_data> inserts;
	std::vector<double> straights;
	double radius{0.0};
};

// the first switch on From diverging towards To and the switch on To facing it, joined by a straight insert when their angles
// agree, by an aligning curve between straight inserts when they don't, the switch on To then starting as near Target as it can
crossing_over cross_over(line const &From, int const Fromindex, double const Along, int const Forward, editor_track::switch_template const &Fromshape, bool const Fromsecond, line const &To, int const Toindex,
                         editor_track::switch_template const &Toshape, bool const Tosecond, double const Target, request const &Request)
{
	crossing_over result;
	if (false == fits(From, Along, Forward, Fromshape.length))
	{
		result.error = STR_C("A switch of the connection runs off the end of its straight");
		return result;
	}
	glm::dvec2 const direction{From.direction * static_cast<double>(Forward)};
	auto const origin{From.point(Along)};
	auto const nearest{To.point(To.along(origin))};
	auto const side{cross(direction, plan_of(nearest) - plan_of(origin)) > 0.0 ? 1 : -1};
	result.one = put(From, Fromindex, Along, Forward, side, Fromshape, Fromsecond);
	if (result.one.paths.size() < 2)
	{
		result.error = STR_C("The switch template has no diverging track");
		return result;
	}
	auto const diverging{branch_of(result.one.paths)};
	auto faced{face(diverging.end, diverging.heading, To, Toindex, Toshape)};
	if (faced.valid)
	{
		faced.partner.second = Tosecond;
		result.two = faced.partner;
		result.straight = true;
		if (faced.length >= 0.01)
			result.inserts.push_back(faced.insert);
		result.straights.push_back(faced.length);
		auto const curves{faced.length + diverging.tail + branch_of(result.two.paths).tail};
		if (curves < Request.insert - 1e-3)
			result.warning = format(STR_C("The straight between the curves of the switches is %.2f m, under %.0f m"), curves, Request.insert);
		result.valid = true;
		return result;
	}
	if (faced.partner.paths.empty())
	{
		result.error = faced.error;
		return result;
	}
	auto const forward{faced.partner.forward};
	auto const otherside{faced.partner.side};
	auto const radius{Request.radius > 0.0 ? Request.radius : sharper(Fromshape, Toshape)};
	auto const trial{branch_of(put(To, Toindex, 0.0, forward, otherside, Toshape, Tosecond).paths)};
	auto const into{-trial.heading};
	auto const turn{signed_angle(diverging.heading, into)};
	auto const determinant{cross(diverging.heading, into)};
	if (std::abs(determinant) < 1e-9 || std::abs(turn) > glm::radians(60.0))
	{
		result.error = STR_C("The switches can't face each other across these tracks");
		return result;
	}
	auto const tangent{radius * std::tan(std::abs(turn) * 0.5)};
	auto const offset{plan_of(trial.end) - plan_of(diverging.end)};
	auto const reach{cross(offset, into) / determinant};
	auto const after{cross(diverging.heading, offset) / determinant};
	auto const reachrate{cross(To.direction, into) / determinant};
	auto const afterrate{cross(diverging.heading, To.direction) / determinant};
	auto low{forward > 0 ? 0.0 : Toshape.length};
	auto high{forward > 0 ? To.length - Toshape.length : To.length};
	auto const bound = [&](double const Value, double const Rate, double const Need) {
		if (std::abs(Rate) < 1e-12)
		{
			if (Value - tangent < Need - 1e-6)
			{
				low = std::numeric_limits<double>::infinity();
				high = -low;
			}
			return;
		}
		auto const limit{(Need + tangent - Value) / Rate};
		if (Rate > 0.0)
			low = std::max(low, limit);
		else
			high = std::min(high, limit);
	};
	bound(reach, reachrate, std::max(0.0, Request.insert - diverging.tail));
	bound(after, afterrate, std::max(0.0, Request.insert - trial.tail));
	if (low > high + 1e-6)
	{
		result.error = format(STR_C("The aligning curve R %.0f m with %.0f m straights doesn't fit between the switches"), radius, Request.insert);
		return result;
	}
	result.two = put(To, Toindex, std::clamp(Target, low, std::max(low, high)), forward, otherside, Toshape, Tosecond);
	auto const arrived{branch_of(result.two.paths)};
	auto const link{editor_track::link_branch({diverging.end, diverging.heading, diverging.grade, 0.0}, radius, {arrived.end, -arrived.heading, -arrived.grade, 0.0}, 1e-4, Request.insert, false)};
	if (false == link.valid || link.pieces.empty() || link.trim != 0.0 || link.reverse)
	{
		result.error = format(STR_C("The aligning curve R %.0f m with %.0f m straights doesn't fit between the switches"), radius, Request.insert);
		return result;
	}
	result.inserts = link.pieces;
	result.straights = {link.before, link.after};
	result.radius = radius;
	result.valid = true;
	return result;
}

void take(result &Result, crossing_over const &Part, bool const Inserts)
{
	Result.switches.push_back(Part.one);
	Result.switches.push_back(Part.two);
	if (Inserts)
		Result.inserts.insert(Result.inserts.end(), Part.inserts.begin(), Part.inserts.end());
	Result.straights.insert(Result.straights.end(), Part.straights.begin(), Part.straights.end());
	if (Part.radius > 0.0)
		Result.radius = Part.radius;
	if (Result.warning.empty())
		Result.warning = Part.warning;
}

bool overlapping(std::vector<placed> const &Switches, std::vector<line> const &Lines)
{
	for (std::size_t i = 0; i < Switches.size(); ++i)
		for (std::size_t j = i + 1; j < Switches.size(); ++j)
		{
			auto const &one{Switches[i]};
			auto const &two{Switches[j]};
			if (one.line != two.line || one.line < 0 || one.line >= static_cast<int>(Lines.size()))
				continue;
			auto const &line{Lines[one.line]};
			auto const span = [&](placed const &Switch) {
				auto const a{line.along(Switch.paths[0].points[segment_data::point::start])};
				auto const b{line.along(Switch.paths[0].points[segment_data::point::end])};
				return std::pair{std::min(a, b), std::max(a, b)};
			};
			auto const [from1, to1]{span(one)};
			auto const [from2, to2]{span(two)};
			if (from2 < to1 - 0.01 && from1 < to2 - 0.01)
				return true;
		}
	return false;
}

} // namespace

char const *word(kind const Kind)
{
	switch (Kind)
	{
	case kind::double_crossover: return "double_crossover";
	case kind::scissors: return "scissors";
	case kind::ladder: return "ladder";
	default: return "crossover";
	}
}

bool parse(std::string const &Word, kind &Kind)
{
	for (int i = 0; i < kind_count; ++i)
		if (Word == word(static_cast<kind>(i)))
		{
			Kind = static_cast<kind>(i);
			return true;
		}
	return false;
}

glm::dvec3 line::point(double const Along) const
{
	return start + glm::dvec3{direction.x, grade, direction.y} * Along;
}

double line::along(glm::dvec3 const &Point) const
{
	return glm::dot(plan_of(Point) - plan_of(start), direction);
}

double line::lateral(glm::dvec3 const &Point) const
{
	return cross(direction, plan_of(Point) - plan_of(start));
}

line of(editor_track::straight const &Straight)
{
	line result;
	result.start = Straight.start;
	result.direction = Straight.direction;
	result.grade = Straight.grade;
	result.length = Straight.length;
	return result;
}

facing face(glm::dvec3 const &End, glm::dvec2 const &Heading, line const &To, int const Index, editor_track::switch_template const &Shape)
{
	facing result;
	auto const forward{glm::dot(To.direction, Heading) > 0.0 ? -1 : 1};
	glm::dvec2 const direction{To.direction * static_cast<double>(forward)};
	auto const side{cross(direction, plan_of(End) - plan_of(To.start)) > 0.0 ? 1 : -1};
	auto const trial{put(To, Index, 0.0, forward, side, Shape, true)};
	if (trial.paths.size() < 2)
	{
		result.error = STR_C("The switch template has no diverging track");
		return result;
	}
	auto const diverging{branch_of(trial.paths)};
	auto const mismatch{signed_angle(Heading, -diverging.heading)};
	if (std::abs(mismatch) > kStraightTolerance)
	{
		result.partner = trial;
		result.error = format(STR_C("The diverging tracks meet at %.2f deg, a straight insert can't join them"), glm::degrees(std::abs(mismatch)));
		return result;
	}
	auto const across{cross(Heading, To.direction)};
	if (std::abs(across) < 1e-9)
	{
		result.error = STR_C("The diverging track runs along the other track");
		return result;
	}
	auto const shift{-cross(Heading, plan_of(diverging.end) - plan_of(End)) / across};
	auto const meeting{plan_of(diverging.end) + To.direction * shift};
	result.length = glm::dot(Heading, meeting - plan_of(End));
	if (result.length < -0.005)
	{
		result.error = format(STR_C("The tracks are %.2f m too close for these switches"), -result.length * std::abs(across));
		return result;
	}
	result.length = std::max(0.0, result.length);
	if (false == fits(To, shift, forward, Shape.length))
	{
		result.error = STR_C("The switch on the other track runs off the end of its straight");
		return result;
	}
	result.partner = put(To, Index, shift, forward, side, Shape, true);
	result.insert = straight(End, result.partner.paths[1].points[segment_data::point::end]);
	result.valid = true;
	return result;
}

result plan(request const &Request)
{
	result out;
	auto const usable = [](editor_track::switch_template const &Shape) { return false == Shape.double_slip && false == Shape.twofold() && Shape.length > 0.5; };
	if (false == usable(Request.first) || false == usable(Request.second))
	{
		out.error = STR_C("A connection is made of ordinary switches, not of slips, crossings or double switches");
		return out;
	}
	auto const &from{Request.from};
	glm::dvec2 const travel{from.direction * static_cast<double>(Request.forward)};
	auto const origin{from.point(Request.along)};
	std::vector<line> lines{from};
	lines.insert(lines.end(), Request.to.begin(), Request.to.end());

	if (Request.type == kind::ladder)
	{
		auto side{Request.side >= 0 ? 1 : -1};
		std::vector<double> offsets;
		if (false == Request.to.empty())
		{
			auto const &first{Request.to.front()};
			side = cross(travel, plan_of(first.point(first.along(origin))) - plan_of(origin)) > 0.0 ? 1 : -1;
			for (auto const &fan : Request.to)
			{
				auto const offset{side * cross(travel, plan_of(fan.point(fan.along(origin))) - plan_of(origin))};
				if (offset < 1.0 || (false == offsets.empty() && offset < offsets.back() + 1.0) || std::abs(cross(fan.direction, from.direction)) > 1e-3)
				{
					out.error = STR_C("The tracks of a fan have to be parallel, on one side and at least 1 m apart");
					return out;
				}
				offsets.push_back(offset);
			}
		}
		else
		{
			double sum{0.0};
			for (auto const spacing : Request.spacings)
			{
				if (spacing < 1.0)
				{
					out.error = STR_C("The spacings of the tracks of a ladder have to be at least 1 m");
					return out;
				}
				offsets.push_back(sum += spacing);
			}
			if (offsets.empty())
			{
				out.error = STR_C("Give the spacings of the tracks of the ladder");
				return out;
			}
		}
		if (false == fits(from, Request.along, Request.forward, Request.first.length))
		{
			out.error = STR_C("A switch of the connection runs off the end of its straight");
			return out;
		}
		auto const start{put(from, 0, Request.along, Request.forward, side, Request.first, false)};
		auto const ladder{branch_of(start.paths)};
		auto const climb{side * cross(travel, ladder.heading)};
		if (climb < 1e-4)
		{
			out.error = STR_C("The switch template has no diverging track");
			return out;
		}
		out.switches.push_back(start);
		auto const lateral = [&](glm::dvec3 const &Point) { return side * cross(travel, plan_of(Point) - plan_of(origin)); };
		glm::dvec3 const step{ladder.heading.x, ladder.grade, ladder.heading.y};
		auto const rung = [&](double const Along) { return ladder.end + step * Along; };
		auto const back{cross(ladder.heading, travel) > 0.0 ? 1 : -1};
		auto const radius{Request.radius > 0.0 ? Request.radius : sharper(Request.second, Request.second)};
		auto const settle = [&](glm::dvec3 const &Point, glm::dvec2 const &Heading, double const Grade) {
			auto const turn{signed_angle(Heading, travel)};
			return std::abs(turn) > kStraightTolerance ? geometry::arc_pieces(Point, Heading, Grade, radius, std::abs(turn), turn > 0.0 ? 1 : -1) : std::vector<segment_data>{};
		};
		double reached{0.0};
		for (std::size_t i = 0; i < offsets.size(); ++i)
		{
			bool const last{i + 1 == offsets.size()};
			std::vector<segment_data> paths;
			std::vector<segment_data> bend;
			glm::dvec3 landing{0.0};
			if (false == last)
			{
				paths = editor_track::place_switch(Request.second, rung(0.0), ladder.heading, back, ladder.grade);
				auto const diverging{branch_of(paths)};
				bend = settle(diverging.end, diverging.heading, diverging.grade);
				landing = bend.empty() ? diverging.end : bend.back().points[segment_data::point::end];
			}
			else
			{
				bend = settle(rung(0.0), ladder.heading, ladder.grade);
				landing = bend.empty() ? rung(0.0) : bend.back().points[segment_data::point::end];
			}
			auto const along{(offsets[i] - lateral(landing)) / climb};
			if (along < reached - 1e-3)
			{
				out.error = format(STR_C("Track %d of the fan is %.2f m too close to the previous one for these switches"), static_cast<int>(i + 1), (reached - along) * climb);
				return out;
			}
			auto const shift{step * along};
			if (along - reached >= 0.01)
			{
				out.inserts.push_back(straight(rung(reached), rung(along)));
				out.straights.push_back(along - reached);
			}
			for (auto &path : paths)
				path.translate(shift);
			for (auto &piece : bend)
				piece.translate(shift);
			if (false == last)
			{
				placed rungswitch;
				rungswitch.line = -1;
				rungswitch.along = along;
				rungswitch.side = back;
				rungswitch.second = true;
				rungswitch.paths = paths;
				out.switches.push_back(rungswitch);
				reached = along + Request.second.length;
			}
			if (false == bend.empty())
				out.radius = radius;
			out.inserts.insert(out.inserts.end(), bend.begin(), bend.end());
			auto const fanstart{landing + shift};
			if (i < Request.to.size())
			{
				auto const &fan{Request.to[i]};
				auto end{fan.start};
				auto ahead{glm::dot(travel, plan_of(fan.start) - plan_of(fanstart))};
				auto const other{fan.point(fan.length)};
				if (auto const further{glm::dot(travel, plan_of(other) - plan_of(fanstart))}; further < ahead)
				{
					end = other;
					ahead = further;
				}
				if (ahead < -0.01)
				{
					out.error = format(STR_C("Track %d of the fan reaches into the ladder, it has to start %.2f m further on"), static_cast<int>(i + 1), -ahead);
					return out;
				}
				if (ahead >= 0.01)
					out.inserts.push_back(straight(fanstart, end));
			}
			else
				out.inserts.push_back(straight(fanstart, fanstart + glm::dvec3{travel.x, from.grade * Request.forward, travel.y} * std::max(1.0, Request.stub)));
		}
		out.fan = offsets;
		out.spacing = offsets.front();
		out.valid = true;
		return out;
	}

	if (Request.to.empty())
	{
		out.error = STR_C("Drag onto the other track");
		return out;
	}
	auto const &to{Request.to.front()};
	out.spacing = std::abs(to.lateral(origin));
	auto const one{cross_over(from, 0, Request.along, Request.forward, Request.first, false, to, 1, Request.second, true, Request.target, Request)};
	if (false == one.valid)
	{
		out.error = one.error;
		return out;
	}
	out.points = glm::distance(middle_point(one.one), middle_point(one.two));
	if (Request.type == kind::crossover)
	{
		take(out, one, true);
		out.valid = true;
		return out;
	}
	auto const towards{glm::dot(to.direction, travel) > 0.0 ? 1 : -1};
	auto const span{glm::dot(travel, plan_of(start_of(one.two)) - plan_of(start_of(one.one)))};
	if (Request.type == kind::double_crossover)
	{
		crossing_over two;
		if (false == Request.together)
		{
			auto const along{one.two.along + towards * std::max(0.0, Request.gap)};
			two = cross_over(to, 1, along, towards, Request.second, true, from, 0, Request.first, false, from.along(to.point(along)) + Request.forward * span, Request);
		}
		else
		{
			auto const along{Request.along - Request.forward * std::max(0.0, Request.gap)};
			two = cross_over(from, 0, along, -Request.forward, Request.first, false, to, 1, Request.second, true, to.along(from.point(along)) - towards * span, Request);
		}
		if (false == two.valid)
		{
			out.error = two.error;
			return out;
		}
		take(out, one, true);
		take(out, two, true);
		if (overlapping(out.switches, lines))
		{
			out.error = STR_C("The switches of the two crossovers overlap, the gap between them has to be longer");
			return out;
		}
		out.valid = true;
		return out;
	}
	if (false == one.straight || std::abs(cross(from.direction, to.direction)) > 1e-4)
	{
		out.error = STR_C("A scissors crossover needs parallel tracks and switches of the same angle");
		return out;
	}
	auto const p1{branch_of(one.one.paths).end};
	auto const p2{branch_of(one.two.paths).end};
	auto const middle{(p1 + p2) * 0.5};
	auto const two{cross_over(from, 0, 2.0 * from.along(middle) - Request.along, -Request.forward, Request.first, false, to, 1, Request.second, true, 0.0, Request)};
	if (false == two.valid || false == two.straight)
	{
		out.error = two.valid ? std::string{STR_C("A scissors crossover needs parallel tracks and switches of the same angle")} : two.error;
		return out;
	}
	take(out, one, false);
	take(out, two, false);
	if (overlapping(out.switches, lines))
	{
		out.error = STR_C("The tracks are too close for a scissors crossover with these switches, its switches overlap");
		return out;
	}
	auto const q1{branch_of(two.one.paths).end};
	auto const q2{branch_of(two.two.paths).end};
	auto const first{plan_of(p2 - p1)};
	auto const second{plan_of(q2 - q1)};
	auto const determinant{cross(first, second)};
	auto const sine{std::abs(cross(glm::normalize(travel), glm::length(first) > 1e-9 ? glm::normalize(first) : travel))};
	auto const shortfall{(2.0 * kCrossingHalf - one.straights.front()) * sine};
	if (std::abs(determinant) < 1e-9 || glm::length(first) < 2.0 * kCrossingHalf || glm::length(second) < 2.0 * kCrossingHalf)
	{
		out.error = format(STR_C("The tracks are too close for the crossing of a scissors crossover: %.2f m more spacing is needed"), std::max(0.01, shortfall));
		return out;
	}
	auto const fraction{cross(plan_of(q1) - plan_of(p1), second) / determinant};
	auto const meeting{plan_of(p1) + first * fraction};
	auto const half{std::min({glm::distance(meeting, plan_of(p1)), glm::distance(meeting, plan_of(p2)), glm::distance(meeting, plan_of(q1)), glm::distance(meeting, plan_of(q2))})};
	if (half < kCrossingHalf)
	{
		out.error = format(STR_C("The tracks are too close for the crossing of a scissors crossover: %.2f m more spacing is needed"), std::max(0.01, shortfall));
		return out;
	}
	out.crossing = true;
	out.ends = {p1, p2, q1, q2};
	out.centre = {meeting.x, p1.y + (p2.y - p1.y) * fraction, meeting.y};
	auto const unitone{glm::normalize(first)};
	auto unittwo{glm::normalize(second)};
	if (glm::dot(unitone, unittwo) < 0.0)
		unittwo = -unittwo;
	out.angle = std::atan2(std::abs(cross(unitone, unittwo)), glm::dot(unitone, unittwo));
	out.valid = true;
	return out;
}

} // namespace connection
