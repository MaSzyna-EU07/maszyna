/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutAnchors.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <limits>

namespace layout
{

namespace
{

double constexpr kReach{30.0}; // m: an object farther from every line stands by none
double constexpr kCoarse{5.0}; // m between the points of a line the nearest one is first looked for among
double constexpr kAsNear{1.0}; // m: another line nearer than this more is as near
double constexpr kPoleReach{8.0}; // m in the plan from the end of a wire to the pole it hangs on
double constexpr kHigh{15.0}; // m above or below the line: an object so far off it hardly stands by it

// the nearest point of a line to an object
struct near_point
{
	line_id line{line_id::none};
	double chainage{0.0};
	double offset{0.0}; // m to the left of the line, below 0 to the right
	double height{0.0};
	double distance{0.0}; // in the plan
	bool inside{true}; // not past an end of the line
	glm::dvec2 heading{0.0, 1.0};
};

double heading_degrees(glm::dvec2 const &Heading)
{
	return glm::degrees(std::atan2(Heading.x, Heading.y));
}

// an angle in degrees within -180 and 180
double circular(double const Degrees)
{
	auto result{std::fmod(Degrees, 360.0)};
	if (result > 180.0)
		result -= 360.0;
	if (result <= -180.0)
		result += 360.0;
	return result;
}

// the box of the line in the plan, widened by the reach
bool within_reach(solved_line const &Line, glm::dvec2 const &Point)
{
	glm::dvec2 low{std::numeric_limits<double>::max()};
	glm::dvec2 high{std::numeric_limits<double>::lowest()};
	for (auto const &e : Line.elements)
		for (auto const t : {0.0, 0.5, 1.0})
		{
			auto const p{e.position(e.length * t)};
			low = glm::min(low, p);
			high = glm::max(high, p);
		}
	return Point.x > low.x - kReach && Point.x < high.x + kReach && Point.y > low.y - kReach && Point.y < high.y + kReach;
}

std::optional<near_point> nearest_on(line const &Line, glm::dvec3 const &Point)
{
	auto const &solved{Line.solved};
	auto const plan{geometry::plan_of(Point)};
	if (false == solved.valid() || false == within_reach(solved, plan))
		return std::nullopt;
	auto const steps{std::max(1, static_cast<int>(std::ceil(solved.length / kCoarse)))};
	double best{std::numeric_limits<double>::max()};
	double s{0.0};
	for (int i = 0; i <= steps; ++i)
	{
		auto const at{solved.length * i / steps};
		if (auto const d{glm::distance(solved.position(at), plan)}; d < best)
		{
			best = d;
			s = at;
		}
	}
	for (int step = 0; step < 16; ++step)
	{
		auto const tangent{glm::normalize(solved.tangent(s))};
		auto const next{std::clamp(s + glm::dot(plan - solved.position(s), tangent), 0.0, solved.length)};
		auto const moved{std::abs(next - s)};
		s = next;
		if (moved < 1e-9)
			break;
	}
	near_point result;
	result.line = Line.id;
	result.chainage = s;
	result.heading = glm::normalize(solved.tangent(s));
	auto const off{plan - solved.position(s)};
	result.offset = geometry::cross(result.heading, off);
	result.height = Point.y - solved.elevation(s);
	result.distance = glm::length(off);
	// past an end: the point lies along the line beyond it, not across it
	result.inside = std::abs(glm::dot(off, result.heading)) < 0.5;
	if (result.distance > kReach)
		return std::nullopt;
	return result;
}

// the lines around the point, the nearest first
std::vector<near_point> nearest_lines(network const &Network, glm::dvec3 const &Point)
{
	std::vector<near_point> result;
	for (auto const &[id, each] : Network.lines())
		if (auto const found{nearest_on(each, Point)})
			result.push_back(*found);
	std::ranges::sort(result, [](near_point const &A, near_point const &B) { return A.distance < B.distance; });
	return result;
}

// the distances from the line its kind of object stands at
struct band
{
	double low{0.0};
	double high{0.0};
};

band band_of(infra::category const Group)
{
	using enum infra::category;
	switch (Group)
	{
	case signal: return {0.0, 6.0}; // on a post to the side, or on a gantry over the track
	case sign: return {1.5, 8.0};
	case pole: return {1.5, 8.0};
	case catenary: return {0.0, 1.5}; // over the track, the stagger with it
	case equipment: return {0.0, 3.0}; // on the track itself
	case logic: return {0.0, 15.0};
	case environment: return {0.0, 10.0};
	case other:
	default: return {0.0, 15.0};
	}
}

// how well the place of the object along the line fits its kind: 1 within the distances its kind stands at, less nearer or farther
double fit_of(infra::category const Group, near_point const &Near, std::vector<std::string> &Reasons)
{
	auto const range{band_of(Group)};
	auto const d{Near.distance};
	double result{1.0};
	if (d < range.low)
		result = 1.0 - 0.5 * (range.low - d) / range.low;
	else if (d > range.high)
		result = std::max(0.0, 1.0 - (d - range.high) / range.high);
	if (result < 1.0)
		Reasons.push_back(std::format("{:.1f} m off the line, where one of its kind stands {:.1f} to {:.1f} m off", d, range.low, range.high));
	if (false == Near.inside)
	{
		result *= 0.3;
		Reasons.emplace_back("past the end of the line");
	}
	if (std::abs(Near.height) > kHigh)
	{
		result *= 0.3;
		Reasons.push_back(std::format("{:.1f} m {} the line", std::abs(Near.height), Near.height > 0.0 ? "above" : "below"));
	}
	return result;
}

// the old tracks naming the object which became lines: the first line they became
std::optional<std::pair<std::string, line_id>> named_by(lineside_object const &Object, std::map<std::string, line_id> const &Tracks)
{
	for (auto const &name : Object.tracks)
		if (auto const found{Tracks.find(name)}; found != Tracks.end())
			return *found;
	return std::nullopt;
}

anchor_point point_along(near_point const &Near)
{
	return {Near.chainage, Near.offset, Near.height};
}

// the anchor of the object, or none where no line is near
std::optional<anchor> anchor_of(network const &Network, lineside_object const &Object, std::map<std::string, line_id> const &Tracks)
{
	auto const around{nearest_lines(Network, Object.location)};
	if (around.empty())
		return std::nullopt;
	anchor result;
	result.kind = Object.kind;
	result.name = Object.name;
	result.key = Object.key.empty() ? std::string{"-"} : Object.key;
	result.group = Object.group;
	result.location = Object.location;
	result.yaw = Object.yaw;
	auto chosen{around.front()};
	double confidence{1.0};
	auto const named{named_by(Object, Tracks)};
	if (named.has_value())
	{
		auto const there{std::ranges::find_if(around, [&](near_point const &Each) { return Each.line == named->second; })};
		if (there == around.end())
		{
			result.reasons.push_back(std::format("named by the track {}, which isn't near it", named->first));
			confidence = 0.5 * fit_of(Object.group, chosen, result.reasons);
		}
		else
		{
			chosen = *there;
			result.reasons.push_back(std::format("named by the track {}", named->first));
			confidence = std::max(chosen.line == around.front().line ? 0.95 : 0.85, fit_of(Object.group, chosen, result.reasons));
		}
	}
	else
	{
		confidence = fit_of(Object.group, chosen, result.reasons);
		// another line as near, on the other side of the object or further along: which one it belongs to is a guess
		if (around.size() > 1 && around[1].inside && around[1].distance < chosen.distance + kAsNear)
		{
			confidence *= 0.5;
			result.reasons.push_back(std::format("line {} is as near, {:.1f} m off", number(around[1].line), around[1].distance));
		}
		if (Object.group == infra::category::environment)
		{
			confidence *= 0.5;
			result.reasons.emplace_back("not a part of the railway");
		}
	}
	result.line = chosen.line;
	result.at = point_along(chosen);
	result.turn = circular(Object.yaw - heading_degrees(chosen.heading));
	// a sign, a signal, a pole turned off the track and off square to it
	if (Object.kind == "model" && (Object.group == infra::category::sign || Object.group == infra::category::signal || Object.group == infra::category::pole))
	{
		auto const off_square{std::abs(circular(result.turn - 90.0 * std::round(result.turn / 90.0)))};
		if (off_square > 10.0)
		{
			confidence *= 0.8;
			result.reasons.push_back(std::format("turned {:.0f} degree off the track", off_square));
		}
	}
	auto const *line{Network.find(chosen.line)};
	for (auto const &point : Object.points)
	{
		auto const found{nearest_on(*line, point)};
		result.points.push_back(found.has_value() ? point_along(*found) : anchor_point{chosen.chainage, chosen.offset, point.y - line->solved.elevation(chosen.chainage)});
	}
	result.confidence = std::clamp(confidence, 0.0, 1.0);
	return result;
}

// the wires bound to the poles their ends hang on: a pole by the end of each, along the same line
void bind_wires(std::vector<anchor> &Anchors, std::vector<lineside_object> const &Objects, std::vector<std::size_t> const &Of)
{
	for (std::size_t a = 0; a < Anchors.size(); ++a)
	{
		auto &wire{Anchors[a]};
		auto const &object{Objects[Of[a]]};
		if (wire.kind != "traction" || object.points.size() != 4)
			continue;
		// p1 and p3 at one end, p2 and p4 at the other
		for (std::size_t end = 0; end < 2; ++end)
		{
			auto const tip{geometry::plan_of(object.points[end])};
			double best{kPoleReach};
			std::optional<std::uint32_t> pole;
			for (std::size_t b = 0; b < Anchors.size(); ++b)
			{
				auto const &other{Anchors[b]};
				// a pole, or a model of the catenary (a bracket, a gantry) the rules of the names take for one
				if ((other.group != infra::category::pole && other.group != infra::category::catenary) || other.kind != "model" || other.line != wire.line)
					continue;
				if (auto const d{glm::distance(geometry::plan_of(Objects[Of[b]].location), tip)}; d < best)
				{
					best = d;
					pole = other.id;
				}
			}
			if (pole.has_value())
				wire.poles.push_back(*pole);
		}
		if (wire.poles.size() == 2)
		{
			wire.confidence = std::max(wire.confidence, 0.9);
			wire.reasons.emplace_back("hangs on the poles at its ends");
		}
		else if (wire.poles.empty())
		{
			wire.confidence *= 0.7;
			wire.reasons.emplace_back("no pole at its ends");
		}
	}
}

glm::dvec3 point_at(solved_line const &Line, anchor_point const &At)
{
	auto const s{std::clamp(At.chainage, 0.0, Line.length)};
	auto const plan{Line.position(s) + left_of(glm::normalize(Line.tangent(s))) * At.offset};
	return {plan.x, Line.elevation(s) + At.height, plan.y};
}

// the anchor put again along the line nearest where it stands, its points with it
void anchor_again(network const &Network, anchor &Anchor, placement const &Where)
{
	auto const around{nearest_lines(Network, Where.location)};
	if (around.empty())
		return;
	auto const &chosen{around.front()};
	auto const *line{Network.find(chosen.line)};
	Anchor.line = chosen.line;
	Anchor.at = point_along(chosen);
	Anchor.turn = circular(Where.yaw - heading_degrees(chosen.heading));
	for (std::size_t i = 0; i < Anchor.points.size() && i < Where.points.size(); ++i)
		if (auto const found{nearest_on(*line, Where.points[i])})
			Anchor.points[i] = point_along(*found);
}

bool same_place(placement const &A, placement const &B)
{
	if (glm::distance(A.location, B.location) > 1e-4 || std::abs(circular(A.yaw - B.yaw)) > 1e-6 || A.points.size() != B.points.size())
		return false;
	for (std::size_t i = 0; i < A.points.size(); ++i)
		if (glm::distance(A.points[i], B.points[i]) > 1e-4)
			return false;
	return true;
}

} // namespace

std::vector<anchor> infer_anchors(network const &Network, std::vector<lineside_object> const &Objects, std::map<std::string, line_id> const &Tracks, std::uint32_t const First)
{
	std::vector<anchor> result;
	std::vector<std::size_t> of;
	auto next{First};
	for (std::size_t i = 0; i < Objects.size(); ++i)
	{
		auto made{anchor_of(Network, Objects[i], Tracks)};
		if (false == made.has_value())
			continue;
		made->id = next++;
		result.push_back(std::move(*made));
		of.push_back(i);
	}
	bind_wires(result, Objects, of);
	return result;
}

std::optional<placement> place(network const &Network, anchor const &Anchor)
{
	auto const *line{Network.find(Anchor.line)};
	if (line == nullptr || false == line->solved.valid())
		return std::nullopt;
	auto const &solved{line->solved};
	placement result;
	result.location = point_at(solved, Anchor.at);
	auto const s{std::clamp(Anchor.at.chainage, 0.0, solved.length)};
	result.yaw = circular(heading_degrees(glm::normalize(solved.tangent(s))) + Anchor.turn);
	for (auto const &point : Anchor.points)
		result.points.push_back(point_at(solved, point));
	return result;
}

std::vector<moved> follow(network const &Before, network const &After, std::vector<anchor> &Anchors)
{
	std::vector<moved> result;
	for (std::size_t i = 0; i < Anchors.size(); ++i)
	{
		auto &each{Anchors[i]};
		auto const was{place(Before, each)};
		if (After.find(each.line) == nullptr && was.has_value())
			anchor_again(After, each, *was);
		auto const now{place(After, each)};
		if (now.has_value() && (false == was.has_value() || false == same_place(*was, *now)))
			result.push_back({i, *now});
	}
	return result;
}

} // namespace layout
