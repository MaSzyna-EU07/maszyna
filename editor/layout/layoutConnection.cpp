/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutConnection.hpp"
#include "editor/layout/layoutCommands.hpp"
#include "editor/layout/layoutTurnout.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSwitchCurve.hpp"
#include "editor/editorSwitchDesign.hpp"

#include <algorithm>
#include <cmath>
#include <format>
#include <limits>
#include <optional>

namespace layout
{

namespace
{

double constexpr kStep{1.0}; // m between the points of the first line the spacing is measured at
double constexpr kAlongside{0.2}; // rad: the lines run alongside where their headings differ by less
double constexpr kSolved{1e-13}; // rad: the line between the ends turns alike at both of them within this
int constexpr kIterations{40};

// the nearest point of the line to the point of the plan: its chainage
double nearest_on(solved_line const &Line, glm::dvec2 const &Point)
{
	auto best{0.0};
	auto nearest{std::numeric_limits<double>::max()};
	auto const count{std::max(2, static_cast<int>(std::ceil(Line.length / 2.0)))};
	for (int i = 0; i <= count; ++i)
	{
		auto const s{Line.length * i / count};
		if (auto const d{glm::distance(Line.position(s), Point)}; d < nearest)
		{
			nearest = d;
			best = s;
		}
	}
	for (int i = 0; i < 20; ++i)
	{
		auto const shift{glm::dot(Point - Line.position(best), glm::normalize(Line.tangent(best)))};
		best = std::clamp(best + shift, 0.0, Line.length);
		if (std::abs(shift) < 1e-12)
			break;
	}
	return best;
}

// the end of the diverging track of the type laid on a straight from the origin along x: how far along, how far aside, its angle
struct diverging_end
{
	double along{0.0};
	double aside{0.0};
	double angle{0.0};
	bool valid{false};
};

diverging_end end_of(std::string const &Type)
{
	diverging_end result;
	auto const type{turnout_type_of(Type)};
	if (false == type.has_value() || type->kind != turnout_kind::ordinary)
		return result;
	turnout_frame frame;
	frame.heading = {1.0, 0.0};
	frame.valid = true;
	auto const laid{lay_diverging(frame, *type, 1)};
	if (false == laid.valid())
		return result;
	result.along = laid.end.position.x;
	result.aside = laid.end.position.z;
	result.angle = geometry::signed_angle({1.0, 0.0}, laid.end.heading);
	result.valid = true;
	return result;
}

// the least of tab. 11.2 the catalogue gives the type, 0 where it gives none
double table_spacing(std::string const &Type)
{
	auto const &entries{switch_design::catalogue_entries()};
	auto const found{std::ranges::find_if(entries, [&](switch_design::design const &Each) { return Each.name == Type; })};
	return found != entries.end() ? found->spacing : 0.0;
}

// m, the least straight the norms ask between two turnouts on the line, at the speed through them: the normal limit of the profile, or the
// one it prefers where it has none, and the hard ones
double least_straight(network const &Network, line const &Line, glm::dvec2 const &At, double const Speed)
{
	norms::query where;
	where.speed = Speed;
	where.kind = Line.category.kind;
	where.track = Line.category.track;
	where.features = Line.category.vmax >= 160.0 ? norms::feature::fast_line : 0u;
	auto const profile{Line.profile.value_or(Network.areas.at({At.x, At.y}, {}))};
	double result{0.0};
	for (auto const &limits : {norms::limits(profile, norms::parameter::switch_straight, where), norms::hard_limits(norms::parameter::switch_straight, where)})
	{
		if (limits.warn.has_value())
			result = std::max(result, limits.warn->value);
		else if (limits.prefer.has_value())
			result = std::max(result, limits.prefer->value);
		if (limits.block.has_value())
			result = std::max(result, limits.block->value);
	}
	return result;
}

// the spacing of the type for the norms on the line: of tab. 11.2, or of its geometry with the least straight at its speed
double spacing_needed(network const &Network, line const &Line, glm::dvec2 const &At, std::string const &Type, double &Straight)
{
	auto const type{turnout_type_of(Type)};
	if (false == type.has_value())
		return 0.0;
	auto const speed{switch_curve::branch_speed(type->radius, type->ratio, 0.0, 0.0, type->movable).speed};
	Straight = least_straight(Network, Line, At, speed);
	return std::max(table_spacing(Type), crossover_spacing(Type, Straight));
}

// the type of the catalogue which fits the spacing: of the rail of the type asked for before the others, the fastest of them
std::string best_fitting(network const &Network, line const &Line, glm::dvec2 const &At, std::string const &Asked, double const Spacing)
{
	auto const asked{turnout_type_of(Asked)};
	std::string best;
	auto best_rank{-1.0};
	for (auto const &name : turnout_types())
	{
		auto const type{turnout_type_of(name)};
		if (type->kind != turnout_kind::ordinary || name.find("16/12") != std::string::npos)
			continue;
		double straight{0.0};
		if (spacing_needed(Network, Line, At, name, straight) > Spacing)
			continue;
		auto const speed{switch_curve::branch_speed(type->radius, type->ratio, 0.0, 0.0, type->movable).speed};
		auto const rank{(asked.has_value() && type->rail == asked->rail ? 1e7 : 0.0) + speed * 1e4 + type->radius};
		if (rank > best_rank)
		{
			best_rank = rank;
			best = name;
		}
	}
	return best;
}

// the second turnout laid on its line in a copy of the network with the first one: the end of its diverging track, its heading
struct second_end
{
	glm::dvec2 point{0.0};
	glm::dvec2 heading{0.0};
	std::vector<std::string> reasons;
};

second_end lay_second(network const &Trial, connect_tracks const &Request, double const Chainage, int const Hand, int const Direction)
{
	second_end result;
	model again{Trial};
	auto const made{again.execute(insert_turnout{Request.to, Chainage, Request.type, Hand, Direction})};
	if (false == made.done)
	{
		result.reasons = made.reasons;
		return result;
	}
	auto const *end{again.state().find(again.state().find(made.turnouts.front())->diverging_end)};
	result.point = geometry::plan_of(end->position);
	result.heading = end->heading;
	return result;
}

} // namespace

double crossover_spacing(std::string const &Type, double const Straight)
{
	auto const end{end_of(Type)};
	if (false == end.valid)
		return 0.0;
	return 2.0 * std::abs(end.aside) + Straight * std::sin(std::abs(end.angle));
}

connection_plan plan_connection(network const &Network, connect_tracks const &Request)
{
	connection_plan plan;
	auto const refuse = [&plan](std::string Why) {
		plan.reasons.push_back(std::move(Why));
		return plan;
	};
	auto const *from{Network.find(Request.from)};
	auto const *to{Network.find(Request.to)};
	if (from == nullptr || to == nullptr || from->turnout != turnout_id::none || to->turnout != turnout_id::none)
		return refuse("a crossover joins two lines of their own");
	if (Request.from == Request.to)
		return refuse("a crossover joins two lines, not a line to itself");
	if (false == from->solved.valid() || false == to->solved.valid())
		return refuse("the lines aren't laid");
	if (Request.direction != 1 && Request.direction != -1)
		return refuse("the first turnout runs along its line (1) or against it (-1)");
	std::string why;
	auto const type{turnout_type_of(Request.type, &why)};
	if (false == type.has_value())
		return refuse(why);
	if (type->kind != turnout_kind::ordinary)
		return refuse(std::format("a crossover is made of ordinary turnouts, not of {}", Request.type));
	auto const end{end_of(Request.type)};
	if (false == end.valid || std::abs(end.angle) < 1e-6)
		return refuse(std::format("{}: its diverging track doesn't leave the main one", Request.type));
	// the first turnout, toward the other line
	auto const &a{from->solved};
	auto const &b{to->solved};
	auto const s1{std::clamp(Request.chainage, 0.0, a.length)};
	auto const at{a.position(s1)};
	auto const heading{glm::normalize(a.tangent(s1)) * static_cast<double>(Request.direction)};
	auto const beside{nearest_on(b, at)};
	auto const there{b.position(beside)};
	auto const other{glm::normalize(b.tangent(beside))};
	if (std::abs(geometry::cross(heading, other)) > std::sin(kAlongside))
		return refuse(std::format("line {} doesn't run alongside line {} there", number(Request.to), number(Request.from)));
	plan.hand = geometry::cross(heading, there - at) > 0.0 ? 1 : -1;
	// the second one runs back toward the first: against the other line where it runs the way of the crossover
	auto const progress{glm::dot(other, heading) > 0.0 ? 1.0 : -1.0};
	plan.direction = progress > 0.0 ? -1 : 1;
	plan.hand2 = geometry::cross(other * static_cast<double>(plan.direction), at - there) > 0.0 ? 1 : -1;
	model trial{Network};
	auto const first{trial.execute(insert_turnout{Request.from, s1, Request.type, plan.hand, Request.direction})};
	if (false == first.done)
	{
		plan.reasons = first.reasons;
		return plan;
	}
	auto const &laid{*trial.state().find(first.turnouts.front())};
	auto const &end1{*trial.state().find(laid.diverging_end)};
	auto const p1{geometry::plan_of(end1.position)};
	auto const e1{end1.heading};
	// where the line between the ends turns by as much at both of them: on one straight between straight lines alongside
	auto const turning = [&](second_end const &Second) {
		auto const toward{glm::normalize(Second.point - p1)};
		return geometry::signed_angle(e1, toward) - geometry::signed_angle(toward, -Second.heading);
	};
	auto const beside_end{nearest_on(b, p1)};
	auto const lateral{glm::distance(p1, b.position(beside_end)) - std::abs(end.aside)};
	auto const guess{beside_end + progress * (lateral / std::tan(std::abs(end.angle)) + end.along)};
	std::vector<std::pair<double, double>> tried;
	auto const evaluate = [&](double const Chainage, second_end &Into) {
		Into = lay_second(trial.state(), Request, Chainage, plan.hand2, plan.direction);
		if (false == Into.reasons.empty())
			return false;
		tried.emplace_back(Chainage, turning(Into));
		return true;
	};
	second_end second;
	for (auto const chainage : {guess, guess + progress})
		if (false == evaluate(chainage, second))
		{
			plan.reasons = second.reasons;
			plan.reasons.insert(plan.reasons.begin(), std::format("the second turnout doesn't fit into line {} at {:.2f} m", number(Request.to), chainage));
			return plan;
		}
	// the secant along the other line, the turn nearly even in the chainage
	for (int i = 0; i < kIterations && std::abs(tried.back().second) > kSolved; ++i)
	{
		auto const &[s0, f0]{tried[tried.size() - 2]};
		auto const &[s1n, f1]{tried.back()};
		if (std::abs(f1 - f0) < 1e-18)
			break;
		auto const next{s1n - f1 * (s1n - s0) / (f1 - f0)};
		if (std::abs(next - s1n) < 1e-12)
			break;
		if (false == evaluate(next, second))
		{
			plan.reasons = second.reasons;
			plan.reasons.insert(plan.reasons.begin(), std::format("the second turnout doesn't fit into line {} at {:.2f} m", number(Request.to), next));
			return plan;
		}
	}
	plan.chainage = tried.back().first;
	plan.straight = glm::distance(second.point, p1);
	plan.turn = geometry::signed_angle(e1, -second.heading);
	// the spacing along the crossover: from the start of the first turnout as far as the start of the second one lies along the first line
	auto const reach{plan.straight * std::cos(std::abs(end.angle)) + 2.0 * end.along};
	plan.spacing_least = std::numeric_limits<double>::max();
	for (double s = 0.0; s <= reach + 1e-9; s += kStep)
	{
		auto const station{std::clamp(s1 + Request.direction * s, 0.0, a.length)};
		auto const point{a.position(station)};
		auto const spacing{glm::distance(point, b.position(nearest_on(b, point)))};
		plan.spacing_least = std::min(plan.spacing_least, spacing);
		plan.spacing_most = std::max(plan.spacing_most, spacing);
	}
	// the norms: the straight between the turnouts at the speed through them, the spacing of the type
	plan.straight_needed = least_straight(Network, *from, at, laid.speed_diverging);
	plan.spacing_needed = std::max(table_spacing(Request.type), crossover_spacing(Request.type, plan.straight_needed));
	if (plan.straight < plan.straight_needed - 1e-9)
		plan.reasons.push_back(std::format("the line between the turnouts is {:.2f} m long, the norms ask for {:.2f} m between two turnouts at {:.0f} km/h (ST-T1-A6 11.3 pkt 7)", plan.straight,
		                                   plan.straight_needed, laid.speed_diverging));
	if (plan.spacing_least < plan.spacing_needed - 1e-9)
		plan.reasons.push_back(std::format("the lines are {:.3f} m apart along the crossover (to {:.3f} m), {} needs {:.3f} m (ST-T1-A6 tab. 11.2 and the straight between its turnouts)",
		                                   plan.spacing_least, plan.spacing_most, Request.type, plan.spacing_needed));
	if (false == plan.reasons.empty())
	{
		auto const best{best_fitting(Network, *from, at, Request.type, plan.spacing_least)};
		if (best.empty())
			plan.proposal = std::format("no ordinary turnout of the catalogue fits lines {:.3f} m apart; {} needs them {:.3f} m apart", plan.spacing_least, Request.type, plan.spacing_needed);
		else
			plan.proposal = std::format("{} fits lines {:.3f} m apart, at {:.0f} km/h; {} needs them {:.3f} m apart", best, plan.spacing_least,
			                            switch_curve::branch_speed(turnout_type_of(best)->radius, turnout_type_of(best)->ratio, 0.0, 0.0, turnout_type_of(best)->movable).speed, Request.type,
			                            plan.spacing_needed);
		plan.reasons.push_back("proposed: " + plan.proposal);
		return plan;
	}
	plan.fits = true;
	return plan;
}

} // namespace layout
