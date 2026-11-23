/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutTurnout.hpp"
#include "editor/editorGeometry.hpp"
#include "editor/editorSpeed.hpp"
#include "editor/editorSwitchCurve.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace layout
{

namespace
{

double constexpr kStraight{1e-6}; // 1/m: a main track turning less is a straight, a radius of 1000 km
double constexpr kSample{0.5}; // m between the points of the tracks the rules of switch_curve look at
double constexpr kReach{20.0}; // m of the line looked at past the ends of a turnout, for the ends of the cant ramps (6 m off it)
double constexpr kLength{1e-6}; // m: the main track is as long as its type within this

// the catalogue, made once
std::vector<switch_design::design> const &catalogue()
{
	static std::vector<switch_design::design> const designs{switch_design::catalogue()};
	return designs;
}

std::optional<turnout_type> type_of(switch_design::design const &Design, std::string *Why)
{
	auto const refuse = [&](std::string const &Reason) -> std::optional<turnout_type> {
		if (Why != nullptr)
			*Why = Reason;
		return std::nullopt;
	};
	if (Design.type != "Rz")
		return refuse(std::format("{} is a {} switch, which the model doesn't lay yet (stage E5)", Design.name, switch_design::symmetric_switch(Design.type) ? "symmetric" : "double"));
	if (Design.entry > 0.0)
		return refuse(std::format("the blades of {} meet the stock rails at an angle, which the model doesn't lay yet (stage E5)", Design.name));
	std::string error;
	auto const shape{switch_design::build(Design, error)};
	if (false == error.empty() || shape.paths.size() < 2)
		return refuse(std::format("{}: {}", Design.name, error));
	turnout_type result;
	result.name = Design.name;
	result.rail = Design.rail;
	result.radius = shape.radius;
	result.ratio = shape.ratio;
	result.a = shape.a;
	result.b = shape.b;
	result.length = shape.length;
	result.lead = Design.lead;
	result.parts = Design.parts;
	result.movable = Design.movable;
	result.heel = Design.heel;
	result.bladethrow = Design.bladethrow;
	return result;
}

// the end of the line at the node
end_ref const *end_at(network const &Network, line const &Line, node_id const Node)
{
	if (Line.start == Node)
		return Network.end_of(Line.id, line_end::start);
	if (Line.end == Node)
		return Network.end_of(Line.id, line_end::end);
	return nullptr;
}

// the points of the line between the chainages, either way, the headings (radians, growing to the left) and the cants as it's run so;
// the stations from the last of the points so far, the headings going on from its heading
void sample(solved_line const &Line, double const From, double const To, std::vector<switch_curve::track_point> &Into)
{
	auto const way{To >= From ? 1.0 : -1.0};
	auto const span{std::abs(To - From)};
	auto const count{std::max(1, static_cast<int>(std::ceil(span / kSample)))};
	auto const offset{Into.empty() ? 0.0 : Into.back().station};
	for (int i = 0; i <= count; ++i)
	{
		if (i == 0 && false == Into.empty())
			continue;
		auto const s{From + way * span * i / count};
		auto const tangent{glm::normalize(Line.tangent(std::clamp(s, 0.0, Line.length))) * way};
		auto heading{std::atan2(tangent.y, tangent.x)};
		if (false == Into.empty())
		{
			auto const previous{Into.back().heading};
			heading = previous + std::remainder(heading - previous, 2.0 * 3.14159265358979323846);
		}
		Into.push_back({offset + span * i / count, heading, Line.cant_at(std::clamp(s, 0.0, Line.length)) * way});
	}
}

std::string turnout_name(turnout const &Turnout)
{
	return std::format("turnout {} ({})", number(Turnout.id), Turnout.type);
}

} // namespace

double turnout_type::diverging_length() const
{
	auto result{lead};
	for (auto const &part : parts)
		result += part.length;
	return result;
}

std::optional<turnout_type> turnout_type_of(std::string const &Name, std::string *Why)
{
	for (auto const &design : catalogue())
		if (design.name == Name)
			return type_of(design, Why);
	if (Why != nullptr)
		*Why = std::format("there's no turnout {} in the catalogue", Name);
	return std::nullopt;
}

std::vector<std::string> turnout_types()
{
	std::vector<std::string> result;
	for (auto const &design : catalogue())
		if (type_of(design, nullptr).has_value())
			result.push_back(design.name);
	return result;
}

turnout_frame frame_of(network const &Network, turnout const &Turnout)
{
	turnout_frame result;
	auto const *at{Network.find(Turnout.start)};
	auto const *main{Network.find(Turnout.main)};
	if (at == nullptr || main == nullptr || false == main->solved.valid())
		return result;
	auto const *held{end_at(Network, *main, Turnout.start)};
	if (held == nullptr)
		return result;
	auto const &solved{main->solved};
	result.side = held->side();
	result.along = held->end == line_end::start ? 1 : -1;
	result.position = at->position;
	result.heading = at->heading * static_cast<double>(result.side);
	// the mean of the main track: the curvature of an arc, the one of the middle of a clothoid
	auto const turn{geometry::signed_angle(solved.start.heading, solved.end.heading)};
	auto const curvature{result.along * turn / solved.length};
	result.curvature = std::abs(curvature) < kStraight ? 0.0 : curvature;
	result.cant = at->cant * result.side;
	result.grade = at->grade * result.side;
	result.valid = true;
	return result;
}

double bending_of(double const Curvature, int const Hand)
{
	if (std::abs(Curvature) < kStraight)
		return 0.0;
	return Curvature * Hand > 0.0 ? 1.0 / std::abs(Curvature) : -1.0 / std::abs(Curvature);
}

solved_line lay_diverging(turnout_frame const &Frame, turnout_type const &Type, int const Hand)
{
	solved_line result;
	auto const curvature{Frame.curvature};
	auto const bending{bending_of(curvature, Hand)};
	auto const hand{static_cast<double>(Hand)};
	auto origin{geometry::plan_of(Frame.position)};
	auto direction{Frame.heading};
	double chainage{0.0};
	auto const push = [&](element Element) {
		Element.origin = origin;
		Element.direction = direction;
		Element.chainage = chainage;
		origin = Element.position(Element.length);
		direction = glm::normalize(Element.tangent(Element.length));
		chainage += Element.length;
		result.elements.push_back(Element);
	};
	// along the main track, bent with it: the lead and the straights through the frog
	auto const along_main = [&](double const Length) {
		auto made{curvature == 0.0 ? make_straight({}, {1.0, 0.0}, Length) : make_arc({}, {1.0, 0.0}, curvature, Length)};
		made.role = element_role::straight;
		return made;
	};
	if (Type.lead > 1e-9)
		push(along_main(Type.lead));
	for (auto const &part : Type.parts)
	{
		auto const from{part.radius1 != 0.0 ? hand / part.radius1 : 0.0};
		auto const to{part.radius2 != 0.0 ? hand / part.radius2 : 0.0};
		if (from == 0.0 && to == 0.0)
		{
			push(along_main(part.length));
			continue;
		}
		if (from != to)
		{
			// a transition of the diverging track: the curvatures of the main track added
			push(make_spiral(element_kind::clothoid, {}, {1.0, 0.0}, from + curvature, to + curvature, part.length));
			continue;
		}
		auto bent{from + curvature};
		auto length{part.length};
		if (curvature != 0.0)
		{
			// the curve of the basic switch bent by table 11.8, an arc of another radius by the sum of the curvatures; as long as keeps the
			// turn off the main track, the angle of the frog
			if (std::abs(std::abs(part.radius1) - Type.radius) < 0.5)
			{
				auto const radius{switch_curve::bent_radius(bending, Type.radius, Type.ratio)};
				bent = radius != 0.0 ? hand / radius : 0.0;
			}
			auto const relative{bent - curvature};
			auto const turn{from * part.length};
			if (std::abs(relative) < 1e-12 || relative * turn <= 0.0)
			{
				result.errors.push_back(std::format("{}: bent into R {:.0f} m its diverging curve doesn't turn off the main track", Type.name, 1.0 / std::abs(curvature)));
				return result;
			}
			length = turn / relative;
		}
		push(std::abs(bent) < 1e-12 ? make_straight({}, {1.0, 0.0}, length) : make_arc({}, {1.0, 0.0}, bent, length));
	}
	if (result.elements.empty() || chainage <= 0.0)
	{
		result.errors.push_back(std::format("{}: no diverging track", Type.name));
		return result;
	}
	result.length = chainage;
	auto const height{Frame.position.y};
	auto const rise{Frame.grade * chainage};
	result.cant = {{0.0, chainage, Frame.cant, Frame.cant, ramp_shape::linear}};
	result.profile = {{0.0, height, 0.0, 0.0}, {chainage, height + rise, 0.0, 0.0}};
	auto const &first{result.elements.front()};
	auto const &last{result.elements.back()};
	result.start = {Frame.position, Frame.heading, first.curvature(0.0), Frame.cant, Frame.grade};
	result.end = {glm::dvec3{origin.x, height + rise, origin.y}, direction, last.curvature(last.length), Frame.cant, Frame.grade};
	return result;
}

void derive_turnout(network const &Network, turnout &Turnout)
{
	auto const type{turnout_type_of(Turnout.type)};
	auto const frame{frame_of(Network, Turnout)};
	auto const *main{Network.find(Turnout.main)};
	if (false == type.has_value() || false == frame.valid || main == nullptr)
		return;
	Turnout.movable = type->movable;
	Turnout.chainage = main->chainage_start + (frame.along > 0 ? 0.0 : main->solved.length);
	Turnout.main_radius = bending_of(frame.curvature, Turnout.hand);
	Turnout.cant = frame.cant * Turnout.hand;
	// the diverging track by the rule the editor writes
	auto const branch{switch_curve::branch_speed(type->radius, type->ratio, Turnout.main_radius, Turnout.cant, type->movable)};
	Turnout.speed_diverging = branch.speed;
	Turnout.diverging_radius = branch.radius;
	// the main track at the speed of its line, bent no faster than the deficiency of the frog lets it
	auto const vmax{main->category.vmax};
	Turnout.speed_main = vmax;
	if (frame.curvature != 0.0)
	{
		auto const into{frame.curvature > 0.0 ? frame.cant : -frame.cant};
		auto const formula{switch_curve::curve_speed(1.0 / std::abs(frame.curvature), into, branch.deficiency)};
		if (std::isfinite(formula))
			Turnout.speed_main = std::min(vmax, speed_check::suggested(formula));
	}
	Turnout.velocity = switch_curve::written_velocity(Turnout.speed_main, Turnout.speed_diverging);
}

void check_turnout(network const &Network, turnout const &Turnout, verdict &Result)
{
	auto const name{turnout_name(Turnout)};
	std::string why;
	auto const type{turnout_type_of(Turnout.type, &why)};
	if (false == type.has_value())
	{
		Result.reasons.push_back(std::format("{}: {}", name, why));
		return;
	}
	auto const *main{Network.find(Turnout.main)};
	auto const *diverging{Network.find(Turnout.diverging)};
	for (auto const leg : {Turnout.start, Turnout.main_end, Turnout.diverging_end})
		if (Network.find(leg) == nullptr)
			Result.reasons.push_back(std::format("{}: no node {} at a leg", name, number(leg)));
	if (main == nullptr || diverging == nullptr)
	{
		Result.reasons.push_back(std::format("{}: a track of it is missing", name));
		return;
	}
	if (main->turnout != Turnout.id || diverging->turnout != Turnout.id)
		Result.reasons.push_back(std::format("{}: a track of it is another's", name));
	auto const main_from{Turnout.direction > 0 ? main->start : main->end};
	auto const main_to{Turnout.direction > 0 ? main->end : main->start};
	if (main_from != Turnout.start || main_to != Turnout.main_end)
		Result.reasons.push_back(std::format("{}: its main track doesn't run from its start to the end of it", name));
	if (diverging->start != Turnout.start || diverging->end != Turnout.diverging_end)
		Result.reasons.push_back(std::format("{}: its diverging track doesn't run from its start to the end of it", name));
	if (Turnout.hand != 1 && Turnout.hand != -1)
		Result.reasons.push_back(std::format("{}: no hand", name));
	if (false == Result.reasons.empty() || false == main->solved.valid() || false == diverging->solved.valid())
		return;
	if (std::abs(main->solved.length - type->length) > kLength)
		Result.reasons.push_back(std::format("{}: its main track is {:.6f} m long, the type {:.6f} m", name, main->solved.length, type->length));
	// both tracks start at the start, on the one side of it
	auto const *main_held{end_at(Network, *main, Turnout.start)};
	auto const *diverging_held{Network.end_of(diverging->id, line_end::start)};
	if (main_held == nullptr || diverging_held == nullptr || main_held->side() != diverging_held->side())
		Result.reasons.push_back(std::format("{}: its tracks don't leave its start to one side", name));
	if (false == Result.reasons.empty())
		return;

	// the tracks from the start, and the line around it: the line it starts from, its main track, the line going on from its end
	auto const frame{frame_of(Network, Turnout)};
	std::vector<switch_curve::track_point> tracks[2];
	sample(main->solved, frame.along > 0 ? 0.0 : main->solved.length, frame.along > 0 ? main->solved.length : 0.0, tracks[0]);
	sample(diverging->solved, 0.0, diverging->solved.length, tracks[1]);
	std::vector<switch_curve::track_point> around;
	auto const *start{Network.find(Turnout.start)};
	for (auto const &end : start->ends)
	{
		if (end.side() == main_held->side())
			continue;
		if (auto const *before{Network.find(end.line)}; before != nullptr && before->solved.valid())
		{
			auto const length{before->solved.length};
			// run toward the start of the turnout
			if (end.end == line_end::end)
				sample(before->solved, std::max(0.0, length - kReach), length, around);
			else
				sample(before->solved, std::min(length, kReach), 0.0, around);
		}
		break;
	}
	auto const from{around.empty() ? 0.0 : around.back().station};
	sample(main->solved, frame.along > 0 ? 0.0 : main->solved.length, frame.along > 0 ? main->solved.length : 0.0, around);
	auto const to{around.back().station};
	if (auto const *at{Network.find(Turnout.main_end)})
		for (auto const &end : at->ends)
		{
			if (end.line == main->id)
				continue;
			if (auto const *after{Network.find(end.line)}; after != nullptr && after->solved.valid())
			{
				auto const length{after->solved.length};
				if (end.end == line_end::start)
					sample(after->solved, 0.0, std::min(length, kReach), around);
				else
					sample(after->solved, length, std::max(0.0, length - kReach), around);
			}
			break;
		}
	// the ratio left out: the rule of the steep switches is the model's own, which knows a main track from a side one
	auto const report{switch_curve::check(tracks[0], tracks[1], around, from, to, type->radius, 0.0)};
	auto const told = [&](std::string const &Text, norms::tier const Level, std::vector<notice> &Into) {
		notice found;
		found.line = main->id;
		found.chainage = frame.along > 0 ? 0.0 : main->solved.length;
		found.finding.level = Level;
		found.finding.chainage = found.chainage;
		found.text = std::format("{}: {}", name, Text);
		found.turnout = Turnout.id;
		Into.push_back(found);
	};
	for (auto const &refusal : report.refusals)
		told(refusal, norms::tier::block, Result.blocks);
	for (auto const &warning : report.warnings)
		told(warning, norms::tier::warn, Result.notices);
	// 11.4 pkt 4: in the running and the main tracks no switch of 1:9.403 nor steeper than 1:9 is bent
	auto const steep{type->ratio < 8.95 || std::abs(type->ratio - 9.403) < 0.05};
	if (Turnout.bent() && steep && main->category.track == norms::track_kind::main)
		told(std::format("bent into R {:.0f} m in a main track, where no switch of 1:9.403 nor steeper than 1:9 is bent (11.4 pkt 4)", std::abs(Turnout.main_radius)), norms::tier::block, Result.blocks);
	// laid on a transition curve: with the consent of the manager of the line (11.4 pkt 5)
	if (report.transition)
		told("its main track lies on a transition curve: only with the consent of the manager of the line (11.4)", norms::tier::warn, Result.notices);
}

} // namespace layout
