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
double constexpr kJoint{0.05}; // m: the slip curves end at least this far inside the joints of the slip

turnout_kind kind_of_type(std::string const &Type)
{
	if (switch_design::symmetric_switch(Type))
		return turnout_kind::symmetric;
	if (switch_design::double_switch(Type))
		return turnout_kind::twofold;
	if (Type == "Skrz")
		return turnout_kind::crossing;
	if (switch_design::slip_switch(Type))
		return turnout_kind::slip;
	return turnout_kind::ordinary;
}

// the slip or the crossing of the catalogue: its half length and its angle, the slip curves of the blades of the ordinary switch of the
// same rail, radius and angle, passing the crossing as the switch tool lays them on a standard gauge track
std::optional<turnout_type> slip_type_of(switch_design::design const &Design)
{
	turnout_type result;
	result.name = Design.name;
	result.rail = Design.rail;
	result.kind = kind_of_type(Design.type);
	result.radius = Design.radius;
	result.ratio = Design.ratio;
	result.half = Design.a;
	result.a = Design.a;
	result.b = Design.a;
	result.length = 2.0 * Design.a;
	result.angle = std::atan(1.0 / Design.ratio);
	if (result.kind == turnout_kind::slip)
	{
		auto const blade{switch_design::secant_blade(Design.rail, Design.radius, Design.ratio)};
		result.curve = switch_design::slip_of(Design.radius, Design.middle, result.angle, kSlipPass, blade.first, blade.second);
		result.curve.tip = switch_design::cut_tip(Design.rail, result.curve.radius);
		result.slip_curves = Design.type == "Rkp" ? 1 : 2;
		// the slip curves of the blades outside of the diamond passing the crossing as near as the rails of the profile of the simulator
		// let them may reach past the joints of Id-4: the slip as long as they are, as the switch tool lays it
		if (auto const tangent{result.curve.tangent()}; false == (tangent < result.half - kJoint))
		{
			result.half = tangent + kJoint;
			result.a = result.half;
			result.b = result.half;
			result.length = 2.0 * result.half;
		}
	}
	return result;
}

std::optional<turnout_type> type_of(switch_design::design const &Design, std::string *Why)
{
	auto const kind{kind_of_type(Design.type)};
	if (kind == turnout_kind::slip || kind == turnout_kind::crossing)
		return slip_type_of(Design);
	std::string error;
	auto const shape{switch_design::build(Design, error)};
	if (false == error.empty() || shape.paths.size() < 2)
	{
		if (Why != nullptr)
			*Why = std::format("{}: {}", Design.name, error);
		return std::nullopt;
	}
	turnout_type result;
	result.name = Design.name;
	result.rail = Design.rail;
	result.kind = kind;
	result.radius = shape.radius;
	result.ratio = shape.ratio;
	result.a = shape.a;
	result.b = shape.b;
	result.length = shape.length;
	result.lead = Design.lead;
	result.entry = glm::radians(Design.entry);
	result.tip = shape.tip;
	result.parts = Design.parts;
	result.movable = Design.movable;
	result.heel = Design.heel;
	result.bladethrow = Design.bladethrow;
	if (kind == turnout_kind::twofold)
	{
		// the second switch as switch_design::build() lays it: of its own parts, or the first one's; the radius the sharper one's
		auto const mirrored{Design.type == "Rps"};
		result.stagger = shape.stagger;
		result.lead2 = Design.lead2 >= 0.0 && false == mirrored ? Design.lead2 : Design.lead;
		result.entry2 = glm::radians(Design.entry2 >= 0.0 && false == mirrored ? Design.entry2 : Design.entry);
		result.parts2 = Design.parts2.empty() || mirrored ? Design.parts : Design.parts2;
		result.tip2 = shape.tip2 - shape.stagger;
		result.opposite = Design.type != "Rpj";
	}
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

// the elements of a track one after another from the origin along the direction, a turn between two where a secant blade meets the stock
// rail; the cant and the grade as given all along it
class track_builder
{
  public:
	track_builder(glm::dvec2 const &Origin, glm::dvec2 const &Direction) : m_origin{Origin}, m_direction{Direction} {}
	void push(element Element)
	{
		Element.origin = m_origin;
		Element.direction = m_direction;
		Element.chainage = m_chainage;
		m_origin = Element.position(Element.length);
		m_direction = glm::normalize(Element.tangent(Element.length));
		m_chainage += Element.length;
		m_result.elements.push_back(Element);
	}
	void straight(double const Length)
	{
		if (Length > 1e-9)
			push(make_straight({}, {1.0, 0.0}, Length));
	}
	void curve(double const Curvature, double const Length)
	{
		if (Length > 1e-9)
			push(std::abs(Curvature) < 1e-12 ? make_straight({}, {1.0, 0.0}, Length) : make_arc({}, {1.0, 0.0}, Curvature, Length));
	}
	void turn(double const Angle) { m_direction = geometry::turned(m_direction, Angle); }
	double chainage() const { return m_chainage; }
	// the track laid from the start, its heights evenly between the ones given
	solved_line finish(glm::dvec3 const &Start, glm::dvec2 const &Heading, double const Cant, double const Grade, double const EndHeight)
	{
		auto result{std::move(m_result)};
		result.length = m_chainage;
		result.cant = {{0.0, m_chainage, Cant, Cant, ramp_shape::linear}};
		result.profile = {{0.0, Start.y, 0.0, 0.0}, {m_chainage, EndHeight, 0.0, 0.0}};
		auto const &first{result.elements.front()};
		auto const &last{result.elements.back()};
		result.start = {Start, Heading, first.curvature(0.0), Cant, Grade};
		result.end = {glm::dvec3{m_origin.x, EndHeight, m_origin.y}, m_direction, last.curvature(last.length), Cant, Grade};
		return result;
	}

  private:
	glm::dvec2 m_origin;
	glm::dvec2 m_direction;
	double m_chainage{0.0};
	solved_line m_result;
};

// the frame at the node the main track goes on from
turnout_frame frame_at(network const &Network, node_id const Node, line_id const Main)
{
	turnout_frame result;
	auto const *at{Network.find(Node)};
	auto const *main{Network.find(Main)};
	if (at == nullptr || main == nullptr)
		return result;
	auto const *held{end_at(Network, *main, Node)};
	if (held == nullptr)
		return result;
	result.side = held->side();
	result.along = held->end == line_end::start ? 1 : -1;
	result.position = at->position;
	result.heading = at->heading * static_cast<double>(result.side);
	result.cant = at->cant * result.side;
	result.grade = at->grade * result.side;
	if (laid_by_turnout(Network, Main))
	{
		// a symmetric switch at the end of its line: as the node holds it
		result.curvature = at->curvature * result.side;
		result.valid = true;
		return result;
	}
	if (false == main->solved.valid())
		return result;
	auto const &solved{main->solved};
	// the mean of the main track: the curvature of an arc, the one of the middle of a clothoid
	auto const turn{geometry::signed_angle(solved.start.heading, solved.end.heading)};
	auto const curvature{result.along * turn / solved.length};
	result.curvature = std::abs(curvature) < kStraight ? 0.0 : curvature;
	result.main = &solved;
	result.main_from = result.along > 0 ? 0.0 : solved.length;
	// the curvature changing under it: on a transition curve, or across the end of one
	result.varying = std::ranges::any_of(solved.elements, [&](element const &Each) {
		return std::abs(Each.curvature_start - Each.curvature_end) > kStraight || std::abs(Each.curvature_start * result.along - result.curvature) > kStraight;
	});
	result.valid = true;
	return result;
}

// the main track of a turnout which isn't bent lies on a straight: every element of it
bool straight_line(line const &Line)
{
	return std::ranges::all_of(Line.solved.elements, [](element const &Each) { return std::abs(Each.curvature_start) < kStraight && std::abs(Each.curvature_end) < kStraight; });
}

// the slip curve from the point along the direction, turning to the side by the angle of the slip: along the track it leaves up to the
// curve, the curve, along the track it meets after it; to the point given at its end, which it reaches within the rounding
solved_line slip_curve_track(turnout_type const &Type, glm::dvec3 const &From, glm::dvec2 const &Direction, int const Side, glm::dvec3 const &To, glm::dvec2 const &Arrival, double const Cant)
{
	auto const &curve{Type.curve};
	auto const side{static_cast<double>(Side)};
	auto const straight{Type.half - curve.tangent()};
	track_builder build{geometry::plan_of(From), Direction};
	build.straight(straight + curve.lead);
	build.turn(side * curve.entry);
	build.curve(side / curve.radius, curve.ends);
	build.curve(curve.middle > 0.0 ? side / curve.middle : 0.0, curve.between);
	build.curve(side / curve.radius, curve.ends);
	build.turn(side * curve.entry);
	build.straight(curve.lead + straight);
	auto const length{build.chainage()};
	auto result{build.finish(From, Direction, Cant, length > 0.0 ? (To.y - From.y) / length : 0.0, To.y)};
	result.end.position = To;
	result.end.heading = Arrival;
	return result;
}

laid_tracks lay_slip(network const &Network, turnout const &Turnout, turnout_type const &Type)
{
	laid_tracks result;
	auto const frame{frame_of(Network, Turnout)};
	if (false == frame.valid)
	{
		result.errors.push_back(std::format("{}: no main track", Type.name));
		return result;
	}
	// in the plane of the main track: the grade along it, the crossing track rising by the grade of its projection on it
	auto const origin{geometry::plan_of(frame.position)};
	auto const along{frame.heading};
	auto const at = [&](glm::dvec2 const &Point) { return glm::dvec3{Point.x, frame.position.y + frame.grade * glm::dot(Point - origin, along), Point.y}; };
	auto const hand{static_cast<double>(Turnout.hand)};
	auto const centre{origin + along * Type.half};
	auto const across{geometry::turned(along, hand * Type.angle)};
	auto const cross_start{at(centre - across * Type.half)};
	auto const cross_end{at(centre + across * Type.half)};
	auto const crossing_grade{frame.grade * std::cos(Type.angle)};
	track_builder crossing{geometry::plan_of(cross_start), across};
	crossing.straight(2.0 * Type.half);
	laid_track track;
	track.line = Turnout.crossing;
	track.start = Turnout.cross_start;
	track.end = Turnout.cross_end;
	track.sets_start = true;
	track.sets_end = true;
	track.solved = crossing.finish(cross_start, across, frame.cant, crossing_grade, cross_end.y);
	track.solved.end.position = cross_end;
	result.tracks.push_back(track);
	if (Type.slip_curves >= 1)
	{
		// from the start to the end of the crossing track, turning to the hand
		laid_track slip;
		slip.line = Turnout.diverging;
		slip.start = Turnout.start;
		slip.end = Turnout.cross_end;
		slip.solved = slip_curve_track(Type, frame.position, along, Turnout.hand, cross_end, track.solved.end.heading, frame.cant);
		result.tracks.push_back(slip);
	}
	if (Type.slip_curves >= 2)
	{
		// from the start of the crossing track to the end of the main one, turning back
		laid_track slip;
		slip.line = Turnout.diverging2;
		slip.start = Turnout.cross_start;
		slip.end = Turnout.main_end;
		auto const *end{Network.find(Turnout.main_end)};
		auto const main_end{end != nullptr ? end->position : at(origin + along * Type.length)};
		slip.solved = slip_curve_track(Type, cross_start, across, -Turnout.hand, main_end, along, frame.cant);
		result.tracks.push_back(slip);
	}
	return result;
}

// the square of the tangent of the basic switch, t = r tan(a/2), of table 11.8
double tangent_squared(turnout_type const &Type)
{
	auto const tangent{Type.ratio > 0.0 ? Type.radius * std::tan(0.5 * std::atan(1.0 / Type.ratio)) : 0.0};
	return tangent * tangent;
}

// the curvature of the diverging track of the curvature Basic in the basic switch, its main track bent to Main, by table 11.8 written for
// the curvatures: r" = (R r -+ t^2) / (R +- r) is k" = (k + K) / (1 - k K t^2), the signs as they come; a straight goes with the main track
double bent_curvature(double const Basic, double const Main, double const Squared)
{
	return (Basic + Main) / (1.0 - Basic * Main * Squared);
}

// the diverging track on a main track whose curvature changes under the turnout, a transition curve: laid on in steps, each one's curvature
// at its ends by table 11.8 with the curvature of the main track at the station it runs by (the main track along the turnout, its heading off
// the main one), each part as long as keeps its turn off the main track; the straights and the lead with the main track
solved_line lay_on_transition(turnout_frame const &Frame, turnout_type const &Type, int const Hand)
{
	double constexpr kStep{2.0}; // m, the most a step of the diverging track runs on
	solved_line result;
	auto const hand{static_cast<double>(Hand)};
	auto const squared{tangent_squared(Type)};
	track_builder build{geometry::plan_of(Frame.position), Frame.heading};
	auto station{0.0}; // along the main track from the start of the turnout
	auto off{0.0}; // the heading of the diverging track off the main one's at the station
	// the curvature of the diverging track of the curvature in the basic switch at the station
	auto const bent = [&](double const Basic, double const Main) {
		if (Basic == 0.0)
			return Main;
		if (std::abs(std::abs(Basic) - 1.0 / Type.radius) < 1e-12)
		{
			auto const radius{switch_curve::bent_radius(bending_of(Main, Hand), Type.radius, Type.ratio)};
			return radius != 0.0 ? hand / radius : 0.0;
		}
		return bent_curvature(Basic, Main, squared);
	};
	auto const step = [&](double const From, double const To, double const Length) {
		build.push(std::abs(From - To) < 1e-15 ? (std::abs(From) < 1e-15 ? make_straight({}, {1.0, 0.0}, Length) : make_arc({}, {1.0, 0.0}, From, Length))
		                                       : make_spiral(element_kind::clothoid, {}, {1.0, 0.0}, From, To, Length));
	};
	// with the main track, as far
	auto const along_main = [&](double const Length) {
		auto const count{std::max(1, static_cast<int>(std::ceil(Length / kStep)))};
		for (int i = 0; i < count; ++i)
		{
			auto const ds{Length / count};
			auto const next{station + ds * std::cos(off)};
			step(Frame.curvature_at(station), Frame.curvature_at(next), ds);
			station = next;
		}
	};
	if (Type.lead > 1e-9)
		along_main(Type.lead);
	if (Type.entry > 0.0)
	{
		build.turn(hand * Type.entry);
		off += hand * Type.entry;
	}
	for (auto const &part : Type.parts)
	{
		auto const from{part.radius1 != 0.0 ? hand / part.radius1 : 0.0};
		auto const to{part.radius2 != 0.0 ? hand / part.radius2 : 0.0};
		if (from == 0.0 && to == 0.0)
		{
			along_main(part.length);
			continue;
		}
		// the turn it keeps off the main track, the curvature of the basic switch by the share of it made
		auto const turn{0.5 * (from + to) * part.length};
		auto const basic_at = [&](double const Made) {
			if (from == to)
				return from;
			// the curvature of the basic transition where it has turned by Made: k^2 = from^2 + 2 Made (to - from) / length
			auto const squared_k{from * from + 2.0 * Made * (to - from) / part.length};
			return std::copysign(std::sqrt(std::max(0.0, squared_k)), to != 0.0 ? to : from);
		};
		auto made{0.0};
		for (int guard = 0; guard < 10000 && std::abs(turn - made) > 1e-12 * std::abs(turn); ++guard)
		{
			auto const main0{Frame.curvature_at(station)};
			auto const k0{bent(basic_at(made), main0)};
			auto const rate0{k0 - main0 * std::cos(off)};
			if (rate0 * turn <= 0.0)
			{
				result.errors.push_back(std::format("{}: bent into R {:.0f} m its diverging curve doesn't turn off the main track", Type.name, 1.0 / std::max(1e-12, std::abs(main0))));
				return result;
			}
			// the step's turn by the rates at both its ends; the last one fitted to the turn left
			auto const last{(turn - made) / rate0 <= kStep};
			auto ds{last ? (turn - made) / rate0 : kStep};
			double k1{k0};
			double turned{0.0};
			for (int i = 0; i < 8; ++i)
			{
				auto const main1{Frame.curvature_at(station + ds * std::cos(off))};
				auto const reached{std::clamp(made + rate0 * ds, std::min(0.0, turn), std::max(0.0, turn))};
				k1 = bent(basic_at(reached), main1);
				auto const rate1{k1 - main1 * std::cos(off)};
				turned = 0.5 * (rate0 + rate1) * ds;
				if ((false == last && std::abs(made + turned) <= std::abs(turn)) || std::abs(turned) < 1e-15)
					break;
				ds *= (turn - made) / turned;
			}
			step(k0, k1, ds);
			station += ds * std::cos(off);
			off += turned;
			made = last ? turn : made + turned;
		}
	}
	if (build.chainage() <= 0.0)
	{
		result.errors.push_back(std::format("{}: no diverging track", Type.name));
		return result;
	}
	auto const rise{Frame.grade * build.chainage()};
	return build.finish(Frame.position, Frame.heading, Frame.cant, Frame.grade, Frame.position.y + rise);
}

// why the curvature of the track changes its sign along it, empty where it doesn't: turning to the hand and the other way
std::string sign_change_of(network const &Network, line_id const Id, int const Hand)
{
	auto const *track{Network.find(Id)};
	if (track == nullptr)
		return {};
	auto toward{0.0};
	auto away{0.0};
	for (auto const &each : track->solved.elements)
		for (auto const curvature : {each.curvature_start * Hand, each.curvature_end * Hand})
		{
			toward = std::max(toward, curvature);
			away = std::max(away, -curvature);
		}
	if (toward <= kStraight || away <= kStraight)
		return {};
	return std::format("the curvature of the diverging track changes its sign within the turnout (KZW II), R {:.0f} m to the side it leaves to and R {:.0f} m the other way, which isn't allowed "
	                   "(ST-T1-A6 9.7 pkt 12, 11.4 pkt 6)",
	                   1.0 / toward, 1.0 / away);
}

// the type of the second switch of a double one, as an ordinary switch is laid
turnout_type second_switch(turnout_type const &Type)
{
	auto result{Type};
	result.kind = turnout_kind::ordinary;
	result.lead = Type.lead2;
	result.entry = Type.entry2;
	result.parts = Type.parts2;
	result.tip = Type.tip2;
	return result;
}

double kilometrage_at_start(network const &Network, turnout const &Turnout, line const &Main, turnout_frame const &Frame)
{
	if (false == laid_by_turnout(Network, Main.id))
		return Main.chainage_start + (Frame.along > 0 ? 0.0 : Main.solved.length);
	// the end of the line it's laid at
	if (auto const *at{Network.find(Turnout.start)})
		for (auto const &end : at->ends)
			if (auto const *before{Network.find(end.line)}; before != nullptr && before->turnout == turnout_id::none)
				return before->chainage_start + (end.end == line_end::end ? before->solved.length : 0.0);
	return 0.0;
}

// the line runs between the nodes as the turnout has it: a part of the line it lies on along the turnout or against it as the turnout runs,
// a track it lays itself from its start
bool runs_between(line const &Line, node_id const From, node_id const To, int const Direction)
{
	return Direction > 0 ? Line.start == From && Line.end == To : Line.start == To && Line.end == From;
}

// the legs and the tracks of the turnout as its kind has them: each there and the turnout's, between its legs, laid; the main tracks as long
// as the type has them, straight where it isn't bent. False where they aren't whole, for the rules not to be looked at
bool check_layout(network const &Network, turnout const &Turnout, turnout_type const &Type, std::vector<std::string> &Reasons)
{
	auto const name{turnout_name(Turnout)};
	auto const count{Reasons.size()};
	auto const kind{Type.kind};
	struct between
	{
		line_id line;
		node_id from;
		node_id to;
		int direction; // 1: from its start to its end, else as the turnout runs along its line
	};
	std::vector<between> tracks;
	auto const direction{Turnout.direction};
	switch (kind)
	{
	case turnout_kind::symmetric:
		tracks = {{Turnout.main, Turnout.start, Turnout.main_end, 1}, {Turnout.diverging, Turnout.start, Turnout.diverging_end, 1}};
		break;
	case turnout_kind::twofold:
		tracks = {{Turnout.main, Turnout.start, Turnout.second, direction},
		          {Turnout.main2, Turnout.second, Turnout.main_end, direction},
		          {Turnout.diverging, Turnout.start, Turnout.diverging_end, 1},
		          {Turnout.diverging2, Turnout.second, Turnout.diverging2_end, 1}};
		break;
	case turnout_kind::slip:
		tracks = {{Turnout.main, Turnout.start, Turnout.main_end, direction}, {Turnout.crossing, Turnout.cross_start, Turnout.cross_end, 1}, {Turnout.diverging, Turnout.start, Turnout.cross_end, 1}};
		if (Type.slip_curves >= 2)
			tracks.push_back({Turnout.diverging2, Turnout.cross_start, Turnout.main_end, 1});
		break;
	case turnout_kind::crossing: tracks = {{Turnout.main, Turnout.start, Turnout.main_end, direction}, {Turnout.crossing, Turnout.cross_start, Turnout.cross_end, 1}}; break;
	case turnout_kind::ordinary:
	default: tracks = {{Turnout.main, Turnout.start, Turnout.main_end, direction}, {Turnout.diverging, Turnout.start, Turnout.diverging_end, 1}}; break;
	}
	for (auto const &track : tracks)
	{
		for (auto const leg : {track.from, track.to})
			if (Network.find(leg) == nullptr)
				Reasons.push_back(std::format("{}: no node {} at a leg", name, number(leg)));
		auto const *found{Network.find(track.line)};
		if (found == nullptr)
		{
			Reasons.push_back(std::format("{}: a track of it is missing", name));
			return false;
		}
		if (found->turnout != Turnout.id)
			Reasons.push_back(std::format("{}: a track of it is another's", name));
		if (false == runs_between(*found, track.from, track.to, track.direction))
			Reasons.push_back(std::format("{}: line {} doesn't run between the legs of its track", name, number(track.line)));
	}
	// the tracks of no other kind
	if (tracks_of(Turnout).size() != tracks.size())
		Reasons.push_back(std::format("{}: tracks of another kind of turnout", name));
	if (Turnout.hand != 1 && Turnout.hand != -1)
		Reasons.push_back(std::format("{}: no hand", name));
	if (Reasons.size() > count)
		return false;
	for (auto const &track : tracks)
		if (false == Network.find(track.line)->solved.valid())
			return false;
	auto const *main{Network.find(Turnout.main)};
	// the main tracks as long as the type has them
	auto const length_of = [&](line_id const Id, double const Length) {
		if (auto const *track{Network.find(Id)}; std::abs(track->solved.length - Length) > kLength)
			Reasons.push_back(std::format("{}: its main track is {:.6f} m long, the type {:.6f} m", name, track->solved.length, Length));
	};
	if (kind == turnout_kind::twofold)
	{
		length_of(Turnout.main, Type.stagger);
		length_of(Turnout.main2, Type.length - Type.stagger);
	}
	else if (kind != turnout_kind::symmetric)
		length_of(Turnout.main, Type.length);
	// both tracks start at the start, on the one side of it
	auto const *main_held{end_at(Network, *main, Turnout.start)};
	if (Turnout.diverging != line_id::none)
	{
		auto const *diverging_held{Network.end_of(Turnout.diverging, line_end::start)};
		if (main_held == nullptr || diverging_held == nullptr || main_held->side() != diverging_held->side())
			Reasons.push_back(std::format("{}: its tracks don't leave its start to one side", name));
	}
	// the kinds which aren't bent: their main tracks on straights, a symmetric switch at the end of a straight
	if (auto const bent{bent_refusal(Network, Turnout, Type)}; false == bent.empty())
		Reasons.push_back(std::format("{}: {}", name, bent));
	return Reasons.size() == count;
}

} // namespace

double turnout_frame::curvature_at(double const Along) const
{
	if (main == nullptr || false == varying)
		return curvature;
	auto const station{std::clamp(main_from + along * Along, 0.0, main->length)};
	return along * main->curvature(station);
}

double turnout_type::diverging_length() const
{
	auto result{lead};
	for (auto const &part : parts)
		result += part.length;
	return result;
}

std::optional<turnout_type> turnout_type_of(std::string const &Name, std::string *Why)
{
	for (auto const &design : switch_design::catalogue_entries())
		if (design.name == Name)
			return type_of(design, Why);
	if (Why != nullptr)
		*Why = std::format("there's no turnout {} in the catalogue", Name);
	return std::nullopt;
}

std::vector<std::string> turnout_types()
{
	std::vector<std::string> result;
	for (auto const &design : switch_design::catalogue_entries())
		if (type_of(design, nullptr).has_value())
			result.push_back(design.name);
	return result;
}

turnout_kind kind_of(turnout const &Turnout)
{
	if (Turnout.crossing != line_id::none)
		return Turnout.diverging != line_id::none ? turnout_kind::slip : turnout_kind::crossing;
	if (Turnout.second != node_id::none)
		return turnout_kind::twofold;
	auto const &entries{switch_design::catalogue_entries()};
	auto const found{std::ranges::find_if(entries, [&](switch_design::design const &Each) { return Each.name == Turnout.type; })};
	return found != entries.end() ? kind_of_type(found->type) : turnout_kind::ordinary;
}

std::string bent_refusal(network const &Network, turnout const &Turnout, turnout_type const &Type)
{
	if (Type.bendable())
		return {};
	auto bent{false};
	if (Type.kind == turnout_kind::symmetric)
		bent = std::abs(frame_of(Network, Turnout).curvature) >= kStraight;
	else
		for (auto const id : {Turnout.main, Turnout.main2})
			if (auto const *part{Network.find(id)}; part != nullptr && false == straight_line(*part))
				bent = true;
	if (false == bent)
		return {};
	char const *what{"slip nor crossing"};
	if (Type.kind == turnout_kind::twofold)
		what = "double switch";
	else if (Type.kind == turnout_kind::symmetric)
		what = "symmetric switch";
	return std::format("it lies in a curve, where no {} is laid (ST-T1-A6 11.4 pkt 4; table 11.8 bends the ordinary switches only)", what);
}

turnout_frame frame_of(network const &Network, turnout const &Turnout)
{
	return frame_at(Network, Turnout.start, Turnout.main);
}

turnout_frame second_frame_of(network const &Network, turnout const &Turnout)
{
	return frame_at(Network, Turnout.second, Turnout.main2);
}

double bending_of(double const Curvature, int const Hand)
{
	if (std::abs(Curvature) < kStraight)
		return 0.0;
	return Curvature * Hand > 0.0 ? 1.0 / std::abs(Curvature) : -1.0 / std::abs(Curvature);
}

solved_line lay_diverging(turnout_frame const &Frame, turnout_type const &Type, int const Hand)
{
	// a main track whose curvature changes under the turnout: each part bent by its curvature where it runs (table 11.8 taken exactly)
	if (Frame.varying)
		return lay_on_transition(Frame, Type, Hand);
	solved_line result;
	// the main track not laid: no place to lay the diverging one from, which would otherwise start at the origin and take its end
	// node there
	if (false == Frame.valid)
	{
		result.errors.push_back(std::format("{}: the main track isn't laid, so the diverging track has nowhere to start", Type.name));
		return result;
	}
	auto const curvature{Frame.curvature};
	auto const bending{bending_of(curvature, Hand)};
	auto const hand{static_cast<double>(Hand)};
	auto const squared{tangent_squared(Type)};
	track_builder build{geometry::plan_of(Frame.position), Frame.heading};
	// along the main track, bent with it: the lead and the straights through the frog
	auto const along_main = [curvature](double const Length) {
		auto made{curvature == 0.0 ? make_straight({}, {1.0, 0.0}, Length) : make_arc({}, {1.0, 0.0}, curvature, Length)};
		made.role = element_role::straight;
		return made;
	};
	if (Type.lead > 1e-9)
		build.push(along_main(Type.lead));
	// secant blades: the diverging track turns off the main one at their tips, the rest of the angle of the frog on its parts
	if (Type.entry > 0.0)
		build.turn(hand * Type.entry);
	for (auto const &part : Type.parts)
	{
		auto const from{part.radius1 != 0.0 ? hand / part.radius1 : 0.0};
		auto const to{part.radius2 != 0.0 ? hand / part.radius2 : 0.0};
		if (from == 0.0 && to == 0.0)
		{
			build.push(along_main(part.length));
			continue;
		}
		if (from != to)
		{
			// a transition of the diverging track: on a straight as it is; bent, its curvatures at its ends by table 11.8, as long as keeps
			// its turn off the main track
			if (curvature == 0.0)
			{
				build.push(make_spiral(element_kind::clothoid, {}, {1.0, 0.0}, from, to, part.length));
				continue;
			}
			auto const start{bent_curvature(from, curvature, squared)};
			auto const end{bent_curvature(to, curvature, squared)};
			auto const relative{0.5 * (start + end) - curvature};
			auto const turn{0.5 * (from + to) * part.length};
			if (std::abs(relative) < 1e-12 || relative * turn <= 0.0)
			{
				result.errors.push_back(std::format("{}: bent into R {:.0f} m its diverging curve doesn't turn off the main track", Type.name, 1.0 / std::abs(curvature)));
				return result;
			}
			build.push(make_spiral(element_kind::clothoid, {}, {1.0, 0.0}, start, end, turn / relative));
			continue;
		}
		auto bent{from};
		auto length{part.length};
		if (curvature != 0.0)
		{
			// the curve of the basic switch bent by table 11.8, an arc of another radius by its formula with the tangent of the basic switch;
			// as long as keeps the turn off the main track, the angle of the frog
			if (std::abs(std::abs(part.radius1) - Type.radius) < 0.5)
			{
				auto const radius{switch_curve::bent_radius(bending, Type.radius, Type.ratio)};
				bent = radius != 0.0 ? hand / radius : 0.0;
			}
			else
				bent = bent_curvature(from, curvature, squared);
			auto const relative{bent - curvature};
			auto const turn{from * part.length};
			if (std::abs(relative) < 1e-12 || relative * turn <= 0.0)
			{
				result.errors.push_back(std::format("{}: bent into R {:.0f} m its diverging curve doesn't turn off the main track", Type.name, 1.0 / std::abs(curvature)));
				return result;
			}
			length = turn / relative;
		}
		build.push(std::abs(bent) < 1e-12 ? make_straight({}, {1.0, 0.0}, length) : make_arc({}, {1.0, 0.0}, bent, length));
	}
	if (build.chainage() <= 0.0)
	{
		result.errors.push_back(std::format("{}: no diverging track", Type.name));
		return result;
	}
	auto const rise{Frame.grade * build.chainage()};
	return build.finish(Frame.position, Frame.heading, Frame.cant, Frame.grade, Frame.position.y + rise);
}

laid_tracks lay_tracks(network const &Network, turnout const &Turnout, turnout_type const &Type)
{
	laid_tracks result;
	auto const add = [&result](line_id const Line, node_id const Start, node_id const End, solved_line Solved) {
		laid_track track;
		track.line = Line;
		track.start = Start;
		track.end = End;
		track.sets_end = true;
		for (auto const &error : Solved.errors)
			result.errors.push_back(error);
		track.solved = std::move(Solved);
		result.tracks.push_back(std::move(track));
	};
	switch (Type.kind)
	{
	case turnout_kind::slip:
	case turnout_kind::crossing: return lay_slip(Network, Turnout, Type);
	case turnout_kind::symmetric:
	{
		// both tracks to both sides of the line it ends, each by half of the angle
		auto const frame{frame_of(Network, Turnout)};
		add(Turnout.main, Turnout.start, Turnout.main_end, lay_diverging(frame, Type, -Turnout.hand));
		add(Turnout.diverging, Turnout.start, Turnout.diverging_end, lay_diverging(frame, Type, Turnout.hand));
		return result;
	}
	case turnout_kind::twofold:
		add(Turnout.diverging, Turnout.start, Turnout.diverging_end, lay_diverging(frame_of(Network, Turnout), Type, Turnout.hand));
		add(Turnout.diverging2, Turnout.second, Turnout.diverging2_end, lay_diverging(second_frame_of(Network, Turnout), second_switch(Type), Type.opposite ? -Turnout.hand : Turnout.hand));
		return result;
	case turnout_kind::ordinary:
	default:
		add(Turnout.diverging, Turnout.start, Turnout.diverging_end, lay_diverging(frame_of(Network, Turnout), Type, Turnout.hand));
		return result;
	}
}

std::vector<line_id> tracks_of(turnout const &Turnout)
{
	std::vector<line_id> result;
	for (auto const id : {Turnout.main, Turnout.main2, Turnout.diverging, Turnout.diverging2, Turnout.crossing})
		if (id != line_id::none)
			result.push_back(id);
	return result;
}

bool laid_by_turnout(network const &Network, line_id const Line)
{
	auto const *owner{owner_of(Network, Line)};
	if (owner == nullptr)
		return false;
	if (Line == owner->diverging || Line == owner->diverging2 || Line == owner->crossing)
		return true;
	// the main track of a symmetric switch is one of its two curved tracks, laid at the free end of its line
	return Line == owner->main && kind_of(*owner) == turnout_kind::symmetric;
}

void derive_turnout(network const &Network, turnout &Turnout)
{
	auto const type{turnout_type_of(Turnout.type)};
	auto const frame{frame_of(Network, Turnout)};
	auto const *main{Network.find(Turnout.main)};
	if (false == type.has_value() || false == frame.valid || main == nullptr)
		return;
	Turnout.movable = type->movable;
	Turnout.chainage = kilometrage_at_start(Network, Turnout, *main, frame);
	Turnout.main_radius = type->bendable() ? bending_of(frame.curvature, Turnout.hand) : 0.0;
	Turnout.cant = frame.cant * Turnout.hand;
	auto const vmax{main->category.vmax};
	if (type->kind == turnout_kind::crossing)
	{
		// two plain tracks crossing
		Turnout.diverging_radius = 0.0;
		Turnout.speed_main = vmax;
		Turnout.speed_diverging = vmax;
		Turnout.velocity = vmax;
		return;
	}
	// the diverging track by the rule the editor writes; of a double switch the sharper of its two, of a slip its slip curves
	auto const branch{switch_curve::branch_speed(type->radius, type->ratio, Turnout.main_radius, Turnout.cant, type->movable)};
	Turnout.speed_diverging = branch.speed;
	Turnout.diverging_radius = branch.radius;
	// the main track at the speed of its line, bent no faster than the deficiency of the frog lets it; a symmetric switch has no straight
	// track, both of its tracks at the speed of its curves
	Turnout.speed_main = type->kind == turnout_kind::symmetric ? branch.speed : vmax;
	if (frame.curvature != 0.0 && type->bendable())
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
	if (false == check_layout(Network, Turnout, *type, Result.reasons))
		return;
	auto const symmetric{type->kind == turnout_kind::symmetric};
	auto const *main{Network.find(Turnout.main)};
	// the track which leaves the main one: the diverging track, the main one itself of a crossing, whose tracks only cross
	auto const *diverging{Network.find(Turnout.diverging != line_id::none ? Turnout.diverging : Turnout.main)};
	auto const *main_held{end_at(Network, *main, Turnout.start)};

	// the tracks from the start, and the line around it: the line it starts from, its main track, the line going on from its end
	auto const frame{frame_of(Network, Turnout)};
	auto const forward{symmetric || frame.along > 0};
	std::vector<switch_curve::track_point> tracks[2];
	sample(main->solved, forward ? 0.0 : main->solved.length, forward ? main->solved.length : 0.0, tracks[0]);
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
	// along its main track, and the one of the second switch of a double switch
	for (auto const id : {Turnout.main, Turnout.main2})
		if (auto const *part{Network.find(id)})
			sample(part->solved, forward ? 0.0 : part->solved.length, forward ? part->solved.length : 0.0, around);
	auto const to{around.back().station};
	if (auto const *at{Network.find(Turnout.main_end)}; at != nullptr && false == symmetric)
		for (auto const &end : at->ends)
		{
			if (auto const *owner{owner_of(Network, end.line)}; owner != nullptr && owner->id == Turnout.id)
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
	auto report{switch_curve::check(tracks[0], tracks[1], around, from, to, type->radius, 0.0)};
	// the curvature of the diverging tracks of one sign all along them, from their elements rather than from the headings sampled
	// (9.7 pkt 12, 11.4 pkt 6)
	std::erase_if(report.refusals, [](std::string const &Refusal) { return Refusal.find("(KZW II)") != std::string::npos; });
	for (auto const id : {Turnout.diverging, Turnout.diverging2})
		if (auto const sign_change{sign_change_of(Network, id, Turnout.hand)}; false == sign_change.empty())
			report.refusals.push_back(sign_change);
	auto const told = [&](std::string const &Text, norms::tier const Level, std::vector<notice> &Into) {
		notice found;
		found.line = main->id;
		found.chainage = forward ? 0.0 : main->solved.length;
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
