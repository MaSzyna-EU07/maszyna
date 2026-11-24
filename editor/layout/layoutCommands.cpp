/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutCommands.hpp"
#include "editor/layout/layoutTurnout.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>
#include <set>

namespace layout
{

namespace
{

double constexpr kParallel{1e-9}; // rad: tangents within this run alike
double constexpr kTouching{1e-6}; // m: a cut this close to the end of an element is at the end
double constexpr kShortest{0.1}; // m: no part of a line split shorter; the simulation takes a path under 5 cm for one of no length

using reasons = std::vector<std::string>;

std::string line_name(line_id const Id)
{
	return std::format("line {}", number(Id));
}

std::string node_name(node_id const Id)
{
	return std::format("node {}", number(Id));
}

std::string turnout_name(turnout_id const Id)
{
	return std::format("turnout {}", number(Id));
}

// the line, or why there's none to edit: a track of a turnout is edited through its turnout
line const *own_line(network const &Network, line_id const Id, reasons &Why)
{
	auto const *found{Network.find(Id)};
	if (found == nullptr)
		Why.push_back(std::format("there's no {}", line_name(Id)));
	else if (found->turnout != turnout_id::none)
		Why.push_back(std::format("{} is a track of {}: move it, change it or take it out", line_name(Id), turnout_name(found->turnout)));
	else
		return found;
	return nullptr;
}

// a radius, transitions and a cant a vertex may have, or why not
void check_curve(std::optional<double> const &Radius, std::optional<double> const &In, std::optional<double> const &Out, std::optional<double> const &Cant, reasons &Why)
{
	if (Radius && *Radius < 1.0)
		Why.push_back(std::format("a radius of {:.1f} m", *Radius));
	for (auto const *length : {&In, &Out})
		if (length->has_value() && **length < 0.0)
			Why.emplace_back("a transition curve of a length below 0");
	if (Cant && *Cant < 0.0)
		Why.emplace_back("a cant leaning out of the turn");
}

reasons propose(transaction &Change, outcome &Result, create_line const &Command)
{
	auto &ids{Change.ids()};
	auto const id{ids.new_line()};
	node first;
	first.id = ids.new_node();
	first.position = Command.start;
	first.ends.push_back({id, line_end::start, 1});
	node last;
	last.id = ids.new_node();
	last.position = Command.end;
	last.ends.push_back({id, line_end::end, 1});
	line laid;
	laid.id = id;
	laid.start = first.id;
	laid.end = last.id;
	laid.shape.vertices = Command.vertices;
	for (auto &vertex : laid.shape.vertices)
		vertex.id = ids.new_vertex();
	laid.shape.grades = Command.grades;
	laid.shape.transition = Command.transition;
	laid.shape.ramp = Command.ramp;
	laid.category = Command.category;
	laid.chainage_start = Command.chainage_start;
	Result.lines.push_back(id);
	Result.nodes = {first.id, last.id};
	Change.add(std::move(first));
	Change.add(std::move(last));
	Change.add(std::move(laid));
	return {};
}

// the vertex of the line, or why there's none
vertex *vertex_of(transaction &Change, line_id const Line, vertex_id const Vertex, reasons &Why)
{
	if (own_line(Change.view(), Line, Why) == nullptr)
		return nullptr;
	auto &vertices{Change.change(Line).shape.vertices};
	auto const found{std::ranges::find_if(vertices, [&](vertex const &Each) { return Each.id == Vertex; })};
	if (found == vertices.end())
	{
		Why.push_back(std::format("{} has no vertex {}", line_name(Line), number(Vertex)));
		return nullptr;
	}
	return &*found;
}

reasons propose(transaction &Change, outcome &, move_vertex const &Command)
{
	reasons why;
	if (auto *moved{vertex_of(Change, Command.line, Command.vertex, why)})
		moved->point = Command.point;
	return why;
}

reasons propose(transaction &Change, outcome &, set_vertex const &Command)
{
	reasons why;
	auto *set{vertex_of(Change, Command.line, Command.vertex, why)};
	if (set == nullptr)
		return why;
	check_curve(Command.radius, Command.transition_in, Command.transition_out, Command.cant, why);
	if (false == why.empty())
		return why;
	set->radius = Command.radius.value_or(set->radius);
	set->transition_in = Command.transition_in.value_or(set->transition_in);
	set->transition_out = Command.transition_out.value_or(set->transition_out);
	set->cant = Command.cant.value_or(set->cant);
	return why;
}

// the point where the line through A along AlongA crosses the one through B along AlongB
std::optional<glm::dvec2> crossing(glm::dvec2 const &A, glm::dvec2 const &AlongA, glm::dvec2 const &B, glm::dvec2 const &AlongB)
{
	double first{0.0};
	double second{0.0};
	if (false == geometry::lines_cross(A, AlongA, B, AlongB, first, second))
		return std::nullopt;
	return A + AlongA * first;
}

// the new node and the new line a split made
struct split_made
{
	node_id node{node_id::none};
	line_id line{line_id::none};
};

// the line cut at the chainage by a node, the part after it a line of its own: the ids given to them, or new ones where none
reasons split_at(transaction &Change, line_id const Line, double const Chainage, node_id const Node, line_id const Tail, split_made &Made)
{
	auto const *found{Change.view().find(Line)};
	if (found == nullptr)
		return {std::format("there's no {}", line_name(Line))};
	auto const original{*found};
	auto const &solved{original.solved};
	if (false == solved.valid())
		return {std::format("{} isn't laid", line_name(original.id))};
	auto const s{Chainage};
	if (s < kShortest || s > solved.length - kShortest)
		return {std::format("{:.2f} m isn't within {} ({:.2f} m long)", s, line_name(original.id), solved.length)};
	for (std::size_t i = 1; i + 1 < solved.profile.size(); ++i)
		if (std::abs(s - solved.profile[i].chainage) < solved.profile[i].tangent)
			return {std::format("{:.2f} m is inside the vertical curve at {:.1f} m", s, solved.profile[i].chainage)};

	auto const &e{solved.elements[solved.element_at(s)]};
	auto const u{s - e.chainage};
	auto &ids{Change.ids()};
	node cut;
	cut.id = Node != node_id::none ? Node : ids.new_node();
	auto const plan{e.position(u)};
	cut.position = {plan.x, solved.elevation(s), plan.y};
	cut.heading = glm::normalize(e.tangent(u));
	cut.curvature = e.curvature(u);
	cut.cant = solved.cant_at(s);
	cut.grade = solved.grade(s);
	auto const after_id{Tail != line_id::none ? Tail : ids.new_line()};
	cut.ends = {{original.id, line_end::end, 1}, {after_id, line_end::start, 1}};

	auto const &vertices{original.shape.vertices};
	auto const count{vertices.size()};
	auto const index_of = [&](unsigned const Vertex) {
		auto const at{std::ranges::find_if(vertices, [&](vertex const &Each) { return number(Each.id) == Vertex; })};
		return static_cast<std::size_t>(at - vertices.begin());
	};
	std::vector<vertex> before;
	std::vector<vertex> after;
	bool held{false}; // the parts go on from the node at its curve
	auto const take = [&](std::size_t const From, std::size_t const To, std::vector<vertex> &Into) {
		for (std::size_t i = From; i < To; ++i)
			Into.push_back(vertices[i]);
	};
	if (e.role == element_role::straight)
	{
		// the straight leading to the vertex, or to the end
		auto const next{e.vertex != 0 ? index_of(e.vertex) : count};
		take(0, next, before);
		take(next, count, after);
	}
	else
	{
		auto const k{index_of(e.vertex)};
		auto const &curve{vertices[k]};
		auto const previous{k == 0 ? geometry::plan_of(Change.view().find(original.start)->position) : vertices[k - 1].point};
		auto const next{k + 1 == count ? geometry::plan_of(Change.view().find(original.end)->position) : vertices[k + 1].point};
		auto const in{glm::normalize(curve.point - previous)};
		auto const out{glm::normalize(next - curve.point)};
		// at the start or the end of the curve: where its transition leaves the straight, or at the tangent point of an arc with none,
		// whose curvature there is already the arc's
		auto const straight_there{std::abs(cut.curvature) < 1e-12};
		auto const arc_starts{e.role == element_role::arc && u < kTouching};
		auto const arc_ends{e.role == element_role::arc && u > e.length - kTouching};
		if (std::abs(geometry::signed_angle(in, cut.heading)) <= kParallel && (straight_there || arc_starts))
		{
			// right at the start of the curve
			take(0, k, before);
			take(k, count, after);
		}
		else if (std::abs(geometry::signed_angle(cut.heading, out)) <= kParallel && (straight_there || arc_ends))
		{
			take(0, k + 1, before);
			take(k + 1, count, after);
		}
		else
		{
			if (e.role != element_role::arc && (original.shape.transition != transition_shape::clothoid || original.shape.ramp != ramp_shape::linear))
				return {std::format("{:.2f} m is inside a transition curve whose parts wouldn't be curves of its kind; split on a straight or an arc", s)};
			auto const first{crossing(curve.point, in, plan, cut.heading)};
			auto const second{crossing(plan, cut.heading, curve.point, out)};
			if (false == first.has_value() || false == second.has_value())
				return {std::format("{:.2f} m: the tangent there doesn't cross the tangents of the curve", s)};
			auto const radius{std::abs(cut.curvature) > 0.0 ? 1.0 / std::abs(cut.curvature) : 1e9};
			vertex head{curve};
			vertex tail{curve};
			head.point = *first;
			tail.point = *second;
			tail.id = ids.new_vertex();
			switch (e.role)
			{
			case element_role::arc:
				head.transition_out = 0.0;
				tail.transition_in = 0.0;
				break;
			case element_role::transition_in:
				head.radius = radius;
				head.transition_in = u;
				head.transition_out = 0.0;
				head.cant = std::abs(cut.cant);
				tail.transition_in = curve.transition_in - u;
				break;
			case element_role::transition_out:
				head.transition_out = u;
				tail.radius = radius;
				tail.transition_in = 0.0;
				tail.transition_out = curve.transition_out - u;
				tail.cant = std::abs(cut.cant);
				break;
			default: break;
			}
			take(0, k, before);
			before.push_back(head);
			after.push_back(tail);
			take(k + 1, count, after);
			held = true;
		}
	}

	line tail_line{original};
	tail_line.id = after_id;
	tail_line.start = cut.id;
	tail_line.shape.vertices = std::move(after);
	tail_line.shape.grades.clear();
	tail_line.start_continues = held;
	tail_line.chainage_start = original.chainage_start + s;
	auto &head_line{Change.change(original.id)};
	head_line.end = cut.id;
	head_line.shape.vertices = std::move(before);
	head_line.shape.grades.clear();
	head_line.end_continues = held;
	for (auto const &point : original.shape.grades)
	{
		if (point.chainage < s)
			head_line.shape.grades.push_back(point);
		else if (point.chainage > s)
			tail_line.shape.grades.push_back({point.chainage - s, point.elevation, point.vertical_radius});
	}
	// the node at the end of the line holds the end of the new one now
	auto &last{Change.change(original.end)};
	for (auto &end : last.ends)
		if (end.line == original.id && end.end == line_end::end)
			end.line = after_id;
	Made = {cut.id, after_id};
	Change.add(std::move(cut));
	Change.add(std::move(tail_line));
	return {};
}

reasons propose(transaction &Change, outcome &Result, split_line const &Command)
{
	reasons why;
	if (own_line(Change.view(), Command.line, why) == nullptr)
		return why;
	split_made made;
	why = split_at(Change, Command.line, Command.chainage, node_id::none, line_id::none, made);
	if (why.empty())
	{
		Result.lines.push_back(made.line);
		Result.nodes.push_back(made.node);
	}
	return why;
}

// lays the line again between its nodes, in the change
solved_line const &lay(transaction &Change, line_id const Id)
{
	auto &laid{Change.change(Id)};
	laid.solved = solve(laid.shape, Change.view().condition(laid, line_end::start), Change.view().condition(laid, line_end::end));
	return laid.solved;
}

// the two lines meeting at the node laid as one, the first one's: where the second goes on from the end of the first, nothing else meets
// there, they're alike (the class, the profile, the transitions, the ramps, the grade going on) and the line laid so is the same track as
// the two were, within a micrometre. False, nothing changed, where they can't be
bool merge_at(transaction &Change, node_id const Node)
{
	auto const &view{Change.view()};
	auto const *at{view.find(Node)};
	if (at == nullptr || at->ends.size() != 2)
		return false;
	auto const ending{at->ends[0].end == line_end::end ? 0 : 1};
	auto const &first_end{at->ends[ending]};
	auto const &second_end{at->ends[1 - ending]};
	if (first_end.end != line_end::end || second_end.end != line_end::start || first_end.sense != second_end.sense)
		return false;
	auto const *first{view.find(first_end.line)};
	auto const *second{view.find(second_end.line)};
	if (first == nullptr || second == nullptr || first->id == second->id || second->end == first->start)
		return false;
	if (first->turnout != turnout_id::none || second->turnout != turnout_id::none || first->category != second->category || first->profile != second->profile ||
	    first->shape.transition != second->shape.transition || first->shape.ramp != second->shape.ramp)
		return false;
	auto const &a{first->solved};
	auto const &b{second->solved};
	if (false == a.valid() || false == b.valid() || first->end_continues != second->start_continues || std::abs(a.grade(a.length) - b.grade(0.0)) > 1e-9)
		return false;
	line merged{*first};
	merged.end = second->end;
	merged.end_continues = second->end_continues;
	for (auto const &point : second->shape.grades)
		merged.shape.grades.push_back({point.chainage + a.length, point.elevation, point.vertical_radius});
	auto const &before{first->shape.vertices};
	auto const &after{second->shape.vertices};
	std::vector<std::vector<vertex>> candidates;
	if (false == first->end_continues)
	{
		candidates.push_back(before);
		candidates.back().insert(candidates.back().end(), after.begin(), after.end());
	}
	else if (false == before.empty() && false == after.empty())
	{
		// the vertex split in two: where the tangent of the first part's last one and of the second part's first one meet, of the arc, or
		// of a transition cut in two
		auto const &head{before.back()};
		auto const &tail{after.front()};
		auto const previous{before.size() > 1 ? before[before.size() - 2].point : geometry::plan_of(view.find(first->start)->position)};
		auto const next{after.size() > 1 ? after[1].point : geometry::plan_of(view.find(second->end)->position)};
		double along{0.0};
		double back{0.0};
		if (false == geometry::lines_cross(previous, head.point - previous, tail.point, next - tail.point, along, back))
			return false;
		auto const corner{previous + (head.point - previous) * along};
		auto const joined = [&](double const Radius, double const In, double const Out, double const Cant) {
			std::vector<vertex> all(before.begin(), std::prev(before.end()));
			vertex one{head};
			one.point = corner;
			one.radius = Radius;
			one.transition_in = In;
			one.transition_out = Out;
			one.cant = Cant;
			all.push_back(one);
			all.insert(all.end(), std::next(after.begin()), after.end());
			return all;
		};
		if (head.transition_out == 0.0 && tail.transition_in == 0.0 && head.radius == tail.radius && head.cant == tail.cant)
			candidates.push_back(joined(head.radius, head.transition_in, tail.transition_out, head.cant));
		if (head.transition_out == 0.0)
			candidates.push_back(joined(tail.radius, head.transition_in + tail.transition_in, tail.transition_out, tail.cant));
		if (tail.transition_in == 0.0)
			candidates.push_back(joined(head.radius, head.transition_in, head.transition_out + tail.transition_out, head.cant));
	}
	auto const total{a.length + b.length};
	for (auto const &vertices : candidates)
	{
		merged.shape.vertices = vertices;
		auto laid{solve(merged.shape, view.condition(*first, line_end::start), view.condition(*second, line_end::end))};
		if (false == laid.valid() || std::abs(laid.length - total) > 1e-6)
			continue;
		auto const steps{std::max(16, static_cast<int>(std::ceil(total)))};
		bool same{true};
		for (int i = 0; i <= steps && same; ++i)
		{
			auto const s{total * i / steps};
			auto const &part{s < a.length ? a : b};
			auto const local{s < a.length ? s : s - a.length};
			same = glm::distance(laid.point(s), part.point(local)) < 1e-6 && std::abs(laid.cant_at(s) - part.cant_at(local)) < 1e-6;
		}
		if (false == same)
			continue;
		merged.solved = std::move(laid);
		auto const gone{second->id};
		auto const beyond{second->end};
		Change.change(merged.id) = std::move(merged);
		for (auto &end : Change.change(beyond).ends)
			if (end.line == gone && end.end == line_end::end)
				end.line = first_end.line;
		Change.erase(gone);
		Change.erase(Node);
		return true;
	}
	return false;
}

// the ids a turnout laid again keeps: of itself, its legs, its tracks and the line beyond its second leg; none where new ones are given
struct turnout_ids
{
	turnout_id turnout{turnout_id::none};
	node_id start{node_id::none};
	node_id main_end{node_id::none};
	node_id diverging_end{node_id::none};
	line_id main{line_id::none};
	line_id beyond{line_id::none};
	line_id diverging{line_id::none};
};

// the turnout laid into the line, its start at the chainage: the line cut at its legs, the part between them its main track, the diverging
// track laid off its start to a node at its end
reasons lay_turnout(transaction &Change, outcome &Result, line_id const Line, double const Chainage, std::string const &Type, int const Hand, int const Direction, turnout_ids const &Ids)
{
	reasons why;
	if (Hand != 1 && Hand != -1)
		return {"a turnout leaves to the left (1) or to the right (-1)"};
	if (Direction != 1 && Direction != -1)
		return {"a turnout runs along its line (1) or against it (-1)"};
	std::string problem;
	auto const type{turnout_type_of(Type, &problem)};
	if (false == type.has_value())
		return {problem};
	auto const *found{own_line(Change.view(), Line, why)};
	if (found == nullptr)
		return why;
	if (false == found->solved.valid())
		return {std::format("{} isn't laid", line_name(Line))};
	auto const length{type->length};
	auto const total{found->solved.length};
	auto const category{found->category};
	// the leg nearer the start of the line
	auto const from{Direction > 0 ? Chainage : Chainage - length};
	if (from < kShortest || from + length > total - kShortest)
		return {std::format("{} is {:.2f} m long: it doesn't fit into {} ({:.2f} m long) with its start at {:.2f} m", Type, length, line_name(Line), total, Chainage)};
	split_made first;
	why = split_at(Change, Line, from, Direction > 0 ? Ids.start : Ids.main_end, Ids.main, first);
	if (false == why.empty())
		return why;
	if (false == lay(Change, first.line).valid())
		return {std::format("{}: the line can't be cut at the legs of {}", line_name(Line), Type)};
	split_made second;
	why = split_at(Change, first.line, length, Direction > 0 ? Ids.main_end : Ids.start, Ids.beyond, second);
	if (false == why.empty())
		return why;
	auto &ids{Change.ids()};
	turnout made;
	made.id = Ids.turnout != turnout_id::none ? Ids.turnout : ids.new_turnout();
	made.type = Type;
	made.hand = Hand;
	made.direction = Direction;
	made.main = first.line;
	made.start = Direction > 0 ? first.node : second.node;
	made.main_end = Direction > 0 ? second.node : first.node;
	made.diverging = Ids.diverging != line_id::none ? Ids.diverging : ids.new_line();
	made.diverging_end = Ids.diverging_end != node_id::none ? Ids.diverging_end : ids.new_node();
	Change.change(made.main).turnout = made.id;
	lay(Change, made.main);
	// the diverging track from the start, on the side of it the main track lies on
	auto const &view{Change.view()};
	auto const *held{view.end_of(made.main, Direction > 0 ? line_end::start : line_end::end)};
	if (held == nullptr)
		return {std::format("{}: no main track at its start", Type)};
	auto const side{held->side()};
	Change.change(made.start).ends.push_back({made.diverging, line_end::start, side});
	derive_turnout(view, made);
	auto laid{lay_diverging(frame_of(view, made), *type, Hand)};
	if (false == laid.valid())
		return laid.errors;
	if (auto const *existing{view.find(made.diverging)})
	{
		auto &track{Change.change(made.diverging)};
		track.start = made.start;
		track.end = made.diverging_end;
		track.turnout = made.id;
		track.solved = laid;
		track.chainage_start = made.chainage;
	}
	else
	{
		line track;
		track.id = made.diverging;
		track.turnout = made.id;
		track.start = made.start;
		track.end = made.diverging_end;
		track.category = category;
		track.profile = view.find(made.main)->profile;
		track.shape.transition = view.find(made.main)->shape.transition;
		track.shape.ramp = view.find(made.main)->shape.ramp;
		track.chainage_start = made.chainage;
		track.solved = laid;
		Change.add(std::move(track));
	}
	if (view.find(made.diverging_end) == nullptr)
	{
		node end;
		end.id = made.diverging_end;
		end.ends.push_back({made.diverging, line_end::end, 1});
		Change.add(std::move(end));
	}
	auto &end{Change.change(made.diverging_end)};
	end.position = laid.end.position;
	end.heading = laid.end.heading;
	end.curvature = laid.end.curvature;
	end.cant = laid.end.cant;
	end.grade = laid.end.grade;
	Result.turnouts.push_back(made.id);
	Result.lines.insert(Result.lines.end(), {made.main, second.line, made.diverging});
	Result.nodes.insert(Result.nodes.end(), {made.start, made.main_end, made.diverging_end});
	Change.add(std::move(made));
	return {};
}

// the turnout lifted off its line: its diverging track off its start, its main track and the lines on both sides of it one line again,
// the first of them; none, and why, where they can't be
std::optional<line_id> lift_turnout(transaction &Change, turnout const &Turnout, reasons &Why)
{
	auto const &view{Change.view()};
	// the line before it along the line it lies on: the one it starts from, or the one going on from the end of its main track
	auto const *before{view.find(Turnout.direction > 0 ? Turnout.start : Turnout.main_end)};
	line_id first{line_id::none};
	if (before != nullptr)
		for (auto const &end : before->ends)
			if (end.end == line_end::end && end.line != Turnout.main && end.line != Turnout.diverging)
				first = end.line;
	auto &start{Change.change(Turnout.start)};
	std::erase_if(start.ends, [&](end_ref const &End) { return End.line == Turnout.diverging && End.end == line_end::start; });
	Change.change(Turnout.main).turnout = turnout_id::none;
	if (first == line_id::none || false == merge_at(Change, Turnout.start) || false == merge_at(Change, Turnout.main_end))
	{
		Why.push_back(std::format("{}: the lines on both sides of it and its main track can't be laid as one line again", turnout_name(Turnout.id)));
		return std::nullopt;
	}
	return first;
}

// the lines going on from the node at the end of a diverging track carried with it, from where it was to where it is: the vertex next to it
// turned and moved as the end did
void carry(transaction &Change, node_id const End, line_id const Track, glm::dvec2 const &From, glm::dvec2 const &FromHeading)
{
	auto const *at{Change.view().find(End)};
	if (at == nullptr)
		return;
	auto const to{geometry::plan_of(at->position)};
	auto const turn{geometry::signed_angle(FromHeading, at->heading)};
	for (auto const &end : at->ends)
	{
		if (end.line == Track)
			continue;
		auto &vertices{Change.change(end.line).shape.vertices};
		if (vertices.empty())
			continue;
		auto &next{end.end == line_end::start ? vertices.front() : vertices.back()};
		next.point = to + geometry::turned(next.point - From, turn);
	}
}

reasons propose(transaction &Change, outcome &Result, insert_turnout const &Command)
{
	return lay_turnout(Change, Result, Command.line, Command.chainage, Command.type, Command.hand, Command.direction, {});
}

// the turnout laid again along its line: moved by the distance, of the type and the hand
reasons relay_turnout(transaction &Change, outcome &Result, turnout_id const Id, double const Distance, std::string const &Type, int const Hand)
{
	auto const *found{Change.view().find(Id)};
	if (found == nullptr)
		return {std::format("there's no {}", turnout_name(Id))};
	auto const old{*found};
	auto const *end{Change.view().find(old.diverging_end)};
	auto const *main{Change.view().find(old.main)};
	if (end == nullptr || main == nullptr)
		return {std::format("{} lacks a leg or a track", turnout_name(Id))};
	auto const was{geometry::plan_of(end->position)};
	auto const heading{end->heading};
	// the start along the line the lifting makes: past the line before it, and its main track where it runs against the line
	auto const *before{Change.view().find(old.direction > 0 ? old.start : old.main_end)};
	double offset{0.0};
	if (before != nullptr)
		for (auto const &each : before->ends)
			if (each.end == line_end::end && each.line != old.main && each.line != old.diverging)
				if (auto const *line{Change.view().find(each.line)})
					offset = line->solved.length;
	auto const start{offset + (old.direction > 0 ? 0.0 : main->solved.length)};
	// the line beyond its second leg along the line keeps its id
	turnout_ids kept{old.id, old.start, old.main_end, old.diverging_end, old.main, line_id::none, old.diverging};
	if (auto const *after{Change.view().find(old.direction > 0 ? old.main_end : old.start)})
		for (auto const &each : after->ends)
			if (each.end == line_end::start && each.line != old.main && each.line != old.diverging)
				kept.beyond = each.line;
	reasons why;
	auto const line{lift_turnout(Change, old, why)};
	if (false == line.has_value())
		return why;
	why = lay_turnout(Change, Result, *line, start + old.direction * Distance, Type, Hand, old.direction, kept);
	if (false == why.empty())
		return why;
	carry(Change, old.diverging_end, old.diverging, was, heading);
	return {};
}

reasons propose(transaction &Change, outcome &Result, move_turnout const &Command)
{
	auto const *found{Change.view().find(Command.turnout)};
	if (found == nullptr)
		return {std::format("there's no {}", turnout_name(Command.turnout))};
	return relay_turnout(Change, Result, Command.turnout, Command.distance, found->type, found->hand);
}

reasons propose(transaction &Change, outcome &Result, change_turnout const &Command)
{
	auto const *found{Change.view().find(Command.turnout)};
	if (found == nullptr)
		return {std::format("there's no {}", turnout_name(Command.turnout))};
	return relay_turnout(Change, Result, Command.turnout, 0.0, Command.type.empty() ? found->type : Command.type, Command.hand.value_or(found->hand));
}

reasons propose(transaction &Change, outcome &, remove_turnout const &Command)
{
	auto const *found{Change.view().find(Command.turnout)};
	if (found == nullptr)
		return {std::format("there's no {}", turnout_name(Command.turnout))};
	auto const gone{*found};
	// the diverging track, and the node at its end where nothing goes on from there
	if (Change.view().find(gone.diverging_end) != nullptr)
	{
		auto &end{Change.change(gone.diverging_end)};
		std::erase_if(end.ends, [&](end_ref const &End) { return End.line == gone.diverging; });
		if (end.ends.empty())
			Change.erase(gone.diverging_end);
	}
	if (Change.view().find(gone.start) != nullptr)
		std::erase_if(Change.change(gone.start).ends, [&](end_ref const &End) { return End.line == gone.diverging; });
	Change.erase(gone.diverging);
	if (Change.view().find(gone.main) != nullptr)
		Change.change(gone.main).turnout = turnout_id::none;
	Change.erase(gone.id);
	// the line again where it can be
	merge_at(Change, gone.start);
	merge_at(Change, gone.main_end);
	return {};
}

reasons propose(transaction &Change, outcome &Result, insert_vertex const &Command)
{
	reasons why;
	if (own_line(Change.view(), Command.line, why) == nullptr)
		return why;
	auto const &shape{Command.shape};
	check_curve(shape.radius, shape.transition_in, shape.transition_out, shape.cant, why);
	if (false == why.empty())
		return why;
	auto &vertices{Change.change(Command.line).shape.vertices};
	auto at{vertices.begin()};
	if (Command.after != vertex_id::none)
	{
		at = std::ranges::find_if(vertices, [&](vertex const &Each) { return Each.id == Command.after; });
		if (at == vertices.end())
			return {std::format("{} has no vertex {}", line_name(Command.line), number(Command.after))};
		++at;
	}
	vertex made{shape};
	made.id = Change.ids().new_vertex();
	Result.vertices.push_back(made.id);
	vertices.insert(at, made);
	return {};
}

reasons propose(transaction &Change, outcome &, remove_vertex const &Command)
{
	reasons why;
	if (vertex_of(Change, Command.line, Command.vertex, why) == nullptr)
		return why;
	std::erase_if(Change.change(Command.line).shape.vertices, [&](vertex const &Each) { return Each.id == Command.vertex; });
	return {};
}

reasons propose(transaction &Change, outcome &, set_grades const &Command)
{
	reasons why;
	if (own_line(Change.view(), Command.line, why) == nullptr)
		return why;
	for (std::size_t i = 0; i < Command.grades.size(); ++i)
	{
		if (Command.grades[i].vertical_radius <= 0.0)
			why.push_back(std::format("the grade break at {:.1f} m has no vertical radius", Command.grades[i].chainage));
		if (i > 0 && Command.grades[i].chainage <= Command.grades[i - 1].chainage)
			why.push_back(std::format("the grade break at {:.1f} m isn't after the one before it", Command.grades[i].chainage));
	}
	if (why.empty())
		Change.change(Command.line).shape.grades = Command.grades;
	return why;
}

reasons propose(transaction &Change, outcome &, set_line_class const &Command)
{
	if (Change.view().find(Command.line) == nullptr)
		return {std::format("there's no {}", line_name(Command.line))};
	if (Command.category.vmax <= 0.0)
		return {"a line of no speed"};
	Change.change(Command.line).category = Command.category;
	return {};
}

reasons propose(transaction &Change, outcome &, set_line_profile const &Command)
{
	if (Change.view().find(Command.line) == nullptr)
		return {std::format("there's no {}", line_name(Command.line))};
	Change.change(Command.line).profile = Command.profile;
	return {};
}

// the free end at the node: the node, the end it holds and the heading a line joined there leaves it with, its curvature and its cant along
// the new line
struct free_end
{
	node const *at{nullptr};
	end_ref held;
	int sense{1}; // of the new line's end
	glm::dvec2 heading{0.0};
	double curvature{0.0};
	double cant{0.0};
	bool curved() const { return std::abs(curvature) > 1e-9 || std::abs(cant) > 0.5; }
};

// a node with the ends on one side of it only: one line, or the two tracks of a turnout starting there
std::optional<free_end> free_end_at(network const &Network, node_id const Id, bool const Start, reasons &Why)
{
	auto const *at{Network.find(Id)};
	if (at == nullptr)
	{
		Why.push_back(std::format("there's no {}", node_name(Id)));
		return std::nullopt;
	}
	auto const turnout_start{at->ends.size() == 2 && (diverging_start(Network, at->ends[0]) || diverging_start(Network, at->ends[1]))};
	if (at->ends.size() != 1 && false == turnout_start)
	{
		Why.push_back(std::format("{} isn't a free end", node_name(Id)));
		return std::nullopt;
	}
	free_end result;
	result.at = at;
	result.held = turnout_start && diverging_start(Network, at->ends[0]) ? at->ends[1] : at->ends[0];
	// the new line lies on the other side of the node: from its start, along the side; to its end, against it
	auto const side{-result.held.side()};
	result.sense = Start ? side : -side;
	result.heading = at->heading * static_cast<double>(result.sense);
	result.curvature = at->curvature * result.sense;
	result.cant = at->cant * result.sense;
	// the end of the diverging track of a turnout, where its curve runs through the frog: a line joined there may start from a straight,
	// the curvature stepping at the end of the turnout as the norms of the track connections allow (tab. 9.15)
	if (auto const *switched{owner_of(Network, result.held.line)}; switched != nullptr && switched->diverging == result.held.line && result.held.end == line_end::end)
		result.curvature = 0.0;
	return result;
}

// the curve of the radius at the vertex as the join lays it: the cant and the transitions given, or the ones the norms prefer
vertex join_vertex(glm::dvec2 const &Point, double const Radius, join_ends const &Command, norms::profile_id const Profile, norms::line_class const &Class)
{
	auto const preferred{preferred_curve(Profile, Class, Radius)};
	vertex result;
	result.point = Point;
	result.radius = Radius;
	result.cant = Command.cant >= 0.0 ? Command.cant : preferred.cant;
	result.transition_in = result.transition_out = Command.transition >= 0.0 ? Command.transition : preferred.transition;
	return result;
}

// the breaks of the grade line of a line of the length between heights and grades at its ends: none where the even grade between the
// heights differs from both by less than half of what needs no rounding, else one break where the tangents of the ends meet or two
// with a grade between them, their vertical curves of the radius the norms prefer, or the least one which fits
std::optional<std::vector<grade_break>> join_grades(double const Length, double const Start, double const StartGrade, double const End, double const EndGrade, norms::profile_id const Profile,
                                                    norms::line_class const &Class)
{
	double constexpr unrounded{0.001};
	auto const even{(End - Start) / Length};
	if (std::abs(StartGrade - even) <= unrounded && std::abs(EndGrade - even) <= unrounded)
		return std::vector<grade_break>{};
	norms::query where;
	where.speed = Class.vmax;
	where.kind = Class.kind;
	where.track = Class.track;
	auto const limits{norms::limits(Profile, norms::parameter::vertical_radius, where)};
	std::vector<double> radii;
	for (auto const *bound : {&limits.prefer, &limits.warn, &limits.block})
		if (bound->has_value())
			radii.push_back((*bound)->value);
	if (radii.empty())
		radii.push_back(5000.0);
	for (auto const radius : radii)
	{
		if (std::abs(StartGrade - EndGrade) > 1e-9)
		{
			auto const at{(End - Start - EndGrade * Length) / (StartGrade - EndGrade)};
			auto const tangent{radius * std::abs(EndGrade - StartGrade) * 0.5};
			if (at - tangent > 1.0 && Length - at - tangent > 1.0)
				return std::vector<grade_break>{{at, Start + StartGrade * at, radius}};
		}
		auto const reach{Length * 0.25};
		auto const first{Start + StartGrade * reach};
		auto const second{End - EndGrade * reach};
		auto const middle{(second - first) / (Length - 2.0 * reach)};
		auto const one{radius * std::abs(middle - StartGrade) * 0.5};
		auto const two{radius * std::abs(EndGrade - middle) * 0.5};
		if (one < reach - 1.0 && two < reach - 1.0 && one + two < Length - 2.0 * reach - 1.0)
			return std::vector<grade_break>{{reach, first, radius}, {Length - reach, second, radius}};
	}
	return std::nullopt;
}

// the tangents the curve of the vertex takes, between the directions
curve_fit join_fit(vertex const &Vertex, glm::dvec2 const &In, glm::dvec2 const &Out, transition_shape const Shape)
{
	return fit_curve(Shape, geometry::signed_angle(In, Out), Vertex.radius, Vertex.transition_in, Vertex.transition_out);
}

// a new line from the free ends, its vertices, how its ends go on from their nodes, the grade line meeting the grades of the ends: one
// break where their tangents meet, or two with a grade between them
reasons add_joined(transaction &Change, outcome &Result, join_ends const &Command, free_end const &From, free_end const &To, std::vector<vertex> Vertices, bool const StartContinues,
                   bool const EndContinues)
{
	auto const &view{Change.view()};
	auto const &neighbour{*view.find(From.held.line)};
	auto const profile{view.areas.at({From.at->position.x, From.at->position.z}, {})};
	auto &ids{Change.ids()};
	line joined;
	joined.id = ids.new_line();
	joined.start = Command.from;
	joined.end = Command.to;
	joined.shape.vertices = std::move(Vertices);
	for (auto &vertex : joined.shape.vertices)
		vertex.id = ids.new_vertex();
	joined.shape.transition = neighbour.shape.transition;
	joined.shape.ramp = neighbour.shape.ramp;
	joined.category = neighbour.category;
	joined.profile = neighbour.profile;
	joined.start_continues = StartContinues;
	joined.end_continues = EndContinues;
	auto const plan{solve(joined.shape, {From.at->position, StartContinues, From.curvature, From.cant}, {To.at->position, EndContinues, To.curvature, To.cant})};
	if (false == plan.valid())
		return {std::format("no line fits between {} and {}: {}", node_name(Command.from), node_name(Command.to), plan.errors.empty() ? std::string{} : plan.errors.front())};
	auto const breaks{join_grades(plan.length, From.at->position.y, From.at->grade * From.sense, To.at->position.y, To.at->grade * To.sense, profile, neighbour.category)};
	if (false == breaks.has_value())
		return {std::format("no grade line fits between the grades of {} and {}", node_name(Command.from), node_name(Command.to))};
	joined.shape.grades = *breaks;
	Change.change(Command.from).ends.push_back({joined.id, line_end::start, From.sense});
	Change.change(Command.to).ends.push_back({joined.id, line_end::end, To.sense});
	Result.lines.push_back(joined.id);
	Change.add(std::move(joined));
	return {};
}

// a line going on from the curve of one of the ends (a line cut inside one), to a straight end: one curve where the tangents meet, which
// begins right at the curved end, its arc going on or a transition from its curvature to one the norms prefer; the transition at the
// straight end as long as makes the curve begin there
reasons join_curved(transaction &Change, outcome &Result, join_ends const &Command, free_end const &From, free_end const &To)
{
	if (From.curved() && To.curved())
		return {std::format("{} and {} both lie in curves: a line going on from both isn't solved yet", node_name(Command.from), node_name(Command.to))};
	auto const &view{Change.view()};
	auto const &neighbour{*view.find(From.held.line)};
	auto const category{neighbour.category};
	auto const shape{neighbour.shape.transition};
	auto const profile{view.areas.at({From.at->position.x, From.at->position.z}, {})};
	auto const p0{geometry::plan_of(From.at->position)};
	auto const p1{geometry::plan_of(To.at->position)};
	double along{0.0};
	double back{0.0};
	if (false == geometry::lines_cross(p0, From.heading, p1, To.heading, along, back) || along <= 0.0 || back >= 0.0)
		return {std::format("the tangents of {} and {} don't meet ahead of them: no one curve going on from the curved end joins them", node_name(Command.from), node_name(Command.to))};
	auto const corner{p0 + From.heading * along};
	auto const deflection{geometry::signed_angle(From.heading, To.heading)};
	auto const &curved{From.curved() ? From : To};
	auto const held{From.curved() ? along : -back}; // the tangent which has to be the curve's own to the bit
	auto const other{From.curved() ? -back : along};
	if (curved.curvature * deflection <= 0.0 || curved.cant * deflection < -0.5)
		return {std::format("the curve at {} turns the other way from the one the line would take", node_name(From.curved() ? Command.from : Command.to))};
	// the radii tried: the curve going on, then the ones the norms prefer, the largest first
	auto const going_on{1.0 / std::abs(curved.curvature)};
	std::vector<double> radii{going_on};
	for (auto const radius : {6000.0, 4000.0, 3000.0, 2000.0, 1500.0, 1200.0, 1000.0, 800.0, 600.0, 500.0, 400.0, 300.0})
		if (radius != going_on)
			radii.push_back(radius);
	for (auto const radius : radii)
	{
		auto const preferred{preferred_curve(profile, category, radius)};
		auto const going{radius == going_on};
		auto const cant{going ? std::abs(curved.cant) : (Command.cant >= 0.0 ? Command.cant : preferred.cant)};
		auto const leading{going ? 0.0 : std::max(1.0, Command.transition >= 0.0 ? Command.transition : preferred.transition)};
		// a cubic parabola runs between a straight and an arc only
		if (false == going && shape == transition_shape::cubic_parabola)
			continue;
		// the tangent at the curved end as the free transition at the other end is long
		auto const tangent = [&](double const Free, bool &Fits) {
			auto const fit{From.curved() ? fit_curve(shape, deflection, radius, leading, Free, From.curvature, 0.0) : fit_curve(shape, deflection, radius, Free, leading, 0.0, To.curvature)};
			Fits = fit.error.empty() && false == fit.elements.empty();
			return From.curved() ? fit.tangent_in : fit.tangent_out;
		};
		// the free transition from none up to 400 m, the first span whose ends lie on both sides of the tangent wanted
		double low{0.0};
		double high{-1.0};
		bool fits{false};
		auto last{tangent(0.0, fits) - held};
		auto last_fits{fits};
		for (int step{1}; step <= 200; ++step)
		{
			// every 2 m, as exact as the sum of the steps was
			auto const free{2.0 * step};
			auto const now{tangent(free, fits) - held};
			if (fits && last_fits && (now == 0.0 || (now < 0.0) != (last < 0.0)))
			{
				low = free - 2.0;
				high = free;
				break;
			}
			last = now;
			last_fits = fits;
		}
		if (high < 0.0)
			continue;
		bool low_side{(tangent(low, fits) - held) < 0.0};
		for (int i = 0; i < 80; ++i)
		{
			auto const middle{0.5 * (low + high)};
			(((tangent(middle, fits) - held) < 0.0) == low_side ? low : high) = middle;
		}
		auto const free{0.5 * (low + high)};
		vertex made;
		made.point = corner;
		made.radius = radius;
		made.cant = cant;
		made.transition_in = From.curved() ? leading : free;
		made.transition_out = From.curved() ? free : leading;
		auto const fit{fit_curve(shape, deflection, radius, made.transition_in, made.transition_out, From.curved() ? From.curvature : 0.0, To.curved() ? To.curvature : 0.0)};
		if (false == fit.error.empty() || (From.curved() ? fit.tangent_out : fit.tangent_in) > other)
			continue;
		auto const why{add_joined(Change, Result, Command, From, To, {made}, From.curved(), To.curved())};
		if (why.empty())
			return {};
	}
	return {std::format("no curve going on from {} fits between {} and {}", node_name(From.curved() ? Command.from : Command.to), node_name(Command.from), node_name(Command.to))};
}

reasons propose(transaction &Change, outcome &Result, join_ends const &Command)
{
	reasons why;
	auto const &view{Change.view()};
	if (Command.from == Command.to)
		return {"a line can't join a node to itself"};
	auto const from{free_end_at(view, Command.from, true, why)};
	auto const to{free_end_at(view, Command.to, false, why)};
	// an end not found says why in Why
	if (false == from.has_value() || false == to.has_value())
		return why;
	auto const &neighbour{*view.find(from->held.line)};
	auto const category{neighbour.category};
	auto const shape{neighbour.shape.transition};
	if (from->curved() || to->curved())
		return join_curved(Change, Result, Command, *from, *to);
	auto const profile{view.areas.at({from->at->position.x, from->at->position.z}, {})};
	auto const p0{geometry::plan_of(from->at->position)};
	auto const p1{geometry::plan_of(to->at->position)};
	auto const t0{from->heading};
	auto const t1{to->heading};
	if (glm::distance(p0, p1) < 1.0)
		return {"the ends are too close to lay a line between them"};

	// the radii tried, the largest first: the one given, or from the largest down to the least the norms allow
	std::vector<double> radii;
	if (Command.radius > 0.0)
		radii.push_back(Command.radius);
	else
	{
		auto const least{std::max(preferred_curve(profile, category, 1000.0).radius_block, 150.0)};
		for (auto const radius : {10000.0, 8000.0, 6000.0, 5000.0, 4000.0, 3000.0, 2500.0, 2000.0, 1800.0, 1500.0, 1400.0, 1200.0, 1100.0, 1000.0, 900.0, 800.0, 700.0, 600.0, 550.0, 500.0, 450.0,
		                          400.0, 350.0, 300.0, 280.0, 250.0, 220.0, 200.0, 180.0, 150.0})
			if (radius >= least)
				radii.push_back(radius);
	}
	std::vector<vertex> vertices;
	auto const straight{glm::normalize(p1 - p0)};
	double along{0.0};
	double back{0.0};
	if (std::abs(geometry::signed_angle(t0, straight)) <= kParallel && std::abs(geometry::signed_angle(t1, straight)) <= kParallel)
	{
		// facing each other on one straight
	}
	else if (geometry::lines_cross(p0, t0, p1, t1, along, back) && along > 0.0 && back < 0.0)
	{
		// one curve where the tangents meet
		auto const corner{p0 + t0 * along};
		for (auto const radius : radii)
		{
			auto const candidate{join_vertex(corner, radius, Command, profile, category)};
			auto const fit{join_fit(candidate, t0, t1, shape)};
			if (fit.error.empty() && fit.tangent_in <= along && fit.tangent_out <= -back)
			{
				vertices.push_back(candidate);
				break;
			}
		}
		if (vertices.empty())
			return {std::format("no curve fits between {} and {}: the tangents meet {:.1f} m and {:.1f} m from them", node_name(Command.from), node_name(Command.to), along, -back)};
	}
	else
	{
		// two curves: out along the heading of the start, into the heading of the end
		auto const ahead{glm::dot(p1 - p0, t0)};
		if (ahead <= 0.0 || glm::dot(t0, t1) <= 0.0)
			return {std::format("{} doesn't lie ahead of {}; the line would have to turn back", node_name(Command.to), node_name(Command.from))};
		for (auto const radius : radii)
		{
			for (auto const share : {1.0 / 3.0, 0.25, 0.4, 0.2, 0.45})
			{
				auto const reach{ahead * share};
				auto const a{p0 + t0 * reach};
				auto const b{p1 - t1 * reach};
				if (glm::distance(a, b) < 1.0)
					continue;
				auto const middle{glm::normalize(b - a)};
				auto const first{join_vertex(a, radius, Command, profile, category)};
				auto const second{join_vertex(b, radius, Command, profile, category)};
				auto const fit_a{join_fit(first, t0, middle, shape)};
				auto const fit_b{join_fit(second, middle, t1, shape)};
				if (fit_a.error.empty() && fit_b.error.empty() && fit_a.tangent_in <= reach && fit_b.tangent_out <= reach && fit_a.tangent_out + fit_b.tangent_in <= glm::distance(a, b))
				{
					vertices = {first, second};
					break;
				}
			}
			if (false == vertices.empty())
				break;
		}
		if (vertices.empty())
			return {std::format("no two curves fit between {} and {}", node_name(Command.from), node_name(Command.to))};
	}

	return add_joined(Change, Result, Command, *from, *to, std::move(vertices), false, false);
}

reasons propose(transaction &Change, outcome &, delete_line const &Command)
{
	reasons why;
	auto const *found{own_line(Change.view(), Command.line, why)};
	if (found == nullptr)
		return why;
	auto const gone{*found};
	for (auto const end : {line_end::start, line_end::end})
	{
		auto const id{gone.node_at(end)};
		if (Change.view().find(id) == nullptr)
			continue;
		auto &at{Change.change(id)};
		std::erase_if(at.ends, [&](end_ref const &Ref) { return Ref.line == gone.id && Ref.end == end; });
		if (at.ends.empty())
			Change.erase(id);
	}
	Change.erase(gone.id);
	return {};
}

} // namespace

char const *command_name(command const &Command)
{
	static std::array<char const *, std::variant_size_v<command>> constexpr names{"create line", "move vertex",   "set vertex",     "split line",     "join ends",
	                                                                               "delete line", "insert vertex", "remove vertex",  "set grades",     "set line class",
	                                                                               "set line profile", "insert turnout", "move turnout", "change turnout", "remove turnout"};
	return names[Command.index()];
}

namespace
{

// the BLOCK finding was there before the edit
bool was_there(std::vector<notice> const &Earlier, notice const &Found)
{
	return std::ranges::any_of(Earlier, [&](notice const &Old) { return same_finding(Old, Found); });
}

} // namespace

outcome model::execute(command const &Command, edit_policy const Policy)
{
	outcome result;
	// the state before the edit, to tell the BLOCK findings it adds from the ones there were
	std::optional<network> earlier;
	if (Policy == edit_policy::no_new_block)
		earlier = m_network;
	transaction change{m_network};
	auto why{std::visit([&](auto const &Each) { return propose(change, result, Each); }, Command)};
	if (why.empty())
	{
		settle(change);
		auto judged{check(m_network, change.touched_lines(), change.touched_nodes(), change.touched_turnouts())};
		if (earlier.has_value() && judged.reasons.empty() && false == judged.blocks.empty())
		{
			std::set<line_id> lines;
			std::set<node_id> nodes;
			std::set<turnout_id> turnouts;
			for (auto const id : change.touched_lines())
				if (earlier->find(id) != nullptr)
					lines.insert(id);
			for (auto const id : change.touched_nodes())
				if (earlier->find(id) != nullptr)
					nodes.insert(id);
			for (auto const id : change.touched_turnouts())
				if (earlier->find(id) != nullptr)
					turnouts.insert(id);
			auto const before{check(*earlier, lines, nodes, turnouts).blocks};
			std::erase_if(judged.blocks, [&](notice const &Found) { return was_there(before, Found); });
		}
		why = judged.refusals();
		result.notices = std::move(judged.notices);
	}
	if (false == why.empty())
	{
		change.roll_back();
		outcome refused;
		refused.reasons = std::move(why);
		return refused;
	}
	result.done = true;
	m_undo.push_back({command_name(Command), change.before(), change.after()});
	m_redo.clear();
	return result;
}

bool model::undo()
{
	if (m_undo.empty())
		return false;
	auto step{std::move(m_undo.back())};
	m_undo.pop_back();
	restore(m_network, step.before);
	m_redo.push_back(std::move(step));
	return true;
}

bool model::redo()
{
	if (m_redo.empty())
		return false;
	auto step{std::move(m_redo.back())};
	m_redo.pop_back();
	restore(m_network, step.after);
	m_undo.push_back(std::move(step));
	return true;
}

} // namespace layout
