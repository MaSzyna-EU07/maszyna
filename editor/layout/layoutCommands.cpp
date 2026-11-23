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
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <format>

namespace layout
{

namespace
{

double constexpr kParallel{1e-9}; // rad: tangents within this run alike
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
	if (Change.view().find(Line) == nullptr)
	{
		Why.push_back(std::format("there's no {}", line_name(Line)));
		return nullptr;
	}
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
	if (Command.radius && *Command.radius < 1.0)
		why.push_back(std::format("a radius of {:.1f} m", *Command.radius));
	for (auto const *length : {&Command.transition_in, &Command.transition_out})
		if (length->has_value() && **length < 0.0)
			why.emplace_back("a transition curve of a length below 0");
	if (Command.cant && *Command.cant < 0.0)
		why.emplace_back("a cant leaning out of the turn");
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

reasons propose(transaction &Change, outcome &Result, split_line const &Command)
{
	auto const *found{Change.view().find(Command.line)};
	if (found == nullptr)
		return {std::format("there's no {}", line_name(Command.line))};
	auto const original{*found};
	auto const &solved{original.solved};
	if (false == solved.valid())
		return {std::format("{} isn't laid", line_name(original.id))};
	auto const s{Command.chainage};
	if (s < kShortest || s > solved.length - kShortest)
		return {std::format("{:.2f} m isn't within {} ({:.2f} m long)", s, line_name(original.id), solved.length)};
	for (std::size_t i = 1; i + 1 < solved.profile.size(); ++i)
		if (std::abs(s - solved.profile[i].chainage) < solved.profile[i].tangent)
			return {std::format("{:.2f} m is inside the vertical curve at {:.1f} m", s, solved.profile[i].chainage)};

	auto const &e{solved.elements[solved.element_at(s)]};
	auto const u{s - e.chainage};
	auto &ids{Change.ids()};
	node cut;
	cut.id = ids.new_node();
	auto const plan{e.position(u)};
	cut.position = {plan.x, solved.elevation(s), plan.y};
	cut.heading = glm::normalize(e.tangent(u));
	cut.curvature = e.curvature(u);
	cut.cant = solved.cant_at(s);
	cut.grade = solved.grade(s);
	auto const after_id{ids.new_line()};
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
		if (std::abs(geometry::signed_angle(in, cut.heading)) <= kParallel && std::abs(cut.curvature) < 1e-12)
		{
			// right at the start of the curve
			take(0, k, before);
			take(k, count, after);
		}
		else if (std::abs(geometry::signed_angle(cut.heading, out)) <= kParallel && std::abs(cut.curvature) < 1e-12)
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
	Result.lines.push_back(after_id);
	Result.nodes.push_back(cut.id);
	Change.add(std::move(cut));
	Change.add(std::move(tail_line));
	return {};
}

// the free end at the node: the node, the end it holds and the heading a line joined there leaves it with
struct free_end
{
	node const *at{nullptr};
	end_ref held;
	int sense{1}; // of the new line's end
	glm::dvec2 heading{0.0};
};

std::optional<free_end> free_end_at(network const &Network, node_id const Id, bool const Start, reasons &Why)
{
	auto const *at{Network.find(Id)};
	if (at == nullptr)
	{
		Why.push_back(std::format("there's no {}", node_name(Id)));
		return std::nullopt;
	}
	if (at->ends.size() != 1)
	{
		Why.push_back(std::format("{} isn't a free end", node_name(Id)));
		return std::nullopt;
	}
	if (std::abs(at->curvature) > 1e-9 || std::abs(at->cant) > 0.5)
	{
		Why.push_back(std::format("{} lies in a curve; a line joined there would have to go on from it", node_name(Id)));
		return std::nullopt;
	}
	free_end result;
	result.at = at;
	result.held = at->ends.front();
	// the new line lies on the other side of the node: from its start, along the side; to its end, against it
	auto const side{-result.held.side()};
	result.sense = Start ? side : -side;
	result.heading = at->heading * static_cast<double>(result.sense);
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

reasons propose(transaction &Change, outcome &Result, join_ends const &Command)
{
	reasons why;
	auto const &view{Change.view()};
	if (Command.from == Command.to)
		return {"a line can't join a node to itself"};
	auto const from{free_end_at(view, Command.from, true, why)};
	auto const to{free_end_at(view, Command.to, false, why)};
	if (false == why.empty())
		return why;
	auto const &neighbour{*view.find(from->held.line)};
	auto const category{neighbour.category};
	auto const shape{neighbour.shape.transition};
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

	auto &ids{Change.ids()};
	line joined;
	joined.id = ids.new_line();
	joined.start = Command.from;
	joined.end = Command.to;
	joined.shape.vertices = std::move(vertices);
	for (auto &vertex : joined.shape.vertices)
		vertex.id = ids.new_vertex();
	joined.shape.transition = shape;
	joined.shape.ramp = neighbour.shape.ramp;
	joined.category = category;
	// the grade line meeting the grades of the ends: one break where their tangents meet, or two with a grade between them
	auto const plan{solve(joined.shape, {from->at->position}, {to->at->position})};
	if (false == plan.valid())
		return {std::format("no line fits between {} and {}: {}", node_name(Command.from), node_name(Command.to), plan.errors.empty() ? std::string{} : plan.errors.front())};
	auto const breaks{join_grades(plan.length, from->at->position.y, from->at->grade * from->sense, to->at->position.y, to->at->grade * to->sense, profile, category)};
	if (false == breaks.has_value())
		return {std::format("no grade line fits between the grades of {} and {}", node_name(Command.from), node_name(Command.to))};
	joined.shape.grades = *breaks;
	Change.change(Command.from).ends.push_back({joined.id, line_end::start, from->sense});
	Change.change(Command.to).ends.push_back({joined.id, line_end::end, to->sense});
	Result.lines.push_back(joined.id);
	Change.add(std::move(joined));
	return {};
}

reasons propose(transaction &Change, outcome &, delete_line const &Command)
{
	auto const *found{Change.view().find(Command.line)};
	if (found == nullptr)
		return {std::format("there's no {}", line_name(Command.line))};
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
	static std::array<char const *, 6> constexpr names{"create line", "move vertex", "set vertex", "split line", "join ends", "delete line"};
	return names[Command.index()];
}

outcome model::execute(command const &Command)
{
	outcome result;
	transaction change{m_network};
	auto why{std::visit([&](auto const &Each) { return propose(change, result, Each); }, Command)};
	if (why.empty())
	{
		settle(change);
		auto judged{check(m_network, change.touched_lines(), change.touched_nodes())};
		why = std::move(judged.reasons);
		result.notices = std::move(judged.notices);
	}
	if (false == why.empty())
	{
		change.roll_back();
		return {false, std::move(why), {}, {}, {}};
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
