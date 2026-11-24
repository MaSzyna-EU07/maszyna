/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/layout/layoutNetwork.hpp"
#include "editor/editorGeometry.hpp"

#include <algorithm>
#include <cmath>
#include <format>

namespace layout
{

namespace
{

double constexpr kHeading{1e-9}; // rad: the ends of a node head the same way within this
double constexpr kCurvature{1e-9}; // 1/m: a smaller difference of the curvature isn't a step
double constexpr kCantStep{0.5}; // mm, as the norms engine takes a step of the cant
double constexpr kGradeBreak{0.002}; // a change of the grade up to 2 per mille needn't be rounded (ST-T1-A6 10.2 pkt 2)

std::string line_name(line_id const Id)
{
	return std::format("line {}", number(Id));
}

std::string node_name(node_id const Id)
{
	return std::format("node {}", number(Id));
}

// the values of the end of the line as the node sees them, along its heading
struct seen_end
{
	glm::dvec2 heading;
	double curvature;
	double cant;
	double grade;
};

seen_end seen(line const &Line, end_ref const &End)
{
	auto const &values{End.end == line_end::start ? Line.solved.start : Line.solved.end};
	auto const sense{static_cast<double>(End.sense)};
	return {values.heading * sense, values.curvature * sense, values.cant * sense, values.grade * sense};
}

// the node takes what the ends it holds tell: the heading of the only one, the curvature and the cant of the first one which doesn't go
// on from it (one going on reads them), the grade of the first; true where the curvature or the cant changed
bool derive(network const &Network, node &Node)
{
	auto const before{std::make_pair(Node.curvature, Node.cant)};
	bool graded{false};
	bool curved{false};
	for (auto const &end : Node.ends)
	{
		auto const *owner{Network.find(end.line)};
		if (owner == nullptr || false == owner->solved.valid())
			continue;
		auto const values{seen(*owner, end)};
		if (Node.ends.size() == 1)
			Node.heading = values.heading;
		if (false == graded)
		{
			Node.grade = values.grade;
			graded = true;
		}
		if (false == curved && false == owner->continues(end.end))
		{
			Node.curvature = values.curvature;
			Node.cant = values.cant;
			curved = true;
		}
	}
	return before != std::make_pair(Node.curvature, Node.cant);
}

norms::element reversed(norms::element Element, double const Length)
{
	auto const start{Element.start};
	Element.start = Length - (start + Element.length);
	auto const curvature{Element.curvature_start};
	Element.curvature_start = -Element.curvature_end;
	Element.curvature_end = -curvature;
	std::swap(Element.from, Element.to);
	std::swap(Element.open_start, Element.open_end);
	return Element;
}

// the elements run on alike: the same kind, the curvature going on as it did
bool runs_on(norms::element const &A, norms::element const &B)
{
	// two halves of Bloss curves make no Bloss curve
	if (A.kind != B.kind || A.bloss || B.bloss || std::abs(A.curvature_end - B.curvature_start) > kCurvature || A.speed != B.speed || A.features != B.features)
		return false;
	if (A.kind != norms::element_kind::transition)
		return std::abs(A.curvature_start - B.curvature_end) <= kCurvature;
	auto const rate_a{(A.curvature_end - A.curvature_start) / std::max(A.length, 1e-9)};
	auto const rate_b{(B.curvature_end - B.curvature_start) / std::max(B.length, 1e-9)};
	return std::abs(rate_a - rate_b) <= 1e-6 * std::max(std::abs(rate_a), std::abs(rate_b)) + 1e-12;
}

char const *parameter_name(norms::parameter const What)
{
	switch (What)
	{
	case norms::parameter::cant_deficiency: return "cant deficiency";
	case norms::parameter::cant_excess: return "cant excess";
	case norms::parameter::deficiency_jump: return "step of the cant deficiency";
	case norms::parameter::cant_gradient: return "cant ramp";
	case norms::parameter::cant_rate: return "rate of the cant change";
	case norms::parameter::deficiency_rate: return "rate of the cant deficiency change";
	case norms::parameter::element_length: return "element length";
	case norms::parameter::transition_length: return "transition length";
	case norms::parameter::transition_shift: return "transition shift";
	case norms::parameter::radius: return "radius";
	case norms::parameter::cant: return "cant";
	case norms::parameter::cant_min: return "least cant";
	case norms::parameter::vertical_radius: return "vertical radius";
	case norms::parameter::grade: return "grade";
	case norms::parameter::intermediate_length: return "element between steps";
	case norms::parameter::profile_boundary: return "arc across a profile boundary";
	default: return "value";
	}
}

} // namespace

node const *network::find(node_id const Id) const
{
	auto const found{m_nodes.find(Id)};
	return found != m_nodes.end() ? &found->second : nullptr;
}

line const *network::find(line_id const Id) const
{
	auto const found{m_lines.find(Id)};
	return found != m_lines.end() ? &found->second : nullptr;
}

end_ref const *network::end_of(line_id const Line, line_end const End) const
{
	auto const *owner{find(Line)};
	if (owner == nullptr)
		return nullptr;
	auto const *at{find(owner->node_at(End))};
	if (at == nullptr)
		return nullptr;
	auto const found{std::ranges::find_if(at->ends, [&](end_ref const &Ref) { return Ref.line == Line && Ref.end == End; })};
	return found != at->ends.end() ? &*found : nullptr;
}

end_condition network::condition(line const &Line, line_end const End) const
{
	end_condition result;
	auto const *at{find(Line.node_at(End))};
	if (at == nullptr)
		return result;
	result.position = at->position;
	result.continues = Line.continues(End);
	if (auto const *ref{end_of(Line.id, End)}; ref != nullptr && result.continues)
	{
		result.curvature = at->curvature * ref->sense;
		result.cant = at->cant * ref->sense;
	}
	return result;
}

line &transaction::change(line_id const Id)
{
	auto &kept{m_before.lines};
	if (false == kept.contains(Id))
	{
		auto const found{m_network.m_lines.find(Id)};
		kept[Id] = found != m_network.m_lines.end() ? std::optional<line>{found->second} : std::nullopt;
	}
	return m_network.m_lines.at(Id);
}

node &transaction::change(node_id const Id)
{
	auto &kept{m_before.nodes};
	if (false == kept.contains(Id))
	{
		auto const found{m_network.m_nodes.find(Id)};
		kept[Id] = found != m_network.m_nodes.end() ? std::optional<node>{found->second} : std::nullopt;
	}
	return m_network.m_nodes.at(Id);
}

line &transaction::add(line Line)
{
	auto const id{Line.id};
	if (false == m_before.lines.contains(id))
		m_before.lines[id] = m_network.find(id) != nullptr ? std::optional<line>{*m_network.find(id)} : std::nullopt;
	return m_network.m_lines[id] = std::move(Line);
}

node &transaction::add(node Node)
{
	auto const id{Node.id};
	if (false == m_before.nodes.contains(id))
		m_before.nodes[id] = m_network.find(id) != nullptr ? std::optional<node>{*m_network.find(id)} : std::nullopt;
	return m_network.m_nodes[id] = std::move(Node);
}

void transaction::erase(line_id const Id)
{
	if (m_network.find(Id) == nullptr)
		return;
	change(Id);
	m_network.m_lines.erase(Id);
}

void transaction::erase(node_id const Id)
{
	if (m_network.find(Id) == nullptr)
		return;
	change(Id);
	m_network.m_nodes.erase(Id);
}

std::set<line_id> transaction::touched_lines() const
{
	std::set<line_id> result;
	for (auto const &[id, before] : m_before.lines)
		result.insert(id);
	return result;
}

std::set<node_id> transaction::touched_nodes() const
{
	std::set<node_id> result;
	for (auto const &[id, before] : m_before.nodes)
		result.insert(id);
	return result;
}

snapshot transaction::after() const
{
	snapshot result;
	for (auto const &[id, before] : m_before.lines)
		result.lines[id] = m_network.find(id) != nullptr ? std::optional<line>{*m_network.find(id)} : std::nullopt;
	for (auto const &[id, before] : m_before.nodes)
		result.nodes[id] = m_network.find(id) != nullptr ? std::optional<node>{*m_network.find(id)} : std::nullopt;
	return result;
}

void transaction::roll_back()
{
	restore(m_network, m_before);
	m_before = {};
}

void restore(network &Network, snapshot const &State)
{
	// through a transaction of its own, which is the friend of the network
	transaction change{Network};
	for (auto const &[id, kept] : State.lines)
	{
		if (kept.has_value())
			change.add(*kept);
		else
			change.erase(id);
	}
	for (auto const &[id, kept] : State.nodes)
	{
		if (kept.has_value())
			change.add(*kept);
		else
			change.erase(id);
	}
}

void settle(transaction &Change)
{
	std::set<line_id> pending;
	for (auto const id : Change.touched_lines())
		if (Change.view().find(id) != nullptr)
			pending.insert(id);
	// the nodes touched take the values of their ends too, the lines going on from them laid again where those changed; a few rounds
	// settle it, each line going on from a curve depending on one node at each end
	std::set<node_id> nodes{Change.touched_nodes()};
	for (int round = 0; round < 8 && (false == pending.empty() || false == nodes.empty()); ++round)
	{
		for (auto const id : pending)
		{
			auto &laid{Change.change(id)};
			auto const &view{Change.view()};
			laid.solved = solve(laid.shape, view.condition(laid, line_end::start), view.condition(laid, line_end::end));
			nodes.insert(laid.start);
			nodes.insert(laid.end);
		}
		std::set<line_id> next;
		for (auto const id : nodes)
		{
			auto const *existing{Change.view().find(id)};
			if (existing == nullptr)
				continue;
			auto updated{*existing};
			auto const curved{derive(Change.view(), updated)};
			if (updated != *existing)
				Change.change(id) = updated;
			if (false == curved)
				continue;
			for (auto const &end : updated.ends)
				if (auto const *owner{Change.view().find(end.line)}; owner != nullptr && owner->continues(end.end))
					next.insert(end.line);
		}
		nodes.clear();
		pending = std::move(next);
	}
}

std::vector<chain_part> chain_of(network const &Network, line_id const Line)
{
	std::vector<chain_part> result;
	auto const *first{Network.find(Line)};
	if (first == nullptr)
		return result;
	result.push_back({Line, true});
	auto const contains = [&](line_id const Id) { return std::ranges::any_of(result, [&](chain_part const &Part) { return Part.line == Id; }); };
	// the other end at the node, where it holds two
	auto const across = [&](node_id const At, line_id const From, line_end const End) -> end_ref const * {
		auto const *at{Network.find(At)};
		if (at == nullptr || at->ends.size() != 2)
			return nullptr;
		for (auto const &end : at->ends)
			if (end.line != From || end.end != End)
				return &end;
		return nullptr;
	};
	// on beyond the end of the last part
	while (true)
	{
		auto const &last{result.back()};
		auto const *owner{Network.find(last.line)};
		if (owner == nullptr)
			break;
		auto const beyond{last.forward ? line_end::end : line_end::start};
		auto const *other{across(owner->node_at(beyond), last.line, beyond)};
		if (other == nullptr || contains(other->line) || Network.find(other->line) == nullptr)
			break;
		result.push_back({other->line, other->end == line_end::start});
	}
	// and back beyond the start of the first one
	while (true)
	{
		auto const &head{result.front()};
		auto const *owner{Network.find(head.line)};
		if (owner == nullptr)
			break;
		auto const behind{head.forward ? line_end::start : line_end::end};
		auto const *other{across(owner->node_at(behind), head.line, behind)};
		if (other == nullptr || contains(other->line) || Network.find(other->line) == nullptr)
			break;
		result.insert(result.begin(), {other->line, other->end == line_end::end});
	}
	return result;
}

described_chain describe_chain(network const &Network, std::vector<chain_part> const &Chain)
{
	described_chain result;
	double offset{0.0};
	for (auto const &part : Chain)
	{
		// the chain is made of the lines of the network: one gone is a chain out of date, of which the rest is described
		auto const *found{Network.find(part.line)};
		result.starts.push_back(offset);
		if (found == nullptr)
			continue;
		auto const &owner{*found};
		auto own{described(owner.solved, owner.category)};
		auto const length{owner.solved.length};
		if (false == part.forward)
		{
			for (auto &element : own.elements)
				element = reversed(element, length);
			std::ranges::reverse(own.elements);
			for (auto &point : own.cant)
				point = {length - point.chainage, -point.cant};
			std::ranges::reverse(own.cant);
			for (auto &grade : own.grades)
				grade = {length - (grade.start + grade.length), grade.length, -grade.grade_end, -grade.grade_start};
			std::ranges::reverse(own.grades);
		}
		for (auto element : own.elements)
		{
			element.start += offset;
			auto &elements{result.line.elements};
			if (false == elements.empty() && runs_on(elements.back(), element))
			{
				elements.back().length = element.end() - elements.back().start;
				elements.back().curvature_end = element.curvature_end;
				elements.back().to = element.to;
				continue;
			}
			elements.push_back(element);
		}
		for (auto point : own.cant)
		{
			point.chainage += offset;
			auto &cant{result.line.cant};
			if (false == cant.empty() && std::abs(cant.back().chainage - point.chainage) < 1e-9 && std::abs(cant.back().cant - point.cant) < 1e-9)
				continue;
			cant.push_back(point);
		}
		for (auto grade : own.grades)
		{
			grade.start += offset;
			result.line.grades.push_back(grade);
		}
		offset += length;
	}
	if (false == result.line.elements.empty())
	{
		result.line.elements.front().open_start = true;
		result.line.elements.back().open_end = true;
	}
	if (false == Chain.empty())
		result.line.name = std::format("l{}", number(Chain.front().line));
	return result;
}

std::string finding_text(norms::finding const &Finding)
{
	auto text{std::format("{} {:.4g}, the limit {:.4g}", parameter_name(Finding.what), Finding.value, Finding.limit)};
	if (false == Finding.from.document.empty())
		text += std::format(" ({} {})", Finding.from.document, Finding.from.place);
	return text;
}

namespace
{

// the invariants of the line: laid between its nodes, which know its ends
void check_line(network const &Network, line const &Line, verdict &Result)
{
	for (auto const &error : Line.solved.errors)
		Result.reasons.push_back(std::format("{}: {}", line_name(Line.id), error));
	if (Line.start == Line.end)
		Result.reasons.push_back(std::format("{} starts and ends at {}", line_name(Line.id), node_name(Line.start)));
	for (auto const end : {line_end::start, line_end::end})
	{
		if (Network.find(Line.node_at(end)) == nullptr)
			Result.reasons.push_back(std::format("{}: no {} at its {}", line_name(Line.id), node_name(Line.node_at(end)), end == line_end::start ? "start" : "end"));
		else if (Network.end_of(Line.id, end) == nullptr)
			Result.reasons.push_back(std::format("{} doesn't hold the {} of {}", node_name(Line.node_at(end)), end == line_end::start ? "start" : "end", line_name(Line.id)));
	}
}

// the invariants of the node: its ends exist, two at most, on its two sides, and run on as one track through it
void check_node(network const &Network, node const &Node, verdict &Result)
{
	if (Node.ends.empty())
		Result.reasons.push_back(std::format("{} holds no end of a line", node_name(Node.id)));
	if (Node.ends.size() > 2)
		Result.reasons.push_back(std::format("{} holds {} ends; more than two meet only at a turnout", node_name(Node.id), Node.ends.size()));
	if (Node.ends.size() == 2 && Node.ends[0].side() == Node.ends[1].side())
		Result.reasons.push_back(std::format("{}: both lines lie on the same side of it", node_name(Node.id)));
	for (auto const &end : Node.ends)
	{
		auto const *owner{Network.find(end.line)};
		if (owner == nullptr || owner->node_at(end.end) != Node.id || std::abs(end.sense) != 1)
		{
			Result.reasons.push_back(std::format("{} holds an end of {}, which doesn't end there", node_name(Node.id), line_name(end.line)));
			continue;
		}
		if (false == owner->solved.valid())
			continue;
		auto const &values{end.end == line_end::start ? owner->solved.start : owner->solved.end};
		if (values.position != Node.position)
			Result.reasons.push_back(std::format("{} doesn't end at {}", line_name(end.line), node_name(Node.id)));
		auto const ends{seen(*owner, end)};
		auto const kink{std::abs(geometry::signed_angle(Node.heading, ends.heading))};
		if (kink > kHeading)
			Result.reasons.push_back(std::format("{} meets {} at an angle of {:.6f} deg", line_name(end.line), node_name(Node.id), glm::degrees(kink)));
		if (std::abs(ends.cant - Node.cant) > kCantStep)
			Result.reasons.push_back(std::format("the cant steps by {:.1f} mm at {} where {} meets it", std::abs(ends.cant - Node.cant), node_name(Node.id), line_name(end.line)));
		if (std::abs(ends.grade - Node.grade) > kGradeBreak)
			Result.reasons.push_back(std::format("the grade breaks by {:.1f} per mille at {} with no vertical curve", 1000.0 * std::abs(ends.grade - Node.grade), node_name(Node.id)));
		// a step of the curvature, which the norms judge along the chain (the step of the cant deficiency); one going on from the node
		// has its curvature by construction
		if (owner->continues(end.end) && std::abs(ends.curvature - Node.curvature) > kCurvature)
			Result.reasons.push_back(std::format("{} goes on from the curve of {} at another curvature", line_name(end.line), node_name(Node.id)));
	}
}

void check_norms(network const &Network, std::set<line_id> const &Lines, verdict &Result)
{
	std::set<line_id> done;
	for (auto const id : Lines)
	{
		if (done.contains(id) || Network.find(id) == nullptr)
			continue;
		auto const chain{chain_of(Network, id)};
		bool laid{true};
		for (auto const &part : chain)
		{
			done.insert(part.line);
			auto const *line{Network.find(part.line)};
			laid = laid && line != nullptr && line->solved.valid();
		}
		if (false == laid)
			continue;
		auto const described{describe_chain(Network, chain)};
		auto const &first{*Network.find(chain.front().line)}; // NOSONAR: every line of the chain was found just above
		for (auto const &finding : norms::evaluate(described.line, first.category, Network.areas))
		{
			if (finding.level == norms::tier::none)
				continue;
			// the line it's on, by the chainage along the chain
			auto part{static_cast<std::size_t>(std::upper_bound(described.starts.begin(), described.starts.end(), finding.chainage + 1e-9) - described.starts.begin())};
			part = part > 0 ? part - 1 : 0;
			auto const &owner{*Network.find(chain[part].line)}; // NOSONAR: every line of the chain was found just above
			auto const along{finding.chainage - described.starts[part]};
			auto const chainage{chain[part].forward ? along : owner.solved.length - along};
			auto const text{finding_text(finding)};
			if (finding.level == norms::tier::block)
				Result.reasons.push_back(std::format("{} at {:.1f} m: {}", line_name(owner.id), chainage, text));
			else if (Lines.contains(owner.id))
				Result.notices.push_back({owner.id, chainage, finding, text});
		}
	}
}

} // namespace

verdict check(network const &Network, std::set<line_id> const &Lines, std::set<node_id> const &Nodes)
{
	verdict result;
	std::set<node_id> nodes{Nodes};
	for (auto const id : Lines)
	{
		auto const *owner{Network.find(id)};
		if (owner == nullptr)
			continue;
		check_line(Network, *owner, result);
		nodes.insert(owner->start);
		nodes.insert(owner->end);
	}
	for (auto const id : nodes)
		if (auto const *at{Network.find(id)}; at != nullptr)
			check_node(Network, *at, result);
	if (result.fine())
		check_norms(Network, Lines, result);
	return result;
}

verdict check_all(network const &Network)
{
	std::set<line_id> lines;
	std::set<node_id> nodes;
	for (auto const &[id, owner] : Network.lines())
		lines.insert(id);
	for (auto const &[id, at] : Network.nodes())
		nodes.insert(id);
	return check(Network, lines, nodes);
}

} // namespace layout
