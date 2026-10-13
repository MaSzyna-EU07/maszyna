/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_template.h"
#include "vehicle/graph/graph_catalog.h"

#include <functional>
#include <queue>
#include <unordered_map>

namespace vgraph
{

namespace
{

void report(std::vector<diagnostic> &Diagnostics, severity const Level, std::string Node, std::string Message)
{
	Diagnostics.push_back({Level, std::move(Node), std::move(Message)});
}

std::string join_choices(std::vector<std::string> const &Choices)
{
	std::string result;
	for (auto const &choice : Choices)
	{
		if (false == result.empty())
		{
			result += ", ";
		}
		result += choice;
	}
	return result;
}

// checks a value read from the file against the parameter definition; text values become choices here
bool resolve_param(param_def const &Definition, param_value const &Value, param_value &Result, std::string &Problem)
{
	switch (Definition.type)
	{
	case param_type::number:
	{
		if (Value.type != param_type::number)
		{
			Problem = "has to be a number";
			return false;
		}
		Result = Value;
		return true;
	}
	case param_type::boolean:
	{
		if (Value.type != param_type::boolean)
		{
			Problem = "has to be true or false";
			return false;
		}
		Result = Value;
		return true;
	}
	case param_type::text:
	{
		if (Value.type != param_type::text)
		{
			Problem = "has to be a text";
			return false;
		}
		Result = Value;
		return true;
	}
	case param_type::choice:
	{
		if (Value.type != param_type::text || std::find(std::begin(Definition.choices), std::end(Definition.choices), Value.text) == std::end(Definition.choices))
		{
			Problem = "has to be one of: " + join_choices(Definition.choices);
			return false;
		}
		Result = param_value::make_choice(Value.text);
		return true;
	}
	default:
	{
		Problem = "has unsupported type";
		return false;
	}
	}
}

struct endpoint
{
	int node{-1};
	int port{-1};
};

} // namespace

std::shared_ptr<graph_template const> graph_template::compile(document const &Document, catalog const &Catalog, std::vector<diagnostic> &Diagnostics)
{
	auto const errorcount{std::count_if(std::begin(Diagnostics), std::end(Diagnostics), [](diagnostic const &Diagnostic) { return Diagnostic.level == severity::error; })};
	auto result{std::make_shared<graph_template>()};
	result->m_name = Document.name;
	result->m_lowvoltage = Document.low_voltage_simulation;
	result->m_body = Document.body;
	result->m_profiles = Document.profiles;

	// nodes and their parameters
	std::unordered_map<std::string, int> nodeindices;
	std::unordered_map<std::string, bool> unresolvednodes; // nodes of unknown types, to avoid reporting their connections again
	for (auto const &desc : Document.nodes)
	{
		auto const *type{Catalog.find(desc.type)};
		if (type == nullptr)
		{
			report(Diagnostics, severity::error, desc.id, "unknown block type \"" + desc.type + "\"");
			unresolvednodes.emplace(desc.id, true);
			continue;
		}
		node_entry node;
		node.id = desc.id;
		node.title = desc.title;
		node.type = type;
		node.params.reserve(type->params.size());
		for (auto const &definition : type->params)
		{
			node.params.push_back(definition.default_value);
		}
		for (auto const &param : desc.params)
		{
			auto const paramindex{type->param_index(param.first)};
			if (paramindex < 0)
			{
				report(Diagnostics, severity::warning, desc.id, "unknown parameter \"" + param.first + "\" of block type \"" + type->id + "\" was ignored");
				continue;
			}
			std::string problem;
			if (false == resolve_param(type->params[paramindex], param.second, node.params[paramindex], problem))
			{
				report(Diagnostics, severity::error, desc.id, "parameter \"" + param.first + "\" " + problem);
			}
		}
		nodeindices.emplace(node.id, static_cast<int>(result->m_nodes.size()));
		result->m_nodes.emplace_back(std::move(node));
	}

	auto &nodes{result->m_nodes};
	auto &signaldefaults{result->m_signaldefaults};
	auto &linkcount{result->m_linkcount};

	// outputs own their slots and links
	for (auto &node : nodes)
	{
		node.slots.assign(node.type->ports.size(), -1);
		node.connected.assign(node.type->ports.size(), 0);
		for (std::size_t portindex = 0; portindex < node.type->ports.size(); ++portindex)
		{
			auto const &port{node.type->ports[portindex]};
			if (port.direction != port_direction::out)
			{
				continue;
			}
			if (port.dom == domain::signal)
			{
				node.slots[portindex] = static_cast<int>(signaldefaults.size());
				signaldefaults.push_back(0.0);
			}
			else
			{
				node.slots[portindex] = static_cast<int>(linkcount++);
			}
		}
	}

	// connections
	std::vector<std::vector<endpoint>> sources(nodes.size());
	for (std::size_t nodeindex = 0; nodeindex < nodes.size(); ++nodeindex)
	{
		sources[nodeindex].resize(nodes[nodeindex].type->ports.size());
	}
	std::vector<std::vector<int>> edges(nodes.size()); // ordering constraints: source node -> consumer node
	bool lowvoltageignored{false};
	for (auto const &connection : Document.connections)
	{
		auto const label{connection.from_node + "." + connection.from_port + " -> " + connection.to_node + "." + connection.to_port};
		auto const fromlookup{nodeindices.find(connection.from_node)};
		auto const tolookup{nodeindices.find(connection.to_node)};
		if (fromlookup == nodeindices.end() || tolookup == nodeindices.end())
		{
			auto const &missing{fromlookup == nodeindices.end() ? connection.from_node : connection.to_node};
			if (unresolvednodes.find(missing) == unresolvednodes.end())
			{
				report(Diagnostics, severity::error, "", "connection " + label + " refers to unknown node \"" + missing + "\"");
			}
			continue;
		}
		auto const fromnode{fromlookup->second};
		auto const tonode{tolookup->second};
		auto const &fromtype{*nodes[fromnode].type};
		auto const &totype{*nodes[tonode].type};
		auto const fromport{fromtype.port_index(connection.from_port)};
		auto const toport{totype.port_index(connection.to_port)};
		if (fromport < 0 || fromtype.ports[fromport].direction != port_direction::out)
		{
			report(Diagnostics, severity::error, connection.from_node, "connection " + label + ": \"" + connection.from_port + "\" isn't an output of block type \"" + fromtype.id + "\"");
			continue;
		}
		if (toport < 0 || totype.ports[toport].direction != port_direction::in)
		{
			report(Diagnostics, severity::error, connection.to_node, "connection " + label + ": \"" + connection.to_port + "\" isn't an input of block type \"" + totype.id + "\"");
			continue;
		}
		auto const fromdomain{fromtype.ports[fromport].dom};
		auto const todomain{totype.ports[toport].dom};
		if (fromdomain != todomain)
		{
			report(Diagnostics, severity::error, connection.to_node,
			       "connection " + label + " joins different domains: " + std::string{info(fromdomain).id} + " and " + std::string{info(todomain).id});
			continue;
		}
		if (sources[tonode][toport].node >= 0)
		{
			report(Diagnostics, severity::error, connection.to_node, "input \"" + connection.to_port + "\" has more than one source; use a junction block to join them");
			continue;
		}
		if (fromdomain == domain::low_voltage && false == result->m_lowvoltage)
		{
			lowvoltageignored = true;
		}
		sources[tonode][toport] = {fromnode, fromport};
		nodes[fromnode].connected[fromport] = 1;
		nodes[tonode].connected[toport] = 1;
		if (false == fromtype.deferred)
		{
			edges[fromnode].push_back(tonode);
		}
	}
	if (lowvoltageignored)
	{
		report(Diagnostics, severity::warning, "", "low voltage connections have no effect, because low voltage simulation is disabled in the graph settings");
	}

	// inputs read their sources, or slots of their own holding the default values
	for (std::size_t nodeindex = 0; nodeindex < nodes.size(); ++nodeindex)
	{
		auto &node{nodes[nodeindex]};
		for (std::size_t portindex = 0; portindex < node.type->ports.size(); ++portindex)
		{
			auto const &port{node.type->ports[portindex]};
			if (port.direction != port_direction::in)
			{
				continue;
			}
			auto const &source{sources[nodeindex][portindex]};
			if (source.node >= 0)
			{
				node.slots[portindex] = nodes[source.node].slots[source.port];
				continue;
			}
			if (false == port.optional)
			{
				report(Diagnostics, severity::warning, node.id, "required input \"" + port.key + "\" isn't connected");
			}
			if (port.dom == domain::signal)
			{
				node.slots[portindex] = static_cast<int>(signaldefaults.size());
				signaldefaults.push_back(port.default_value);
			}
			else
			{
				node.slots[portindex] = static_cast<int>(linkcount++);
			}
		}
	}

	// order of evaluation. ties are resolved by the order of nodes in the file, so the result is stable
	std::vector<int> indegree(nodes.size(), 0);
	for (auto const &targets : edges)
	{
		for (auto const target : targets)
		{
			++indegree[target];
		}
	}
	std::priority_queue<int, std::vector<int>, std::greater<int>> ready;
	for (std::size_t nodeindex = 0; nodeindex < nodes.size(); ++nodeindex)
	{
		if (indegree[nodeindex] == 0)
		{
			ready.push(static_cast<int>(nodeindex));
		}
	}
	while (false == ready.empty())
	{
		auto const nodeindex{ready.top()};
		ready.pop();
		result->m_order.push_back(nodeindex);
		for (auto const target : edges[nodeindex])
		{
			if (--indegree[target] == 0)
			{
				ready.push(target);
			}
		}
	}
	if (result->m_order.size() != nodes.size())
	{
		// strip nodes merely fed by a loop, so the report lists only the nodes the loops pass through
		std::vector<bool> remaining(nodes.size(), false);
		for (std::size_t nodeindex = 0; nodeindex < nodes.size(); ++nodeindex)
		{
			remaining[nodeindex] = (indegree[nodeindex] > 0);
		}
		bool changed{true};
		while (changed)
		{
			changed = false;
			for (std::size_t nodeindex = 0; nodeindex < nodes.size(); ++nodeindex)
			{
				if (false == remaining[nodeindex])
				{
					continue;
				}
				auto const feedsloop{std::any_of(std::begin(edges[nodeindex]), std::end(edges[nodeindex]), [&](int const Target) { return remaining[Target]; })};
				if (false == feedsloop)
				{
					remaining[nodeindex] = false;
					changed = true;
				}
			}
		}
		std::string loop;
		for (std::size_t nodeindex = 0; nodeindex < nodes.size(); ++nodeindex)
		{
			if (remaining[nodeindex])
			{
				loop += (loop.empty() ? "" : ", ") + nodes[nodeindex].id;
			}
		}
		report(Diagnostics, severity::error, "",
		       "connections form a loop without a deferred block (e.g. logic.sr, logic.delay, logic.ton), so the order of evaluation can't be resolved; nodes in the loop: " + loop);
	}

	auto const failed{errorcount != std::count_if(std::begin(Diagnostics), std::end(Diagnostics), [](diagnostic const &Diagnostic) { return Diagnostic.level == severity::error; })};
	if (failed)
	{
		return nullptr;
	}
	return result;
}

int graph_template::find_node(std::string_view const Id) const
{
	for (std::size_t idx = 0; idx < m_nodes.size(); ++idx)
	{
		if (m_nodes[idx].id == Id)
		{
			return static_cast<int>(idx);
		}
	}
	return -1;
}

int graph_template::find_signal(std::string_view const Endpoint) const
{
	auto const dot{Endpoint.find('.')};
	if (dot == std::string_view::npos)
	{
		return -1;
	}
	auto const nodeindex{find_node(Endpoint.substr(0, dot))};
	if (nodeindex < 0)
	{
		return -1;
	}
	auto const &node{m_nodes[nodeindex]};
	auto const portindex{node.type->port_index(Endpoint.substr(dot + 1))};
	if (portindex < 0 || node.type->ports[portindex].dom != domain::signal)
	{
		return -1;
	}
	return node.slots[portindex];
}

std::shared_ptr<graph_template const> load_graph(std::string const &Path, std::vector<diagnostic> &Diagnostics)
{
	document graph;
	if (false == load_document(Path, graph, Diagnostics))
	{
		return nullptr;
	}
	return graph_template::compile(graph, catalog::get(), Diagnostics);
}

std::vector<std::pair<std::string, int>> graph_template::signal_outputs() const
{
	std::vector<std::pair<std::string, int>> result;
	for (auto const &node : m_nodes)
	{
		for (std::size_t portindex = 0; portindex < node.type->ports.size(); ++portindex)
		{
			auto const &port{node.type->ports[portindex]};
			if (port.direction == port_direction::out && port.dom == domain::signal)
			{
				result.emplace_back(node.id + "." + port.key, node.slots[portindex]);
			}
		}
	}
	return result;
}

} // namespace vgraph
