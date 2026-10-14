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
#include "vehicle/graph/graph_body.h"

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
	case param_type::table:
	{
		if (Value.type != param_type::table)
		{
			Problem = "has to be a table: an array of rows, each an array of numbers";
			return false;
		}
		if (false == Definition.columns.empty())
		{
			for (std::size_t rowindex = 0; rowindex < Value.rows.size(); ++rowindex)
			{
				if (Value.rows[rowindex].size() != Definition.columns.size())
				{
					Problem = "row " + std::to_string(rowindex + 1) + " has " + std::to_string(Value.rows[rowindex].size()) + " values, expected " +
					          std::to_string(Definition.columns.size()) + " (" + join_choices(Definition.columns) + ")";
					return false;
				}
			}
		}
		Result = Value;
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
	result->m_profiles = Document.profiles;

	// body of the vehicle
	for (auto const &entry : Document.body)
	{
		auto const *field{find_body_field(entry.first)};
		if (field == nullptr)
		{
			report(Diagnostics, severity::warning, "", "unknown body parameter \"" + entry.first + "\" was ignored");
			continue;
		}
		param_def definition;
		definition.key = field->key;
		definition.type = field->type;
		definition.choices = field->choices;
		param_value value;
		std::string problem;
		if (false == resolve_param(definition, entry.second, value, problem))
		{
			report(Diagnostics, severity::error, "", "body parameter \"" + entry.first + "\" " + problem);
			continue;
		}
		result->m_body.emplace_back(entry.first, std::move(value));
	}
	// interface profiles
	for (auto const &profile : Document.profiles)
	{
		auto const &known{known_profiles()};
		auto const lookup{std::find_if(std::begin(known), std::end(known), [&](profile_info const &Info) { return Info.id == profile.first; })};
		if (lookup == std::end(known))
		{
			report(Diagnostics, severity::warning, "", "unknown interface profile \"" + profile.first + "\" was ignored");
		}
		else if (profile.second > lookup->version)
		{
			report(Diagnostics, severity::error, "",
			       "interface profile \"" + profile.first + "\" version " + std::to_string(profile.second) + " is newer than the supported version " + std::to_string(lookup->version));
		}
	}

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
		node.script = desc.script;
		if (type->resolve != nullptr)
		{
			std::string problem;
			if (false == type->resolve(desc, node.ports, node.param_defs, problem))
			{
				report(Diagnostics, severity::error, desc.id, problem);
				unresolvednodes.emplace(desc.id, true);
				continue;
			}
		}
		else
		{
			node.ports = type->ports;
			node.param_defs = type->params;
		}
		if (type->id == "lua.block")
		{
			result->m_useslua = true;
		}
		node.params.reserve(node.param_defs.size());
		for (auto const &definition : node.param_defs)
		{
			node.params.push_back(definition.default_value);
		}
		for (auto const &param : desc.params)
		{
			auto const paramindex{find_param(node.param_defs, param.first)};
			if (paramindex < 0)
			{
				report(Diagnostics, severity::warning, desc.id, "unknown parameter \"" + param.first + "\" of block type \"" + type->id + "\" was ignored");
				continue;
			}
			std::string problem;
			if (false == resolve_param(node.param_defs[paramindex], param.second, node.params[paramindex], problem))
			{
				report(Diagnostics, severity::error, desc.id, "parameter \"" + param.first + "\" " + problem);
			}
		}
		nodeindices.emplace(node.id, static_cast<int>(result->m_nodes.size()));
		result->m_nodes.emplace_back(std::move(node));
	}

	auto &nodes{result->m_nodes};
	// blocks standing for parts of the vehicle the simulator has only one of
	{
		std::unordered_map<std::string, std::string> singles;
		for (auto const &node : nodes)
		{
			if (false == node.type->single)
			{
				continue;
			}
			auto const insert{singles.emplace(node.type->id, node.id)};
			if (false == insert.second)
			{
				report(Diagnostics, severity::error, node.id, "the graph can have only one block of type \"" + node.type->id + "\"; the other one is \"" + insert.first->second + "\"");
			}
		}
	}
	auto &signaldefaults{result->m_signaldefaults};
	auto &linkcount{result->m_linkcount};

	// outputs own their slots and links
	for (auto &node : nodes)
	{
		node.slots.assign(node.ports.size(), -1);
		node.connected.assign(node.ports.size(), 0);
		for (std::size_t portindex = 0; portindex < node.ports.size(); ++portindex)
		{
			auto const &port{node.ports[portindex]};
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
		sources[nodeindex].resize(nodes[nodeindex].ports.size());
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
		auto const &fromports{nodes[fromnode].ports};
		auto const &toports{nodes[tonode].ports};
		auto const fromport{find_port(fromports, connection.from_port)};
		auto const toport{find_port(toports, connection.to_port)};
		if (fromport < 0 || fromports[fromport].direction != port_direction::out)
		{
			report(Diagnostics, severity::error, connection.from_node, "connection " + label + ": \"" + connection.from_port + "\" isn't an output of block type \"" + fromtype.id + "\"");
			continue;
		}
		if (toport < 0 || toports[toport].direction != port_direction::in)
		{
			report(Diagnostics, severity::error, connection.to_node, "connection " + label + ": \"" + connection.to_port + "\" isn't an input of block type \"" + totype.id + "\"");
			continue;
		}
		auto const fromdomain{fromports[fromport].dom};
		auto const todomain{toports[toport].dom};
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
		for (std::size_t portindex = 0; portindex < node.ports.size(); ++portindex)
		{
			auto const &port{node.ports[portindex]};
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
	auto const portindex{find_port(node.ports, Endpoint.substr(dot + 1))};
	if (portindex < 0 || node.ports[portindex].dom != domain::signal)
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
	auto const separator{Path.find_last_of("/\\")};
	if (false == expand_modules(graph, separator == std::string::npos ? std::string{} : Path.substr(0, separator + 1), Diagnostics))
	{
		return nullptr;
	}
	auto result{graph_template::compile(graph, catalog::get(), Diagnostics)};
	if (result != nullptr)
	{
		std::const_pointer_cast<graph_template>(result)->m_path = Path;
	}
	return result;
}

std::vector<std::pair<std::string, int>> graph_template::signal_outputs() const
{
	std::vector<std::pair<std::string, int>> result;
	for (auto const &node : m_nodes)
	{
		for (std::size_t portindex = 0; portindex < node.ports.size(); ++portindex)
		{
			auto const &port{node.ports[portindex]};
			if (port.direction == port_direction::out && port.dom == domain::signal)
			{
				result.emplace_back(node.id + "." + port.key, node.slots[portindex]);
			}
		}
	}
	return result;
}

} // namespace vgraph
