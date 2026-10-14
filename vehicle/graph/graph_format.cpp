/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_format.h"

#include <algorithm>
#include <fstream>
#include <iterator>
#include <map>
#include <set>
#include <nlohmann/json.hpp>

namespace vgraph
{

namespace
{

using json = nlohmann::json;

void report(std::vector<diagnostic> &Diagnostics, severity const Level, std::string Node, std::string Message)
{
	Diagnostics.push_back({Level, std::move(Node), std::move(Message)});
}

bool is_valid_id(std::string const &Id)
{
	if (Id.empty())
	{
		return false;
	}
	return std::all_of(std::begin(Id), std::end(Id), [](char const Char) {
		return (Char >= 'a' && Char <= 'z') || (Char >= 'A' && Char <= 'Z') || (Char >= '0' && Char <= '9') || Char == '_' || Char == '-';
	});
}

// reads a json scalar as a parameter value. text values become choices later, if their definition says so
bool read_value(json const &Value, param_value &Result)
{
	if (Value.is_boolean())
	{
		Result = param_value::make_boolean(Value.get<bool>());
		return true;
	}
	if (Value.is_number())
	{
		Result = param_value::make_number(Value.get<double>());
		return true;
	}
	if (Value.is_string())
	{
		Result = param_value::make_text(Value.get<std::string>());
		return true;
	}
	if (Value.is_array())
	{
		// table: array of rows, each row an array of numbers
		std::vector<std::vector<double>> rows;
		rows.reserve(Value.size());
		for (auto const &row : Value)
		{
			if (false == row.is_array())
			{
				return false;
			}
			std::vector<double> cells;
			cells.reserve(row.size());
			for (auto const &cell : row)
			{
				if (false == cell.is_number())
				{
					return false;
				}
				cells.push_back(cell.get<double>());
			}
			rows.emplace_back(std::move(cells));
		}
		Result = param_value::make_table(std::move(rows));
		return true;
	}
	return false;
}

void read_values(json const &Object, std::vector<named_value> &Values, std::vector<diagnostic> &Diagnostics, std::string const &Node, std::string const &What)
{
	for (auto const &item : Object.items())
	{
		param_value value;
		if (read_value(item.value(), value))
		{
			Values.emplace_back(item.key(), std::move(value));
		}
		else
		{
			report(Diagnostics, severity::error, Node, What + " \"" + item.key() + "\" has to be a number, a boolean, a text or a table (array of rows of numbers)");
		}
	}
}

// splits "node.port" into its parts
bool split_endpoint(json const &Value, std::string &Node, std::string &Port)
{
	if (false == Value.is_string())
	{
		return false;
	}
	auto const text{Value.get<std::string>()};
	auto const dot{text.find('.')};
	if (dot == std::string::npos || dot == 0 || dot + 1 == text.size())
	{
		return false;
	}
	Node = text.substr(0, dot);
	Port = text.substr(dot + 1);
	return true;
}

bool parse_document_(std::string const &Text, document &Document, std::vector<diagnostic> &Diagnostics)
{
	auto const errorcount{std::count_if(std::begin(Diagnostics), std::end(Diagnostics), [](diagnostic const &Diagnostic) { return Diagnostic.level == severity::error; })};

	// comments are allowed, to make hand editing of the files easier
	auto const root = json::parse(Text, nullptr, false, true);
	if (root.is_discarded() || false == root.is_object())
	{
		report(Diagnostics, severity::error, "", "the file isn't a valid json object");
		return false;
	}
	if (root.value("format", std::string{}) != file_format)
	{
		report(Diagnostics, severity::error, "", "the file isn't a vehicle graph (missing \"format\": \"" + std::string{file_format} + "\")");
		return false;
	}
	auto const version{root.value("version", 0)};
	if (version != file_version)
	{
		report(Diagnostics, severity::error, "", "unsupported file version " + std::to_string(version) + ", expected " + std::to_string(file_version) +
		                                             (version < file_version ? "; open and save the file in the current editor to convert it" : ""));
		return false;
	}

	if (root.contains("vehicle") && root["vehicle"].is_object())
	{
		Document.name = root["vehicle"].value("name", "");
	}
	if (root.contains("settings"))
	{
		auto const &settings = root["settings"];
		if (settings.is_object())
		{
			for (auto const &item : settings.items())
			{
				if (item.key() == "low_voltage_simulation" && item.value().is_boolean())
				{
					Document.low_voltage_simulation = item.value().get<bool>();
				}
				else
				{
					report(Diagnostics, severity::warning, "", "unknown or malformed setting \"" + item.key() + "\" was ignored");
				}
			}
		}
		else
		{
			report(Diagnostics, severity::error, "", "\"settings\" has to be an object");
		}
	}
	if (root.contains("body"))
	{
		if (root["body"].is_object())
		{
			read_values(root["body"], Document.body, Diagnostics, "", "body parameter");
		}
		else
		{
			report(Diagnostics, severity::error, "", "\"body\" has to be an object");
		}
	}
	if (root.contains("profiles"))
	{
		if (root["profiles"].is_object())
		{
			for (auto const &item : root["profiles"].items())
			{
				if (item.value().is_number_integer())
				{
					Document.profiles.emplace_back(item.key(), item.value().get<int>());
				}
				else
				{
					report(Diagnostics, severity::error, "", "version of the interface profile \"" + item.key() + "\" has to be an integer");
				}
			}
		}
		else
		{
			report(Diagnostics, severity::error, "", "\"profiles\" has to be an object");
		}
	}

	// nodes
	if (false == root.contains("nodes") || false == root["nodes"].is_array())
	{
		report(Diagnostics, severity::error, "", "missing \"nodes\" array");
		return false;
	}
	std::set<std::string> ids;
	for (auto const &item : root["nodes"])
	{
		if (false == item.is_object())
		{
			report(Diagnostics, severity::error, "", "every entry of \"nodes\" has to be an object");
			continue;
		}
		node_desc node;
		node.id = item.value("id", "");
		node.type = item.value("type", "");
		node.title = item.value("title", "");
		node.script = item.value("lua", "");
		if (false == is_valid_id(node.id))
		{
			report(Diagnostics, severity::error, node.id, "node id has to be made of letters, digits, '_' and '-'");
			continue;
		}
		if (false == ids.insert(node.id).second)
		{
			report(Diagnostics, severity::error, node.id, "duplicate node id");
			continue;
		}
		if (node.type.empty())
		{
			report(Diagnostics, severity::error, node.id, "missing block type");
			continue;
		}
		if (item.contains("params"))
		{
			if (item["params"].is_object())
			{
				read_values(item["params"], node.params, Diagnostics, node.id, "parameter");
			}
			else
			{
				report(Diagnostics, severity::error, node.id, "\"params\" has to be an object");
			}
		}
		Document.nodes.emplace_back(std::move(node));
	}

	// connections
	if (root.contains("connections"))
	{
		if (root["connections"].is_array())
		{
			for (auto const &item : root["connections"])
			{
				connection_desc connection;
				if (false == item.is_object() || false == split_endpoint(item.value("from", json{}), connection.from_node, connection.from_port) ||
				    false == split_endpoint(item.value("to", json{}), connection.to_node, connection.to_port))
				{
					report(Diagnostics, severity::error, "", "malformed connection " + item.dump() + ", expected {\"from\": \"node.port\", \"to\": \"node.port\"}");
					continue;
				}
				Document.connections.emplace_back(std::move(connection));
			}
		}
		else
		{
			report(Diagnostics, severity::error, "", "\"connections\" has to be an array");
		}
	}
	// "layout" holds the positions of the blocks in the editor and is of no interest to the simulator

	return errorcount == std::count_if(std::begin(Diagnostics), std::end(Diagnostics), [](diagnostic const &Diagnostic) { return Diagnostic.level == severity::error; });
}

} // namespace

namespace
{

struct endpoint
{
	std::string node;
	std::string port;
};

std::string directory_of(std::string const &Path)
{
	auto const separator{Path.find_last_of("/\\")};
	return separator == std::string::npos ? std::string{} : Path.substr(0, separator + 1);
}

bool expand_modules_(document &Document, std::string const &Directory, std::vector<diagnostic> &Diagnostics, std::vector<std::string> &Chain)
{
	auto ok{true};
	std::vector<node_desc> nodes;
	std::vector<connection_desc> connections{Document.connections};
	for (auto &node : Document.nodes)
	{
		if (node.type != module_type)
		{
			nodes.emplace_back(std::move(node));
			continue;
		}
		std::string file;
		for (auto const &param : node.params)
		{
			if (param.first == "file")
			{
				file = param.second.text;
			}
			else
			{
				report(Diagnostics, severity::warning, node.id, "unknown parameter \"" + param.first + "\" of the module was ignored");
			}
		}
		std::replace(std::begin(file), std::end(file), '\\', '/');
		if (file.empty())
		{
			report(Diagnostics, severity::error, node.id, "module without a file");
			ok = false;
			continue;
		}
		auto const path{(file.front() == '/' || (file.size() > 1 && file[1] == ':')) ? file : Directory + file};
		if (std::find(std::begin(Chain), std::end(Chain), path) != std::end(Chain) || Chain.size() >= 16)
		{
			report(Diagnostics, severity::error, node.id, "module \"" + file + "\" uses itself");
			ok = false;
			continue;
		}
		// problems of the module file are reported under the module node
		std::vector<diagnostic> problems;
		document module;
		auto loaded{load_document(path, module, problems)};
		if (loaded)
		{
			Chain.push_back(path);
			loaded = expand_modules_(module, directory_of(path), problems, Chain);
			Chain.pop_back();
		}
		if (false == module.body.empty() || false == module.profiles.empty())
		{
			problems.push_back({severity::warning, "", "the body and the profiles of a module are ignored"});
		}
		for (auto const &problem : problems)
		{
			report(Diagnostics, problem.level, node.id + (problem.node.empty() ? "" : "/" + problem.node), "module \"" + file + "\": " + problem.message);
		}
		if (false == loaded)
		{
			ok = false;
			continue;
		}

		auto const prefix{node.id + "/"};
		std::set<std::string> inputs, outputs;
		std::map<std::string, node_desc> inputnodes;
		for (auto &inner : module.nodes)
		{
			// the interface of this module; blocks of the modules it uses have ids with '/', and an input block kept
			// by such a module (for its unconnected input) isn't an input of this one
			auto const own{inner.id.find('/') == std::string::npos};
			if (own && inner.type == "module.input")
			{
				inputs.insert(inner.id);
				inputnodes.emplace(inner.id, inner);
			}
			else if (own && inner.type == "module.output")
			{
				outputs.insert(inner.id);
			}
			else
			{
				inner.id = prefix + inner.id;
				nodes.emplace_back(std::move(inner));
			}
		}
		// connections inside the module
		std::map<std::string, std::vector<endpoint>> inputtargets; // input of the module -> inputs of its blocks
		std::map<std::string, endpoint> outputsources; // output of the module -> output of a block, or "input" node for a passed input
		for (auto const &inner : module.connections)
		{
			auto const frominput{inputs.count(inner.from_node) > 0};
			if (outputs.count(inner.to_node) > 0)
			{
				outputsources[inner.to_node] = frominput ? endpoint{inner.from_node, ""} : endpoint{prefix + inner.from_node, inner.from_port};
			}
			else if (frominput)
			{
				inputtargets[inner.from_node].push_back({prefix + inner.to_node, inner.to_port});
			}
			else
			{
				connections.push_back({prefix + inner.from_node, inner.from_port, prefix + inner.to_node, inner.to_port});
			}
		}
		// connections of the module node in the graph using it
		std::map<std::string, endpoint> externalsources; // input of the module -> what feeds it
		std::map<std::string, std::vector<endpoint>> externaltargets; // output of the module -> what it feeds
		for (auto it = std::begin(connections); it != std::end(connections);)
		{
			if (it->to_node == node.id)
			{
				if (inputs.count(it->to_port) == 0)
				{
					report(Diagnostics, severity::error, node.id, "module \"" + file + "\" has no input \"" + it->to_port + "\"");
					ok = false;
				}
				externalsources[it->to_port] = {it->from_node, it->from_port};
				it = connections.erase(it);
			}
			else if (it->from_node == node.id)
			{
				if (outputs.count(it->from_port) == 0)
				{
					report(Diagnostics, severity::error, node.id, "module \"" + file + "\" has no output \"" + it->from_port + "\"");
					ok = false;
				}
				externaltargets[it->from_port].push_back({it->to_node, it->to_port});
				it = connections.erase(it);
			}
			else
			{
				++it;
			}
		}
		// an input fed from an output of the same module (a loop through the module) is fed by the block inside it
		for (auto &source : externalsources)
		{
			if (source.second.node != node.id)
			{
				continue;
			}
			auto const inner{outputsources.find(source.second.port)};
			if (inner != std::end(outputsources) && false == inner->second.port.empty())
			{
				source.second = inner->second;
			}
		}
		// an input without a connection keeps its interface block, which gives the default value of the input
		auto const source_of = [&](std::string const &Input) -> endpoint {
			auto const source{externalsources.find(Input)};
			if (source != std::end(externalsources))
			{
				return source->second;
			}
			auto const id{prefix + Input};
			if (std::none_of(std::begin(nodes), std::end(nodes), [&](node_desc const &Node) { return Node.id == id; }))
			{
				auto interfacenode{inputnodes[Input]};
				interfacenode.id = id;
				nodes.emplace_back(std::move(interfacenode));
			}
			return {id, "out"};
		};
		for (auto const &input : inputtargets)
		{
			auto const source{source_of(input.first)};
			for (auto const &target : input.second)
			{
				connections.push_back({source.node, source.port, target.node, target.port});
			}
		}
		for (auto const &output : externaltargets)
		{
			auto source{outputsources.find(output.first)};
			if (source == std::end(outputsources))
			{
				continue;
			}
			auto from{source->second};
			if (from.port.empty())
			{
				// an input passed straight to an output
				from = source_of(from.node);
			}
			for (auto const &target : output.second)
			{
				connections.push_back({from.node, from.port, target.node, target.port});
			}
		}
	}
	Document.nodes = std::move(nodes);
	Document.connections = std::move(connections);
	return ok;
}

} // namespace

bool expand_modules(document &Document, std::string const &Directory, std::vector<diagnostic> &Diagnostics)
{
	std::vector<std::string> chain;
	return expand_modules_(Document, Directory, Diagnostics, chain);
}

bool parse_document(std::string const &Text, document &Document, std::vector<diagnostic> &Diagnostics)
{
	try
	{
		return parse_document_(Text, Document, Diagnostics);
	}
	catch (json::exception const &Error)
	{
		// values of unexpected type, e.g. a number where the id should be
		report(Diagnostics, severity::error, "", std::string{"malformed file: "} + Error.what());
		return false;
	}
}

bool load_document(std::string const &Path, document &Document, std::vector<diagnostic> &Diagnostics)
{
	std::ifstream file{Path, std::ios::binary};
	if (false == file.is_open())
	{
		report(Diagnostics, severity::error, "", "can't open file \"" + Path + "\"");
		return false;
	}
	std::string const text{std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>()};
	return parse_document(text, Document, Diagnostics);
}

} // namespace vgraph
