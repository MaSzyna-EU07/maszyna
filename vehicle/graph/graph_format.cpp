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

#include <fstream>
#include <iterator>
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
			report(Diagnostics, severity::error, Node, What + " \"" + item.key() + "\" has to be a number, a boolean or a text");
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
