/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_catalog.h"
#include "vehicle/graph/graph_body.h"
#include "input/command.h"

#include <nlohmann/json.hpp>

namespace vgraph
{

std::string_view to_string(block_origin const Origin)
{
	switch (Origin)
	{
	case block_origin::cab:
		return "cab";
	case block_origin::simulator:
		return "simulator";
	case block_origin::extracted:
		return "extracted";
	case block_origin::native:
		return "native";
	case block_origin::interface_profile:
		return "interface";
	case block_origin::script:
		return "script";
	default:
		return "primitive";
	}
}

int block_type::port_index(std::string_view const Key) const
{
	for (std::size_t idx = 0; idx < ports.size(); ++idx)
	{
		if (ports[idx].key == Key)
		{
			return static_cast<int>(idx);
		}
	}
	return -1;
}

int block_type::param_index(std::string_view const Key) const
{
	for (std::size_t idx = 0; idx < params.size(); ++idx)
	{
		if (params[idx].key == Key)
		{
			return static_cast<int>(idx);
		}
	}
	return -1;
}

catalog const &catalog::get()
{
	static catalog const instance{[]() {
		catalog result;
		register_logic_blocks(result);
		register_cab_blocks(result);
		register_script_blocks(result);
		register_vehicle_blocks(result);
		register_pneumatic_blocks(result);
		register_security_blocks(result);
		register_electric_blocks(result);
		register_drive_blocks(result);
		register_diesel_blocks(result);
		register_interface_blocks(result);
		register_module_blocks(result);
		return result;
	}()};
	return instance;
}

block_type const *catalog::find(std::string_view const Id) const
{
	for (auto const &type : m_types)
	{
		if (type.id == Id)
		{
			return &type;
		}
	}
	return nullptr;
}

void catalog::add(block_type Type)
{
	m_types.emplace_back(std::move(Type));
}

namespace
{

nlohmann::json to_json(param_value const &Value)
{
	switch (Value.type)
	{
	case param_type::boolean:
		return Value.boolean;
	case param_type::text:
	case param_type::choice:
		return Value.text;
	case param_type::table:
	{
		auto result{nlohmann::json::array()};
		for (auto const &row : Value.rows)
		{
			result.push_back(row);
		}
		return result;
	}
	default:
		return Value.number;
	}
}

} // namespace

std::string catalog::export_json() const
{
	nlohmann::ordered_json root;
	root["format"] = "maszyna-vehicle-graph-catalog";
	root["version"] = 1;
	root["graph_format"] = std::string{file_format};
	root["graph_version"] = file_version;

	auto &domains = (root["domains"] = nlohmann::ordered_json::array());
	for (std::size_t idx = 0; idx < static_cast<std::size_t>(domain::count_); ++idx)
	{
		auto const &domaininfo{info(static_cast<domain>(idx))};
		nlohmann::ordered_json item;
		item["id"] = std::string{domaininfo.id};
		item["physical"] = domaininfo.physical;
		item["forward"] = std::string{domaininfo.forward};
		item["forward_unit"] = std::string{domaininfo.forward_unit};
		if (domaininfo.physical)
		{
			item["back"] = std::string{domaininfo.back};
			item["back_unit"] = std::string{domaininfo.back_unit};
		}
		domains.push_back(std::move(item));
	}

	auto &blocks = (root["blocks"] = nlohmann::ordered_json::array());
	for (auto const &type : m_types)
	{
		nlohmann::ordered_json item;
		item["id"] = type.id;
		item["category"] = type.category;
		item["name"] = type.name;
		item["description"] = type.description;
		item["origin"] = std::string{to_string(type.origin)};
		item["deferred"] = type.deferred;
		if (type.single)
		{
			// at most one block of this type in a graph
			item["single"] = true;
		}
		if (type.dynamic_ports)
		{
			// ports and parameters come from the node, e.g. from the script of a lua block
			item["dynamic_ports"] = true;
		}
		auto &ports = (item["ports"] = nlohmann::ordered_json::array());
		for (auto const &port : type.ports)
		{
			nlohmann::ordered_json portitem;
			portitem["key"] = port.key;
			portitem["direction"] = (port.direction == port_direction::in ? "in" : "out");
			portitem["domain"] = std::string{info(port.dom).id};
			if (port.dom == domain::signal)
			{
				portitem["type"] = std::string{to_string(port.type)};
			}
			if (false == port.unit.empty())
			{
				portitem["unit"] = port.unit;
			}
			if (port.direction == port_direction::in)
			{
				portitem["optional"] = port.optional;
				if (port.dom == domain::signal)
				{
					portitem["default"] = port.default_value;
				}
			}
			ports.push_back(std::move(portitem));
		}
		auto &params = (item["params"] = nlohmann::ordered_json::array());
		for (auto const &param : type.params)
		{
			nlohmann::ordered_json paramitem;
			paramitem["key"] = param.key;
			paramitem["type"] = std::string{to_string(param.type)};
			if (false == param.unit.empty())
			{
				paramitem["unit"] = param.unit;
			}
			paramitem["default"] = to_json(param.default_value);
			if (param.type == param_type::choice)
			{
				paramitem["choices"] = param.choices;
			}
			if (param.type == param_type::table)
			{
				paramitem["columns"] = param.columns;
			}
			if (false == param.fiz_key.empty())
			{
				paramitem["fiz"] = param.fiz_section + " " + param.fiz_key;
			}
			params.push_back(std::move(paramitem));
		}
		blocks.push_back(std::move(item));
	}

	// parameters of the vehicle body, for the form of the editor
	auto &body = (root["body"] = nlohmann::ordered_json::array());
	for (auto const &field : body_schema())
	{
		nlohmann::ordered_json item;
		item["key"] = field.key;
		item["type"] = std::string{to_string(field.type)};
		if (false == field.unit.empty())
		{
			item["unit"] = field.unit;
		}
		item["default"] = to_json(field.default_value);
		if (field.type == param_type::choice)
		{
			item["choices"] = field.choices;
		}
		item["fiz"] = field.fiz_section + " " + field.fiz_key;
		body.push_back(std::move(item));
	}
	auto &profiles = (root["profiles"] = nlohmann::ordered_json::array());
	for (auto const &profile : known_profiles())
	{
		nlohmann::ordered_json item;
		item["id"] = std::string{profile.id};
		item["version"] = profile.version;
		profiles.push_back(std::move(item));
	}
	// commands of the simulator, which the cab switches can be bound to (by name), for the pickers of the editor
	auto &commands = (root["commands"] = nlohmann::ordered_json::array());
	{
		std::vector<std::pair<std::size_t, std::string>> names;
		for (auto const &entry : simulation::commandMap)
		{
			if (entry.second != user_command::none)
			{
				names.emplace_back(static_cast<std::size_t>(entry.second), entry.first);
			}
		}
		// in the order of the command list, which groups the related commands
		std::sort(std::begin(names), std::end(names));
		for (auto const &name : names)
		{
			nlohmann::ordered_json item;
			item["name"] = name.second;
			if (name.first < simulation::Commands_descriptions.size())
			{
				auto const &description{simulation::Commands_descriptions[name.first]};
				switch (description.target)
				{
				case command_target::userinterface:
					item["target"] = "userinterface";
					break;
				case command_target::vehicle:
					item["target"] = "vehicle";
					break;
				case command_target::signal:
					item["target"] = "signal";
					break;
				case command_target::entity:
					item["target"] = "entity";
					break;
				case command_target::simulation:
					item["target"] = "simulation";
					break;
				default:
					break;
				}
				item["mode"] = (description.mode == command_mode::continuous ? "continuous" : "oneoff");
			}
			commands.push_back(std::move(item));
		}
	}
	// live preview server of the simulator, see the README
	root["preview"] = "protocol 1";
	return root.dump(1, '\t');
}

port_def signal_in(std::string Key, value_type const Type, double const Default, std::string Unit, bool const Optional)
{
	port_def result;
	result.key = std::move(Key);
	result.direction = port_direction::in;
	result.dom = domain::signal;
	result.type = Type;
	result.unit = std::move(Unit);
	result.optional = Optional;
	result.default_value = Default;
	return result;
}

port_def signal_out(std::string Key, value_type const Type, std::string Unit)
{
	port_def result;
	result.key = std::move(Key);
	result.direction = port_direction::out;
	result.dom = domain::signal;
	result.type = Type;
	result.unit = std::move(Unit);
	return result;
}

port_def physical_in(std::string Key, domain const Domain, bool const Optional)
{
	port_def result;
	result.key = std::move(Key);
	result.direction = port_direction::in;
	result.dom = Domain;
	result.optional = Optional;
	return result;
}

port_def physical_out(std::string Key, domain const Domain)
{
	port_def result;
	result.key = std::move(Key);
	result.direction = port_direction::out;
	result.dom = Domain;
	return result;
}

param_def number_param(std::string Key, double const Default, std::string Unit)
{
	param_def result;
	result.key = std::move(Key);
	result.type = param_type::number;
	result.unit = std::move(Unit);
	result.default_value = param_value::make_number(Default);
	return result;
}

param_def boolean_param(std::string Key, bool const Default)
{
	param_def result;
	result.key = std::move(Key);
	result.type = param_type::boolean;
	result.default_value = param_value::make_boolean(Default);
	return result;
}

param_def text_param(std::string Key, std::string Default)
{
	param_def result;
	result.key = std::move(Key);
	result.type = param_type::text;
	result.default_value = param_value::make_text(std::move(Default));
	return result;
}

param_def choice_param(std::string Key, std::vector<std::string> Choices, std::string Default)
{
	param_def result;
	result.key = std::move(Key);
	result.type = param_type::choice;
	result.choices = std::move(Choices);
	result.default_value = param_value::make_choice(std::move(Default));
	return result;
}

param_def table_param(std::string Key, std::vector<std::string> Columns)
{
	param_def result;
	result.key = std::move(Key);
	result.type = param_type::table;
	result.columns = std::move(Columns);
	result.default_value = param_value::make_table({});
	return result;
}

param_def fiz(param_def Definition, std::string Section, std::string Key)
{
	Definition.fiz_section = std::move(Section);
	Definition.fiz_key = std::move(Key);
	return Definition;
}

param_def cab_param()
{
	// "0": the engine room (cab0definition: of the .mmd files)
	return choice_param("cab", {"any", "1", "2", "0"}, "any");
}

} // namespace vgraph
