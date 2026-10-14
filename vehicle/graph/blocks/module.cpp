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
#include "vehicle/graph/graph_format.h"

// modules: graphs of logic made once and used by many vehicles. a node of type "module" names the file of the module;
// when the vehicle graph is loaded, the node is replaced with the blocks of the module (expand_modules), connected
// through the module's interface blocks, module.input and module.output, one per port of the module.
// the interface blocks work also on their own, so a module file can be tried with -graph-run: an input gives its
// default value (forced values come from the script), an output shows the value it receives
namespace vgraph
{

namespace
{

std::vector<std::string> interface_types()
{
	std::vector<std::string> result{"bool", "int", "float"};
	for (std::size_t idx = 1; idx < static_cast<std::size_t>(domain::count_); ++idx)
	{
		result.emplace_back(info(static_cast<domain>(idx)).id);
	}
	return result;
}

std::vector<param_def> interface_params(bool const Input)
{
	std::vector<param_def> params{choice_param("type", interface_types(), "float")};
	if (Input)
	{
		params.push_back(number_param("default", 0.0));
	}
	params.push_back(text_param("label"));
	return params;
}

// the port of an interface block, of the type chosen by its "type" parameter
bool interface_port(node_desc const &Node, bool const Input, std::vector<port_def> &Ports, std::vector<param_def> &Params, std::string &Error)
{
	std::string type{"float"};
	for (auto const &param : Node.params)
	{
		if (param.first == "type")
		{
			type = param.second.text;
		}
	}
	// the input of the module is an output of the interface block inside it, and the other way round
	auto const key{Input ? "out" : "in"};
	if (type == "bool" || type == "int" || type == "float")
	{
		auto const valuetype{type == "bool" ? value_type::boolean : type == "int" ? value_type::integer : value_type::real};
		Ports = {Input ? signal_out(key, valuetype) : signal_in(key, valuetype, 0.0, "", true)};
	}
	else
	{
		domain dom;
		if (false == parse_domain(type, dom) || dom == domain::signal)
		{
			Error = "unknown type \"" + type + "\" of the module port; expected bool, int, float or a physical domain";
			return false;
		}
		Ports = {Input ? physical_out(key, dom) : physical_in(key, dom, true)};
	}
	Params = interface_params(Input);
	return true;
}

bool resolve_input(node_desc const &Node, std::vector<port_def> &Ports, std::vector<param_def> &Params, std::string &Error)
{
	return interface_port(Node, true, Ports, Params, Error);
}

bool resolve_output(node_desc const &Node, std::vector<port_def> &Ports, std::vector<param_def> &Params, std::string &Error)
{
	return interface_port(Node, false, Ports, Params, Error);
}

bool resolve_module(node_desc const &, std::vector<port_def> &, std::vector<param_def> &, std::string &Error)
{
	// expand_modules replaces these nodes when the file is loaded, so one left here means the module couldn't be read
	Error = "the module wasn't expanded; graphs with modules have to be loaded from a file, next to the module files";
	return false;
}

class module_input final : public block
{
public:
	explicit module_input(block_setup const &Setup) : block(Setup), m_signal(Setup.ports[0].dom == domain::signal), m_value(number(Setup, 1)) {}

	void update(frame &Frame) override
	{
		if (m_signal)
		{
			set(Frame, 0, m_value);
		}
		else
		{
			set_forward(Frame, 0, m_value);
		}
	}

private:
	bool const m_signal;
	double const m_value;
};

class module_output final : public block
{
public:
	explicit module_output(block_setup const &Setup) : block(Setup), m_signal(Setup.ports[0].dom == domain::signal), m_connected(is_connected(Setup, 0)) {}

	void update(frame &Frame) override
	{
		if (false == m_connected)
		{
			return;
		}
		m_value = m_signal ? get(Frame, 0) : forward_in(Frame, 0);
	}
	std::size_t state_count() const override
	{
		return 1;
	}
	char const *state_name(std::size_t) const override
	{
		return "value";
	}
	double state_value(std::size_t) const override
	{
		return m_value;
	}

private:
	bool const m_signal;
	bool const m_connected;
	double m_value{0.0};
};

block_type interface_type(bool const Input)
{
	block_type type;
	type.id = Input ? "module.input" : "module.output";
	type.category = "module";
	type.name = Input ? "Module input" : "Module output";
	type.description = Input ? "Input of a module: the value given to the module node in the vehicle graph. On its own it gives the default value."
	                         : "Output of a module: the value the module node gives in the vehicle graph.";
	type.origin = block_origin::primitive;
	type.dynamic_ports = true;
	type.resolve = Input ? resolve_input : resolve_output;
	type.params = interface_params(Input);
	type.create = Input ? make_block<module_input> : make_block<module_output>;
	return type;
}

} // namespace

void register_module_blocks(catalog &Catalog)
{
	Catalog.add(interface_type(true));
	Catalog.add(interface_type(false));

	block_type type;
	type.id = std::string{module_type};
	type.category = "module";
	type.name = "Module";
	type.description = "Graph of logic from another file (a module), used as one block. Its ports are the module.input and module.output blocks "
	                   "of the module; the file is given relative to the vehicle graph.";
	type.origin = block_origin::primitive;
	type.dynamic_ports = true;
	type.resolve = resolve_module;
	type.params = {text_param("file")};
	type.create = nullptr;
	Catalog.add(std::move(type));
}

} // namespace vgraph
