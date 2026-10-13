/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "vehicle/graph/graph_types.h"

#include <algorithm>
#include <array>

namespace vgraph
{

namespace
{

std::array<domain_info, static_cast<std::size_t>(domain::count_)> const domains{{
    {"signal", "value", "", "", "", false},
    {"high_voltage", "voltage", "V", "current", "A", true},
    {"low_voltage", "voltage", "V", "current", "A", true},
    {"aux_ac", "voltage", "V", "power", "W", true},
    {"drive", "demand", "", "power", "W", true},
    {"mechanical", "torque", "Nm", "rotational speed", "rad/s", true},
    {"traction", "force", "N", "velocity", "m/s", true},
    {"air", "pressure", "bar", "flow", "", true},
    {"brake_pipe", "pressure", "bar", "flow", "", true},
    {"feed_pipe", "pressure", "bar", "flow", "", true},
}};

} // namespace

domain_info const &info(domain const Domain)
{
	return domains[static_cast<std::size_t>(Domain)];
}

bool parse_domain(std::string_view const Text, domain &Domain)
{
	for (std::size_t idx = 0; idx < domains.size(); ++idx)
	{
		if (domains[idx].id == Text)
		{
			Domain = static_cast<domain>(idx);
			return true;
		}
	}
	return false;
}

std::string_view to_string(value_type const Type)
{
	switch (Type)
	{
	case value_type::boolean:
		return "bool";
	case value_type::integer:
		return "int";
	default:
		return "float";
	}
}

std::string_view to_string(param_type const Type)
{
	switch (Type)
	{
	case param_type::boolean:
		return "bool";
	case param_type::text:
		return "text";
	case param_type::choice:
		return "choice";
	default:
		return "number";
	}
}

param_value param_value::make_number(double const Value)
{
	param_value result;
	result.type = param_type::number;
	result.number = Value;
	return result;
}

param_value param_value::make_boolean(bool const Value)
{
	param_value result;
	result.type = param_type::boolean;
	result.boolean = Value;
	return result;
}

param_value param_value::make_text(std::string Value)
{
	param_value result;
	result.type = param_type::text;
	result.text = std::move(Value);
	return result;
}

param_value param_value::make_choice(std::string Value)
{
	param_value result;
	result.type = param_type::choice;
	result.text = std::move(Value);
	return result;
}

bool has_errors(std::vector<diagnostic> const &Diagnostics)
{
	return std::any_of(std::begin(Diagnostics), std::end(Diagnostics), [](diagnostic const &Diagnostic) { return Diagnostic.level == severity::error; });
}

std::string to_string(diagnostic const &Diagnostic)
{
	std::string result{Diagnostic.level == severity::error ? "error" : "warning"};
	if (false == Diagnostic.node.empty())
	{
		result += " [" + Diagnostic.node + "]";
	}
	result += ": " + Diagnostic.message;
	return result;
}

} // namespace vgraph
