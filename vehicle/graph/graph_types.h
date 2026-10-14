/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

// vehicle graph: block based description of vehicle logic and physics, loaded from .vhg files
// in place of .fiz. the core of the graph depends only on the standard library and nlohmann json
// (the latter only in the .cpp files), so it can be built and tested without the rest of the simulator
namespace vgraph
{

// format identification of the .vhg (vehicle graph) files
inline constexpr std::string_view file_format{"maszyna-vehicle-graph"};
inline constexpr int file_version{2};
inline constexpr std::string_view file_extension{".vhg"};

// signal ports pass one value in one direction. physical ports pass a pair of values in opposite directions,
// e.g. voltage from the source towards the load and current from the load back towards the source
enum class domain : std::uint8_t
{
	signal,
	high_voltage, // U [V] forward, I [A] back
	low_voltage, // U [V] forward, I [A] back; only simulated when the graph enables low voltage simulation
	aux_ac, // U [V] forward, P [W] back
	drive, // demand [0-1] forward, P [W] back
	mechanical, // torque [Nm] forward, rotational speed [rad/s] back
	traction, // force at the wheel rim [N] forward, velocity [m/s] back
	air, // pressure [bar] forward, flow back
	brake_pipe, // pressure [bar] forward, flow back
	feed_pipe, // pressure [bar] forward, flow back
	count_
};

enum class value_type : std::uint8_t
{
	boolean,
	integer,
	real
};

enum class port_direction : std::uint8_t
{
	in,
	out
};

struct domain_info
{
	std::string_view id;
	std::string_view forward; // quantity passed from the output towards the inputs
	std::string_view forward_unit;
	std::string_view back; // quantity passed from the inputs back towards the output
	std::string_view back_unit;
	bool physical;
};

domain_info const &info(domain Domain);
bool parse_domain(std::string_view Text, domain &Domain);
std::string_view to_string(value_type Type);

enum class param_type : std::uint8_t
{
	number,
	boolean,
	text,
	choice,
	table // rows of numbers, e.g. the resistor table of a series motor
};

std::string_view to_string(param_type Type);

// value of a block parameter, as read from the file or taken from the catalog default
struct param_value
{
	param_type type{param_type::number};
	double number{0.0};
	bool boolean{false};
	std::string text; // also holds the choice id
	std::vector<std::vector<double>> rows; // table parameters only

	static param_value make_number(double Value);
	static param_value make_boolean(bool Value);
	static param_value make_text(std::string Value);
	static param_value make_choice(std::string Value);
	static param_value make_table(std::vector<std::vector<double>> Rows);
};

struct port_def
{
	std::string key;
	port_direction direction{port_direction::in};
	domain dom{domain::signal};
	value_type type{value_type::real}; // signal ports only
	std::string unit;
	bool optional{false}; // inputs only; unconnected required inputs are reported as warnings
	double default_value{0.0}; // signal inputs only; value seen by the block when the input isn't connected
};

struct param_def
{
	std::string key;
	param_type type{param_type::number};
	std::string unit;
	param_value default_value;
	std::vector<std::string> choices; // choice parameters only, stable ascii ids
	std::vector<std::string> columns; // table parameters only: names of the columns, empty for rows of free width
	// .fiz entry the parameter stands for, if any, e.g. "Brake:" and "MaxBP". the simulator writes such parameters
	// to the vehicle body (TMoverParameters) with the code reading the .fiz files
	std::string fiz_section;
	std::string fiz_key;
};

enum class severity : std::uint8_t
{
	warning,
	error
};

// problem found while loading or compiling a graph. the node is empty for problems of the whole file
struct diagnostic
{
	severity level{severity::error};
	std::string node;
	std::string message;
};

// index of the port or parameter with the specified key, or -1
int find_port(std::vector<port_def> const &Ports, std::string_view Key);
int find_param(std::vector<param_def> const &Params, std::string_view Key);

bool has_errors(std::vector<diagnostic> const &Diagnostics);
std::string to_string(diagnostic const &Diagnostic);

} // namespace vgraph
