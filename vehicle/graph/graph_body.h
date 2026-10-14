/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/graph/graph_types.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

class TMoverParameters;

namespace vgraph
{

// entry of the "body" section of the .vhg files: parameters of the vehicle body, which stays native in the simulator
// (mass, dimensions, axles, couplers). each entry names the .fiz section and key it replaces; the simulator builds
// the .fiz lines from the entries and reads them with the same code that reads the .fiz files
struct body_field
{
	std::string key;
	param_type type{param_type::number};
	std::string unit;
	param_value default_value;
	std::vector<std::string> choices; // choice fields only
	std::vector<std::string> fiz_values; // choice fields only: the .fiz value of each choice
	std::string fiz_section; // e.g. "Param."
	std::string fiz_key; // e.g. "M"
};

std::vector<body_field> const &body_schema();
body_field const *find_body_field(std::string_view Key);

// standard interfaces between vehicles, or between the vehicle and the AI, with the versions the simulator knows
struct profile_info
{
	std::string_view id;
	int version;
};

std::vector<profile_info> const &known_profiles();

// what the AI driver wants from the vehicle (if.ai outputs), and what it reads back (if.ai inputs)
struct ai_intents
{
	double dir{0.0};
	double traction{0.0}; // 0..1
	double ed_brake{0.0}; // 0..1
	double brake{0.0}; // train brake demand 0..1, 1 = full service braking
	bool emergency{false};
	double local_brake{0.0}; // 0..1
	bool prepare{false};
	bool shutdown{false};
	bool doors_left{false};
	bool doors_right{false};
	int lights{0};
	bool horn{false};
	bool pantographs{false};
	bool sand{false};
};

struct ai_feedback
{
	bool ready{false};
	double i_ratio{0.0};
	bool slip{false};
	double pg{0.0};
	double zg{0.0};
	double bc{0.0};
	double vmax{0.0};
};

// property of the vehicle body set by several blocks, e.g. the Mains closed by any of the line breakers. the first block
// updated in a step starts the value anew and the next ones combine theirs with it, so after the step it covers all of them
struct shared_value
{
	std::uint64_t step{0};
	double value{0.0};

	// the largest of the values given in the step
	double max(std::uint64_t const Step, double const Value)
	{
		value = (step != Step ? Value : std::max(value, Value));
		step = Step;
		return value;
	}
	// whether any of the values given in the step is set
	bool any(std::uint64_t const Step, bool const Value)
	{
		return max(Step, Value ? 1.0 : 0.0) > 0.5;
	}
};

// value one vehicle gives to a train line (if.line), read by the vehicles coupled with it
struct train_line_value
{
	std::string name;
	double value{0.0};
	bool active{false}; // the vehicle drives the line
};

// state of the vehicle body shared with the blocks during a step. the owner of the graph (the simulator, or the
// standalone runner) fills the inputs before every step and reads the outputs after it
struct body_state
{
	// inputs
	double velocity{0.0}; // m/s, signed, positive when moving towards the front coupler
	double acceleration{0.0}; // m/s^2, signed like the velocity
	double wheel_speed{0.0}; // rotational speed of the wheels, rev/s, signed like the velocity
	bool slipping{false};
	int cab{0}; // occupied cab: 1, -1 or 0
	double distance{0.0}; // m, travelled
	bool standalone{false}; // the graph runs without the simulator (-graph-run): no catenary, no neighbours
	// set when the graph has no br.pipe or pn.reservoir: the pipe of the body still passes the air between the neighbours
	bool native_brake_pipe{true};
	bool native_feed_pipe{true};
	// outputs, cleared by the owner before every step
	double tractive_force{0.0}; // N, at the wheel rim, sum of drive.axle blocks; positive towards the front
	double brake_force{0.0}; // N, magnitude, sum of br.rigging blocks
	double current{0.0}; // A, drawn from the catenary by the vehicle
	bool headlights_dimmed{false};
	// AI interface (if.ai)
	ai_intents ai;
	ai_feedback ai_reply;
	// properties of the body combined from several blocks
	shared_value mains; // closed line breakers (el.hsb)
	shared_value converter; // running converters (el.converter)
	shared_value converter_allow;
	// train lines of the vehicle, published by its if.line blocks for the neighbours
	std::vector<train_line_value> lines;
	// body of the vehicle; the standalone runner (-graph-run) creates one as well, so it is never null during update
	TMoverParameters *mover{nullptr};
};

} // namespace vgraph
