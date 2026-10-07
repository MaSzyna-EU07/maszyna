/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware/protocol/protocol_defs.h"
#include "input/command.h"

// command namespace of the hardware api
// the list is generated from simulation::Commands_descriptions, with hardware metadata added on top,
// so that external devices never depend on the numeric values of the user_command enumeration

namespace hardware
{

// optional conversion applied to the value received from the device before the command is posted
enum class command_transform : std::uint8_t
{
	none = 0,
	// 0...1 fraction of the master controller range of the occupied vehicle
	mastercontroller_percent = 1
};

struct command_descriptor
{
	// registry name without namespace prefix, i.e. "trainbrakeset" rather than "command.trainbrakeset"
	std::string name;
	user_command command = user_command::none;
	// true for controls which may be driven with CONTROL_SET, false for event-only commands
	bool continuous = false;
	value_type parameter_type = value_type::float32;
	std::optional<double> minimum;
	std::optional<double> maximum;
	command_transform transform = command_transform::none;
};

class command_registry
{

public:
// methods
	static command_registry const &instance();

	std::vector<command_descriptor> const &entries() const { return m_entries; }
	command_descriptor const *find( std::string const &Name ) const;

private:
// methods
	command_registry();

// members
	std::vector<command_descriptor> m_entries;
	std::unordered_map<std::string, std::size_t> m_index;
};

} // namespace hardware
