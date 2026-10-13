/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/graph/graph_block.h"

#include <string>
#include <string_view>
#include <vector>

namespace vgraph
{

// where the behaviour of a block comes from; shown by the editor
enum class block_origin : std::uint8_t
{
	primitive, // generic building block of the graph
	cab, // cab control or indicator
	simulator, // wraps existing simulator code without changing it
	extracted, // simulator code extracted from Mover, shared with the .fiz vehicles
	native, // part of the simulator the graph connects to (vehicle body, couplers)
	interface_profile, // standard interface between vehicles, or between the vehicle and the AI
	script // lua function block
};

std::string_view to_string(block_origin Origin);

struct block_type
{
	std::string id; // e.g. "logic.and"; stored in the .vhg files, never change it once released
	std::string category;
	std::string name;
	std::string description;
	block_origin origin{block_origin::primitive};
	// outputs of a deferred block are published after the whole step, so connections leaving it don't take part
	// in ordering the blocks, and loops passing through it are allowed
	bool deferred{false};
	std::vector<port_def> ports;
	std::vector<param_def> params;
	block_factory create{nullptr};

	int port_index(std::string_view Key) const;
	int param_index(std::string_view Key) const;
};

// registry of all block types known to the simulator. it's also exported for the external editor
class catalog
{
public:
	static catalog const &get();

	block_type const *find(std::string_view Id) const;
	std::vector<block_type> const &types() const
	{
		return m_types;
	}
	// description of all block types and domains, for the editor
	std::string export_json() const;

	// used by the block registration functions
	void add(block_type Type);

private:
	catalog() = default;

	std::vector<block_type> m_types;
};

// helpers for the definitions of block types
port_def signal_in(std::string Key, value_type Type, double Default = 0.0, std::string Unit = "", bool Optional = false);
port_def signal_out(std::string Key, value_type Type, std::string Unit = "");
port_def physical_in(std::string Key, domain Domain, bool Optional = false);
port_def physical_out(std::string Key, domain Domain);
param_def number_param(std::string Key, double Default, std::string Unit = "");
param_def boolean_param(std::string Key, bool Default);
param_def text_param(std::string Key, std::string Default = "");
param_def choice_param(std::string Key, std::vector<std::string> Choices, std::string Default);

// registration functions of the block families, called once when the catalog is built
void register_logic_blocks(catalog &Catalog);
void register_cab_blocks(catalog &Catalog);

} // namespace vgraph
