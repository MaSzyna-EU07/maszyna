/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "vehicle/graph/graph_format.h"

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace vgraph
{

struct block_type;
class catalog;

// graph checked against the catalog and resolved to flat arrays: node parameters in definition order, ports bound to
// signal slots or physical links, and the order of evaluation. it holds no simulation state and is shared by all
// vehicles of the same type
class graph_template
{
public:
	struct node_entry
	{
		std::string id;
		std::string title;
		block_type const *type{nullptr};
		std::vector<param_value> params; // in the order of the type's parameter definitions
		std::vector<int> slots; // in the order of the type's port definitions: signal slot, or link index for physical ports
		std::vector<std::uint8_t> connected; // in the order of the type's port definitions: whether the port has a connection
	};

	// checks the document and builds the template. problems are appended to the diagnostics; returns null on errors
	static std::shared_ptr<graph_template const> compile(document const &Document, catalog const &Catalog, std::vector<diagnostic> &Diagnostics);

	std::string const &name() const
	{
		return m_name;
	}
	bool low_voltage_simulation() const
	{
		return m_lowvoltage;
	}
	std::vector<node_entry> const &nodes() const
	{
		return m_nodes;
	}
	// node indices in the order of evaluation
	std::vector<int> const &order() const
	{
		return m_order;
	}
	// initial values of all signal slots; for unconnected inputs these are the values the blocks will see
	std::vector<double> const &signal_defaults() const
	{
		return m_signaldefaults;
	}
	std::size_t link_count() const
	{
		return m_linkcount;
	}
	std::vector<named_value> const &body() const
	{
		return m_body;
	}
	std::vector<std::pair<std::string, int>> const &profiles() const
	{
		return m_profiles;
	}

	// index of the node with the specified id, or -1
	int find_node(std::string_view Id) const;
	// signal slot of the specified port, given as "node.port", or -1 if there's no such signal port
	int find_signal(std::string_view Endpoint) const;
	// names of signal outputs in "node.port" form, with their slots, in the order of nodes and ports
	std::vector<std::pair<std::string, int>> signal_outputs() const;

private:
	std::string m_name;
	bool m_lowvoltage{false};
	std::vector<node_entry> m_nodes;
	std::vector<int> m_order;
	std::vector<double> m_signaldefaults;
	std::size_t m_linkcount{0};
	std::vector<named_value> m_body;
	std::vector<std::pair<std::string, int>> m_profiles;
};

// loads the specified .vhg file and compiles it against the simulator's catalog.
// problems are appended to the diagnostics; returns null on errors
std::shared_ptr<graph_template const> load_graph(std::string const &Path, std::vector<diagnostic> &Diagnostics);

} // namespace vgraph
