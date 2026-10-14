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

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace vgraph
{

using named_value = std::pair<std::string, param_value>;

struct node_desc
{
	std::string id;
	std::string type;
	std::string title;
	std::vector<named_value> params; // as stored in the file; checked against the catalog when the graph is compiled
	std::string script; // source of a lua block ("lua" in the file)
};

struct connection_desc
{
	std::string from_node;
	std::string from_port;
	std::string to_node;
	std::string to_port;
};

// content of a .vhg file, before it's checked against the block catalog
struct document
{
	std::string name;
	// when enabled, blocks with low voltage ports simulate the voltages and currents of the control circuits.
	// when disabled, low voltage connections are ignored and the apparatus behaves as if always powered
	bool low_voltage_simulation{false};
	std::vector<named_value> body; // parameters of the vehicle body (mass, dimensions, axles...); used by the simulator integration
	std::vector<std::pair<std::string, int>> profiles; // standard interfaces used by the vehicle, with their versions
	std::vector<node_desc> nodes;
	std::vector<connection_desc> connections;
};

// type of the nodes standing for modules: graphs of logic kept in their own files (by convention .vhgm) and used as
// one block. the "file" parameter names the module file, relative to the file of the graph using it
inline constexpr std::string_view module_type{"module"};

// replaces the module nodes of the document with the blocks of their modules, read from their files relative to
// Directory (with a trailing separator, or empty). the blocks get ids "<module node id>/<id in the module>", and the
// connections of the module node are passed to the blocks connected to its module.input and module.output blocks.
// modules can use other modules. problems are appended to the diagnostics; returns false on errors
bool expand_modules(document &Document, std::string const &Directory, std::vector<diagnostic> &Diagnostics);

// parses the content of a .vhg file. problems are appended to the diagnostics; returns false on errors
bool parse_document(std::string const &Text, document &Document, std::vector<diagnostic> &Diagnostics);
// reads and parses the specified .vhg file
bool load_document(std::string const &Path, document &Document, std::vector<diagnostic> &Diagnostics);

} // namespace vgraph
