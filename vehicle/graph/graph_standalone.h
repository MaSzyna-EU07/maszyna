/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>

namespace vgraph
{

// -graph-catalog [<file.json>]: writes the description of all block types, for the external editor.
// returns: 0 on success, 1 otherwise
int graph_catalog_standalone(std::string const &Output);
// -graph-run <file.vhg> [options]: checks the graph and runs it without the simulator, driven by a script of events.
// returns: 0 on success, 1 otherwise
int graph_run_standalone(int Argc, char *Argv[]);

} // namespace vgraph
