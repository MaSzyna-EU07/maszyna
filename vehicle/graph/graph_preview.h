/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// live preview of the vehicle graphs for the external editor: a TCP server on the local machine, speaking json lines
// (protocol 1, see tools/vehicle_graph/README.md). off unless enabled with -graph-preview [port] or with
// "vehiclegraph_preview yes [port]" in eu07.ini. everything runs on the simulation thread: the connections are served
// by polling, once per frame
namespace vgraph
{

// serves the preview connections; does nothing when the preview is disabled
void preview_update();

} // namespace vgraph
