/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "editor/editorAlignment.hpp"
#include "scene/scenenode.h"

#include <set>
#include <utility>

// what the build tools share among themselves and no other part of the editor uses; owned by build_tools
struct build_shared
{
	alignment::design route_drag_from; // the design of the line as the drag of its gizmo began, taken back when the drag is cancelled
	std::set<std::pair<scene::basic_node const *, scene::instance_handle>> infra_carried; // the objects their tracks moved since the tool cleared it
};
