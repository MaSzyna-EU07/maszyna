/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "application/editormode.h"
#include "application/editormoderoad.h"

// the build tools: laying track, routes, straights, roads, sweeps, vertical profiles and grade lines, the terrain model, connections and the infrastructure; owned by editor_mode, made by make_build_tools()
struct build_tools
{
	explicit build_tools(editor_build_services &Build) : road{Build} {}
	road_editor road;
};
