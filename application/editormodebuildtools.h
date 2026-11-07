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
#include "application/editormodegradeline.h"
#include "application/editormodeprofile.h"
#include "application/editormoderoad.h"
#include "application/editormodesweep.h"

// the build tools: laying track, routes, straights, roads, sweeps, vertical profiles and grade lines, the terrain model, connections and the infrastructure; owned by editor_mode, made by make_build_tools()
struct build_tools
{
	build_tools(editor_services &Services, editor_build_services &Build) : road{Build}, sweep{Services, Build, bend_state}, bend{Services, Build, bend_state, sweep}, library{Build, profile_edit}, profile_edit{Services, Build, library, grade_lines}, profile_window{Services, Build, library, profile_edit, grade_lines}, grade_lines{Services, Build, library} {}
	road_editor road;
	// the model bent along the track, edited by the bend tool and set by the sweep tool once it lays one
	bend_tool bend_state;
	sweep_editor sweep;
	bend_editor bend;
	profile_library library;
	profile_editor profile_edit;
	profile_panel profile_window;
	grade_line_tool grade_lines;
};
