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
#include "application/editormodeextend.h"
#include "application/editormodegradeline.h"
#include "application/editormodeinfra.h"
#include "application/editormodelay.h"
#include "application/editormodelaytrace.h"
#include "application/editormodenmt.h"
#include "application/editormodeprofile.h"
#include "application/editormoderoad.h"
#include "application/editormoderoute.h"
#include "application/editormodestraights.h"
#include "application/editormodesweep.h"

// the vertical profile of a route: the grade lines kept, the editor of the design, its window, and the grade lines shared by the tracks
struct profile_tools
{
	profile_tools(editor_services &Services, editor_build_services &Build) : library{Build, profile_edit}, profile_edit{Services, Build, library, grade_lines}, profile_window{Services, Build, library, profile_edit, grade_lines}, grade_lines{Services, Build, library} {}
	profile_library library;
	profile_editor profile_edit;
	profile_panel profile_window;
	grade_line_tool grade_lines;
};

// the terrain model: built from the elevation data, its window, the area downloaded, and the tracks fitted to it
struct terrain_tools
{
	terrain_tools(editor_services &Services, editor_build_services &Build, profile_tools &Profiles)
	    : nmt_build{Services, Build, nmt_window, download_area, nmt_auto}, nmt_window{Services, Build, nmt_build, nmt_auto, Profiles.library, Profiles.profile_edit}, download_area{Services, Build, nmt_build},
	      nmt_auto{Services, Build, nmt_build, Profiles.grade_lines, Profiles.library, nmt_window, Profiles.profile_edit}
	{
	}
	nmt_builder nmt_build;
	nmt_panel nmt_window;
	download_area_tool download_area;
	nmt_auto_fit nmt_auto;
};

// the line designed and its digitization, drawn and in its window, and the straights of the paths
struct route_tools
{
	route_tools(editor_services &Services, editor_build_services &Build, build_shared &Shared, profile_tools &Profiles)
	    : designer{Services, Build, Shared, route_relay}, route_relay{Services, Build, designer, Profiles.library}, route_view{Services, Build, Shared, designer}, route_panel{Services, Build, designer, route_relay, route_view},
	      straight_edit{Services, Build, Shared, designer, route_relay}
	{
	}
	route_designer designer;
	route_relay_tool route_relay;
	route_overlay route_view;
	route_window route_panel;
	straights_editor straight_edit;
};

// the track laid through the points clicked: the line traced, worked out, driven by the clicks and the panel, and a free end lengthened
struct lay_tools
{
	lay_tools(editor_services &Services, editor_build_services &Build, build_shared &Shared, terrain_tools &Terrain, route_tools &Routes)
	    : tracer{Build, Shared}, planner{Services, Build, Shared, extender, tracer}, lay_input{Services, Build, planner, extender, Terrain.nmt_auto, Terrain.nmt_window, Routes.straight_edit},
	      extender{Services, Build, planner, Terrain.nmt_auto, Routes.straight_edit}
	{
	}
	lay_tracer tracer;
	lay_planner planner;
	lay_controls lay_input;
	extend_editor extender;
};

// the build tools: laying track, routes, straights, roads, sweeps, vertical profiles and grade lines, the terrain model, connections and the infrastructure; owned by editor_mode, made by make_build_tools()
struct build_tools
{
	build_tools(editor_services &Services, editor_build_services &Build)
	    : road{Build}, sweep{Services, Build, bend_state}, bend{Services, Build, bend_state, sweep}, profiles{Services, Build}, terrain{Services, Build, profiles}, routes{Services, Build, shared, profiles},
	      laying{Services, Build, shared, terrain, routes}, binder{Services, Build, shared}, infra_view{Services, Build, binder}
	{
	}
	// what the tools share among themselves
	build_shared shared;
	road_editor road;
	// the model bent along the track, edited by the bend tool and set by the sweep tool once it lays one
	bend_tool bend_state;
	sweep_editor sweep;
	bend_editor bend;
	profile_tools profiles;
	terrain_tools terrain;
	route_tools routes;
	lay_tools laying;
	infra_binder binder;
	infra_panel infra_view;
};
