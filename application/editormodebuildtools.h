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

// the build tools: laying track, routes, straights, roads, sweeps, vertical profiles and grade lines, the terrain model, connections and the infrastructure; owned by editor_mode, made by make_build_tools()
struct build_tools
{
	build_tools(editor_services &Services, editor_build_services &Build) : road{Build}, sweep{Services, Build, bend_state}, bend{Services, Build, bend_state, sweep}, library{Build, profile_edit}, profile_edit{Services, Build, library, grade_lines}, profile_window{Services, Build, library, profile_edit, grade_lines}, grade_lines{Services, Build, library}, nmt_build{Services, Build, nmt_window, download_area, nmt_auto}, nmt_window{Services, Build, nmt_build, nmt_auto, library, profile_edit}, download_area{Services, Build, nmt_build}, nmt_auto{Services, Build, nmt_build, grade_lines, library, nmt_window, profile_edit}, designer{Services, Build, shared, route_relay}, route_relay{Services, Build, designer, library}, route_view{Services, Build, shared, designer}, route_panel{Services, Build, designer, route_relay, route_view}, straight_edit{Services, Build, shared, designer, route_relay}, tracer{Build, shared}, planner{Services, Build, shared, extender, tracer}, lay_input{Services, Build, planner, extender, nmt_auto, nmt_window, straight_edit}, extender{Services, Build, planner, nmt_auto, straight_edit}, binder{Services, Build, shared}, infra_view{Services, Build, binder} {}
	// what the tools share among themselves
	build_shared shared;
	road_editor road;
	// the model bent along the track, edited by the bend tool and set by the sweep tool once it lays one
	bend_tool bend_state;
	sweep_editor sweep;
	bend_editor bend;
	profile_library library;
	profile_editor profile_edit;
	profile_panel profile_window;
	grade_line_tool grade_lines;
	nmt_builder nmt_build;
	nmt_panel nmt_window;
	download_area_tool download_area;
	nmt_auto_fit nmt_auto;
	route_designer designer;
	route_relay_tool route_relay;
	route_overlay route_view;
	route_window route_panel;
	straights_editor straight_edit;
	lay_tracer tracer;
	lay_planner planner;
	lay_controls lay_input;
	extend_editor extender;
	infra_binder binder;
	infra_panel infra_view;
};
