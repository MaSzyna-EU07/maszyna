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
#include "application/editormodeswitchdesign.h"

// the rail tools: the track tab and its panels, the gizmo, track properties and bed, switches, signals, the turntable and the lineside; owned by editor_mode, made by make_rail_tools()
struct rail_tools
{
	rail_tools(editor_services &Services, editor_rail_services &Rail);
	// the tools of the track tab which work on their own: the check of the speeds, the signalling of a station, the kilometrage
	speed_tool speed;
	srk_tool srk;
	km_tool km;
	// the window of the switch templates
	switch_designer_tool switch_designer;
};
