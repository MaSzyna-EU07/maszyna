/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "imgui/imgui.h"

// icons of the scenery editor. they're drawn by editor/icons/editor_icons.py into editor_icons.png, which CMake writes out
// as the bytes of an array built into the executable, so they're there without any file of the simulator
namespace editor_icons
{

// in the order of the cells of editor_icons.png
enum class icon
{
	select,
	insert,
	brush,
	area_fill,
	copy_to_bank,
	translate,
	rotate,
	scale,
	local_space,
	ortho,
	perspective,
	sculpt,
	smooth,
	chunks,
	orthophoto,
	road,
	place,
	lanes,
	lay_track,
	turnout,
	straight,
	curve,
	signal,
	objects,
	vehicle,
	profile,
	speed,
	joints,
	infra,
	gauge,
	turntable,
	// the fields of work
	work_surroundings,
	work_tracks,
	work_roads,
	work_terrain,
	count
};

// pixels of the side of an icon, at the scale of the user interface
float size();
// draws the icon in the colour given, centred on the point. returns: false if the renderer can't show images
bool draw(ImDrawList *List, icon const Icon, ImVec2 const &Center, ImU32 const Color);

} // namespace editor_icons
