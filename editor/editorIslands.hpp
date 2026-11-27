/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <glm/glm.hpp>

// the layout of the islands of the light and the dark palettes of the editor: where each window floats over the 3d view.
// worked out in one place from the size of the main window and what is shown, without imgui, so it can be tested
namespace editor_islands
{

// a part of the main window, in pixels; max is never left or above min
struct rect
{
	glm::vec2 min{0.0f};
	glm::vec2 max{0.0f};

	glm::vec2 size() const
	{
		return max - min;
	}
};

// what the layout is worked out from
struct input
{
	glm::vec2 work_min{0.0f}; // the part of the main window under the menu
	glm::vec2 work_max{0.0f};
	float scale{1.0f}; // of the user interface
	float capsule_width{0.0f}; // the capsule as wide as it was in the frame before, 0 before it was drawn
	float tool_content{0.0f}; // the content of the tool window as high as it was in the frame before, 0 before it was drawn
	bool tray{false}; // the node bank is shown in the tray at the bottom
	bool sets{false}; // the model sets share the tray with it
};

struct layout
{
	rect capsule; // as wide as it was drawn, the window sizes itself by its content
	rect rail; // its position and width, the window is as high as its content
	rect tool;
	rect inspector;
	rect drawer;
	rect bank;
	rect sets;
	rect view; // the free part of the 3d view between the islands, where its overlays go
	bool column_below_capsule{false}; // the capsule is too wide for the column at its side
	bool tray_full_width{false}; // the tray is too narrow between the rail and the column, it goes under the column
};

layout compute(input const &In);

} // namespace editor_islands
