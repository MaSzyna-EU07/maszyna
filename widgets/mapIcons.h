#pragma once

#include "imgui/imgui.h"

namespace map_icons
{

enum class icon
{
	pan,
	rotate,
	vehicle,
	camera,
	zoom_in,
	zoom_out,
	count
};

bool draw(ImDrawList *List, icon Icon, ImVec2 const &Center, ImU32 Color);

} // namespace map_icons
