/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/hudlayout.h"

#include <algorithm>
#include <cmath>
#include <format>
#include <optional>
#include <sstream>
#include <vector>

namespace hud_layout
{
namespace
{

// how far an edge from Start to Start + Length is to move to lie on the nearest of Targets, none farther than Range
std::optional<float> snap_offset(float const Start, float const Length, std::span<float const> Targets, float const Range)
{
	std::optional<float> best;
	for (auto const target : Targets)
	{
		for (auto const offset : {target - Start, target - (Start + Length)})
		{
			if (std::abs(offset) <= Range && (false == best.has_value() || std::abs(offset) < std::abs(*best)))
				best = offset;
		}
	}
	return best;
}

// the edges a part can snap to along one axis: those of the screen, and both of each of the other parts
std::vector<float> edges(float const Screen, std::span<rect const> Others, bool const Horizontal)
{
	std::vector<float> targets{0.f, Screen};
	for (auto const &other : Others)
	{
		auto const start{Horizontal ? other.left : other.top};
		targets.push_back(start);
		targets.push_back(start + (Horizontal ? other.width : other.height));
	}
	return targets;
}

// within the screen as far as it fits
float held_in(float const Start, float const Length, float const Screen)
{
	return std::max(0.f, std::min(Start, Screen - Length));
}

// a side of a line of hud_layout.ini: left or right, top or bottom, and the distance from it
bool read_side(std::istringstream &Line, std::string_view const Start, std::string_view const End, bool &Far, float &Distance)
{
	std::string side;
	if (false == static_cast<bool>(Line >> side >> Distance))
		return false;
	Far = (side == End);
	return side == Start || side == End;
}

} // namespace

anchor anchor_of(rect const &Part, size const &Screen)
{
	anchor result;
	result.right = Part.left + Part.width * 0.5f > Screen.width * 0.5f;
	result.x = result.right ? Screen.width - (Part.left + Part.width) : Part.left;
	result.bottom = Part.top + Part.height * 0.5f > Screen.height * 0.5f;
	result.y = result.bottom ? Screen.height - (Part.top + Part.height) : Part.top;
	return result;
}

rect place(anchor const &Anchor, size const &Part, size const &Screen)
{
	rect result;
	result.width = Part.width;
	result.height = Part.height;
	result.left = Anchor.right ? Screen.width - Anchor.x - Part.width : Anchor.x;
	result.top = Anchor.bottom ? Screen.height - Anchor.y - Part.height : Anchor.y;
	return result;
}

rect snap(rect const &Part, std::span<rect const> Others, size const &Screen, float const Range)
{
	auto result{Part};
	auto const horizontal{edges(Screen.width, Others, true)};
	auto const vertical{edges(Screen.height, Others, false)};
	result.left += snap_offset(Part.left, Part.width, horizontal, Range).value_or(0.f);
	result.top += snap_offset(Part.top, Part.height, vertical, Range).value_or(0.f);
	result.left = held_in(result.left, result.width, Screen.width);
	result.top = held_in(result.top, result.height, Screen.height);
	return result;
}

layout parse(std::string_view const Text)
{
	layout result;
	std::istringstream lines{std::string{Text}};
	std::string text;
	while (std::getline(lines, text))
	{
		std::istringstream line{text};
		std::string name;
		if (false == static_cast<bool>(line >> name) || name.starts_with("//"))
			continue;
		anchor position;
		if (read_side(line, "left", "right", position.right, position.x) && read_side(line, "top", "bottom", position.bottom, position.y))
			result.insert_or_assign(name, position);
	}
	return result;
}

std::string write(layout const &Layout)
{
	std::string text{"// the places of the parts of the hud of the driver, as the menu of the pause (Edit HUD) left them:\n"
	                 "// the name of a part, the edges of the screen it's held at and its distances from them in dp\n"};
	for (auto const &[name, position] : Layout)
		text += std::format("{} {} {:.0f} {} {:.0f}\n", name, position.right ? "right" : "left", position.x, position.bottom ? "bottom" : "top", position.y);
	return text;
}

} // namespace hud_layout
