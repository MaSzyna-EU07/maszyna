/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <map>
#include <span>
#include <string>
#include <string_view>

// where the parts of the hud of the driver stand, as the player places them (the menu of the pause, Edit HUD): each part is
// held at the edges of the screen nearer to it, so it keeps its corner when the size of the screen changes, and a part dragged
// close to an edge of the screen or of another part snaps to it. kept in hud_layout.ini of the profile of the user

namespace hud_layout
{

struct rect
{
	float left{0.f};
	float top{0.f};
	float width{0.f};
	float height{0.f};
};

struct size
{
	float width{0.f};
	float height{0.f};
};

// the place of a part, from the edges it's held at
struct anchor
{
	bool right{false}; // from the right edge of the screen, else from the left one
	float x{0.f}; // the distance from that edge to the side of the part facing it
	bool bottom{false}; // from the bottom edge, else from the top one
	float y{0.f};
};

using layout = std::map<std::string, anchor, std::less<>>;

// the place of Part, held at the edges of Screen nearer to it
anchor anchor_of(rect const &Part, size const &Screen);
// the top left corner of a part of Part size placed by Anchor on Screen
rect place(anchor const &Anchor, size const &Part, size const &Screen);
// Part moved where it snaps: each of its edges within Range of an edge of Screen or of the Others (to the side of one, or in
// line with it) is moved onto it, the nearest edge in each direction
rect snap(rect const &Part, std::span<rect const> Others, size const &Screen, float Range);

// the places in the text of hud_layout.ini: lines of the name of a part, left or right and a distance, top or bottom and a
// distance; anything else in it is passed over
layout parse(std::string_view Text);
std::string write(layout const &Layout);

} // namespace hud_layout
