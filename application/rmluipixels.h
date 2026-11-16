/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <span>

// the colours of RmlUi turned into those imgui draws with. RmlUi gives its colours and the pixels it makes (the glyphs of the
// fonts) with the alpha premultiplied, imgui blends straight ones

namespace rmlui
{

// a colour with premultiplied alpha as a straight one, packed as imgui packs its colours (IM_COL32: red in the lowest byte)
std::uint32_t straight_colour(std::uint8_t Red, std::uint8_t Green, std::uint8_t Blue, std::uint8_t Alpha);

// rgba pixels with premultiplied alpha made straight, in place. a size which isn't a multiple of 4 leaves the bytes over alone
void unpremultiply(std::span<std::uint8_t> Rgba);

} // namespace rmlui
