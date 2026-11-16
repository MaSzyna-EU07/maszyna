/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "application/rmluipixels.h"

#include <algorithm>

namespace rmlui
{
namespace
{

// a channel premultiplied by Alpha as a straight one, rounded to the nearest
std::uint8_t straight_channel(std::uint8_t const Channel, std::uint8_t const Alpha)
{
	if (Alpha == 0)
		return 0;
	auto const straight{(static_cast<unsigned>(Channel) * 255U + Alpha / 2U) / Alpha};
	return static_cast<std::uint8_t>(std::min(straight, 255U));
}

} // namespace

std::uint32_t straight_colour(std::uint8_t const Red, std::uint8_t const Green, std::uint8_t const Blue, std::uint8_t const Alpha)
{
	return static_cast<std::uint32_t>(straight_channel(Red, Alpha))
	       | (static_cast<std::uint32_t>(straight_channel(Green, Alpha)) << 8U)
	       | (static_cast<std::uint32_t>(straight_channel(Blue, Alpha)) << 16U)
	       | (static_cast<std::uint32_t>(Alpha) << 24U);
}

void unpremultiply(std::span<std::uint8_t> const Rgba)
{
	for (std::size_t pixel{0}; pixel + 4 <= Rgba.size(); pixel += 4)
	{
		auto const alpha{Rgba[pixel + 3]};
		Rgba[pixel + 0] = straight_channel(Rgba[pixel + 0], alpha);
		Rgba[pixel + 1] = straight_channel(Rgba[pixel + 1], alpha);
		Rgba[pixel + 2] = straight_channel(Rgba[pixel + 2], alpha);
	}
}

} // namespace rmlui
