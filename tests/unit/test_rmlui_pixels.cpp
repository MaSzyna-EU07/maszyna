#include "stdafx.h"
#include "doctest.h"
#include "application/rmluipixels.h"

#include <array>
#include <cstdint>

// the colours of RmlUi, with the alpha premultiplied, turned into the straight ones imgui draws with
namespace
{
std::uint32_t packed(std::uint32_t const Red, std::uint32_t const Green, std::uint32_t const Blue, std::uint32_t const Alpha)
{
	return Red | (Green << 8U) | (Blue << 16U) | (Alpha << 24U);
}
} // namespace

TEST_CASE("a colour of RmlUi as imgui packs it")
{
	CHECK(rmlui::straight_colour(10, 20, 30, 255) == packed(10, 20, 30, 255));
	CHECK(rmlui::straight_colour(0, 0, 0, 0) == 0U);
	// half transparent: the channels doubled, rounded to the nearest
	CHECK(rmlui::straight_colour(64, 32, 0, 128) == packed(128, 64, 0, 128));
	// a channel over its alpha can't be premultiplied, it's held at full
	CHECK(rmlui::straight_colour(200, 0, 0, 100) == packed(255, 0, 0, 100));
}

TEST_CASE("pixels of RmlUi made straight in place")
{
	std::array<std::uint8_t, 13> pixels{255, 255, 255, 255, 64, 64, 64, 128, 9, 9, 9, 0, 7};
	rmlui::unpremultiply(pixels);
	std::array<std::uint8_t, 13> const straight{255, 255, 255, 255, 128, 128, 128, 128, 0, 0, 0, 0, 7};
	CHECK(pixels == straight);
}
