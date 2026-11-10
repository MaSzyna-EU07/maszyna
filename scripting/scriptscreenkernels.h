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

// image filters the script screens call through the host (led_post, resize_lanczos), run natively rather than
// in the interpreter. they give the very pixels of the Python screens: led_util.render_post() and Pillow's
// resize(..., Image.LANCZOS). the images are RGBA, 4 bytes a pixel, row by row; alpha is ignored and written as 255
namespace script_screens
{

// the size in bytes of an RGBA image
constexpr std::size_t image_size(int const Width, int const Height)
{
	return static_cast<std::size_t>(Width) * static_cast<std::size_t>(Height) * 4;
}

// led_util's render_post: every diode becomes 2x2 pixels with a darker grid between them, glowing into its neighbours
// (Bloom), in a 1 pixel black frame. Target takes (2 Width + 2) x (2 Height + 2) pixels. returns false for sizes that
// don't fit the spans
bool led_post(std::span<std::uint8_t const> Source, int Width, int Height, std::span<std::uint8_t> Target, bool Bloom);

// Pillow's resize with the Lanczos filter, horizontally then vertically, each pass rounded to 8 bits. returns false
// for sizes that don't fit the spans
bool resize_lanczos(std::span<std::uint8_t const> Source, int Width, int Height, std::span<std::uint8_t> Target, int TargetWidth, int TargetHeight);

} // namespace script_screens
