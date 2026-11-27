/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

// the drawing the Lua screens get from the game (eu07.image, eu07.font, eu07.sprite): bitmap fonts and sprites in the
// format of the python LED screens (scripts/led_util.py: a .png with the glyphs side by side and a .txt with their
// widths), and RGBA images to draw them on. drawn natively, the script only says what goes where

#include <array>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace script_screens
{

using rgb = std::array<std::uint8_t, 3>;

// the code points of UTF-8 text; a byte that doesn't start a valid sequence stands for itself
std::u32string decode_utf8(std::string_view Text);

// a character or a sprite: its lit pixels, column by column
class glyph
{
  public:
	glyph() = default;
	glyph(int Width, int Height, std::vector<bool> Lit) : m_width{Width}, m_height{Height}, m_lit{std::move(Lit)} {}

	int width() const
	{
		return m_width;
	}
	int height() const
	{
		return m_height;
	}
	bool lit(int const X, int const Y) const
	{
		return m_lit[static_cast<std::size_t>(X) * m_height + Y];
	}

  private:
	int m_width{0};
	int m_height{0};
	std::vector<bool> m_lit;
};

// what a font may be: an image no larger than a screen image, a glyph no wider; no more glyphs, nor columns of them
// side by side, than a font can use. a glyph past the image's edge is unlit there, as with led_util
constexpr int max_font_image_side{4096};
constexpr int max_font_glyphs{4096};
constexpr int max_font_columns{2 * max_font_image_side};

class bitmap_font
{
  public:
	// a font as led_util.load_font() reads it: Rgba (Width x Height) has the glyphs side by side, a column apart;
	// Description has a comment line, then a line per glyph: the character and its width. a pixel is lit unless it is
	// opaque white. a glyph wider than max_font_image_side is skipped. nothing for a description without glyphs, or
	// beyond the limits above
	static std::optional<bitmap_font> parse(std::span<std::uint8_t const> Rgba, int Width, int Height, std::string_view Description);
	// a sprite as led_util.load_spr() reads it: the whole image as one glyph
	static glyph sprite(std::span<std::uint8_t const> Rgba, int Width, int Height);

	int height() const
	{
		return m_height;
	}
	// the glyph of the character. a character the font lacks gets the font's first glyph (led_util picks a random one;
	// here every refresh draws the same)
	glyph const &find(char32_t Character) const;
	// the width of the text, a column of Spacing after every character (led_util's get_str_len)
	int text_width(std::string_view Text, int Spacing) const;

  private:
	int m_height{0};
	std::unordered_map<char32_t, glyph> m_glyphs;
	glyph m_fallback;
};

// an RGBA image, from the top row down
class canvas
{
  public:
	canvas(int Width, int Height, rgb Color);
	canvas(int Width, int Height, std::vector<std::uint8_t> Pixels);

	int width() const
	{
		return m_width;
	}
	int height() const
	{
		return m_height;
	}
	std::vector<std::uint8_t> const &pixels() const
	{
		return m_pixels;
	}

	void fill(rgb Color);
	// a pixel outside the image is skipped
	void set(int X, int Y, rgb Color);
	// black outside the image
	rgb get(int X, int Y) const;
	// clipped to the image
	void fill_rect(int X, int Y, int Width, int Height, rgb Color);
	// the lit pixels of the glyph, its top left corner at (X, Y)
	void draw_glyph(glyph const &Glyph, int X, int Y, rgb Color);
	// text from X rightwards, Spacing columns between the characters (led_util's draw_str). returns the X after it
	int draw_text(bitmap_font const &Font, int X, int Y, std::string_view Text, rgb Color, int Spacing);
	// another image, its top left corner at (X, Y), clipped
	void draw_image(canvas const &Image, int X, int Y);

  private:
	int m_width;
	int m_height;
	std::vector<std::uint8_t> m_pixels;
};

} // namespace script_screens
