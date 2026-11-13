/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scripting/scriptscreendrawing.h"

#include <algorithm>
#include <charconv>

namespace script_screens
{

namespace
{

struct sequence
{
	std::size_t length{1};
	char32_t first_bits{0};
};

// the length of the UTF-8 sequence starting with the byte and the bits it carries; length 0 for a byte that can't
// start one
sequence utf8_sequence(std::uint8_t const Lead)
{
	if (Lead < 0x80)
	{
		return {1, Lead};
	}
	if (Lead >= 0xC2 && Lead <= 0xDF)
	{
		return {2, static_cast<char32_t>(Lead & 0x1F)};
	}
	if (Lead >= 0xE0 && Lead <= 0xEF)
	{
		return {3, static_cast<char32_t>(Lead & 0x0F)};
	}
	if (Lead >= 0xF0 && Lead <= 0xF4)
	{
		return {4, static_cast<char32_t>(Lead & 0x07)};
	}
	return {0, 0};
}

bool continuation(std::uint8_t const Byte)
{
	return (Byte & 0xC0) == 0x80;
}

// a pixel is lit unless it is opaque white, as led_util compares it with (255, 255, 255) and (255, 255, 255, 255)
bool lit_pixel(std::span<std::uint8_t const> const Rgba, int const Width, int const X, int const Y)
{
	auto const at{(static_cast<std::size_t>(Y) * Width + X) * 4};
	return false == (Rgba[at] == 255 && Rgba[at + 1] == 255 && Rgba[at + 2] == 255 && Rgba[at + 3] == 255);
}

// the columns [First, First + Count) of the image as a glyph; columns past the image's edge are unlit
glyph cut_glyph(std::span<std::uint8_t const> const Rgba, int const Width, int const Height, int const First, int const Count)
{
	std::vector<bool> lit(static_cast<std::size_t>(Count) * Height, false);
	// only the columns inside the image are read
	auto const last{static_cast<int>(std::min<std::int64_t>(static_cast<std::int64_t>(First) + Count, Width))};
	for (auto x = std::max(First, 0); x < last; ++x)
	{
		auto const column{x - First};
		for (auto y = 0; y < Height; ++y)
		{
			lit[static_cast<std::size_t>(column) * Height + y] = lit_pixel(Rgba, Width, x, y);
		}
	}
	return {Count, Height, std::move(lit)};
}

// the lines of the text, without the line ends (\n or \r\n, as python's text files read them)
std::vector<std::string_view> lines(std::string_view Text)
{
	std::vector<std::string_view> result;
	while (false == Text.empty())
	{
		auto const end{std::min(Text.find('\n'), Text.size())};
		auto line{Text.substr(0, end)};
		if (false == line.empty() && line.back() == '\r')
		{
			line.remove_suffix(1);
		}
		result.push_back(line);
		Text.remove_prefix(std::min(end + 1, Text.size()));
	}
	return result;
}

} // namespace

std::u32string decode_utf8(std::string_view const Text)
{
	std::u32string result;
	std::size_t idx{0};
	while (idx < Text.size())
	{
		auto const lead{static_cast<std::uint8_t>(Text[idx])};
		auto [length, code]{utf8_sequence(lead)};
		auto valid{length > 0 && idx + length <= Text.size()};
		for (std::size_t offset = 1; valid && offset < length; ++offset)
		{
			auto const byte{static_cast<std::uint8_t>(Text[idx + offset])};
			valid = continuation(byte);
			code = (code << 6) | (byte & 0x3F);
		}
		if (false == valid)
		{
			result.push_back(lead);
			++idx;
			continue;
		}
		result.push_back(code);
		idx += length;
	}
	return result;
}

std::optional<bitmap_font> bitmap_font::parse(std::span<std::uint8_t const> const Rgba, int const Width, int const Height, std::string_view const Description)
{
	if (Width <= 0 || Height <= 0 || Width > max_font_image_side || Height > max_font_image_side || Rgba.size() < static_cast<std::size_t>(Width) * Height * 4)
	{
		return std::nullopt;
	}
	bitmap_font font;
	font.m_height = Height;
	auto const all{lines(Description)};
	auto x{0}; // where the glyph starts: at most max_font_columns
	auto glyphs{0};
	// the first line is a comment
	for (std::size_t line = 1; line < all.size(); ++line)
	{
		auto const characters{decode_utf8(all[line])};
		if (characters.empty())
		{
			continue;
		}
		// the character, then its width in digits
		auto const lead{utf8_sequence(static_cast<std::uint8_t>(all[line][0])).length};
		auto digits{all[line].substr(std::max<std::size_t>(lead, 1))};
		// int() of python lets spaces around the number through
		while (false == digits.empty() && (digits.front() == ' ' || digits.front() == '	'))
		{
			digits.remove_prefix(1);
		}
		while (false == digits.empty() && (digits.back() == ' ' || digits.back() == '	'))
		{
			digits.remove_suffix(1);
		}
		auto width{0};
		if (std::from_chars(digits.data(), digits.data() + digits.size(), width).ec != std::errc{} || width < 0 || width > max_font_image_side)
		{
			continue;
		}
		if (++glyphs > max_font_glyphs || x + width > max_font_columns)
		{
			return std::nullopt;
		}
		auto cut{cut_glyph(Rgba, Width, Height, x, width)};
		if (font.m_glyphs.empty())
		{
			font.m_fallback = cut;
		}
		font.m_glyphs.insert_or_assign(characters.front(), std::move(cut));
		x += width + 1;
	}
	if (font.m_glyphs.empty())
	{
		return std::nullopt;
	}
	return font;
}

glyph bitmap_font::sprite(std::span<std::uint8_t const> const Rgba, int const Width, int const Height)
{
	if (Width <= 0 || Height <= 0 || Rgba.size() < static_cast<std::size_t>(Width) * Height * 4)
	{
		return {};
	}
	return cut_glyph(Rgba, Width, Height, 0, Width);
}

glyph const &bitmap_font::find(char32_t const Character) const
{
	if (auto const lookup{m_glyphs.find(Character)}; lookup != m_glyphs.end())
	{
		return lookup->second;
	}
	return m_fallback;
}

int bitmap_font::text_width(std::string_view const Text, int const Spacing) const
{
	auto width{0};
	for (auto const character : decode_utf8(Text))
	{
		width += find(character).width() + Spacing;
	}
	return width;
}

canvas::canvas(int const Width, int const Height, rgb const Color)
    : m_width{std::max(Width, 0)}, m_height{std::max(Height, 0)}, m_pixels(static_cast<std::size_t>(m_width) * m_height * 4, 255)
{
	fill(Color);
}

canvas::canvas(int const Width, int const Height, std::vector<std::uint8_t> Pixels) : m_width{Width}, m_height{Height}, m_pixels{std::move(Pixels)}
{
	m_pixels.resize(static_cast<std::size_t>(std::max(Width, 0)) * std::max(Height, 0) * 4, 255);
}

void canvas::fill(rgb const Color)
{
	for (std::size_t at = 0; at < m_pixels.size(); at += 4)
	{
		std::ranges::copy(Color, m_pixels.begin() + static_cast<std::ptrdiff_t>(at));
		m_pixels[at + 3] = 255;
	}
}

void canvas::set(int const X, int const Y, rgb const Color)
{
	if (X < 0 || Y < 0 || X >= m_width || Y >= m_height)
	{
		return;
	}
	auto const at{(static_cast<std::size_t>(Y) * m_width + X) * 4};
	std::ranges::copy(Color, m_pixels.begin() + static_cast<std::ptrdiff_t>(at));
}

rgb canvas::get(int const X, int const Y) const
{
	if (X < 0 || Y < 0 || X >= m_width || Y >= m_height)
	{
		return {0, 0, 0};
	}
	auto const at{(static_cast<std::size_t>(Y) * m_width + X) * 4};
	return {m_pixels[at], m_pixels[at + 1], m_pixels[at + 2]};
}

void canvas::fill_rect(int const X, int const Y, int const Width, int const Height, rgb const Color)
{
	for (auto y = std::max(Y, 0); y < std::min(Y + Height, m_height); ++y)
	{
		for (auto x = std::max(X, 0); x < std::min(X + Width, m_width); ++x)
		{
			set(x, y, Color);
		}
	}
}

void canvas::draw_glyph(glyph const &Glyph, int const X, int const Y, rgb const Color)
{
	for (auto column = 0; column < Glyph.width(); ++column)
	{
		for (auto row = 0; row < Glyph.height(); ++row)
		{
			if (Glyph.lit(column, row))
			{
				set(X + column, Y + row, Color);
			}
		}
	}
}

int canvas::draw_text(bitmap_font const &Font, int X, int const Y, std::string_view const Text, rgb const Color, int const Spacing)
{
	for (auto const character : decode_utf8(Text))
	{
		auto const &shape{Font.find(character)};
		draw_glyph(shape, X, Y, Color);
		X += shape.width() + Spacing;
	}
	return X;
}

void canvas::draw_image(canvas const &Image, int const X, int const Y)
{
	for (auto y = 0; y < Image.height(); ++y)
	{
		for (auto x = 0; x < Image.width(); ++x)
		{
			set(X + x, Y + y, Image.get(x, y));
		}
	}
}

} // namespace script_screens
