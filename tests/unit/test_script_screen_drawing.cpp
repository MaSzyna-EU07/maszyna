#include "stdafx.h"
#include "doctest.h"

#include "scripting/scriptscreendrawing.h"

#include <algorithm>
#include <array>
#include <vector>

// the drawing the Lua screens get from the game: fonts read as scripts/led_util.py reads them, text and images

using script_screens::bitmap_font;
using script_screens::canvas;
using script_screens::rgb;

namespace {

// an RGBA image Width x Height, white, with the listed pixels set to the colour
std::vector<std::uint8_t> image(int const Width, int const Height, std::vector<std::pair<int, int>> const &Pixels,
                                std::array<std::uint8_t, 4> const Color = {0, 0, 0, 255}) {
	std::vector<std::uint8_t> pixels(static_cast<std::size_t>(Width) * Height * 4, 255);
	for (auto const &[x, y] : Pixels) {
		std::copy(Color.begin(), Color.end(), pixels.begin() + (static_cast<std::ptrdiff_t>(y) * Width + x) * 4);
	}
	return pixels;
}

} // namespace

TEST_CASE("script screen drawing: UTF-8 text becomes code points, a stray byte stands for itself") {
	CHECK(script_screens::decode_utf8("ab") == U"ab");
	CHECK(script_screens::decode_utf8("\xc5\x81\xc3\xb3\xc5\xbc") == U"Łóż"); // Łóż
	CHECK(script_screens::decode_utf8("a\xb3z") == U"a³z");
	CHECK(script_screens::decode_utf8("\xe2\x82") == U"â\u0082");
}

TEST_CASE("script screen drawing: a font as led_util reads it, glyphs a column apart") {
	// "a" 2 wide (x 0-1), "Ł" 1 wide (x 3), " " 1 wide (x 5)
	auto const pixels{image(6, 2, {{0, 0}, {1, 1}, {3, 0}, {3, 1}})};
	auto const font{bitmap_font::parse(pixels, 6, 2, "// comment\r\na2\r\n\xc5\x81" "1\r\n 1\r\n")};
	REQUIRE(font.has_value());
	CHECK(font->height() == 2);

	auto const &a{font->find(U'a')};
	REQUIRE(a.width() == 2);
	CHECK(a.lit(0, 0));
	CHECK_FALSE(a.lit(0, 1));
	CHECK(a.lit(1, 1));

	auto const &l{font->find(U'Ł')};
	REQUIRE(l.width() == 1);
	CHECK(l.lit(0, 0));
	CHECK(l.lit(0, 1));

	auto const &space{font->find(U' ')};
	REQUIRE(space.width() == 1);
	CHECK_FALSE(space.lit(0, 0));

	// a character the font lacks: its first glyph
	CHECK(font->find(U'q').width() == 2);
	// led_util's get_str_len: a spacing column after every character
	CHECK(font->text_width("a\xc5\x81", 1) == 5);
	CHECK(font->text_width("", 1) == 0);
}

TEST_CASE("script screen drawing: a pixel is lit unless it is opaque white") {
	auto pixels{image(3, 1, {{0, 0}})};
	// transparent white: lit, as led_util compares the whole pixel with (255, 255, 255, 255)
	pixels[1 * 4 + 3] = 0;
	auto const sprite{bitmap_font::sprite(pixels, 3, 1)};
	REQUIRE(sprite.width() == 3);
	CHECK(sprite.lit(0, 0));
	CHECK(sprite.lit(1, 0));
	CHECK_FALSE(sprite.lit(2, 0));
}

TEST_CASE("script screen drawing: a description without glyphs, or one wider than its image, does no harm") {
	auto const pixels{image(2, 1, {})};
	CHECK_FALSE(bitmap_font::parse(pixels, 2, 1, "// only a comment\n").has_value());
	CHECK_FALSE(bitmap_font::parse(pixels, 2, 1, "").has_value());
	// the glyph runs past the image: the columns past its edge are unlit
	auto const font{bitmap_font::parse(pixels, 2, 1, "//\nw5\n")};
	REQUIRE(font.has_value());
	CHECK(font->find(U'w').width() == 5);
	CHECK_FALSE(font->find(U'w').lit(4, 0));
}

TEST_CASE("script screen drawing: text is drawn from x rightwards and returns where it ends") {
	auto const pixels{image(4, 1, {{0, 0}, {1, 0}, {3, 0}})};
	auto const font{bitmap_font::parse(pixels, 4, 1, "//\na2\nb1\n")};
	REQUIRE(font.has_value());
	canvas board{8, 1, rgb{0, 0, 0}};
	auto const end{board.draw_text(*font, 1, 0, "ab", rgb{9, 8, 7}, 1)};
	CHECK(end == 6);
	CHECK(board.get(0, 0) == rgb{0, 0, 0});
	CHECK(board.get(1, 0) == rgb{9, 8, 7});
	CHECK(board.get(2, 0) == rgb{9, 8, 7});
	CHECK(board.get(3, 0) == rgb{0, 0, 0});
	CHECK(board.get(4, 0) == rgb{9, 8, 7});
	// partly off the image: clipped
	canvas narrow{2, 1, rgb{0, 0, 0}};
	narrow.draw_text(*font, -1, 0, "ab", rgb{1, 1, 1}, 1);
	CHECK(narrow.get(0, 0) == rgb{1, 1, 1});
}

TEST_CASE("script screen drawing: rectangles and images are clipped to the image") {
	canvas board{3, 2, rgb{0, 0, 0}};
	board.fill_rect(-5, 1, 100, 100, rgb{1, 2, 3});
	CHECK(board.get(0, 0) == rgb{0, 0, 0});
	CHECK(board.get(0, 1) == rgb{1, 2, 3});
	CHECK(board.get(2, 1) == rgb{1, 2, 3});
	CHECK(board.get(5, 5) == rgb{0, 0, 0});

	canvas const stamp{2, 2, rgb{7, 7, 7}};
	board.draw_image(stamp, 2, -1);
	CHECK(board.get(2, 0) == rgb{7, 7, 7});
	CHECK(board.get(1, 0) == rgb{0, 0, 0});
	// every pixel opaque
	CHECK(board.pixels()[3] == 255);
}
