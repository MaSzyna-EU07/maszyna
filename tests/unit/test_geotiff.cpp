#include "stdafx.h"
#include "doctest.h"
#include "editor/editorGeoTiff.hpp"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <vector>

namespace {

struct raster {
	int width{0};
	int height{0};
	std::vector<float> heights;
	double pixel{0.0};
	double east{0.0};
	double north{0.0};
};

float constexpr missing{-1001.f};

// a GeoTIFF like the one of the terrain model service: little-endian, one band of floats, Rows rows a strip, Strips of them
// listed, the pixel of 1 m, the north-west corner at 1000, 2000. The height of a pixel is its row * 10 + its column
std::vector<std::uint8_t> geotiff(int const Width, int const Height, std::uint32_t const Rows, std::uint32_t const Strips) {
	std::vector<std::uint8_t> out;
	auto const put16 = [&](std::uint32_t const Value) {
		for (int i = 0; i < 2; ++i)
			out.push_back(static_cast<std::uint8_t>(Value >> (8 * i)));
	};
	auto const put32 = [&](std::uint32_t const Value) {
		for (int i = 0; i < 4; ++i)
			out.push_back(static_cast<std::uint8_t>(Value >> (8 * i)));
	};
	auto const put64 = [&](double const Value) {
		std::uint64_t bits;
		std::memcpy(&bits, &Value, sizeof(bits));
		for (int i = 0; i < 8; ++i)
			out.push_back(static_cast<std::uint8_t>(bits >> (8 * i)));
	};
	std::uint32_t constexpr count{9};
	std::uint32_t constexpr ifd{8};
	std::uint32_t const offsets{ifd + 2 + count * 12 + 4};
	std::uint32_t const scale{offsets + 4 * Strips};
	std::uint32_t const tie{scale + 3 * 8};
	std::uint32_t const pixels{tie + 6 * 8};
	auto const rows{std::min<std::uint32_t>(Rows, Height)};
	out.push_back('I');
	out.push_back('I');
	put16(42);
	put32(ifd);
	put16(count);
	auto const entry = [&](std::uint32_t const Tag, std::uint32_t const Type, std::uint32_t const Count, std::uint32_t const Value) {
		put16(Tag);
		put16(Type);
		put32(Count);
		put32(Value);
	};
	entry(256, 4, 1, Width);
	entry(257, 4, 1, Height);
	entry(258, 3, 1, 32);
	entry(259, 3, 1, 1);
	entry(273, 4, Strips, Strips == 1 ? pixels : offsets);
	entry(278, 4, 1, Rows);
	entry(339, 3, 1, 3);
	entry(33550, 12, 3, scale);
	entry(33922, 12, 6, tie);
	put32(0);
	for (std::uint32_t strip = 0; strip < Strips; ++strip)
		put32(pixels + strip * rows * Width * 4);
	for (double const value : {1.0, 1.0, 0.0})
		put64(value);
	for (double const value : {0.0, 0.0, 0.0, 1000.0, 2000.0, 0.0})
		put64(value);
	for (int row = 0; row < Height; ++row)
		for (int column = 0; column < Width; ++column) {
			float const height{static_cast<float>(row * 10 + column)};
			std::uint32_t bits;
			std::memcpy(&bits, &height, sizeof(bits));
			put32(bits);
		}
	return out;
}

} // namespace

TEST_CASE("terrain model: a GeoTIFF of the service read whole") {
	auto const data{geotiff(5, 7, 2, 4)};
	raster square;
	REQUIRE(editor_geotiff::read(data, square, missing));
	CHECK(square.width == 5);
	CHECK(square.height == 7);
	REQUIRE(square.heights.size() == 35);
	CHECK(square.heights[0] == 0.f);
	CHECK(square.heights[2 * 5 + 3] == 23.f);
	CHECK(square.heights[6 * 5 + 4] == 64.f); // in the last strip, of one row
	CHECK(square.pixel == 1.0);
	CHECK(square.east == 1000.0);
	CHECK(square.north == 2000.0);
}

TEST_CASE("terrain model: the rows all in one strip") {
	raster square;
	REQUIRE(editor_geotiff::read(geotiff(5, 7, 0xffffffffu, 1), square, missing));
	CHECK(square.heights[6 * 5 + 4] == 64.f);
}

TEST_CASE("terrain model: a GeoTIFF cut short isn't read, its missing pixels would be heights of 0 m") {
	auto data{geotiff(5, 7, 2, 4)};
	raster square;
	SUBCASE("by a byte") {
		data.pop_back();
		CHECK_FALSE(editor_geotiff::read(data, square, missing));
	}
	SUBCASE("by its last strip") {
		data.resize(data.size() - 5 * 4);
		CHECK_FALSE(editor_geotiff::read(data, square, missing));
	}
	SUBCASE("to the half of its pixels") {
		data.resize(data.size() - 5 * 7 * 2);
		CHECK_FALSE(editor_geotiff::read(data, square, missing));
	}
	CHECK(square.heights.empty());
}

TEST_CASE("terrain model: a GeoTIFF listing fewer strips than its rows need isn't read") {
	raster square;
	CHECK_FALSE(editor_geotiff::read(geotiff(5, 7, 2, 3), square, missing));
}
