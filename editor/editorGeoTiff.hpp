/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <vector>

namespace editor_geotiff
{

// a GeoTIFF as the terrain model service gives it: one band of 32-bit floats in strips, not compressed, placed on the grid by its
// tie point and pixel scale, into the width, height, heights (rows from the north, Missing where the file has none), pixel, east
// and north of Out. false for anything else, and for a file cut short
template <typename Square>
bool read(std::vector<std::uint8_t> const &Data, Square &Out, float const Missing)
{
	if (Data.size() < 8)
		return false;
	bool const little{Data[0] == 'I' && Data[1] == 'I'};
	if (false == little && false == (Data[0] == 'M' && Data[1] == 'M'))
		return false;
	auto const u16 = [&](std::size_t const At) -> std::uint32_t {
		if (At + 2 > Data.size())
			return 0;
		return little ? Data[At] | (Data[At + 1] << 8) : (Data[At] << 8) | Data[At + 1];
	};
	auto const u32 = [&](std::size_t const At) -> std::uint32_t {
		if (At + 4 > Data.size())
			return 0;
		return little ? Data[At] | (Data[At + 1] << 8) | (Data[At + 2] << 16) | (static_cast<std::uint32_t>(Data[At + 3]) << 24) :
		                (static_cast<std::uint32_t>(Data[At]) << 24) | (Data[At + 1] << 16) | (Data[At + 2] << 8) | Data[At + 3];
	};
	auto const f64 = [&](std::size_t const At) {
		std::uint64_t bits{0};
		for (int i = 0; i < 8; ++i)
			bits |= static_cast<std::uint64_t>(At + i < Data.size() ? Data[At + (little ? i : 7 - i)] : 0) << (8 * i);
		double value;
		std::memcpy(&value, &bits, sizeof(value));
		return value;
	};
	if (u16(2) != 42)
		return false;
	struct entry
	{
		std::uint32_t type{0};
		std::uint32_t count{0};
		std::size_t at{0}; // of the values: in the entry when they fit in four bytes
	};
	std::map<std::uint32_t, entry> entries;
	auto const ifd{static_cast<std::size_t>(u32(4))};
	auto const count{u16(ifd)};
	for (std::uint32_t i = 0; i < count; ++i)
	{
		auto const at{ifd + 2 + i * 12};
		entry value{u16(at + 2), u32(at + 4), at + 8};
		auto const size{(value.type == 3 ? 2u : value.type == 12 ? 8u : 4u) * value.count};
		if (size > 4)
			value.at = u32(at + 8);
		entries[u16(at)] = value;
	}
	auto const number = [&](std::uint32_t const Tag, std::uint32_t const Index) -> std::uint32_t {
		auto const found{entries.find(Tag)};
		if (found == entries.end() || Index >= found->second.count)
			return 0;
		return found->second.type == 3 ? u16(found->second.at + Index * 2) : u32(found->second.at + Index * 4);
	};
	auto const real = [&](std::uint32_t const Tag, std::uint32_t const Index) {
		auto const found{entries.find(Tag)};
		return found == entries.end() || Index >= found->second.count ? 0.0 : f64(found->second.at + Index * 8);
	};
	auto const width{static_cast<int>(number(256, 0))};
	auto const height{static_cast<int>(number(257, 0))};
	if (width < 2 || height < 2 || width > 8192 || height > 8192 || number(258, 0) != 32 || number(339, 0) != 3 || std::max(1u, number(259, 0)) != 1 || false == entries.contains(273) || false == entries.contains(33550) ||
	    false == entries.contains(33922))
		return false;
	auto const rows{std::min<std::uint32_t>(std::max(1u, number(278, 0)), height)};
	// a file cut short, by a write or a fetch broken off, lacks strips or a part of the last one: the rest read past its end would
	// come out as heights of 0 m
	auto const strips{(height + rows - 1) / rows};
	if (entries[273].count < strips)
		return false;
	for (std::uint32_t strip = 0; strip < strips; ++strip)
	{
		auto const size{static_cast<std::size_t>(std::min<std::uint32_t>(rows, height - strip * rows)) * width * 4};
		if (static_cast<std::size_t>(number(273, strip)) + size > Data.size())
			return false;
	}
	Out.width = width;
	Out.height = height;
	Out.heights.assign(static_cast<std::size_t>(width) * height, Missing);
	for (std::uint32_t strip = 0; strip < strips; ++strip)
	{
		auto const offset{static_cast<std::size_t>(number(273, strip))};
		auto const first{strip * rows};
		for (std::uint32_t row = first; row < std::min<std::uint32_t>(first + rows, height); ++row)
			for (int column = 0; column < width; ++column)
			{
				auto const bits{u32(offset + ((row - first) * width + column) * 4)};
				float value;
				std::memcpy(&value, &bits, sizeof(value));
				Out.heights[static_cast<std::size_t>(row) * width + column] = value;
			}
	}
	Out.pixel = real(33550, 0);
	Out.east = real(33922, 3) - real(33922, 0) * Out.pixel;
	Out.north = real(33922, 4) + real(33922, 1) * Out.pixel;
	return Out.pixel > 0.0;
}

} // namespace editor_geotiff
