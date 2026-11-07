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
#include <bit>
#include <cstddef>
#include <cstdint>
#include <map>
#include <span>
#include <vector>

namespace editor_geotiff
{

namespace detail
{

// the bytes of a TIFF read as numbers in its byte order: 0 for a number past the end
struct bytes
{
	std::span<std::uint8_t const> data;
	bool little{true};

	// a byte, widened before it's shifted
	std::uint32_t widened(std::size_t const At) const { return static_cast<std::uint32_t>(data[At]); }

	std::uint32_t u16(std::size_t const At) const
	{
		if (At + 2 > data.size())
			return 0;
		return little ? widened(At) | (widened(At + 1) << 8) : (widened(At) << 8) | widened(At + 1);
	}

	std::uint32_t u32(std::size_t const At) const
	{
		if (At + 4 > data.size())
			return 0;
		return little ? widened(At) | (widened(At + 1) << 8) | (widened(At + 2) << 16) | (widened(At + 3) << 24) :
		                (widened(At) << 24) | (widened(At + 1) << 16) | (widened(At + 2) << 8) | widened(At + 3);
	}

	double f64(std::size_t const At) const
	{
		std::uint64_t bits{0};
		for (int i = 0; i < 8; ++i)
		{
			// a byte past the end adds nothing
			if (At + i >= data.size())
				continue;
			auto const from{At + (little ? i : 7 - i)};
			bits |= static_cast<std::uint64_t>(data[from]) << (8 * i);
		}
		return std::bit_cast<double>(bits);
	}
};

// a field of the image file directory
struct entry
{
	std::uint32_t type{0};
	std::uint32_t count{0};
	std::size_t at{0}; // of the values: in the entry when they fit in four bytes
};

using entries = std::map<std::uint32_t, entry>;

// the bytes a value of a field of the type takes: a short, a double, four for the rest
inline std::uint32_t type_size(std::uint32_t const Type)
{
	if (Type == 3)
		return 2u;
	if (Type == 12)
		return 8u;
	return 4u;
}

// the fields of the image file directory at the offset, by their tags
inline entries entries_read(bytes const &File, std::size_t const Directory)
{
	entries out;
	auto const count{File.u16(Directory)};
	for (std::uint32_t i = 0; i < count; ++i)
	{
		auto const at{Directory + 2 + i * 12};
		entry value{File.u16(at + 2), File.u32(at + 4), at + 8};
		if (auto const size{type_size(value.type) * value.count}; size > 4)
			value.at = File.u32(at + 8);
		out[File.u16(at)] = value;
	}
	return out;
}

// a value of a field as a whole number, 0 when the field has none
inline std::uint32_t number(bytes const &File, entries const &Entries, std::uint32_t const Tag, std::uint32_t const Index)
{
	auto const found{Entries.find(Tag)};
	if (found == Entries.end() || Index >= found->second.count)
		return 0;
	return found->second.type == 3 ? File.u16(found->second.at + Index * 2) : File.u32(found->second.at + Index * 4);
}

// a value of a field as a double, 0 when the field has none
inline double real(bytes const &File, entries const &Entries, std::uint32_t const Tag, std::uint32_t const Index)
{
	auto const found{Entries.find(Tag)};
	return found == Entries.end() || Index >= found->second.count ? 0.0 : File.f64(found->second.at + Index * 8);
}

} // namespace detail

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
	detail::bytes const file{Data, little};
	if (file.u16(2) != 42)
		return false;
	auto entries{detail::entries_read(file, static_cast<std::size_t>(file.u32(4)))};
	auto const number = [&file, &entries](std::uint32_t const Tag, std::uint32_t const Index) { return detail::number(file, entries, Tag, Index); };
	auto const real = [&file, &entries](std::uint32_t const Tag, std::uint32_t const Index) { return detail::real(file, entries, Tag, Index); };
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
				Out.heights[static_cast<std::size_t>(row) * width + column] = std::bit_cast<float>(file.u32(offset + ((row - first) * width + column) * 4));
	}
	Out.pixel = real(33550, 0);
	Out.east = real(33922, 3) - real(33922, 0) * Out.pixel;
	Out.north = real(33922, 4) + real(33922, 1) * Out.pixel;
	return Out.pixel > 0.0;
}

} // namespace editor_geotiff
