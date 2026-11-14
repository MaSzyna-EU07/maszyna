/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "utilities/asciiname.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <vector>

namespace
{

// the characters of Windows-1250 from 0x80 up; 0 where the code page has none
constexpr std::array<std::uint16_t, 128> cp1250{
    0x20AC, 0x0000, 0x201A, 0x0000, 0x201E, 0x2026, 0x2020, 0x2021, 0x0000, 0x2030, 0x0160, 0x2039, 0x015A, 0x0164, 0x017D, 0x0179,
    0x0000, 0x2018, 0x2019, 0x201C, 0x201D, 0x2022, 0x2013, 0x2014, 0x0000, 0x2122, 0x0161, 0x203A, 0x015B, 0x0165, 0x017E, 0x017A,
    0x00A0, 0x02C7, 0x02D8, 0x0141, 0x00A4, 0x0104, 0x00A6, 0x00A7, 0x00A8, 0x00A9, 0x015E, 0x00AB, 0x00AC, 0x00AD, 0x00AE, 0x017B,
    0x00B0, 0x00B1, 0x02DB, 0x0142, 0x00B4, 0x00B5, 0x00B6, 0x00B7, 0x00B8, 0x0105, 0x015F, 0x00BB, 0x013D, 0x02DD, 0x013E, 0x017C,
    0x0154, 0x00C1, 0x00C2, 0x0102, 0x00C4, 0x0139, 0x0106, 0x00C7, 0x010C, 0x00C9, 0x0118, 0x00CB, 0x011A, 0x00CD, 0x00CE, 0x010E,
    0x0110, 0x0143, 0x0147, 0x00D3, 0x00D4, 0x0150, 0x00D6, 0x00D7, 0x0158, 0x016E, 0x00DA, 0x0170, 0x00DC, 0x00DD, 0x0162, 0x00DF,
    0x0155, 0x00E1, 0x00E2, 0x0103, 0x00E4, 0x013A, 0x0107, 0x00E7, 0x010D, 0x00E9, 0x0119, 0x00EB, 0x011B, 0x00ED, 0x00EE, 0x010F,
    0x0111, 0x0144, 0x0148, 0x00F3, 0x00F4, 0x0151, 0x00F6, 0x00F7, 0x0159, 0x016F, 0x00FA, 0x0171, 0x00FC, 0x00FD, 0x0163, 0x02D9};

// the letters of Latin-1 from U+00C0 up, without their marks
constexpr std::array<std::string_view, 64> latin1{
    "a", "a", "a", "a", "a", "a", "ae", "c", "e", "e", "e", "e", "i", "i", "i", "i",
    "d", "n", "o", "o", "o", "o", "o", "_", "o", "u", "u", "u", "u", "y", "th", "ss",
    "a", "a", "a", "a", "a", "a", "ae", "c", "e", "e", "e", "e", "i", "i", "i", "i",
    "d", "n", "o", "o", "o", "o", "o", "_", "o", "u", "u", "u", "u", "y", "th", "y"};

// the letters of Latin Extended-A, U+0100 to U+017F, in runs of the same plain letter
struct run
{
	int count;
	std::string_view letter;
};
constexpr std::array<run, 22> extended{{{6, "a"},  {8, "c"},  {4, "d"}, {10, "e"}, {8, "g"}, {4, "h"}, {10, "i"}, {2, "ij"}, {2, "j"}, {3, "k"}, {10, "l"},
                                        {9, "n"},  {6, "o"},  {2, "oe"}, {6, "r"}, {8, "s"}, {6, "t"}, {12, "u"}, {2, "w"},  {3, "y"}, {6, "z"},  {1, "s"}}};

std::string_view plain_letter(char32_t const Code)
{
	if (Code >= 0xC0 && Code < 0x100)
		return latin1[Code - 0xC0];
	if (Code >= 0x100 && Code < 0x180)
	{
		auto left{static_cast<int>(Code - 0x100)};
		for (auto const &letters : extended)
		{
			if (left < letters.count)
				return letters.letter;
			left -= letters.count;
		}
	}
	return "_";
}

// the length of the UTF-8 sequence the byte begins, with the bits of the character it holds; 0 for a byte which can't begin one
int sequence_length(unsigned char const Lead, char32_t &Code)
{
	if (Lead < 0x80)
	{
		Code = Lead;
		return 1;
	}
	if (Lead < 0xC2 || Lead >= 0xF5)
		return 0;
	if (Lead < 0xE0)
	{
		Code = Lead & 0x1Fu;
		return 2;
	}
	if (Lead < 0xF0)
	{
		Code = Lead & 0x0Fu;
		return 3;
	}
	Code = Lead & 0x07u;
	return 4;
}

// the character of the UTF-8 sequence at the given place; false if there's no valid one
bool decode_one(std::string_view const Text, std::size_t const At, int const Length, char32_t &Code)
{
	if (At + Length > Text.size())
		return false;
	for (int i = 1; i < Length; ++i)
	{
		auto const next{static_cast<unsigned char>(Text[At + i])};
		if ((next & 0xC0u) != 0x80u)
			return false;
		Code = (Code << 6) | (next & 0x3Fu);
	}
	// the shortest form only, as UTF-8 has it
	return false == ((Length == 3 && Code < 0x800) || (Length == 4 && (Code < 0x10000 || Code > 0x10FFFF)));
}

// the characters of a valid UTF-8 text; false if it isn't one
bool decode_utf8(std::string_view const Text, std::vector<char32_t> &Codes)
{
	Codes.clear();
	std::size_t at{0};
	while (at < Text.size())
	{
		char32_t code{0};
		auto const length{sequence_length(static_cast<unsigned char>(Text[at]), code)};
		if (length == 0 || false == decode_one(Text, at, length, code))
			return false;
		Codes.push_back(code);
		at += length;
	}
	return true;
}

std::vector<char32_t> characters(std::string_view const Name)
{
	std::vector<char32_t> codes;
	if (decode_utf8(Name, codes))
		return codes;
	codes.clear();
	for (auto const character : Name)
	{
		auto const byte{static_cast<unsigned char>(character)};
		if (byte < 0x80)
			codes.push_back(byte);
		else
			codes.push_back(cp1250[byte - 0x80] != 0 ? cp1250[byte - 0x80] : 0xFFFD);
	}
	return codes;
}

void append_utf8(std::string &Output, char32_t const Code)
{
	if (Code < 0x80)
		Output += static_cast<char>(Code);
	else if (Code < 0x800)
	{
		Output += static_cast<char>(0xC0 | (Code >> 6));
		Output += static_cast<char>(0x80 | (Code & 0x3F));
	}
	else if (Code < 0x10000)
	{
		Output += static_cast<char>(0xE0 | (Code >> 12));
		Output += static_cast<char>(0x80 | ((Code >> 6) & 0x3F));
		Output += static_cast<char>(0x80 | (Code & 0x3F));
	}
	else
	{
		Output += static_cast<char>(0xF0 | (Code >> 18));
		Output += static_cast<char>(0x80 | ((Code >> 12) & 0x3F));
		Output += static_cast<char>(0x80 | ((Code >> 6) & 0x3F));
		Output += static_cast<char>(0x80 | (Code & 0x3F));
	}
}

} // namespace

namespace ascii_name
{

bool plain(std::string_view const Name)
{
	return std::ranges::all_of(Name, [](char const Character) { return static_cast<unsigned char>(Character) < 0x80; });
}

std::string fold(std::string_view const Name)
{
	std::string result;
	result.reserve(Name.size());
	for (auto const code : characters(Name))
	{
		if (code < 0x80)
			result += static_cast<char>(std::tolower(static_cast<int>(code)));
		else
			result += plain_letter(code);
	}
	return result;
}

std::string readable(std::string_view const Name)
{
	std::string result;
	result.reserve(Name.size() * 2);
	for (auto const code : characters(Name))
		append_utf8(result, code);
	return result;
}

} // namespace ascii_name
