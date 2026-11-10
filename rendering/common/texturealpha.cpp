/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/texturealpha.h"

#include <array>
#include <cstddef>

namespace gfx
{

namespace
{

using block_check = bool (*)(std::uint8_t const *Block);

bool rgba8_kept(std::span<std::uint8_t const> const Data)
{
	for (std::size_t alpha = 3; alpha < Data.size(); alpha += 4)
	{
		if (Data[alpha] < alpha_test_kept)
		{
			return false;
		}
	}
	return true;
}

// bc1: with the first colour above the second the block has four colours and no alpha; otherwise the index 3 is transparent black
bool bc1_kept(std::uint8_t const *Block)
{
	auto const colour0{static_cast<unsigned>(Block[0]) | (static_cast<unsigned>(Block[1]) << 8)};
	auto const colour1{static_cast<unsigned>(Block[2]) | (static_cast<unsigned>(Block[3]) << 8)};
	if (colour0 > colour1)
	{
		return true;
	}
	for (std::size_t row = 4; row < 8; ++row)
	{
		// both bits of a 2 bit index set: the index 3
		if ((Block[row] & (Block[row] >> 1) & 0x55u) != 0)
		{
			return false;
		}
	}
	return true;
}

// bc2: 4 bits of alpha per texel, kept from 8 up (the top bit of each half of a byte)
bool bc2_kept(std::uint8_t const *Block)
{
	for (std::size_t byte = 0; byte < 8; ++byte)
	{
		if ((Block[byte] & 0x88u) != 0x88u)
		{
			return false;
		}
	}
	return true;
}

// bc3: two alpha endpoints and 3 bit indices into the palette they make
bool bc3_kept(std::uint8_t const *Block)
{
	auto const alpha0{static_cast<unsigned>(Block[0])};
	auto const alpha1{static_cast<unsigned>(Block[1])};
	std::array<unsigned, 8> palette{alpha0, alpha1, 0u, 0u, 0u, 0u, 0u, 0u};
	if (alpha0 > alpha1)
	{
		for (unsigned index = 2; index < 8; ++index)
		{
			palette[index] = ((8 - index) * alpha0 + (index - 1) * alpha1) / 7;
		}
	}
	else
	{
		for (unsigned index = 2; index < 6; ++index)
		{
			palette[index] = ((6 - index) * alpha0 + (index - 1) * alpha1) / 5;
		}
		palette[6] = 0;
		palette[7] = 255;
	}
	std::uint64_t indices{0};
	for (std::size_t byte = 0; byte < 6; ++byte)
	{
		indices |= static_cast<std::uint64_t>(Block[2 + byte]) << (8 * byte);
	}
	for (unsigned texel = 0; texel < 16; ++texel)
	{
		if (palette[(indices >> (3 * texel)) & 7u] < alpha_test_kept)
		{
			return false;
		}
	}
	return true;
}

// Count bits of a bc7 block from bit Offset, the lowest first
unsigned bc7_bits(std::uint8_t const *Block, unsigned const Offset, unsigned const Count)
{
	unsigned result{0};
	for (unsigned bit = 0; bit < Count; ++bit)
	{
		auto const index{Offset + bit};
		result |= ((static_cast<unsigned>(Block[index >> 3]) >> (index & 7u)) & 1u) << bit;
	}
	return result;
}

// an endpoint of Bits bits as the decoder makes it 8 bits wide
unsigned bc7_expand(unsigned const Value, unsigned const Bits)
{
	return (Value << (8 - Bits)) | (Value >> (2 * Bits - 8));
}

// modes 4 and 5: one subset with alpha endpoints of their own, which the rotation swaps with one of the colour channels
bool bc7_rotated_kept(std::uint8_t const *Block, unsigned const Mode)
{
	auto const rotation{bc7_bits(Block, Mode + 1, 2)};
	auto const colourbits{Mode == 4 ? 5u : 7u};
	auto const alphabits{Mode == 4 ? 6u : 8u};
	auto const bits{rotation == 0 ? alphabits : colourbits};
	auto const start{rotation == 0 ? 8 + 6 * colourbits : 8 + 2 * colourbits * (rotation - 1)};
	for (unsigned endpoint = 0; endpoint < 2; ++endpoint)
	{
		if (bc7_expand(bc7_bits(Block, start + endpoint * bits, bits), bits) < alpha_test_kept)
		{
			return false;
		}
	}
	return true;
}

// bc7: the alpha of a texel lies between the alpha endpoints of its subset, so the endpoints decide
bool bc7_kept(std::uint8_t const *Block)
{
	if (Block[0] == 0)
	{
		// the reserved mode decodes to transparent black
		return false;
	}
	unsigned mode{0};
	while ((Block[0] & (1u << mode)) == 0)
	{
		++mode;
	}
	switch (mode)
	{
	case 4:
	case 5:
		return bc7_rotated_kept(Block, mode);
	case 6:
		// 7 bit alpha endpoints, each with its own p-bit
		for (unsigned endpoint = 0; endpoint < 2; ++endpoint)
		{
			if (((bc7_bits(Block, 49 + endpoint * 7, 7) << 1) | bc7_bits(Block, 63 + endpoint, 1)) < alpha_test_kept)
			{
				return false;
			}
		}
		return true;
	case 7:
		// two subsets, 5 bit alpha endpoints made 6 bits by their p-bits
		for (unsigned endpoint = 0; endpoint < 4; ++endpoint)
		{
			if (bc7_expand((bc7_bits(Block, 74 + endpoint * 5, 5) << 1) | bc7_bits(Block, 94 + endpoint, 1), 6) < alpha_test_kept)
			{
				return false;
			}
		}
		return true;
	default:
		// modes 0-3 hold no alpha: opaque
		return true;
	}
}

bool blocks_kept(std::span<std::uint8_t const> const Data, std::size_t const Size, block_check const Check)
{
	for (std::size_t offset = 0; offset + Size <= Data.size(); offset += Size)
	{
		if (false == Check(Data.data() + offset))
		{
			return false;
		}
	}
	return true;
}

} // namespace

bool texels_pass_alpha_test(alpha_layout const Layout, std::span<std::uint8_t const> const Data)
{
	switch (Layout)
	{
	case alpha_layout::rgba8:
		return rgba8_kept(Data);
	case alpha_layout::bc1:
		return blocks_kept(Data, 8, bc1_kept);
	case alpha_layout::bc2:
		return blocks_kept(Data, 16, bc2_kept);
	case alpha_layout::bc3:
		return blocks_kept(Data, 16, bc3_kept);
	case alpha_layout::bc7:
		return blocks_kept(Data, 16, bc7_kept);
	default:
		return false;
	}
}

} // namespace gfx
