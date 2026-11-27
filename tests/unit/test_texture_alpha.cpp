#include "stdafx.h"
#include "doctest.h"
#include "rendering/common/texturealpha.h"

#include <algorithm>
#include <vector>

// which textures the shadow maps draw without the alpha test: every texel has to pass it, so a block which can decode to a
// translucent texel makes the whole texture keep the test
namespace
{
using gfx::alpha_layout;
using gfx::texels_pass_alpha_test;

bool passes(alpha_layout const Layout, std::vector<std::uint8_t> const &Data)
{
	return texels_pass_alpha_test(Layout, Data);
}

// a bc7 block of mode 6 (7 bit colour and alpha endpoints, a p-bit each) with the given alpha endpoints before the p-bits
std::vector<std::uint8_t> bc7_mode6(unsigned const Alpha0, unsigned const Alpha1, unsigned const Pbits)
{
	std::vector<std::uint8_t> block(16, 0);
	auto const put = [&block](unsigned const Offset, unsigned const Count, unsigned const Value) {
		for (unsigned bit = 0; bit < Count; ++bit)
		{
			if (((Value >> bit) & 1u) != 0)
			{
				block[(Offset + bit) >> 3] |= static_cast<std::uint8_t>(1u << ((Offset + bit) & 7u));
			}
		}
	};
	put(6, 1, 1);
	put(49, 7, Alpha0);
	put(56, 7, Alpha1);
	put(63, 2, Pbits);
	return block;
}
} // namespace

TEST_CASE("texture alpha: rgba texels pass when every alpha is from 136 up")
{
	CHECK(passes(alpha_layout::rgba8, {10, 20, 30, 255, 0, 0, 0, 136}));
	CHECK_FALSE(passes(alpha_layout::rgba8, {10, 20, 30, 255, 0, 0, 0, 135}));
	CHECK_FALSE(passes(alpha_layout::rgba8, {255, 255, 255, 0}));
}

TEST_CASE("texture alpha: bc1 is opaque unless a three colour block uses the index 3")
{
	// the first colour above the second: four colours, the index 3 opaque
	CHECK(passes(alpha_layout::bc1, {0x00, 0xf8, 0x1f, 0x00, 0xff, 0xff, 0xff, 0xff}));
	// three colours, no index 3
	CHECK(passes(alpha_layout::bc1, {0x1f, 0x00, 0x00, 0xf8, 0x00, 0x55, 0xaa, 0x66}));
	// three colours, one texel of the index 3: transparent black
	CHECK_FALSE(passes(alpha_layout::bc1, {0x1f, 0x00, 0x00, 0xf8, 0x00, 0x00, 0x30, 0x00}));
	// equal colours count as three
	CHECK_FALSE(passes(alpha_layout::bc1, {0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00}));
	// one translucent block among opaque ones
	CHECK_FALSE(passes(alpha_layout::bc1, {0x00, 0xf8, 0x1f, 0x00, 0xff, 0xff, 0xff, 0xff, 0x1f, 0x00, 0x00, 0xf8, 0xc0, 0x00, 0x00, 0x00}));
}

TEST_CASE("texture alpha: bc2 passes when every 4 bit alpha is from 8 up")
{
	std::vector<std::uint8_t> block(16, 0);
	std::fill_n(block.begin(), 8, std::uint8_t{0x88});
	CHECK(passes(alpha_layout::bc2, block));
	block[5] = 0x87;
	CHECK_FALSE(passes(alpha_layout::bc2, block));
}

TEST_CASE("texture alpha: bc3 is checked by the palette entries its texels use")
{
	// both endpoints opaque
	std::vector<std::uint8_t> block{255, 255, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	CHECK(passes(alpha_layout::bc3, block));
	// a transparent second endpoint which no texel uses (all indices 0)
	block[1] = 0;
	CHECK(passes(alpha_layout::bc3, block));
	// one texel on the index 1, the transparent endpoint
	block[2] = 0x01;
	CHECK_FALSE(passes(alpha_layout::bc3, block));
	// six interpolated values: the index 6 is 0, the index 7 is 255
	std::vector<std::uint8_t> six{200, 220, 0x07, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	CHECK(passes(alpha_layout::bc3, six));
	six[2] = 0x06;
	CHECK_FALSE(passes(alpha_layout::bc3, six));
}

TEST_CASE("texture alpha: bc7 is checked by its alpha endpoints")
{
	// mode 6, alpha endpoints 254 and 255
	CHECK(passes(alpha_layout::bc7, bc7_mode6(127, 127, 2)));
	// one endpoint at 100
	CHECK_FALSE(passes(alpha_layout::bc7, bc7_mode6(50, 127, 3)));
	// mode 1 holds no alpha
	std::vector<std::uint8_t> mode1(16, 0);
	mode1[0] = 0x02;
	CHECK(passes(alpha_layout::bc7, mode1));
	// the reserved mode decodes to transparent black
	CHECK_FALSE(passes(alpha_layout::bc7, std::vector<std::uint8_t>(16, 0)));
}
