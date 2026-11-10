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

namespace gfx
{

// how the texels of a texture's data are laid out: rgba (or bgra) bytes, or the 4x4 blocks of the compressed formats
enum class alpha_layout
{
	rgba8,
	bc1,
	bc2,
	bc3,
	bc7
};

// the alpha the alpha test of the shadow maps (alphashadowmap.frag) keeps for sure: it discards below a half, and this leaves a
// margin for the decoders and the filtering of the gpus (136/255, the 8/15 of the 4 bit alpha of bc2)
constexpr std::uint8_t alpha_test_kept{136};

// whether every texel of Data (the levels of the texture one after another, whole blocks) decodes to an alpha of at least
// alpha_test_kept. the filtering only mixes texels, so the alpha test then keeps every fragment, and a program without the test
// draws the same depth. the compressed blocks are checked by what they can decode to, so the answer errs on the side of a
// translucent texture
bool texels_pass_alpha_test(alpha_layout Layout, std::span<std::uint8_t const> Data);

} // namespace gfx
