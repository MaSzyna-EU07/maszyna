/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// what the automatic exposure and the bloom of the renderers share (the part of the stage S3 of
// claude/gl46-renderer-features-plan.md which isn't the work of the gpu, taken out of opengl46_exposure for the vulkan renderer,
// stage P1 of claude/vulkan14-renderer-plan.md): the parameters of the histogram and of the adaptation
// (exposure_histogram.comp, exposure_average.comp), the sizes of the levels of the bloom chain and the parameters of its steps
// (postfx_bloom_*.frag), the parameters of the tonemapping and the controls of the debug window

#pragma once

#include <cstddef>
#include <vector>

namespace gfx
{

struct exposure_settings
{
	// x: log2 of the lowest luminance binned, y: bins per unit of log2 luminance (exposure_histogram.comp)
	static glm::vec2 histogram_range();
	// x: log2 of the lowest luminance binned, y: log2 luminance per bin (exposure_average.comp)
	static glm::vec2 average_range();
	// x: time since the last measurement (Deltatime, s), y, z: rates of the adaptation to a darker and a brighter image, w: key
	static glm::vec4 adaptation(double const Deltatime);
	// the lowest and the highest exposure (multipliers)
	static glm::vec2 exposure_range();
	// the sizes of the levels of the bloom chain of an image, half the size each, as many as there are (empty: no chain)
	static std::vector<glm::ivec2> bloom_sizes(int const Width, int const Height);
	// param[0] of a step down the chain (postfx_bloom_down.frag): Level 0 is the first one, from the image itself
	static glm::vec4 bloom_params(std::size_t const Level);
	// parameters of the tonemapping (postfx_tonemapping.frag), with bloom from a chain of Bloomlevels levels (0: none)
	static glm::vec4 tonemap_params(std::size_t const Bloomlevels);
	// controls of the debug window; Readback: the adapted luminance, the exposure and the average luminance the gpu worked out
	static void draw_debug_ui(float const *Readback);
};

} // namespace gfx
