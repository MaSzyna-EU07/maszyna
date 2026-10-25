/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/exposure.h"

#include "gl/ubo.h"
#include "utilities/Globals.h"

namespace
{

// luminance range of the histogram, log2: 2^-12 .. 2^6. darker pixels count as black, brighter ones as the brightest
float const HISTOGRAM_LOG_MIN{-12.f};
float const HISTOGRAM_LOG_MAX{6.f};
// the bloom chain goes down this many levels at most, and not below this size
std::size_t const BLOOM_LEVELS{6};
int const BLOOM_MIN_SIZE{8};
// soft knee of the threshold, a fraction of it
float const BLOOM_KNEE{0.5f};

} // namespace

namespace gfx
{

glm::vec2 exposure_settings::histogram_range()
{
	auto const bins{static_cast<float>(gl::EXPOSURE_HISTOGRAM_BINS - 2)};
	return glm::vec2{HISTOGRAM_LOG_MIN, bins / (HISTOGRAM_LOG_MAX - HISTOGRAM_LOG_MIN)};
}

glm::vec2 exposure_settings::average_range()
{
	auto const bins{static_cast<float>(gl::EXPOSURE_HISTOGRAM_BINS - 2)};
	return glm::vec2{HISTOGRAM_LOG_MIN, (HISTOGRAM_LOG_MAX - HISTOGRAM_LOG_MIN) / bins};
}

glm::vec4 exposure_settings::adaptation(double const Deltatime)
{
	return glm::vec4{static_cast<float>(std::clamp(Deltatime, 0.0, 1.0)), Global.gfx_postfx_autoexposure_speed_dark, Global.gfx_postfx_autoexposure_speed_bright, Global.gfx_postfx_autoexposure_key};
}

glm::vec2 exposure_settings::exposure_range()
{
	return glm::vec2{std::exp2(Global.gfx_postfx_autoexposure_min), std::exp2(Global.gfx_postfx_autoexposure_max)};
}

std::vector<glm::ivec2> exposure_settings::bloom_sizes(int const Width, int const Height)
{
	std::vector<glm::ivec2> sizes;
	glm::ivec2 size{Width, Height};
	for (std::size_t level = 0; level < BLOOM_LEVELS; ++level)
	{
		size = glm::max(size / 2, glm::ivec2{1});
		if (level > 0 && std::min(size.x, size.y) < BLOOM_MIN_SIZE)
			break;
		sizes.push_back(size);
	}
	return sizes;
}

glm::vec4 exposure_settings::bloom_params(std::size_t const Level)
{
	return glm::vec4{(Level == 0 ? 1.f : 0.f), Global.gfx_postfx_bloom_threshold, BLOOM_KNEE, 0.f};
}

glm::vec4 exposure_settings::tonemap_params(std::size_t const Bloomlevels)
{
	auto const bloom{(Global.gfx_postfx_bloom && Bloomlevels > 0) ? Global.gfx_postfx_bloom_intensity : 0.f};
	return glm::vec4{bloom, Global.gfx_postfx_autoexposure ? 1.f : 0.f, std::exp2(Global.gfx_postfx_autoexposure_compensation), Bloomlevels > 0 ? 1.f / static_cast<float>(Bloomlevels) : 0.f};
}

void exposure_settings::draw_debug_ui(float const *Readback)
{
	ImGui::Checkbox("automatic exposure (gfx.postfx.autoexposure)", &Global.gfx_postfx_autoexposure);
	ImGui::SliderFloat("key", &Global.gfx_postfx_autoexposure_key, 0.02f, 2.f, "%.3f", ImGuiSliderFlags_Logarithmic);
	ImGui::SliderFloat("min EV", &Global.gfx_postfx_autoexposure_min, -4.f, 0.f);
	ImGui::SliderFloat("max EV", &Global.gfx_postfx_autoexposure_max, 0.f, 4.f);
	ImGui::SliderFloat("adaptation to dark (1/s)", &Global.gfx_postfx_autoexposure_speed_dark, 0.05f, 10.f);
	ImGui::SliderFloat("adaptation to bright (1/s)", &Global.gfx_postfx_autoexposure_speed_bright, 0.05f, 10.f);
	ImGui::SliderFloat("compensation EV", &Global.gfx_postfx_autoexposure_compensation, -4.f, 4.f);
	ImGui::Text("average luminance %.3f, adapted %.3f, exposure %.3f (%+.2f EV)", Readback[2], Readback[0], Readback[1], std::log2(std::max(Readback[1], 1e-6f)));
	ImGui::Checkbox("bloom (gfx.postfx.bloom)", &Global.gfx_postfx_bloom);
	ImGui::SliderFloat("bloom intensity", &Global.gfx_postfx_bloom_intensity, 0.f, 0.5f);
	ImGui::SliderFloat("bloom threshold", &Global.gfx_postfx_bloom_threshold, 0.f, 10.f);
}

} // namespace gfx
