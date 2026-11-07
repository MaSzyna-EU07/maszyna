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
#include "rendering/common/skyocclusion.h"
#include "simulation/simulation.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"
#include "vehicle/DynObj.h"
#include "vehicle/Train.h"

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
// colour temperature of the white the image is balanced to: the white of the display (d65 is the daylight of about 6500 K)
float const WHITE_BALANCE_REFERENCE{6500.f};
// distance off the daylight locus (duv, cie 1960 ucs) of the tint at its ends (-1, 1)
float const WHITE_BALANCE_TINT_DUV{0.02f};

// how much of the daylight is around the camera in the cab of its vehicle, 0..1: 0 out of the cab, at night, in a tunnel. the eye
// seated in the cab takes to its light, which the image of the cab with the bright windows would leave dark
float cab_daylight()
{
	if (FreeFlyModeFlag || (false == Global.render_cab) || (simulation::Train == nullptr))
		return 0.f;
	auto const *vehicle{simulation::Train->Dynamic()};
	if ((vehicle == nullptr) || (false == vehicle->bDisplayCab) || (vehicle->mdKabina == nullptr) || (vehicle->mdKabina == vehicle->mdModel))
		return 0.f;
	return std::clamp(static_cast<float>(Global.fLuminance) * gfx::vehicle_light_level(vehicle), 0.f, 1.f);
}

// chromaticity (cie xy) of the cie daylight of the correlated colour temperature Kelvin; the formula holds for 4000..25000 K
glm::dvec2 daylight_chromaticity(double const Kelvin)
{
	auto const t{std::clamp(Kelvin, 4000.0, 25000.0)};
	auto const x{t <= 7000.0 ? (-4.6070e9 / (t * t * t) + 2.9678e6 / (t * t) + 0.09911e3 / t + 0.244063)
	                         : (-2.0064e9 / (t * t * t) + 1.9018e6 / (t * t) + 0.24748e3 / t + 0.237040)};
	return glm::dvec2{x, -3.0 * x * x + 2.87 * x - 0.275};
}

glm::dvec2 xy_to_uv(glm::dvec2 const &Xy)
{
	auto const d{-2.0 * Xy.x + 12.0 * Xy.y + 3.0};
	return glm::dvec2{4.0 * Xy.x / d, 6.0 * Xy.y / d};
}

glm::dvec2 uv_to_xy(glm::dvec2 const &Uv)
{
	auto const d{2.0 * Uv.x - 8.0 * Uv.y + 4.0};
	return glm::dvec2{3.0 * Uv.x / d, 2.0 * Uv.y / d};
}

// cone responses (bradford) of the light of the temperature Kelvin, taken off the daylight locus by Tint: + towards green
// (v grows), so a positive tint takes the green out of the image and makes it magenta
glm::dvec3 white_cone_response(double const Kelvin, double const Tint)
{
	auto uv{xy_to_uv(daylight_chromaticity(Kelvin))};
	if (Tint != 0.0)
	{
		auto const along{xy_to_uv(daylight_chromaticity(Kelvin + 100.0)) - xy_to_uv(daylight_chromaticity(Kelvin - 100.0))};
		auto normal{glm::normalize(glm::dvec2{-along.y, along.x})};
		if (normal.y < 0.0)
			normal = -normal;
		uv += normal * (Tint * WHITE_BALANCE_TINT_DUV);
	}
	auto const xy{uv_to_xy(uv)};
	glm::dvec3 const xyz{xy.x / xy.y, 1.0, (1.0 - xy.x - xy.y) / xy.y};
	return glm::dvec3{0.8951 * xyz.x + 0.2664 * xyz.y - 0.1614 * xyz.z,
	                  -0.7502 * xyz.x + 1.7135 * xyz.y + 0.0367 * xyz.z,
	                  0.0389 * xyz.x - 0.0685 * xyz.y + 1.0296 * xyz.z};
}

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
	// seen from the cab by day the image may be brightened more (gfx.postfx.autoexposure.max.cab), by the daylight around the cab
	auto const highest{std::max(Global.gfx_postfx_autoexposure_max, Global.gfx_postfx_autoexposure_max + (Global.gfx_postfx_autoexposure_max_cab - Global.gfx_postfx_autoexposure_max) * cab_daylight())};
	return glm::vec2{std::exp2(Global.gfx_postfx_autoexposure_min), std::exp2(highest)};
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

glm::vec3 exposure_settings::white_balance()
{
	if (!Global.gfx_postfx_whitebalance)
		return glm::vec3{0.f};
	// von kries: the light of the set temperature becomes the white of the display; the luminance of grey stays
	auto const gains{white_cone_response(WHITE_BALANCE_REFERENCE, 0.0) / white_cone_response(Global.gfx_postfx_whitebalance_temperature, Global.gfx_postfx_whitebalance_tint)};
	return glm::vec3{gains};
}

void exposure_settings::draw_debug_ui(float const *Readback)
{
	ImGui::Checkbox("automatic exposure (gfx.postfx.autoexposure)", &Global.gfx_postfx_autoexposure);
	ImGui::SliderFloat("key", &Global.gfx_postfx_autoexposure_key, 0.02f, 2.f, "%.3f", ImGuiSliderFlags_Logarithmic);
	ImGui::SliderFloat("min EV", &Global.gfx_postfx_autoexposure_min, -4.f, 0.f);
	ImGui::SliderFloat("max EV", &Global.gfx_postfx_autoexposure_max, 0.f, 4.f);
	ImGui::SliderFloat("max EV in the cab by day", &Global.gfx_postfx_autoexposure_max_cab, 0.f, 4.f);
	ImGui::SliderFloat("adaptation to dark (1/s)", &Global.gfx_postfx_autoexposure_speed_dark, 0.05f, 10.f);
	ImGui::SliderFloat("adaptation to bright (1/s)", &Global.gfx_postfx_autoexposure_speed_bright, 0.05f, 10.f);
	ImGui::SliderFloat("compensation EV", &Global.gfx_postfx_autoexposure_compensation, -4.f, 4.f);
	ImGui::Text("average luminance %.3f, adapted %.3f, exposure %.3f (%+.2f EV)", Readback[2], Readback[0], Readback[1], std::log2(std::max(Readback[1], 1e-6f)));
	ImGui::Checkbox("bloom (gfx.postfx.bloom)", &Global.gfx_postfx_bloom);
	ImGui::SliderFloat("bloom intensity", &Global.gfx_postfx_bloom_intensity, 0.f, 0.5f);
	ImGui::SliderFloat("bloom threshold", &Global.gfx_postfx_bloom_threshold, 0.f, 10.f);
	ImGui::Checkbox("white balance (gfx.postfx.whitebalance)", &Global.gfx_postfx_whitebalance);
	ImGui::SliderFloat("temperature K, 6500 = none, more = warmer", &Global.gfx_postfx_whitebalance_temperature, 4000.f, 12000.f, "%.0f");
	ImGui::SliderFloat("tint, + magenta, - green", &Global.gfx_postfx_whitebalance_tint, -1.f, 1.f, "%.2f");
}

} // namespace gfx
