/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46exposure.h"

#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

namespace
{

// luminance range of the histogram, log2: 2^-12 .. 2^6. darker pixels count as black, brighter ones as the brightest
float const HISTOGRAM_LOG_MIN{-12.f};
float const HISTOGRAM_LOG_MAX{6.f};
// the bloom chain goes down this many levels at most, and not below this size
std::size_t const BLOOM_LEVELS{6};
int const BLOOM_MIN_SIZE{8};
// format of the levels: no alpha, no negative values, half the memory of rgba16f
GLint const BLOOM_FORMAT{GL_R11F_G11F_B10F};
// soft knee of the threshold, a fraction of it
float const BLOOM_KNEE{0.5f};

} // namespace

void opengl46_exposure::init()
{
	auto const make_compute = [](std::string const &Name) {
		gl::shader shader(Name);
		return std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
	};
	m_histogram_program = make_compute("exposure_histogram.comp");
	m_average_program = make_compute("exposure_average.comp");
	m_bloom_down = std::make_unique<gl::postfx>("bloom_down");
	m_bloom_up = std::make_unique<gl::postfx>("bloom_up");

	m_exposure_ssbo = std::make_unique<gl::ssbo>(sizeof(gl::exposure_data), gl::EXPOSURE_SSBO_BINDING);
	gl::exposure_data data{};
	data.adapted_luminance = Global.gfx_postfx_autoexposure_key;
	data.exposure = 1.f;
	data.average_luminance = Global.gfx_postfx_autoexposure_key;
	data.valid = 0;
	m_exposure_ssbo->update(data);
}

void opengl46_exposure::bind()
{
	m_exposure_ssbo->bind_base();
}

std::unique_ptr<opengl46_exposure::bloom_chain> opengl46_exposure::make_bloom_chain(int const Width, int const Height)
{
	auto chain{std::make_unique<bloom_chain>()};
	glm::ivec2 size{Width, Height};
	for (std::size_t level = 0; level < BLOOM_LEVELS; ++level)
	{
		size = glm::max(size / 2, glm::ivec2{1});
		if (level > 0 && std::min(size.x, size.y) < BLOOM_MIN_SIZE)
			break;
		chain->sizes.push_back(size);
		auto &texture{chain->down.emplace_back(std::make_unique<opengl_texture>())};
		texture->alloc_rendertarget(BLOOM_FORMAT, GL_RGB, size.x, size.y, 1, 1, GL_CLAMP_TO_EDGE);
		auto &framebuffer{chain->down_fb.emplace_back(std::make_unique<gl::framebuffer>())};
		framebuffer->attach(*texture, GL_COLOR_ATTACHMENT0);
		if (!framebuffer->is_complete())
			return nullptr;
	}
	for (std::size_t level = 0; level + 1 < chain->sizes.size(); ++level)
	{
		auto const &size{chain->sizes[level]};
		auto &texture{chain->up.emplace_back(std::make_unique<opengl_texture>())};
		texture->alloc_rendertarget(BLOOM_FORMAT, GL_RGB, size.x, size.y, 1, 1, GL_CLAMP_TO_EDGE);
		auto &framebuffer{chain->up_fb.emplace_back(std::make_unique<gl::framebuffer>())};
		framebuffer->attach(*texture, GL_COLOR_ATTACHMENT0);
		if (!framebuffer->is_complete())
			return nullptr;
	}
	return chain->sizes.empty() ? nullptr : std::move(chain);
}

void opengl46_exposure::measure(opengl_texture &Image, double const Deltatime)
{
	float data[4];
	if (m_readback.pop(data))
		std::copy(std::begin(data), std::end(data), std::begin(m_readback_data));

	m_exposure_timer.begin();
	m_exposure_ssbo->bind_base();
	Image.bind(0);
	auto const bins{static_cast<float>(gl::EXPOSURE_HISTOGRAM_BINS - 2)};
	glProgramUniform2f(*m_histogram_program, 0, HISTOGRAM_LOG_MIN, bins / (HISTOGRAM_LOG_MAX - HISTOGRAM_LOG_MIN));
	auto const groupsize{2 * static_cast<int>(gl::EXPOSURE_HISTOGRAM_GROUP_SIZE)};
	m_histogram_program->dispatch((Image.get_width() + groupsize - 1) / groupsize, (Image.get_height() + groupsize - 1) / groupsize);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	glProgramUniform2f(*m_average_program, 0, HISTOGRAM_LOG_MIN, (HISTOGRAM_LOG_MAX - HISTOGRAM_LOG_MIN) / bins);
	glProgramUniform4f(*m_average_program, 1,
	    static_cast<float>(std::clamp(Deltatime, 0.0, 1.0)),
	    Global.gfx_postfx_autoexposure_speed_dark,
	    Global.gfx_postfx_autoexposure_speed_bright,
	    Global.gfx_postfx_autoexposure_key);
	glProgramUniform2f(*m_average_program, 2, std::exp2(Global.gfx_postfx_autoexposure_min), std::exp2(Global.gfx_postfx_autoexposure_max));
	m_average_program->dispatch(1);
	// read by the tonemapping, and copied for the debug window
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT | GL_BUFFER_UPDATE_BARRIER_BIT);
	m_exposure_timer.end();

	m_readback.push(*m_exposure_ssbo, offsetof(gl::exposure_data, adapted_luminance));
	opengl_texture::reset_unit_cache();
}

opengl_texture *opengl46_exposure::bloom(bloom_chain &Chain, opengl_texture &Image, gl::stream_ubo &Modelubo, gl::model_ubs &Model)
{
	m_bloom_timer.begin();
	// down, the first step from the image itself
	for (std::size_t level = 0; level < Chain.sizes.size(); ++level)
	{
		Model.param[0] = glm::vec4{(level == 0 ? 1.f : 0.f), Global.gfx_postfx_bloom_threshold, BLOOM_KNEE, 0.f};
		Modelubo.update(Model);
		glViewport(0, 0, Chain.sizes[level].x, Chain.sizes[level].y);
		m_bloom_down->apply(level == 0 ? Image : *Chain.down[level - 1], Chain.down_fb[level].get());
	}
	// up, from the lowest level
	for (std::size_t level = Chain.up.size(); level-- > 0;)
	{
		auto &lower{(level + 1 < Chain.up.size()) ? *Chain.up[level + 1] : *Chain.down[level + 1]};
		glViewport(0, 0, Chain.sizes[level].x, Chain.sizes[level].y);
		m_bloom_up->apply({&lower, Chain.down[level].get()}, Chain.up_fb[level].get());
	}
	m_bloom_timer.end();
	opengl_texture::reset_unit_cache();
	return Chain.up.empty() ? Chain.down.front().get() : Chain.up.front().get();
}

glm::vec4 opengl46_exposure::tonemap_params(std::size_t const Bloomlevels) const
{
	auto const bloom{(Global.gfx_postfx_bloom && Bloomlevels > 0) ? Global.gfx_postfx_bloom_intensity : 0.f};
	return glm::vec4{
	    bloom,
	    Global.gfx_postfx_autoexposure ? 1.f : 0.f,
	    std::exp2(Global.gfx_postfx_autoexposure_compensation),
	    Bloomlevels > 0 ? 1.f / static_cast<float>(Bloomlevels) : 0.f};
}

std::string opengl46_exposure::info_times()
{
	std::string text;
	if (auto const time = m_exposure_timer.result())
		text += " exposure: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	if (auto const time = m_bloom_timer.result())
		text += " bloom: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}

void opengl46_exposure::draw_debug_ui()
{
	ImGui::Checkbox("automatic exposure (gfx.postfx.autoexposure)", &Global.gfx_postfx_autoexposure);
	ImGui::SliderFloat("key", &Global.gfx_postfx_autoexposure_key, 0.02f, 2.f, "%.3f", ImGuiSliderFlags_Logarithmic);
	ImGui::SliderFloat("min EV", &Global.gfx_postfx_autoexposure_min, -4.f, 0.f);
	ImGui::SliderFloat("max EV", &Global.gfx_postfx_autoexposure_max, 0.f, 4.f);
	ImGui::SliderFloat("adaptation to dark (1/s)", &Global.gfx_postfx_autoexposure_speed_dark, 0.05f, 10.f);
	ImGui::SliderFloat("adaptation to bright (1/s)", &Global.gfx_postfx_autoexposure_speed_bright, 0.05f, 10.f);
	ImGui::SliderFloat("compensation EV", &Global.gfx_postfx_autoexposure_compensation, -4.f, 4.f);
	ImGui::Text("average luminance %.3f, adapted %.3f, exposure %.3f (%+.2f EV)", m_readback_data[2], m_readback_data[0], m_readback_data[1], std::log2(std::max(m_readback_data[1], 1e-6f)));
	ImGui::Checkbox("bloom (gfx.postfx.bloom)", &Global.gfx_postfx_bloom);
	ImGui::SliderFloat("bloom intensity", &Global.gfx_postfx_bloom_intensity, 0.f, 0.5f);
	ImGui::SliderFloat("bloom threshold", &Global.gfx_postfx_bloom_threshold, 0.f, 10.f);
}
