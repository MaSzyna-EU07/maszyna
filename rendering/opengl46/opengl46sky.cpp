/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46sky.h"

#include <cstring>

#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"


opengl46_sky::~opengl46_sky()
{
	GLuint const tables[]{m_transmittance_table, m_multiscattering_table, m_skyview_table};
	for (auto const table : tables)
	{
		if (table != 0)
			glDeleteTextures(1, &table);
	}
}

GLuint opengl46_sky::create_table(GLsizei const Width, GLsizei const Height)
{
	GLuint table{0};
	glCreateTextures(GL_TEXTURE_2D, 1, &table);
	glTextureStorage2D(table, 1, GL_RGBA16F, Width, Height);
	glTextureParameteri(table, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTextureParameteri(table, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTextureParameteri(table, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(table, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	float const black[4]{0.f, 0.f, 0.f, 0.f};
	glClearTexImage(table, 0, GL_RGBA, GL_FLOAT, black);
	return table;
}

void opengl46_sky::init()
{
	auto const make_compute = [](std::string const &Name) {
		gl::shader shader(Name);
		return std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
	};
	m_transmittance_program = make_compute("atmosphere_transmittance.comp");
	m_multiscattering_program = make_compute("atmosphere_multiscattering.comp");
	m_skyview_program = make_compute("atmosphere_skyview.comp");
	m_ambient_program = make_compute("atmosphere_ambient.comp");
	m_prefilter_program = make_compute("envmap_prefilter.comp");

	m_sky_ssbo = std::make_unique<gl::ssbo>(sizeof(gl::sky_data), gl::SKY_SSBO_BINDING);
	m_sky_ssbo->update(m_sky_data);
	m_atmosphere_ubo = std::make_unique<gl::ubo>(sizeof(gl::atmosphere_ubs), gl::ATMOSPHERE_UBO_BINDING);
	m_atmosphere_result_ssbo = std::make_unique<gl::ssbo>(sizeof(gl::atmosphere_result), gl::ATMOSPHERE_RESULT_SSBO_BINDING);

	m_transmittance_table = create_table(gl::ATMOSPHERE_TRANSMITTANCE_WIDTH, gl::ATMOSPHERE_TRANSMITTANCE_HEIGHT);
	m_multiscattering_table = create_table(gl::ATMOSPHERE_MULTISCATTERING_SIZE, gl::ATMOSPHERE_MULTISCATTERING_SIZE);
	m_skyview_table = create_table(gl::ATMOSPHERE_SKYVIEW_WIDTH, gl::ATMOSPHERE_SKYVIEW_HEIGHT);

	// a valid atmosphere for the sky shader from the start, the tables stay black until the first update
	m_atmosphere_ubo->update(m_light.atmosphere());
}

void opengl46_sky::bind()
{
	m_sky_ssbo->bind_base();
	m_atmosphere_ubo->bind_uniform();
	m_atmosphere_result_ssbo->bind_base();
}

void opengl46_sky::update_atmosphere(glm::vec3 const &Sundirection)
{
	bool mediumchanged{false};
	if (false == m_light.atmosphere_changed(Sundirection, mediumchanged))
		return;

	m_atmosphere_ubo->update(m_light.atmosphere());
	m_atmosphere_ubo->bind_uniform();
	m_atmosphere_result_ssbo->bind_base();

	m_atmosphere_timer.begin();
	if (mediumchanged)
	{
		glBindImageTexture(gl::ATMOSPHERE_TRANSMITTANCE_UNIT, m_transmittance_table, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		m_transmittance_program->dispatch((gl::ATMOSPHERE_TRANSMITTANCE_WIDTH + 7) / 8, (gl::ATMOSPHERE_TRANSMITTANCE_HEIGHT + 7) / 8);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

		glBindTextureUnit(gl::ATMOSPHERE_TRANSMITTANCE_UNIT, m_transmittance_table);
		glBindImageTexture(gl::ATMOSPHERE_MULTISCATTERING_UNIT, m_multiscattering_table, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		m_multiscattering_program->dispatch(gl::ATMOSPHERE_MULTISCATTERING_SIZE, gl::ATMOSPHERE_MULTISCATTERING_SIZE);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	}

	glBindTextureUnit(gl::ATMOSPHERE_TRANSMITTANCE_UNIT, m_transmittance_table);
	glBindTextureUnit(gl::ATMOSPHERE_MULTISCATTERING_UNIT, m_multiscattering_table);
	glBindImageTexture(gl::ATMOSPHERE_SKYVIEW_UNIT, m_skyview_table, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
	m_skyview_program->dispatch((gl::ATMOSPHERE_SKYVIEW_WIDTH + 7) / 8, (gl::ATMOSPHERE_SKYVIEW_HEIGHT + 7) / 8);
	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);

	glBindTextureUnit(gl::ATMOSPHERE_SKYVIEW_UNIT, m_skyview_table);
	m_ambient_program->dispatch(1);
	glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT | GL_SHADER_STORAGE_BARRIER_BIT);
	m_atmosphere_timer.end();

	// the units the passes used are below the ones of the texture cache, which is told they changed
	for (GLuint unit : {gl::ATMOSPHERE_TRANSMITTANCE_UNIT, gl::ATMOSPHERE_MULTISCATTERING_UNIT, gl::ATMOSPHERE_SKYVIEW_UNIT})
		glBindTextureUnit(unit, 0);
	opengl_texture::reset_unit_cache();

	m_result_readback.push(*m_atmosphere_result_ssbo);
}

void opengl46_sky::update(opengl46_light &Sunlight, glm::vec3 &Fogcolor, CSkyDome const &Skydome, glm::vec3 const &Sundirection, float const Sunangle)
{
	m_light.begin_update();
	// atmosphere
	if (Global.gfx_atmosphere)
	{
		update_atmosphere(glm::normalize(Sundirection));
		gl::atmosphere_result result;
		if (m_result_readback.pop(&result))
			m_light.set_result(result);
	}
	m_light.end_update(Sunlight, Fogcolor, Skydome, Sundirection, Sunangle);

	// directional part of the ambient light for the shading
	auto const &skydata{m_light.sky_data()};
	if (!m_sky_data_uploaded || std::memcmp(&skydata, &m_sky_data, sizeof(skydata)) != 0)
	{
		m_sky_data = skydata;
		m_sky_ssbo->update(m_sky_data);
		m_sky_data_uploaded = true;
	}
}

void opengl46_sky::setup_sky(gl::model_ubs &Model) const
{
	glBindTextureUnit(gl::SKY_VIEW_TEX, m_skyview_table);
	m_light.sky_params(Model.param[0], Model.param[1]);
}

glm::vec3 opengl46_sky::sun_disc_color(glm::vec3 const &Oldcolor) const
{
	return m_light.sun_disc_color(Oldcolor);
}

void opengl46_sky::prefilter(gl::cubemap const &Source)
{
	if (!m_prefiltered)
	{
		m_prefiltered = std::make_unique<gl::cubemap>();
		m_prefiltered->alloc_levels(GL_RGBA16F, static_cast<int>(gl::ENVMAP_GGX_SIZE), static_cast<int>(gl::ENVMAP_GGX_LEVELS));
	}

	m_prefilter_timer.begin();
	Source.bind(GL_TEXTURE0 + gl::ENV_TEX);
	glActiveTexture(GL_TEXTURE0);
	for (std::size_t level = 0; level < gl::ENVMAP_GGX_LEVELS; ++level)
	{
		auto const size{static_cast<int>(gl::ENVMAP_GGX_SIZE >> level)};
		auto const roughness{gl::ENVMAP_GGX_ROUGHNESS_MIN + (1.f - gl::ENVMAP_GGX_ROUGHNESS_MIN) * static_cast<float>(level) / static_cast<float>(gl::ENVMAP_GGX_LEVELS - 1)};
		glBindImageTexture(0, *m_prefiltered, static_cast<GLint>(level), GL_TRUE, 0, GL_WRITE_ONLY, GL_RGBA16F);
		glProgramUniform1f(*m_prefilter_program, 0, roughness);
		glProgramUniform1i(*m_prefilter_program, 1, size);
		glProgramUniform1f(*m_prefilter_program, 2, static_cast<float>(gl::ENVMAP_SIZE));
		glProgramUniform1i(*m_prefilter_program, 3, 64);
		auto const groups{static_cast<GLuint>((size + gl::ENVMAP_GGX_GROUP_SIZE - 1) / gl::ENVMAP_GGX_GROUP_SIZE)};
		m_prefilter_program->dispatch(groups, groups, 6);
	}
	glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
	m_prefilter_timer.end();
	glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_RGBA16F);
	m_prefiltered_ready = true;
}

std::string opengl46_sky::info_times()
{
	std::string text;
	if (auto const time = m_atmosphere_timer.result())
		text += " atmosphere: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	if (auto const time = m_prefilter_timer.result())
		text += " envmap ggx: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}

void opengl46_sky::draw_debug_ui()
{
	m_light.draw_debug_ui();
}
