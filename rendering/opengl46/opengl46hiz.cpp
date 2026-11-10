/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46hiz.h"

#include "gl/ubo.h"
#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

opengl46_hiz::~opengl46_hiz()
{
	release();
}

void opengl46_hiz::init()
{
	gl::shader shader("hiz_build.comp");
	m_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
}

void opengl46_hiz::release()
{
	if (m_depthfb != 0)
		glDeleteFramebuffers(1, &m_depthfb);
	if (m_depth != 0)
		glDeleteTextures(1, &m_depth);
	if (m_pyramid != 0)
		glDeleteTextures(1, &m_pyramid);
	m_depthfb = 0;
	m_depth = 0;
	m_pyramid = 0;
	m_sourcesize = glm::ivec2{0};
	m_size = glm::ivec2{0};
	m_levels = 0;
}

void opengl46_hiz::allocate(int const Width, int const Height)
{
	release();
	m_sourcesize = glm::ivec2{Width, Height};

	// the blit of depth needs the format of the source
	glCreateTextures(GL_TEXTURE_2D, 1, &m_depth);
	glTextureStorage2D(m_depth, 1, Global.gfx_format_depth, Width, Height);
	glTextureParameteri(m_depth, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTextureParameteri(m_depth, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTextureParameteri(m_depth, GL_TEXTURE_COMPARE_MODE, GL_NONE);
	glCreateFramebuffers(1, &m_depthfb);
	glNamedFramebufferTexture(m_depthfb, GL_DEPTH_ATTACHMENT, m_depth, 0);
	glNamedFramebufferDrawBuffer(m_depthfb, GL_NONE);
	glNamedFramebufferReadBuffer(m_depthfb, GL_NONE);

	m_size = glm::ivec2{(Width + 1) / 2, (Height + 1) / 2};
	m_levels = 1;
	while ((std::max(m_size.x, m_size.y) >> m_levels) > 0)
		++m_levels;
	glCreateTextures(GL_TEXTURE_2D, 1, &m_pyramid);
	glTextureStorage2D(m_pyramid, m_levels, GL_R32F, m_size.x, m_size.y);
	glTextureParameteri(m_pyramid, GL_TEXTURE_MIN_FILTER, GL_NEAREST_MIPMAP_NEAREST);
	glTextureParameteri(m_pyramid, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTextureParameteri(m_pyramid, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(m_pyramid, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

void opengl46_hiz::build(GLuint const Source, int const Width, int const Height, bool const Second)
{
	if ((Width <= 0) || (Height <= 0))
		return;
	if (m_sourcesize != glm::ivec2{Width, Height})
		allocate(Width, Height);

	auto &timer{Second ? m_secondtimer : m_timer};
	timer.begin();

	// resolve of the multisampled depth; the scissor test would limit the blit
	auto const scissor{glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE};
	if (scissor)
		glDisable(GL_SCISSOR_TEST);
	glBlitNamedFramebuffer(Source, m_depthfb, 0, 0, Width, Height, 0, 0, Width, Height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
	if (scissor)
		glEnable(GL_SCISSOR_TEST);

	auto source{m_depth};
	auto sourcelevel{0};
	auto sourcesize{m_sourcesize};
	for (int level = 0; level < m_levels; ++level)
	{
		auto const targetsize{glm::max(m_size >> level, glm::ivec2{1})};
		glBindTextureUnit(0, source);
		glBindImageTexture(0, m_pyramid, level, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
		glProgramUniform4i(*m_program, 0, sourcesize.x, sourcesize.y, targetsize.x, targetsize.y);
		glProgramUniform4i(*m_program, 1, sourcelevel, 0, 0, 0);
		m_program->dispatch((targetsize.x + gl::HIZ_GROUP_SIZE - 1) / gl::HIZ_GROUP_SIZE, (targetsize.y + gl::HIZ_GROUP_SIZE - 1) / gl::HIZ_GROUP_SIZE);
		glMemoryBarrier(GL_TEXTURE_FETCH_BARRIER_BIT | GL_SHADER_IMAGE_ACCESS_BARRIER_BIT);
		source = m_pyramid;
		sourcelevel = level;
		sourcesize = targetsize;
	}
	glBindImageTexture(0, 0, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
	glBindTextureUnit(0, 0);
	opengl_texture::reset_unit_cache();

	timer.end();
}

std::string opengl46_hiz::info_times(bool const Second)
{
	std::string text;
	if (auto const time = m_timer.result())
	{
		text += " occlusion depth: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms";
		auto const second{m_secondtimer.result()};
		if (Second && second)
			text += " (+" + to_string(static_cast<double>(*second) / 1000000.0, 3) + " ms with the instances drawn first)";
		text += "\n";
	}
	return text;
}
