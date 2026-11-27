/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46shadowfit.h"

#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

opengl46_shadow_fit::~opengl46_shadow_fit()
{
	release();
}

void opengl46_shadow_fit::init()
{
	auto const make_compute = [](std::string const &Name) {
		gl::shader shader(Name);
		return std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{shader});
	};
	m_depthprogram = make_compute("shadow_fit_depth.comp");
	m_boundsprogram = make_compute("shadow_fit_bounds.comp");
	m_buffer = std::make_unique<gl::ssbo>(sizeof(gl::shadow_fit_data), gl::SHADOW_FIT_SSBO_BINDING);
	m_readback = std::make_unique<gl::readback>(sizeof(gl::shadow_fit_data));
}

void opengl46_shadow_fit::release()
{
	if (m_depthfb != 0)
		glDeleteFramebuffers(1, &m_depthfb);
	if (m_depth != 0)
		glDeleteTextures(1, &m_depth);
	m_depthfb = 0;
	m_depth = 0;
	m_depthsize = glm::ivec2{0};
}

void opengl46_shadow_fit::allocate(int const Width, int const Height)
{
	release();
	m_depthsize = glm::ivec2{Width, Height};
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
}

void opengl46_shadow_fit::measure(GLuint const Depth, GLuint const Source, int const Width, int const Height, view const &View, unsigned int const Frame)
{
	if ((Width <= 0) || (Height <= 0) || (View.range <= 0.f))
		return;

	m_timer.begin();

	auto depth{Depth};
	if (depth == 0)
	{
		if (m_depthsize != glm::ivec2{Width, Height})
			allocate(Width, Height);
		// the scissor test would limit the blit
		auto const scissor{glIsEnabled(GL_SCISSOR_TEST) == GL_TRUE};
		if (scissor)
			glDisable(GL_SCISSOR_TEST);
		glBlitNamedFramebuffer(Source, m_depthfb, 0, 0, Width, Height, 0, 0, Width, Height, GL_DEPTH_BUFFER_BIT, GL_NEAREST);
		if (scissor)
			glEnable(GL_SCISSOR_TEST);
		depth = m_depth;
	}

	auto const data{begin_measurement(View, Frame)};
	m_buffer->update(data);
	m_buffer->bind_base();

	auto const inverseprojection{glm::inverse(View.projection)};
	auto const viewtolight{glm::mat4{View.light * glm::inverse(View.rotation)}};
	auto const groupsx{(static_cast<GLuint>(Width) + gl::SHADOW_FIT_GROUP_SIZE - 1) / gl::SHADOW_FIT_GROUP_SIZE};
	auto const groupsy{(static_cast<GLuint>(Height) + gl::SHADOW_FIT_GROUP_SIZE - 1) / gl::SHADOW_FIT_GROUP_SIZE};
	glBindTextureUnit(0, depth);

	glProgramUniformMatrix4fv(*m_depthprogram, 0, 1, GL_FALSE, glm::value_ptr(inverseprojection));
	m_depthprogram->dispatch(groupsx, groupsy);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);

	auto const bounds{bounds_params(View)};
	auto const margins{margin_params()};
	glProgramUniformMatrix4fv(*m_boundsprogram, 0, 1, GL_FALSE, glm::value_ptr(inverseprojection));
	glProgramUniformMatrix4fv(*m_boundsprogram, 4, 1, GL_FALSE, glm::value_ptr(viewtolight));
	glProgramUniform4f(*m_boundsprogram, 8, bounds.x, bounds.y, bounds.z, bounds.w);
	glProgramUniform4f(*m_boundsprogram, 9, margins.x, margins.y, margins.z, margins.w);
	m_boundsprogram->dispatch(groupsx, groupsy);
	// for the copy of the read back
	glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);

	glBindTextureUnit(0, 0);
	opengl_texture::reset_unit_cache();

	m_timer.end();

	m_readback->push(*m_buffer);
}

opengl46_shadow_fit::result const *opengl46_shadow_fit::latest()
{
	gl::shadow_fit_data data;
	if (m_readback && m_readback->pop(&data))
		arrived(data);
	return gfx::shadow_fit::latest();
}

std::string opengl46_shadow_fit::info_times()
{
	std::string text;
	if (auto const time = m_timer.result())
		text += " sun shadows fitted to the depth: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}
