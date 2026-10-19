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

#include <cstring>

#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/utilities.h"

namespace
{

// the nearest distance the first cascade may start from and the least one the last reaches, metres: the cascades don't
// get any closer to the camera than this, however close the pixels are
float const SPLIT_NEAR_MIN{0.5f};
float const SPLIT_FAR_MIN{16.f};
// how far past its distances a cascade takes the pixels, this many metres and this share of the distance: the camera moves
// in the frame or two before the cascades are drawn from the measurement, and a pixel which came into another cascade since
// has to be within its map
float const MARGIN_METRES{1.f};
float const MARGIN_SHARE{0.05f};

} // namespace

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

	// numbers start from 1, 0 is no measurement
	if (++m_measurement == 0)
		++m_measurement;
	gl::shadow_fit_data data{};
	data.measurement = m_measurement;
	data.depthmin = std::numeric_limits<float>::infinity();
	for (std::size_t cascade = 0; cascade < gl::MAX_CASCADES; ++cascade)
	{
		for (std::size_t axis = 0; axis < 4; ++axis)
		{
			data.bounds[cascade * 8 + axis] = 0xFFFFFFFFu;
			data.bounds[cascade * 8 + 4 + axis] = 0u;
		}
	}
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

	auto const farmin{std::min(SPLIT_FAR_MIN, View.range)};
	glProgramUniformMatrix4fv(*m_boundsprogram, 0, 1, GL_FALSE, glm::value_ptr(inverseprojection));
	glProgramUniformMatrix4fv(*m_boundsprogram, 4, 1, GL_FALSE, glm::value_ptr(viewtolight));
	glProgramUniform4f(*m_boundsprogram, 8, View.range, std::min(SPLIT_NEAR_MIN, farmin * 0.25f), farmin, std::clamp(Global.gfx_shadow_sdsm_split, 0.f, 1.f));
	glProgramUniform4f(*m_boundsprogram, 9, MARGIN_METRES, MARGIN_SHARE, 0.f, 0.f);
	m_boundsprogram->dispatch(groupsx, groupsy);
	// for the copy of the read back
	glMemoryBarrier(GL_BUFFER_UPDATE_BARRIER_BIT);

	glBindTextureUnit(0, 0);
	opengl_texture::reset_unit_cache();

	m_timer.end();

	m_readback->push(*m_buffer);
	m_pending[m_measurement % PENDING] = pending{m_measurement, View, Frame};
}

opengl46_shadow_fit::result const *opengl46_shadow_fit::latest()
{
	gl::shadow_fit_data data;
	if (m_readback && m_readback->pop(&data))
	{
		auto const &source{m_pending[data.measurement % PENDING]};
		if ((data.measurement != 0) && (source.measurement == data.measurement))
			m_latest = result{data, source.source, source.frame};
	}
	return m_latest ? &(*m_latest) : nullptr;
}

float opengl46_shadow_fit::unorder(std::uint32_t const Value)
{
	auto const bits{((Value & 0x80000000u) != 0u) ? (Value & 0x7FFFFFFFu) : ~Value};
	float result;
	std::memcpy(&result, &bits, sizeof(result));
	return result;
}

std::string opengl46_shadow_fit::info_times()
{
	std::string text;
	if (auto const time = m_timer.result())
		text += " sun shadows fitted to the depth: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}
