/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46skyocclusion.h"

#include "gl/ubo.h"

namespace
{

// a toroidal map read texel by texel (textureGather): nearest, repeated
GLuint make_map(GLenum const Format, int const Side)
{
	GLuint texture{0};
	glCreateTextures(GL_TEXTURE_2D, 1, &texture);
	glTextureStorage2D(texture, 1, Format, Side, Side);
	glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_T, GL_REPEAT);
	return texture;
}

} // namespace

opengl46_sky_occlusion::~opengl46_sky_occlusion()
{
	gpu_release();
	if (m_emptyheights != 0)
		glDeleteTextures(1, &m_emptyheights);
	if (m_emptyocclusion != 0)
		glDeleteTextures(1, &m_emptyocclusion);
}

void opengl46_sky_occlusion::init()
{
	// a sampler with no texture to read from is undefined, also where the shading doesn't read it
	m_emptyheights = make_map(GL_RGBA32F, 1);
	float const none[4]{-1.0e4f, 1.0e4f, 1.0e4f, -1.0e4f};
	glClearTexImage(m_emptyheights, 0, GL_RGBA, GL_FLOAT, none);
	m_emptyocclusion = make_map(GL_RGBA16F, 1);
	float const open[4]{1.f, 1.f, 1.f, -1000.f};
	glClearTexImage(m_emptyocclusion, 0, GL_RGBA, GL_FLOAT, open);
	bind();
}

void opengl46_sky_occlusion::bind() const
{
	glBindTextureUnit(gl::SKY_OCCLUSION_HEIGHT_TEX, (m_heights != 0) ? m_heights : m_emptyheights);
	glBindTextureUnit(gl::SKY_OCCLUSION_TEX, (m_occlusion != 0) ? m_occlusion : m_emptyocclusion);
}

bool opengl46_sky_occlusion::gpu_allocate(int const Side)
{
	gpu_release();
	m_heights = make_map(GL_RGBA32F, Side);
	m_occlusion = make_map(GL_RGBA16F, Side);
	if ((m_heights == 0) || (m_occlusion == 0))
		return false;
	bind();
	return true;
}

void opengl46_sky_occlusion::gpu_release()
{
	if (m_heights != 0)
		glDeleteTextures(1, &m_heights);
	if (m_occlusion != 0)
		glDeleteTextures(1, &m_occlusion);
	m_heights = 0;
	m_occlusion = 0;
	if (m_emptyheights != 0)
		bind();
}

void opengl46_sky_occlusion::gpu_upload(int const X, int const Y, int const Size, float const *Heights, std::uint16_t const *Occlusion)
{
	if ((m_heights == 0) || (m_occlusion == 0))
		return;
	// from the memory of the process: the buffer the texture streaming may have bound and the alignment are put back after
	GLint unpack{0}, alignment{4};
	glGetIntegerv(GL_PIXEL_UNPACK_BUFFER_BINDING, &unpack);
	glGetIntegerv(GL_UNPACK_ALIGNMENT, &alignment);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
	glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
	glTextureSubImage2D(m_heights, 0, X, Y, Size, Size, GL_RGBA, GL_FLOAT, Heights);
	glTextureSubImage2D(m_occlusion, 0, X, Y, Size, Size, GL_RGBA, GL_HALF_FLOAT, Occlusion);
	glBindBuffer(GL_PIXEL_UNPACK_BUFFER, static_cast<GLuint>(unpack));
	glPixelStorei(GL_UNPACK_ALIGNMENT, alignment);
}
