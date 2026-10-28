/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl46/opengl46cabprobe.h"

#include "gl/framebuffer.h"
#include "gl/ubo.h"
#include "model/Texture.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"
#include "utilities/utilities.h"

namespace
{

// uniform location of cabprobe_sh.comp: the level of the probe read and its size
GLint const SH_LEVEL_LOCATION{0};

// cube map of Size x Size faces with its mipmaps
GLuint make_cube(GLenum const Format, int const Size, bool const Mipmaps)
{
	GLuint texture{0};
	glCreateTextures(GL_TEXTURE_CUBE_MAP, 1, &texture);
	auto const levels{Mipmaps ? static_cast<GLsizei>(std::floor(std::log2(static_cast<float>(std::max(Size, 1))))) + 1 : 1};
	glTextureStorage2D(texture, levels, Format, Size, Size);
	glTextureParameteri(texture, GL_TEXTURE_MIN_FILTER, Mipmaps ? GL_LINEAR_MIPMAP_LINEAR : GL_NEAREST);
	glTextureParameteri(texture, GL_TEXTURE_MAG_FILTER, Mipmaps ? GL_LINEAR : GL_NEAREST);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTextureParameteri(texture, GL_TEXTURE_WRAP_R, GL_CLAMP_TO_EDGE);
	return texture;
}

} // namespace

opengl46_cab_probe::~opengl46_cab_probe()
{
	gpu_release();
	if (m_empty != 0)
		glDeleteTextures(1, &m_empty);
	if (m_white != 0)
		glDeleteTextures(1, &m_white);
}

void opengl46_cab_probe::init()
{
	gl::shader vertex("cabprobe.vert");
	gl::shader geometry("cabprobe.geom");
	{
		gl::shader fragment("cabprobe.frag");
		m_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, geometry, fragment});
	}
	{
		gl::shader fragment("cabprobe_alpha.frag");
		m_alpha_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{vertex, geometry, fragment});
	}
	{
		gl::shader compute("cabprobe_sh.comp");
		m_sh_program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<const gl::shader>>{compute});
	}
	m_sh = std::make_unique<gl::ssbo>(static_cast<GLsizeiptr>(sizeof(gl::cab_probe_data)), gl::CAB_PROBE_SSBO_BINDING);
	m_sh->update(gl::cab_probe_data{});

	// a sampler with no texture to read from is undefined, also where the shading doesn't read it
	m_empty = make_cube(GL_RGBA8, 1, false);
	std::uint8_t const nothing[4]{0, 0, 0, 0};
	glClearTexImage(m_empty, 0, GL_RGBA, GL_UNSIGNED_BYTE, nothing);
	glCreateTextures(GL_TEXTURE_2D, 1, &m_white);
	glTextureStorage2D(m_white, 1, GL_RGBA8, 1, 1);
	std::uint8_t const white[4]{255, 255, 255, 255};
	glClearTexImage(m_white, 0, GL_RGBA, GL_UNSIGNED_BYTE, white);
}

void opengl46_cab_probe::bind()
{
	glBindTextureUnit(gl::CAB_PROBE_TEX, (m_light != 0) ? m_light : m_empty);
	glBindTextureUnit(gl::CAB_PROBE_ALBEDO_TEX, (m_albedo != 0) ? m_albedo : m_empty);
	glBindTextureUnit(gl::CAB_PROBE_MASK_TEX, (m_mask != 0) ? m_mask : m_empty);
	if (m_sh)
		m_sh->bind_base();
}

bool opengl46_cab_probe::gpu_allocate(int const Size)
{
	m_light = make_cube(GL_RGBA16F, Size, true);
	m_albedo = make_cube(GL_RGBA8, Size, true);
	m_depth = make_cube(GL_DEPTH_COMPONENT32F, Size, false);
	// the mask: the level of the albedo of at most CAB_PROBE_MASK_SIZE pixels a face, as mask_size() has it once the size is set
	auto masksize{Size};
	while (masksize > static_cast<int>(gl::CAB_PROBE_MASK_SIZE))
		masksize >>= 1;
	m_mask = make_cube(GL_RGBA8, std::max(masksize, 1), false);
	glTextureParameteri(m_mask, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTextureParameteri(m_mask, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	// all the faces at once, the geometry shader picks the face
	glCreateFramebuffers(1, &m_framebuffer);
	glNamedFramebufferTexture(m_framebuffer, GL_COLOR_ATTACHMENT0, m_light, 0);
	glNamedFramebufferTexture(m_framebuffer, GL_COLOR_ATTACHMENT1, m_albedo, 0);
	glNamedFramebufferTexture(m_framebuffer, GL_DEPTH_ATTACHMENT, m_depth, 0);
	GLenum const buffers[2]{GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
	glNamedFramebufferDrawBuffers(m_framebuffer, 2, buffers);
	glNamedFramebufferReadBuffer(m_framebuffer, GL_NONE);
	return (glCheckNamedFramebufferStatus(m_framebuffer, GL_FRAMEBUFFER) == GL_FRAMEBUFFER_COMPLETE);
}

void opengl46_cab_probe::gpu_release()
{
	if (m_framebuffer != 0)
		glDeleteFramebuffers(1, &m_framebuffer);
	for (auto *texture : {&m_light, &m_albedo, &m_depth, &m_mask})
	{
		if (*texture != 0)
			glDeleteTextures(1, texture);
		*texture = 0;
	}
	m_framebuffer = 0;
	if (m_sh)
		m_sh->update(gl::cab_probe_data{});
}

void opengl46_cab_probe::gpu_begin()
{
	m_timer.begin();
	// the framebuffer is bound past the cache of gl::framebuffer, which the end of the drawing resets
	gl::framebuffer::unbind();
	glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);
	glViewport(0, 0, m_size, m_size);
	float const nothing[4]{0.f, 0.f, 0.f, 0.f};
	glClearNamedFramebufferfv(m_framebuffer, GL_COLOR, 0, nothing);
	glClearNamedFramebufferfv(m_framebuffer, GL_COLOR, 1, nothing);
	float const farthest{0.f};
	glClearNamedFramebufferfv(m_framebuffer, GL_DEPTH, 0, &farthest);
	glDisable(GL_BLEND);
	glDepthMask(GL_TRUE);
	glEnable(GL_DEPTH_TEST);
	// one sided walls face the probe from either side
	glDisable(GL_CULL_FACE);
	glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
}

void opengl46_cab_probe::gpu_end()
{
	glEnable(GL_CULL_FACE);
	gl::framebuffer::unbind();
	glGenerateTextureMipmap(m_light);
	glGenerateTextureMipmap(m_albedo);
	// where the windows are, for the next drawing
	auto const masksize{mask_size()};
	glCopyImageSubData(m_albedo, GL_TEXTURE_CUBE_MAP, mask_level(), 0, 0, 0, m_mask, GL_TEXTURE_CUBE_MAP, 0, 0, 0, 0, masksize, masksize, 6);
	// the light the cab lights itself with again, from a small level of the light reflected
	glBindTextureUnit(0, m_light);
	glProgramUniform2i(*m_sh_program, SH_LEVEL_LOCATION, sh_level(), sh_size());
	m_sh->bind_base();
	m_sh_program->dispatch(1);
	glMemoryBarrier(GL_SHADER_STORAGE_BARRIER_BIT);
	opengl_texture::reset_unit_cache();
	m_timer.end();
}

std::string opengl46_cab_probe::info_times()
{
	std::string text;
	if (auto const time = m_timer.result())
		text += " cab probe: " + to_string(static_cast<double>(*time) / 1000000.0, 3) + " ms\n";
	return text;
}
