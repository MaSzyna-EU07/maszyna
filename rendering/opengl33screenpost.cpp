/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/opengl33screenpost.h"

#include "model/Texture.h"
#include "utilities/Logs.h"

#include <format>
#include <functional>

namespace
{

// the state of the context the passes change, put back after them
class saved_state
{
  public:
	saved_state()
	{
		glGetIntegerv(GL_VIEWPORT, m_viewport.data());
		glGetBooleanv(GL_COLOR_WRITEMASK, m_colormask.data());
		glGetBooleanv(GL_DEPTH_WRITEMASK, &m_depthmask);
		for (std::size_t idx = 0; idx < m_capabilities.size(); ++idx)
		{
			m_enabled[idx] = glIsEnabled(m_capabilities[idx]);
			glDisable(m_capabilities[idx]);
		}
		glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
		glDepthMask(GL_FALSE);
	}
	~saved_state()
	{
		for (std::size_t idx = 0; idx < m_capabilities.size(); ++idx)
		{
			if (m_enabled[idx] == GL_TRUE)
			{
				glEnable(m_capabilities[idx]);
			}
		}
		glColorMask(m_colormask[0], m_colormask[1], m_colormask[2], m_colormask[3]);
		glDepthMask(m_depthmask);
		glViewport(m_viewport[0], m_viewport[1], m_viewport[2], m_viewport[3]);
	}
	saved_state(saved_state const &) = delete;
	saved_state &operator=(saved_state const &) = delete;

  private:
	// GL_FRAMEBUFFER_SRGB off: the passes write the bytes of the image into the screen's sRGB texture as they are, as
	// update_from_memory puts them there
	static constexpr std::array<GLenum, 6> m_capabilities{GL_DEPTH_TEST, GL_BLEND, GL_CULL_FACE, GL_SCISSOR_TEST, GL_STENCIL_TEST, GL_FRAMEBUFFER_SRGB};
	std::array<GLboolean, 6> m_enabled{};
	std::array<GLint, 4> m_viewport{};
	std::array<GLboolean, 4> m_colormask{};
	GLboolean m_depthmask{GL_TRUE};
};

void unbind_all()
{
	gl::framebuffer::unbind();
	gl::program::unbind();
	gl::vao::unbind();
}

// the screen's texture made the size of the result, in the format update_from_memory gives it
void prepare_target(opengl_texture &Target, int const Width, int const Height)
{
	if (Target.id != opengl_texture::invalid_id && Target.data_width == static_cast<unsigned int>(Width) && Target.data_height == static_cast<unsigned int>(Height) &&
	    Target.data_format == GL_SRGB8_ALPHA8 && Target.data_components == GL_RGBA)
	{
		return;
	}
	Target.update_from_memory(Width, Height, nullptr);
}

} // namespace

opengl33_screen_post::~opengl33_screen_post()
{
	for (auto const *owned : {&m_source, &m_between[0], &m_between[1]})
	{
		if (owned->texture != 0)
		{
			glDeleteTextures(1, &owned->texture);
		}
	}
}

opengl33_screen_post::stage opengl33_screen_post::make_stage(gl::shader const &Vertex, std::string const &Fragment)
{
	gl::shader const fragment{Fragment};
	stage result;
	result.program = std::make_unique<gl::program>(std::vector<std::reference_wrapper<gl::shader const>>{Vertex, fragment});
	auto const program{static_cast<GLuint>(*result.program)};
	result.source_size = glGetUniformLocation(program, "source_size");
	result.target_size = glGetUniformLocation(program, "target_size");
	result.flip = glGetUniformLocation(program, "flip");
	result.mode = glGetUniformLocation(program, "mode");
	result.program->bind();
	glUniform1i(glGetUniformLocation(program, "source"), 0);
	return result;
}

bool opengl33_screen_post::setup()
{
	if (m_failed)
	{
		return false;
	}
	if (m_led.program != nullptr)
	{
		return true;
	}
	try
	{
		gl::shader const vertex{"quad.vert"};
		m_led = make_stage(vertex, "screen_led.frag");
		m_resize = make_stage(vertex, "screen_resize.frag");
	}
	catch (gl::shader_exception const &Error)
	{
		ErrorLog(std::format("Script screens: the shaders of the post-processing failed: {}", Error.what()));
		m_failed = true;
		return false;
	}
	m_vao = std::make_unique<gl::vao>();
	m_framebuffer = std::make_unique<gl::framebuffer>();
	WriteLog("Script screens: the post-processing of the images runs on the gpu");
	return true;
}

void opengl33_screen_post::store(surface &Surface, int const Width, int const Height, std::uint8_t const *Pixels)
{
	if (Surface.texture == 0)
	{
		glGenTextures(1, &Surface.texture);
		glBindTexture(GL_TEXTURE_2D, Surface.texture);
		// read with texelFetch: no mipmaps, nothing filtered
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	}
	else
	{
		glBindTexture(GL_TEXTURE_2D, Surface.texture);
	}
	if (Surface.width != Width || Surface.height != Height)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, Width, Height, 0, GL_RGBA, GL_UNSIGNED_BYTE, Pixels);
		Surface.width = Width;
		Surface.height = Height;
	}
	else if (Pixels != nullptr)
	{
		glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, Width, Height, GL_RGBA, GL_UNSIGNED_BYTE, Pixels);
	}
}

std::vector<opengl33_screen_post::pass> opengl33_screen_post::passes_of(std::span<script_screens::post_step const> const Steps, int Width, int Height) const
{
	std::vector<pass> result;
	for (auto const &step : Steps)
	{
		if (step.type == script_screens::post_step::kind::led)
		{
			Width = Width * 2 + 2;
			Height = Height * 2 + 2;
			result.emplace_back(&m_led, Width, Height, step.bloom ? 1 : 0);
			continue;
		}
		result.emplace_back(&m_resize, step.width, Height, 0);
		result.emplace_back(&m_resize, step.width, step.height, 1);
		Width = step.width;
		Height = step.height;
	}
	return result;
}

bool opengl33_screen_post::draw(pass const &Pass, surface const &Source, GLuint const Target, bool const Last) const
{
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, Target, 0);
	if (auto const status{glCheckFramebufferStatus(GL_FRAMEBUFFER)}; status != GL_FRAMEBUFFER_COMPLETE)
	{
		ErrorLog(std::format("Script screens: the post-processing can't draw into a texture, framebuffer status {:#x}", status));
		return false;
	}
	glViewport(0, 0, Pass.width, Pass.height);
	Pass.shader->program->bind();
	glUniform2i(Pass.shader->source_size, Source.width, Source.height);
	glUniform2i(Pass.shader->target_size, Pass.width, Pass.height);
	glUniform1i(Pass.shader->flip, Last ? 1 : 0);
	glUniform1i(Pass.shader->mode, Pass.mode);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Source.texture);
	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	return true;
}

bool opengl33_screen_post::apply(ITexture &Target, std::span<std::uint8_t const> const Pixels, int const Width, int const Height,
                                 std::span<script_screens::post_step const> const Steps)
{
	auto *target{dynamic_cast<opengl_texture *>(&Target)};
	if (target == nullptr || Steps.empty() || Pixels.size() < script_screens::image_size(Width, Height) || false == setup())
	{
		return false;
	}
	auto const passes{passes_of(Steps, Width, Height)};
	prepare_target(*target, passes.back().width, passes.back().height);

	auto succeeded{true};
	{
		saved_state const saved;
		// what the binding caches of gl:: hold may not be what's bound now: bound anew, and left unbound after the passes
		unbind_all();
		m_framebuffer->bind();
		m_vao->bind();
		store(m_source, Width, Height, Pixels.data());
		auto const *source{&m_source};
		for (std::size_t idx = 0; idx < passes.size() && succeeded; ++idx)
		{
			auto const last{idx + 1 == passes.size()};
			auto &between{m_between[idx % 2]};
			if (false == last)
			{
				store(between, passes[idx].width, passes[idx].height, nullptr);
			}
			succeeded = draw(passes[idx], *source, last ? target->id : between.texture, last);
			source = &between;
		}
		// the screen's texture isn't held by the framebuffer when it's made anew
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, 0, 0);
		unbind_all();
	}
	if (succeeded)
	{
		glBindTexture(GL_TEXTURE_2D, target->id);
		glGenerateMipmap(GL_TEXTURE_2D);
	}
	// the textures bound here went past the unit cache
	opengl_texture::reset_unit_cache();
	return succeeded;
}
