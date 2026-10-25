/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

#include "gl/ubo.h"
#include "gl/ssbo.h"
#include "gl/shader.h"
#include "gl/query.h"
#include "gl/readback.h"
#include "rendering/common/shadowfit.h"

// the cascades of the sun shadows fitted to the depth of the main view (gfx.shadow.sdsm): the passes and the buffers of
// gfx::shadow_fit, which tells what they do. the result comes back to the cpu a frame or two later without waiting
// (gl::readback), and opengl46_renderer::Shadow_Fit() fits the cascades of the frame to it
class opengl46_shadow_fit : public gfx::shadow_fit
{
  public:
	opengl46_shadow_fit() = default;
	~opengl46_shadow_fit();
	opengl46_shadow_fit(const opengl46_shadow_fit &) = delete;
	opengl46_shadow_fit &operator=(const opengl46_shadow_fit &) = delete;

	// creates the programs and the buffer. throws gl::shader_exception
	void init();
	// measures the depth of the view: Depth, its resolved depth texture of Width x Height pixels, or 0 to resolve the depth
	// attachment of Source, the multisampled framebuffer, into a texture of its own. Frame: the frame of the renderer.
	// texture unit 0 is used (the unit cache is reset), framebuffer bindings are left alone
	void measure(GLuint const Depth, GLuint const Source, int const Width, int const Height, view const &View, unsigned int const Frame);
	// takes in the newest measurement the gpu has finished, if any, and returns the newest one taken in so far
	result const *latest();
	// the time of the passes, for the debug panel
	std::string info_times();

  private:
	// the texture the depth is resolved to when there is no resolved depth to measure
	void allocate(int const Width, int const Height);
	void release();

	std::unique_ptr<gl::program> m_depthprogram;
	std::unique_ptr<gl::program> m_boundsprogram;
	std::unique_ptr<gl::ssbo> m_buffer;
	std::unique_ptr<gl::readback> m_readback;
	GLuint m_depth{0};
	GLuint m_depthfb{0};
	glm::ivec2 m_depthsize{0};
	gl::gpu_timer m_timer;
};
