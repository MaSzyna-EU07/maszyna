/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "gl/ubo.h"
#include "gl/ssbo.h"
#include "gl/streambuffer.h"
#include "gl/shader.h"
#include "gl/postfx.h"
#include "gl/framebuffer.h"
#include "gl/query.h"
#include "gl/readback.h"

class opengl_texture;

// automatic exposure and bloom of the post processing of the opengl 4.6 renderer:
// - a luminance histogram of the final hdr image in a compute pass, and the exposure adapted to its average over time,
//   within the limits of gfx.postfx.autoexposure.* (exposure_*.comp). the exposure stays on the gpu, the tonemapping reads it
// - bloom: the image taken down a chain of half size levels and back up, each level added (postfx_bloom_*.frag)
// both are switched by Global.gfx_postfx_autoexposure and gfx_postfx_bloom; off, the tonemapping gets the image as before
class opengl46_exposure
{
  public:
	// the levels of the bloom chain of one viewport
	struct bloom_chain
	{
		std::vector<std::unique_ptr<opengl_texture>> down;
		std::vector<std::unique_ptr<gl::framebuffer>> down_fb;
		// one fewer than down: the lowest level goes up as it is
		std::vector<std::unique_ptr<opengl_texture>> up;
		std::vector<std::unique_ptr<gl::framebuffer>> up_fb;
		std::vector<glm::ivec2> sizes;
	};

	opengl46_exposure() = default;
	opengl46_exposure(const opengl46_exposure &) = delete;
	opengl46_exposure &operator=(const opengl46_exposure &) = delete;

	// creates the programs and the buffer. throws gl::shader_exception
	void init();
	// binds the buffer the tonemapping reads; bindings are context state, so done in every context
	void bind();
	// the bloom chain for an image of the given size; null if it could not be made
	static std::unique_ptr<bloom_chain> make_bloom_chain(int const Width, int const Height);
	// measures the luminance of the image and adapts the exposure to it. Deltatime: seconds since the last measurement
	void measure(opengl_texture &Image, double const Deltatime);
	// takes the image down and up the chain; returns the result, which the tonemapping blends in. leaves the viewport
	// and the framebuffer binding changed. Model is the model_ubs the post processing shaders read the parameters from
	opengl_texture *bloom(bloom_chain &Chain, opengl_texture &Image, gl::stream_ubo &Modelubo, gl::model_ubs &Model);
	// parameters of the tonemapping (postfx_tonemapping.frag), with bloom from a chain of Bloomlevels levels (0: none)
	glm::vec4 tonemap_params(std::size_t const Bloomlevels) const;
	// lines for the gpu times of the debug panel
	std::string info_times();
	// controls of the debug window
	void draw_debug_ui();

  private:
	std::unique_ptr<gl::program> m_histogram_program;
	std::unique_ptr<gl::program> m_average_program;
	std::unique_ptr<gl::postfx> m_bloom_down;
	std::unique_ptr<gl::postfx> m_bloom_up;
	std::unique_ptr<gl::ssbo> m_exposure_ssbo;
	// the exposure and luminances, read back for the debug window only
	gl::readback m_readback{4 * sizeof(float)};
	float m_readback_data[4]{1.f, 1.f, 1.f, 0.f};
	gl::gpu_timer m_exposure_timer;
	gl::gpu_timer m_bloom_timer;
};
