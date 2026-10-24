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

#include "gl/shader.h"
#include "gl/vao.h"
#include "gl/framebuffer.h"
#include "gl/query.h"

class opengl_texture;

// screen space ambient occlusion of the post processing of the opengl 4.6 renderer:
// - ground truth ambient occlusion (Jimenez et al. 2016) at half the resolution in two compute passes, the horizon search
//   (gtao_main.comp) and a depth aware smoothing (gtao_denoise.comp)
// - its application to the image: a multiplier drawn over it (postfx_ssao_apply.frag), which also applies the older
//   ssao (postfx_ssao.frag, run by the renderer) when that one is chosen
// switched by Global.gfx_postfx_ao (0 off, 1 older ssao, 2 gtao); the targets are made with the viewport when it is not 0
class opengl46_ao
{
  public:
	opengl46_ao() = default;
	opengl46_ao(const opengl46_ao &) = delete;
	opengl46_ao &operator=(const opengl46_ao &) = delete;

	// creates the programs. throws gl::shader_exception
	void init();
	// ground truth ambient occlusion from the depth of the scene drawn with the given projection: Raw gets the result of
	// the horizon search, Smooth the smoothed one. both half the size of Depth
	void compute(opengl_texture &Depth, opengl_texture &Raw, opengl_texture &Smooth, glm::mat4 const &Projection);
	// multiplies the image of the framebuffer by the occlusion. Mode: Global.gfx_postfx_ao. the viewport has to be set
	// to the size of Depth. leaves the framebuffer bound
	void apply(int const Mode, opengl_texture &Depth, opengl_texture &Occlusion, gl::framebuffer &Target, glm::mat4 const &Projection);
	// lines for the gpu times of the debug panel
	std::string info_times();
	// controls of the debug window. Available: the targets were made, so the mode can be changed
	void draw_debug_ui(bool const Available);

  private:
	std::unique_ptr<gl::program> m_main_program;
	std::unique_ptr<gl::program> m_denoise_program;
	std::unique_ptr<gl::program> m_apply_program;
	std::unique_ptr<gl::vao> m_vao;
	gl::gpu_timer m_timer;
	bool m_timing{false};
};
