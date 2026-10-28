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
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "gl/shader.h"
#include "gl/query.h"
#include "rendering/common/locallights.h"
#include "rendering/common/localshadows.h"
#include "rendering/lightarray.h"

// the shadow maps of the lamps of the opengl 4.6 renderer (Global.gfx_lights_shadows): the maps and the drawing to them of
// gfx::local_shadows, which tells what they hold. the six faces of a cube map are drawn at once, by a geometry shader
// (shadowcube.vert/.geom)
class opengl46_local_shadows : public gfx::local_shadows
{
  public:
	opengl46_local_shadows() = default;
	~opengl46_local_shadows() override;

	// creates the programs and the stand-in maps. throws gl::shader_exception
	void init();
	// puts the maps (or the stand-ins, which shadow nothing) on their texture units, gl::LOCAL_SHADOW_TEX and
	// gl::LOCAL_SHADOW_LAMPS_TEX. once per context and frame
	void bind();
	// shadow program of the cube maps, of the alpha tested materials when Alpha
	gl::program &program(bool const Alpha)
	{
		return Alpha ? *m_alpha_program : *m_program;
	}
	// the same for the multi draws of the instances drawn from the gpu culling, which read the model block of a draw by its
	// number (MODEL_DRAWS). nullptr when they couldn't be made: the draws are made one by one with program() then
	gl::program *multidraw_program(bool const Alpha)
	{
		return Alpha ? m_alpha_multidraw_program.get() : m_multidraw_program.get();
	}
	// lines for the gpu times of the debug panel
	std::string info_times();

  protected:
	bool gpu_allocate(int const Size) override;
	void gpu_release() override;
	bool gpu_allocate_lamps(int const Count, int const Size) override;
	void gpu_release_lamps() override;
	bool gpu_allocate_headlights(int const Count, int const Size) override;
	void gpu_release_headlights() override;
	void gpu_begin_cab() override;
	void gpu_end_cab() override;
	void gpu_begin_lamp(int const Scenerycube, int const Workingcube, bool const Vehicles) override;
	void gpu_end_lamp() override;
	void gpu_begin_headlight(std::size_t const Slot) override;
	void gpu_end_headlight() override;
	void gpu_lamps_timer(bool const Begin) override;
	void gpu_headlights_timer(bool const Begin) override;

  private:
	void set_layer(int const Layer);
	void set_near(float const Near);

	std::unique_ptr<gl::program> m_program;
	std::unique_ptr<gl::program> m_alpha_program;
	std::unique_ptr<gl::program> m_multidraw_program;
	std::unique_ptr<gl::program> m_alpha_multidraw_program;
	GLuint m_empty{0}; // stand-in: a cube map array of 1x1 faces with the farthest depth
	GLuint m_emptyheadlights{0}; // the same, a 2d array of one 1x1 layer
	// cab lamp: cube map array, one cube
	GLuint m_texture{0};
	GLuint m_framebuffer{0};
	gl::gpu_timer m_timer;
	// scenery lamps: cube map array, the scenery parts of the slots first, then their working copies
	GLuint m_lampstexture{0};
	GLuint m_lampsframebuffer{0};
	gl::gpu_timer m_lampstimer;
	// headlights: 2d array, a layer per slot
	GLuint m_headlightstexture{0};
	GLuint m_headlightsframebuffer{0};
	gl::gpu_timer m_headlightstimer;
};
