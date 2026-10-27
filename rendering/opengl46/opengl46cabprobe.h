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
#include "gl/query.h"
#include "gl/ssbo.h"
#include "rendering/common/cabprobe.h"

// the probe of the occupied cab of the opengl 4.6 renderer (Global.gfx_cabprobe): the maps and the drawing to them of gfx::cab_probe,
// which tells when they're drawn and what the shading reads. the six faces of the maps are drawn at once, by a geometry shader
// (cabprobe.vert/.geom/.frag), the spherical harmonics of the light worked out by cabprobe_sh.comp
class opengl46_cab_probe : public gfx::cab_probe
{
  public:
	opengl46_cab_probe() = default;
	~opengl46_cab_probe() override;

	// creates the programs, the stand-ins and the storage of the spherical harmonics. throws gl::shader_exception
	void init();
	// puts the maps (or the stand-ins) on their texture units, gl::CAB_PROBE_TEX and gl::CAB_PROBE_ALBEDO_TEX, and the spherical
	// harmonics on gl::CAB_PROBE_SSBO_BINDING. once per context and frame
	void bind();
	// program of the probe, of the alpha tested materials when Alpha
	gl::program &program(bool const Alpha)
	{
		return Alpha ? *m_alpha_program : *m_program;
	}
	// a white texture, for the materials without a diffuse texture
	GLuint white() const
	{
		return m_white;
	}
	// lines for the gpu times of the debug panel
	std::string info_times();

  protected:
	bool gpu_allocate(int const Size) override;
	void gpu_release() override;
	void gpu_begin() override;
	void gpu_end() override;

  private:
	std::unique_ptr<gl::program> m_program;
	std::unique_ptr<gl::program> m_alpha_program;
	std::unique_ptr<gl::program> m_sh_program;
	std::unique_ptr<gl::ssbo> m_sh;
	// the light reflected and the albedo (cube maps with mipmaps), the depth of the faces, and the framebuffer drawing to all faces
	GLuint m_light{0};
	GLuint m_albedo{0};
	GLuint m_depth{0};
	GLuint m_framebuffer{0};
	// stand-ins: a cube map of 1x1 faces of nothing, a 1x1 white texture
	GLuint m_empty{0};
	GLuint m_white{0};
	gl::gpu_timer m_timer;
};
