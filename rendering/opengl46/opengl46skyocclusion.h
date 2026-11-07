/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "rendering/common/skyocclusion.h"

// the maps of the occlusion of the sky by the scenery (gfx::sky_occlusion) for the opengl 4.6 renderer: two textures bound to their
// units (gl::SKY_OCCLUSION_HEIGHT_TEX, gl::SKY_OCCLUSION_TEX) for every pass, stand-ins of one texel while there are none
class opengl46_sky_occlusion : public gfx::sky_occlusion
{
  public:
	opengl46_sky_occlusion() = default;
	~opengl46_sky_occlusion() override;

	// the stand-ins
	void init();
	// the textures (or the stand-ins) bound to their units
	void bind() const;

  protected:
	bool gpu_allocate(int const Side) override;
	void gpu_release() override;
	void gpu_upload(int const X, int const Y, int const Size, float const *Heights, std::uint16_t const *Occlusion) override;

  private:
	GLuint m_heights{0};
	GLuint m_occlusion{0};
	GLuint m_emptyheights{0};
	GLuint m_emptyocclusion{0};
};
