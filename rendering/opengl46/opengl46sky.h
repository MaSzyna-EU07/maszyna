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
#include "gl/cubemap.h"
#include "gl/query.h"
#include "gl/readback.h"
#include "rendering/opengl46/opengl46light.h"
#include "rendering/common/skylight.h"

class CSkyDome;

// light of the sky for the opengl 4.6 renderer:
// - the directional part of the ambient light, as spherical harmonics in sky_ssbo, worked out from the colours of the sky
//   dome or from the physical atmosphere. the shading adds it to the uniform ambient light (light_common.glsl), so the
//   average of the ambient light stays what the rest of the renderer sets
// - the physical atmosphere (Hillaire 2020, atmosphere_*.comp): look-up tables made in compute passes, the sky drawn from
//   them (sky_atmosphere.frag), and the sunlight, ambient light and fog colour read back from them a few frames later.
//   these are values for the render only: the simulation (Global.DayLight, fLuminance) keeps its own
// - the environment map prefiltered with the GGX lobe, for rough reflections (envmap_prefilter.comp)
// all of it is switched by Global.gfx_ambient_sh, gfx_atmosphere and gfx_envmap_ggx; off, the renderer works as before.
// what isn't the work of the gpu is the light of the sky shared with the vulkan renderer (gfx::sky_light)
class opengl46_sky
{
  public:
	opengl46_sky() = default;
	~opengl46_sky();
	opengl46_sky(const opengl46_sky &) = delete;
	opengl46_sky &operator=(const opengl46_sky &) = delete;

	// creates the buffers, tables and programs. throws gl::shader_exception
	void init();
	// binds the buffers the shaders read; bindings are context state, so done in every context
	void bind();
	// works out the light of the sky for the frame and replaces with it the colours of Sunlight (diffuse, ambient) and
	// Fogcolor, which come in as the simulation set them. runs the compute passes, so it is called in the main context,
	// before the passes of the frame. Sundirection and Sunangle (degrees) are those of the sun, also when the moon is the key light
	void update(opengl46_light &Sunlight, glm::vec3 &Fogcolor, CSkyDome const &Skydome, glm::vec3 const &Sundirection, float const Sunangle);
	// share of the atmosphere in the sky of the frame; 0 when the old sky is drawn alone
	float atmosphere_weight() const
	{
		return m_light.atmosphere_weight();
	}
	// the sky-view table of the atmosphere, its white balance, brightness and the overcast it was drawn with, for the colours of
	// the fog (opengl46_weather)
	GLuint skyview_table() const
	{
		return m_skyview_table;
	}
	glm::vec3 const &white_balance() const
	{
		return m_light.white_balance();
	}
	float sky_scale() const
	{
		return m_light.sky_scale();
	}
	float overcast() const
	{
		return m_light.overcast();
	}
	// the volumetric clouds are drawn over the sky (it doesn't take the overcast look then), and their shadows hold back the
	// sunlight (it isn't dimmed by the overcast then)
	void set_clouds(bool const Clouds, bool const Shadows)
	{
		m_light.set_clouds(Clouds, Shadows);
	}
	// sets the shader parameters of the sky drawn with the atmosphere (sky_atmosphere.frag) and binds its table
	void setup_sky(gl::model_ubs &Model) const;
	// colour of the sun disc, the old one blended with the colour the atmosphere gives it
	glm::vec3 sun_disc_color(glm::vec3 const &Oldcolor) const;
	// prefilters the levels of the GGX map from the environment map, which has to have its mipmaps
	void prefilter(gl::cubemap const &Source);
	// the prefiltered map, null until it was made once
	gl::cubemap const *prefiltered() const
	{
		return m_prefiltered_ready ? m_prefiltered.get() : nullptr;
	}
	// lines for the gpu times of the debug panel
	std::string info_times();
	// controls of the debug window
	void draw_debug_ui();

  private:
	// sets up the parameters of the atmosphere for the frame and runs the passes whose input changed
	void update_atmosphere(glm::vec3 const &Sundirection);
	// creates an immutable 2d texture for a look-up table
	static GLuint create_table(GLsizei Width, GLsizei Height);

	// buffers the shaders read
	std::unique_ptr<gl::ssbo> m_sky_ssbo;
	std::unique_ptr<gl::ubo> m_atmosphere_ubo;
	std::unique_ptr<gl::ssbo> m_atmosphere_result_ssbo;
	gl::sky_data m_sky_data{};
	bool m_sky_data_uploaded{false};

	// atmosphere
	std::unique_ptr<gl::program> m_transmittance_program;
	std::unique_ptr<gl::program> m_multiscattering_program;
	std::unique_ptr<gl::program> m_skyview_program;
	std::unique_ptr<gl::program> m_ambient_program;
	GLuint m_transmittance_table{0};
	GLuint m_multiscattering_table{0};
	GLuint m_skyview_table{0};
	gl::readback m_result_readback{sizeof(gl::atmosphere_result)};
	gl::gpu_timer m_atmosphere_timer;

	// the light of the sky, shared with the vulkan renderer
	gfx::sky_light m_light;

	// prefiltered environment map
	std::unique_ptr<gl::program> m_prefilter_program;
	std::unique_ptr<gl::cubemap> m_prefiltered;
	bool m_prefiltered_ready{false};
	gl::gpu_timer m_prefilter_timer;
};
