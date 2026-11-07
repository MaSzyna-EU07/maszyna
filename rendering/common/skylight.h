/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the light of the sky the renderers share (the part of the stage S of claude/gl46-renderer-features-plan.md which isn't the
// work of the gpu, taken out of opengl46_sky for the vulkan renderer, stage P1 of claude/vulkan14-renderer-plan.md):
// - the directional part of the ambient light, as spherical harmonics (gl::sky_data), worked out from the colours of the sky
//   dome or from the physical atmosphere
// - the parameters of the physical atmosphere (gl::atmosphere_ubs) and when its look-up tables are to be made again
// - what the result of the atmosphere read back from the gpu (gl::atmosphere_result) makes of the sunlight, the ambient light
//   and the fog colour of the frame, the white balance, the share of the atmosphere in the sky and the colour of the sun disc
// switched by Global.gfx_ambient_sh and gfx_atmosphere; off, the light the simulation set is left as it is

#pragma once

#include <array>
#include <optional>

#include "gl/ubo.h"
#include "rendering/light.h"

class CSkyDome;

namespace gfx
{

class sky_light
{
  public:
	// the parameters of the atmosphere valid from the start, the tables stay black until they are first made
	sky_light();

	// the first part of the update of a frame: the overcast of the frame, no atmosphere yet
	void begin_update();
	// the inputs of the look-up tables of the atmosphere for the frame (Sundirection normalized): true when the tables are to be
	// made now, Medium when the transmittance and multiple scattering ones are too (the sky-view one and the ambient light follow
	// the sun). the parameters are set up for them, and the tables are taken as made
	bool atmosphere_changed(glm::vec3 const &Sundirection, bool &Medium);
	// a result of the atmosphere read back from the gpu
	void set_result(gl::atmosphere_result const &Result)
	{
		m_result = Result;
	}
	// the rest of the update: replaces the colours of Sunlight (diffuse, ambient) and Fogcolor, which come in as the simulation
	// set them, with the light of the sky, and works out the directional part of the ambient light. Sundirection and Sunangle
	// (degrees) are those of the sun, also when the moon is the key light
	void end_update(basic_light &Sunlight, glm::vec3 &Fogcolor, CSkyDome const &Skydome, glm::vec3 const &Sundirection, float const Sunangle);

	gl::atmosphere_ubs const &atmosphere() const
	{
		return m_atmosphere;
	}
	// the directional part of the ambient light for the shading
	gl::sky_data const &sky_data() const
	{
		return m_sky_data;
	}
	// share of the atmosphere in the sky of the frame; 0 when the old sky is drawn alone
	float atmosphere_weight() const
	{
		return m_weight;
	}
	bool has_result() const
	{
		return m_result.has_value();
	}
	glm::vec3 const &white_balance() const
	{
		return m_white_balance;
	}
	float sky_scale() const
	{
		return m_sky_scale;
	}
	float overcast() const
	{
		return m_overcast;
	}
	// the volumetric clouds are drawn over the sky (it doesn't take the overcast look then), and their shadows hold back the
	// sunlight (it isn't dimmed by the overcast then)
	void set_clouds(bool const Clouds, bool const Shadows)
	{
		m_clouds = Clouds;
		m_cloud_shadows = Shadows;
	}
	// the shader parameters of the sky drawn with the atmosphere (sky_atmosphere.frag), param[0] and param[1]
	void sky_params(glm::vec4 &Param0, glm::vec4 &Param1) const;
	// colour of the sun disc, the old one blended with the colour the atmosphere gives it
	glm::vec3 sun_disc_color(glm::vec3 const &Oldcolor) const;
	// controls of the debug window
	void draw_debug_ui() const;

  private:
	// spherical harmonics of order 2, rgb
	using sh_coefficients = std::array<glm::vec3, gl::SKY_SH_COEFFICIENTS>;

	// directional part of the ambient light from the colours of the dome, scaled so its average is Ambient
	sh_coefficients dome_directional(CSkyDome const &Skydome, glm::vec3 const &Ambient, glm::vec3 const &Sunlight, float const Sunheight) const;

	gl::atmosphere_ubs m_atmosphere{};
	// inputs the tables were last made for: the transmittance and multiple scattering tables depend on the medium only,
	// the sky-view table also on the sun
	std::optional<glm::vec4> m_tables_medium;
	std::optional<glm::vec3> m_tables_sun;
	std::optional<gl::atmosphere_result> m_result;
	gl::sky_data m_sky_data{};

	// what the update made of the frame, for the sky, the sun disc and the debug window
	float m_weight{0.f};
	glm::vec3 m_white_balance{1.f};
	float m_sky_scale{0.f};
	float m_overcast{0.f};
	bool m_clouds{false};
	bool m_cloud_shadows{false};
	float m_sky_flat_luminance{0.f};
	glm::vec3 m_sun_disc_color{1.f};
	glm::vec3 m_sunlight{0.f};
	glm::vec3 m_ambient{0.f};
	glm::vec3 m_fog{0.f};

	// cache of the directional ambient light of the dome, remade when its inputs change
	std::optional<std::array<glm::vec4, 3>> m_dome_key;
	sh_coefficients m_dome_sh{};
};

} // namespace gfx
