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
#include "gl/shader.h"
#include "gl/vao.h"
#include "gl/buffer.h"
#include "gl/query.h"
#include "rendering/common/weatherstate.h"

class opengl46_sky;

// the weather of the opengl 4.6 renderer, drawn from the weather of the simulation (environment/weather.h):
// - the fog, for the shading (apply_fog.glsl): in the colour pass of the main view a volume laid over the view (fog_inject.comp,
//   fog_integrate.comp) with the shafts of the sun, the shadows of the clouds and the light of the lamps and headlights, and
//   along the ray (weather_common.glsl) past it and in the other passes; gfx.fog analytic leaves the volume out. lit with the
//   colour the atmosphere has towards the horizon (fog_horizon.comp), so far fog and the sky behind it meet
// - the volumetric clouds of the layers of the weather (clouds_common.glsl, clouds_march.comp): the main view at a part of the
//   resolution, blended over frames (clouds_temporal.comp), and a panorama around the camera for the environment map and the
//   other views; laid over the sky with the fog (sky_weather.frag)
// - the shadows of the clouds on the ground around the camera (clouds_shadow.comp), which hold back the sun in the shading and
//   in the fog
// - the stars of the catalogue (environment/starcatalog.h)
// the weather, cloud and fog buffer is weather_ubo (gl::weather_ubs), bound for every pass
class opengl46_weather
{
  public:
	opengl46_weather() = default;
	~opengl46_weather();
	opengl46_weather(const opengl46_weather &) = delete;
	opengl46_weather &operator=(const opengl46_weather &) = delete;

	// creates the programs, buffers and noise textures. throws gl::shader_exception
	void init();
	// binds the buffer and textures the shading reads; bindings are context state, so done in every context
	void bind();
	// what the weather of the simulation is for the frame: the buffer, the shadows of the clouds, a band of the panorama, the
	// colours of the fog. in the main context, before the passes of the frame. Keylight: world direction towards the sun or the
	// moon, Keycolor: its colour as the shading gets it, Ambient: the ambient light of the sky, Fogcolor: the old colour of the fog
	void update(opengl46_sky const &Sky, glm::vec3 const &Keylight, glm::vec3 const &Keycolor, glm::vec3 const &Ambient, glm::vec3 const &Fogcolor, glm::dvec3 const &Camera);
	// the clouds and the fog volume of the colour pass of the main view. Projection and View (rotation, world axes to the view)
	// of its camera, Size of the target the scene is drawn into, Cablamp the light of the cab lamp (it lights no fog) or -1.
	// the light buffer, the light clusters and the shadow map of the pass are bound
	void render_view(glm::mat4 const &Projection, glm::mat4 const &View, glm::dvec3 const &Camera, glm::ivec2 const &Size, int const Cablamp);
	// the fog of a pass: the volume of the main view (after render_view() in the same frame), or along the ray
	void setup_pass(bool const Volume, glm::ivec2 const &Size);
	// the stars, in the sky pass. View: rotation of the camera of the pass
	void draw_stars(glm::mat4 const &View, float const Sunangle, double const Siderealtime, double const Latitude);
	// the clouds and the fog over the sky, in the sky pass. Mainview: the clouds of the main view, the panorama otherwise
	void draw_sky(glm::mat4 const &Projection, glm::mat4 const &View, bool const Mainview);

	// true when the volumetric clouds are drawn (the sky leaves its own overcast look then)
	bool clouds_drawn() const
	{
		return m_state.clouds_drawn();
	}
	// true when the clouds cast shadows (the sunlight isn't dimmed by the overcast then, the shadows do it)
	bool cloud_shadows() const
	{
		return m_state.cloud_shadows();
	}
	// fog density of a regular draw (model_ubs.fog_density): 1 / the fog range
	float fog_density() const
	{
		return m_state.fog_density();
	}
	// distance the fog hides things at, for the draw range: a dark object fades to about 2% there
	float fog_range() const
	{
		return m_state.fog_range();
	}
	// lines for the gpu times of the debug panel
	std::string info_times();
	// controls of the debug window
	void draw_debug_ui();

  private:
	using quality_level = gfx::weather_state::quality_level;

	static GLuint create_texture(GLenum Target, GLenum Format, glm::ivec3 const &Size, GLint Filter, GLint Wrap);
	static void release(GLuint &Texture);
	// the three noises, once
	void make_noise();
	// the textures of the clouds of the main view and of the fog volume, for the size and quality
	void prepare_view(glm::ivec2 const &Size, quality_level const &Level);
	void bind_noise() const;
	void upload_header();

	// programs
	std::unique_ptr<gl::program> m_noise_program;
	std::unique_ptr<gl::program> m_march_program;
	std::unique_ptr<gl::program> m_temporal_program;
	std::unique_ptr<gl::program> m_shadow_program;
	std::unique_ptr<gl::program> m_horizon_program;
	std::unique_ptr<gl::program> m_inject_program;
	std::unique_ptr<gl::program> m_integrate_program;
	std::unique_ptr<gl::program> m_sky_program;
	std::unique_ptr<gl::program> m_stars_program;

	std::unique_ptr<gl::ubo> m_weather_ubo;
	// the block and the decisions of the frame, shared with the vulkan renderer
	gfx::weather_state m_state;

	// noise
	GLuint m_shape_noise{0};
	GLuint m_detail_noise{0};
	GLuint m_coverage_noise{0};

	// shadows of the clouds
	GLuint m_shadow_map{0};
	int m_shadow_size{0};
	gl::gpu_timer m_shadow_timer;

	// clouds of the main view
	glm::ivec2 m_view_size{0};
	quality_level m_view_level{};
	GLuint m_clouds_color{0};
	GLuint m_clouds_depth{0};
	std::array<GLuint, 2> m_clouds_history{0, 0};
	int m_clouds_current{0}; // the history holding the latest result
	gl::gpu_timer m_clouds_timer;
	// panorama
	GLuint m_panorama{0};
	glm::ivec2 m_panorama_size{0};
	gl::gpu_timer m_panorama_timer;

	// fog volume
	glm::ivec3 m_fog_grid{0};
	std::array<GLuint, 2> m_fog_scattering{0, 0};
	int m_fog_current{0};
	GLuint m_fog_integrated{0};
	gl::gpu_timer m_fog_timer;

	// stars
	std::optional<gl::vao> m_stars_vao;
	std::optional<gl::buffer> m_stars_buffer;
	int m_stars_count{0};
};
