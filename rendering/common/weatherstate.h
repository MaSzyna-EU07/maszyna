/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the weather the renderers draw (the part of the stage F of claude/gl46-renderer-features-plan.md which isn't the work of the
// gpu, taken out of opengl46_weather for the vulkan renderer, stage P2 of claude/vulkan14-renderer-plan.md): the block of the
// weather of the shaders (gl::weather_ubs) made from the weather of the simulation (environment/weather.h) for the frame and for
// each pass, the quality levels, the place of the map of the shadows of the clouds, the bands of the panorama of the clouds, the
// sampling of the fog volume over the frames, and the stars of the catalogue (environment/starcatalog.h)

#pragma once

#include <string>
#include <vector>

#include "gl/ubo.h"

namespace gfx
{

class weather_state
{
  public:
	struct quality_level
	{
		int divisor; // of the resolution of the clouds of the main view
		int steps; // samples through a layer at most
		int light_steps; // samples towards the sun
		bool detail; // with the small noise
		glm::ivec2 panorama; // size of the panorama
		glm::ivec3 fog_grid; // cells of the fog volume
	};
	static quality_level quality(int const Clouds, int const Fog);

	// the camera of the previous frame of the main view, for the blending over frames
	struct view_state
	{
		glm::mat4 projection_view{1.f}; // world axes to clip space, camera at the origin
		glm::dvec3 camera{0.0};
		glm::vec4 slicing{0.f};
		bool valid{false};
	};

	// a grey fog and no clouds until the first update
	weather_state();

	// what the weather of the simulation is for the frame: the block, whether the clouds are drawn and cast shadows. before the
	// gpu work of the frame. Atmosphereweight: share of the atmosphere in the sky, Skyview: its sky-view table is there to
	// give the colours of the horizon; Keylight: world direction towards the sun or the moon, Keycolor: its colour as the
	// shading gets it, Ambient: the ambient light of the sky, Fogcolor: the old colour of the fog
	void update(float const Atmosphereweight, bool const Skyview, glm::vec3 const &Keylight, glm::vec3 const &Keycolor, glm::vec3 const &Ambient, glm::vec3 const &Fogcolor, glm::dvec3 const &Camera);
	// the shadows of the clouds are to be drawn in this frame (after update())
	bool shadows_wanted() const
	{
		return m_shadows_wanted;
	}
	// the place of the map of the shadows of Size texels, snapped to its texels so it doesn't crawl as the camera moves
	void place_shadows(int const Size, glm::dvec3 const &Camera);
	// the map of the shadows was drawn (or not) in this frame
	void shadows_drawn(bool const Drawn);
	// the rows of the panorama of the clouds of Size to draw in this frame: a band of it, all of it the first time after
	// panorama_resized()
	void panorama_band(glm::ivec2 const &Size, int &First, int &Rows);
	void panorama_resized();
	// a camera close enough to the one of the previous frame to blend with
	static bool continuity(view_state const &Previous, glm::dvec3 const &Camera);
	// the point the cells of the fog volume are sampled at in this frame (Temporal: it moves from frame to frame) and the slicing of
	// the volume of Slices slices it makes
	glm::vec3 fog_jitter(bool const Temporal) const;
	static glm::vec4 fog_slicing(int const Slices, glm::vec3 const &Jitter);
	// the clouds of the main view and the fog volume were made in this frame
	void clouds_made()
	{
		m_clouds_view_valid = true;
	}
	void fog_made(glm::vec4 const &Slicing, glm::ivec3 const &Grid);
	// the fog of a pass: the volume of the main view (made in the same frame), or along the ray. Size: the target of the pass
	void setup_pass(bool const Volume, glm::ivec2 const &Size);

	gl::weather_ubs const &block() const
	{
		return m_weather;
	}
	// the colours of the horizon come from the sky-view table of the atmosphere on the gpu, not from the cpu
	bool horizon_on_gpu() const
	{
		return m_horizon_on_gpu;
	}
	// true when the volumetric clouds are drawn (the sky leaves its own overcast look then)
	bool clouds_drawn() const
	{
		return m_clouds_drawn;
	}
	bool clouds_view_valid() const
	{
		return m_clouds_view_valid;
	}
	// true when the clouds cast shadows (the sunlight isn't dimmed by the overcast then, the shadows do it)
	bool cloud_shadows() const
	{
		return m_shadows_valid;
	}
	bool fog_volume_valid() const
	{
		return m_fog_volume_valid;
	}
	// fog density of a regular draw (model_ubs.fog_density): 1 / the fog range
	float fog_density() const
	{
		return 1.f / m_fog_range;
	}
	// distance the fog hides things at, for the draw range: a dark object fades to about 2% there
	float fog_range() const
	{
		return m_fog_range;
	}
	// frames updated, the sequence of the sampling over frames
	int frame() const
	{
		return m_frame;
	}

	// the stars, in the equatorial frame: for each star the direction and the magnitude, then the colour and the phase of its
	// twinkling
	static std::vector<glm::vec4> star_vertices();
	// whether the stars are drawn in the sky pass, and what with: the rotation from the equatorial frame to the world, from the
	// world to the view of the pass (View: rotation of its camera), the brightness, the time, the size of the points
	bool stars(float const Sunangle, double const Siderealtime, double const Latitude, glm::mat4 const &View, glm::mat4 &Toworld, glm::mat4 &Toview, glm::vec4 &Params) const;

	// controls of the debug window, Volumeavailable: the renderer has the fog volume
	void draw_debug_ui();

	// the cameras of the clouds and of the fog volume of the previous frame
	view_state previous_clouds;
	view_state previous_fog;

  private:
	gl::weather_ubs m_weather{};
	bool m_horizon_on_gpu{false};
	bool m_shadows_wanted{false};
	bool m_shadows_valid{false};
	bool m_clouds_drawn{false};
	bool m_clouds_view_valid{false};
	int m_panorama_band{0};
	bool m_panorama_full{false};
	glm::ivec3 m_fog_grid{0};
	bool m_fog_volume_valid{false};
	float m_fog_range{5000.f};
	glm::vec4 m_fog_slicing{0.f};
	int m_frame{0};

	// tuning of the look of the clouds in the debug window, not saved: scales of their sunlight, of their sky light and of the
	// erosion of their edges by the detail noise (weather_sun_color.w, weather_ambient.w, cloud_wind.w)
	float m_cloud_sun_scale{1.f};
	float m_cloud_sky_scale{1.f};
	float m_cloud_detail_scale{1.f};
	// what the debug window shows
	float m_sun_share{0.f};
	float m_average_transmittance{1.f};
	std::string m_report;
};

} // namespace gfx
