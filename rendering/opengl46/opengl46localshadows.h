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

#include "gl/shader.h"
#include "gl/query.h"

class TDynamicObject;
class TModel3d;

// cube shadow maps of the lamps of the opengl 4.6 renderer (Global.gfx_lights_shadows). for now one map, of the lamp of the
// occupied cab, with the shadows cast by the cab (the model drawn by Render_cab()).
// the map is kept in the space of the cab model, centred on the lamp: it stays right while the vehicle moves and turns, and is
// drawn anew only every few frames (for the moving parts of the cab: levers, doors, wipers) or when the lamp, the cab or the
// level change. the six faces are drawn at once, by a geometry shader (shadowcube.vert/.geom), so the cab costs one draw call
// per submodel, as in the colour pass. the depth is n / d (d the distance along the axis of the face), see local_light_shadow()
// in light_common.glsl, which reads the map for the light with the layer in its headlight_weights.x
class opengl46_local_shadows
{
  public:
	opengl46_local_shadows() = default;
	~opengl46_local_shadows();
	opengl46_local_shadows(const opengl46_local_shadows &) = delete;
	opengl46_local_shadows &operator=(const opengl46_local_shadows &) = delete;

	// creates the programs and the stand-in map. throws gl::shader_exception
	void init();
	// puts the map (or the stand-in, which shadows nothing) on its texture unit, gl::LOCAL_SHADOW_TEX. once per context and frame
	void bind();
	// whether the map of the cab lamp is to be drawn in this frame: Vehicle and Cab the occupied vehicle and its cab model,
	// Lamp the position of the lamp in the space of the cab model; Vehicle nullptr when the lamp is off. (re)makes the map to
	// the size of the level when the level changed
	bool update_wanted(TDynamicObject const *Vehicle, TModel3d const *Cab, glm::vec3 const &Lamp, unsigned int const Framestamp);
	// sets up the drawing to the map: the framebuffer, viewport, cleared faces, the state of the faces. the geometry is to be
	// drawn with program() and a modelview from the space of the model to the space of the map
	void begin();
	// ends the drawing to the map, leaves the default framebuffer bound and the culling of faces on
	void end(std::uint32_t const Drawcalls);
	// shadow program of the cube maps, of the alpha tested materials when Alpha
	gl::program &program(bool const Alpha)
	{
		return Alpha ? *m_alpha_program : *m_program;
	}
	// whether the map holds the shadows of the lamp of Vehicle, the occupied vehicle
	bool ready(TDynamicObject const *Vehicle) const
	{
		return m_valid && (Vehicle != nullptr) && (Vehicle == m_vehicle);
	}
	// position of the lamp the map was drawn for, in the space of the cab model
	glm::vec3 const &lamp() const
	{
		return m_lamp;
	}
	// the headlight_weights of the light which reads the map: layer, near plane, size of a texel one metre from the lamp,
	// radius of the filter in texels
	glm::vec4 weights() const;
	// lines for the gpu times and statistics of the debug panel
	std::string info_times();
	std::string info() const;
	// controls for the debug window
	void draw_debug_ui();

  private:
	// size, refresh interval (frames) and filter radius (texels, 0: the 2x2 filter of the hardware) of a level
	struct level_setup
	{
		int size;
		unsigned int interval;
		float radius;
	};
	static level_setup setup(int const Level);
	void allocate(int const Size);
	void release();

	std::unique_ptr<gl::program> m_program;
	std::unique_ptr<gl::program> m_alpha_program;
	GLuint m_texture{0}; // cube map array, one cube
	GLuint m_empty{0}; // stand-in: a cube map array of 1x1 faces with the farthest depth
	GLuint m_framebuffer{0};
	int m_size{0}; // of a face of m_texture, 0 when there is none
	int m_level{-1}; // level the map was made for
	bool m_valid{false}; // m_texture holds the map of m_vehicle
	TDynamicObject const *m_vehicle{nullptr};
	TModel3d const *m_cab{nullptr};
	glm::vec3 m_lamp{0.f};
	unsigned int m_drawnframe{0}; // framestamp the map was last drawn in
	gl::gpu_timer m_timer;
	// statistics
	std::uint32_t m_drawcalls{0}; // of the last drawing of the map
	std::uint32_t m_updatecount{0}; // drawings of the map so far
	double m_cputime{0.0}; // milliseconds, smoothed
	std::chrono::steady_clock::time_point m_cpustart;
};
