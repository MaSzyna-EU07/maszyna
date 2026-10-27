/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "gl/ubo.h"
#include "rendering/common/locallights.h"

class TDynamicObject;
class TModel3d;

namespace gfx
{

// the probe of the occupied cab (Global.gfx_cabprobe, the stage A4 of claude/renderers-plan.md), the part shared by the renderers;
// the backend (opengl46_cab_probe, vk14::cab_probe_maps) keeps the maps and draws to them through the gpu_* methods.
// - two cube maps drawn from the middle of the box of the cab model (mdKabina), in the space of the model, so they stay right while
//   the vehicle moves and turns: the light of the cab lamp the cab reflects (with the shadow map of the lamp) and the light of what
//   glows in it, and the albedo of the cab. both are cleared to nothing and premultiplied by the share of the direction the cab
//   fills, which is in their alpha (the glasses aren't drawn: through them the cab shows nothing), and have mipmaps;
// - the six faces are drawn at once (a geometry shader with opengl, multiview with vulkan) by the programs of the probe
//   (cabprobe.vert, cabprobe.frag), every few frames (Global.gfx_cabprobe_interval: the moving parts of the cab, the lights of the
//   desk) and at once when the cab, its box, the level or the cab lamp change; the light reflected is then made into spherical
//   harmonics (cabprobe_sh.comp, gl::cab_probe_data);
// - the shading of the parts of the cab (gl::model_ubs::cab_interior) reads them through gl::scene_ubs::cab_probe_*: the reflected
//   light lights the cab again, in place of the share of the light of the lamp the ambient light kept (Global.gfx_lights_cab_bounce),
//   and the cab reflects itself where the screen space reflections don't find it, the box standing for its walls; what the cab
//   shows there is the light of the lamp it reflects and its albedo lit by the ambient light of the reflecting surface.
//   the ambient light and the sun are left out of the probe: the cab gets them as before
class cab_probe
{
  public:
	cab_probe() = default;
	virtual ~cab_probe() = default;
	cab_probe(const cab_probe &) = delete;
	cab_probe &operator=(const cab_probe &) = delete;

	// size of a face of the maps of the level (Global.gfx_cabprobe), 0 for none
	static int size_of(int const Level);
	// whether the probe is to be drawn in this frame. Vehicle: the occupied vehicle whose cab is drawn (nullptr: none), Cab its cab
	// model, Center and Extent the box of the model (model space); Lamp the cab lamp when it's a light of the frame (nullptr: none).
	// (re)makes the maps when the level changed
	bool update_wanted(TDynamicObject const *Vehicle, TModel3d const *Cab, glm::vec3 const &Center, glm::vec3 const &Extent, gfx::local_lights::light const *Lamp, unsigned int const Framestamp);
	// the fields of the block of the scene the drawing of the probe reads: the cab lamp (Lamp, at Lamplocal in the space of the cab
	// model, nullptr: none) with its shadow map when Shadows (Shadowweights: the headlight_weights of the map)
	void setup_drawing(gl::scene_ubs &Scene, gfx::local_lights::light const *Lamp, glm::vec3 const &Lamplocal, bool const Shadows, glm::vec4 const &Shadowweights) const;
	// the drawing to the probe: the target, the viewport, the cleared faces, the state of the faces. the cab is to be drawn with the
	// programs of the probe and a modelview from the space of the cab model to the space of the probe (model_to_probe())
	void begin();
	// ends the drawing: the mipmaps and the spherical harmonics of the light
	void end(std::uint32_t const Drawcalls);
	// the space of the cab model to the space of the probe: the axes of the model, the origin in the middle of the box
	glm::dmat4 model_to_probe() const;
	// whether the probe holds the cab of Vehicle
	bool ready(TDynamicObject const *Vehicle) const
	{
		return m_valid && (Vehicle != nullptr) && (Vehicle == m_vehicle);
	}
	// the fields of the block of the scene the colour pass of the main view reads the probe with: the cab of Vehicle drawn, seen from
	// Camera (world space) with the rotation of the view View. cleared when the probe doesn't hold the cab
	void setup_scene(gl::scene_ubs &Scene, TDynamicObject const *Vehicle, glm::dvec3 const &Camera, glm::mat3 const &View) const;
	// the fields cleared, for the passes other than the colour pass of the main view
	static void clear_scene(gl::scene_ubs &Scene);
	// size of a face of the maps, 0 when there are none
	int size() const
	{
		return m_size;
	}
	// the level of the mipmaps the spherical harmonics are taken from, and its size
	int sh_level() const;
	int sh_size() const;
	// line for the statistics of the debug panel
	std::string info() const;
	// controls and counters for the debug window
	void draw_debug_ui();

  protected:
	// the maps of Size x Size faces with their mipmaps; false when they can't be made
	virtual bool gpu_allocate(int const Size) = 0;
	virtual void gpu_release() = 0;
	virtual void gpu_begin() = 0;
	virtual void gpu_end() = 0;

	int m_size{0};

  private:
	void allocate(int const Size);
	void release();

	int m_level{-1};
	bool m_valid{false};
	TDynamicObject const *m_vehicle{nullptr};
	TModel3d const *m_cab{nullptr};
	glm::vec3 m_center{0.f};
	glm::vec3 m_extent{0.f};
	// the light of the cab lamp the probe was drawn with, 0 without the lamp
	glm::vec3 m_lamp{0.f};
	unsigned int m_drawnframe{0};
	// statistics
	std::uint32_t m_drawcalls{0};
	std::uint32_t m_updatecount{0};
	double m_cputime{0.0};
	std::chrono::steady_clock::time_point m_cpustart;
};

} // namespace gfx
