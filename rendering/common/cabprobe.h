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
#include "rendering/common/localshadows.h"

class TDynamicObject;
class TModel3d;

namespace gfx
{

// the probe of the occupied cab (Global.gfx_cabprobe, the stage A4 of claude/renderers-plan.md), the part shared by the renderers;
// the backend (opengl46_cab_probe, vk14::cab_probe_maps) keeps the maps and draws to them through the gpu_* methods.
// - two cube maps drawn from the middle of the box of the cab model (mdKabina), in the space of the model, so they stay right while
//   the vehicle moves and turns: the light of the lights of the cab (the lamp, the light points of the cab model, the brightest
//   gl::MAX_CAB_PROBE_LAMPS of them, with their shadow maps) the cab reflects and the light of what glows in it, and the albedo of the cab. both are cleared to nothing and premultiplied by the share of the direction the cab
//   fills, which is in their alpha (the glasses aren't drawn: through them the cab shows nothing), and have mipmaps;
// - the six faces are drawn at once (a geometry shader with opengl, multiview with vulkan) by the programs of the probe
//   (cabprobe.vert, cabprobe.frag), every few frames (Global.gfx_cabprobe_interval: the moving parts of the cab, the lights of the
//   desk) and at once when the cab, its box, the level or the cab lamp change; the light reflected is then made into spherical
//   harmonics (cabprobe_sh.comp, gl::cab_probe_data);
// - the shading of the parts of the cab (gl::model_ubs::cab_interior) reads them through gl::scene_ubs::cab_probe_*: the reflected
//   light lights the cab again, in place of the share of the light of the lamp the ambient light kept (Global.gfx_lights_cab_bounce),
//   and the cab reflects itself where the screen space reflections don't find it, the box standing for its walls; what the cab
//   shows there is the light of the lamp it reflects and its albedo lit by the ambient light of the reflecting surface.
//   the ambient light is left out of the probe, the cab gets it as before. the sun (the moon) lights the probe where it comes in
//   through the windows (Global.gfx_cabprobe_sun), so the light which bounces around the cab is also the light of the sun where it
//   falls in the cab; the cab gets the sun itself from the shading, as before. where the windows are the probe drawn last tells:
//   a small level of its albedo is copied after each drawing (the mask, CAB_PROBE_MASK_TEX), and the ray towards the sun is taken
//   from a point of the cab to the box of the cab and looked up in it. the first drawing of a cab has no mask yet, so it's drawn
//   without the sun and again in the next frame
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
	// model, Center and Extent the box of the model (model space); Lights the lights of the cab of the frame, the brightest first
	// (empty: none). (re)makes the maps when the level changed
	bool update_wanted(TDynamicObject const *Vehicle, TModel3d const *Cab, glm::vec3 const &Center, glm::vec3 const &Extent, std::vector<gfx::local_lights::cab_light> const &Lights,
	                   unsigned int const Framestamp);
	// the fields of the block of the scene the drawing of the probe reads: the lights of the cab of Vehicle (Lights, the brightest first)
	// with the shadow maps Shadows holds for them
	void setup_drawing(gl::scene_ubs &Scene, std::vector<gfx::local_lights::cab_light> const &Lights, gfx::local_shadows const &Shadows, TDynamicObject const *Vehicle) const;
	// the fields of the block of the scene the drawing of the probe takes the sun with: Vehicle the vehicle whose cab is drawn, Tosun the
	// direction towards the sun (the moon), world space, Sunlight its light as the shading gets it (the colour of the light buffer times
	// its intensity), Camera the camera of the main view (world space). no sun until the mask holds the cab
	void setup_sun(gl::scene_ubs &Scene, TDynamicObject const *Vehicle, glm::vec3 const &Tosun, glm::vec3 const &Sunlight, glm::dvec3 const &Camera) const;
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
	// the level of the mipmaps of the albedo the mask is a copy of, and its size
	int mask_level() const;
	int mask_size() const;
	// whether the mask holds the cab being drawn (the drawing of the probe reads it then)
	bool masked() const
	{
		return m_masked;
	}
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
	// the mask holds the cab drawn; the probe was drawn without it and is drawn again in the next frame
	bool m_masked{false};
	bool m_settle{false};
	TDynamicObject const *m_vehicle{nullptr};
	TModel3d const *m_cab{nullptr};
	glm::vec3 m_center{0.f};
	glm::vec3 m_extent{0.f};
	// the light of the lights of the cab the probe was drawn with (their sum), 0 without them
	glm::vec3 m_lamp{0.f};
	unsigned int m_drawnframe{0};
	// statistics
	std::uint32_t m_drawcalls{0};
	std::uint32_t m_updatecount{0};
	double m_cputime{0.0};
	std::chrono::steady_clock::time_point m_cpustart;
};

} // namespace gfx
