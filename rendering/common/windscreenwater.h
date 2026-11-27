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
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
#include <glm/glm.hpp>

#include "gl/ubo.h"
#include "interfaces/ITexture.h"
#include "utilities/Classes.h"

struct opengl_material;
class TModel3d;
class TSubModel;
class TDynamicObject;

namespace gfx
{

// the water on the windscreens of the cab of the train, simulated on the gpu, whichever api runs it.
//
// a glass is a material of mat_rain_windscreen (one with a mask of the wipers) in the model of the cab of the train, up to
// gl::WATER_SLOTS of them; the material is told its slot in its param water_slot (1 + the slot, the drawing of the glass reads
// the water of the slot where it's there, mat_rain_windscreen.frag). the water lies in the coordinates of the texture of the glass
// (uv), as the mask does. the map of a glass is made here from the triangles of the model with its material: for the texels of
// the uv on the glass, the step along the glass (metres, the space of the model of the cab) for a step of u and of v, and which
// way is out of the cab; the passes of the gpu bake the mask of the wipers into it. once a frame the step of the simulation is
// worked out here (the time, the rain or snow, gravity and the air against the cab in the space of its model, the blades) and
// the passes run it (water_*.comp: the landing drops, the drops moving and carried by the blades, the grids, the drops flowing
// together); what they keep and the glasses read is the storage of the water (water.glsl, gl::WATER_*_OFFSET)
class windscreen_water
{
  public:
	// what the renderer gives: the materials of its manager, which get their slots
	struct host
	{
		std::function<opengl_material *(material_handle)> material;
	};

	windscreen_water() = default;
	virtual ~windscreen_water() = default;
	windscreen_water(windscreen_water const &) = delete;
	windscreen_water &operator=(windscreen_water const &) = delete;

	void set_host(host Host)
	{
		m_host = std::move(Host);
	}
	// whether the passes could be made
	bool available();
	// once a frame, before the colour passes: the cab of the train followed, the step of the simulation run. Seconds: of the
	// simulation since the frame before (0 when it's paused)
	void update(double Seconds);
	// the materials let go of their slots, the glasses forgotten (the storage stays as it is, unused)
	void release();
	std::string info() const;

  protected:
	// the glasses forgotten and the programs taken as not made yet (the renderer let go of them), made again when next needed
	void reset();
	// the programs and the storage; false: the simulation can't be used, the glasses make their water up
	virtual bool gpu_init() = 0;
	// the slot set up for a glass: its drops, its grid and its paths cleared, its header saying it isn't simulated (until the
	// first step), the axes of its map (two vec4 a texel, gl::WATER_MAP x WATER_MAP texels, see water.glsl) put in the storage
	virtual void gpu_setup(std::size_t Slot, std::vector<glm::vec4> const &Axes) = 0;
	// the mask of the wipers of a glass (a texture of the renderer) baked into its map; false when the texture isn't ready yet
	// (tried again with the next frame)
	virtual bool gpu_bake(std::size_t Slot, texture_handle Mask, gl::water_ubs const &Block) = 0;
	// the step of the simulation of the glasses Block says are simulated; Tries: the most landings tried on a glass
	virtual void gpu_step(gl::water_ubs const &Block, std::uint32_t Tries) = 0;

  private:
	struct glass
	{
		material_handle material{null_handle};
		texture_handle mask{null_handle};
		float densest{0.f}; // the most metres squared a unit of the uv holds
		float area{0.f}; // metres squared of the glass
		double landings{0.0}; // the share of a landing left over from the steps before
		bool baked{false};
	};

	// the glasses of the model of a cab found and set up
	void build(TModel3d &Cab);
	// the triangles of a glass laid into its map; Inside: a point inside the cab
	void rasterize(TSubModel *Submodel, material_handle Material, glm::vec3 const &Inside, std::vector<glm::vec4> &Axes) const;
	// the slots given to the materials of the glasses (every frame: a material made anew takes its default params)
	void assign(bool Assigned);

	host m_host;
	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used
	TDynamicObject const *m_vehicle{nullptr};
	TModel3d const *m_cab{nullptr};
	std::array<glass, gl::WATER_SLOTS> m_glasses{};
	std::size_t m_glasscount{0};
	glm::vec4 m_bladesbefore{-1.f};
	double m_time{0.0}; // seconds the simulation runs
	std::uint32_t m_step{0};
	std::uint32_t m_tries{0}; // landings tried in the last step, all glasses
};

} // namespace gfx
