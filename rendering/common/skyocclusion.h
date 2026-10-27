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
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <limits>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "gl/ubo.h"

class TDynamicObject;
class TModel3d;
class TSubModel;

namespace scene
{
class basic_region;
}

namespace gfx
{

// the occlusion of the sky and of the sun by the scenery around the camera (Global.gfx_skyvisibility, the track T of
// claude/renderers-plan.md), the part shared by the renderers; the backend (opengl46_sky_occlusion, vk14::sky_occlusion_maps) keeps
// two textures the shading reads and copies to them what this part works out, through the gpu_* methods.
// tunnels, the ground under bridges and roofs, cuttings and the foot of slopes are dark because the geometry around holds back the
// sky and the sun, not because of the environment flags of the tracks (tunnel, canyon), which the renderers leave alone now
// (Global.gfx_tunnelflags brings them back). the maps cover a square around the camera, in tiles of the world; a tile is worked out
// once when it comes into the square and kept while it stays there (the scenery doesn't change, the editor and the streamed terrain
// are noticed by a signature of what the tile holds):
// - the main thread gathers the triangles of the static opaque scenery over a tile: the shapes (the terrain, the heightmap
//   terrain, the roads and the platforms) and the opaque submodels of the models at their nearest level of detail; not the
//   vehicles, the tracks, the traction, the lines, nor what is see-through (the trees);
// - a thread of its own lays them out seen from above: the height of the highest surface over each texel (the top) and of the
//   lowest one facing down (the ceiling: the vault of a tunnel, the deck of a bridge seen from below, a roof); things narrower than
//   a few texels (poles, masts, fences) are taken out of both by an opening (an erosion and a dilation);
// - and works out for each texel, from the heights around it: the share of the sky a surface facing up sees (the irradiance
//   weighed by the cosine) above the top, and under the ceiling, where there's one, a little below it (light comes in under the
//   vault along the tunnel from the portal); and the heights between which the key light (the sun, the moon at night) is let
//   through: above the top it lights what is higher than a height worked out by marching towards it, under the ceiling what is
//   lower than one;
// - the shading (light_common.glsl, sky_occlusion()) reads the height of the top (GL: SKY_OCCLUSION_HEIGHT_TEX, R32F) and the four
//   values above (SKY_OCCLUSION_TEX, RGBA16F) where the fragment is, decides from its height whether it's under the top, and holds
//   back the ambient light, the reflection of the sky and the key light by them. the lamps, the headlights and the light of the cab
//   are direct light and are left as they are;
// - the light around a vehicle (the headlights fade by day, the light of its interior, the cab lamp) is taken from the same maps
//   on the cpu (vehicle_light_level()), with no delay.
// the maps are toroidal: the texel of a world position is its world texel index modulo the size of the map, so a tile which comes
// into the square takes the place of the one which left it on the other side and nothing is moved
class sky_occlusion
{
  public:
	sky_occlusion() = default;
	virtual ~sky_occlusion();
	sky_occlusion(const sky_occlusion &) = delete;
	sky_occlusion &operator=(const sky_occlusion &) = delete;

	// the layout of a level (Global.gfx_skyvisibility 1..3)
	struct layout
	{
		float tile{64.f}; // side of a tile, metres
		int tile_texels{64}; // texels across a tile
		int tiles{8}; // tiles across the map
		int azimuths{12}; // directions the sky is searched in from a texel
		int steps{10}; // samples along each direction
		float reach{48.f}; // how far, metres
		int sun_steps{16}; // samples towards the key light
		float sun_reach{128.f}; // how far, metres
		float texel() const
		{
			return tile / static_cast<float>(tile_texels);
		}
		int side() const
		{
			return tile_texels * tiles;
		}
	};
	static layout layout_of(int const Level);

	// the frame: the square follows the camera (world space), tiles come in and go out, what the thread worked out goes to the
	// textures. Keylight: the direction towards the key light (the sun, the moon at night), world space; it is followed when it
	// moved enough. called by the renderer once a frame, before the passes which read the maps, with the region of the scenery
	void update(scene::basic_region *Region, glm::dvec3 const &Camera, glm::vec3 const &Keylight);
	// the fields of the block of the scene the shading reads the maps with, for a pass seen from Camera (world space); zero when
	// there are no maps
	void setup_scene(gl::scene_ubs &Scene, glm::dvec3 const &Camera) const;
	static void clear_scene(gl::scene_ubs &Scene);
	// the share of the sky (x) and of the key light (y) a point of the world gets, 0..1, as the shading works them out; 1 where
	// there are no maps
	glm::vec2 sample(glm::dvec3 const &Position) const;
	// whether the maps are there
	bool active() const
	{
		return m_side > 0;
	}
	// the maps of the renderer in use, nullptr when there are none
	static sky_occlusion const *current();
	// a scenery is being loaded or let go: the tiles and the signatures go
	void clear();
	// line for the statistics of the debug panel
	std::string info() const;
	// controls, counters and a view of the maps for the debug window
	void draw_debug_ui();

  protected:
	// the textures of Side x Side texels: the heights (four 32 bit floats per texel: the top, the ceiling, the height below which the
	// key light comes in under the ceiling, the height the deep share of the sky under the ceiling is worked out at) and the occlusion
	// (four 16 bit floats: the share of the sky above the top, under the ceiling near it and deep, the height above the top over which
	// the key light comes); false when they can't be made
	virtual bool gpu_allocate(int const Side) = 0;
	virtual void gpu_release() = 0;
	// copies a square of Size x Size texels at X, Y of the textures: Heights four floats per texel, Occlusion four halves per texel,
	// rows along x, one after another along z
	virtual void gpu_upload(int const X, int const Y, int const Size, float const *Heights, std::uint16_t const *Occlusion) = 0;

	int m_side{0};

  private:
	// the triangle of the gathered scenery, x and z relative to the corner of its tile, y in the world; Down: it faces down
	struct triangle
	{
		glm::vec3 a, b, c;
		bool down;
	};
	// what the thread is asked to do
	struct raster_job
	{
		int slot{0};
		std::uint32_t generation{0};
		std::vector<triangle> triangles;
	};
	// what it gives back: the heights (four per texel) and the occlusion of the texels of a tile
	struct tile_result
	{
		int slot{0};
		std::uint32_t generation{0};
		std::vector<float> heights;
		std::vector<std::uint16_t> occlusion;
		double milliseconds{0.0};
	};
	enum class slot_state
	{
		empty, // nothing asked yet: neutral values
		queued, // the triangles went to the thread
		ready // the thread gave its result
	};
	struct slot
	{
		glm::ivec2 tile{std::numeric_limits<int>::min()}; // world tile index
		std::uint32_t generation{0};
		slot_state state{slot_state::empty};
		std::uint64_t signature{0};
		bool regather{false}; // the scenery of the tile changed: gathered again, the old result kept until the new one comes
	};

	void start(int const Level);
	void stop();
	// the slot of a world tile
	int slot_of(glm::ivec2 const &Tile) const;
	// the triangles of the scenery over a tile
	void gather(scene::basic_region *Region, glm::ivec2 const &Tile, std::vector<triangle> &Triangles);
	void gather_model(TModel3d *Model, std::uint32_t const Alpha, glm::dmat4 const &Transform, glm::dvec2 const &Corner, glm::dvec2 const &Far, std::vector<triangle> &Triangles);
	void gather_submodel(TSubModel *Submodel, std::uint32_t const Alpha, glm::dmat4 const &Transform, glm::dvec2 const &Corner, glm::dvec2 const &Far, std::vector<triangle> &Triangles);
	// what the scenery over a tile is made of, cheap to work out: a change of it means the tile is gathered again
	std::uint64_t signature(scene::basic_region *Region, glm::ivec2 const &Tile);
	// the values of a tile nothing is known of yet, in the textures and in the copy of the cpu
	void neutral(int const Slot);
	void apply(tile_result &Result);

	// the thread
	void work();
	void rasterize(raster_job &Job);
	void filter(int const Slot);
	void occlude(int const Slot, tile_result &Result);
	bool worker_slot_valid(glm::ivec2 const &Tile) const;

	layout m_layout;
	int m_level{0};
	bool m_sun{true};
	std::vector<slot> m_slots;
	glm::ivec2 m_cameratile{0};
	glm::dvec3 m_camera{0.0};
	glm::vec3 m_keylight{0.f, 1.f, 0.f}; // the direction the thread was given last
	// the copy of the maps the cpu samples (sample()): the heights, and the occlusion as floats
	std::vector<glm::vec4> m_heights;
	std::vector<glm::vec4> m_occlusion;
	// neutral tiles, uploaded when a tile comes into the square
	std::vector<glm::vec4> m_neutralheights;
	std::vector<std::uint16_t> m_neutralocclusion;
	std::size_t m_checkslot{0}; // the slot whose signature is checked next
	scene::basic_region const *m_region{nullptr};

	// shared with the thread, under m_mutex
	std::mutex m_mutex;
	std::condition_variable m_wake;
	bool m_quit{false};
	std::deque<raster_job> m_jobs;
	std::deque<tile_result> m_results;
	std::vector<glm::ivec2> m_workertiles; // the world tile of each slot, as the thread knows it
	std::vector<std::uint32_t> m_workergenerations;
	std::vector<bool> m_workerraster; // the slot holds the rasterized scenery of its tile
	std::vector<bool> m_workerdirty; // the occlusion of the slot is to be worked out again
	glm::ivec2 m_workercamera{0};
	glm::vec3 m_workerkeylight{0.f, 1.f, 0.f};
	bool m_workersun{true};
	std::thread m_thread;
	// the maps of the thread: the top and the ceiling as rasterized, and opened
	std::vector<float> m_rawtop, m_rawceiling, m_top, m_ceiling;

	// statistics
	std::uint32_t m_gathered{0};
	std::uint32_t m_triangles{0};
	std::uint32_t m_regathered{0};
	std::uint32_t m_uploaded{0};
	double m_gathertime{0.0};
	double m_workertime{0.0}; // under m_mutex
	std::uint32_t m_workertiles_done{0}; // under m_mutex
	int m_pending{0};
	std::uint64_t m_previewimage{0};
	int m_previewmode{0};
	std::vector<std::uint8_t> m_previewrgba;
};

// the strength the shading of a vehicle gives the sun and the ambient light (walk_sunlight()), from the environment flags of the
// tracks under it: only with Global.gfx_tunnelflags; 0 otherwise (the light isn't changed)
float vehicle_shade(TDynamicObject const *Vehicle);
// the light around a vehicle, 0 (none: the depth of a tunnel) .. 1 (the open sky), which fades its headlights by day and the
// lights of its interior: with Global.gfx_tunnelflags from the environment flags of the tracks under it (TDynamicObject::fShade),
// otherwise from the maps of the occlusion around the camera (1 without them, or away from the camera)
float vehicle_light_level(TDynamicObject const *Vehicle);

} // namespace gfx
