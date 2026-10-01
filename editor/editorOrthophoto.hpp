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
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "interfaces/ITexture.h"    // texture_handle
#include "utilities/Classes.h"      // material_handle
#include "rendering/geometrybank.h" // gfx::geometry_handle

namespace scene
{
class basic_region;
class basic_section;
class basic_cell;
} // namespace scene

// Editor aid: aerial orthophoto tiles from geoportal.gov.pl (GUGiK WMS), shown around the camera either
// on a horizontal plane or fitted onto the terrain.
//
// Tiles are aligned to an absolute 256 m grid in PUWG 1992 (EPSG:2180), so the disk cache is shared
// between sceneries. The scenery is placed on that grid through the PUWG coordinates of its (0,0,0)
// point; MaSzyna's +Z points north and +X points west, hence easting = E0 - x, northing = N0 + z.
//
// Downloading, cache I/O and image decoding run on worker threads; the main thread only uploads
// finished images (a bounded amount per frame) and emits the geometry. By default the layer goes into
// the ImGui background draw list, like the other editor viewport overlays: translucent and drawn over
// everything. Optionally it is put into the scene instead (shapes in the region sections, like the editor
// terrain, or translucent shapes in the cells when it isn't fully opaque), so tracks, models and terrain in
// front of it cover it.
class editor_orthophoto
{
  public:
	struct config
	{
		double north{0.0};      // PUWG 1992 X (northing) of the scenery origin
		double east{0.0};       // PUWG 1992 Y (easting) of the scenery origin
		int radius{2};          // tiles loaded around the camera tile (Chebyshev distance)
		float height{0.0f};     // world Y of the layer (with drape: only where no ground was found)
		float opacity{0.6f};    // 0..1
		int year{0};            // 0 = newest available imagery, otherwise newest imagery taken up to the end of that year
		bool hires{false};      // 4096 px tiles from the high-resolution product next to the camera, where it exists
		bool drape{true};       // fit the imagery onto the ground geometry instead of a flat plane
		float lift{0.05f};      // with drape: distance above the ground (in the scene it has to clear the ground to be seen)
		bool in_scene{false};   // render as opaque scene geometry, covered by whatever is in front of it
	};

	using world_triangle = std::array<glm::dvec3, 3>;
	// supplies the ground triangles overlapping an XZ rectangle (Min/Max are x,z); used to drape the imagery
	using ground_query = std::function<void(glm::dvec2 const &Min, glm::dvec2 const &Max, std::vector<world_triangle> &Out)>;

	struct statistics
	{
		int resident{0}; // tiles with imagery on screen
		int loading{0};  // tiles waiting for or being processed by a worker
		int failed{0};   // tiles whose last attempt failed
	};

	static constexpr double tile_size{256.0}; // metres per tile side
	static constexpr int max_radius{10};
	static constexpr int oldest_year{1995}; // first year offered by the archival service

	editor_orthophoto() = default;
	~editor_orthophoto();
	editor_orthophoto(editor_orthophoto const &) = delete;
	editor_orthophoto &operator=(editor_orthophoto const &) = delete;

	bool enabled() const { return m_enabled; }
	// disabling drops the queue and returns the gpu memory of the resident tiles
	void enabled(bool State);
	config const &settings() const { return m_config; }
	// changes of year / resolution invalidate resident tiles, the rest applies immediately
	void settings(config const &Config);
	void ground_source(ground_query Query) { m_ground = std::move(Query); }
	// samples the ground again (after it was edited); tiles are refitted progressively
	void refit();
	// takes the scene geometry out of the region (e.g. when the editor is left); it's re-created on the next update
	void detach_scene();
	// true for geometry created by the layer, so ground queries can skip it
	static bool owns(gfx::geometry_handle const &Geometry);

	// main thread, once per frame: schedules tiles around the camera and uploads finished ones
	void update(glm::dvec3 const &Camera);
	// main thread, inside an ImGui frame: emits the overlay into the background draw list.
	// ViewProjection must map camera-relative world positions (world - CameraPos) to clip space.
	void draw(glm::mat4 const &ViewProjection, glm::dvec3 const &CameraPos, float ScreenWidth, float ScreenHeight) const;
	// forgets pending (not yet started) downloads, e.g. when the editor is left
	void cancel_pending();
	// clears failure state so failed tiles are attempted again
	void retry_failed();

	statistics stats() const;
	// false when the build has no http client; cached tiles are still shown
	static bool can_download();
	static std::string cache_directory();

  private:
	using tile_key = std::pair<int, int>; // (easting index, northing index) on the PUWG tile grid

	struct source
	{
		int year{0};
		bool hires{false};
	};

	struct job
	{
		tile_key key;
		int level{0};        // requested texture size in pixels
		std::uint8_t alpha{255}; // baked into the texture, for translucency in the scene
		source src;
		std::uint32_t generation{0};
	};

	struct result
	{
		tile_key key;
		int level{0};
		std::uint8_t alpha{255};
		std::uint32_t generation{0};
		bool ok{false};
		bool empty{false}; // no imagery for this place/year: nothing to draw
		int width{0};
		int height{0};
		std::vector<std::uint8_t> pixels; // RGBA8, first row = north edge
	};

	struct texture_slot
	{
		int index{-1}; // names the texture and its material
		texture_handle texture{null_handle};
		material_handle material{null_handle}; // created on first use by the scene geometry
	};

	struct tile
	{
		texture_slot slot;
		int level{0};     // size of the imagery currently held (0 = nothing yet)
		int requested{0}; // level of the job scheduled for this tile (0 = none)
		std::uint8_t alpha{255};           // alpha of the imagery currently held
		std::uint8_t requested_alpha{255}; // alpha of the scheduled job
		bool empty{false};
		int failures{0};
		double retry_at{0.0}; // steady clock seconds
		// ground fit
		bool grounded{false};             // ground was sampled
		std::vector<float> heights;       // (ground_grid+1)^2 world Y, rows north->south, columns west->east; empty if no ground
		float height_min{0.0f};
		float height_max{0.0f};
		std::vector<glm::dvec3> surface;  // upward facing ground triangles clipped to the tile (3 vertices each)
		// scene geometry
		scene::basic_section *section{nullptr}; // section holding the shape (nullptr = not in the scene)
		scene::basic_cell *cell{nullptr};       // set instead of the section's shape list when the shape is translucent
		gfx::geometry_handle geometry{0, 0};
		std::size_t capacity{0};                // vertex count of the geometry chunk
		bool scene_dirty{true};                 // the shape needs to be rebuilt
	};

	static int level_for(int Distance, bool Hires);
	tile_key camera_tile(glm::dvec3 const &Camera) const;
	glm::dvec2 tile_corner(tile_key const &Key) const; // world (x,z) of the north-west corner
	void schedule(tile_key const &Key, tile &Tile, int Level, std::uint8_t Alpha);
	std::uint8_t texture_alpha() const; // alpha the textures should carry with the current settings
	std::size_t apply(result &Result, double Now); // returns bytes uploaded
	void release_tile(tile &Tile);
	texture_slot acquire_slot();
	void sample_ground(tile_key const &Key, tile &Tile);
	void build_shape(tile_key const &Key, tile &Tile);
	void remove_shape(tile &Tile);
	void release_geometry(tile &Tile);
	void update_scene();
	void start_workers();
	void worker();

	config m_config;
	bool m_enabled{false};
	std::map<tile_key, tile> m_tiles; // main thread only
	tile_key m_camera_tile{0, 0};
	std::vector<texture_slot> m_free_slots; // pool of textures, reused between tiles
	int m_texture_count{0};
	ground_query m_ground;
	scene::basic_region *m_region{nullptr};                  // region the scene geometry was put into
	std::multimap<std::size_t, gfx::geometry_handle> m_free_chunks; // geometry chunks by capacity, reused between tiles
	// the scene opacity is baked into the textures, which means reading the tiles again; applied once the value settles
	std::uint8_t m_alpha{255};         // in effect
	std::uint8_t m_alpha_pending{255}; // latest requested
	double m_alpha_changed{0.0};       // when the requested value last changed

	// shared with the workers, guarded by m_mutex
	mutable std::mutex m_mutex;
	std::condition_variable m_wakeup;
	std::deque<job> m_queue;
	std::deque<result> m_done;
	tile_key m_focus{0, 0}; // workers pick the queued tile closest to it

	std::atomic<bool> m_stop{false};
	std::atomic<std::uint32_t> m_generation{1};
	std::vector<std::thread> m_workers;
};
