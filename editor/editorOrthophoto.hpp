/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include <glm/glm.hpp>

#include "interfaces/ITexture.h" // texture_handle

// Editor aid: aerial orthophoto tiles from geoportal.gov.pl (GUGiK WMS), drawn as a translucent
// horizontal layer around the camera.
//
// Tiles are aligned to an absolute 256 m grid in PUWG 1992 (EPSG:2180), so the disk cache is shared
// between sceneries. The scenery is placed on that grid through the PUWG coordinates of its (0,0,0)
// point; MaSzyna's +Z points north and +X points west, hence easting = E0 - x, northing = N0 + z.
//
// Downloading, cache I/O and image decoding run on worker threads; the main thread only uploads
// finished images (a bounded amount per frame) and emits the overlay geometry. The overlay goes into
// the ImGui background draw list, like the other editor viewport overlays, so it works with every
// renderer backend, but it isn't depth-tested against the scene.
class editor_orthophoto
{
  public:
	struct config
	{
		double north{0.0};      // PUWG 1992 X (northing) of the scenery origin
		double east{0.0};       // PUWG 1992 Y (easting) of the scenery origin
		int radius{2};          // tiles loaded around the camera tile (Chebyshev distance)
		float height{0.0f};     // world Y of the layer
		float opacity{0.6f};    // 0..1
		int year{0};            // 0 = newest available imagery, otherwise newest imagery taken up to the end of that year
		bool hires{false};      // 4096 px tiles from the high-resolution product next to the camera, where it exists
	};

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
		int level{0}; // requested texture size in pixels
		source src;
		std::uint32_t generation{0};
	};

	struct result
	{
		tile_key key;
		int level{0};
		std::uint32_t generation{0};
		bool ok{false};
		bool empty{false}; // no imagery for this place/year: nothing to draw
		int width{0};
		int height{0};
		std::vector<std::uint8_t> pixels; // RGBA8, first row = north edge
	};

	struct tile
	{
		texture_handle texture{null_handle};
		int level{0};     // size of the imagery currently held (0 = nothing yet)
		int requested{0}; // level of the job scheduled for this tile (0 = none)
		bool empty{false};
		int failures{0};
		double retry_at{0.0}; // steady clock seconds
	};

	static int level_for(int Distance, bool Hires);
	tile_key camera_tile(glm::dvec3 const &Camera) const;
	void schedule(tile_key const &Key, tile &Tile, int Level);
	std::size_t apply(result &Result, double Now); // returns bytes uploaded
	void release_tile(tile &Tile);
	texture_handle acquire_texture();
	void start_workers();
	void worker();

	config m_config;
	bool m_enabled{false};
	std::map<tile_key, tile> m_tiles; // main thread only
	tile_key m_camera_tile{0, 0};
	std::vector<texture_handle> m_free_textures; // pool of texture slots, reused between tiles
	int m_texture_count{0};

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
