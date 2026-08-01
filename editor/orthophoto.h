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
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "maj0sted/editor/tile_cache.hpp"

namespace editor
{

// orthophoto backdrop for the plan view. the tiling scheme, the cache and the retry policy come
// from the track layout library, which already speaks EPSG:2180; this adds what a native build
// needs and the web one got from the browser - the download itself, the decode and the texture.
//
// downloads run on worker threads, but nothing else does: finished transfers are handed back on
// the thread that calls collect(), so both the library's cache and the graphics API are only ever
// touched from there.
class orthophoto_source
{

  public:
	struct ready_tile
	{
		maj0sted::editor::TileBBox box;
		unsigned int texture{0};
	};

	// default: Geoportal ortophoto on the fine 100 m grid. pass topo config and a coarse
	// grid zoom (e.g. 1000) for the zoomed-out topographic underlay
	explicit orthophoto_source(maj0sted::editor::WmsConfig Config = maj0sted::editor::WmsConfig::geoportal_ortho(), int const Gridzoom = maj0sted::editor::TileGrid::kLevel,
	                           std::string Cachesubdir = "orto");
	~orthophoto_source();
	orthophoto_source(orthophoto_source const &) = delete;
	orthophoto_source &operator=(orthophoto_source const &) = delete;

	// kicks off fetches for the view (capped) and returns every already-uploaded tile that still
	// intersects it - including ones outside the fetch window, so a zoom cannot blink them away.
	// tiles still on their way simply aren't in the answer yet; call again next frame
	std::vector<ready_tile> collect(maj0sted::editor::TileBBox const &View, int const Maxtiles = 400);
	// how many tiles are still being fetched, for the panel to report
	std::size_t pending() const;
	// how many tiles gave up after exhausting their retries
	std::size_t failed() const;

  private:
	// types
	struct transfer
	{
		std::string url;
		maj0sted::editor::TileService::DoneFn done;
		bool ok{false};
		std::vector<std::uint8_t> bytes;
		std::string content_type;
	};
	// methods
	void worker();
	// decodes the cached bytes of a tile and uploads them; returns 0 when the image is unusable
	unsigned int upload(maj0sted::editor::TileKey const &Key);
	// imagery of a given piece of ground never changes, so a fetched tile is kept on disk and the
	// next session starts with it instead of going back to a service that is slow and flaky
	std::string tile_path(maj0sted::editor::TileKey const &Key) const;
	bool load_from_disk(maj0sted::editor::TileKey const &Key);
	void save_to_disk(maj0sted::editor::TileKey const &Key);
	// members
	maj0sted::editor::TileService m_service;
	int m_gridzoom{maj0sted::editor::TileGrid::kLevel};
	std::string m_cachesubdir{"orto"};
	std::unordered_map<maj0sted::editor::TileKey, unsigned int, maj0sted::editor::TileKeyHash> m_textures;

	mutable std::mutex m_mutex;
	std::condition_variable m_wakeup;
	std::deque<transfer> m_queued;
	std::deque<transfer> m_finished;
	std::vector<std::thread> m_workers;
	std::atomic<bool> m_quitting{false};
	std::atomic<std::size_t> m_inflight{0};
	std::unordered_set<maj0sted::editor::TileKey, maj0sted::editor::TileKeyHash> m_failedkeys;
};

// a single WMS image covering an arbitrary box, for views too coarse for the tile grid - the whole
// country will not fit into 100 m cells. one request is in flight at a time: asking for a different
// box supersedes whatever was being fetched
class wms_image
{

  public:
	explicit wms_image(maj0sted::editor::WmsConfig Config);
	~wms_image();
	wms_image(wms_image const &) = delete;
	wms_image &operator=(wms_image const &) = delete;

	// asks for the given box at the given size and returns the texture currently available, which
	// may still be the previous box's until the new one arrives; 0 when there is nothing to show yet
	unsigned int texture_for(maj0sted::editor::TileBBox const &Box, int const Pixels);
	// the box the returned texture actually covers
	maj0sted::editor::TileBBox const &covered() const { return m_covered; }
	bool loading() const { return m_loading.load(); }

  private:
	void worker();

	maj0sted::editor::WmsConfig m_config;
	maj0sted::editor::TileBBox m_requested{};
	maj0sted::editor::TileBBox m_inflightbox{};
	maj0sted::editor::TileBBox m_covered{};
	unsigned int m_texture{0};

	mutable std::mutex m_mutex;
	std::condition_variable m_wakeup;
	std::string m_url;
	bool m_haswork{false};
	std::vector<std::uint8_t> m_bytes;
	bool m_hasresult{false};
	std::thread m_worker;
	std::atomic<bool> m_quitting{false};
	std::atomic<bool> m_loading{false};
};

} // namespace editor
