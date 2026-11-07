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
#include <functional>
#include <unordered_map>
#include <vector>
#include <glm/glm.hpp>

#include "rendering/renderer.h"
#include "scene/scenenode.h"

namespace gfx
{

// grass of the heightmap terrain, whichever api draws it: blades made up by the vertex shader (grass.vert) where the grass is
// painted, in the chunks near the camera.
//
// a chunk is cut into tiles; each tile visible within the reach of the grass is drawn as instances of a blade, as many as
// its densest kind and its distance ask for. a blade finds its place, its kind and its shape from its number, so the same
// blades stand at the same places in every frame. further away the blades thin out and grow wider, and fade out at the end
// of the reach. the heights, the densities and the holes of a chunk are kept in textures, made by the renderer (the gpu_
// functions) when the chunk comes near; what is drawn in a frame is worked out here (prepare())
class grass_field
{
  public:
	virtual ~grass_field() = default;

	// see gfx_renderer::Terrain_Grass()
	void update(std::vector<gfx::grass_kind> const &Kinds, std::vector<gfx::grass_chunk> const &Chunks);
	// drops everything made on the gpu, e.g. when the context goes
	void clear();
	bool empty() const
	{
		return m_chunks.empty();
	}

	static int constexpr tiles{16}; // along a side of a chunk

	// what a frame draws (the uniforms of grass.vert): the same for all draws
	struct frame_draw
	{
		glm::vec4 lod{0.f}; // grass_lod
		glm::vec4 season{0.f}; // grass_season
		glm::vec4 wind{0.f}; // grass_wind
	};
	// a chunk with tiles seen, drawn relative to its corner
	struct chunk_draw
	{
		std::uint64_t id{0}; // of the chunk, as the renderer got it in gpu_upload()
		glm::dvec3 offset{0.0}; // of the corner from the camera, the heights as they are
		glm::vec4 chunk{0.f}; // grass_chunk
		glm::vec4 camera{0.f}; // grass_camera
		std::array<glm::vec4, 4> base{}, tip{}, shape{}, dry{};
		glm::vec4 share{0.f};
		std::size_t first{0}; // tiles drawn, in tile_draws()
		std::size_t count{0};
	};
	// a tile drawn: Count instances of a strip of 2 * Segments + 1 vertices
	struct tile_draw
	{
		glm::vec4 tile{0.f}; // grass_tile
		glm::vec4 part{0.f}; // grass_part
		int count{0};
		int segments{0};
	};
	// the draws of the grass seen from Camera: false when there is nothing to draw (grass off, no chunks or kinds), the draws of
	// the last call are left then. Visible: whether the bounds of a tile are seen
	bool prepare(glm::dvec3 const &Camera, std::function<bool(scene::bounding_area const &)> const &Visible);
	frame_draw const &frame() const
	{
		return m_frame;
	}
	std::vector<chunk_draw> const &chunk_draws() const
	{
		return m_chunkdraws;
	}
	std::vector<tile_draw> const &tile_draws() const
	{
		return m_tiledraws;
	}
	// the triangles of the draws prepared
	std::size_t triangles() const
	{
		return m_triangles;
	}

  protected:
	// the textures of the chunk Id made of Source: heights (R32F, the grid points, cells + 1 along a side), densities (RGBA8, the
	// densities of the kinds side by side, samples + 1 along a side), holes (R8, a single texel of 0 without holes)
	virtual void gpu_upload(std::uint64_t const Id, gfx::grass_chunk const &Source) = 0;
	// the textures of the chunk go; nothing when it has none
	virtual void gpu_release(std::uint64_t const Id) = 0;

  private:
	struct tile
	{
		float low{0.f}, high{0.f}; // of the ground
		std::array<float, 4> most{}; // the highest density of each kind of the chunk within the tile, 0..1
	};
	struct chunk
	{
		std::uint64_t version{0};
		glm::dvec2 corner{0.0};
		float size{0.f};
		int cells{0};
		int samples{0};
		std::array<int, 4> kinds{-1, -1, -1, -1};
		std::array<tile, tiles * tiles> parts;
		bool uploaded{false}; // has its textures
		bool listed{false}; // in the last list given
	};

	void upload(std::uint64_t const Id, chunk &Chunk, gfx::grass_chunk const &Source);
	void release(std::uint64_t const Id, chunk &Chunk);

	std::vector<gfx::grass_kind> m_kinds;
	std::unordered_map<std::uint64_t, chunk> m_chunks;
	frame_draw m_frame;
	std::vector<chunk_draw> m_chunkdraws;
	std::vector<tile_draw> m_tiledraws;
	std::size_t m_triangles{0};
};

} // namespace gfx
