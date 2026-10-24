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
#include <memory>
#include <unordered_map>
#include <vector>
#include <glm/glm.hpp>

#include "rendering/renderer.h"
#include "gl/shader.h"
#include "gl/vao.h"

class opengl_camera;

// grass of the heightmap terrain for the opengl 4.6 renderer: blades made up by the vertex shader (grass.vert) where the
// grass is painted, in the chunks near the camera.
//
// a chunk is cut into tiles; each tile visible within the reach of the grass is drawn as instances of a blade, as many as
// its densest kind and its distance ask for. a blade finds its place, its kind and its shape from its number, so the same
// blades stand at the same places in every frame. further away the blades thin out and grow wider, and fade out at the end
// of the reach. the heights, the densities and the holes of a chunk are kept in textures, made when the chunk comes near
class opengl46_grass
{
  public:
	opengl46_grass() = default;
	~opengl46_grass();

	// see gfx_renderer::Terrain_Grass()
	void update(std::vector<gfx::grass_kind> const &Kinds, std::vector<gfx::grass_chunk> const &Chunks);
	// draws the grass seen by the camera. View: the modelview of the pass with the camera at its origin; Modelview: hands a
	// modelview to the model block of the draws which follow. returns: triangles drawn
	std::size_t render(opengl_camera const &Camera, glm::mat4 const &View, std::function<void(glm::mat4 const &)> const &Modelview);
	// drops everything made on the gpu, e.g. when the context goes
	void clear();
	bool empty() const { return m_chunks.empty(); }

	static int constexpr tiles{16}; // along a side of a chunk

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
		GLuint heights{0}; // R32F, the grid points
		GLuint density{0}; // RGBA8, the densities of the kinds
		GLuint holes{0}; // R8, a single texel of 0 without holes
		std::array<tile, tiles * tiles> parts;
		bool listed{false}; // in the last list given
	};

	bool init();
	void upload(chunk &Chunk, gfx::grass_chunk const &Source);
	void release(chunk &Chunk);

	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used
	std::unique_ptr<gl::program> m_program;
	std::unique_ptr<gl::vao> m_vao; // without attributes, the blades are made of the numbers of their vertices
	std::vector<gfx::grass_kind> m_kinds;
	std::unordered_map<std::uint64_t, chunk> m_chunks;
	// locations of the uniforms of the program
	struct locations
	{
		GLint tile{-1}, part{-1}, chunk{-1}, camera{-1}, lod{-1}, wind{-1};
		GLint base{-1}, tip{-1}, shape{-1}, share{-1}, dry{-1}, season{-1};
	} m_uniforms;
};
