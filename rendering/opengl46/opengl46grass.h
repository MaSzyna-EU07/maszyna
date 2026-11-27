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
#include <glm/glm.hpp>

#include "rendering/common/grassfield.h"
#include "gl/shader.h"
#include "gl/vao.h"

class opengl_camera;

// grass of the heightmap terrain for the opengl 4.6 renderer (gfx::grass_field): the textures of the chunks, and the draws of
// the tiles seen, instances of a blade made up by grass.vert
class opengl46_grass : public gfx::grass_field
{
  public:
	opengl46_grass() = default;
	~opengl46_grass() override;

	// draws the grass seen by the camera. View: the modelview of the pass with the camera at its origin; Modelview: hands a
	// modelview to the model block of the draws which follow. returns: triangles drawn
	std::size_t render(opengl_camera const &Camera, glm::mat4 const &View, std::function<void(glm::mat4 const &)> const &Modelview);

  protected:
	void gpu_upload(std::uint64_t const Id, gfx::grass_chunk const &Source) override;
	void gpu_release(std::uint64_t const Id) override;

  private:
	struct chunk_textures
	{
		GLuint heights{0}; // R32F, the grid points
		GLuint density{0}; // RGBA8, the densities of the kinds
		GLuint holes{0}; // R8, a single texel of 0 without holes
	};

	bool init();

	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used
	std::unique_ptr<gl::program> m_program;
	std::unique_ptr<gl::vao> m_vao; // without attributes, the blades are made of the numbers of their vertices
	std::unordered_map<std::uint64_t, chunk_textures> m_textures;
	// locations of the uniforms of the program
	struct locations
	{
		GLint tile{-1}, part{-1}, chunk{-1}, camera{-1}, lod{-1}, wind{-1};
		GLint base{-1}, tip{-1}, shape{-1}, share{-1}, dry{-1}, season{-1};
	} m_uniforms;
};
