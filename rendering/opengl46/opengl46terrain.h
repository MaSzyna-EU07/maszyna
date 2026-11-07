/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <memory>

#include "rendering/common/terrainmaterials.h"
#include "gl/framebuffer.h"
#include "gl/postfx.h"

// materials of the heightmap terrain for the opengl 4.6 renderer (shader mat_terrain, mat_terrain_holes for chunks with holes):
// the textures of gfx::terrain_materials as opengl textures, the layer maps copied into the arrays by a draw of the copy shader
class opengl46_terrain_materials : public gfx::terrain_materials
{
  public:
	opengl46_terrain_materials(material_manager &Materials, texture_manager &Textures) : gfx::terrain_materials(Materials, Textures), m_gltextures(Textures) {}

  protected:
	bool gpu_init() override;
	void gpu_texture(texture_source &Texture, texture_kind const Kind) override;
	void gpu_white(texture_source &Texture) override;
	void gpu_storage(texture_array const &Array, int const Layers, bool const Fresh) override;
	void gpu_blank(slot const &Slot) override;
	void gpu_weights(texture_handle const Texture, int const Side, std::uint8_t const *Texels) override;
	void gpu_table(texture_handle const Texture, float const *Table) override;
	void gpu_holes(texture_handle const Texture, int const Side, std::uint8_t const *Samples) override;
	void gpu_begin_copies() override;
	void gpu_copy(texture_array const &Array, int const Layer, texture_source *Source, std::array<float, 4> const &Default) override;
	void gpu_end_copies(bool const Copied) override;
	void gpu_changed() override;

  private:
	texture_manager &m_gltextures;
	std::unique_ptr<gl::postfx> m_copy;
	std::unique_ptr<gl::framebuffer> m_framebuffer;
	// the state of opengl the copies change, given back after them
	struct
	{
		GLboolean blend{GL_FALSE};
		GLboolean scissor{GL_FALSE};
		GLboolean cull{GL_FALSE};
		GLboolean depth{GL_FALSE};
		GLboolean srgb{GL_FALSE};
	} m_saved;
};
