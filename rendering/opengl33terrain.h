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
#include <memory>
#include <unordered_map>
#include <vector>

#include "rendering/renderer.h"
#include "model/material.h"
#include "model/Texture.h"
#include "gl/framebuffer.h"
#include "gl/postfx.h"

// materials of the heightmap terrain for the opengl 3.3 renderer (shader mat_terrain).
// the textures of the layer materials are copied into texture arrays shared by all chunks; each chunk gets a material
// of its own, with textures holding the weights of its layers and a table of what its layers are
class opengl33_terrain_materials
{
  public:
	opengl33_terrain_materials(material_manager &Materials, texture_manager &Textures) : m_materials(Materials), m_textures(Textures) {}

	// see gfx_renderer::Terrain_Material. returns null_handle if the terrain shader can't be used
	material_handle make(material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement);
	void release(material_handle const Material);
	// copies the textures of new layers into the arrays; to be called before the frame is drawn
	void update();

  private:
	struct layer
	{
		material_handle material{null_handle};
		float size{0.f}; // as given by the chunk
		float repeat{8.f}; // metres the textures are repeated at
		int index{0}; // in the arrays
		bool normalmap{false};
		bool specgloss{false};
		float reflection{0.f};
		bool copied{false};
		int attempts{0};
	};
	struct texture_array
	{
		texture_handle handle{null_handle};
		int size{0};
		GLint format{0};
		bool srgb{false};
	};
	struct slot
	{
		material_handle material{null_handle};
		std::array<texture_handle, 2> weights{null_handle, null_handle};
		texture_handle table{null_handle};
		bool used{false};
	};

	bool init();
	texture_handle make_texture(std::string const &Name, GLenum const Target);
	void make_storage(texture_array &Array, int const Layers);
	int layer_index(gfx::terrain_layer const &Layer);
	bool copy_layer(layer &Layer);
	std::size_t acquire_slot(material_handle const Reuse);

	material_manager &m_materials;
	texture_manager &m_textures;
	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used
	texture_handle m_white{null_handle};
	std::array<texture_array, 3> m_arrays; // diffuse, normal maps, specular/gloss
	int m_capacity{0}; // layers the arrays have room for
	std::vector<layer> m_layers;
	bool m_pending{false};
	std::vector<slot> m_slots;
	std::vector<std::size_t> m_free;
	std::unordered_map<material_handle, std::size_t> m_slotof;
	std::unique_ptr<gl::postfx> m_copy;
	std::unique_ptr<gl::framebuffer> m_framebuffer;
};
