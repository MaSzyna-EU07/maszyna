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

namespace gfx
{

// materials of the heightmap terrain (shader mat_terrain, mat_terrain_holes for chunks with holes), whichever api draws them.
// the textures of the layer materials are copied into texture arrays shared by all chunks; each chunk gets a material
// of its own, with textures holding the weights of its layers and a table of what its layers are.
// what the layers and the chunks are is kept here; the textures are made and filled by the renderer (the gpu_ functions)
class terrain_materials
{
  public:
	terrain_materials(material_manager &Materials, texture_bank &Textures) : m_materials(Materials), m_textures(Textures) {}
	virtual ~terrain_materials() = default;

	// see gfx_renderer::Terrain_Material. returns null_handle if the terrain shader can't be used
	material_handle make(material_handle const Reuse, std::vector<gfx::terrain_layer> const &Layers, int const Samples, std::uint8_t const *Weights, glm::vec3 const &Placement,
	                     int const HoleSamples = 0, std::uint8_t const *Holes = nullptr);
	void release(material_handle const Material);
	// copies the textures of new layers into the arrays; to be called before the frame is drawn
	void update();
	// the layer materials which change with the season or the weather were reloaded, their textures are copied again
	void on_environment_change();

  protected:
	static constexpr std::size_t max_layers{8};

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
		bool srgb{false};
	};
	struct slot
	{
		material_handle material{null_handle};
		std::array<texture_handle, 2> weights{null_handle, null_handle};
		texture_handle table{null_handle};
		texture_handle holes{null_handle}; // where the ground is cut, R8; a single texel of 0 without holes
		bool used{false};
		// what the table was made of, kept to make it again when a layer changes
		std::size_t count{0};
		std::array<int, max_layers> layers{};
		std::array<float, max_layers> rotations{};
	};
	// what a texture made here holds
	enum class texture_kind
	{
		white, // a single white texel, rgba8
		layers, // an array of the layer maps, rgba8 (srgb for the diffuse)
		weights, // weights of four layers, rgba8 with mipmaps, clamped
		table, // the table of the layers of a chunk, max_layers x 2 rgba32f, nearest
		holes // the mask of the holes, r8, led between the samples, clamped
	};

	// the textures of the arrays: diffuse, normal maps, specular/gloss
	std::array<texture_array, 3> const &arrays() const
	{
		return m_arrays;
	}

	// the objects the textures are drawn with (the copy of the layer maps); false: the terrain can't be drawn
	virtual bool gpu_init() = 0;
	// the object of a texture made by the bank, of the kind
	virtual void gpu_texture(texture_source &Texture, texture_kind const Kind) = 0;
	// the white texel
	virtual void gpu_white(texture_source &Texture) = 0;
	// room for Layers layers in the array; Fresh: the array had layers before, they go
	virtual void gpu_storage(texture_array const &Array, int const Layers, bool const Fresh) = 0;
	// the content of the textures of a new slot, before its material is made of them: no weights, a table of zeros, no holes
	virtual void gpu_blank(slot const &Slot) = 0;
	// Side x Side texels of rgba8, the mipmaps made of them
	virtual void gpu_weights(texture_handle const Texture, int const Side, std::uint8_t const *Texels) = 0;
	// max_layers x 2 texels of rgba32f
	virtual void gpu_table(texture_handle const Texture, float const *Table) = 0;
	// Side x Side samples of r8
	virtual void gpu_holes(texture_handle const Texture, int const Side, std::uint8_t const *Samples) = 0;
	// the copies of the layer maps into the arrays: begun, then a copy per array of each layer copied, ended; Copied: whether any
	// was made (the mipmaps of the arrays are made then)
	virtual void gpu_begin_copies() = 0;
	// Source: the map of the layer, or null when it has none, the array is filled with Default then
	virtual void gpu_copy(texture_array const &Array, int const Layer, texture_source *Source, std::array<float, 4> const &Default) = 0;
	virtual void gpu_end_copies(bool const Copied) = 0;
	// textures were made or given new content (the opengl renderer forgets the textures bound)
	virtual void gpu_changed() {}

	material_manager &m_materials;
	texture_bank &m_textures;

  private:
	bool init();
	texture_handle make_texture(std::string const &Name, texture_kind const Kind);
	void read_material(layer &Layer);
	int layer_index(gfx::terrain_layer const &Layer);
	void write_table(slot const &Slot);
	bool copy_layer(layer &Layer);
	std::size_t acquire_slot(material_handle const Reuse);

	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used
	texture_handle m_white{null_handle};
	std::array<texture_array, 3> m_arrays; // diffuse, normal maps, specular/gloss
	int m_capacity{0}; // layers the arrays have room for
	std::vector<layer> m_layers;
	bool m_pending{false};
	std::vector<slot> m_slots;
	std::vector<std::size_t> m_free;
	std::unordered_map<material_handle, std::size_t> m_slotof;
	std::shared_ptr<gfx::shader_program> m_program; // of the chunks, mat_terrain
	std::shared_ptr<gfx::shader_program> m_holesprogram; // of the chunks with holes, mat_terrain_holes
};

} // namespace gfx
