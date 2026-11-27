/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the materials of the heightmap terrain in the vulkan renderer (the stage P6 of the plan, opengl46_terrain_materials): the
// textures of gfx::terrain_materials as images of the textures of the bank (vulkan14_texture::adopt()). the weights, the tables
// and the holes of the chunks go through the uploads; the layer maps are drawn into the layers of the arrays by the copy shader
// (Draw, given by the renderer) in the commands of the frame, the mipmaps of the arrays made by blits after them

#pragma once

#include "rendering/common/terrainmaterials.h"
#include "rendering/vulkan14/vulkan14texture.h"

#include <functional>

namespace vk14
{

class terrain_materials : public gfx::terrain_materials
{
  public:
	// the copy shader drawn into Target (a layer of an array in the layout of a colour attachment, of Format and Extent) from the
	// image Source sampled with Sampler
	using copy_draw = std::function<void(command_stream *Commands, VkImageView Target, VkFormat Format, VkExtent2D Extent, VkImageView Source, VkSampler Sampler)>;

	terrain_materials(material_manager &Materials, texture_bank &Textures) : gfx::terrain_materials(Materials, Textures) {}
	~terrain_materials() override;

	// called once, before anything is made
	void init(resources const &Resources, copy_draw Draw);
	// the views go (the device is idle)
	void destroy();
	// the commands of the frame and the value of the timeline the gpu reaches when it's done with it: the arrays made since the last
	// frame are cleared. before update()
	void begin_frame(command_stream *Commands, std::uint64_t Value);
	// the frame was recorded: no commands go to it any more
	void end_frame()
	{
		m_commands = nullptr;
	}

  protected:
	bool gpu_init() override;
	void gpu_texture(texture_source &Texture, texture_kind const Kind) override {}
	void gpu_white(texture_source &Texture) override;
	void gpu_storage(texture_array const &Array, int const Layers, bool const Fresh) override;
	void gpu_blank(slot const &Slot) override;
	void gpu_weights(texture_handle const Texture, int const Side, std::uint8_t const *Texels) override;
	void gpu_table(texture_handle const Texture, float const *Table) override;
	void gpu_holes(texture_handle const Texture, int const Side, std::uint8_t const *Samples) override;
	void gpu_begin_copies() override;
	void gpu_copy(texture_array const &Array, int const Layer, texture_source *Source, std::array<float, 4> const &Default) override;
	void gpu_end_copies(bool const Copied) override;

  private:
	struct array_images
	{
		std::shared_ptr<image> map;
		std::vector<VkImageView> layers; // the first level of each layer, drawn to
		bool clear{false}; // made since the last frame, its content not defined yet
		bool drawn{false}; // a layer was drawn in the copies being made
	};

	vulkan14_texture &texture(texture_handle const Handle);
	// an image of Format, Width x Height, a single layer, with Mips levels, sampled and uploaded to
	std::shared_ptr<image> make_image(VkFormat const Format, std::uint32_t const Width, std::uint32_t const Height, std::uint32_t const Mips, char const *Name);
	// Data into the image of the texture, made anew of Format when there is none of the size; mipmaps made of the first level when Mips
	void upload(texture_handle const Texture, VkFormat const Format, std::uint32_t const Width, std::uint32_t const Height, void const *Data, VkDeviceSize const Size, bool const Mips, VkSampler const Sampler);
	void let_go(array_images &Images);
	void make_mipmaps(image &Map);

	resources const *m_resources{nullptr};
	copy_draw m_draw;
	command_stream *m_commands{nullptr};
	std::uint64_t m_value{0};
	std::array<array_images, 3> m_images; // of the arrays: diffuse, normal maps, specular/gloss
};

} // namespace vk14
