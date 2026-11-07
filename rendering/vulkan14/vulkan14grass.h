/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the grass of the heightmap terrain in the vulkan renderer (the stage P6 of the plan, opengl46_grass): the images of the chunks
// of gfx::grass_field, uploaded when a chunk comes near. what the opengl renderer gives grass.vert in uniforms goes to the push
// constants (the tile and the frame) and to a table of the chunk (its kinds, an image the vertex shader reads), which is made
// again when what it holds changes

#pragma once

#include "rendering/common/grassfield.h"
#include "rendering/vulkan14/vulkan14texture.h"

#include <array>
#include <memory>
#include <unordered_map>

namespace vk14
{

class grass_field : public gfx::grass_field
{
  public:
	// the images a chunk is drawn with, in the order of the bindings of grass.vert: heights, densities, holes, the table
	struct chunk_images
	{
		std::array<std::shared_ptr<image>, 4> images;
		std::array<VkSampler, 4> samplers{};
		std::array<glm::vec4, 20> table{}; // what the table holds
		bool table_made{false};
	};

	grass_field() = default;
	~grass_field() override;

	// called once, before anything is made
	void init(resources const &Resources);
	// the images go (the device is idle)
	void destroy();
	// the images of a chunk drawn, its table made for the draw first; null: the chunk has none (yet)
	chunk_images const *images(chunk_draw const &Draw);

  protected:
	void gpu_upload(std::uint64_t const Id, gfx::grass_chunk const &Source) override;
	void gpu_release(std::uint64_t const Id) override;

  private:
	std::shared_ptr<image> make_image(VkFormat const Format, std::uint32_t const Width, std::uint32_t const Height, void const *Data, VkDeviceSize const Size, char const *Name);

	resources const *m_resources{nullptr};
	std::unordered_map<std::uint64_t, chunk_images> m_chunks;
};

} // namespace vk14
