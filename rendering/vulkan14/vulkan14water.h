/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the water on the windscreens in the vulkan renderer (gfx::windscreen_water, opengl46_windscreen_water): the storage of the water,
// which the pass sets hold for the glasses (WATER_SSBO), and the compute kernels of the simulation, run with the commands of the
// frame before its passes (outside a rendering). the kernels push everything they read (set COMPUTE_SET: the storage, the block of
// the step in the ring of the frame, the mask of the wipers being baked)

#pragma once

#include "rendering/common/windscreenwater.h"
#include "rendering/vulkan14/vulkan14compute.h"
#include "rendering/vulkan14/vulkan14texture.h"

#include <array>
#include <functional>
#include <memory>

namespace vk14
{

class windscreen_water : public gfx::windscreen_water
{
  public:
	// what the renderer does for the water
	struct host
	{
		// the commands of the frame, outside a rendering
		std::function<command_stream *()> commands;
		// the view and the sampler of a texture of the renderer, made when needed; false when it isn't ready
		std::function<bool(texture_handle, VkImageView &, VkSampler &)> texture;
	};

	windscreen_water() = default;
	~windscreen_water() override;

	// called once, before the pass sets are made: the storage made (the pass sets hold it from then on). false: no storage
	bool init(VkDevice Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout Passlayout, allocator &Memory, uploader &Uploads, deletion_queue &Deletions, frame_ring &Ring,
	          VkDeviceSize Alignment, host Host);
	// everything goes (the device is idle)
	void destroy();
	// the storage the pass sets hold
	VkBuffer storage() const
	{
		return m_storage ? m_storage->handle() : VK_NULL_HANDLE;
	}

  protected:
	bool gpu_init() override;
	void gpu_setup(std::size_t Slot, std::vector<glm::vec4> const &Axes) override;
	bool gpu_bake(std::size_t Slot, texture_handle Mask, gl::water_ubs const &Block) override;
	void gpu_step(gl::water_ubs const &Block, std::uint32_t Tries) override;

  private:
	// the passes of the step, in their order
	enum pass
	{
		trail,
		spawn,
		move,
		clear,
		grid,
		merge,
		gather,
		settle,
		bake,
		count
	};
	// the block of the step in the ring of the frame; empty when the ring is full
	ring_slice block(gl::water_ubs const &Block);
	// the frames before are done with the storage before the commands of this one write it
	void begin(command_stream *Commands);

	VkDevice m_device{VK_NULL_HANDLE};
	pipeline_library *m_pipelines{nullptr};
	shader_library *m_shaders{nullptr};
	VkDescriptorSetLayout m_passlayout{VK_NULL_HANDLE};
	allocator *m_memory{nullptr};
	uploader *m_uploads{nullptr};
	deletion_queue *m_deletions{nullptr};
	frame_ring *m_ring{nullptr};
	VkDeviceSize m_alignment{256};
	host m_host;
	std::array<compute_kernel, pass::count> m_kernels;
	std::shared_ptr<buffer> m_storage;
	bool m_full{false}; // the storage is as large as the simulation needs (else a stand-in, the glasses make their water up)
	bool m_cleared{false};
};

} // namespace vk14
