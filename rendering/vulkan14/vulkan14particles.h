/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the particles of the gpu in the vulkan renderer (the stage P6 of the plan, opengl46_gpu_particles): the storage buffers, the
// compute kernels and the indirect draws of gfx::gpu_particles. the compute passes run between two parts of the rendering of
// the colour pass (the host ends it and begins it again); the instances counted by a pass are read back once the gpu is done
// with the frame, as the opengl renderer reads them a few frames later

#pragma once

#include "rendering/common/gpuparticles.h"
#include "rendering/vulkan14/vulkan14compute.h"
#include "rendering/vulkan14/vulkan14texture.h"

#include <functional>
#include <memory>

namespace vk14
{

struct scene_program;

class gpu_particles : public gfx::gpu_particles
{
  public:
	// what the renderer does for the particles
	struct host
	{
		// the rendering under way ended for the compute passes; the commands of the frame
		std::function<command_stream *()> compute;
		// the pass set of the pass bound for the kernel
		std::function<void(compute_kernel const &Kernel)> bind;
		// the rendering begun again and the draw of the instances recorded: Program with the blending of the scene, the instances
		// from Output, the command from Command, Data0 and Data1 in the push constants (data[0], data[1])
		std::function<void(scene_program &Program, VkBuffer Output, VkBuffer Command, glm::vec4 const &Data0, glm::vec4 const &Data1)> draw;
		// the slot of the frame in flight, the value of the timeline the gpu reaches when it's done with the frame, the one it reached
		std::function<std::uint32_t()> slot;
		std::function<std::uint64_t()> value;
		std::function<std::uint64_t()> completed;
	};

	gpu_particles() = default;
	~gpu_particles() override;

	// called once, before anything is made. Frames: the frames in flight
	void init(VkDevice Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout Passlayout, allocator &Memory, uploader &Uploads, deletion_queue &Deletions, frame_ring &Ring,
	          VkDeviceSize Alignment, std::uint32_t Frames, host Host);
	// everything goes (the device is idle)
	void destroy();

  protected:
	bool gpu_init() override;
	void gpu_layout(std::uint32_t const Places, std::uint32_t const Kept, std::vector<std::uint32_t> const &Owners, std::size_t const Sources) override;
	void gpu_simulate(smoke_simulation const &Simulation, std::vector<gl::particle_smoke_source> const &Records) override;
	void gpu_render_smoke(smoke_draw const &Draw, std::size_t &Drawn) override;
	void gpu_collect_precipitation(std::size_t &Drawn) override;
	void gpu_precipitation_storage(std::uint32_t const Capacity) override;
	void gpu_render_precipitation(precipitation_draw const &Draw) override;
	void gpu_clear() override;

  private:
	std::shared_ptr<buffer> make_buffer(VkDeviceSize const Size, VkBufferUsageFlags const Usage, char const *Name);
	void let_go(std::shared_ptr<buffer> &Buffer);
	// the command of a draw reset before the compute pass counts its instances
	void reset_command(command_stream *Commands, buffer const &Command);
	static void collect(readback_ring &Readback, std::uint64_t Completed, std::size_t &Drawn);

	VkDevice m_device{VK_NULL_HANDLE};
	pipeline_library *m_pipelines{nullptr};
	shader_library *m_shaders{nullptr};
	VkDescriptorSetLayout m_passlayout{VK_NULL_HANDLE};
	allocator *m_memory{nullptr};
	uploader *m_uploads{nullptr};
	deletion_queue *m_deletions{nullptr};
	frame_ring *m_ring{nullptr};
	VkDeviceSize m_alignment{256};
	std::uint32_t m_frames{0};
	host m_host;

	compute_kernel m_smokesimulate;
	compute_kernel m_smokeprepare;
	compute_kernel m_precipitationprepare;
	std::shared_ptr<scene_program> m_smokedraw;
	std::shared_ptr<scene_program> m_precipitationdraw;

	// smoke
	std::shared_ptr<buffer> m_particles;
	std::shared_ptr<buffer> m_sources;
	std::shared_ptr<buffer> m_owners;
	std::shared_ptr<buffer> m_smokeoutput;
	std::shared_ptr<buffer> m_smokecommand;
	readback_ring m_smokereadback;
	// rain and snow
	std::shared_ptr<buffer> m_precipitationoutput;
	std::shared_ptr<buffer> m_precipitationcommand;
	readback_ring m_precipitationreadback;
};

} // namespace vk14
