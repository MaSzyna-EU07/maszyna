/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the gpu culling of the instances in the vulkan renderer (the stage P5 of the plan): the buffers and the compute passes of
// gfx::instance_cull, the opengl46_instance_cull of this renderer, with the same shaders (instance_cull.comp,
// instance_cull_offsets.comp, instance_cull_commands.comp), and the depth pyramid of its occlusion test (hiz_build.comp, the
// opengl46_hiz of this renderer). what differs from the opengl renderer:
// - the records go to the gpu with the uploads of the frame, which come before its commands; a record buffer started over gets
//   a new buffer, so the passes of the frame culled before keep the places they had
// - the jobs, the groups, the lod bounds and the parameters of a pass are read by the kernels from the ring of the frame
// - the counters are copied for the cpu at the end of the last stage of the culling, in the frame being recorded (the rendering
//   of the pass is suspended for the dispatches)
// - the output is the instance buffer of the indirect draws in pass sets of its own (vulkan14_renderer::cull_pass_sets())

#pragma once

#include "rendering/common/instancecull.h"
#include "rendering/common/shadowfit.h"
#include "rendering/vulkan14/vulkan14compute.h"
#include "rendering/vulkan14/vulkan14memory.h"
#include "rendering/vulkan14/vulkan14texture.h"

#include <array>
#include <memory>
#include <vector>

namespace vk14
{

// what the cullings need of the renderer
struct cull_environment
{
	allocator *memory{nullptr};
	uploader *uploads{nullptr};
	deletion_queue *deletions{nullptr};
	frame_ring *ring{nullptr}; // the ring of the frame, the data of the passes go through it
	VkDeviceSize uniform_alignment{256};
	VkDeviceSize storage_alignment{256};
	std::uint32_t frames{2}; // in flight
	// the frame being recorded: its slot among the frames in flight, the value of the timeline the gpu reaches when it's done with it,
	// and the value the gpu reached
	std::uint32_t slot{0};
	std::uint64_t value{0};
	std::uint64_t completed{0};
	// a sampled image to give the occlusion test of the kernel when there's no pyramid
	descriptor no_pyramid{};
};

// the occlusion test of a pass: the pyramid (the view of all its levels, in the general layout) and what it was made from
struct cull_occlusion
{
	VkImageView pyramid{VK_NULL_HANDLE};
	VkSampler sampler{VK_NULL_HANDLE};
	glm::ivec2 size{0}; // of its first level
	int levels{0};
	glm::vec2 screen{0.f}; // size of the depth it was made from, pixels
	glm::mat4 projection{1.f}; // of the pass
};

class instance_cull : public gfx::instance_cull
{
  public:
	instance_cull() = default;
	~instance_cull() override;

	// the kernels and the record buffer made, or taken from Shared. Shaders: the shader library
	bool init(cull_environment const &Environment, VkDevice Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout Passlayout, instance_cull const *Shared = nullptr);
	void destroy(VkDevice Device);
	explicit operator bool() const
	{
		return bool(m_shared);
	}
	// the frame being recorded (cull_environment::slot, value, completed)
	void begin_frame(std::uint32_t const Slot, std::uint64_t const Value, std::uint64_t const Completed);
	// as opengl46_instance_cull::dispatch(), dispatch_second(), fill_commands(), outside a rendering
	void dispatch(command_stream *Commands, glm::mat4 const &View, cull_occlusion const *Occlusion = nullptr, bool const Twophase = false);
	void dispatch_second(command_stream *Commands, cull_occlusion const &Occlusion);
	void fill_commands(command_stream *Commands, std::vector<gl::instance_draw_command> const &Commands_);
	// the indirect commands (draw_command_offset()) and the matrices of the instances drawn
	VkBuffer command_buffer() const
	{
		return m_commands ? m_commands->handle() : VK_NULL_HANDLE;
	}
	VkBuffer output_buffer() const
	{
		return m_output ? m_output->handle() : VK_NULL_HANDLE;
	}
	VkDeviceSize output_size() const
	{
		return m_output ? m_output->size() : 0;
	}
	// the pass sets of the renderer with the output as the instance buffer, made for the generation of the pass sets and the output
	std::array<VkDescriptorSet, 3> sets{};
	VkDescriptorPool set_pool{VK_NULL_HANDLE};
	std::uint64_t sets_generation{0};
	VkBuffer sets_output{VK_NULL_HANDLE};
	// the gpu time of the passes, for the panel (the parts of the pass list)
	std::string info_times(std::string const &Label = "instance cull") const
	{
		return {};
	}

  protected:
	std::int64_t record_buffer_size() const override;
	void resize_record_buffer(std::int64_t const Size) override;
	void send_records(std::uint32_t const First, scene::basic_cell::instance_record const *Records, std::uint32_t const Count) override;
	void restart_records() override;
	void push_counts() override;
	bool pop_counts(std::uint32_t *Counts) override;

  private:
	// what the cullings of all passes share: the kernels, the record buffer and the records sent since the last dispatch
	struct shared_data
	{
		compute_kernel cull;
		compute_kernel offsets;
		compute_kernel commands;
		std::shared_ptr<buffer> records;
		std::vector<std::uint8_t> pending;
		std::vector<std::pair<VkDeviceSize, VkDeviceSize>> parts;
		cull_environment environment;
	};
	// a buffer of the culling grown to hold at least Size bytes, at least doubled; the old one goes once the gpu is done with it
	bool reserve(std::shared_ptr<buffer> &Buffer, VkDeviceSize Size, VkBufferUsageFlags Usage, char const *Name);
	// the records sent since the last dispatch go with the uploads of the frame
	void flush_records();
	// the descriptors of the culling kernel; Occlusion: the pyramid, or none
	std::vector<descriptor> cull_descriptors(cull_occlusion const *Occlusion) const;
	// the counters copied for the cpu
	void read_counters(command_stream *Commands);

	std::shared_ptr<shared_data> m_shared;
	std::shared_ptr<buffer> m_counters;
	std::shared_ptr<buffer> m_output;
	std::shared_ptr<buffer> m_commands;
	readback_ring m_readback;
	// the slices of the ring the jobs, the groups, the lod bounds and the parameters of the pass were placed in
	ring_slice m_jobs;
	ring_slice m_groups;
	ring_slice m_lods;
	ring_slice m_block;
	VkDeviceSize m_countersize{0};
};

// the depth pyramid of the occlusion test of the main view (opengl46_hiz): made from the resolved depth of the pass, the first
// level half its size, each level the farthest depth of the 2x2 texels below it
class hiz_pyramid
{
  public:
	bool init(VkDevice Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout Passlayout);
	void destroy(VkDevice Device, deletion_queue *Deletions = nullptr, std::uint64_t Value = 0);
	// the pyramid made from Depth (a sampled image of Width x Height, in the read only layout), the images made anew for another size
	bool build(command_stream *Commands, VkDevice Device, allocator &Memory, deletion_queue &Deletions, std::uint64_t Value, VkImageView Depth, VkSampler Sampler, int Width, int Height);
	// the occlusion test of the pass
	cull_occlusion occlusion(VkSampler Sampler, glm::mat4 const &Projection) const;

  private:
	compute_kernel m_kernel;
	std::shared_ptr<image> m_pyramid;
	std::vector<VkImageView> m_levels;
	glm::ivec2 m_sourcesize{0};
	glm::ivec2 m_size{0};
};

// the cascades of the sun shadows fitted to the depth of the main view (gfx.shadow.sdsm, opengl46_shadow_fit): the passes and the
// buffers of gfx::shadow_fit, the result read back through a ring
class shadow_fitter : public gfx::shadow_fit
{
  public:
	bool init(VkDevice Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout Passlayout, allocator const &Memory, std::uint32_t Frames);
	void destroy(VkDevice Device);
	explicit operator bool() const
	{
		return bool(m_depthkernel) && bool(m_boundskernel) && bool(m_buffer);
	}
	// measures the depth of the view: Depth, the resolved depth of Width x Height pixels in the read only layout, outside a rendering.
	// Frame: the frame of the renderer; Slot, Value: the frame being recorded, as the readback rings take it
	void measure(command_stream *Commands, VkImageView Depth, VkSampler Sampler, int Width, int Height, view const &View, unsigned int Frame, std::uint32_t Slot, std::uint64_t Value);
	// takes in the newest measurement the gpu has finished (Completed: the value of the timeline it reached), and returns the
	// newest one taken in so far
	result const *latest(std::uint64_t Completed);

  private:
	compute_kernel m_depthkernel;
	compute_kernel m_boundskernel;
	buffer m_buffer;
	readback_ring m_readback;
};

} // namespace vk14
