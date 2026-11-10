/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the compute passes of the vulkan renderer (the stages P of the plan): a kernel is a compute shader of shaders/vk14 with its
// pipeline. its layout is the pass set of the scene (set 0, the persistent one, for the shaders which read the lights, the
// clusters, the shadow maps or the weather of the pass) and a pushed set of its own (set 1), whose bindings are read from the
// SPIR-V of the shader, with push constants for the parameters the opengl renderer gives its compute programs as loose
// uniforms. the data of the gpu read back by the cpu goes through a readback ring, a buffer seen by the cpu for each frame in
// flight, taken once the gpu is done with the frame

#pragma once

#include "rendering/vulkan14/vulkan14common.h"
#include "rendering/vulkan14/vulkan14descriptors.h"
#include "rendering/vulkan14/vulkan14memory.h"
#include "rendering/vulkan14/vulkan14shaders.h"

#include <optional>

namespace vk14
{

// what a template of a pushed set reads for each of its bindings: an image or a buffer, laid out alike
union descriptor
{
	VkDescriptorImageInfo image;
	VkDescriptorBufferInfo buffer;
};
static_assert(sizeof(VkDescriptorImageInfo) == sizeof(VkDescriptorBufferInfo), "the images and the buffers of the templates are laid out by one stride");

inline descriptor image_descriptor(VkSampler Sampler, VkImageView View, VkImageLayout Layout)
{
	descriptor result;
	result.image = VkDescriptorImageInfo{Sampler, View, Layout};
	return result;
}
inline descriptor storage_image(VkImageView View)
{
	descriptor result;
	result.image = VkDescriptorImageInfo{VK_NULL_HANDLE, View, VK_IMAGE_LAYOUT_GENERAL};
	return result;
}
inline descriptor buffer_descriptor(VkBuffer Buffer, VkDeviceSize Offset, VkDeviceSize Range)
{
	descriptor result;
	result.buffer = VkDescriptorBufferInfo{Buffer, Offset, Range};
	return result;
}

// the bindings of the descriptor set Set the shader uses, from the decorations of its variables, in the order of the bindings
std::vector<VkDescriptorSetLayoutBinding> reflect_bindings(std::vector<std::uint32_t> const &Spirv, std::uint32_t Set, VkShaderStageFlags Stages);

// the set of the compute kernels pushed with their dispatches
constexpr std::uint32_t COMPUTE_PUSH_SET = 1;
// the push constants of the kernels at most, vulkan 1.4 guarantees them
constexpr std::uint32_t COMPUTE_PUSH_CONSTANTS = 256;

class compute_kernel
{
  public:
	compute_kernel() = default;
	compute_kernel(compute_kernel const &) = delete;
	compute_kernel &operator=(compute_kernel const &) = delete;

	// the kernel of the shader: Passlayout is the layout of the pass set of the scene, set 0 of every kernel; the bindings of
	// set 1 are those the shader declares. Specialization: the values of the constants 0, 1, 2 ...
	bool create(VkDevice Device, pipeline_library &Pipelines, std::shared_ptr<shader_code const> const &Shader, VkDescriptorSetLayout Passlayout, std::vector<std::uint32_t> const &Specialization, std::string const &Name);
	void destroy(VkDevice Device);
	explicit operator bool() const
	{
		return m_pipeline != VK_NULL_HANDLE;
	}
	VkPipelineLayout layout() const
	{
		return m_layout;
	}
	std::size_t bindings() const
	{
		return m_count;
	}
	// the pipeline bound, the set pushed (Descriptors in the order of the bindings of the shader), the constants pushed (Size
	// bytes) and the groups dispatched
	void dispatch(command_stream *Commands, descriptor const *Descriptors, void const *Constants, std::uint32_t Size, std::uint32_t X, std::uint32_t Y = 1, std::uint32_t Z = 1) const;
	template <typename Constants> void dispatch(command_stream *Commands, std::vector<descriptor> const &Descriptors, Constants const &Data, std::uint32_t X, std::uint32_t Y = 1, std::uint32_t Z = 1) const
	{
		static_assert(sizeof(Constants) <= COMPUTE_PUSH_CONSTANTS, "too many push constants");
		dispatch(Commands, Descriptors.data(), &Data, static_cast<std::uint32_t>(sizeof(Constants)), X, Y, Z);
	}
	// the same without push constants
	void run(command_stream *Commands, std::vector<descriptor> const &Descriptors, std::uint32_t X, std::uint32_t Y = 1, std::uint32_t Z = 1) const
	{
		dispatch(Commands, Descriptors.data(), nullptr, 0, X, Y, Z);
	}
	// the same with the groups taken from a buffer
	void dispatch_indirect(command_stream *Commands, descriptor const *Descriptors, void const *Constants, std::uint32_t Size, VkBuffer Buffer, VkDeviceSize Offset) const;

  private:
	void bind(command_stream *Commands, descriptor const *Descriptors, void const *Constants, std::uint32_t Size) const;

	push_set m_set;
	VkPipelineLayout m_layout{VK_NULL_HANDLE};
	VkPipeline m_pipeline{VK_NULL_HANDLE};
	std::size_t m_count{0};
	std::string m_name;
};

// a global memory barrier, between the passes which write and read buffers (the images have their own, vk14::barriers())
void memory_barrier(command_stream *Commands, VkPipelineStageFlags2 SrcStage, VkAccessFlags2 SrcAccess, VkPipelineStageFlags2 DstStage, VkAccessFlags2 DstAccess);
// the writes of the compute passes made seen by what follows: the compute passes, the draws, the indirect commands, the copies
void compute_barrier(command_stream *Commands);
// Size bytes of Data (a multiple of 4, at most 64 KiB) written to the buffer from Offset with the commands of the frame, between
// its reads by the shaders before and after. outside a rendering
void update_buffer(command_stream *Commands, buffer const &Target, void const *Data, VkDeviceSize Size, VkDeviceSize Offset);

// data copied from the gpu for the cpu: a buffer per frame in flight, the copy of a frame read once the gpu is done with it
class readback_ring
{
  public:
	// Shared: copied to by the queues of the families the allocator shares between (the async compute of M6)
	bool create(allocator const &Allocator, VkDeviceSize Size, std::uint32_t Frames, std::string const &Name, bool Shared = false);
	void destroy();
	// the copy of Size bytes (at most the size of the ring) of Source from Offset, in the frame of the slot, done by the gpu once
	// it reaches Value of the timeline
	void push(command_stream *Commands, std::uint32_t Slot, std::uint64_t Value, VkBuffer Source, VkDeviceSize Offset, VkDeviceSize Size);
	// the newest copy the gpu is done with (Completed: the value of the timeline it reached), copied to Data; false when there's
	// none newer than the one taken last
	bool pop(std::uint64_t Completed, void *Data, VkDeviceSize Size);
	// the frame of the copy taken last, for the data which tell what they were made for
	std::uint64_t popped_value() const
	{
		return m_popped;
	}
	std::uint32_t popped_slot() const
	{
		return m_poppedslot;
	}

  private:
	struct slot
	{
		buffer data;
		std::uint64_t value{0};
		bool pending{false};
	};
	std::vector<slot> m_slots;
	VkDeviceSize m_size{0};
	std::uint64_t m_popped{0};
	std::uint32_t m_poppedslot{0};
};

} // namespace vk14
