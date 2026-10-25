/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the commands of a frame kept to be recorded into command buffers later (the stage M1 of the plan): the renderer makes the calls
// of vkCmd* on a stream, which keeps them with their data in the order they were made. the stream is cut into chunks as it's
// made, every so many draws: a chunk is recorded into a command buffer of its own, by a worker thread, and the command buffers go
// to the queue in one submit in the order of the chunks. a chunk which starts inside a rendering begins it again resumed
// (VK_RENDERING_RESUMING_BIT, the one before it ended it suspended) and sets again the state the draws before it left: the
// pipelines, the descriptor sets, the pushed descriptors and constants, the vertex and index buffers and the dynamic state, as
// the last calls of each kind made them. where the cuts are doesn't depend on the number of the threads, so the gpu gets the
// same commands with any of them

#pragma once

#include "rendering/vulkan14/vulkan14common.h"

#include <array>
#include <cstddef>
#include <vector>

namespace vk14
{

class command_stream
{
  public:
	// the draws, dispatches and copies a chunk holds at least before it's cut (inside a rendering, between two draws)
	static constexpr std::uint32_t chunk_work{256};

	// the calls, with the arguments vkCmd* takes (the command buffer left out); the data they point to is copied. chains of pNext
	// aren't kept: none of the calls is made with one
	void begin_rendering(VkRenderingInfo const *Info);
	void end_rendering();
	void pipeline_barrier2(VkDependencyInfo const *Dependency);
	void bind_pipeline(VkPipelineBindPoint BindPoint, VkPipeline Pipeline);
	void bind_descriptor_sets2(VkBindDescriptorSetsInfo const *Info);
	// Size: the bytes of the data of the template; BindPoint: the one the template was made for
	void push_descriptor_set_with_template2(VkPushDescriptorSetWithTemplateInfo const *Info, std::size_t Size, VkPipelineBindPoint BindPoint);
	void push_constants2(VkPushConstantsInfo const *Info);
	void bind_vertex_buffers(std::uint32_t First, std::uint32_t Count, VkBuffer const *Buffers, VkDeviceSize const *Offsets);
	void bind_index_buffer(VkBuffer Buffer, VkDeviceSize Offset, VkIndexType Type);
	void draw(std::uint32_t Vertices, std::uint32_t Instances, std::uint32_t FirstVertex, std::uint32_t FirstInstance);
	void draw_indexed(std::uint32_t Indices, std::uint32_t Instances, std::uint32_t FirstIndex, std::int32_t VertexOffset, std::uint32_t FirstInstance);
	void draw_indirect(VkBuffer Buffer, VkDeviceSize Offset, std::uint32_t Count, std::uint32_t Stride);
	void draw_indexed_indirect(VkBuffer Buffer, VkDeviceSize Offset, std::uint32_t Count, std::uint32_t Stride);
	void dispatch(std::uint32_t X, std::uint32_t Y, std::uint32_t Z);
	void dispatch_indirect(VkBuffer Buffer, VkDeviceSize Offset);
	void set_viewport(std::uint32_t First, std::uint32_t Count, VkViewport const *Viewports);
	void set_scissor(std::uint32_t First, std::uint32_t Count, VkRect2D const *Scissors);
	void set_cull_mode(VkCullModeFlags Mode);
	void set_front_face(VkFrontFace Face);
	void set_primitive_topology(VkPrimitiveTopology Topology);
	void set_depth_test_enable(VkBool32 Enable);
	void set_depth_write_enable(VkBool32 Enable);
	void set_depth_compare_op(VkCompareOp Op);
	void set_depth_bias_enable(VkBool32 Enable);
	void set_depth_bias(float Constant, float Clamp, float Slope);
	void set_primitive_restart_enable(VkBool32 Enable);
	void set_stencil_test_enable(VkBool32 Enable);
	void fill_buffer(VkBuffer Buffer, VkDeviceSize Offset, VkDeviceSize Size, std::uint32_t Data);
	void update_buffer(VkBuffer Buffer, VkDeviceSize Offset, VkDeviceSize Size, void const *Data);
	void copy_buffer2(VkCopyBufferInfo2 const *Info);
	void copy_image2(VkCopyImageInfo2 const *Info);
	void copy_buffer_to_image2(VkCopyBufferToImageInfo2 const *Info);
	void copy_image_to_buffer2(VkCopyImageToBufferInfo2 const *Info);
	void blit_image2(VkBlitImageInfo2 const *Info);
	void clear_color_image(VkImage Image, VkImageLayout Layout, VkClearColorValue const *Color, std::uint32_t Count, VkImageSubresourceRange const *Ranges);
	void clear_depth_stencil_image(VkImage Image, VkImageLayout Layout, VkClearDepthStencilValue const *Value, std::uint32_t Count, VkImageSubresourceRange const *Ranges);
	void write_timestamp2(VkPipelineStageFlags2 Stage, VkQueryPool Pool, std::uint32_t Query);
	void reset_query_pool(VkQueryPool Pool, std::uint32_t First, std::uint32_t Count);
	void begin_query(VkQueryPool Pool, std::uint32_t Query, VkQueryControlFlags Flags);
	void end_query(VkQueryPool Pool, std::uint32_t Query);
	void begin_debug_utils_label(VkDebugUtilsLabelEXT const *Label);
	void end_debug_utils_label();
	// the builds of acceleration structures (VK_KHR_acceleration_structure, gfx.raytracing), outside of a rendering
	void build_acceleration_structures(std::uint32_t Count, VkAccelerationStructureBuildGeometryInfoKHR const *Infos, VkAccelerationStructureBuildRangeInfoKHR const *const *Ranges);

	// the stream is made anew
	void clear();
	bool empty() const
	{
		return m_chunks.empty() || (m_data.empty());
	}
	// the stream is ended: no calls after it
	void finish();
	// the chunks, valid after finish()
	std::size_t chunks() const
	{
		return m_chunks.size();
	}
	// the calls recorded since the start
	std::size_t calls() const
	{
		return m_calls;
	}
	// the chunk recorded into Buffer (begun already, ended by the caller)
	void replay(std::size_t Chunk, VkCommandBuffer Buffer) const;
	// all of it into one command buffer
	void replay(VkCommandBuffer Buffer) const;

  private:
	// the calls of the state a chunk sets again; the dynamic states have a slot each
	enum state_slot : std::uint32_t
	{
		pipeline_graphics,
		pipeline_compute,
		viewport,
		scissor,
		cull_mode,
		front_face,
		topology,
		depth_test,
		depth_write,
		depth_compare,
		depth_bias_enable,
		depth_bias,
		primitive_restart,
		stencil_test,
		index_buffer,
		sets_graphics, // set_slots of each
		sets_compute = sets_graphics + 8,
		pushed_graphics = sets_compute + 8,
		pushed_compute = pushed_graphics + 8,
		vertex_buffers = pushed_compute + 8, // vertex_slots
		slot_count = vertex_buffers + 8
	};
	static constexpr std::size_t no_call{~static_cast<std::size_t>(0)};
	static constexpr std::uint32_t set_slots{8};
	static constexpr std::uint32_t vertex_slots{8};
	struct chunk
	{
		std::size_t first{0}; // offset of its first call in the data
		std::size_t end{0}; // past its last one
		// the begin of the rendering the chunk starts inside of, resumed, and the calls of the state set again, in their order
		std::size_t resumed{no_call};
		std::vector<std::size_t> state;
		// the begin of the rendering the chunk ends inside of, suspended
		std::size_t suspended{no_call};
	};

	// room for a call of Type with Size bytes of its own data after its header, aligned; the offset of the call
	std::size_t add(std::uint32_t Type, std::size_t Size);
	template <typename Type_> Type_ &payload(std::size_t Call)
	{
		return *reinterpret_cast<Type_ *>(m_data.data() + Call + header_size);
	}
	template <typename Type_> Type_ const &payload(std::size_t Call) const
	{
		return *reinterpret_cast<Type_ const *>(m_data.data() + Call + header_size);
	}
	template <typename Type_> Type_ const *at(std::size_t Offset) const
	{
		return reinterpret_cast<Type_ const *>(m_data.data() + Offset);
	}
	// copies Bytes of Data after the call made last (aligned), returns their offset in the data
	std::size_t append(void const *Data, std::size_t Bytes);
	// the call made last ends here
	void close(std::size_t Call);
	void remember(std::uint32_t Slot, std::size_t Call)
	{
		m_state[Slot] = Call;
	}
	// a draw, dispatch or copy was added: the chunk is cut after it when it has enough of them
	void work();
	void cut();
	// Flags: added to those of a begin of a rendering
	void execute(std::size_t Call, VkCommandBuffer Buffer, VkRenderingFlags Flags = 0) const;

	static constexpr std::size_t header_size{8};
	std::vector<std::uint8_t> m_data;
	std::vector<chunk> m_chunks;
	std::array<std::size_t, slot_count> m_state{};
	// the pushes of the constants: the last one of each range and stages, in their order
	std::vector<std::size_t> m_pushes;
	std::size_t m_rendering{no_call}; // the begin of the rendering under way
	std::uint32_t m_queries{0}; // queries active
	std::uint32_t m_work{0}; // of the chunk under way
	std::size_t m_calls{0};
	bool m_finished{false};
};

} // namespace vk14
