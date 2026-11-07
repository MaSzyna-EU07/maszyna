/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14commands.h"

#include <algorithm>
#include <cassert>
#include <cstring>

namespace vk14
{

namespace
{

enum call_type : std::uint32_t
{
	call_begin_rendering,
	call_end_rendering,
	call_pipeline_barrier,
	call_bind_pipeline,
	call_bind_descriptor_sets,
	call_push_descriptors,
	call_push_constants,
	call_bind_vertex_buffers,
	call_bind_index_buffer,
	call_draw,
	call_draw_indexed,
	call_draw_indirect,
	call_draw_indexed_indirect,
	call_dispatch,
	call_dispatch_indirect,
	call_set_viewport,
	call_set_scissor,
	call_set_cull_mode,
	call_set_front_face,
	call_set_topology,
	call_set_depth_test,
	call_set_depth_write,
	call_set_depth_compare,
	call_set_depth_bias_enable,
	call_set_depth_bias,
	call_set_primitive_restart,
	call_set_stencil_test,
	call_fill_buffer,
	call_update_buffer,
	call_copy_buffer,
	call_copy_image,
	call_copy_buffer_to_image,
	call_copy_image_to_buffer,
	call_blit_image,
	call_clear_color_image,
	call_clear_depth_stencil_image,
	call_write_timestamp,
	call_reset_query_pool,
	call_begin_query,
	call_end_query,
	call_begin_label,
	call_end_label,
	call_build_acceleration_structures,
	call_set_checkpoint,
	call_external
};

struct call_header
{
	std::uint32_t type;
	std::uint32_t size; // bytes of the call with its header and its data, a multiple of 8
};
static_assert(sizeof(call_header) == 8, "the header of a call of a stream");

std::size_t constexpr no_data{~static_cast<std::size_t>(0)};

struct rendering_call
{
	VkRenderingInfo info;
	std::size_t colours;
	std::size_t depth;
	std::size_t stencil;
};
struct barrier_call
{
	VkDependencyFlags flags;
	std::uint32_t memory_count, buffer_count, image_count;
	std::size_t memory, buffers, images;
};
struct pipeline_call
{
	VkPipelineBindPoint bind_point;
	VkPipeline pipeline;
};
struct sets_call
{
	VkBindDescriptorSetsInfo info;
	std::size_t sets;
	std::size_t offsets;
};
struct pushed_call
{
	VkPushDescriptorSetWithTemplateInfo info;
	std::size_t data;
};
struct constants_call
{
	VkPushConstantsInfo info;
	std::size_t data;
};
struct vertex_buffers_call
{
	std::uint32_t first, count;
	std::size_t buffers, offsets;
};
struct index_buffer_call
{
	VkBuffer buffer;
	VkDeviceSize offset;
	VkIndexType type;
};
struct draw_call
{
	std::uint32_t vertices, instances, first_vertex, first_instance;
};
struct draw_indexed_call
{
	std::uint32_t indices, instances, first_index;
	std::int32_t vertex_offset;
	std::uint32_t first_instance;
};
struct indirect_call
{
	VkBuffer buffer;
	VkDeviceSize offset;
	std::uint32_t count, stride;
};
struct dispatch_call
{
	std::uint32_t x, y, z;
};
struct viewports_call
{
	std::uint32_t first, count;
	std::size_t data;
};
struct value_call
{
	std::uint32_t value;
};
struct bias_call
{
	float constant, clamp, slope;
};
struct fill_call
{
	VkBuffer buffer;
	VkDeviceSize offset, size;
	std::uint32_t data;
};
struct update_call
{
	VkBuffer buffer;
	VkDeviceSize offset, size;
	std::size_t data;
};
// the builds of acceleration structures: the infos, the geometries of all of them one after another and their ranges
struct build_call
{
	std::uint32_t count;
	std::size_t infos;
	std::size_t geometries;
	std::size_t ranges;
};
struct checkpoint_call
{
	std::uint64_t marker;
};
struct external_call
{
	command_stream::external_function function;
	void *data;
};
// the copies and the blit: the info with its regions
template <typename Info_> struct copy_call
{
	Info_ info;
	std::size_t regions;
};
struct clear_call
{
	VkImage image;
	VkImageLayout layout;
	VkClearColorValue colour;
	VkClearDepthStencilValue depth;
	std::uint32_t count;
	std::size_t ranges;
};
struct query_call
{
	VkPipelineStageFlags2 stage;
	VkQueryPool pool;
	std::uint32_t query, count;
	VkQueryControlFlags flags;
};
struct label_call
{
	float colour[4];
	std::size_t name;
};

// the stage and the access of one side of a barrier, without what a queue of the compute family doesn't have (the async compute of M6)
void compute_queue_scope(VkPipelineStageFlags2 &Stage, VkAccessFlags2 &Access)
{
	auto stage{Stage};
	if ((stage & VK_PIPELINE_STAGE_2_ALL_TRANSFER_BIT) != 0)
	{
		stage = (stage & ~VK_PIPELINE_STAGE_2_ALL_TRANSFER_BIT) | VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT;
	}
	stage &= (VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT | VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_COPY_BIT |
	          VK_PIPELINE_STAGE_2_CLEAR_BIT | VK_PIPELINE_STAGE_2_HOST_BIT | VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT);
	VkAccessFlags2 allowed{VK_ACCESS_2_MEMORY_READ_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT};
	if ((stage & VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT) != 0)
	{
		allowed |= VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT | VK_ACCESS_2_UNIFORM_READ_BIT | VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_SAMPLED_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
		           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT | VK_ACCESS_2_HOST_READ_BIT | VK_ACCESS_2_HOST_WRITE_BIT;
	}
	if ((stage & VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT) != 0)
	{
		allowed |= VK_ACCESS_2_UNIFORM_READ_BIT | VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_SAMPLED_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
	}
	if ((stage & VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT) != 0)
	{
		allowed |= VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT;
	}
	if ((stage & VK_PIPELINE_STAGE_2_COPY_BIT) != 0)
	{
		allowed |= VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT;
	}
	if ((stage & VK_PIPELINE_STAGE_2_CLEAR_BIT) != 0)
	{
		allowed |= VK_ACCESS_2_TRANSFER_WRITE_BIT;
	}
	if ((stage & VK_PIPELINE_STAGE_2_HOST_BIT) != 0)
	{
		allowed |= VK_ACCESS_2_HOST_READ_BIT | VK_ACCESS_2_HOST_WRITE_BIT;
	}
	Stage = (stage != 0 ? stage : VK_PIPELINE_STAGE_2_NONE);
	Access = (stage != 0 ? (Access & allowed) : VK_ACCESS_2_NONE);
}

template <typename Barrier_> void compute_queue_barriers(Barrier_ *Barriers, std::uint32_t const Count)
{
	for (std::uint32_t i = 0; i < Count; ++i)
	{
		compute_queue_scope(Barriers[i].srcStageMask, Barriers[i].srcAccessMask);
		compute_queue_scope(Barriers[i].dstStageMask, Barriers[i].dstAccessMask);
	}
}

bool compute_stages(VkShaderStageFlags const Stages)
{
	return (Stages & VK_SHADER_STAGE_COMPUTE_BIT) != 0;
}
bool graphics_stages(VkShaderStageFlags const Stages)
{
	return (Stages & ~static_cast<VkShaderStageFlags>(VK_SHADER_STAGE_COMPUTE_BIT)) != 0;
}

} // namespace

void command_stream::clear()
{
	m_data.clear();
	m_chunks.clear();
	m_chunks.emplace_back();
	m_state.fill(no_call);
	m_pushes.clear();
	m_rendering = no_call;
	m_queries = 0;
	m_work = 0;
	m_calls = 0;
	m_finished = false;
	m_pieces.clear();
	m_piecefirst = 0;
	m_order.clear();
	m_split = no_call;
	m_splitchunk = 0;
}

void command_stream::split()
{
	assert((m_rendering == no_call) && (m_queries == 0));
	if (m_chunks.empty())
	{
		clear();
	}
	if (m_split != no_call)
	{
		return;
	}
	cut();
	m_split = m_chunks.size() - 1;
}

void command_stream::splice(command_stream const &Other)
{
	assert((m_rendering == no_call) && (m_queries == 0));
	assert(Other.m_pieces.empty());
	if (m_chunks.empty())
	{
		clear();
	}
	// the chunk under way ends here, with the part of the stream it closes and the state the part leaves
	auto &current{m_chunks.back()};
	current.end = m_data.size();
	current.suspended = no_call;
	piece own;
	own.stream = this;
	own.first = m_piecefirst;
	own.end = m_chunks.size();
	own.state.assign(m_state.begin(), m_state.end());
	own.pushes = m_pushes;
	m_pieces.push_back(std::move(own));
	piece spliced;
	spliced.stream = &Other;
	m_pieces.push_back(std::move(spliced));
	// the calls after it begin a part of their own, with no state of its own yet: what they inherit comes from the spliced stream
	chunk next;
	next.first = m_data.size();
	m_chunks.push_back(std::move(next));
	m_piecefirst = m_chunks.size() - 1;
	m_state.fill(no_call);
	m_pushes.clear();
	m_work = 0;
}

bool command_stream::empty() const
{
	if (false == (m_chunks.empty() || m_data.empty()))
	{
		return false;
	}
	for (auto const &piece : m_pieces)
	{
		if ((piece.stream != this) && (false == piece.stream->empty()))
		{
			return false;
		}
	}
	return true;
}

std::size_t command_stream::calls() const
{
	auto calls{m_calls};
	for (auto const &piece : m_pieces)
	{
		if (piece.stream != this)
		{
			calls += piece.stream->m_calls;
		}
	}
	return calls;
}

void command_stream::leave_state(std::size_t const Piece, merged_state &State) const
{
	auto const &piece{m_pieces[Piece]};
	auto const *stream{piece.stream};
	// the part of the stream itself left what was kept when it was closed, a spliced stream what it set last
	auto const &slots{(stream == this) ? piece.state : std::vector<std::size_t>{stream->m_state.begin(), stream->m_state.end()}};
	auto const &pushes{(stream == this) ? piece.pushes : stream->m_pushes};
	State.slots.resize(slot_count);
	for (std::size_t slot = 0; (slot < slots.size()) && (slot < State.slots.size()); ++slot)
	{
		if (slots[slot] != no_call)
		{
			State.slots[slot] = state_ref{stream, slots[slot], Piece};
		}
	}
	for (auto const push : pushes)
	{
		auto const &info{stream->payload<constants_call>(push).info};
		State.pushes.erase(std::remove_if(State.pushes.begin(), State.pushes.end(),
		                                  [&](state_ref const &Other) {
			                                  auto const &other{Other.stream->payload<constants_call>(Other.call).info};
			                                  return (other.offset == info.offset) && (other.size == info.size) && (other.stageFlags == info.stageFlags);
		                                  }),
		                   State.pushes.end());
		State.pushes.push_back(state_ref{stream, push, Piece});
	}
}

std::size_t command_stream::add(std::uint32_t const Type, std::size_t const Size)
{
	if (m_chunks.empty())
	{
		clear();
	}
	auto const call{m_data.size()};
	auto const bytes{(header_size + Size + 7) & ~static_cast<std::size_t>(7)};
	m_data.resize(call + bytes);
	auto &header{*reinterpret_cast<call_header *>(m_data.data() + call)};
	header.type = Type;
	header.size = static_cast<std::uint32_t>(bytes);
	++m_calls;
	return call;
}

std::size_t command_stream::append(void const *Data, std::size_t const Bytes)
{
	if ((Data == nullptr) || (Bytes == 0))
	{
		return no_data;
	}
	auto const offset{m_data.size()};
	m_data.resize(offset + ((Bytes + 7) & ~static_cast<std::size_t>(7)));
	std::memcpy(m_data.data() + offset, Data, Bytes);
	return offset;
}

void command_stream::close(std::size_t const Call)
{
	reinterpret_cast<call_header *>(m_data.data() + Call)->size = static_cast<std::uint32_t>(m_data.size() - Call);
}

void command_stream::work()
{
	if ((++m_work >= chunk_work) && (m_queries == 0))
	{
		cut();
	}
}

void command_stream::cut()
{
	auto &current{m_chunks.back()};
	current.end = m_data.size();
	current.suspended = m_rendering;
	chunk next;
	next.first = m_data.size();
	next.resumed = m_rendering;
	// the state the next chunk starts with: the last call of each kind, in the order they were made
	for (auto const call : m_state)
	{
		if (call != no_call)
		{
			next.state.push_back(call);
		}
	}
	next.state.insert(next.state.end(), m_pushes.begin(), m_pushes.end());
	std::sort(next.state.begin(), next.state.end());
	next.state.erase(std::unique(next.state.begin(), next.state.end()), next.state.end());
	m_chunks.push_back(std::move(next));
	m_work = 0;
}

void command_stream::finish()
{
	if (m_chunks.empty())
	{
		clear();
	}
	m_chunks.back().end = m_data.size();
	m_chunks.back().suspended = no_call;
	// a chunk left without calls
	if ((m_chunks.size() > 1) && (m_chunks.back().first == m_chunks.back().end) && (m_chunks.size() - 1 > m_piecefirst))
	{
		m_chunks.pop_back();
		m_chunks.back().suspended = no_call;
	}
	m_finished = true;
	if (m_pieces.empty())
	{
		m_splitchunk = std::min(m_split, m_chunks.size());
		return;
	}
	// the last part of the stream itself
	piece last;
	last.stream = this;
	last.first = m_piecefirst;
	last.end = m_chunks.size();
	m_pieces.push_back(std::move(last));
	// the spliced streams with all their chunks, and the state each part begins with: what the parts before it left
	merged_state state;
	state.slots.resize(slot_count);
	m_order.clear();
	for (std::size_t index = 0; index < m_pieces.size(); ++index)
	{
		auto &piece{m_pieces[index]};
		if (piece.stream != this)
		{
			assert(piece.stream->m_finished);
			piece.first = 0;
			piece.end = piece.stream->m_chunks.size();
		}
		piece.inherited.clear();
		std::vector<state_ref> refs;
		for (auto const &slot : state.slots)
		{
			if (slot.stream != nullptr)
			{
				refs.push_back(slot);
			}
		}
		refs.insert(refs.end(), state.pushes.begin(), state.pushes.end());
		// in the order they were made: by the part, then by the place in its stream
		std::sort(refs.begin(), refs.end(), [](state_ref const &Left, state_ref const &Right) { return (Left.piece != Right.piece) ? (Left.piece < Right.piece) : (Left.call < Right.call); });
		refs.erase(std::unique(refs.begin(), refs.end(), [](state_ref const &Left, state_ref const &Right) { return (Left.stream == Right.stream) && (Left.call == Right.call); }), refs.end());
		for (auto const &ref : refs)
		{
			piece.inherited.emplace_back(ref.stream, ref.call);
		}
		for (auto chunk = piece.first; chunk < piece.end; ++chunk)
		{
			auto const &source{piece.stream->m_chunks[chunk]};
			if ((source.first == source.end) && (source.resumed == no_call))
			{
				// nothing in it: no command buffer
				continue;
			}
			m_order.emplace_back(index, chunk);
		}
		leave_state(index, state);
	}
	if (m_order.empty())
	{
		// a frame with nothing in it still has a command buffer
		m_order.emplace_back(0, m_pieces.front().first);
	}
	// the split is in a part of the stream itself: the chunks after it begin the second submit, with the parts after that part
	m_splitchunk = m_order.size();
	if (m_split != no_call)
	{
		for (std::size_t index = 0; index < m_pieces.size(); ++index)
		{
			auto const &piece{m_pieces[index]};
			if ((piece.stream != this) || (m_split < piece.first) || (m_split >= piece.end))
			{
				continue;
			}
			for (std::size_t position = 0; position < m_order.size(); ++position)
			{
				if ((m_order[position].first > index) || ((m_order[position].first == index) && (m_order[position].second >= m_split)))
				{
					m_splitchunk = position;
					break;
				}
			}
			break;
		}
	}
}

// the calls

void command_stream::begin_rendering(VkRenderingInfo const *Info)
{
	auto const call{add(call_begin_rendering, sizeof(rendering_call))};
	auto const colours{append(Info->pColorAttachments, sizeof(VkRenderingAttachmentInfo) * Info->colorAttachmentCount)};
	auto const depth{append(Info->pDepthAttachment, Info->pDepthAttachment != nullptr ? sizeof(VkRenderingAttachmentInfo) : 0)};
	auto const stencil{append(Info->pStencilAttachment, Info->pStencilAttachment != nullptr ? sizeof(VkRenderingAttachmentInfo) : 0)};
	auto &data{payload<rendering_call>(call)};
	data.info = *Info;
	assert(Info->pNext == nullptr);
	data.info.pNext = nullptr;
	data.info.pColorAttachments = nullptr;
	data.info.pDepthAttachment = nullptr;
	data.info.pStencilAttachment = nullptr;
	data.colours = colours;
	data.depth = depth;
	data.stencil = stencil;
	close(call);
	m_rendering = call;
}

void command_stream::end_rendering()
{
	add(call_end_rendering, 0);
	m_rendering = no_call;
	work();
}

void command_stream::pipeline_barrier2(VkDependencyInfo const *Dependency)
{
	assert(Dependency->pNext == nullptr);
	auto const call{add(call_pipeline_barrier, sizeof(barrier_call))};
	auto const memory{append(Dependency->pMemoryBarriers, sizeof(VkMemoryBarrier2) * Dependency->memoryBarrierCount)};
	auto const buffers{append(Dependency->pBufferMemoryBarriers, sizeof(VkBufferMemoryBarrier2) * Dependency->bufferMemoryBarrierCount)};
	auto const images{append(Dependency->pImageMemoryBarriers, sizeof(VkImageMemoryBarrier2) * Dependency->imageMemoryBarrierCount)};
	auto &data{payload<barrier_call>(call)};
	data.flags = Dependency->dependencyFlags;
	data.memory_count = Dependency->memoryBarrierCount;
	data.buffer_count = Dependency->bufferMemoryBarrierCount;
	data.image_count = Dependency->imageMemoryBarrierCount;
	data.memory = memory;
	data.buffers = buffers;
	data.images = images;
	if (m_compute)
	{
		compute_queue_barriers(memory != no_data ? reinterpret_cast<VkMemoryBarrier2 *>(m_data.data() + memory) : nullptr, data.memory_count);
		compute_queue_barriers(buffers != no_data ? reinterpret_cast<VkBufferMemoryBarrier2 *>(m_data.data() + buffers) : nullptr, data.buffer_count);
		compute_queue_barriers(images != no_data ? reinterpret_cast<VkImageMemoryBarrier2 *>(m_data.data() + images) : nullptr, data.image_count);
	}
	close(call);
}

void command_stream::bind_pipeline(VkPipelineBindPoint const BindPoint, VkPipeline const Pipeline)
{
	auto const call{add(call_bind_pipeline, sizeof(pipeline_call))};
	payload<pipeline_call>(call) = pipeline_call{BindPoint, Pipeline};
	remember(BindPoint == VK_PIPELINE_BIND_POINT_COMPUTE ? pipeline_compute : pipeline_graphics, call);
}

void command_stream::bind_descriptor_sets2(VkBindDescriptorSetsInfo const *Info)
{
	auto const call{add(call_bind_descriptor_sets, sizeof(sets_call))};
	auto const sets{append(Info->pDescriptorSets, sizeof(VkDescriptorSet) * Info->descriptorSetCount)};
	auto const offsets{append(Info->pDynamicOffsets, sizeof(std::uint32_t) * Info->dynamicOffsetCount)};
	auto &data{payload<sets_call>(call)};
	data.info = *Info;
	assert(Info->pNext == nullptr);
	data.info.pNext = nullptr;
	data.info.pDescriptorSets = nullptr;
	data.info.pDynamicOffsets = nullptr;
	data.sets = sets;
	data.offsets = offsets;
	close(call);
	assert(Info->firstSet + Info->descriptorSetCount <= set_slots);
	for (std::uint32_t set = Info->firstSet; (set < Info->firstSet + Info->descriptorSetCount) && (set < set_slots); ++set)
	{
		// a bind for the stages of both kinds stands for both
		if (compute_stages(Info->stageFlags))
		{
			remember(sets_compute + set, call);
		}
		if (graphics_stages(Info->stageFlags))
		{
			remember(sets_graphics + set, call);
		}
	}
}

void command_stream::push_descriptor_set_with_template2(VkPushDescriptorSetWithTemplateInfo const *Info, std::size_t const Size, VkPipelineBindPoint const BindPoint)
{
	auto const call{add(call_push_descriptors, sizeof(pushed_call))};
	auto const data{append(Info->pData, Size)};
	auto &call_data{payload<pushed_call>(call)};
	call_data.info = *Info;
	assert(Info->pNext == nullptr);
	call_data.info.pNext = nullptr;
	call_data.info.pData = nullptr;
	call_data.data = data;
	close(call);
	assert(Info->set < set_slots);
	if (Info->set < set_slots)
	{
		remember((BindPoint == VK_PIPELINE_BIND_POINT_COMPUTE ? pushed_compute : pushed_graphics) + Info->set, call);
	}
}

void command_stream::push_constants2(VkPushConstantsInfo const *Info)
{
	auto const call{add(call_push_constants, sizeof(constants_call))};
	auto const data{append(Info->pValues, Info->size)};
	auto &call_data{payload<constants_call>(call)};
	call_data.info = *Info;
	assert(Info->pNext == nullptr);
	call_data.info.pNext = nullptr;
	call_data.info.pValues = nullptr;
	call_data.data = data;
	close(call);
	// a push of a part of the constants leaves the rest as the pushes before it made them: the last push of each range and stages
	// is kept, the pushes are made again in their order
	m_pushes.erase(std::remove_if(m_pushes.begin(), m_pushes.end(),
	                              [&](std::size_t const Push) {
		                              auto const &info{payload<constants_call>(Push).info};
		                              return (info.offset == Info->offset) && (info.size == Info->size) && (info.stageFlags == Info->stageFlags);
	                              }),
	               m_pushes.end());
	m_pushes.push_back(call);
}

void command_stream::bind_vertex_buffers(std::uint32_t const First, std::uint32_t const Count, VkBuffer const *Buffers, VkDeviceSize const *Offsets)
{
	auto const call{add(call_bind_vertex_buffers, sizeof(vertex_buffers_call))};
	auto const buffers{append(Buffers, sizeof(VkBuffer) * Count)};
	auto const offsets{append(Offsets, sizeof(VkDeviceSize) * Count)};
	payload<vertex_buffers_call>(call) = vertex_buffers_call{First, Count, buffers, offsets};
	close(call);
	assert(First + Count <= vertex_slots);
	for (std::uint32_t binding = First; (binding < First + Count) && (binding < vertex_slots); ++binding)
	{
		remember(vertex_buffers + binding, call);
	}
}

void command_stream::bind_index_buffer(VkBuffer const Buffer, VkDeviceSize const Offset, VkIndexType const Type)
{
	auto const call{add(call_bind_index_buffer, sizeof(index_buffer_call))};
	payload<index_buffer_call>(call) = index_buffer_call{Buffer, Offset, Type};
	remember(index_buffer, call);
}

void command_stream::draw(std::uint32_t const Vertices, std::uint32_t const Instances, std::uint32_t const FirstVertex, std::uint32_t const FirstInstance)
{
	auto const call{add(call_draw, sizeof(draw_call))};
	payload<draw_call>(call) = draw_call{Vertices, Instances, FirstVertex, FirstInstance};
	work();
}

void command_stream::draw_indexed(std::uint32_t const Indices, std::uint32_t const Instances, std::uint32_t const FirstIndex, std::int32_t const VertexOffset, std::uint32_t const FirstInstance)
{
	auto const call{add(call_draw_indexed, sizeof(draw_indexed_call))};
	payload<draw_indexed_call>(call) = draw_indexed_call{Indices, Instances, FirstIndex, VertexOffset, FirstInstance};
	work();
}

void command_stream::draw_indirect(VkBuffer const Buffer, VkDeviceSize const Offset, std::uint32_t const Count, std::uint32_t const Stride)
{
	auto const call{add(call_draw_indirect, sizeof(indirect_call))};
	payload<indirect_call>(call) = indirect_call{Buffer, Offset, Count, Stride};
	work();
}

void command_stream::draw_indexed_indirect(VkBuffer const Buffer, VkDeviceSize const Offset, std::uint32_t const Count, std::uint32_t const Stride)
{
	auto const call{add(call_draw_indexed_indirect, sizeof(indirect_call))};
	payload<indirect_call>(call) = indirect_call{Buffer, Offset, Count, Stride};
	work();
}

void command_stream::dispatch(std::uint32_t const X, std::uint32_t const Y, std::uint32_t const Z)
{
	auto const call{add(call_dispatch, sizeof(dispatch_call))};
	payload<dispatch_call>(call) = dispatch_call{X, Y, Z};
	work();
}

void command_stream::dispatch_indirect(VkBuffer const Buffer, VkDeviceSize const Offset)
{
	auto const call{add(call_dispatch_indirect, sizeof(indirect_call))};
	payload<indirect_call>(call) = indirect_call{Buffer, Offset, 0, 0};
	work();
}

void command_stream::set_viewport(std::uint32_t const First, std::uint32_t const Count, VkViewport const *Viewports)
{
	auto const call{add(call_set_viewport, sizeof(viewports_call))};
	auto const data{append(Viewports, sizeof(VkViewport) * Count)};
	payload<viewports_call>(call) = viewports_call{First, Count, data};
	close(call);
	remember(viewport, call);
}

void command_stream::set_scissor(std::uint32_t const First, std::uint32_t const Count, VkRect2D const *Scissors)
{
	auto const call{add(call_set_scissor, sizeof(viewports_call))};
	auto const data{append(Scissors, sizeof(VkRect2D) * Count)};
	payload<viewports_call>(call) = viewports_call{First, Count, data};
	close(call);
	remember(scissor, call);
}

#define VK14_VALUE_CALL(Name, Type, Call, Slot)                                                                                                                                                        \
	void command_stream::Name(Type const Value)                                                                                                                                                         \
	{                                                                                                                                                                                                   \
		auto const call{add(Call, sizeof(value_call))};                                                                                                                                                 \
		payload<value_call>(call) = value_call{static_cast<std::uint32_t>(Value)};                                                                                                                     \
		remember(Slot, call);                                                                                                                                                                           \
	}
VK14_VALUE_CALL(set_cull_mode, VkCullModeFlags, call_set_cull_mode, cull_mode)
VK14_VALUE_CALL(set_front_face, VkFrontFace, call_set_front_face, front_face)
VK14_VALUE_CALL(set_primitive_topology, VkPrimitiveTopology, call_set_topology, topology)
VK14_VALUE_CALL(set_depth_test_enable, VkBool32, call_set_depth_test, depth_test)
VK14_VALUE_CALL(set_depth_write_enable, VkBool32, call_set_depth_write, depth_write)
VK14_VALUE_CALL(set_depth_compare_op, VkCompareOp, call_set_depth_compare, depth_compare)
VK14_VALUE_CALL(set_depth_bias_enable, VkBool32, call_set_depth_bias_enable, depth_bias_enable)
VK14_VALUE_CALL(set_primitive_restart_enable, VkBool32, call_set_primitive_restart, primitive_restart)
VK14_VALUE_CALL(set_stencil_test_enable, VkBool32, call_set_stencil_test, stencil_test)
#undef VK14_VALUE_CALL

void command_stream::set_depth_bias(float const Constant, float const Clamp, float const Slope)
{
	auto const call{add(call_set_depth_bias, sizeof(bias_call))};
	payload<bias_call>(call) = bias_call{Constant, Clamp, Slope};
	remember(depth_bias, call);
}

void command_stream::fill_buffer(VkBuffer const Buffer, VkDeviceSize const Offset, VkDeviceSize const Size, std::uint32_t const Data)
{
	auto const call{add(call_fill_buffer, sizeof(fill_call))};
	payload<fill_call>(call) = fill_call{Buffer, Offset, Size, Data};
	work();
}

void command_stream::update_buffer(VkBuffer const Buffer, VkDeviceSize const Offset, VkDeviceSize const Size, void const *Data)
{
	auto const call{add(call_update_buffer, sizeof(update_call))};
	auto const data{append(Data, static_cast<std::size_t>(Size))};
	payload<update_call>(call) = update_call{Buffer, Offset, Size, data};
	close(call);
	work();
}

void command_stream::copy_buffer2(VkCopyBufferInfo2 const *Info)
{
	auto const call{add(call_copy_buffer, sizeof(copy_call<VkCopyBufferInfo2>))};
	auto const regions{append(Info->pRegions, sizeof(VkBufferCopy2) * Info->regionCount)};
	auto &data{payload<copy_call<VkCopyBufferInfo2>>(call)};
	data.info = *Info;
	assert(Info->pNext == nullptr);
	data.info.pNext = nullptr;
	data.info.pRegions = nullptr;
	data.regions = regions;
	close(call);
	work();
}

void command_stream::copy_image2(VkCopyImageInfo2 const *Info)
{
	auto const call{add(call_copy_image, sizeof(copy_call<VkCopyImageInfo2>))};
	auto const regions{append(Info->pRegions, sizeof(VkImageCopy2) * Info->regionCount)};
	auto &data{payload<copy_call<VkCopyImageInfo2>>(call)};
	data.info = *Info;
	assert(Info->pNext == nullptr);
	data.info.pNext = nullptr;
	data.info.pRegions = nullptr;
	data.regions = regions;
	close(call);
	work();
}

void command_stream::copy_buffer_to_image2(VkCopyBufferToImageInfo2 const *Info)
{
	auto const call{add(call_copy_buffer_to_image, sizeof(copy_call<VkCopyBufferToImageInfo2>))};
	auto const regions{append(Info->pRegions, sizeof(VkBufferImageCopy2) * Info->regionCount)};
	auto &data{payload<copy_call<VkCopyBufferToImageInfo2>>(call)};
	data.info = *Info;
	assert(Info->pNext == nullptr);
	data.info.pNext = nullptr;
	data.info.pRegions = nullptr;
	data.regions = regions;
	close(call);
	work();
}

void command_stream::copy_image_to_buffer2(VkCopyImageToBufferInfo2 const *Info)
{
	auto const call{add(call_copy_image_to_buffer, sizeof(copy_call<VkCopyImageToBufferInfo2>))};
	auto const regions{append(Info->pRegions, sizeof(VkBufferImageCopy2) * Info->regionCount)};
	auto &data{payload<copy_call<VkCopyImageToBufferInfo2>>(call)};
	data.info = *Info;
	assert(Info->pNext == nullptr);
	data.info.pNext = nullptr;
	data.info.pRegions = nullptr;
	data.regions = regions;
	close(call);
	work();
}

void command_stream::blit_image2(VkBlitImageInfo2 const *Info)
{
	auto const call{add(call_blit_image, sizeof(copy_call<VkBlitImageInfo2>))};
	auto const regions{append(Info->pRegions, sizeof(VkImageBlit2) * Info->regionCount)};
	auto &data{payload<copy_call<VkBlitImageInfo2>>(call)};
	data.info = *Info;
	assert(Info->pNext == nullptr);
	data.info.pNext = nullptr;
	data.info.pRegions = nullptr;
	data.regions = regions;
	close(call);
	work();
}

void command_stream::clear_color_image(VkImage const Image, VkImageLayout const Layout, VkClearColorValue const *Color, std::uint32_t const Count, VkImageSubresourceRange const *Ranges)
{
	auto const call{add(call_clear_color_image, sizeof(clear_call))};
	auto const ranges{append(Ranges, sizeof(VkImageSubresourceRange) * Count)};
	auto &data{payload<clear_call>(call)};
	data.image = Image;
	data.layout = Layout;
	data.colour = *Color;
	data.depth = {};
	data.count = Count;
	data.ranges = ranges;
	close(call);
	work();
}

void command_stream::clear_depth_stencil_image(VkImage const Image, VkImageLayout const Layout, VkClearDepthStencilValue const *Value, std::uint32_t const Count, VkImageSubresourceRange const *Ranges)
{
	auto const call{add(call_clear_depth_stencil_image, sizeof(clear_call))};
	auto const ranges{append(Ranges, sizeof(VkImageSubresourceRange) * Count)};
	auto &data{payload<clear_call>(call)};
	data.image = Image;
	data.layout = Layout;
	data.colour = {};
	data.depth = *Value;
	data.count = Count;
	data.ranges = ranges;
	close(call);
	work();
}

void command_stream::write_timestamp2(VkPipelineStageFlags2 const Stage, VkQueryPool const Pool, std::uint32_t const Query)
{
	auto const call{add(call_write_timestamp, sizeof(query_call))};
	payload<query_call>(call) = query_call{Stage, Pool, Query, 0, 0};
}

void command_stream::reset_query_pool(VkQueryPool const Pool, std::uint32_t const First, std::uint32_t const Count)
{
	auto const call{add(call_reset_query_pool, sizeof(query_call))};
	payload<query_call>(call) = query_call{VK_PIPELINE_STAGE_2_NONE, Pool, First, Count, 0};
}

void command_stream::begin_query(VkQueryPool const Pool, std::uint32_t const Query, VkQueryControlFlags const Flags)
{
	auto const call{add(call_begin_query, sizeof(query_call))};
	payload<query_call>(call) = query_call{VK_PIPELINE_STAGE_2_NONE, Pool, Query, 0, Flags};
	// a query doesn't go on in another command buffer: no cut while it's active
	++m_queries;
}

void command_stream::end_query(VkQueryPool const Pool, std::uint32_t const Query)
{
	auto const call{add(call_end_query, sizeof(query_call))};
	payload<query_call>(call) = query_call{VK_PIPELINE_STAGE_2_NONE, Pool, Query, 0, 0};
	if (m_queries > 0)
	{
		--m_queries;
	}
}

void command_stream::begin_debug_utils_label(VkDebugUtilsLabelEXT const *Label)
{
	auto const call{add(call_begin_label, sizeof(label_call))};
	auto const name{append(Label->pLabelName, (Label->pLabelName != nullptr) ? std::strlen(Label->pLabelName) + 1 : 0)};
	auto &data{payload<label_call>(call)};
	std::copy(std::begin(Label->color), std::end(Label->color), data.colour);
	data.name = name;
	close(call);
}

void command_stream::end_debug_utils_label()
{
	add(call_end_label, 0);
}

void command_stream::set_checkpoint(std::uint64_t const Marker)
{
	auto const call{add(call_set_checkpoint, sizeof(checkpoint_call))};
	payload<checkpoint_call>(call) = checkpoint_call{Marker};
}

void command_stream::build_acceleration_structures(std::uint32_t const Count, VkAccelerationStructureBuildGeometryInfoKHR const *Infos, VkAccelerationStructureBuildRangeInfoKHR const *const *Ranges)
{
	if (Count == 0)
	{
		return;
	}
	std::vector<VkAccelerationStructureBuildGeometryInfoKHR> infos(Infos, Infos + Count);
	std::vector<VkAccelerationStructureGeometryKHR> geometries;
	std::vector<VkAccelerationStructureBuildRangeInfoKHR> ranges;
	for (std::uint32_t i = 0; i < Count; ++i)
	{
		auto const &info{Infos[i]};
		assert(info.pNext == nullptr);
		for (std::uint32_t geometry = 0; geometry < info.geometryCount; ++geometry)
		{
			geometries.push_back(info.pGeometries != nullptr ? info.pGeometries[geometry] : *info.ppGeometries[geometry]);
			ranges.push_back(Ranges[i][geometry]);
		}
		infos[i].pGeometries = nullptr;
		infos[i].ppGeometries = nullptr;
	}
	auto const call{add(call_build_acceleration_structures, sizeof(build_call))};
	auto const infodata{append(infos.data(), sizeof(VkAccelerationStructureBuildGeometryInfoKHR) * infos.size())};
	auto const geometrydata{append(geometries.data(), sizeof(VkAccelerationStructureGeometryKHR) * geometries.size())};
	auto const rangedata{append(ranges.data(), sizeof(VkAccelerationStructureBuildRangeInfoKHR) * ranges.size())};
	payload<build_call>(call) = build_call{Count, infodata, geometrydata, rangedata};
	close(call);
	work();
}

void command_stream::external(external_function const Function, void *const Data)
{
	assert(m_rendering == no_call);
	auto const call{add(call_external, sizeof(external_call))};
	payload<external_call>(call) = external_call{Function, Data};
	work();
}

// the replay

void command_stream::execute(std::size_t const Call, VkCommandBuffer const Buffer, VkRenderingFlags const Flags) const
{
	auto const &header{*reinterpret_cast<call_header const *>(m_data.data() + Call)};
	switch (header.type)
	{
	case call_begin_rendering:
	{
		auto const &data{payload<rendering_call>(Call)};
		auto info{data.info};
		info.flags |= Flags;
		info.pColorAttachments = (data.colours != no_data ? at<VkRenderingAttachmentInfo>(data.colours) : nullptr);
		info.pDepthAttachment = (data.depth != no_data ? at<VkRenderingAttachmentInfo>(data.depth) : nullptr);
		info.pStencilAttachment = (data.stencil != no_data ? at<VkRenderingAttachmentInfo>(data.stencil) : nullptr);
		vkCmdBeginRendering(Buffer, &info);
		break;
	}
	case call_end_rendering:
		vkCmdEndRendering(Buffer);
		break;
	case call_pipeline_barrier:
	{
		auto const &data{payload<barrier_call>(Call)};
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.dependencyFlags = data.flags;
		dependency.memoryBarrierCount = data.memory_count;
		dependency.pMemoryBarriers = (data.memory != no_data ? at<VkMemoryBarrier2>(data.memory) : nullptr);
		dependency.bufferMemoryBarrierCount = data.buffer_count;
		dependency.pBufferMemoryBarriers = (data.buffers != no_data ? at<VkBufferMemoryBarrier2>(data.buffers) : nullptr);
		dependency.imageMemoryBarrierCount = data.image_count;
		dependency.pImageMemoryBarriers = (data.images != no_data ? at<VkImageMemoryBarrier2>(data.images) : nullptr);
		vkCmdPipelineBarrier2(Buffer, &dependency);
		break;
	}
	case call_bind_pipeline:
	{
		auto const &data{payload<pipeline_call>(Call)};
		vkCmdBindPipeline(Buffer, data.bind_point, data.pipeline);
		break;
	}
	case call_bind_descriptor_sets:
	{
		auto const &data{payload<sets_call>(Call)};
		auto info{data.info};
		info.pDescriptorSets = (data.sets != no_data ? at<VkDescriptorSet>(data.sets) : nullptr);
		info.pDynamicOffsets = (data.offsets != no_data ? at<std::uint32_t>(data.offsets) : nullptr);
		vkCmdBindDescriptorSets2(Buffer, &info);
		break;
	}
	case call_push_descriptors:
	{
		auto const &data{payload<pushed_call>(Call)};
		auto info{data.info};
		info.pData = (data.data != no_data ? m_data.data() + data.data : nullptr);
		vkCmdPushDescriptorSetWithTemplate2(Buffer, &info);
		break;
	}
	case call_push_constants:
	{
		auto const &data{payload<constants_call>(Call)};
		auto info{data.info};
		info.pValues = (data.data != no_data ? m_data.data() + data.data : nullptr);
		vkCmdPushConstants2(Buffer, &info);
		break;
	}
	case call_bind_vertex_buffers:
	{
		auto const &data{payload<vertex_buffers_call>(Call)};
		vkCmdBindVertexBuffers(Buffer, data.first, data.count, at<VkBuffer>(data.buffers), at<VkDeviceSize>(data.offsets));
		break;
	}
	case call_bind_index_buffer:
	{
		auto const &data{payload<index_buffer_call>(Call)};
		vkCmdBindIndexBuffer(Buffer, data.buffer, data.offset, data.type);
		break;
	}
	case call_draw:
	{
		auto const &data{payload<draw_call>(Call)};
		vkCmdDraw(Buffer, data.vertices, data.instances, data.first_vertex, data.first_instance);
		break;
	}
	case call_draw_indexed:
	{
		auto const &data{payload<draw_indexed_call>(Call)};
		vkCmdDrawIndexed(Buffer, data.indices, data.instances, data.first_index, data.vertex_offset, data.first_instance);
		break;
	}
	case call_draw_indirect:
	{
		auto const &data{payload<indirect_call>(Call)};
		vkCmdDrawIndirect(Buffer, data.buffer, data.offset, data.count, data.stride);
		break;
	}
	case call_draw_indexed_indirect:
	{
		auto const &data{payload<indirect_call>(Call)};
		vkCmdDrawIndexedIndirect(Buffer, data.buffer, data.offset, data.count, data.stride);
		break;
	}
	case call_dispatch:
	{
		auto const &data{payload<dispatch_call>(Call)};
		vkCmdDispatch(Buffer, data.x, data.y, data.z);
		break;
	}
	case call_dispatch_indirect:
	{
		auto const &data{payload<indirect_call>(Call)};
		vkCmdDispatchIndirect(Buffer, data.buffer, data.offset);
		break;
	}
	case call_set_viewport:
	{
		auto const &data{payload<viewports_call>(Call)};
		vkCmdSetViewport(Buffer, data.first, data.count, at<VkViewport>(data.data));
		break;
	}
	case call_set_scissor:
	{
		auto const &data{payload<viewports_call>(Call)};
		vkCmdSetScissor(Buffer, data.first, data.count, at<VkRect2D>(data.data));
		break;
	}
	case call_set_cull_mode:
		vkCmdSetCullMode(Buffer, static_cast<VkCullModeFlags>(payload<value_call>(Call).value));
		break;
	case call_set_front_face:
		vkCmdSetFrontFace(Buffer, static_cast<VkFrontFace>(payload<value_call>(Call).value));
		break;
	case call_set_topology:
		vkCmdSetPrimitiveTopology(Buffer, static_cast<VkPrimitiveTopology>(payload<value_call>(Call).value));
		break;
	case call_set_depth_test:
		vkCmdSetDepthTestEnable(Buffer, static_cast<VkBool32>(payload<value_call>(Call).value));
		break;
	case call_set_depth_write:
		vkCmdSetDepthWriteEnable(Buffer, static_cast<VkBool32>(payload<value_call>(Call).value));
		break;
	case call_set_depth_compare:
		vkCmdSetDepthCompareOp(Buffer, static_cast<VkCompareOp>(payload<value_call>(Call).value));
		break;
	case call_set_depth_bias_enable:
		vkCmdSetDepthBiasEnable(Buffer, static_cast<VkBool32>(payload<value_call>(Call).value));
		break;
	case call_set_depth_bias:
	{
		auto const &data{payload<bias_call>(Call)};
		vkCmdSetDepthBias(Buffer, data.constant, data.clamp, data.slope);
		break;
	}
	case call_set_primitive_restart:
		vkCmdSetPrimitiveRestartEnable(Buffer, static_cast<VkBool32>(payload<value_call>(Call).value));
		break;
	case call_set_stencil_test:
		vkCmdSetStencilTestEnable(Buffer, static_cast<VkBool32>(payload<value_call>(Call).value));
		break;
	case call_fill_buffer:
	{
		auto const &data{payload<fill_call>(Call)};
		vkCmdFillBuffer(Buffer, data.buffer, data.offset, data.size, data.data);
		break;
	}
	case call_update_buffer:
	{
		auto const &data{payload<update_call>(Call)};
		vkCmdUpdateBuffer(Buffer, data.buffer, data.offset, data.size, m_data.data() + data.data);
		break;
	}
	case call_copy_buffer:
	{
		auto const &data{payload<copy_call<VkCopyBufferInfo2>>(Call)};
		auto info{data.info};
		info.pRegions = at<VkBufferCopy2>(data.regions);
		vkCmdCopyBuffer2(Buffer, &info);
		break;
	}
	case call_copy_image:
	{
		auto const &data{payload<copy_call<VkCopyImageInfo2>>(Call)};
		auto info{data.info};
		info.pRegions = at<VkImageCopy2>(data.regions);
		vkCmdCopyImage2(Buffer, &info);
		break;
	}
	case call_copy_buffer_to_image:
	{
		auto const &data{payload<copy_call<VkCopyBufferToImageInfo2>>(Call)};
		auto info{data.info};
		info.pRegions = at<VkBufferImageCopy2>(data.regions);
		vkCmdCopyBufferToImage2(Buffer, &info);
		break;
	}
	case call_copy_image_to_buffer:
	{
		auto const &data{payload<copy_call<VkCopyImageToBufferInfo2>>(Call)};
		auto info{data.info};
		info.pRegions = at<VkBufferImageCopy2>(data.regions);
		vkCmdCopyImageToBuffer2(Buffer, &info);
		break;
	}
	case call_blit_image:
	{
		auto const &data{payload<copy_call<VkBlitImageInfo2>>(Call)};
		auto info{data.info};
		info.pRegions = at<VkImageBlit2>(data.regions);
		vkCmdBlitImage2(Buffer, &info);
		break;
	}
	case call_clear_color_image:
	{
		auto const &data{payload<clear_call>(Call)};
		vkCmdClearColorImage(Buffer, data.image, data.layout, &data.colour, data.count, at<VkImageSubresourceRange>(data.ranges));
		break;
	}
	case call_clear_depth_stencil_image:
	{
		auto const &data{payload<clear_call>(Call)};
		vkCmdClearDepthStencilImage(Buffer, data.image, data.layout, &data.depth, data.count, at<VkImageSubresourceRange>(data.ranges));
		break;
	}
	case call_write_timestamp:
	{
		auto const &data{payload<query_call>(Call)};
		vkCmdWriteTimestamp2(Buffer, data.stage, data.pool, data.query);
		break;
	}
	case call_reset_query_pool:
	{
		auto const &data{payload<query_call>(Call)};
		vkCmdResetQueryPool(Buffer, data.pool, data.query, data.count);
		break;
	}
	case call_begin_query:
	{
		auto const &data{payload<query_call>(Call)};
		vkCmdBeginQuery(Buffer, data.pool, data.query, data.flags);
		break;
	}
	case call_end_query:
	{
		auto const &data{payload<query_call>(Call)};
		vkCmdEndQuery(Buffer, data.pool, data.query);
		break;
	}
	case call_begin_label:
	{
		if (vkCmdBeginDebugUtilsLabelEXT != nullptr)
		{
			auto const &data{payload<label_call>(Call)};
			VkDebugUtilsLabelEXT label{};
			label.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_LABEL_EXT;
			label.pLabelName = (data.name != no_data ? at<char>(data.name) : "");
			std::copy(std::begin(data.colour), std::end(data.colour), label.color);
			vkCmdBeginDebugUtilsLabelEXT(Buffer, &label);
		}
		break;
	}
	case call_end_label:
		if (vkCmdEndDebugUtilsLabelEXT != nullptr)
		{
			vkCmdEndDebugUtilsLabelEXT(Buffer);
		}
		break;
	case call_build_acceleration_structures:
	{
		auto const &data{payload<build_call>(Call)};
		std::vector<VkAccelerationStructureBuildGeometryInfoKHR> infos(at<VkAccelerationStructureBuildGeometryInfoKHR>(data.infos), at<VkAccelerationStructureBuildGeometryInfoKHR>(data.infos) + data.count);
		std::vector<VkAccelerationStructureBuildRangeInfoKHR const *> ranges(data.count);
		auto const *geometries{data.geometries != no_data ? at<VkAccelerationStructureGeometryKHR>(data.geometries) : nullptr};
		auto const *range{data.ranges != no_data ? at<VkAccelerationStructureBuildRangeInfoKHR>(data.ranges) : nullptr};
		for (std::uint32_t i = 0; i < data.count; ++i)
		{
			infos[i].pGeometries = geometries;
			ranges[i] = range;
			if (geometries != nullptr)
			{
				geometries += infos[i].geometryCount;
				range += infos[i].geometryCount;
			}
		}
		vkCmdBuildAccelerationStructuresKHR(Buffer, data.count, infos.data(), ranges.data());
		break;
	}
	case call_set_checkpoint:
		if (vkCmdSetCheckpointNV != nullptr)
		{
			// the marker is the number itself, not a pointer to anything
			vkCmdSetCheckpointNV(Buffer, reinterpret_cast<void const *>(static_cast<std::uintptr_t>(payload<checkpoint_call>(Call).marker)));
		}
		break;
	case call_external:
	{
		auto const &data{payload<external_call>(Call)};
		data.function(Buffer, data.data);
		break;
	}
	default:
		break;
	}
}

void command_stream::replay(std::size_t const Chunk, VkCommandBuffer const Buffer) const
{
	if (false == m_pieces.empty())
	{
		if (Chunk >= m_order.size())
		{
			return;
		}
		auto const &[index, chunk]{m_order[Chunk]};
		auto const &piece{m_pieces[index]};
		// the state the parts before it left, then the chunk of its stream as it was recorded there
		for (auto const &[stream, call] : piece.inherited)
		{
			stream->execute(call, Buffer);
		}
		piece.stream->replay_chunk(chunk, Buffer);
		return;
	}
	replay_chunk(Chunk, Buffer);
}

void command_stream::replay_chunk(std::size_t const Chunk, VkCommandBuffer const Buffer) const
{
	if (Chunk >= m_chunks.size())
	{
		return;
	}
	auto const &chunk{m_chunks[Chunk]};
	if (chunk.resumed != no_call)
	{
		// the rendering the chunk before suspended, and the state it left
		execute(chunk.resumed, Buffer, VK_RENDERING_RESUMING_BIT | (chunk.suspended == chunk.resumed ? VK_RENDERING_SUSPENDING_BIT : 0));
	}
	for (auto const call : chunk.state)
	{
		execute(call, Buffer);
	}
	for (auto call = chunk.first; call < chunk.end;)
	{
		auto const &header{*reinterpret_cast<call_header const *>(m_data.data() + call)};
		// the rendering going on in the next chunk is suspended
		execute(call, Buffer, ((header.type == call_begin_rendering) && (call == chunk.suspended)) ? VK_RENDERING_SUSPENDING_BIT : 0);
		call += header.size;
	}
	if (chunk.suspended != no_call)
	{
		vkCmdEndRendering(Buffer);
	}
}

void command_stream::replay(VkCommandBuffer const Buffer) const
{
	if (false == m_pieces.empty())
	{
		// the parts in their order, each a range of the calls of its stream
		for (auto const &piece : m_pieces)
		{
			if (piece.first >= piece.end)
			{
				continue;
			}
			auto const &chunks{piece.stream->m_chunks};
			piece.stream->replay_calls(chunks[piece.first].first, chunks[piece.end - 1].end, Buffer);
		}
		return;
	}
	replay_calls(0, m_data.size(), Buffer);
}

void command_stream::replay_calls(std::size_t const First, std::size_t const End, VkCommandBuffer const Buffer) const
{
	// the chunks one after another, the cuts made whole again: each call as it was made
	for (auto call = First; call < End;)
	{
		auto const &header{*reinterpret_cast<call_header const *>(m_data.data() + call)};
		execute(call, Buffer);
		call += header.size;
	}
}

} // namespace vk14
