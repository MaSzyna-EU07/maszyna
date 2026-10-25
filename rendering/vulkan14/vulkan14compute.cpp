/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14compute.h"
#include "rendering/vulkan14/vulkan14commands.h"

#include <map>
#include <unordered_map>

namespace vk14
{

std::vector<VkDescriptorSetLayoutBinding> reflect_bindings(std::vector<std::uint32_t> const &Spirv, std::uint32_t const Set, VkShaderStageFlags const Stages)
{
	std::uint32_t constexpr op_constant{43};
	std::uint32_t constexpr op_type_image{25};
	std::uint32_t constexpr op_type_sampler{26};
	std::uint32_t constexpr op_type_sampled_image{27};
	std::uint32_t constexpr op_type_array{28};
	std::uint32_t constexpr op_type_runtime_array{29};
	std::uint32_t constexpr op_type_struct{30};
	std::uint32_t constexpr op_type_pointer{32};
	std::uint32_t constexpr op_variable{59};
	std::uint32_t constexpr op_decorate{71};
	std::uint32_t constexpr decoration_block{2};
	std::uint32_t constexpr decoration_buffer_block{3};
	std::uint32_t constexpr decoration_binding{33};
	std::uint32_t constexpr decoration_set{34};
	std::uint32_t constexpr storage_uniform_constant{0};
	std::uint32_t constexpr storage_uniform{2};
	std::uint32_t constexpr storage_storage_buffer{12};

	struct type_info
	{
		std::uint32_t opcode{0};
		std::uint32_t operand1{0}; // pointer: the type pointed to; array: the element; sampled image: the image
		std::uint32_t operand2{0}; // pointer: the storage class; array: the length; image: sampled
		std::uint32_t dim{0};
	};
	std::unordered_map<std::uint32_t, type_info> types;
	std::unordered_map<std::uint32_t, std::uint32_t> constants;
	std::unordered_map<std::uint32_t, std::uint32_t> sets;
	std::unordered_map<std::uint32_t, std::uint32_t> bindingsof;
	std::unordered_map<std::uint32_t, std::uint32_t> blocks; // struct -> decoration
	std::vector<std::pair<std::uint32_t, std::uint32_t>> variables; // id, pointer type

	for (std::size_t word = 5; word < Spirv.size();)
	{
		auto const count{Spirv[word] >> 16};
		auto const opcode{Spirv[word] & 0xffff};
		if ((count == 0) || (word + count > Spirv.size()))
		{
			break;
		}
		auto const *op{&Spirv[word]};
		switch (opcode)
		{
		case op_decorate:
			if (count >= 3)
			{
				if ((op[2] == decoration_set) && (count >= 4))
				{
					sets[op[1]] = op[3];
				}
				else if ((op[2] == decoration_binding) && (count >= 4))
				{
					bindingsof[op[1]] = op[3];
				}
				else if ((op[2] == decoration_block) || (op[2] == decoration_buffer_block))
				{
					blocks[op[1]] = op[2];
				}
			}
			break;
		case op_constant:
			if (count >= 4)
			{
				constants[op[2]] = op[3];
			}
			break;
		case op_type_image:
			if (count >= 9)
			{
				types[op[1]] = {opcode, 0, op[7], op[3]};
			}
			break;
		case op_type_sampler:
			types[op[1]] = {opcode, 0, 0, 0};
			break;
		case op_type_sampled_image:
			types[op[1]] = {opcode, op[2], 0, 0};
			break;
		case op_type_array:
			types[op[1]] = {opcode, op[2], op[3], 0};
			break;
		case op_type_runtime_array:
			types[op[1]] = {opcode, op[2], 0, 0};
			break;
		case op_type_struct:
			types[op[1]] = {opcode, 0, 0, 0};
			break;
		case op_type_pointer:
			types[op[1]] = {opcode, op[3], op[2], 0};
			break;
		case op_variable:
			if (count >= 4)
			{
				variables.emplace_back(op[2], op[1]);
			}
			break;
		default:
			break;
		}
		word += count;
	}

	std::map<std::uint32_t, VkDescriptorSetLayoutBinding> result;
	for (auto const &[id, pointertype] : variables)
	{
		auto const set{sets.find(id)};
		auto const binding{bindingsof.find(id)};
		if ((set == sets.end()) || (binding == bindingsof.end()) || (set->second != Set))
		{
			continue;
		}
		auto const pointer{types.find(pointertype)};
		if (pointer == types.end())
		{
			continue;
		}
		auto const storage{pointer->second.operand2};
		auto type{pointer->second.operand1};
		std::uint32_t descriptors{1};
		auto lookup{types.find(type)};
		if ((lookup != types.end()) && (lookup->second.opcode == op_type_array))
		{
			auto const length{constants.find(lookup->second.operand2)};
			descriptors = (length != constants.end() ? length->second : 1);
			type = lookup->second.operand1;
			lookup = types.find(type);
		}
		if (lookup == types.end())
		{
			continue;
		}
		auto descriptortype{VK_DESCRIPTOR_TYPE_MAX_ENUM};
		switch (lookup->second.opcode)
		{
		case op_type_sampled_image:
			descriptortype = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
			break;
		case op_type_image:
			descriptortype = (lookup->second.operand2 == 2 ? VK_DESCRIPTOR_TYPE_STORAGE_IMAGE : VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE);
			break;
		case op_type_sampler:
			descriptortype = VK_DESCRIPTOR_TYPE_SAMPLER;
			break;
		case op_type_struct:
		{
			auto const block{blocks.find(type)};
			if (storage == storage_storage_buffer)
			{
				descriptortype = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
			}
			else if (storage == storage_uniform)
			{
				descriptortype = ((block != blocks.end()) && (block->second == decoration_buffer_block) ? VK_DESCRIPTOR_TYPE_STORAGE_BUFFER : VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER);
			}
			break;
		}
		default:
			break;
		}
		if ((descriptortype == VK_DESCRIPTOR_TYPE_MAX_ENUM) || ((storage != storage_uniform_constant) && (storage != storage_uniform) && (storage != storage_storage_buffer)))
		{
			continue;
		}
		result[binding->second] = VkDescriptorSetLayoutBinding{binding->second, descriptortype, descriptors, Stages, nullptr};
	}
	std::vector<VkDescriptorSetLayoutBinding> bindings;
	for (auto const &entry : result)
	{
		bindings.push_back(entry.second);
	}
	return bindings;
}

bool compute_kernel::create(VkDevice Device, pipeline_library &Pipelines, std::shared_ptr<shader_code const> const &Shader, VkDescriptorSetLayout Passlayout, std::vector<std::uint32_t> const &Specialization, std::string const &Name)
{
	m_name = Name;
	auto const bindings{reflect_bindings(Shader->spirv, COMPUTE_PUSH_SET, VK_SHADER_STAGE_COMPUTE_BIT)};
	for (std::size_t i = 0; i < bindings.size(); ++i)
	{
		// the data of the template is read in the order of the bindings, one descriptor each
		if ((bindings[i].binding != i) || (bindings[i].descriptorCount != 1))
		{
			log_error("the compute shader of " + Name + " has to take the bindings of its pushed set one by one from 0");
			return false;
		}
	}
	m_count = bindings.size();
	std::vector<VkDescriptorSetLayout> sets{Passlayout};
	if (m_count > 0)
	{
		if (false == m_set.create(Device, bindings, COMPUTE_PUSH_SET, VK_PIPELINE_BIND_POINT_COMPUTE, sizeof(descriptor), Name))
		{
			return false;
		}
		sets.push_back(m_set.layout);
	}
	VkPushConstantRange const constants{VK_SHADER_STAGE_COMPUTE_BIT, 0, COMPUTE_PUSH_CONSTANTS};
	m_layout = make_pipeline_layout(Device, sets, {constants}, Name);
	if (m_layout == VK_NULL_HANDLE)
	{
		return false;
	}
	if ((m_count > 0) && (false == m_set.create_template(Device, m_layout, Name)))
	{
		return false;
	}
	m_pipeline = Pipelines.compute(Shader, m_layout, Specialization, Name);
	return m_pipeline != VK_NULL_HANDLE;
}

void compute_kernel::destroy(VkDevice Device)
{
	m_set.destroy(Device);
	if (m_layout != VK_NULL_HANDLE)
	{
		vkDestroyPipelineLayout(Device, m_layout, nullptr);
		m_layout = VK_NULL_HANDLE;
	}
	// the pipeline belongs to the pipeline library
	m_pipeline = VK_NULL_HANDLE;
}

void compute_kernel::bind(command_stream *Commands, descriptor const *Descriptors, void const *Constants, std::uint32_t const Size) const
{
	Commands->bind_pipeline(VK_PIPELINE_BIND_POINT_COMPUTE, m_pipeline);
	if ((m_count > 0) && (Descriptors != nullptr))
	{
		m_set.push(Commands, m_layout, Descriptors);
	}
	if ((Size > 0) && (Constants != nullptr))
	{
		VkPushConstantsInfo info{};
		info.sType = VK_STRUCTURE_TYPE_PUSH_CONSTANTS_INFO;
		info.layout = m_layout;
		info.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
		info.offset = 0;
		info.size = Size;
		info.pValues = Constants;
		Commands->push_constants2(&info);
	}
}

void compute_kernel::dispatch(command_stream *Commands, descriptor const *Descriptors, void const *Constants, std::uint32_t const Size, std::uint32_t const X, std::uint32_t const Y, std::uint32_t const Z) const
{
	if ((m_pipeline == VK_NULL_HANDLE) || (X == 0) || (Y == 0) || (Z == 0))
	{
		return;
	}
	bind(Commands, Descriptors, Constants, Size);
	Commands->dispatch(X, Y, Z);
}

void compute_kernel::dispatch_indirect(command_stream *Commands, descriptor const *Descriptors, void const *Constants, std::uint32_t const Size, VkBuffer Buffer, VkDeviceSize const Offset) const
{
	if (m_pipeline == VK_NULL_HANDLE)
	{
		return;
	}
	bind(Commands, Descriptors, Constants, Size);
	Commands->dispatch_indirect(Buffer, Offset);
}

void memory_barrier(command_stream *Commands, VkPipelineStageFlags2 const SrcStage, VkAccessFlags2 const SrcAccess, VkPipelineStageFlags2 const DstStage, VkAccessFlags2 const DstAccess)
{
	VkMemoryBarrier2 barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
	barrier.srcStageMask = SrcStage;
	barrier.srcAccessMask = SrcAccess;
	barrier.dstStageMask = DstStage;
	barrier.dstAccessMask = DstAccess;
	VkDependencyInfo dependency{};
	dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
	dependency.memoryBarrierCount = 1;
	dependency.pMemoryBarriers = &barrier;
	Commands->pipeline_barrier2(&dependency);
}

void compute_barrier(command_stream *Commands)
{
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT,
	               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT,
	               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT | VK_PIPELINE_STAGE_2_VERTEX_INPUT_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT |
	                   VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT,
	               VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT | VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT | VK_ACCESS_2_UNIFORM_READ_BIT |
	                   VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
}

// the buffer written with the commands of the frame, between its reads by the shaders of the frames before and after
void update_buffer(command_stream *Commands, buffer const &Target, void const *Data, VkDeviceSize const Size, VkDeviceSize const Offset)
{
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_UNIFORM_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_TRANSFER_READ_BIT,
	               VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
	Commands->update_buffer(Target.handle(), Offset, Size, Data);
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_READ_BIT | VK_ACCESS_2_UNIFORM_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
}

// readback

bool readback_ring::create(allocator const &Allocator, VkDeviceSize const Size, std::uint32_t const Frames, std::string const &Name)
{
	m_size = Size;
	m_slots.clear();
	m_slots.resize(Frames);
	for (std::uint32_t i = 0; i < Frames; ++i)
	{
		if (false == m_slots[i].data.create(Allocator, Size, VK_BUFFER_USAGE_TRANSFER_DST_BIT, memory_kind::readback, Name + " " + std::to_string(i)))
		{
			return false;
		}
	}
	return true;
}

void readback_ring::destroy()
{
	m_slots.clear();
}

void readback_ring::push(command_stream *Commands, std::uint32_t const Slot, std::uint64_t const Value, VkBuffer Source, VkDeviceSize const Offset, VkDeviceSize const Size)
{
	if (Slot >= m_slots.size())
	{
		return;
	}
	auto &slot{m_slots[Slot]};
	VkBufferCopy2 region{};
	region.sType = VK_STRUCTURE_TYPE_BUFFER_COPY_2;
	region.srcOffset = Offset;
	region.dstOffset = 0;
	region.size = std::min(Size, m_size);
	VkCopyBufferInfo2 copy{};
	copy.sType = VK_STRUCTURE_TYPE_COPY_BUFFER_INFO_2;
	copy.srcBuffer = Source;
	copy.dstBuffer = slot.data.handle();
	copy.regionCount = 1;
	copy.pRegions = &region;
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT | VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
	               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
	Commands->copy_buffer2(&copy);
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_HOST_BIT, VK_ACCESS_2_HOST_READ_BIT);
	slot.value = Value;
	slot.pending = true;
}

bool readback_ring::pop(std::uint64_t const Completed, void *Data, VkDeviceSize const Size)
{
	slot *newest{nullptr};
	for (auto &slot : m_slots)
	{
		if (slot.pending && (slot.value <= Completed) && ((newest == nullptr) || (slot.value > newest->value)))
		{
			newest = &slot;
		}
	}
	if (newest == nullptr)
	{
		return false;
	}
	for (auto &slot : m_slots)
	{
		if (slot.pending && (slot.value <= newest->value))
		{
			slot.pending = false;
		}
	}
	auto const size{std::min(Size, m_size)};
	newest->data.invalidate(0, size);
	std::memcpy(Data, newest->data.mapped(), static_cast<std::size_t>(size));
	m_popped = newest->value;
	m_poppedslot = static_cast<std::uint32_t>(newest - m_slots.data());
	return true;
}

} // namespace vk14
