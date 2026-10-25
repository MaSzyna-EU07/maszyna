/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14raytracing.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14texture.h"
#include "rendering/vulkan14/vulkan14bindings.h"

#include <algorithm>
#include <cstring>

namespace vk14
{

namespace
{

// the record of an instance the shading reads (shaders/vk14/rt_reflections.glsl, rt_record)
struct record
{
	VkDeviceAddress vertices;
	VkDeviceAddress indices;
	std::uint32_t texture;
	std::uint32_t flags;
	float cutoff;
	float glow;
	glm::vec4 colour;
};
static_assert(sizeof(record) == 48, "the record of an instance of the ray tracing doesn't match the shaders");

std::uint32_t constexpr record_alpha{1};
// the bottom level structures built in a frame at most, and their triangles: the rest waits for the next frames
std::uint32_t constexpr builds_per_frame{96};
std::uint64_t constexpr triangles_per_frame{2000000};
// the instances of the top level structure at most (the records take 24 bits of the custom index)
std::uint32_t constexpr max_instances{1u << 20};
// a bottom level structure not met in this many frames goes, and a texture keeps its index in the array this long
std::uint64_t constexpr bottom_lifetime{1800};
std::uint64_t constexpr texture_lifetime{600};
// the size of the vertices of the banks (gfx::basic_vertex)
VkDeviceSize constexpr vertex_stride{48};

VkDeviceSize grow(VkDeviceSize const Needed, VkDeviceSize const Current)
{
	return std::max(Needed, Current + Current / 2);
}

void destroy_structure(VkDevice const Device, VkAccelerationStructureKHR const Structure)
{
	if (Structure != VK_NULL_HANDLE)
	{
		vkDestroyAccelerationStructureKHR(Device, Structure, nullptr);
	}
}

// waits for the writes of the builds before the structures are read: by the builds which follow and by the shaders
void build_barrier(command_stream *Commands, VkPipelineStageFlags2 const Source, VkAccessFlags2 const SourceAccess, VkPipelineStageFlags2 const Destination, VkAccessFlags2 const DestinationAccess)
{
	VkMemoryBarrier2 barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
	barrier.srcStageMask = Source;
	barrier.srcAccessMask = SourceAccess;
	barrier.dstStageMask = Destination;
	barrier.dstAccessMask = DestinationAccess;
	VkDependencyInfo dependency{};
	dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
	dependency.memoryBarrierCount = 1;
	dependency.pMemoryBarriers = &barrier;
	Commands->pipeline_barrier2(&dependency);
}

} // namespace

ray_tracing::~ray_tracing()
{
	destroy();
}

bool ray_tracing::init(device const &Device, allocator &Memory, deletion_queue &Deletions, std::uint32_t const Frames, std::string &Reason)
{
	using namespace vk14::bindings;
	m_device = &Device;
	m_memory = &Memory;
	m_deletions = &Deletions;
	m_scratchalignment = Device.caps().scratch_alignment;
	m_texturecount = std::min<std::uint32_t>(Device.caps().max_indexed_textures, 16384);
	auto const handle{Device.handle()};

	// the structure, the records and the textures, the last binding as many as the set is made with and partly bound
	std::array<VkDescriptorSetLayoutBinding, 3> bindings{{{RT_SCENE, VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
	                                                       {RT_RECORDS, VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, 1, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr},
	                                                       {RT_TEXTURES, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, m_texturecount, VK_SHADER_STAGE_FRAGMENT_BIT, nullptr}}};
	std::array<VkDescriptorBindingFlags, 3> const flags{0, 0, VK_DESCRIPTOR_BINDING_PARTIALLY_BOUND_BIT | VK_DESCRIPTOR_BINDING_VARIABLE_DESCRIPTOR_COUNT_BIT};
	VkDescriptorSetLayoutBindingFlagsCreateInfo bindingflags{};
	bindingflags.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_BINDING_FLAGS_CREATE_INFO;
	bindingflags.bindingCount = static_cast<std::uint32_t>(flags.size());
	bindingflags.pBindingFlags = flags.data();
	VkDescriptorSetLayoutCreateInfo layout{};
	layout.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	layout.pNext = &bindingflags;
	layout.bindingCount = static_cast<std::uint32_t>(bindings.size());
	layout.pBindings = bindings.data();
	if (!VK14_VERIFY(vkCreateDescriptorSetLayout(handle, &layout, nullptr, &m_layout)))
	{
		m_layout = VK_NULL_HANDLE;
		Reason = "no layout of the set of the ray tracing";
		return false;
	}
	name_object(handle, VK_OBJECT_TYPE_DESCRIPTOR_SET_LAYOUT, handle_value(m_layout), "ray tracing set");

	std::array<VkDescriptorPoolSize, 3> const sizes{{{VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR, Frames}, {VK_DESCRIPTOR_TYPE_STORAGE_BUFFER, Frames}, {VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER, m_texturecount * Frames}}};
	VkDescriptorPoolCreateInfo pool{};
	pool.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	pool.maxSets = Frames;
	pool.poolSizeCount = static_cast<std::uint32_t>(sizes.size());
	pool.pPoolSizes = sizes.data();
	if (!VK14_VERIFY(vkCreateDescriptorPool(handle, &pool, nullptr, &m_pool)))
	{
		m_pool = VK_NULL_HANDLE;
		Reason = "no pool of the sets of the ray tracing";
		return false;
	}
	m_frames.resize(Frames);
	std::vector<VkDescriptorSetLayout> const layouts(Frames, m_layout);
	std::vector<std::uint32_t> const counts(Frames, m_texturecount);
	std::vector<VkDescriptorSet> sets(Frames, VK_NULL_HANDLE);
	VkDescriptorSetVariableDescriptorCountAllocateInfo variable{};
	variable.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_VARIABLE_DESCRIPTOR_COUNT_ALLOCATE_INFO;
	variable.descriptorSetCount = Frames;
	variable.pDescriptorCounts = counts.data();
	VkDescriptorSetAllocateInfo allocation{};
	allocation.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocation.pNext = &variable;
	allocation.descriptorPool = m_pool;
	allocation.descriptorSetCount = Frames;
	allocation.pSetLayouts = layouts.data();
	if (!VK14_VERIFY(vkAllocateDescriptorSets(handle, &allocation, sets.data())))
	{
		Reason = "no sets of the ray tracing";
		return false;
	}
	for (std::uint32_t frame = 0; frame < Frames; ++frame)
	{
		m_frames[frame].set = sets[frame];
		m_frames[frame].written_textures.assign(m_texturecount, {});
	}
	// the index 0 is the white texture of the records without one
	m_textureowners.assign(m_texturecount, 0);
	m_textureused.assign(m_texturecount, 0);
	m_freetextures.clear();
	for (std::uint32_t index = m_texturecount; index > 1; --index)
	{
		m_freetextures.push_back(index - 1);
	}
	log_info("ray traced reflections: up to " + std::to_string(m_texturecount) + " textures of the scenery, " + std::to_string(Frames) + " sets");
	return true;
}

void ray_tracing::destroy()
{
	if (m_device == nullptr)
	{
		return;
	}
	auto const handle{m_device->handle()};
	for (auto &bottom : m_bottoms)
	{
		destroy_structure(handle, bottom.second.handle);
	}
	m_bottoms.clear();
	m_bottombytes = 0;
	for (auto &frame : m_frames)
	{
		destroy_structure(handle, frame.top);
	}
	m_frames.clear();
	if (m_pool != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorPool(handle, m_pool, nullptr);
		m_pool = VK_NULL_HANDLE;
	}
	if (m_layout != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorSetLayout(handle, m_layout, nullptr);
		m_layout = VK_NULL_HANDLE;
	}
	m_textureindices.clear();
	m_device = nullptr;
}

void ray_tracing::clear(std::uint64_t const Value)
{
	if (m_device == nullptr)
	{
		return;
	}
	auto const handle{m_device->handle()};
	for (auto &entry : m_bottoms)
	{
		m_deletions->push(Value, [handle, structure = entry.second.handle, storage = entry.second.storage]() { destroy_structure(handle, structure); });
	}
	m_bottoms.clear();
	m_bottombytes = 0;
}

bool ray_tracing::ensure(std::shared_ptr<buffer> &Buffer, VkDeviceSize const Size, VkBufferUsageFlags const Usage, memory_kind const Kind, char const *Name, std::uint64_t const Value)
{
	if (Buffer && (Buffer->size() >= Size))
	{
		return true;
	}
	auto const current{Buffer ? Buffer->size() : 0};
	auto made{std::make_shared<buffer>()};
	if (false == made->create(*m_memory, grow(Size, current), Usage, Kind, Name))
	{
		return false;
	}
	if (Buffer)
	{
		m_deletions->push(Value, [old = std::move(Buffer)]() {});
	}
	Buffer = std::move(made);
	return true;
}

VkDeviceAddress ray_tracing::aligned_scratch(frame_data const &Frame) const
{
	auto const address{Frame.scratch->address()};
	auto const alignment{static_cast<VkDeviceAddress>(m_scratchalignment)};
	return (address + alignment - 1) / alignment * alignment;
}

void ray_tracing::begin_frame(command_stream *Commands, std::uint32_t const Slot, std::uint64_t const Value)
{
	if ((m_device == nullptr) || (Slot >= m_frames.size()) || (m_frames[Slot].top != VK_NULL_HANDLE) || (m_white == VK_NULL_HANDLE))
	{
		return;
	}
	// the shaders statically use the set: it gets an empty structure, a record and the white texture before anything else
	auto &frame{m_frames[Slot]};
	if (false == ensure(frame.records, sizeof(record), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, memory_kind::upload, "ray tracing records", Value))
	{
		return;
	}
	std::memset(frame.records->mapped(), 0, sizeof(record));
	frame.records->flush(0, sizeof(record));
	if (false == build_top(Commands, frame, 0, Value))
	{
		return;
	}
	// the build of the frame later on reuses the structure and the scratch
	build_barrier(Commands, VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR,
	              VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR);
	VkWriteDescriptorSetAccelerationStructureKHR structure{};
	structure.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
	structure.accelerationStructureCount = 1;
	structure.pAccelerationStructures = &frame.top;
	VkDescriptorBufferInfo const records{frame.records->handle(), 0, VK_WHOLE_SIZE};
	VkDescriptorImageInfo const white{m_whitesampler, m_white, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL};
	std::array<VkWriteDescriptorSet, 3> writes{};
	for (auto &write : writes)
	{
		write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		write.dstSet = frame.set;
		write.descriptorCount = 1;
	}
	writes[0].pNext = &structure;
	writes[0].dstBinding = vk14::bindings::RT_SCENE;
	writes[0].descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
	writes[1].dstBinding = vk14::bindings::RT_RECORDS;
	writes[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
	writes[1].pBufferInfo = &records;
	writes[2].dstBinding = vk14::bindings::RT_TEXTURES;
	writes[2].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
	writes[2].pImageInfo = &white;
	vkUpdateDescriptorSets(m_device->handle(), static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
	frame.written_top = frame.top;
	frame.written_records = frame.records->handle();
	frame.written_textures[0] = {m_white, m_whitesampler, {}};
}

bool ray_tracing::build_top(command_stream *Commands, frame_data &Frame, std::uint32_t const Count, std::uint64_t const Value)
{
	auto const handle{m_device->handle()};
	// the instances: room for the count, kept from frame to frame
	auto const capacity{std::max<std::uint32_t>(Count, 1)};
	if (false == ensure(Frame.instances, sizeof(VkAccelerationStructureInstanceKHR) * capacity, VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR,
	                    memory_kind::upload, "ray tracing instances", Value))
	{
		return false;
	}
	VkAccelerationStructureGeometryKHR geometry{};
	geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
	geometry.geometryType = VK_GEOMETRY_TYPE_INSTANCES_KHR;
	geometry.geometry.instances.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_INSTANCES_DATA_KHR;
	geometry.geometry.instances.arrayOfPointers = VK_FALSE;
	geometry.geometry.instances.data.deviceAddress = Frame.instances->address();
	VkAccelerationStructureBuildGeometryInfoKHR info{};
	info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
	info.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
	info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
	info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
	info.geometryCount = 1;
	info.pGeometries = &geometry;
	// the structure is made for more instances than the frame has, so it lasts a while as the count changes
	auto count{Count};
	if ((Frame.top == VK_NULL_HANDLE) || (Count > Frame.top_capacity))
	{
		auto const room{std::max<std::uint32_t>(static_cast<std::uint32_t>(grow(Count, Frame.top_capacity)), 64)};
		VkAccelerationStructureBuildSizesInfoKHR sizes{};
		sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
		vkGetAccelerationStructureBuildSizesKHR(handle, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &info, &room, &sizes);
		auto storage{std::make_shared<buffer>()};
		VkAccelerationStructureKHR made{VK_NULL_HANDLE};
		auto ok{storage->create(*m_memory, sizes.accelerationStructureSize, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, memory_kind::device,
		                        "ray tracing top level")};
		if (ok)
		{
			VkAccelerationStructureCreateInfoKHR create{};
			create.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
			create.buffer = storage->handle();
			create.size = sizes.accelerationStructureSize;
			create.type = VK_ACCELERATION_STRUCTURE_TYPE_TOP_LEVEL_KHR;
			ok = VK14_VERIFY(vkCreateAccelerationStructureKHR(handle, &create, nullptr, &made));
		}
		if (ok)
		{
			name_object(handle, VK_OBJECT_TYPE_ACCELERATION_STRUCTURE_KHR, handle_value(made), "ray tracing top level");
			if (Frame.top != VK_NULL_HANDLE)
			{
				m_deletions->push(Value, [handle, structure = Frame.top, oldstorage = Frame.storage]() { destroy_structure(handle, structure); });
			}
			Frame.top = made;
			Frame.storage = std::move(storage);
			Frame.top_capacity = room;
			Frame.top_scratch = sizes.buildScratchSize + m_scratchalignment;
		}
		else
		{
			// no memory for a bigger one: the old one (the set holds it) with as many instances as it has room for
			if (Frame.top == VK_NULL_HANDLE)
			{
				return false;
			}
			count = Frame.top_capacity;
		}
	}
	if (false == ensure(Frame.scratch, Frame.top_scratch, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, memory_kind::device, "ray tracing scratch", Value))
	{
		return false;
	}
	info.dstAccelerationStructure = Frame.top;
	info.scratchData.deviceAddress = aligned_scratch(Frame);
	VkAccelerationStructureBuildRangeInfoKHR range{};
	range.primitiveCount = count;
	VkAccelerationStructureBuildRangeInfoKHR const *ranges{&range};
	Commands->build_acceleration_structures(1, &info, &ranges);
	return true;
}

std::uint32_t ray_tracing::texture_index(std::uint64_t const Id, std::uint64_t const Frame)
{
	if (Id == 0)
	{
		return 0;
	}
	auto const lookup{m_textureindices.find(Id)};
	if (lookup != m_textureindices.end())
	{
		m_textureused[lookup->second] = Frame;
		return lookup->second;
	}
	if (m_freetextures.empty())
	{
		// the indices not used for a while are taken back (the sets write them anew before they're read again)
		for (std::uint32_t index = 1; index < m_texturecount; ++index)
		{
			if ((m_textureowners[index] != 0) && (m_textureused[index] + texture_lifetime < Frame))
			{
				m_textureindices.erase(m_textureowners[index]);
				m_textureowners[index] = 0;
				m_freetextures.push_back(index);
			}
		}
		if (m_freetextures.empty())
		{
			return 0;
		}
	}
	auto const index{m_freetextures.back()};
	m_freetextures.pop_back();
	m_textureindices.emplace(Id, index);
	m_textureowners[index] = Id;
	m_textureused[index] = Frame;
	return index;
}

void ray_tracing::build(command_stream *Commands, std::uint32_t const Slot, std::uint64_t const Value, std::uint64_t const Frame, std::vector<ray_draw> const &Draws, image_table const &Images)
{
	if ((m_device == nullptr) || (Slot >= m_frames.size()))
	{
		return;
	}
	auto const handle{m_device->handle()};
	auto &frame{m_frames[Slot]};

	// the bottom level structures the draws need: the ones not there yet or out of date are built, as many as a frame lets
	struct pending_build
	{
		VkAccelerationStructureGeometryKHR geometry;
		VkAccelerationStructureBuildRangeInfoKHR range;
		VkAccelerationStructureBuildGeometryInfoKHR info;
		VkDeviceSize scratch;
	};
	std::vector<pending_build> builds;
	std::uint64_t triangles{0};
	std::uint32_t waiting{0};
	VkDeviceSize scratchsize{0};
	auto const scratchalignment{static_cast<VkDeviceSize>(m_scratchalignment)};
	for (auto const &draw : Draws)
	{
		auto lookup{m_bottoms.find(draw.key)};
		if ((lookup != m_bottoms.end()) && (lookup->second.generation == draw.source.generation))
		{
			lookup->second.used = Frame;
			continue;
		}
		if ((builds.size() >= builds_per_frame) || (triangles >= triangles_per_frame))
		{
			++waiting;
			continue;
		}
		auto const &source{draw.source};
		auto const indexed{source.indices != VK_NULL_HANDLE};
		auto const primitives{source.count / 3};
		if ((primitives == 0) || (source.vertex_address == 0) || (indexed && (source.index_address == 0)) || (source.vertexcount == 0))
		{
			continue;
		}
		pending_build build{};
		build.geometry.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_KHR;
		build.geometry.geometryType = VK_GEOMETRY_TYPE_TRIANGLES_KHR;
		auto &triangles_data{build.geometry.geometry.triangles};
		triangles_data.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_GEOMETRY_TRIANGLES_DATA_KHR;
		triangles_data.vertexFormat = VK_FORMAT_R32G32B32_SFLOAT;
		triangles_data.vertexData.deviceAddress = source.vertex_address + vertex_stride * source.vertexfirst;
		triangles_data.vertexStride = vertex_stride;
		triangles_data.maxVertex = source.vertexcount - 1;
		triangles_data.indexType = (indexed ? VK_INDEX_TYPE_UINT32 : VK_INDEX_TYPE_NONE_KHR);
		triangles_data.indexData.deviceAddress = (indexed ? source.index_address + sizeof(std::uint32_t) * source.first : 0);
		build.range.primitiveCount = primitives;
		build.info.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_GEOMETRY_INFO_KHR;
		build.info.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		build.info.flags = VK_BUILD_ACCELERATION_STRUCTURE_PREFER_FAST_TRACE_BIT_KHR;
		build.info.mode = VK_BUILD_ACCELERATION_STRUCTURE_MODE_BUILD_KHR;
		build.info.geometryCount = 1;
		VkAccelerationStructureBuildSizesInfoKHR sizes{};
		sizes.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_BUILD_SIZES_INFO_KHR;
		build.info.pGeometries = &build.geometry;
		vkGetAccelerationStructureBuildSizesKHR(handle, VK_ACCELERATION_STRUCTURE_BUILD_TYPE_DEVICE_KHR, &build.info, &primitives, &sizes);
		build.info.pGeometries = nullptr;
		auto storage{std::make_shared<buffer>()};
		if (false == storage->create(*m_memory, sizes.accelerationStructureSize, VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_STORAGE_BIT_KHR | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, memory_kind::device, "ray tracing bottom level"))
		{
			continue;
		}
		VkAccelerationStructureCreateInfoKHR create{};
		create.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_CREATE_INFO_KHR;
		create.buffer = storage->handle();
		create.size = sizes.accelerationStructureSize;
		create.type = VK_ACCELERATION_STRUCTURE_TYPE_BOTTOM_LEVEL_KHR;
		VkAccelerationStructureKHR structure{VK_NULL_HANDLE};
		if (!VK14_VERIFY(vkCreateAccelerationStructureKHR(handle, &create, nullptr, &structure)))
		{
			continue;
		}
		if (lookup != m_bottoms.end())
		{
			// the data of the chunk changed: the old structure goes once the frames which read it are done
			m_bottombytes -= lookup->second.bytes;
			m_deletions->push(Value, [handle, old = lookup->second.handle, oldstorage = lookup->second.storage]() { destroy_structure(handle, old); });
		}
		VkAccelerationStructureDeviceAddressInfoKHR addressinfo{};
		addressinfo.sType = VK_STRUCTURE_TYPE_ACCELERATION_STRUCTURE_DEVICE_ADDRESS_INFO_KHR;
		addressinfo.accelerationStructure = structure;
		auto &entry{m_bottoms[draw.key]};
		entry.storage = std::move(storage);
		entry.handle = structure;
		entry.address = vkGetAccelerationStructureDeviceAddressKHR(handle, &addressinfo);
		entry.generation = source.generation;
		entry.used = Frame;
		entry.bytes = sizes.accelerationStructureSize;
		m_bottombytes += entry.bytes;
		build.info.dstAccelerationStructure = structure;
		build.scratch = scratchsize;
		scratchsize += (sizes.buildScratchSize + scratchalignment - 1) / scratchalignment * scratchalignment;
		builds.push_back(build);
		triangles += primitives;
	}

	// the vertices and the indices copied to the banks in this frame (the uploads go to the queue before the frame) are read by the builds
	if (false == builds.empty())
	{
		build_barrier(Commands, VK_PIPELINE_STAGE_2_ALL_TRANSFER_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_2_SHADER_READ_BIT);
		if (false == ensure(frame.scratch, scratchsize + scratchalignment, VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT, memory_kind::device, "ray tracing scratch", Value))
		{
			// no memory for the builds: the structures made for them go, they're tried again
			for (auto const &build : builds)
			{
				for (auto it = m_bottoms.begin(); it != m_bottoms.end(); ++it)
				{
					if (it->second.handle == build.info.dstAccelerationStructure)
					{
						m_bottombytes -= it->second.bytes;
						m_deletions->push(Value, [handle, old = it->second.handle, oldstorage = it->second.storage]() { destroy_structure(handle, old); });
						m_bottoms.erase(it);
						break;
					}
				}
			}
			builds.clear();
		}
		else
		{
			auto const scratchbase{aligned_scratch(frame)};
			std::vector<VkAccelerationStructureBuildGeometryInfoKHR> infos;
			std::vector<VkAccelerationStructureBuildRangeInfoKHR const *> ranges;
			infos.reserve(builds.size());
			ranges.reserve(builds.size());
			for (auto &build : builds)
			{
				build.info.pGeometries = &build.geometry;
				build.info.scratchData.deviceAddress = scratchbase + build.scratch;
				infos.push_back(build.info);
				ranges.push_back(&build.range);
			}
			Commands->build_acceleration_structures(static_cast<std::uint32_t>(infos.size()), infos.data(), ranges.data());
			// the top level structure reads them, and its build uses the scratch memory again
			build_barrier(Commands, VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR, VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR,
			              VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR | VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR);
		}
	}

	// the records and the instances of the draws with their structures
	std::size_t instancecount{0};
	for (auto const &draw : Draws)
	{
		auto const lookup{m_bottoms.find(draw.key)};
		if ((lookup != m_bottoms.end()) && (lookup->second.generation == draw.source.generation))
		{
			++instancecount;
		}
	}
	instancecount = std::min<std::size_t>(instancecount, max_instances);
	if ((false == ensure(frame.records, sizeof(record) * std::max<std::size_t>(instancecount, 1), VK_BUFFER_USAGE_STORAGE_BUFFER_BIT, memory_kind::upload, "ray tracing records", Value)) ||
	    (false == ensure(frame.instances, sizeof(VkAccelerationStructureInstanceKHR) * std::max<std::size_t>(instancecount, 1),
	                     VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_ACCELERATION_STRUCTURE_BUILD_INPUT_READ_ONLY_BIT_KHR, memory_kind::upload, "ray tracing instances", Value)))
	{
		instancecount = 0;
	}
	auto *records{reinterpret_cast<record *>(frame.records ? frame.records->mapped() : nullptr)};
	auto *instances{reinterpret_cast<VkAccelerationStructureInstanceKHR *>(frame.instances ? frame.instances->mapped() : nullptr)};
	std::vector<std::uint32_t> usedtextures;
	std::uint32_t written{0};
	for (auto const &draw : Draws)
	{
		if ((written >= instancecount) || (records == nullptr) || (instances == nullptr))
		{
			break;
		}
		auto const lookup{m_bottoms.find(draw.key)};
		if ((lookup == m_bottoms.end()) || (lookup->second.generation != draw.source.generation))
		{
			continue;
		}
		auto const &source{draw.source};
		auto const texture{texture_index(draw.texture, Frame)};
		if (texture != 0)
		{
			usedtextures.push_back(texture);
		}
		auto &entry{records[written]};
		entry.vertices = source.vertex_address + vertex_stride * source.vertexfirst;
		entry.indices = (source.indices != VK_NULL_HANDLE ? source.index_address + sizeof(std::uint32_t) * source.first : 0);
		entry.texture = texture;
		auto const alpha{(texture != 0) && (draw.cutoff > 0.f)};
		entry.flags = (alpha ? record_alpha : 0);
		entry.cutoff = draw.cutoff;
		entry.glow = draw.glow;
		entry.colour = glm::vec4{draw.colour, 1.f};
		auto &instance{instances[written]};
		// the rows of the transform, which glm keeps by the columns
		for (int row = 0; row < 3; ++row)
		{
			for (int column = 0; column < 4; ++column)
			{
				instance.transform.matrix[row][column] = draw.transform[column][row];
			}
		}
		instance.instanceCustomIndex = written;
		instance.mask = 0xFF;
		instance.instanceShaderBindingTableRecordOffset = 0;
		// the models have faces of one side, seen from both in the reflections; an alpha tested texture leaves holes in its faces
		instance.flags = VK_GEOMETRY_INSTANCE_TRIANGLE_FACING_CULL_DISABLE_BIT_KHR | (alpha ? VK_GEOMETRY_INSTANCE_FORCE_NO_OPAQUE_BIT_KHR : VK_GEOMETRY_INSTANCE_FORCE_OPAQUE_BIT_KHR);
		instance.accelerationStructureReference = lookup->second.address;
		++written;
	}
	if (written > 0)
	{
		frame.records->flush(0, sizeof(record) * written);
		frame.instances->flush(0, sizeof(VkAccelerationStructureInstanceKHR) * written);
	}
	if (build_top(Commands, frame, written, Value))
	{
		build_barrier(Commands, VK_PIPELINE_STAGE_2_ACCELERATION_STRUCTURE_BUILD_BIT_KHR, VK_ACCESS_2_ACCELERATION_STRUCTURE_WRITE_BIT_KHR, VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT,
		              VK_ACCESS_2_ACCELERATION_STRUCTURE_READ_BIT_KHR);
	}

	// the set of the frame: what changed in it
	std::vector<VkWriteDescriptorSet> writes;
	std::vector<VkDescriptorImageInfo> images;
	images.reserve(usedtextures.size());
	VkWriteDescriptorSetAccelerationStructureKHR structure{};
	structure.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET_ACCELERATION_STRUCTURE_KHR;
	structure.accelerationStructureCount = 1;
	structure.pAccelerationStructures = &frame.top;
	VkDescriptorBufferInfo const recordsinfo{frame.records ? frame.records->handle() : VK_NULL_HANDLE, 0, VK_WHOLE_SIZE};
	if ((frame.top != VK_NULL_HANDLE) && (frame.top != frame.written_top))
	{
		VkWriteDescriptorSet write{};
		write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		write.pNext = &structure;
		write.dstSet = frame.set;
		write.dstBinding = vk14::bindings::RT_SCENE;
		write.descriptorCount = 1;
		write.descriptorType = VK_DESCRIPTOR_TYPE_ACCELERATION_STRUCTURE_KHR;
		writes.push_back(write);
		frame.written_top = frame.top;
	}
	if (frame.records && (frame.records->handle() != frame.written_records))
	{
		VkWriteDescriptorSet write{};
		write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		write.dstSet = frame.set;
		write.dstBinding = vk14::bindings::RT_RECORDS;
		write.descriptorCount = 1;
		write.descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
		write.pBufferInfo = &recordsinfo;
		writes.push_back(write);
		frame.written_records = frame.records->handle();
	}
	std::sort(usedtextures.begin(), usedtextures.end());
	usedtextures.erase(std::unique(usedtextures.begin(), usedtextures.end()), usedtextures.end());
	for (auto const index : usedtextures)
	{
		// the texture as its material samples it; a texture which isn't a plain 2d image (or is gone) reads as white
		auto view{m_white};
		auto sampler{m_whitesampler};
		std::shared_ptr<image> source;
		if (auto const *entry{Images.find(m_textureowners[index])})
		{
			if (entry->target && (entry->target->desc().view_type == VK_IMAGE_VIEW_TYPE_2D) && (entry->sampler != VK_NULL_HANDLE))
			{
				view = entry->target->view();
				sampler = entry->sampler;
				source = entry->target;
			}
		}
		auto &cached{frame.written_textures[index]};
		if ((cached.view == view) && (cached.sampler == sampler) && (cached.source.lock() == source))
		{
			continue;
		}
		cached = {view, sampler, source};
		images.push_back({sampler, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL});
		VkWriteDescriptorSet write{};
		write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
		write.dstSet = frame.set;
		write.dstBinding = vk14::bindings::RT_TEXTURES;
		write.dstArrayElement = index;
		write.descriptorCount = 1;
		write.descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
		write.pImageInfo = &images.back();
		writes.push_back(write);
	}
	if (false == writes.empty())
	{
		vkUpdateDescriptorSets(handle, static_cast<std::uint32_t>(writes.size()), writes.data(), 0, nullptr);
	}

	// the structures not met for a while go
	for (auto it = m_bottoms.begin(); it != m_bottoms.end();)
	{
		if (it->second.used + bottom_lifetime < Frame)
		{
			m_bottombytes -= it->second.bytes;
			m_deletions->push(Value, [handle, old = it->second.handle, oldstorage = it->second.storage]() { destroy_structure(handle, old); });
			it = m_bottoms.erase(it);
		}
		else
		{
			++it;
		}
	}

	m_lastinstances = written;
	m_lastrecords = written;
	m_lastbuilt = static_cast<std::uint32_t>(builds.size());
	m_lastwaiting = waiting;
	m_lasttextures = static_cast<std::uint32_t>(usedtextures.size());
}

std::string ray_tracing::info() const
{
	return "ray traced reflections: " + std::to_string(m_lastinstances) + " instances, " + std::to_string(m_bottoms.size()) + " chunks (" + memory_size_string(m_bottombytes) + "), built " + std::to_string(m_lastbuilt) +
	       ", waiting " + std::to_string(m_lastwaiting) + ", textures " + std::to_string(m_lasttextures);
}

} // namespace vk14
