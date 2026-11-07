/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14water.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "utilities/Logs.h"

#include <cstring>

namespace vk14
{

namespace
{

std::uint32_t groups(std::size_t const Count)
{
	return static_cast<std::uint32_t>((Count + gl::WATER_GROUP_SIZE - 1) / gl::WATER_GROUP_SIZE);
}

VkBufferUsageFlags constexpr storage_usage{VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT};

} // namespace

windscreen_water::~windscreen_water()
{
	// the materials may be gone already
	set_host({});
	destroy();
}

bool windscreen_water::init(VkDevice const Device, pipeline_library &Pipelines, shader_library &Shaders, VkDescriptorSetLayout const Passlayout, allocator &Memory, uploader &Uploads,
                            deletion_queue &Deletions, frame_ring &Ring, VkDeviceSize const Alignment, host Host)
{
	m_device = Device;
	m_pipelines = &Pipelines;
	m_shaders = &Shaders;
	m_passlayout = Passlayout;
	m_memory = &Memory;
	m_uploads = &Uploads;
	m_deletions = &Deletions;
	m_ring = &Ring;
	m_alignment = std::max<VkDeviceSize>(Alignment, 16);
	m_host = std::move(Host);
	m_storage = std::make_shared<buffer>();
	m_full = m_storage->create(Memory, static_cast<VkDeviceSize>(gl::WATER_STORAGE_SIZE), storage_usage, memory_kind::device, "windscreen water");
	if (false == m_full)
	{
		// the pass sets need a buffer to hold: a small one, which no glass reads (none gets a slot)
		ErrorLog("windscreen water: no storage of " + std::to_string(gl::WATER_STORAGE_SIZE / (1024 * 1024)) + " MiB, the windscreens make their water up");
		m_storage = std::make_shared<buffer>();
		if (false == m_storage->create(Memory, 65536, storage_usage, memory_kind::device, "windscreen water"))
		{
			m_storage.reset();
			return false;
		}
	}
	return true;
}

void windscreen_water::destroy()
{
	if (m_device == VK_NULL_HANDLE)
	{
		return;
	}
	for (auto &kernel : m_kernels)
	{
		kernel.destroy(m_device);
	}
	m_storage.reset();
	m_device = VK_NULL_HANDLE;
	m_cleared = false;
	reset();
}

bool windscreen_water::gpu_init()
{
	if ((m_device == VK_NULL_HANDLE) || (false == m_full) || (false == bool(m_host.commands)) || (false == bool(m_host.texture)))
	{
		return false;
	}
	try
	{
		std::array<char const *, pass::count> const names{"water_trail.comp", "water_spawn.comp", "water_move.comp", "water_clear.comp", "water_grid.comp",
		                                                  "water_merge.comp", "water_gather.comp", "water_settle.comp", "water_bake.comp"};
		for (std::size_t i = 0; i < names.size(); ++i)
		{
			if (false == m_kernels[i].create(m_device, *m_pipelines, m_shaders->get(names[i]), m_passlayout, {}, names[i]))
			{
				throw gfx::shader_exception(std::string("no compute pipeline of ") + names[i]);
			}
		}
	}
	catch (gfx::shader_exception const &e)
	{
		ErrorLog("windscreen water: can't use the programs of the simulation: " + std::string(e.what()));
		return false;
	}
	return true;
}

void windscreen_water::begin(command_stream *const Commands)
{
	// the glasses of the frames before read the storage, the fills and the copies write it
	memory_barrier(Commands, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT,
	               VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT | VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_SHADER_STORAGE_READ_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
	if (false == m_cleared)
	{
		// nothing in the storage yet
		m_cleared = true;
		Commands->fill_buffer(m_storage->handle(), 0, VK_WHOLE_SIZE, 0);
		compute_barrier(Commands);
	}
}

void windscreen_water::gpu_setup(std::size_t const Slot, std::vector<glm::vec4> const &Axes)
{
	auto const commands{m_host.commands()};
	if ((commands == nullptr) || (false == m_full))
	{
		return;
	}
	begin(commands);
	// the drops, the grid, the paths, the header and the counts of the slot cleared with the commands of the frame
	auto const parts{gl::water_slot_parts(Slot)};
	for (std::size_t part = 0; part < parts.size(); ++part)
	{
		if (part != 1)
		{
			commands->fill_buffer(m_storage->handle(), static_cast<VkDeviceSize>(parts[part].first), static_cast<VkDeviceSize>(parts[part].second), 0);
		}
	}
	// the axes of the map copied after that, from a staging buffer kept until the gpu is done with the frame
	auto const size{std::min<VkDeviceSize>(static_cast<VkDeviceSize>(parts[1].second), static_cast<VkDeviceSize>(Axes.size() * sizeof(glm::vec4)))};
	auto staging{std::make_shared<buffer>()};
	if ((false == staging->create(*m_memory, size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, memory_kind::upload, "windscreen water map")) || (staging->mapped() == nullptr))
	{
		ErrorLog("windscreen water: no staging buffer for the map of a glass");
		compute_barrier(commands);
		return;
	}
	std::memcpy(staging->mapped(), Axes.data(), static_cast<std::size_t>(size));
	staging->flush(0, size);
	VkBufferCopy2 region{};
	region.sType = VK_STRUCTURE_TYPE_BUFFER_COPY_2;
	region.dstOffset = static_cast<VkDeviceSize>(parts[1].first);
	region.size = size;
	VkCopyBufferInfo2 copy{};
	copy.sType = VK_STRUCTURE_TYPE_COPY_BUFFER_INFO_2;
	copy.srcBuffer = staging->handle();
	copy.dstBuffer = m_storage->handle();
	copy.regionCount = 1;
	copy.pRegions = &region;
	commands->copy_buffer2(&copy);
	compute_barrier(commands);
	m_deletions->push(m_uploads->current_value(), [staging]() {});
}

ring_slice windscreen_water::block(gl::water_ubs const &Block)
{
	auto const slice{m_ring->allocate(sizeof(gl::water_ubs), m_alignment)};
	if (slice)
	{
		std::memcpy(slice.data, &Block, sizeof(gl::water_ubs));
	}
	return slice;
}

bool windscreen_water::gpu_bake(std::size_t const Slot, texture_handle const Mask, gl::water_ubs const &Block)
{
	auto const commands{m_host.commands()};
	if ((commands == nullptr) || (false == m_full) || (Mask == null_handle))
	{
		return false;
	}
	VkImageView view{VK_NULL_HANDLE};
	VkSampler sampler{VK_NULL_HANDLE};
	if ((false == m_host.texture(Mask, view, sampler)) || (view == VK_NULL_HANDLE))
	{
		return false;
	}
	auto const slice{block(Block)};
	if (false == bool(slice))
	{
		return false;
	}
	begin(commands);
	m_kernels[pass::bake].run(commands,
	                          {buffer_descriptor(m_storage->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(slice.buffer, slice.offset, sizeof(gl::water_ubs)),
	                           image_descriptor(sampler, view, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL)},
	                          groups(static_cast<std::size_t>(gl::WATER_MAP) * gl::WATER_MAP));
	compute_barrier(commands);
	return true;
}

void windscreen_water::gpu_step(gl::water_ubs const &Block, std::uint32_t const Tries)
{
	auto const commands{m_host.commands()};
	if ((commands == nullptr) || (false == m_full))
	{
		return;
	}
	auto const slice{block(Block)};
	if (false == bool(slice))
	{
		return;
	}
	begin(commands);
	std::vector<descriptor> const descriptors{buffer_descriptor(m_storage->handle(), 0, VK_WHOLE_SIZE), buffer_descriptor(slice.buffer, slice.offset, sizeof(gl::water_ubs))};
	auto const run = [&](pass const Pass, std::uint32_t const Groups) {
		if (Groups == 0)
		{
			return;
		}
		m_kernels[Pass].run(commands, descriptors, Groups, gl::WATER_SLOTS);
		compute_barrier(commands);
	};
	auto const drops{groups(gl::WATER_DROPS)};
	auto const cells{groups(static_cast<std::size_t>(gl::WATER_GRID) * gl::WATER_GRID)};
	// the wetted paths the blades went over in the step wiped
	if (Block.blades != Block.blades_before)
	{
		run(pass::trail, groups(static_cast<std::size_t>(gl::WATER_TRAIL) * gl::WATER_TRAIL));
	}
	run(pass::spawn, groups(Tries));
	run(pass::move, drops);
	run(pass::clear, cells);
	run(pass::grid, drops);
	run(pass::merge, drops);
	run(pass::gather, drops);
	run(pass::settle, drops);
	run(pass::clear, cells);
	run(pass::grid, drops);
}

} // namespace vk14
