/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14passes.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14device.h"

namespace vk14
{

image_use use_of(image &Image, VkImageLayout Layout, VkPipelineStageFlags2 Stage, VkAccessFlags2 Access)
{
	return image_use{Image.handle(), Image.aspect(), &Image.state, Layout, Stage, Access};
}

void barriers(command_stream *Commands, std::vector<image_use> const &Uses)
{
	auto const writes{VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT |
	                  VK_ACCESS_2_HOST_WRITE_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT};
	std::vector<VkImageMemoryBarrier2> list;
	for (auto const &use : Uses)
	{
		if ((use.image == VK_NULL_HANDLE) || (use.state == nullptr))
		{
			continue;
		}
		auto &state{*use.state};
		// reads after reads in the same layout need nothing, unless they are in stages the barrier after the last write didn't
		// wait for: those wait for the stages which read the image already (which came after the write)
		if ((state.layout == use.layout) && ((state.access & writes) == 0) && ((use.access & writes) == 0))
		{
			if ((use.stage & ~state.stage) != 0)
			{
				VkMemoryBarrier2 chain{};
				chain.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
				chain.srcStageMask = (state.stage != VK_PIPELINE_STAGE_2_NONE ? state.stage : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT);
				chain.dstStageMask = use.stage;
				chain.dstAccessMask = use.access;
				VkDependencyInfo dependency{};
				dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
				dependency.memoryBarrierCount = 1;
				dependency.pMemoryBarriers = &chain;
				Commands->pipeline_barrier2(&dependency);
			}
			state.stage |= use.stage;
			state.access |= use.access;
			continue;
		}
		VkImageMemoryBarrier2 barrier{};
		barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
		barrier.srcStageMask = (state.stage != VK_PIPELINE_STAGE_2_NONE ? state.stage : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT);
		barrier.srcAccessMask = (state.access & writes);
		barrier.dstStageMask = use.stage;
		barrier.dstAccessMask = use.access;
		barrier.oldLayout = state.layout;
		barrier.newLayout = use.layout;
		barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		barrier.image = use.image;
		barrier.subresourceRange = {use.aspect, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS};
		list.push_back(barrier);
		state = image_state{use.layout, use.stage, use.access};
	}
	if (list.empty())
	{
		return;
	}
	VkDependencyInfo dependency{};
	dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
	dependency.imageMemoryBarrierCount = static_cast<std::uint32_t>(list.size());
	dependency.pImageMemoryBarriers = list.data();
	Commands->pipeline_barrier2(&dependency);
}

void barriers_again(command_stream *Commands, std::vector<image_use> const &Uses)
{
	// what barriers() makes of a use following the same use
	std::vector<image_use> uses;
	std::vector<image_state> states(Uses.size());
	for (std::size_t i = 0; i < Uses.size(); ++i)
	{
		auto use{Uses[i]};
		states[i] = image_state{use.layout, use.stage, use.access};
		use.state = &states[i];
		uses.push_back(use);
	}
	barriers(Commands, uses);
}

pass_list::~pass_list()
{
	destroy();
}

bool pass_list::create(device const &Device, std::uint32_t Frames, std::string &Reason)
{
	m_device = Device.handle();
	m_period = Device.caps().timestamp_period;
	// the bits of the timestamps of the queue the frames go to
	std::uint32_t count{0};
	vkGetPhysicalDeviceQueueFamilyProperties(Device.physical(), &count, nullptr);
	std::vector<VkQueueFamilyProperties> families(count);
	vkGetPhysicalDeviceQueueFamilyProperties(Device.physical(), &count, families.data());
	auto const bits{(Device.families().graphics < count) ? families[Device.families().graphics].timestampValidBits : 64u};
	m_mask = (bits >= 64 ? ~0ull : ((1ull << bits) - 1));
	auto const asyncbits{(Device.families().compute < count) ? families[Device.families().compute].timestampValidBits : 64u};
	m_asyncmask = (asyncbits >= 64 ? ~0ull : ((1ull << asyncbits) - 1));

	m_frames.resize(Frames);
	for (std::uint32_t i = 0; i < Frames; ++i)
	{
		VkQueryPoolCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_QUERY_POOL_CREATE_INFO;
		info.queryType = VK_QUERY_TYPE_TIMESTAMP;
		info.queryCount = max_passes * 2;
		if (!VK14_VERIFY(vkCreateQueryPool(m_device, &info, nullptr, &m_frames[i].pool)))
		{
			m_frames[i].pool = VK_NULL_HANDLE;
			Reason = "no query pool for the times of the passes";
			return false;
		}
		// the queries are reset from the host before their first use (hostQueryReset)
		vkResetQueryPool(m_device, m_frames[i].pool, 0, info.queryCount);
		name_object(m_device, VK_OBJECT_TYPE_QUERY_POOL, handle_value(m_frames[i].pool), "pass times " + std::to_string(i));
		info.queryCount = max_async * 2;
		if (!VK14_VERIFY(vkCreateQueryPool(m_device, &info, nullptr, &m_frames[i].async_pool)))
		{
			m_frames[i].async_pool = VK_NULL_HANDLE;
			Reason = "no query pool for the times of the passes";
			return false;
		}
		vkResetQueryPool(m_device, m_frames[i].async_pool, 0, info.queryCount);
		name_object(m_device, VK_OBJECT_TYPE_QUERY_POOL, handle_value(m_frames[i].async_pool), "async pass times " + std::to_string(i));
	}
	return true;
}

void pass_list::destroy()
{
	for (auto &frame : m_frames)
	{
		if (frame.pool != VK_NULL_HANDLE)
		{
			vkDestroyQueryPool(m_device, frame.pool, nullptr);
		}
		if (frame.async_pool != VK_NULL_HANDLE)
		{
			vkDestroyQueryPool(m_device, frame.async_pool, nullptr);
		}
	}
	m_frames.clear();
	m_device = VK_NULL_HANDLE;
}

void pass_list::begin_frame(command_stream *Commands, std::uint32_t Frame, std::uint64_t Number)
{
	m_commands = Commands;
	m_graphics = nullptr;
	m_async = false;
	m_frame = Frame;
	m_open = -1;
	auto &frame{m_frames[Frame]};
	frame.names.clear();
	frame.used = 0;
	frame.number = Number;
	frame.recorded = true;
	Commands->reset_query_pool(frame.pool, 0, max_passes * 2);
	// the gpu is done with the frame which used the slot before: the queries of the compute queue are reset from here, the queue may
	// write them before the graphics queue gets to a reset of its own
	if (frame.async_used > 0)
	{
		vkResetQueryPool(m_device, frame.async_pool, 0, max_async * 2);
	}
	frame.async_names.clear();
	frame.async_used = 0;
}

void pass_list::begin_pass(char const *Name, std::vector<image_use> const &Uses)
{
	if (m_open >= 0)
	{
		end_pass();
	}
	barriers(m_commands, Uses);
	begin_label(m_commands, Name);
	auto &frame{m_frames[m_frame]};
	if (m_async)
	{
		if ((frame.async_used < max_async) && (m_asyncmask != 0))
		{
			m_commands->write_timestamp2(VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, frame.async_pool, frame.async_used * 2);
			frame.async_names.emplace_back(Name);
			m_open = static_cast<int>(frame.async_used);
			++frame.async_used;
		}
		else
		{
			m_open = static_cast<int>(max_passes);
		}
		return;
	}
	// (no timestamps on a queue without valid bits; the device is chosen with them, this is for safety)
	if ((frame.used < max_passes) && (m_mask != 0))
	{
		m_commands->write_timestamp2(VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT, frame.pool, frame.used * 2);
		frame.names.emplace_back(Name);
		m_open = static_cast<int>(frame.used);
		++frame.used;
	}
	else
	{
		m_open = static_cast<int>(max_passes);
	}
}

void pass_list::end_pass()
{
	if (m_open < 0)
	{
		return;
	}
	if (m_open < static_cast<int>(max_passes))
	{
		m_commands->write_timestamp2(VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT, m_async ? m_frames[m_frame].async_pool : m_frames[m_frame].pool, static_cast<std::uint32_t>(m_open) * 2 + 1);
	}
	end_label(m_commands);
	m_open = -1;
}

void pass_list::begin_async(command_stream *Commands)
{
	end_pass();
	if (m_async)
	{
		return;
	}
	m_graphics = m_commands;
	m_commands = Commands;
	m_async = true;
}

void pass_list::end_async()
{
	end_pass();
	if (false == m_async)
	{
		return;
	}
	m_commands = m_graphics;
	m_graphics = nullptr;
	m_async = false;
}

void pass_list::end_frame()
{
	end_async();
	end_pass();
	m_commands = nullptr;
}

void pass_list::collect(std::uint32_t Frame)
{
	auto &frame{m_frames[Frame]};
	if ((false == frame.recorded) || (frame.used == 0))
	{
		return;
	}
	frame.recorded = false;
	// a running average, so the numbers can be read
	auto const blend = [](double &Average, double const Value) { Average = (Average <= 0.0 ? Value : Average * 0.9 + Value * 0.1); };
	// the passes of the compute queue, apart
	if (frame.async_used > 0)
	{
		std::vector<std::uint64_t> stamps(frame.async_used * 2);
		if (vkGetQueryPoolResults(m_device, frame.async_pool, 0, frame.async_used * 2, stamps.size() * sizeof(std::uint64_t), stamps.data(), sizeof(std::uint64_t), VK_QUERY_RESULT_64_BIT) == VK_SUCCESS)
		{
			for (std::uint32_t i = 0; i < frame.async_used; ++i)
			{
				auto const ticks{((stamps[i * 2 + 1] & m_asyncmask) - (stamps[i * 2] & m_asyncmask)) & m_asyncmask};
				auto const name{frame.async_names[i] + " (async)"};
				auto lookup{std::find_if(m_asynctimes.begin(), m_asynctimes.end(), [&](auto const &Entry) { return Entry.first == name; })};
				if (lookup == m_asynctimes.end())
				{
					m_asynctimes.emplace_back(name, 0.0);
					lookup = std::prev(m_asynctimes.end());
				}
				blend(lookup->second, static_cast<double>(ticks) * m_period / 1000000.0);
			}
		}
	}
	std::vector<std::uint64_t> stamps(frame.used * 2);
	auto const result{vkGetQueryPoolResults(m_device, frame.pool, 0, frame.used * 2, stamps.size() * sizeof(std::uint64_t), stamps.data(), sizeof(std::uint64_t), VK_QUERY_RESULT_64_BIT)};
	if (result != VK_SUCCESS)
	{
		return;
	}
	double total{0.0};
	for (std::uint32_t i = 0; i < frame.used; ++i)
	{
		auto const ticks{((stamps[i * 2 + 1] & m_mask) - (stamps[i * 2] & m_mask)) & m_mask};
		auto const ms{static_cast<double>(ticks) * m_period / 1000000.0};
		total += ms;
		auto lookup{std::find_if(m_times.begin(), m_times.end(), [&](auto const &Entry) { return Entry.first == frame.names[i]; })};
		if (lookup == m_times.end())
		{
			m_times.emplace_back(frame.names[i], 0.0);
			lookup = std::prev(m_times.end());
		}
		blend(lookup->second, ms);
	}
	blend(m_framems, total);
}

std::string pass_list::passes_of(std::uint64_t Number) const
{
	std::string result;
	for (auto const &frame : m_frames)
	{
		if (frame.number != Number)
		{
			continue;
		}
		for (auto const &name : frame.names)
		{
			result += (result.empty() ? "" : ", ") + name;
		}
	}
	return (result.empty() ? std::string("(not known)") : result);
}

std::string pass_list::info() const
{
	std::string text;
	char buffer[64] = {};
	for (auto const &[name, ms] : m_times)
	{
		std::snprintf(buffer, sizeof(buffer), "  %-14s %7.3f ms\n", name.c_str(), ms);
		text += buffer;
	}
	std::snprintf(buffer, sizeof(buffer), "  %-14s %7.3f ms\n", "total", m_framems);
	text += buffer;
	for (auto const &[name, ms] : m_asynctimes)
	{
		std::snprintf(buffer, sizeof(buffer), "  %-14s %7.3f ms\n", name.c_str(), ms);
		text += buffer;
	}
	return text;
}

} // namespace vk14
