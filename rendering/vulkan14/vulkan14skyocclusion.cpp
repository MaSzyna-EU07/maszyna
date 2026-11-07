/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14skyocclusion.h"

#include "rendering/vulkan14/vulkan14commands.h"
#include "utilities/Logs.h"

namespace vk14
{

namespace
{

// the shading and the compute passes (the fog, the particles) read them
VkPipelineStageFlags2 const READ_STAGES{VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT | VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT | VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT};

} // namespace

sky_occlusion_maps::~sky_occlusion_maps()
{
	let_go();
}

void sky_occlusion_maps::init(allocator &Memory, deletion_queue &Deletions)
{
	m_memory = &Memory;
	m_deletions = &Deletions;
}

void sky_occlusion_maps::destroy()
{
	// the device is idle: the maps go at once
	m_deletions = nullptr;
	let_go();
	m_memory = nullptr;
}

bool sky_occlusion_maps::gpu_allocate(int const Side)
{
	let_go();
	if ((m_memory == nullptr) || (Side <= 0))
	{
		return false;
	}
	auto make = [&](std::shared_ptr<image> &Map, VkFormat const Format, char const *Name) {
		image_desc desc;
		desc.format = Format;
		desc.extent = {static_cast<std::uint32_t>(Side), static_cast<std::uint32_t>(Side), 1};
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
		Map = std::make_shared<image>();
		if (false == Map->create(*m_memory, desc, Name))
		{
			Map.reset();
			return false;
		}
		return true;
	};
	if ((false == make(m_heights, VK_FORMAT_R32G32B32A32_SFLOAT, "sky occlusion heights")) || (false == make(m_occlusion, VK_FORMAT_R16G16B16A16_SFLOAT, "sky occlusion")))
	{
		let_go();
		return false;
	}
	m_ready = false;
	return true;
}

void sky_occlusion_maps::gpu_release()
{
	let_go();
}

void sky_occlusion_maps::let_go()
{
	m_ready = false;
	m_squares.clear();
	m_data.clear();
	if ((m_deletions != nullptr) && (m_heights || m_occlusion))
	{
		// the frames in flight may still read them
		m_deletions->push(m_value, [heights = std::move(m_heights), occlusion = std::move(m_occlusion)]() {});
	}
	m_heights.reset();
	m_occlusion.reset();
}

void sky_occlusion_maps::gpu_upload(int const X, int const Y, int const Size, float const *Heights, std::uint16_t const *Occlusion)
{
	if (false == bool(m_heights))
	{
		return;
	}
	auto const texels{static_cast<std::size_t>(Size) * Size};
	auto const offset{m_data.size()};
	m_data.resize(offset + texels * (4 * sizeof(float) + 4 * sizeof(std::uint16_t)));
	std::memcpy(m_data.data() + offset, Heights, texels * 4 * sizeof(float));
	std::memcpy(m_data.data() + offset + texels * 4 * sizeof(float), Occlusion, texels * 4 * sizeof(std::uint16_t));
	m_squares.push_back(square{X, Y, Size, offset});
}

void sky_occlusion_maps::flush(command_stream *Commands)
{
	if ((Commands == nullptr) || (false == bool(m_heights)) || m_squares.empty())
	{
		return;
	}
	auto staging{std::make_shared<buffer>()};
	if ((false == staging->create(*m_memory, static_cast<VkDeviceSize>(m_data.size()), VK_BUFFER_USAGE_TRANSFER_SRC_BIT, memory_kind::upload, "sky occlusion upload")) || (staging->mapped() == nullptr))
	{
		ErrorLog("sky occlusion: no staging buffer for the maps");
		return;
	}
	std::memcpy(staging->mapped(), m_data.data(), m_data.size());
	staging->flush(0, static_cast<VkDeviceSize>(m_data.size()));
	transition(Commands, *m_heights, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
	transition(Commands, *m_occlusion, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
	std::vector<VkBufferImageCopy2> heights, occlusion;
	for (auto const &entry : m_squares)
	{
		VkBufferImageCopy2 region{};
		region.sType = VK_STRUCTURE_TYPE_BUFFER_IMAGE_COPY_2;
		region.bufferOffset = static_cast<VkDeviceSize>(entry.offset);
		region.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
		region.imageOffset = {entry.x, entry.y, 0};
		region.imageExtent = {static_cast<std::uint32_t>(entry.size), static_cast<std::uint32_t>(entry.size), 1};
		heights.push_back(region);
		region.bufferOffset += static_cast<VkDeviceSize>(entry.size) * entry.size * 4 * sizeof(float);
		occlusion.push_back(region);
	}
	auto copy = [&](image &Target, std::vector<VkBufferImageCopy2> const &Regions) {
		VkCopyBufferToImageInfo2 info{};
		info.sType = VK_STRUCTURE_TYPE_COPY_BUFFER_TO_IMAGE_INFO_2;
		info.srcBuffer = staging->handle();
		info.dstImage = Target.handle();
		info.dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		info.regionCount = static_cast<std::uint32_t>(Regions.size());
		info.pRegions = Regions.data();
		Commands->copy_buffer_to_image2(&info);
	};
	copy(*m_heights, heights);
	copy(*m_occlusion, occlusion);
	transition(Commands, *m_heights, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, READ_STAGES, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
	transition(Commands, *m_occlusion, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, READ_STAGES, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
	if (m_deletions != nullptr)
	{
		m_deletions->push(m_value, [staging]() {});
	}
	m_squares.clear();
	m_data.clear();
	m_ready = true;
}

} // namespace vk14
