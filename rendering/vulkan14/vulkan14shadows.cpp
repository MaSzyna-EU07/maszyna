/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14shadows.h"
#include "rendering/vulkan14/vulkan14commands.h"

namespace vk14
{

local_shadow_maps::~local_shadow_maps()
{
	destroy();
}

void local_shadow_maps::init(VkDevice const Device, allocator &Memory, deletion_queue &Deletions)
{
	m_device = Device;
	m_memory = &Memory;
	m_deletions = &Deletions;
}

void local_shadow_maps::destroy()
{
	// the device is idle: the maps go at once
	m_deletions = nullptr;
	let_go(m_cab, m_cabviews);
	let_go(m_lamps, m_lampviews);
	let_go(m_headlights, m_headlightviews);
}

bool local_shadow_maps::make(std::shared_ptr<image> &Map, std::vector<VkImageView> &Views, int const Size, int const Layers, bool const Cube, char const *Name)
{
	if ((m_memory == nullptr) || (Size <= 0) || (Layers <= 0))
	{
		return false;
	}
	image_desc desc;
	desc.format = VK_FORMAT_D32_SFLOAT;
	desc.extent = {static_cast<std::uint32_t>(Size), static_cast<std::uint32_t>(Size), 1};
	desc.layers = static_cast<std::uint32_t>(Layers);
	desc.view_type = (Cube ? VK_IMAGE_VIEW_TYPE_CUBE_ARRAY : VK_IMAGE_VIEW_TYPE_2D_ARRAY);
	desc.flags = (Cube ? VK_IMAGE_CREATE_CUBE_COMPATIBLE_BIT : 0);
	desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	Map = std::make_shared<image>();
	if (false == Map->create(*m_memory, desc, Name))
	{
		Map.reset();
		return false;
	}
	// the cubes drawn with multiview, six layers each, or the layers drawn one by one
	auto const step{Cube ? 6 : 1};
	for (int first = 0; first < Layers; first += step)
	{
		VkImageViewCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		info.image = Map->handle();
		info.viewType = (Cube ? VK_IMAGE_VIEW_TYPE_2D_ARRAY : VK_IMAGE_VIEW_TYPE_2D);
		info.format = desc.format;
		info.subresourceRange = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, static_cast<std::uint32_t>(first), static_cast<std::uint32_t>(step)};
		VkImageView view{VK_NULL_HANDLE};
		if (vkCreateImageView(m_device, &info, nullptr, &view) != VK_SUCCESS)
		{
			let_go(Map, Views);
			return false;
		}
		Views.push_back(view);
	}
	return true;
}

void local_shadow_maps::let_go(std::shared_ptr<image> &Map, std::vector<VkImageView> &Views)
{
	if (m_deletions != nullptr)
	{
		// the frames in flight may still read them
		m_deletions->push(m_value, [device = m_device, views = std::move(Views), map = std::move(Map)]() {
			for (auto *view : views)
			{
				vkDestroyImageView(device, view, nullptr);
			}
		});
	}
	else
	{
		for (auto *view : Views)
		{
			vkDestroyImageView(m_device, view, nullptr);
		}
	}
	Views.clear();
	Map.reset();
}

bool local_shadow_maps::gpu_allocate(int const Size)
{
	return make(m_cab, m_cabviews, Size, 6 * (cab_slot + 1), true, "lamp shadow map");
}

void local_shadow_maps::gpu_release()
{
	let_go(m_cab, m_cabviews);
}

bool local_shadow_maps::gpu_allocate_lamps(int const Count, int const Size)
{
	// the scenery parts and the working copies
	return make(m_lamps, m_lampviews, Size, 6 * 2 * Count, true, "scenery lamp shadow maps");
}

void local_shadow_maps::gpu_release_lamps()
{
	let_go(m_lamps, m_lampviews);
}

bool local_shadow_maps::gpu_allocate_headlights(int const Count, int const Size)
{
	return make(m_headlights, m_headlightviews, Size, Count, false, "headlight shadow maps");
}

void local_shadow_maps::gpu_release_headlights()
{
	let_go(m_headlights, m_headlightviews);
}

void local_shadow_maps::gpu_begin_cab()
{
	m_target = draw_target{m_cab.get(), m_cabviews.empty() ? VK_NULL_HANDLE : m_cabviews[cab_slot], static_cast<std::uint32_t>(m_size), 0x3f, true, true};
}

void local_shadow_maps::gpu_begin_lamp(int const Scenerycube, int const Workingcube, bool const Vehicles)
{
	auto const cube{Vehicles ? Workingcube : Scenerycube};
	m_target = draw_target{m_lamps.get(), (static_cast<std::size_t>(cube) < m_lampviews.size()) ? m_lampviews[cube] : VK_NULL_HANDLE, static_cast<std::uint32_t>(m_lampssize), 0x3f, (false == Vehicles), true};
	if (Vehicles && m_lamps && (m_commands != nullptr))
	{
		// the vehicles go over the scenery: the copy of the scenery part
		transition(m_commands, *m_lamps, VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_TRANSFER_READ_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT);
		VkImageCopy2 region{};
		region.sType = VK_STRUCTURE_TYPE_IMAGE_COPY_2;
		region.srcSubresource = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, static_cast<std::uint32_t>(Scenerycube * 6), 6};
		region.dstSubresource = {VK_IMAGE_ASPECT_DEPTH_BIT, 0, static_cast<std::uint32_t>(Workingcube * 6), 6};
		region.extent = {static_cast<std::uint32_t>(m_lampssize), static_cast<std::uint32_t>(m_lampssize), 1};
		VkCopyImageInfo2 copy{};
		copy.sType = VK_STRUCTURE_TYPE_COPY_IMAGE_INFO_2;
		copy.srcImage = m_lamps->handle();
		copy.srcImageLayout = VK_IMAGE_LAYOUT_GENERAL;
		copy.dstImage = m_lamps->handle();
		copy.dstImageLayout = VK_IMAGE_LAYOUT_GENERAL;
		copy.regionCount = 1;
		copy.pRegions = &region;
		m_commands->copy_image2(&copy);
	}
}

void local_shadow_maps::gpu_begin_headlight(std::size_t const Slot)
{
	m_target = draw_target{m_headlights.get(), (Slot < m_headlightviews.size()) ? m_headlightviews[Slot] : VK_NULL_HANDLE, static_cast<std::uint32_t>(m_headlightssize), 0, true, false};
}

} // namespace vk14
