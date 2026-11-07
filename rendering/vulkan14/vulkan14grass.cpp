/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14grass.h"
#include "utilities/Logs.h"

#include <cstring>

namespace vk14
{

grass_field::~grass_field()
{
	destroy();
}

void grass_field::init(resources const &Resources)
{
	m_resources = &Resources;
}

void grass_field::destroy()
{
	if (m_resources == nullptr)
	{
		return;
	}
	// the images go with the chunks; the device is idle
	clear();
	m_chunks.clear();
	m_resources = nullptr;
}

std::shared_ptr<image> grass_field::make_image(VkFormat const Format, std::uint32_t const Width, std::uint32_t const Height, void const *Data, VkDeviceSize const Size, char const *Name)
{
	image_desc desc;
	desc.format = Format;
	desc.extent = {Width, Height, 1};
	desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	auto result{std::make_shared<image>()};
	if (false == result->create(*m_resources->memory, desc, Name))
	{
		ErrorLog(std::string("heightmap terrain: no gpu image for the ") + Name);
		return nullptr;
	}
	if (false == m_resources->uploads->upload(result, Data, Size, {{0, {Width, Height, 1}}}, false))
	{
		return nullptr;
	}
	return result;
}

void grass_field::gpu_upload(std::uint64_t const Id, gfx::grass_chunk const &Source)
{
	if (m_resources == nullptr)
	{
		return;
	}
	auto &chunk{m_chunks[Id]};
	auto const side{static_cast<std::uint32_t>(Source.cells + 1)};
	chunk.images[0] = make_image(VK_FORMAT_R32_SFLOAT, side, side, Source.heights, static_cast<VkDeviceSize>(side) * side * sizeof(float), "grass heights");
	chunk.samplers[0] = m_resources->sampling->clamp_nearest;

	// the densities of the kinds side by side, as the channels of a texel
	auto const samples{static_cast<std::uint32_t>(Source.samples + 1)};
	auto const plane{static_cast<std::size_t>(samples) * samples};
	std::size_t kinds{0};
	while (kinds < Source.kinds.size() && Source.kinds[kinds] >= 0)
		++kinds;
	std::vector<std::uint8_t> texels(plane * 4, 0);
	for (std::size_t kind = 0; kind < kinds; ++kind)
		for (std::size_t i = 0; i < plane; ++i)
			texels[i * 4 + kind] = Source.density[kind * plane + i];
	chunk.images[1] = make_image(VK_FORMAT_R8G8B8A8_UNORM, samples, samples, texels.data(), texels.size(), "grass densities");
	chunk.samplers[1] = m_resources->sampling->clamp_linear;

	std::uint8_t const none{0};
	auto const holeside{static_cast<std::uint32_t>((Source.holes > 0 && Source.hole_mask != nullptr) ? Source.holes + 1 : 1)};
	chunk.images[2] = make_image(VK_FORMAT_R8_UNORM, holeside, holeside, holeside > 1 ? Source.hole_mask : &none, static_cast<VkDeviceSize>(holeside) * holeside, "grass holes");
	chunk.samplers[2] = m_resources->sampling->clamp_linear;
	// the table is made when the chunk is drawn
	chunk.images[3].reset();
	chunk.samplers[3] = m_resources->sampling->clamp_nearest;
	chunk.table_made = false;
}

void grass_field::gpu_release(std::uint64_t const Id)
{
	auto const lookup{m_chunks.find(Id)};
	if (lookup == m_chunks.end())
	{
		return;
	}
	if (m_resources != nullptr)
	{
		// the frames in flight may still read them
		m_resources->deletions->push(m_resources->uploads->current_value(), [images = std::move(lookup->second.images)]() {});
	}
	m_chunks.erase(lookup);
}

grass_field::chunk_images const *grass_field::images(chunk_draw const &Draw)
{
	auto const lookup{m_chunks.find(Draw.id)};
	if ((lookup == m_chunks.end()) || (m_resources == nullptr))
	{
		return nullptr;
	}
	auto &chunk{lookup->second};
	for (std::size_t i = 0; i < 3; ++i)
	{
		if (chunk.images[i] == nullptr)
		{
			return nullptr;
		}
	}
	// the kinds of the chunk, by columns: base, tip, shape, dry; the share and the chunk in the last row
	std::array<glm::vec4, 20> table{};
	for (std::size_t i = 0; i < 4; ++i)
	{
		table[0 * 4 + i] = Draw.base[i];
		table[1 * 4 + i] = Draw.tip[i];
		table[2 * 4 + i] = Draw.shape[i];
		table[3 * 4 + i] = Draw.dry[i];
	}
	table[4 * 4 + 0] = Draw.share;
	table[4 * 4 + 1] = Draw.chunk;
	if ((false == chunk.table_made) || (std::memcmp(table.data(), chunk.table.data(), sizeof(table)) != 0) || (chunk.images[3] == nullptr))
	{
		auto map{chunk.images[3]};
		if (map == nullptr)
		{
			map = make_image(VK_FORMAT_R32G32B32A32_SFLOAT, 4, 5, table.data(), sizeof(table), "grass kinds");
		}
		else if (false == m_resources->uploads->upload(map, table.data(), sizeof(table), {{0, {4, 5, 1}}}, false))
		{
			map.reset();
		}
		if (map == nullptr)
		{
			return nullptr;
		}
		chunk.images[3] = map;
		chunk.table = table;
		chunk.table_made = true;
	}
	return &chunk;
}

} // namespace vk14
