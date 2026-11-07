/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14terrain.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "utilities/Logs.h"

namespace vk14
{

namespace
{

std::uint32_t mip_count(std::uint32_t const Size)
{
	return static_cast<std::uint32_t>(std::floor(std::log2(std::max(Size, 1u)))) + 1;
}

} // namespace

terrain_materials::~terrain_materials()
{
	destroy();
}

void terrain_materials::init(resources const &Resources, copy_draw Draw)
{
	m_resources = &Resources;
	m_draw = std::move(Draw);
}

void terrain_materials::destroy()
{
	if (m_resources == nullptr)
	{
		return;
	}
	auto const device{m_resources->gpu->handle()};
	for (auto &images : m_images)
	{
		for (auto *view : images.layers)
		{
			vkDestroyImageView(device, view, nullptr);
		}
		images = array_images{};
	}
	m_resources = nullptr;
}

bool terrain_materials::gpu_init()
{
	return (m_resources != nullptr) && bool(m_draw);
}

vulkan14_texture &terrain_materials::texture(texture_handle const Handle)
{
	return static_cast<vulkan14_texture &>(m_textures.texture(Handle));
}

std::shared_ptr<image> terrain_materials::make_image(VkFormat const Format, std::uint32_t const Width, std::uint32_t const Height, std::uint32_t const Mips, char const *Name)
{
	image_desc desc;
	desc.format = Format;
	desc.extent = {Width, Height, 1};
	desc.mips = Mips;
	desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	auto result{std::make_shared<image>()};
	if (false == result->create(*m_resources->memory, desc, Name))
	{
		ErrorLog(std::string("heightmap terrain: no gpu image for ") + Name);
		return nullptr;
	}
	return result;
}

void terrain_materials::upload(texture_handle const Texture, VkFormat const Format, std::uint32_t const Width, std::uint32_t const Height, void const *Data, VkDeviceSize const Size, bool const Mips, VkSampler const Sampler)
{
	auto &target{texture(Texture)};
	auto map{target.adopted()};
	auto const generate{Mips && m_resources->uploads->can_blit(Format)};
	auto const mips{generate ? mip_count(std::max(Width, Height)) : 1u};
	if ((map == nullptr) || (map->desc().format != Format) || (map->desc().extent.width != Width) || (map->desc().extent.height != Height) || (map->desc().mips != mips))
	{
		map = make_image(Format, Width, Height, mips, target.name.c_str());
		if (map == nullptr)
		{
			return;
		}
	}
	// the frames sent before are done with the content the upload replaces before it's written (the barrier of the upload)
	if (m_resources->uploads->upload(map, Data, Size, {{0, {Width, Height, 1}}}, generate))
	{
		target.adopt(map, Sampler);
	}
}

void terrain_materials::gpu_white(texture_source &Texture)
{
	std::uint8_t const texel[4]{255, 255, 255, 255};
	auto &target{static_cast<vulkan14_texture &>(Texture)};
	auto map{make_image(VK_FORMAT_R8G8B8A8_UNORM, 1, 1, 1, target.name.c_str())};
	if ((map != nullptr) && m_resources->uploads->upload(map, texel, sizeof(texel), {{0, {1, 1, 1}}}, false))
	{
		target.adopt(map, m_resources->sampling->repeat_nearest);
	}
}

void terrain_materials::gpu_storage(texture_array const &Array, int const Layers, bool const Fresh)
{
	auto const index{static_cast<std::size_t>(&Array - arrays().data())};
	auto &images{m_images[index]};
	let_go(images);
	image_desc desc;
	desc.format = (Array.srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM);
	desc.extent = {static_cast<std::uint32_t>(Array.size), static_cast<std::uint32_t>(Array.size), 1};
	desc.mips = mip_count(static_cast<std::uint32_t>(Array.size));
	desc.layers = static_cast<std::uint32_t>(Layers);
	desc.view_type = VK_IMAGE_VIEW_TYPE_2D_ARRAY;
	desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	auto &target{texture(Array.handle)};
	auto map{std::make_shared<image>()};
	if (false == map->create(*m_resources->memory, desc, target.name))
	{
		ErrorLog("heightmap terrain: no gpu image for the layers of \"" + target.name + "\"");
		return;
	}
	auto const device{m_resources->gpu->handle()};
	for (int layer = 0; layer < Layers; ++layer)
	{
		VkImageViewCreateInfo info{};
		info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		info.image = map->handle();
		info.viewType = VK_IMAGE_VIEW_TYPE_2D;
		info.format = desc.format;
		info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, static_cast<std::uint32_t>(layer), 1};
		VkImageView view{VK_NULL_HANDLE};
		if (vkCreateImageView(device, &info, nullptr, &view) != VK_SUCCESS)
		{
			ErrorLog("heightmap terrain: no view of a layer of \"" + target.name + "\"");
			images.layers.clear();
			break;
		}
		images.layers.push_back(view);
	}
	images.map = map;
	// cleared by the next frame, before anything reads it
	images.clear = true;
	target.adopt(map, m_resources->sampling->repeat_aniso);
}

void terrain_materials::let_go(array_images &Images)
{
	if (Images.layers.empty() && (Images.map == nullptr))
	{
		return;
	}
	// the frames in flight may still read them
	m_resources->deletions->push(m_resources->uploads->current_value(), [device = m_resources->gpu->handle(), views = std::move(Images.layers), map = std::move(Images.map)]() {
		for (auto *view : views)
		{
			vkDestroyImageView(device, view, nullptr);
		}
	});
	Images = array_images{};
}

void terrain_materials::gpu_blank(slot const &Slot)
{
	std::uint8_t const blank[4]{0, 0, 0, 0};
	for (auto const handle : Slot.weights)
	{
		upload(handle, VK_FORMAT_R8G8B8A8_UNORM, 1, 1, blank, sizeof(blank), true, m_resources->sampling->clamp_linear);
	}
	std::array<float, max_layers * 2 * 4> const zeros{};
	upload(Slot.table, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<std::uint32_t>(max_layers), 2, zeros.data(), sizeof(zeros), false, m_resources->sampling->clamp_nearest);
	// led between the samples, so the edges of the holes run smoothly across the cells
	std::uint8_t const none{0};
	upload(Slot.holes, VK_FORMAT_R8_UNORM, 1, 1, &none, 1, false, m_resources->sampling->clamp_linear);
}

void terrain_materials::gpu_weights(texture_handle const Texture, int const Side, std::uint8_t const *Texels)
{
	auto const side{static_cast<std::uint32_t>(Side)};
	upload(Texture, VK_FORMAT_R8G8B8A8_UNORM, side, side, Texels, static_cast<VkDeviceSize>(side) * side * 4, true, m_resources->sampling->clamp_linear);
}

void terrain_materials::gpu_table(texture_handle const Texture, float const *Table)
{
	upload(Texture, VK_FORMAT_R32G32B32A32_SFLOAT, static_cast<std::uint32_t>(max_layers), 2, Table, max_layers * 2 * 4 * sizeof(float), false, m_resources->sampling->clamp_nearest);
}

void terrain_materials::gpu_holes(texture_handle const Texture, int const Side, std::uint8_t const *Samples)
{
	auto const side{static_cast<std::uint32_t>(Side)};
	upload(Texture, VK_FORMAT_R8_UNORM, side, side, Samples, static_cast<VkDeviceSize>(side) * side, false, m_resources->sampling->clamp_linear);
}

void terrain_materials::begin_frame(command_stream *const Commands, std::uint64_t const Value)
{
	m_commands = Commands;
	m_value = Value;
	for (auto &images : m_images)
	{
		if ((false == images.clear) || (images.map == nullptr))
		{
			continue;
		}
		images.clear = false;
		// the layers not copied yet read black, as the storage of opengl starts out on most drivers
		transition(Commands, *images.map, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_CLEAR_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
		VkClearColorValue const zero{};
		VkImageSubresourceRange const range{VK_IMAGE_ASPECT_COLOR_BIT, 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS};
		Commands->clear_color_image(images.map->handle(), VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &zero, 1, &range);
		transition(Commands, *images.map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
	}
}

void terrain_materials::gpu_begin_copies()
{
	for (auto &images : m_images)
	{
		images.drawn = false;
	}
}

void terrain_materials::gpu_copy(texture_array const &Array, int const Layer, texture_source *Source, std::array<float, 4> const &Default)
{
	auto const index{static_cast<std::size_t>(&Array - arrays().data())};
	auto &images{m_images[index]};
	if ((m_commands == nullptr) || (images.map == nullptr) || (static_cast<std::size_t>(Layer) >= images.layers.size()))
	{
		return;
	}
	if (false == images.drawn)
	{
		images.drawn = true;
		transition(m_commands, *images.map, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT);
	}
	auto const view{images.layers[Layer]};
	auto const format{images.map->desc().format};
	VkExtent2D const extent{images.map->desc().extent.width, images.map->desc().extent.height};
	if (Source != nullptr)
	{
		auto const *entry{m_resources->images->find(static_cast<vulkan14_texture &>(*Source).get_id())};
		if (entry != nullptr)
		{
			m_draw(m_commands, view, format, extent, entry->target->view(), entry->sampler);
			return;
		}
	}
	// a map the layer doesn't have; one which failed reads as the texture 0 of opengl
	VkRenderingAttachmentInfo attachment{};
	attachment.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
	attachment.imageView = view;
	attachment.imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
	attachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	attachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	if (Source == nullptr)
	{
		std::copy(Default.begin(), Default.end(), attachment.clearValue.color.float32);
	}
	else
	{
		attachment.clearValue.color = VkClearColorValue{{0.f, 0.f, 0.f, 1.f}};
	}
	VkRenderingInfo rendering{};
	rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
	rendering.renderArea = {{0, 0}, extent};
	rendering.layerCount = 1;
	rendering.colorAttachmentCount = 1;
	rendering.pColorAttachments = &attachment;
	m_commands->begin_rendering(&rendering);
	m_commands->end_rendering();
}

void terrain_materials::gpu_end_copies(bool const Copied)
{
	for (auto &images : m_images)
	{
		if ((false == images.drawn) || (images.map == nullptr))
		{
			continue;
		}
		images.drawn = false;
		if (Copied && (images.map->desc().mips > 1) && m_resources->uploads->can_blit(images.map->desc().format))
		{
			make_mipmaps(*images.map);
		}
		else
		{
			transition(m_commands, *images.map, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
		}
	}
}

// the other levels of every layer made by blits, each from the one before it (glGenerateMipmap of the opengl renderer)
void terrain_materials::make_mipmaps(image &Map)
{
	auto const commands{m_commands};
	auto const layers{Map.desc().layers};
	auto const mips{Map.desc().mips};
	auto const extent{Map.desc().extent};
	auto barrier = [&](std::uint32_t Level, std::uint32_t Count, VkImageLayout From, VkImageLayout To, VkPipelineStageFlags2 SrcStage, VkAccessFlags2 SrcAccess, VkPipelineStageFlags2 DstStage, VkAccessFlags2 DstAccess) {
		VkImageMemoryBarrier2 entry{};
		entry.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
		entry.srcStageMask = SrcStage;
		entry.srcAccessMask = SrcAccess;
		entry.dstStageMask = DstStage;
		entry.dstAccessMask = DstAccess;
		entry.oldLayout = From;
		entry.newLayout = To;
		entry.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		entry.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
		entry.image = Map.handle();
		entry.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, Level, Count, 0, layers};
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.imageMemoryBarrierCount = 1;
		dependency.pImageMemoryBarriers = &entry;
		commands->pipeline_barrier2(&dependency);
	};
	// the first level drawn, the others written by the blits
	barrier(0, 1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
	barrier(1, mips - 1, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT, VK_ACCESS_2_NONE, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);
	for (std::uint32_t level = 1; level < mips; ++level)
	{
		if (level > 1)
		{
			barrier(level - 1, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_READ_BIT);
		}
		VkImageBlit2 blit{};
		blit.sType = VK_STRUCTURE_TYPE_IMAGE_BLIT_2;
		blit.srcSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level - 1, 0, layers};
		blit.srcOffsets[1] = {static_cast<std::int32_t>(std::max(extent.width >> (level - 1), 1u)), static_cast<std::int32_t>(std::max(extent.height >> (level - 1), 1u)), 1};
		blit.dstSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, level, 0, layers};
		blit.dstOffsets[1] = {static_cast<std::int32_t>(std::max(extent.width >> level, 1u)), static_cast<std::int32_t>(std::max(extent.height >> level, 1u)), 1};
		VkBlitImageInfo2 info{};
		info.sType = VK_STRUCTURE_TYPE_BLIT_IMAGE_INFO_2;
		info.srcImage = Map.handle();
		info.srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
		info.dstImage = Map.handle();
		info.dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
		info.regionCount = 1;
		info.pRegions = &blit;
		info.filter = VK_FILTER_LINEAR;
		commands->blit_image2(&info);
	}
	barrier(0, mips - 1, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_NONE, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
	barrier(mips - 1, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
	Map.state = image_state{VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
}

} // namespace vk14
