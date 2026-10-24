/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14texture.h"
#include "rendering/vulkan14/vulkan14device.h"
#include "utilities/Globals.h"
#include "utilities/Logs.h"

#include <cstring>

namespace vk14
{

// uploader

void uploader::init(allocator const &Allocator, deletion_queue &Deletions, VkPhysicalDevice Physical)
{
	m_allocator = &Allocator;
	m_deletions = &Deletions;
	m_physical = Physical;
}

bool uploader::can_blit(VkFormat Format) const
{
	auto const lookup{m_blittable.find(Format)};
	if (lookup != m_blittable.end())
	{
		return lookup->second;
	}
	VkFormatProperties properties{};
	vkGetPhysicalDeviceFormatProperties(m_physical, Format, &properties);
	auto const needed{VK_FORMAT_FEATURE_BLIT_SRC_BIT | VK_FORMAT_FEATURE_BLIT_DST_BIT | VK_FORMAT_FEATURE_SAMPLED_IMAGE_FILTER_LINEAR_BIT};
	auto const result{(properties.optimalTilingFeatures & needed) == needed};
	m_blittable.emplace(Format, result);
	return result;
}

void uploader::begin_frame(VkCommandBuffer Commands, std::uint64_t Value)
{
	m_commands = Commands;
	m_value = Value;
	m_recorded = false;
	auto pending{std::move(m_pending)};
	m_pending.clear();
	for (auto &job : pending)
	{
		record(job);
	}
}

bool uploader::end_frame()
{
	if (m_buffercopies && (m_commands != VK_NULL_HANDLE))
	{
		// the draws of the frame read what was copied
		VkMemoryBarrier2 barrier{};
		barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
		barrier.srcStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
		barrier.srcAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
		barrier.dstStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
		barrier.dstAccessMask = VK_ACCESS_2_MEMORY_READ_BIT;
		VkDependencyInfo dependency{};
		dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
		dependency.memoryBarrierCount = 1;
		dependency.pMemoryBarriers = &barrier;
		vkCmdPipelineBarrier2(m_commands, &dependency);
	}
	m_buffercopies = false;
	m_commands = VK_NULL_HANDLE;
	return m_recorded;
}

bool uploader::upload_buffer(std::shared_ptr<buffer> const &Target, void const *Data, VkDeviceSize Size, std::vector<std::pair<VkDeviceSize, VkDeviceSize>> const &Parts)
{
	if ((Target == nullptr) || (Size == 0))
	{
		return true;
	}
	job upload;
	upload.buffertarget = Target;
	upload.parts = Parts;
	if (upload.parts.empty())
	{
		upload.parts.emplace_back(0, Size);
	}
	upload.staging = std::make_shared<buffer>();
	if ((false == upload.staging->create(*m_allocator, Size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, memory_kind::upload)) || (upload.staging->mapped() == nullptr))
	{
		log_error("no staging buffer of " + memory_size_string(Size) + " for an upload");
		return false;
	}
	std::memcpy(upload.staging->mapped(), Data, static_cast<std::size_t>(Size));
	upload.staging->flush(0, Size);
	m_uploaded += Size;
	if (m_commands != VK_NULL_HANDLE)
	{
		record(upload);
	}
	else
	{
		m_pending.emplace_back(std::move(upload));
	}
	return true;
}

bool uploader::upload(std::shared_ptr<image> const &Image, void const *Data, VkDeviceSize Size, std::vector<level> const &Levels, bool const Mipmaps)
{
	job upload;
	upload.target = Image;
	upload.levels = Levels;
	upload.mipmaps = Mipmaps;
	upload.staging = std::make_shared<buffer>();
	if ((false == upload.staging->create(*m_allocator, Size, VK_BUFFER_USAGE_TRANSFER_SRC_BIT, memory_kind::upload)) || (upload.staging->mapped() == nullptr))
	{
		log_error("no staging buffer of " + memory_size_string(Size) + " for an upload");
		return false;
	}
	std::memcpy(upload.staging->mapped(), Data, static_cast<std::size_t>(Size));
	upload.staging->flush(0, Size);
	m_uploaded += Size;
	if (m_commands != VK_NULL_HANDLE)
	{
		record(upload);
	}
	else
	{
		m_pending.emplace_back(std::move(upload));
	}
	return true;
}

void uploader::record(job &Job)
{
	if (Job.buffertarget != nullptr)
	{
		if (false == m_buffercopies)
		{
			// the frames sent before may still read the buffers the copies write to
			VkMemoryBarrier2 barrier{};
			barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
			barrier.srcStageMask = VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT;
			barrier.srcAccessMask = VK_ACCESS_2_NONE;
			barrier.dstStageMask = VK_PIPELINE_STAGE_2_COPY_BIT;
			barrier.dstAccessMask = VK_ACCESS_2_TRANSFER_WRITE_BIT;
			VkDependencyInfo dependency{};
			dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
			dependency.memoryBarrierCount = 1;
			dependency.pMemoryBarriers = &barrier;
			vkCmdPipelineBarrier2(m_commands, &dependency);
			m_buffercopies = true;
		}
		std::vector<VkBufferCopy2> regions;
		VkDeviceSize source{0};
		for (auto const &part : Job.parts)
		{
			VkBufferCopy2 region{};
			region.sType = VK_STRUCTURE_TYPE_BUFFER_COPY_2;
			region.srcOffset = source;
			region.dstOffset = part.first;
			region.size = part.second;
			regions.push_back(region);
			source += part.second;
		}
		VkCopyBufferInfo2 copy{};
		copy.sType = VK_STRUCTURE_TYPE_COPY_BUFFER_INFO_2;
		copy.srcBuffer = Job.staging->handle();
		copy.dstBuffer = Job.buffertarget->handle();
		copy.regionCount = static_cast<std::uint32_t>(regions.size());
		copy.pRegions = regions.data();
		vkCmdCopyBuffer2(m_commands, &copy);
		m_recorded = true;
		m_deletions->push(m_value, [staging = Job.staging, target = Job.buffertarget]() {});
		return;
	}
	auto &target{*Job.target};
	auto const commands{m_commands};
	auto const aspect{target.aspect()};
	auto const layers{target.desc().layers};
	auto const mips{target.desc().mips};

	transition(commands, target, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT);

	std::vector<VkBufferImageCopy2> regions;
	for (std::size_t i = 0; i < Job.levels.size(); ++i)
	{
		VkBufferImageCopy2 region{};
		region.sType = VK_STRUCTURE_TYPE_BUFFER_IMAGE_COPY_2;
		region.bufferOffset = Job.levels[i].offset;
		region.imageSubresource = {aspect, static_cast<std::uint32_t>(i), 0, layers};
		region.imageExtent = Job.levels[i].extent;
		regions.push_back(region);
	}
	VkCopyBufferToImageInfo2 copy{};
	copy.sType = VK_STRUCTURE_TYPE_COPY_BUFFER_TO_IMAGE_INFO_2;
	copy.srcBuffer = Job.staging->handle();
	copy.dstImage = target.handle();
	copy.dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
	copy.regionCount = static_cast<std::uint32_t>(regions.size());
	copy.pRegions = regions.data();
	vkCmdCopyBufferToImage2(commands, &copy);

	auto const given{static_cast<std::uint32_t>(Job.levels.size())};
	if (Job.mipmaps && (mips > given) && (given > 0))
	{
		// the other levels made by blits, each from the one before it
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
			entry.image = target.handle();
			entry.subresourceRange = {aspect, Level, Count, 0, layers};
			VkDependencyInfo dependency{};
			dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
			dependency.imageMemoryBarrierCount = 1;
			dependency.pImageMemoryBarriers = &entry;
			vkCmdPipelineBarrier2(commands, &dependency);
		};
		auto const extent{target.desc().extent};
		for (std::uint32_t level = given; level < mips; ++level)
		{
			barrier(level - 1, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_PIPELINE_STAGE_2_COPY_BIT | VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_BLIT_BIT,
			        VK_ACCESS_2_TRANSFER_READ_BIT);
			VkImageBlit2 blit{};
			blit.sType = VK_STRUCTURE_TYPE_IMAGE_BLIT_2;
			blit.srcSubresource = {aspect, level - 1, 0, layers};
			blit.srcOffsets[1] = {static_cast<std::int32_t>(std::max(extent.width >> (level - 1), 1u)), static_cast<std::int32_t>(std::max(extent.height >> (level - 1), 1u)), 1};
			blit.dstSubresource = {aspect, level, 0, layers};
			blit.dstOffsets[1] = {static_cast<std::int32_t>(std::max(extent.width >> level, 1u)), static_cast<std::int32_t>(std::max(extent.height >> level, 1u)), 1};
			VkBlitImageInfo2 info{};
			info.sType = VK_STRUCTURE_TYPE_BLIT_IMAGE_INFO_2;
			info.srcImage = target.handle();
			info.srcImageLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
			info.dstImage = target.handle();
			info.dstImageLayout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
			info.regionCount = 1;
			info.pRegions = &blit;
			info.filter = VK_FILTER_LINEAR;
			vkCmdBlitImage2(commands, &info);
		}
		barrier(0, mips - 1, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_NONE, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
		barrier(mips - 1, 1, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_BLIT_BIT, VK_ACCESS_2_TRANSFER_WRITE_BIT, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT,
		        VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
		target.state = image_state{VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT};
	}
	else
	{
		transition(commands, target, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL, VK_PIPELINE_STAGE_2_ALL_COMMANDS_BIT, VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
	}
	m_recorded = true;
	// the staging buffer goes, and the image may, once the gpu is done with the frame
	m_deletions->push(m_value, [staging = Job.staging, image = Job.target]() {});
}

// image table

std::uint64_t image_table::add(std::shared_ptr<image> const &Image, VkSampler Sampler)
{
	std::uint64_t id;
	if (false == m_free.empty())
	{
		id = m_free.back();
		m_free.pop_back();
	}
	else
	{
		id = m_entries.size();
		m_entries.emplace_back();
	}
	m_entries[id] = entry{Image, Sampler};
	return id;
}

void image_table::replace(std::uint64_t Id, std::shared_ptr<image> const &Image, VkSampler Sampler)
{
	if ((Id > 0) && (Id < m_entries.size()))
	{
		m_entries[Id] = entry{Image, Sampler};
	}
}

void image_table::remove(std::uint64_t Id, deletion_queue &Deletions, std::uint64_t Value)
{
	if ((Id == 0) || (Id >= m_entries.size()) || (m_entries[Id].target == nullptr))
	{
		return;
	}
	Deletions.push(Value, [image = std::move(m_entries[Id].target)]() {});
	m_entries[Id] = entry{};
	m_free.push_back(Id);
}

image_table::entry const *image_table::find(std::uint64_t Id) const
{
	if ((Id == 0) || (Id >= m_entries.size()) || (m_entries[Id].target == nullptr))
	{
		return nullptr;
	}
	return &m_entries[Id];
}

void image_table::clear()
{
	m_entries.assign(1, entry{});
	m_free.clear();
}

// formats

texture_format texture_format_of(GLint Format, GLint Hint)
{
	// (etc2 of the ktx files isn't mapped: the desktop gpus don't sample it, and the feature isn't enabled on the device; the
	// opengl driver decodes it)
	// the material wants srgb colours (the default), or values: the compressed formats keep their alpha either way, as in the
	// opengl renderer (opengl_texture::mapping)
	auto const srgb{(Hint == 0) || (Hint == GL_SRGB_ALPHA) || (Hint == GL_SRGB)};
	texture_format result;
	switch (Format)
	{
	case GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT:
	case GL_COMPRESSED_RGBA_S3TC_DXT1_EXT:
		result.format = ((srgb || (Format == GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT1_EXT)) ? VK_FORMAT_BC1_RGBA_SRGB_BLOCK : VK_FORMAT_BC1_RGBA_UNORM_BLOCK);
		result.compressed = true;
		result.block_bytes = 8;
		return result;
	case GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT:
	case GL_COMPRESSED_RGBA_S3TC_DXT3_EXT:
		result.format = ((srgb || (Format == GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT3_EXT)) ? VK_FORMAT_BC2_SRGB_BLOCK : VK_FORMAT_BC2_UNORM_BLOCK);
		result.compressed = true;
		result.block_bytes = 16;
		return result;
	case GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT:
	case GL_COMPRESSED_RGBA_S3TC_DXT5_EXT:
		result.format = ((srgb || (Format == GL_COMPRESSED_SRGB_ALPHA_S3TC_DXT5_EXT)) ? VK_FORMAT_BC3_SRGB_BLOCK : VK_FORMAT_BC3_UNORM_BLOCK);
		result.compressed = true;
		result.block_bytes = 16;
		return result;
	case GL_RGBA:
	case GL_BGRA:
	case GL_RGB:
		result.format = (Format == GL_BGRA ? (srgb ? VK_FORMAT_B8G8R8A8_SRGB : VK_FORMAT_B8G8R8A8_UNORM) : (srgb ? VK_FORMAT_R8G8B8A8_SRGB : VK_FORMAT_R8G8B8A8_UNORM));
		// rgb texels aren't sampled by every gpu, they are made rgba
		result.expand_rgb = (Format == GL_RGB);
		break;
	default:
		return result;
	}
	// what the material takes of the colour, as the internal formats of opengl give it (GL_SRGB8, GL_RG8, GL_R8: the missing
	// channels read 0, the alpha 1)
	switch (Hint)
	{
	case GL_SRGB:
	case GL_RGB:
		result.components.a = VK_COMPONENT_SWIZZLE_ONE;
		break;
	case GL_RG:
		result.components.b = VK_COMPONENT_SWIZZLE_ZERO;
		result.components.a = VK_COMPONENT_SWIZZLE_ONE;
		break;
	case GL_RED:
		result.components.g = VK_COMPONENT_SWIZZLE_ZERO;
		result.components.b = VK_COMPONENT_SWIZZLE_ZERO;
		result.components.a = VK_COMPONENT_SWIZZLE_ONE;
		break;
	default:
		break;
	}
	return result;
}

// texture

vulkan14_texture::~vulkan14_texture()
{
	if (m_resources.pending != nullptr)
	{
		std::lock_guard<std::mutex> lock(m_resources.pending->mutex);
		auto &list{m_resources.pending->textures};
		list.erase(std::remove(list.begin(), list.end(), this), list.end());
	}
}

bool vulkan14_texture::create(bool const Static)
{
	if ((data_state == resource_state::none) && (true == reload_on_use))
	{
		// the data was dropped along with the gpu image when the texture was far away
		reload_on_use = false;
		load();
	}
	if ((data_state == resource_state::loading) && (request != nullptr))
	{
		complete_data();
	}
	if (m_made)
	{
		return true;
	}
	if (data_state != resource_state::good)
	{
		// don't bother until we have useful texture data
		return false;
	}

	auto const format{texture_format_of(data_format, components_hint != 0 ? components_hint : GL_SRGB_ALPHA)};
	VkFormatProperties properties{};
	if (format.format != VK_FORMAT_UNDEFINED)
	{
		vkGetPhysicalDeviceFormatProperties(m_resources.gpu->physical(), format.format, &properties);
	}
	if ((format.format == VK_FORMAT_UNDEFINED) || ((properties.optimalTilingFeatures & VK_FORMAT_FEATURE_SAMPLED_IMAGE_BIT) == 0))
	{
		ErrorLog("Bad texture: the format of \"" + name + "\" can't be sampled by the gpu", logtype::texture);
		on_failed();
		is_ready = true;
		return true;
	}

	std::vector<uploader::level> levels;
	std::vector<std::uint8_t> expanded;
	std::uint8_t const *source{data.data()};
	VkDeviceSize size{data.size()};
	auto width{data_width}, height{data_height};
	if (format.compressed)
	{
		VkDeviceSize offset{0};
		for (unsigned int level = 0; level < data_mapcount; ++level)
		{
			auto const levelsize{static_cast<VkDeviceSize>((std::max(width, 4u) + 3) / 4) * ((std::max(height, 4u) + 3) / 4) * format.block_bytes};
			if (offset + levelsize > data.size())
			{
				break;
			}
			levels.push_back({offset, {width, height, 1}});
			offset += levelsize;
			width = std::max(width / 2, 1u);
			height = std::max(height / 2, 1u);
		}
		size = offset;
	}
	else
	{
		if (format.expand_rgb)
		{
			auto const pixels{static_cast<std::size_t>(data_width) * data_height};
			expanded.resize(pixels * 4);
			for (std::size_t i = 0; i < pixels; ++i)
			{
				expanded[i * 4 + 0] = data[i * 3 + 0];
				expanded[i * 4 + 1] = data[i * 3 + 1];
				expanded[i * 4 + 2] = data[i * 3 + 2];
				expanded[i * 4 + 3] = 0xff;
			}
			source = expanded.data();
			size = expanded.size();
		}
		else
		{
			size = static_cast<VkDeviceSize>(data_width) * data_height * 4;
		}
		levels.push_back({0, {data_width, data_height, 1}});
	}
	auto const available{expanded.empty() ? data.size() : expanded.size()};
	if (levels.empty() || (size > available) || (data_width == 0) || (data_height == 0))
	{
		ErrorLog("Bad texture: \"" + name + "\" holds less data than its size needs", logtype::texture);
		on_failed();
		is_ready = true;
		return true;
	}

	// a single level is made into the whole chain on the gpu, as glGenerateMipmap() does for the opengl renderer
	auto const generate{(levels.size() == 1) && (false == format.compressed) && m_resources.uploads->can_blit(format.format)};
	image_desc desc;
	desc.format = format.format;
	desc.extent = {data_width, data_height, 1};
	desc.mips = (generate ? static_cast<std::uint32_t>(std::floor(std::log2(std::max(data_width, data_height)))) + 1 : static_cast<std::uint32_t>(levels.size()));
	desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | (generate ? VK_IMAGE_USAGE_TRANSFER_SRC_BIT : 0);
	desc.components = format.components;
	auto image{std::make_shared<vk14::image>()};
	if (false == image->create(*m_resources.memory, desc, name))
	{
		ErrorLog("Bad texture: no gpu image for \"" + name + "\"", logtype::texture);
		on_failed();
		is_ready = true;
		return true;
	}
	if (false == m_resources.uploads->upload(image, source, size, levels, generate))
	{
		// never filled, so never sampled
		ErrorLog("Bad texture: \"" + name + "\" couldn't be uploaded", logtype::texture);
		on_failed();
		is_ready = true;
		return true;
	}

	// the traits of the file: clamped along s or t
	auto const clamped{(traits.find('s') != std::string::npos) || (traits.find('t') != std::string::npos)};
	m_image = image;
	m_id = m_resources.images->add(image, clamped ? m_resources.sampling->clamp_aniso : m_resources.sampling->repeat_aniso);
	m_made = true;

	if ((true == Global.ResourceMove) || (false == Global.ResourceSweep) || (true == streamed()))
	{
		// if garbage collection is disabled we don't expect having to upload the texture more than once
		data = std::vector<unsigned char>();
		data_state = resource_state::none;
	}
	if (type == "make:")
	{
		// for generated textures send a request to have the actual content of the texture generated
		make_request();
	}
	is_static = Static;
	is_ready = true;
	return true;
}

void vulkan14_texture::release()
{
	if (false == m_made)
	{
		return;
	}
	if (is_static)
	{
		return;
	}
	if (true == streamed())
	{
		// no copy of the data is kept, it's read from the file again when the texture is needed (prefetch(), create())
		data = std::vector<unsigned char>();
		data_state = resource_state::none;
		reload_on_use = true;
	}
	else if (data_state != resource_state::good)
	{
		if (type == "make:")
		{
			// auto generated textures only store a stub
			make_stub();
		}
		else if (type != "internalsrc:")
		{
			// the data went after the upload (gfx.resource.move): read from the file again, instead of from the gpu
			reload_on_use = true;
		}
	}
	m_resources.images->remove(m_id, *m_resources.deletions, m_resources.uploads->current_value());
	m_image.reset();
	m_id = 0;
	m_made = false;
	is_ready = false;
}

void vulkan14_texture::destroy()
{
	if (m_id != 0)
	{
		m_resources.images->remove(m_id, *m_resources.deletions, m_resources.uploads->current_value());
	}
	m_image.reset();
	m_id = 0;
	m_made = false;
}

void vulkan14_texture::on_failed()
{
	// the texture counts as made, without an image (the id 0), as with the opengl renderer
	if (m_id != 0)
	{
		m_resources.images->remove(m_id, *m_resources.deletions, m_resources.uploads->current_value());
	}
	m_image.reset();
	m_id = 0;
	m_made = true;
}

void vulkan14_texture::update_from_memory(size_t width, size_t height, const uint8_t *raw)
{
	if (std::this_thread::get_id() != m_resources.thread)
	{
		// the renderer takes it over in its frame (apply_pending())
		std::lock_guard<std::mutex> lock(m_pendingmutex);
		m_pending.assign(raw, raw + width * height * 4);
		m_pendingwidth = width;
		m_pendingheight = height;
		if (m_resources.pending != nullptr)
		{
			std::lock_guard<std::mutex> listlock(m_resources.pending->mutex);
			auto &list{m_resources.pending->textures};
			if (std::find(list.begin(), list.end(), this) == list.end())
			{
				list.push_back(this);
			}
		}
		return;
	}
	upload_rgba(raw, width, height);
}

void vulkan14_texture::apply_pending()
{
	std::vector<std::uint8_t> pending;
	std::size_t width{0}, height{0};
	{
		std::lock_guard<std::mutex> lock(m_pendingmutex);
		pending.swap(m_pending);
		width = m_pendingwidth;
		height = m_pendingheight;
	}
	if (false == pending.empty())
	{
		upload_rgba(pending.data(), width, height);
	}
}

void vulkan14_texture::upload_rgba(std::uint8_t const *Data, std::size_t Width, std::size_t Height)
{
	// as the opengl renderer: srgb rgba with mipmaps, the image made anew when the size changes
	auto const width{static_cast<std::uint32_t>(Width)}, height{static_cast<std::uint32_t>(Height)};
	auto const format{VK_FORMAT_R8G8B8A8_SRGB};
	auto const generate{m_resources.uploads->can_blit(format)};
	auto target{m_image};
	auto const fresh{(m_image == nullptr) || (m_image->desc().extent.width != width) || (m_image->desc().extent.height != height) || (m_image->desc().format != format)};
	if (fresh)
	{
		image_desc desc;
		desc.format = format;
		desc.extent = {width, height, 1};
		desc.mips = (generate ? static_cast<std::uint32_t>(std::floor(std::log2(std::max(width, height)))) + 1 : 1);
		desc.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT | (generate ? VK_IMAGE_USAGE_TRANSFER_SRC_BIT : 0);
		target = std::make_shared<vk14::image>();
		if (false == target->create(*m_resources.memory, desc, name))
		{
			return;
		}
	}
	// a new image goes to the table only once it has its content
	if (false == m_resources.uploads->upload(target, Data, static_cast<VkDeviceSize>(width) * height * 4, {{0, {width, height, 1}}}, generate))
	{
		return;
	}
	if (fresh)
	{
		auto const sampler{m_resources.sampling->repeat_aniso};
		if (m_id != 0)
		{
			// the old image goes once the frames which drew it are done
			m_resources.deletions->push(m_resources.uploads->current_value(), [old = m_image]() {});
			m_resources.images->replace(m_id, target, sampler);
		}
		else
		{
			m_id = m_resources.images->add(target, sampler);
		}
		m_image = target;
	}
	data_width = width;
	data_height = height;
	data_format = GL_SRGB8_ALPHA8;
	data_components = GL_RGBA;
	m_made = true;
	// the content given straight to the gpu can't be read again: released by the garbage sweep, the texture would come back as
	// the grey stub
	if (type == "internalsrc:")
	{
		is_static = true;
		is_ready = true;
	}
}

} // namespace vk14
