/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14memory.h"
#include "rendering/vulkan14/vulkan14commands.h"
#include "rendering/vulkan14/vulkan14instance.h"
#include "rendering/vulkan14/vulkan14device.h"

namespace vk14
{

// allocator

allocator::~allocator()
{
	destroy();
}

bool allocator::create(instance const &Instance, device const &Device, std::string &Reason)
{
	VmaAllocatorCreateInfo info{};
	info.vulkanApiVersion = VK_API_VERSION_1_4;
	info.instance = Instance.handle();
	info.physicalDevice = Device.physical();
	info.device = Device.handle();
	info.flags = VMA_ALLOCATOR_CREATE_KHR_MAINTENANCE5_BIT;
	if (Device.memory_budget_enabled())
	{
		info.flags |= VMA_ALLOCATOR_CREATE_EXT_MEMORY_BUDGET_BIT;
	}
	if (Device.raytracing_enabled())
	{
		// the acceleration structures are built from the device addresses of the buffers
		info.flags |= VMA_ALLOCATOR_CREATE_BUFFER_DEVICE_ADDRESS_BIT;
	}
	// the functions as volk loaded them
	VmaVulkanFunctions functions{};
	if (!VK14_VERIFY(vmaImportVulkanFunctionsFromVolk(&info, &functions)))
	{
		Reason = "the functions of the memory allocator can't be loaded";
		return false;
	}
	info.pVulkanFunctions = &functions;
	if (!VK14_VERIFY(vmaCreateAllocator(&info, &m_allocator)))
	{
		m_allocator = VK_NULL_HANDLE;
		Reason = "the memory allocator can't be made";
		return false;
	}
	m_device = Device.handle();

	VkPhysicalDeviceMemoryProperties memory{};
	vkGetPhysicalDeviceMemoryProperties(Device.physical(), &memory);
	m_heapcount = memory.memoryHeapCount;
	m_deviceheaps.assign(m_heapcount, false);
	for (std::uint32_t i = 0; i < m_heapcount; ++i)
	{
		m_deviceheaps[i] = ((memory.memoryHeaps[i].flags & VK_MEMORY_HEAP_DEVICE_LOCAL_BIT) != 0);
	}
	return true;
}

void allocator::destroy()
{
	if (m_allocator != VK_NULL_HANDLE)
	{
		vmaDestroyAllocator(m_allocator);
		m_allocator = VK_NULL_HANDLE;
	}
	m_device = VK_NULL_HANDLE;
}

void allocator::budget(VkDeviceSize &Usage, VkDeviceSize &Budget) const
{
	Usage = 0;
	Budget = 0;
	if (m_allocator == VK_NULL_HANDLE)
	{
		return;
	}
	std::vector<VmaBudget> budgets(m_heapcount);
	vmaGetHeapBudgets(m_allocator, budgets.data());
	for (std::uint32_t i = 0; i < m_heapcount; ++i)
	{
		if (m_deviceheaps[i])
		{
			Usage += budgets[i].usage;
			Budget += budgets[i].budget;
		}
	}
}

// buffer

buffer::buffer(buffer &&Other) noexcept
{
	*this = std::move(Other);
}

buffer &buffer::operator=(buffer &&Other) noexcept
{
	if (this != &Other)
	{
		destroy();
		std::swap(m_allocator, Other.m_allocator);
		std::swap(m_buffer, Other.m_buffer);
		std::swap(m_allocation, Other.m_allocation);
		std::swap(m_size, Other.m_size);
		std::swap(m_mapped, Other.m_mapped);
		std::swap(m_address, Other.m_address);
	}
	return *this;
}

buffer::~buffer()
{
	destroy();
}

bool buffer::create(allocator const &Allocator, VkDeviceSize Size, VkBufferUsageFlags Usage, memory_kind Kind, std::string const &Name)
{
	destroy();

	VkBufferCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	info.size = std::max<VkDeviceSize>(Size, 4);
	info.usage = Usage;
	info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

	VmaAllocationCreateInfo allocation{};
	allocation.usage = VMA_MEMORY_USAGE_AUTO;
	switch (Kind)
	{
	case memory_kind::device:
		allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
		break;
	case memory_kind::upload:
		allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
		break;
	case memory_kind::readback:
		allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_RANDOM_BIT | VMA_ALLOCATION_CREATE_MAPPED_BIT;
		break;
	}

	VmaAllocationInfo allocated{};
	if (!VK14_VERIFY(vmaCreateBuffer(Allocator.handle(), &info, &allocation, &m_buffer, &m_allocation, &allocated)))
	{
		m_buffer = VK_NULL_HANDLE;
		m_allocation = VK_NULL_HANDLE;
		return false;
	}
	m_allocator = Allocator.handle();
	m_size = Size;
	m_mapped = static_cast<std::uint8_t *>(allocated.pMappedData);
	name_object(Allocator.device_handle(), VK_OBJECT_TYPE_BUFFER, handle_value(m_buffer), Name);
	return true;
}

void buffer::destroy()
{
	if (m_buffer != VK_NULL_HANDLE)
	{
		vmaDestroyBuffer(m_allocator, m_buffer, m_allocation);
	}
	m_buffer = VK_NULL_HANDLE;
	m_allocation = VK_NULL_HANDLE;
	m_size = 0;
	m_mapped = nullptr;
	m_address = 0;
}

VkDeviceAddress buffer::address() const
{
	if ((m_buffer == VK_NULL_HANDLE) || (m_address != 0))
	{
		return m_address;
	}
	VmaAllocatorInfo allocator{};
	vmaGetAllocatorInfo(m_allocator, &allocator);
	VkBufferDeviceAddressInfo info{};
	info.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
	info.buffer = m_buffer;
	m_address = vkGetBufferDeviceAddress(allocator.device, &info);
	return m_address;
}

void buffer::flush(VkDeviceSize Offset, VkDeviceSize Size) const
{
	if ((m_allocation != VK_NULL_HANDLE) && (Size > 0))
	{
		// vma does nothing for coherent memory
		vmaFlushAllocation(m_allocator, m_allocation, Offset, Size);
	}
}

void buffer::invalidate(VkDeviceSize Offset, VkDeviceSize Size) const
{
	if ((m_allocation != VK_NULL_HANDLE) && (Size > 0))
	{
		vmaInvalidateAllocation(m_allocator, m_allocation, Offset, Size);
	}
}

// image

image::~image()
{
	destroy();
}

VkImageAspectFlags image::aspect() const
{
	switch (m_desc.format)
	{
	case VK_FORMAT_D16_UNORM:
	case VK_FORMAT_D32_SFLOAT:
	case VK_FORMAT_X8_D24_UNORM_PACK32:
		return VK_IMAGE_ASPECT_DEPTH_BIT;
	case VK_FORMAT_D24_UNORM_S8_UINT:
	case VK_FORMAT_D32_SFLOAT_S8_UINT:
	case VK_FORMAT_D16_UNORM_S8_UINT:
		return VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
	default:
		break;
	}
	return VK_IMAGE_ASPECT_COLOR_BIT;
}

bool image::create(allocator const &Allocator, image_desc const &Desc, std::string const &Name)
{
	destroy();
	m_desc = Desc;

	VkImageCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	info.flags = Desc.flags;
	info.imageType = Desc.type;
	info.format = Desc.format;
	info.extent = Desc.extent;
	info.mipLevels = std::max(Desc.mips, 1u);
	info.arrayLayers = std::max(Desc.layers, 1u);
	info.samples = Desc.samples;
	info.tiling = VK_IMAGE_TILING_OPTIMAL;
	info.usage = Desc.usage;
	info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

	VmaAllocationCreateInfo allocation{};
	allocation.usage = VMA_MEMORY_USAGE_AUTO_PREFER_DEVICE;
	VmaAllocationInfo allocated{};
	if (!VK14_VERIFY(vmaCreateImage(Allocator.handle(), &info, &allocation, &m_image, &m_allocation, &allocated)))
	{
		m_image = VK_NULL_HANDLE;
		m_allocation = VK_NULL_HANDLE;
		return false;
	}
	m_allocator = Allocator.handle();
	m_device = Allocator.device_handle();
	m_bytes = allocated.size;

	VkImageViewCreateInfo viewinfo{};
	viewinfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	viewinfo.image = m_image;
	viewinfo.viewType = Desc.view_type;
	viewinfo.format = Desc.format;
	viewinfo.components = Desc.components;
	viewinfo.subresourceRange = {aspect(), 0, info.mipLevels, 0, info.arrayLayers};
	if (!VK14_VERIFY(vkCreateImageView(m_device, &viewinfo, nullptr, &m_view)))
	{
		m_view = VK_NULL_HANDLE;
		destroy();
		return false;
	}
	state = image_state{};
	name_object(m_device, VK_OBJECT_TYPE_IMAGE, handle_value(m_image), Name);
	name_object(m_device, VK_OBJECT_TYPE_IMAGE_VIEW, handle_value(m_view), Name);
	return true;
}

void image::destroy()
{
	if (m_view != VK_NULL_HANDLE)
	{
		vkDestroyImageView(m_device, m_view, nullptr);
	}
	if (m_image != VK_NULL_HANDLE)
	{
		vmaDestroyImage(m_allocator, m_image, m_allocation);
	}
	m_view = VK_NULL_HANDLE;
	m_image = VK_NULL_HANDLE;
	m_allocation = VK_NULL_HANDLE;
	m_bytes = 0;
	state = image_state{};
}

void transition(command_stream *Commands, image &Image, VkImageLayout Layout, VkPipelineStageFlags2 Stage, VkAccessFlags2 Access)
{
	auto const writes{VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT | VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT | VK_ACCESS_2_TRANSFER_WRITE_BIT |
	                  VK_ACCESS_2_HOST_WRITE_BIT | VK_ACCESS_2_MEMORY_WRITE_BIT};
	auto const &from{Image.state};
	// reads after reads in the same layout need nothing, unless they are in stages the barrier after the last write didn't wait
	// for: those wait for the stages which read the image already (which came after the write)
	if ((from.layout == Layout) && ((from.access & writes) == 0) && ((Access & writes) == 0))
	{
		if ((Stage & ~from.stage) != 0)
		{
			VkMemoryBarrier2 chain{};
			chain.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
			chain.srcStageMask = (from.stage != VK_PIPELINE_STAGE_2_NONE ? from.stage : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT);
			chain.dstStageMask = Stage;
			chain.dstAccessMask = Access;
			VkDependencyInfo dependency{};
			dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
			dependency.memoryBarrierCount = 1;
			dependency.pMemoryBarriers = &chain;
			Commands->pipeline_barrier2(&dependency);
		}
		Image.state.stage |= Stage;
		Image.state.access |= Access;
		return;
	}
	VkImageMemoryBarrier2 barrier{};
	barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
	barrier.srcStageMask = (from.stage != VK_PIPELINE_STAGE_2_NONE ? from.stage : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT);
	barrier.srcAccessMask = (from.access & writes);
	barrier.dstStageMask = Stage;
	barrier.dstAccessMask = Access;
	barrier.oldLayout = from.layout;
	barrier.newLayout = Layout;
	barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
	barrier.image = Image.handle();
	barrier.subresourceRange = {Image.aspect(), 0, VK_REMAINING_MIP_LEVELS, 0, VK_REMAINING_ARRAY_LAYERS};

	VkDependencyInfo dependency{};
	dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
	dependency.imageMemoryBarrierCount = 1;
	dependency.pImageMemoryBarriers = &barrier;
	Commands->pipeline_barrier2(&dependency);

	Image.state = image_state{Layout, Stage, Access};
}

// deletion queue

void deletion_queue::push(std::uint64_t Value, std::function<void()> &&Deleter)
{
	m_entries.emplace_back(Value, std::move(Deleter));
}

void deletion_queue::collect(std::uint64_t Completed)
{
	// the values only grow, so the queue is in their order
	while ((false == m_entries.empty()) && (m_entries.front().first <= Completed))
	{
		auto deleter{std::move(m_entries.front().second)};
		m_entries.pop_front();
		deleter();
	}
}

void deletion_queue::flush()
{
	while (false == m_entries.empty())
	{
		auto deleter{std::move(m_entries.front().second)};
		m_entries.pop_front();
		deleter();
	}
}

// frame ring

bool frame_ring::create(allocator const &Allocator, VkDeviceSize SizePerFrame, std::uint32_t Frames, VkBufferUsageFlags Usage, std::string const &Name, VkDeviceSize Tail)
{
	m_sizeperframe = SizePerFrame;
	m_base = 0;
	m_used = 0;
	return m_buffer.create(Allocator, SizePerFrame * Frames + Tail, Usage, memory_kind::upload, Name) && (m_buffer.mapped() != nullptr);
}

void frame_ring::destroy()
{
	m_buffer.destroy();
	m_sizeperframe = 0;
	m_used = 0;
}

void frame_ring::begin_frame(std::uint32_t Frame)
{
	m_base = m_sizeperframe * Frame;
	m_used = 0;
}

ring_slice frame_ring::allocate(VkDeviceSize Size, VkDeviceSize Alignment)
{
	auto const alignment{std::max<VkDeviceSize>(Alignment, 4)};
	auto const offset{(m_used + alignment - 1) / alignment * alignment};
	if ((m_buffer.mapped() == nullptr) || (offset + Size > m_sizeperframe))
	{
		return {};
	}
	m_used = offset + Size;
	ring_slice slice;
	slice.buffer = m_buffer.handle();
	slice.offset = m_base + offset;
	slice.size = Size;
	slice.data = m_buffer.mapped() + slice.offset;
	return slice;
}

void frame_ring::flush() const
{
	m_buffer.flush(m_base, m_used);
}

} // namespace vk14
