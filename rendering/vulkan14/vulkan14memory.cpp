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

#include <cinttypes>
#include <cstdio>

namespace vk14
{

// address registry

namespace address_registry
{

namespace
{

struct entry
{
	VkDeviceAddress address{0}; // of a buffer; 0 for an image, placed by its memory and offset
	VkDeviceMemory memory{VK_NULL_HANDLE};
	VkDeviceSize offset{0};
	VkDeviceSize size{0};
	std::string name;
	bool image{false};
	bool dedicated{false}; // an image in memory of its own, which has no device address: it isn't placed
	std::chrono::steady_clock::time_point freed{};
};

std::atomic<bool> g_enabled{false};
std::mutex g_mutex;
// by the handle of the buffer or the image
std::unordered_map<std::uint64_t, entry> g_live;
// the last ones let go, the oldest first
std::deque<entry> g_freed;
std::size_t constexpr freed_kept{16384};

void add(std::uint64_t const Handle, entry &&Entry)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	g_live[Handle] = std::move(Entry);
}

void remove(std::uint64_t const Handle)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	auto const lookup{g_live.find(Handle)};
	if (lookup == g_live.end())
	{
		return;
	}
	lookup->second.freed = std::chrono::steady_clock::now();
	g_freed.push_back(std::move(lookup->second));
	g_live.erase(lookup);
	if (g_freed.size() > freed_kept)
	{
		g_freed.pop_front();
	}
}

std::string hex(std::uint64_t const Value)
{
	char text[24];
	std::snprintf(text, sizeof(text), "0x%" PRIx64, Value);
	return text;
}

std::string bytes(VkDeviceSize const Size)
{
	return (Size >= 1024 * 1024 ? std::to_string(Size / (1024 * 1024)) + " MiB" : Size >= 1024 ? std::to_string(Size / 1024) + " KiB" : std::to_string(Size) + " B");
}

} // namespace

void enable(bool const Enabled)
{
	std::lock_guard<std::mutex> lock(g_mutex);
	g_enabled = Enabled;
	g_live.clear();
	g_freed.clear();
}

bool enabled()
{
	return g_enabled;
}

std::string describe(VkDeviceAddress const Lower, VkDeviceAddress const Upper)
{
	if (false == g_enabled)
	{
		return {};
	}
	std::lock_guard<std::mutex> lock(g_mutex);
	// the addresses of the blocks of memory, from the buffers in them: an image is at the address of its block and its offset
	std::unordered_map<VkDeviceMemory, VkDeviceAddress> blocks;
	for (auto const &[handle, item] : g_live)
	{
		if ((item.address != 0) && (item.memory != VK_NULL_HANDLE) && (item.address >= item.offset))
		{
			blocks.emplace(item.memory, item.address - item.offset);
		}
	}
	auto const placed = [&](entry const &Item) -> VkDeviceAddress {
		if (Item.address != 0)
		{
			return Item.address;
		}
		auto const block{blocks.find(Item.memory)};
		return (block != blocks.end() ? block->second + Item.offset : 0);
	};
	auto const label = [&](entry const &Item, VkDeviceAddress const Address) {
		return std::string(Item.image ? "image" : "buffer") + " \"" + Item.name + "\" (" + hex(Address) + ", " + bytes(Item.size) + ")";
	};
	auto const now{std::chrono::steady_clock::now()};
	std::string result;
	auto const note = [&](std::string const &Text) { result += (result.empty() ? "" : ", ") + Text; };
	// what holds the addresses
	entry const *below{nullptr};
	entry const *above{nullptr};
	VkDeviceAddress belowend{0}, abovestart{0};
	std::size_t holders{0};
	for (auto const &[handle, item] : g_live)
	{
		auto const address{placed(item)};
		if (address == 0)
		{
			continue;
		}
		auto const end{address + item.size};
		if ((address <= Upper) && (end > Lower))
		{
			if (++holders <= 4)
			{
				note("in the " + label(item, address) + " at its byte " + std::to_string(Lower > address ? Lower - address : 0));
			}
			continue;
		}
		if ((end <= Lower) && ((below == nullptr) || (end > belowend)))
		{
			below = &item;
			belowend = end;
		}
		if ((address > Upper) && ((above == nullptr) || (address < abovestart)))
		{
			above = &item;
			abovestart = address;
		}
	}
	// what was let go there not long ago, the last ones first
	std::size_t freed{0};
	for (auto it = g_freed.rbegin(); (it != g_freed.rend()) && (freed < 4); ++it)
	{
		auto const address{placed(*it)};
		if ((address != 0) && (address <= Upper) && (address + it->size > Lower))
		{
			++freed;
			auto const ago{std::chrono::duration<double, std::milli>(now - it->freed).count()};
			note("in the " + label(*it, address) + " let go " + std::to_string(static_cast<long long>(ago)) + " ms ago");
		}
	}
	if (holders == 0)
	{
		if (below != nullptr)
		{
			note(bytes(Lower - belowend) + " past the end of the " + label(*below, belowend - below->size));
		}
		if (above != nullptr)
		{
			note(bytes(abovestart - Upper) + " before the " + label(*above, abovestart));
		}
		if (freed == 0)
		{
			// the images in memory of their own aren't placed: those let go just before may be what the address was
			std::string recent;
			std::size_t count{0};
			for (auto it = g_freed.rbegin(); it != g_freed.rend(); ++it)
			{
				auto const ago{std::chrono::duration<double, std::milli>(now - it->freed).count()};
				if (ago > 5000.0)
				{
					break;
				}
				if (it->dedicated && (++count <= 8))
				{
					recent += (recent.empty() ? "" : ", ") + std::string("\"") + it->name + "\" (" + bytes(it->size) + ", " + std::to_string(static_cast<long long>(ago)) + " ms ago)";
				}
			}
			std::size_t live{0};
			for (auto const &[handle, item] : g_live)
			{
				live += (item.dedicated ? 1 : 0);
			}
			note(std::to_string(live) + " images in memory of their own aren't placed; let go of them in the last 5 s: " + (recent.empty() ? std::string("none") : recent));
		}
	}
	// an index below zero of a buffer read through a descriptor wraps around the 32 bits of the offset, to almost 4 GiB past its
	// start: what is there 4 GiB lower is the buffer read
	auto constexpr wrap{VkDeviceAddress{1} << 32};
	if ((holders == 0) && (Lower >= wrap))
	{
		auto const lower{Lower - wrap};
		auto const upper{Upper - wrap};
		entry const *holder{nullptr};
		entry const *next{nullptr};
		VkDeviceAddress holderstart{0}, nextstart{0};
		for (auto const &[handle, item] : g_live)
		{
			auto const address{placed(item)};
			if (address == 0)
			{
				continue;
			}
			if ((address <= upper) && (address + item.size > lower))
			{
				holder = &item;
				holderstart = address;
				break;
			}
			if ((address > upper) && ((next == nullptr) || (address < nextstart)))
			{
				next = &item;
				nextstart = address;
			}
		}
		if (holder != nullptr)
		{
			note("4 GiB lower (an offset wrapped around 32 bits) in the " + label(*holder, holderstart) + " at its byte " + std::to_string(lower > holderstart ? lower - holderstart : 0));
		}
		else if (next != nullptr)
		{
			note("4 GiB lower (an index below zero wrapped around 32 bits?) " + bytes(nextstart - upper) + " before the " + label(*next, nextstart));
		}
	}
	return (result.empty() ? std::string("in nothing known") : result);
}

// the device address of the memory of an image in a block of the allocator: a buffer bound to the same memory tells it, and goes.
// 0 where that can't be (the memory type or the alignment doesn't take a buffer)
VkDeviceAddress image_address(VmaAllocator const Allocator, VkDevice const Device, VmaAllocation const Allocation, VmaAllocationInfo const &Allocated)
{
	VkBufferCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	info.size = Allocated.size;
	info.usage = VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
	info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	VkBuffer probe{VK_NULL_HANDLE};
	if (vkCreateBuffer(Device, &info, nullptr, &probe) != VK_SUCCESS)
	{
		return 0;
	}
	VkDeviceAddress address{0};
	VkMemoryRequirements requirements{};
	vkGetBufferMemoryRequirements(Device, probe, &requirements);
	if (((requirements.memoryTypeBits & (1u << Allocated.memoryType)) != 0) && (requirements.alignment > 0) && ((Allocated.offset % requirements.alignment) == 0) && (requirements.size <= Allocated.size) &&
	    (vmaBindBufferMemory2(Allocator, Allocation, 0, probe, nullptr) == VK_SUCCESS))
	{
		VkBufferDeviceAddressInfo addressinfo{};
		addressinfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
		addressinfo.buffer = probe;
		address = vkGetBufferDeviceAddress(Device, &addressinfo);
	}
	vkDestroyBuffer(Device, probe, nullptr);
	return address;
}

} // namespace address_registry

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
	// the addresses of the buffers and the images for the report of a lost device, where the driver tells where the gpu went wrong
	address_registry::enable(Device.fault_enabled() && Device.raytracing_enabled());
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

void allocator::share_between(std::uint32_t const First, std::uint32_t const Second)
{
	m_shared = {First, Second};
}

VkSharingMode allocator::sharing(bool const Shared, std::uint32_t &Count, std::uint32_t const *&Families) const
{
	Count = 0;
	Families = nullptr;
	if ((false == Shared) || (m_shared[0] == VK_QUEUE_FAMILY_IGNORED) || (m_shared[1] == VK_QUEUE_FAMILY_IGNORED) || (m_shared[0] == m_shared[1]))
	{
		return VK_SHARING_MODE_EXCLUSIVE;
	}
	Count = 2;
	Families = m_shared.data();
	return VK_SHARING_MODE_CONCURRENT;
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

bool buffer::create(allocator const &Allocator, VkDeviceSize Size, VkBufferUsageFlags Usage, memory_kind Kind, std::string const &Name, bool const Shared)
{
	destroy();

	VkBufferCreateInfo info{};
	info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
	info.size = std::max<VkDeviceSize>(Size, 4);
	info.usage = Usage;
	info.sharingMode = Allocator.sharing(Shared, info.queueFamilyIndexCount, info.pQueueFamilyIndices);

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
	auto const registered{address_registry::enabled()};
	if (registered)
	{
		// every buffer has its address, so the report of a lost device can tell which one the gpu went wrong in
		info.usage |= VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT;
	}
	if (!VK14_VERIFY(vmaCreateBuffer(Allocator.handle(), &info, &allocation, &m_buffer, &m_allocation, &allocated)))
	{
		m_buffer = VK_NULL_HANDLE;
		m_allocation = VK_NULL_HANDLE;
		return false;
	}
	if (registered)
	{
		VkBufferDeviceAddressInfo addressinfo{};
		addressinfo.sType = VK_STRUCTURE_TYPE_BUFFER_DEVICE_ADDRESS_INFO;
		addressinfo.buffer = m_buffer;
		m_address = vkGetBufferDeviceAddress(Allocator.device_handle(), &addressinfo);
		address_registry::add(handle_value(m_buffer), {m_address, allocated.deviceMemory, allocated.offset, info.size, Name.empty() ? std::string("unnamed") : Name, false, false, {}});
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
		if (address_registry::enabled())
		{
			address_registry::remove(handle_value(m_buffer));
		}
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
	info.sharingMode = Allocator.sharing(Desc.shared, info.queueFamilyIndexCount, info.pQueueFamilyIndices);
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
	if (address_registry::enabled())
	{
		// an image in a block of the allocator (the memory has device addresses there) is placed exactly; one in memory of its own isn't
		VmaAllocationInfo2 extended{};
		vmaGetAllocationInfo2(m_allocator, m_allocation, &extended);
		auto const dedicated{extended.dedicatedMemory == VK_TRUE};
		auto const address{dedicated ? VkDeviceAddress{0} : address_registry::image_address(m_allocator, m_device, m_allocation, allocated)};
		address_registry::add(handle_value(m_image), {address, allocated.deviceMemory, allocated.offset, allocated.size, Name.empty() ? std::string("unnamed") : Name, true, dedicated, {}});
	}

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
		if (address_registry::enabled())
		{
			address_registry::remove(handle_value(m_image));
		}
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
	std::lock_guard<std::mutex> lock(m_mutex);
	m_entries.emplace_back(Value, std::move(Deleter));
}

void deletion_queue::collect(std::uint64_t Completed)
{
	// the values only grow, so the queue is in their order (an object let go by another thread while the frame is made takes the
	// value of the frame too). the deleters run outside of the lock
	std::vector<std::function<void()>> deleters;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		while ((false == m_entries.empty()) && (m_entries.front().first <= Completed))
		{
			deleters.emplace_back(std::move(m_entries.front().second));
			m_entries.pop_front();
		}
	}
	for (auto &deleter : deleters)
	{
		deleter();
	}
}

void deletion_queue::flush()
{
	std::deque<std::pair<std::uint64_t, std::function<void()>>> entries;
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		entries.swap(m_entries);
	}
	for (auto &entry : entries)
	{
		entry.second();
	}
}

// frame ring

bool frame_ring::create(allocator const &Allocator, VkDeviceSize SizePerFrame, std::uint32_t Frames, VkBufferUsageFlags Usage, std::string const &Name, VkDeviceSize Tail, bool const Shared)
{
	m_sizeperframe = SizePerFrame;
	m_base = 0;
	m_used = 0;
	return m_buffer.create(Allocator, SizePerFrame * Frames + Tail, Usage, memory_kind::upload, Name, Shared) && (m_buffer.mapped() != nullptr);
}

void frame_ring::destroy()
{
	m_buffer.destroy();
	m_sizeperframe = 0;
	m_used = 0;
}

void frame_ring::begin_frame(std::uint32_t Frame)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	m_base = m_sizeperframe * Frame;
	m_used = 0;
}

ring_slice frame_ring::allocate(VkDeviceSize Size, VkDeviceSize Alignment)
{
	std::lock_guard<std::mutex> lock(m_mutex);
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
	std::lock_guard<std::mutex> lock(m_mutex);
	m_buffer.flush(m_base, m_used);
}

void ring_cache::reset(frame_ring *const Ring, VkDeviceSize const Part)
{
	m_ring = Ring;
	m_partsize = Part;
	m_part = ring_slice{};
	m_used = 0;
}

VkDeviceSize ring_cache::next(VkDeviceSize const Alignment) const
{
	auto const alignment{std::max<VkDeviceSize>(Alignment, 4)};
	return (m_part.offset + m_used + alignment - 1) / alignment * alignment;
}

ring_slice ring_cache::allocate_before(VkDeviceSize const Size, VkDeviceSize const Alignment, VkDeviceSize const Base, VkDeviceSize const Limit)
{
	if (false == bool(m_part))
	{
		return {};
	}
	auto const offset{next(Alignment)};
	if ((offset < Base) || (offset + Size > m_part.offset + m_part.size) || (offset + Size > Limit))
	{
		return {};
	}
	m_used = offset + Size - m_part.offset;
	ring_slice slice;
	slice.buffer = m_part.buffer;
	slice.offset = offset;
	slice.size = Size;
	slice.data = m_part.data + (offset - m_part.offset);
	return slice;
}

ring_slice ring_cache::allocate(VkDeviceSize const Size, VkDeviceSize const Alignment)
{
	if (auto const slice{allocate_before(Size, Alignment, 0, std::numeric_limits<VkDeviceSize>::max())})
	{
		return slice;
	}
	if (m_ring == nullptr)
	{
		return {};
	}
	// the parts begin aligned for any of the slices
	auto const partalignment{std::max<VkDeviceSize>(Alignment, 256)};
	if (Size > m_partsize / 2)
	{
		// a large one gets a part of its own, the part held is kept for the small ones
		return m_ring->allocate(Size, partalignment);
	}
	m_part = m_ring->allocate(m_partsize, partalignment);
	m_used = 0;
	if (false == bool(m_part))
	{
		// the ring is nearly full: the rest of it in a part of the size of the slice
		m_part = m_ring->allocate(Size, partalignment);
		if (false == bool(m_part))
		{
			return {};
		}
	}
	return allocate_before(Size, Alignment, 0, std::numeric_limits<VkDeviceSize>::max());
}

} // namespace vk14
