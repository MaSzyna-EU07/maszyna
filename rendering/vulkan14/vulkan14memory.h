/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the memory of the vulkan renderer: the allocator (VMA), buffers and images which give their memory back when they go, the
// ring each frame in flight streams its data through, the uploads of data to the gpu through staging buffers, and the queue
// of objects deleted once the gpu is done with the frames which used them (by the value of the timeline semaphore)

#pragma once

#include "rendering/vulkan14/vulkan14common.h"

#include <deque>
#include <functional>
#include <vk_mem_alloc.h>

namespace vk14
{

class instance;
class device;

// the allocator of the device memory
class allocator
{
  public:
	allocator() = default;
	allocator(allocator const &) = delete;
	allocator &operator=(allocator const &) = delete;
	~allocator();

	bool create(instance const &Instance, device const &Device, std::string &Reason);
	void destroy();

	VmaAllocator handle() const
	{
		return m_allocator;
	}
	VkDevice device_handle() const
	{
		return m_device;
	}
	// device memory used and the budget of the heaps which are local to the device, bytes
	void budget(VkDeviceSize &Usage, VkDeviceSize &Budget) const;

  private:
	VmaAllocator m_allocator{VK_NULL_HANDLE};
	VkDevice m_device{VK_NULL_HANDLE};
	std::uint32_t m_heapcount{0};
	std::vector<bool> m_deviceheaps;
};

// where the memory of a buffer is
enum class memory_kind
{
	device, // device local, filled by copies
	upload, // seen by the cpu, written in order (staging, streamed data); device local where the device has such memory
	readback // seen by the cpu, cached, for the data read back from the gpu
};

class buffer
{
  public:
	buffer() = default;
	buffer(buffer const &) = delete;
	buffer &operator=(buffer const &) = delete;
	buffer(buffer &&Other) noexcept;
	buffer &operator=(buffer &&Other) noexcept;
	~buffer();

	bool create(allocator const &Allocator, VkDeviceSize Size, VkBufferUsageFlags Usage, memory_kind Kind, std::string const &Name = {});
	void destroy();

	VkBuffer handle() const
	{
		return m_buffer;
	}
	VkDeviceSize size() const
	{
		return m_size;
	}
	// the memory as the cpu sees it, null for memory the cpu doesn't see. the upload and readback memory stays mapped
	std::uint8_t *mapped() const
	{
		return m_mapped;
	}
	// makes the writes of the cpu seen by the gpu where the memory isn't coherent
	void flush(VkDeviceSize Offset, VkDeviceSize Size) const;
	explicit operator bool() const
	{
		return m_buffer != VK_NULL_HANDLE;
	}

  private:
	VmaAllocator m_allocator{VK_NULL_HANDLE};
	VkBuffer m_buffer{VK_NULL_HANDLE};
	VmaAllocation m_allocation{VK_NULL_HANDLE};
	VkDeviceSize m_size{0};
	std::uint8_t *m_mapped{nullptr};
};

// the state of an image the barriers start from: its layout and the last stage and access which used it
struct image_state
{
	VkImageLayout layout{VK_IMAGE_LAYOUT_UNDEFINED};
	VkPipelineStageFlags2 stage{VK_PIPELINE_STAGE_2_NONE};
	VkAccessFlags2 access{VK_ACCESS_2_NONE};
};

struct image_desc
{
	VkImageType type{VK_IMAGE_TYPE_2D};
	VkImageViewType view_type{VK_IMAGE_VIEW_TYPE_2D};
	VkFormat format{VK_FORMAT_R8G8B8A8_UNORM};
	VkExtent3D extent{1, 1, 1};
	std::uint32_t mips{1};
	std::uint32_t layers{1};
	VkSampleCountFlagBits samples{VK_SAMPLE_COUNT_1_BIT};
	VkImageUsageFlags usage{VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT};
	VkImageCreateFlags flags{0};
	// the channels the default view reads, e.g. a single channel texture seen as rrr1
	VkComponentMapping components{VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY};
};

class image
{
  public:
	image() = default;
	image(image const &) = delete;
	image &operator=(image const &) = delete;
	~image();

	// a device local image with a view of all its levels and layers
	bool create(allocator const &Allocator, image_desc const &Desc, std::string const &Name = {});
	void destroy();

	VkImage handle() const
	{
		return m_image;
	}
	VkImageView view() const
	{
		return m_view;
	}
	image_desc const &desc() const
	{
		return m_desc;
	}
	VkImageAspectFlags aspect() const;
	VkDeviceSize bytes() const
	{
		return m_bytes;
	}
	// the state the next barrier starts from (the pass list and the uploads keep it)
	image_state state{};
	explicit operator bool() const
	{
		return m_image != VK_NULL_HANDLE;
	}

  private:
	VmaAllocator m_allocator{VK_NULL_HANDLE};
	VkDevice m_device{VK_NULL_HANDLE};
	VkImage m_image{VK_NULL_HANDLE};
	VkImageView m_view{VK_NULL_HANDLE};
	VmaAllocation m_allocation{VK_NULL_HANDLE};
	VkDeviceSize m_bytes{0};
	image_desc m_desc;
};

// a barrier of an image from its state to the given one, the state of the image updated. nothing when the image already is
// in the layout and was only read by the same stage
void transition(VkCommandBuffer Commands, image &Image, VkImageLayout Layout, VkPipelineStageFlags2 Stage, VkAccessFlags2 Access);

// objects destroyed once the gpu is done with the frame which used them last
class deletion_queue
{
  public:
	// Value: the value of the timeline semaphore the gpu reaches when it's done with the object
	void push(std::uint64_t Value, std::function<void()> &&Deleter);
	// runs the deleters of the values the gpu reached
	void collect(std::uint64_t Completed);
	// runs all of them (the device is idle)
	void flush();
	std::size_t size() const
	{
		return m_entries.size();
	}

  private:
	std::deque<std::pair<std::uint64_t, std::function<void()>>> m_entries;
};

// a part of the ring of a frame: where the data goes on the cpu, and where the gpu reads it
struct ring_slice
{
	std::uint8_t *data{nullptr};
	VkBuffer buffer{VK_NULL_HANDLE};
	VkDeviceSize offset{0};
	VkDeviceSize size{0};
	explicit operator bool() const
	{
		return data != nullptr;
	}
};

// the data streamed by the frames: one mapped buffer split into a part for each frame in flight, the part of a frame taken
// from the start again once the gpu is done with the frame which used it before (the counterpart of gl::frame_stream)
class frame_ring
{
  public:
	bool create(allocator const &Allocator, VkDeviceSize SizePerFrame, std::uint32_t Frames, VkBufferUsageFlags Usage, std::string const &Name);
	void destroy();
	// the part of the frame Frame is used from its start again
	void begin_frame(std::uint32_t Frame);
	// Size bytes aligned to Alignment; empty when the part of the frame is full
	ring_slice allocate(VkDeviceSize Size, VkDeviceSize Alignment);
	// makes the writes of the frame seen by the gpu, where the memory isn't coherent
	void flush() const;
	VkDeviceSize used() const
	{
		return m_used;
	}
	VkDeviceSize capacity() const
	{
		return m_sizeperframe;
	}

  private:
	buffer m_buffer;
	VkDeviceSize m_sizeperframe{0};
	VkDeviceSize m_base{0};
	VkDeviceSize m_used{0};
};

} // namespace vk14
