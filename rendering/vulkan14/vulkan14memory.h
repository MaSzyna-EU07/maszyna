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
#include <mutex>
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
	// the buffers and images made shared from now on are used by the queues of both families without transfers of their ownership
	// (VK_SHARING_MODE_CONCURRENT, the async compute of M6); nothing changes when the families are the same
	void share_between(std::uint32_t First, std::uint32_t Second);
	// the sharing of a buffer or an image made shared: the mode, and the families of the concurrent one (two of them)
	VkSharingMode sharing(bool Shared, std::uint32_t &Count, std::uint32_t const *&Families) const;

  private:
	VmaAllocator m_allocator{VK_NULL_HANDLE};
	VkDevice m_device{VK_NULL_HANDLE};
	std::array<std::uint32_t, 2> m_shared{VK_QUEUE_FAMILY_IGNORED, VK_QUEUE_FAMILY_IGNORED};
	std::uint32_t m_heapcount{0};
	std::vector<bool> m_deviceheaps;
};

// what the memory of the gpu holds by its device addresses, for the report of a lost device (device::fault_report()): the buffers, all
// made with device addresses while it's on, and the images, placed by the addresses of the buffers in the same blocks of memory (an
// image in a block of its own isn't placed). those let go not long ago are kept too. on with the fault info of the driver and the
// device addresses of the ray tracing
namespace address_registry
{
void enable(bool Enabled);
bool enabled();
// what holds the addresses Lower to Upper the gpu went wrong at, or what is next to them
std::string describe(VkDeviceAddress Lower, VkDeviceAddress Upper);
} // namespace address_registry

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

	// Shared: used by the queues of the families the allocator shares between (allocator::share_between())
	bool create(allocator const &Allocator, VkDeviceSize Size, VkBufferUsageFlags Usage, memory_kind Kind, std::string const &Name = {}, bool Shared = false);
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
	// the device address of the buffer, made with VK_BUFFER_USAGE_SHADER_DEVICE_ADDRESS_BIT (the ray tracing)
	VkDeviceAddress address() const;
	// makes the writes of the cpu seen by the gpu where the memory isn't coherent
	void flush(VkDeviceSize Offset, VkDeviceSize Size) const;
	// makes the writes of the gpu seen by the cpu where the memory isn't coherent (readback memory may be cached)
	void invalidate(VkDeviceSize Offset, VkDeviceSize Size) const;
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
	mutable VkDeviceAddress m_address{0}; // asked for the first time it's needed
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
	// used by the queues of the families the allocator shares between (allocator::share_between())
	bool shared{false};
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
void transition(command_stream *Commands, image &Image, VkImageLayout Layout, VkPipelineStageFlags2 Stage, VkAccessFlags2 Access);

// objects destroyed once the gpu is done with the frame which used them last
// pushed to from any thread (the walks of the passes and the background threads make and let go of objects too), collected by the
// thread of the renderer
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
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_entries.size();
	}

  private:
	mutable std::mutex m_mutex;
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
	// Tail: bytes past the part of the last frame, so a descriptor of a fixed range can be given an offset anywhere in a part
	bool create(allocator const &Allocator, VkDeviceSize SizePerFrame, std::uint32_t Frames, VkBufferUsageFlags Usage, std::string const &Name, VkDeviceSize Tail = 0, bool Shared = false);
	void destroy();
	// the part of the frame Frame is used from its start again
	void begin_frame(std::uint32_t Frame);
	// Size bytes aligned to Alignment; empty when the part of the frame is full. any thread (the walks of the passes, M2)
	ring_slice allocate(VkDeviceSize Size, VkDeviceSize Alignment);
	// makes the writes of the frame seen by the gpu, where the memory isn't coherent
	void flush() const;
	VkDeviceSize used() const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		return m_used;
	}
	// the offset in the buffer the next allocation aligned to Alignment would get
	VkDeviceSize next_offset(VkDeviceSize Alignment) const
	{
		std::lock_guard<std::mutex> lock(m_mutex);
		auto const alignment{std::max<VkDeviceSize>(Alignment, 4)};
		return m_base + (m_used + alignment - 1) / alignment * alignment;
	}
	VkDeviceSize capacity() const
	{
		return m_sizeperframe;
	}
	VkBuffer handle() const
	{
		return m_buffer.handle();
	}

  private:
	mutable std::mutex m_mutex;
	buffer m_buffer;
	VkDeviceSize m_sizeperframe{0};
	VkDeviceSize m_base{0};
	VkDeviceSize m_used{0};
};

// a part of a frame ring taken by a thread for the many small slices it places there (the blocks of the draws, the matrices of the
// instances), so the threads walking the passes (M2) don't take the lock of the ring for each of them. the slices follow one another
// in the part; one which doesn't fit in what's left of it takes a new part (or a part of its own size when it's larger than a part)
class ring_cache
{
  public:
	// Ring: where the parts come from (null: none), Part: their size, bytes. the part held is let go
	void reset(frame_ring *Ring, VkDeviceSize Part);
	// Size bytes aligned to Alignment (at most 256); empty when the ring is full
	ring_slice allocate(VkDeviceSize Size, VkDeviceSize Alignment);
	// the same, only when the slice fits in the part held and lies within Base to Limit (offsets in the buffer: a window the shaders
	// index from Base); empty, with nothing taken, otherwise. the part held may lie below Base: a large slice which got a part of its
	// own opened the window above it
	ring_slice allocate_before(VkDeviceSize Size, VkDeviceSize Alignment, VkDeviceSize Base, VkDeviceSize Limit);

  private:
	// where the next slice aligned to Alignment would begin in the part held, as an offset in the buffer
	VkDeviceSize next(VkDeviceSize Alignment) const;

	frame_ring *m_ring{nullptr};
	VkDeviceSize m_partsize{0};
	ring_slice m_part;
	VkDeviceSize m_used{0}; // of the part held
};

} // namespace vk14
