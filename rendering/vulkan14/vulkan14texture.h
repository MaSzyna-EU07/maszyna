/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the images the vulkan renderer samples: the textures of the files (texture_source, decoded as for the opengl renderer and
// laid out the same way in the memory, decision K8 of the plan), the images of the user interface, and their uploads.
//
// the uploads are copies through staging buffers recorded into the command buffer of uploads of the frame being made, which goes
// to the queue before the commands of the frame in the same submit; an upload asked for between the frames waits for the next
// one. every image has an id in one table (decision K10): ITexture::get_id() of the textures, the ids of the images of the user
// interface and the font of imgui are all ImTextureID, so the user interface draws any of them

#pragma once

#include "rendering/vulkan14/vulkan14memory.h"
#include "rendering/vulkan14/vulkan14descriptors.h"
#include "model/Texture.h"

#include <memory>
#include <mutex>
#include <thread>

namespace vk14
{

class device;

// the uploads of data to the images
class uploader
{
  public:
	struct level
	{
		VkDeviceSize offset{0}; // in the data
		VkExtent3D extent{1, 1, 1};
	};

	void init(allocator const &Allocator, deletion_queue &Deletions, VkPhysicalDevice Physical);
	// the commands of the uploads go to Commands until end_frame(); Value: the value of the timeline the gpu reaches when it's done
	// with the frame. the uploads waiting from between the frames are recorded first
	void begin_frame(command_stream *Commands, std::uint64_t Value);
	// whether anything was recorded to the commands of the frame
	bool end_frame();
	// the last value of the timeline the gpu may use the objects with: the frame being made, or the last one sent
	std::uint64_t current_value() const
	{
		return m_value;
	}
	// copies Data into Image (the levels given, the others made by blits of the first one when Mipmaps), the image left ready to be
	// sampled. Data is copied away. false: no staging memory, nothing will be copied
	bool upload(std::shared_ptr<image> const &Image, void const *Data, VkDeviceSize Size, std::vector<level> const &Levels, bool const Mipmaps);
	// copies Data into Target, done before the draws of the frame read it (and after the frames sent before did). Data holds the
	// pieces of Parts (offset in Target, size) one after another; all of Data goes to the start of Target when there are none.
	// false: no staging memory
	bool upload_buffer(std::shared_ptr<buffer> const &Target, void const *Data, VkDeviceSize Size, std::vector<std::pair<VkDeviceSize, VkDeviceSize>> const &Parts = {});
	// the bytes uploaded since the start, for the debug panel
	std::uint64_t uploaded() const
	{
		return m_uploaded;
	}
	// whether the format can be filtered by blits (the mipmaps made on the gpu)
	bool can_blit(VkFormat Format) const;

  private:
	struct job
	{
		std::shared_ptr<image> target;
		std::shared_ptr<buffer> staging;
		std::vector<level> levels;
		bool mipmaps{false};
		// a copy to a buffer instead: the pieces copied
		std::shared_ptr<buffer> buffertarget;
		std::vector<std::pair<VkDeviceSize, VkDeviceSize>> parts;
	};
	void record(job &Job);

	allocator const *m_allocator{nullptr};
	deletion_queue *m_deletions{nullptr};
	VkPhysicalDevice m_physical{VK_NULL_HANDLE};
	command_stream *m_commands{nullptr};
	std::uint64_t m_value{0};
	bool m_recorded{false};
	// buffers were copied to in the frame: the barriers before the first copy and after the last one
	bool m_buffercopies{false};
	std::vector<job> m_pending;
	std::uint64_t m_uploaded{0};
	mutable std::unordered_map<VkFormat, bool> m_blittable;
};

// the images the shaders can be given by an id
class image_table
{
  public:
	struct entry
	{
		std::shared_ptr<image> target;
		VkSampler sampler{VK_NULL_HANDLE};
	};
	// a new id for the image (0 is no image)
	std::uint64_t add(std::shared_ptr<image> const &Image, VkSampler Sampler);
	void replace(std::uint64_t Id, std::shared_ptr<image> const &Image, VkSampler Sampler);
	// the id is free again, the image goes once the gpu is done with it
	void remove(std::uint64_t Id, deletion_queue &Deletions, std::uint64_t Value);
	entry const *find(std::uint64_t Id) const;
	void clear();
	std::size_t size() const
	{
		return m_entries.size() - m_free.size();
	}

  private:
	std::vector<entry> m_entries{entry{}};
	std::vector<std::uint64_t> m_free;
};

struct vulkan14_texture;

// the textures given content by other threads, waiting for the thread of the renderer
struct pending_textures
{
	std::mutex mutex;
	std::vector<vulkan14_texture *> textures;
};

// what the images of the renderer need from it
struct resources
{
	device const *gpu{nullptr};
	allocator *memory{nullptr};
	uploader *uploads{nullptr};
	deletion_queue *deletions{nullptr};
	samplers const *sampling{nullptr};
	image_table *images{nullptr};
	// the thread of the renderer: the content given by other threads waits for it (update_from_memory())
	std::thread::id thread;
	pending_textures *pending{nullptr};
	// the ray tracing is on (gfx.raytracing): the geometry is built into acceleration structures and read by its device address
	bool raytracing{false};
};

// the vulkan format of the data of a texture and how the data has to be changed for it
struct texture_format
{
	VkFormat format{VK_FORMAT_UNDEFINED};
	bool expand_rgb{false}; // rgb texels made rgba
	bool compressed{false};
	int block_bytes{0}; // of a block of 4x4 texels, for the compressed formats
	// the channels the material reads, as the internal format of opengl would leave them
	VkComponentMapping components{VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY, VK_COMPONENT_SWIZZLE_IDENTITY};
};
// Format: data_format of a texture_source, Hint: the components the material wants (GL_SRGB_ALPHA ...)
texture_format texture_format_of(GLint Format, GLint Hint);

// a texture of the vulkan renderer
struct vulkan14_texture : public texture_source
{
	explicit vulkan14_texture(resources const &Resources) : m_resources(Resources) {}
	~vulkan14_texture() override;

	bool create(bool const Static = false) override;
	void release() override;
	// as the opengl textures give it: one not made yet has no id of 0, which the materials take for a texture which is missing
	size_t get_id() const override
	{
		return (m_made ? static_cast<size_t>(m_id) : std::numeric_limits<size_t>::max());
	}
	void update_from_memory(size_t width, size_t height, const uint8_t *data) override;
	bool resident() const override
	{
		return m_made;
	}
	void destroy() override;

	// the content given by update_from_memory() from another thread, uploaded by the thread of the renderer
	void apply_pending();
	// the image of a texture the renderer fills itself (the textures of the heightmap terrain): sampled with Sampler from now on,
	// the image it had before goes once the gpu is done with it
	void adopt(std::shared_ptr<image> const &Image, VkSampler Sampler);
	std::shared_ptr<image> const &adopted() const
	{
		return m_image;
	}

  protected:
	void on_failed() override;

  private:
	void upload_rgba(std::uint8_t const *Data, std::size_t Width, std::size_t Height);

	resources const &m_resources;
	std::shared_ptr<image> m_image;
	std::uint64_t m_id{0};
	bool m_made{false};
	// content from another thread, waiting for the thread of the renderer
	std::mutex m_pendingmutex;
	std::vector<std::uint8_t> m_pending;
	std::size_t m_pendingwidth{0};
	std::size_t m_pendingheight{0};
};

} // namespace vk14
