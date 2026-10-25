/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the ray traced reflections of the vulkan renderer (gfx.raytracing, on a gpu with ray queries): what the screen space reflections
// don't find is traced through the scenery around the camera (shaders/vk14/rt_reflections.glsl).
//
// the renderer walks the scene for it once a frame (the walk in its reflection mode, gathering the draws instead of making them) and
// hands the draws over: each chunk of geometry gets a bottom level acceleration structure of its own the first time it's met (a few
// of them a frame, the rest wait for the next ones; made anew when its data changes, let go when it isn't met for a while), each draw
// an instance of the top level structure of the frame with a record the shading reads (where its vertices are, the texture and the
// colour of its material). each frame in flight has its own top level structure, buffers and set of the pipelines of the scene (set
// RT_SET), written while the frame is made: the gpu is done with the frame which used them before

#pragma once

#include "rendering/vulkan14/vulkan14memory.h"
#include "rendering/vulkan14/vulkan14geometry.h"

#include <memory>
#include <unordered_map>
#include <vector>

namespace vk14
{

class device;
class command_stream;
class image_table;

// a draw of the walk for the ray tracing: what an instance of the top level structure is made of
struct ray_draw
{
	glm::mat4 transform{1.f}; // the vertices of the chunk to the world around the camera (the axes of the world, the camera at the origin)
	std::uint64_t key{0}; // the chunk, gfx::geometry_handle
	chunk_source source; // where its data is
	std::uint64_t texture{0}; // the id of the diffuse texture in the image table, 0: none
	glm::vec3 colour{1.f}; // the diffuse colour of the material, linear
	float cutoff{0.f}; // the alpha test of the texture, 0: none
	float glow{0.f}; // the light of its own
};

class ray_tracing
{
  public:
	ray_tracing() = default;
	ray_tracing(ray_tracing const &) = delete;
	ray_tracing &operator=(ray_tracing const &) = delete;
	~ray_tracing();

	// Frames: the frames in flight
	bool init(device const &Device, allocator &Memory, deletion_queue &Deletions, std::uint32_t Frames, std::string &Reason);
	// the texture of the records without one (index 0 of the array), before the first frame
	void set_white(VkImageView View, VkSampler Sampler)
	{
		m_white = View;
		m_whitesampler = Sampler;
	}
	void destroy();
	bool ready() const
	{
		return m_layout != VK_NULL_HANDLE;
	}
	// the layout of the set of the pipelines of the scene
	VkDescriptorSetLayout layout() const
	{
		return m_layout;
	}
	// the frame Slot begins (its frame before is done on the gpu): the set of the slot gets an empty structure the first time, so the
	// shaders always have one to read. Value: of the timeline, when the gpu is done with the frame
	void begin_frame(command_stream *Commands, std::uint32_t Slot, std::uint64_t Value);
	// the scenery of the frame: the bottom level structures of the draws which aren't there yet (as many as a frame lets), the top
	// level structure with the instances of the draws, their records and textures written to the set of the frame. outside of a
	// rendering, before the passes which trace the rays
	void build(command_stream *Commands, std::uint32_t Slot, std::uint64_t Value, std::uint64_t Frame, std::vector<ray_draw> const &Draws, image_table const &Images);
	// the set of the frame
	VkDescriptorSet set(std::uint32_t Slot) const
	{
		return (Slot < m_frames.size() ? m_frames[Slot].set : VK_NULL_HANDLE);
	}
	// a scenery is being loaded, or the geometry is gone: the bottom level structures go
	void clear(std::uint64_t Value);
	// for the debug panel: the structures, their memory, the instances of the last frame
	std::string info() const;

  private:
	struct bottom
	{
		std::shared_ptr<buffer> storage;
		VkAccelerationStructureKHR handle{VK_NULL_HANDLE};
		VkDeviceAddress address{0};
		std::uint64_t generation{0}; // of the data of the chunk it was built from
		std::uint64_t used{0}; // the frame it was last met in
		VkDeviceSize bytes{0};
	};
	struct frame_data
	{
		VkDescriptorSet set{VK_NULL_HANDLE};
		// the top level structure, its memory and the instances it's built from, the records of the shading, the scratch of the builds
		std::shared_ptr<buffer> storage;
		VkAccelerationStructureKHR top{VK_NULL_HANDLE};
		std::uint32_t top_capacity{0}; // instances
		VkDeviceSize top_scratch{0}; // what a build of the structure needs of the scratch, with the alignment
		std::shared_ptr<buffer> instances;
		std::shared_ptr<buffer> records;
		std::shared_ptr<buffer> scratch;
		// what the set holds: the structure, the records, and the textures by their index
		VkAccelerationStructureKHR written_top{VK_NULL_HANDLE};
		VkBuffer written_records{VK_NULL_HANDLE};
		struct written_texture
		{
			VkImageView view{VK_NULL_HANDLE};
			VkSampler sampler{VK_NULL_HANDLE};
			std::weak_ptr<image> source; // a view made after this one is gone can get its handle
		};
		std::vector<written_texture> written_textures;
	};

	// a buffer of the frame at least Size bytes, grown when it isn't (the old one stays when the new one can't be made)
	bool ensure(std::shared_ptr<buffer> &Buffer, VkDeviceSize Size, VkBufferUsageFlags Usage, memory_kind Kind, char const *Name, std::uint64_t Value);
	// the top level structure of the frame for Count instances, recorded to Commands; false: no memory
	bool build_top(command_stream *Commands, frame_data &Frame, std::uint32_t Count, std::uint64_t Value);
	// the index of a texture in the array of the set, 0: none (white, or no room)
	std::uint32_t texture_index(std::uint64_t Id, std::uint64_t Frame);
	VkDeviceAddress aligned_scratch(frame_data const &Frame) const;

	device const *m_device{nullptr};
	allocator *m_memory{nullptr};
	deletion_queue *m_deletions{nullptr};
	VkDescriptorSetLayout m_layout{VK_NULL_HANDLE};
	VkDescriptorPool m_pool{VK_NULL_HANDLE};
	VkImageView m_white{VK_NULL_HANDLE};
	VkSampler m_whitesampler{VK_NULL_HANDLE};
	std::uint32_t m_texturecount{0};
	std::uint32_t m_scratchalignment{256};
	std::vector<frame_data> m_frames;
	std::unordered_map<std::uint64_t, bottom> m_bottoms; // by the chunk
	VkDeviceSize m_bottombytes{0};
	// the indices of the textures in the array: by the id of the image, the image of each index and the frame it was last used in
	std::unordered_map<std::uint64_t, std::uint32_t> m_textureindices;
	std::vector<std::uint64_t> m_textureowners;
	std::vector<std::uint64_t> m_textureused;
	std::vector<std::uint32_t> m_freetextures;
	// what the last frame had, for the panel
	std::uint32_t m_lastinstances{0};
	std::uint32_t m_lastrecords{0};
	std::uint32_t m_lastbuilt{0};
	std::uint32_t m_lastwaiting{0};
	std::uint32_t m_lasttextures{0};
};

} // namespace vk14
