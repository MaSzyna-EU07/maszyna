/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the geometry banks of the vulkan renderer (stage D1 of the plan): the chunks of a bank in three device buffers of the bank
// (vertices, their user data, 32 bit indices), made the first time the bank is drawn after its chunks changed. a chunk replaced
// with the same size is copied in place, otherwise the buffers are made anew and the old ones go once the gpu is done with them.
// the data stays in memory as well (gfx::geometry_bank), the simulation reads it back. the primitives are the values of opengl
// the program gives them with; a line loop is drawn as a strip which goes back to its first vertex

#pragma once

#include "rendering/geometrybank.h"
#include "rendering/vulkan14/vulkan14common.h"
#include "rendering/vulkan14/vulkan14memory.h"

#include <memory>

namespace vk14
{

struct resources;

// where the data of a chunk is in the buffers of its bank
struct chunk_source
{
	VkBuffer vertices{VK_NULL_HANDLE};
	VkBuffer userdata{VK_NULL_HANDLE};
	VkBuffer indices{VK_NULL_HANDLE}; // null for a chunk without indices
	std::uint32_t count{0}; // vertices, or indices
	std::uint32_t first{0}; // first index, or vertex
	std::int32_t basevertex{0};
	VkPrimitiveTopology topology{VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST};
};

// the vulkan primitive of a primitive of opengl (GL_TRIANGLES ...)
VkPrimitiveTopology topology_of(unsigned int Type);

class vulkan14_geometrybank : public gfx::geometry_bank
{
  public:
	explicit vulkan14_geometrybank(resources const &Resources) : m_resources(Resources) {}
	~vulkan14_geometrybank() override;

	// the chunk with its data on the gpu: uploaded now when it changed since the last time. false: nothing to draw.
	// gfx::geometrybank_manager::indirect_source() calls it too (which marks the bank as used), and keeps its answer in last()
	bool source(gfx::geometry_handle const &Geometry, chunk_source &Source);
	chunk_source const &last() const
	{
		return m_last;
	}
	// the bytes on the gpu, for the debug panel
	VkDeviceSize gpu_bytes() const
	{
		return m_bytes;
	}

  private:
	struct chunk_place
	{
		std::uint32_t vertexfirst{0};
		std::uint32_t vertexcount{0}; // on the gpu: one more than the chunk for a line loop
		std::uint32_t indexfirst{0};
		std::uint32_t indexcount{0};
		bool dirty{true};
	};

	void create_(gfx::geometry_handle const &Geometry) override;
	void replace_(gfx::geometry_handle const &Geometry) override;
	std::size_t draw_(gfx::geometry_handle const &Geometry, gfx::stream_units const &Units, unsigned int const Streams) override
	{
		return 0;
	}
	bool indirect_source_(gfx::geometry_handle const &Geometry, gfx::indirect_source &Source) override;
	void release_() override;
	// the buffers made anew for all the chunks
	bool build();
	// the chunks changed in place copied to the buffers
	void update();
	// the vertices of a chunk as they go to the gpu
	void gpu_vertices(geometry_chunk const &Chunk, std::vector<gfx::basic_vertex> &Vertices, std::vector<gfx::vertex_userdata> &Userdata) const;

	resources const &m_resources;
	std::shared_ptr<buffer> m_vertices;
	std::shared_ptr<buffer> m_userdata;
	std::shared_ptr<buffer> m_indices;
	std::vector<chunk_place> m_places;
	bool m_layoutdirty{true}; // the buffers have to be made anew
	bool m_chunksdirty{false}; // some chunks have to be copied
	VkDeviceSize m_bytes{0};
	chunk_source m_last;
};

} // namespace vk14
