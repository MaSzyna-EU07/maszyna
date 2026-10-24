/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/vulkan14/vulkan14geometry.h"
#include "rendering/vulkan14/vulkan14texture.h"

namespace vk14
{

namespace
{

// what opengl reads for the user data of a vertex without any (the attribute isn't given)
gfx::vertex_userdata const no_userdata{glm::vec4{0.f, 0.f, 0.f, 1.f}};

} // namespace

VkPrimitiveTopology topology_of(unsigned int const Type)
{
	switch (Type)
	{
	case GL_POINTS:
		return VK_PRIMITIVE_TOPOLOGY_POINT_LIST;
	case GL_LINES:
		return VK_PRIMITIVE_TOPOLOGY_LINE_LIST;
	case GL_LINE_STRIP:
	case GL_LINE_LOOP:
		return VK_PRIMITIVE_TOPOLOGY_LINE_STRIP;
	case GL_TRIANGLE_STRIP:
		return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
	case GL_TRIANGLE_FAN:
		return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_FAN;
	case GL_TRIANGLES:
	default:
		return VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
	}
}

vulkan14_geometrybank::~vulkan14_geometrybank()
{
	release_();
}

void vulkan14_geometrybank::create_(gfx::geometry_handle const &Geometry)
{
	// a new chunk: the buffers are made anew with it
	m_layoutdirty = true;
}

void vulkan14_geometrybank::replace_(gfx::geometry_handle const &Geometry)
{
	if (m_layoutdirty)
	{
		return;
	}
	auto const index{Geometry.chunk - 1};
	if (index >= m_places.size())
	{
		m_layoutdirty = true;
		return;
	}
	auto const &chunk{gfx::geometry_bank::chunk(Geometry)};
	auto &place{m_places[index]};
	auto const loop{chunk.type == GL_LINE_LOOP};
	auto const vertexcount{static_cast<std::uint32_t>(chunk.vertices.size()) + ((loop && chunk.indices.empty() && (false == chunk.vertices.empty())) ? 1u : 0u)};
	auto const indexcount{static_cast<std::uint32_t>(chunk.indices.size()) + ((loop && (false == chunk.indices.empty())) ? 1u : 0u)};
	if ((vertexcount != place.vertexcount) || (indexcount != place.indexcount))
	{
		// it doesn't fit in its place any more
		m_layoutdirty = true;
		return;
	}
	place.dirty = true;
	m_chunksdirty = true;
}

void vulkan14_geometrybank::release_()
{
	auto *deletions{m_resources.deletions};
	if ((deletions != nullptr) && (m_resources.uploads != nullptr) && (m_vertices || m_userdata || m_indices))
	{
		deletions->push(m_resources.uploads->current_value(), [vertices = std::move(m_vertices), userdata = std::move(m_userdata), indices = std::move(m_indices)]() {});
	}
	m_vertices.reset();
	m_userdata.reset();
	m_indices.reset();
	m_places.clear();
	m_bytes = 0;
	m_layoutdirty = true;
	m_chunksdirty = false;
}

void vulkan14_geometrybank::gpu_vertices(geometry_chunk const &Chunk, std::vector<gfx::basic_vertex> &Vertices, std::vector<gfx::vertex_userdata> &Userdata) const
{
	Vertices.insert(Vertices.end(), Chunk.vertices.begin(), Chunk.vertices.end());
	if (Chunk.userdata.size() >= Chunk.vertices.size())
	{
		Userdata.insert(Userdata.end(), Chunk.userdata.begin(), Chunk.userdata.begin() + Chunk.vertices.size());
	}
	else
	{
		Userdata.insert(Userdata.end(), Chunk.userdata.begin(), Chunk.userdata.end());
		Userdata.resize(Vertices.size(), no_userdata);
	}
	if ((Chunk.type == GL_LINE_LOOP) && Chunk.indices.empty() && (false == Chunk.vertices.empty()))
	{
		// back to the first vertex
		Vertices.push_back(Chunk.vertices.front());
		Userdata.push_back(Chunk.userdata.empty() ? no_userdata : Chunk.userdata.front());
	}
}

bool vulkan14_geometrybank::build()
{
	release_();

	std::vector<gfx::basic_vertex> vertices;
	std::vector<gfx::vertex_userdata> userdata;
	std::vector<gfx::basic_index> indices;
	m_places.resize(m_chunks.size());
	for (std::size_t i = 0; i < m_chunks.size(); ++i)
	{
		auto const &chunk{m_chunks[i]};
		auto &place{m_places[i]};
		place.vertexfirst = static_cast<std::uint32_t>(vertices.size());
		place.indexfirst = static_cast<std::uint32_t>(indices.size());
		gpu_vertices(chunk, vertices, userdata);
		place.vertexcount = static_cast<std::uint32_t>(vertices.size()) - place.vertexfirst;
		indices.insert(indices.end(), chunk.indices.begin(), chunk.indices.end());
		if ((chunk.type == GL_LINE_LOOP) && (false == chunk.indices.empty()))
		{
			indices.push_back(chunk.indices.front());
		}
		place.indexcount = static_cast<std::uint32_t>(indices.size()) - place.indexfirst;
		place.dirty = false;
	}
	m_layoutdirty = false;
	m_chunksdirty = false;
	if (vertices.empty())
	{
		return false;
	}

	auto &memory{*m_resources.memory};
	auto &uploads{*m_resources.uploads};
	auto const make = [&](std::shared_ptr<buffer> &Buffer, void const *Data, VkDeviceSize const Size, VkBufferUsageFlags const Usage, char const *Name) {
		Buffer = std::make_shared<buffer>();
		if ((false == Buffer->create(memory, Size, Usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT, memory_kind::device, Name)) || (false == uploads.upload_buffer(Buffer, Data, Size)))
		{
			Buffer.reset();
			return false;
		}
		m_bytes += Size;
		return true;
	};
	auto ok{make(m_vertices, vertices.data(), vertices.size() * sizeof(gfx::basic_vertex), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, "geometry vertices") &&
	        make(m_userdata, userdata.data(), userdata.size() * sizeof(gfx::vertex_userdata), VK_BUFFER_USAGE_VERTEX_BUFFER_BIT, "geometry user data")};
	if (ok && (false == indices.empty()))
	{
		ok = make(m_indices, indices.data(), indices.size() * sizeof(gfx::basic_index), VK_BUFFER_USAGE_INDEX_BUFFER_BIT, "geometry indices");
	}
	if (false == ok)
	{
		log_error("no memory for the geometry of a bank of " + std::to_string(vertices.size()) + " vertices");
		release_();
		// tried again when it's drawn the next time
		m_layoutdirty = true;
		return false;
	}
	return true;
}

void vulkan14_geometrybank::update()
{
	m_chunksdirty = false;
	if ((false == bool(m_vertices)) || (false == bool(m_userdata)))
	{
		return;
	}
	std::vector<gfx::basic_vertex> vertices;
	std::vector<gfx::vertex_userdata> userdata;
	std::vector<gfx::basic_index> indices;
	std::vector<std::pair<VkDeviceSize, VkDeviceSize>> vertexparts, userdataparts, indexparts;
	for (std::size_t i = 0; i < m_places.size(); ++i)
	{
		auto &place{m_places[i]};
		if (false == place.dirty)
		{
			continue;
		}
		place.dirty = false;
		auto const &chunk{m_chunks[i]};
		gpu_vertices(chunk, vertices, userdata);
		vertexparts.emplace_back(place.vertexfirst * sizeof(gfx::basic_vertex), place.vertexcount * sizeof(gfx::basic_vertex));
		userdataparts.emplace_back(place.vertexfirst * sizeof(gfx::vertex_userdata), place.vertexcount * sizeof(gfx::vertex_userdata));
		if ((place.indexcount > 0) && m_indices)
		{
			indices.insert(indices.end(), chunk.indices.begin(), chunk.indices.end());
			if (chunk.type == GL_LINE_LOOP)
			{
				indices.push_back(chunk.indices.front());
			}
			indexparts.emplace_back(place.indexfirst * sizeof(gfx::basic_index), place.indexcount * sizeof(gfx::basic_index));
		}
	}
	auto &uploads{*m_resources.uploads};
	if (false == vertices.empty())
	{
		uploads.upload_buffer(m_vertices, vertices.data(), vertices.size() * sizeof(gfx::basic_vertex), vertexparts);
		uploads.upload_buffer(m_userdata, userdata.data(), userdata.size() * sizeof(gfx::vertex_userdata), userdataparts);
	}
	if (false == indices.empty())
	{
		uploads.upload_buffer(m_indices, indices.data(), indices.size() * sizeof(gfx::basic_index), indexparts);
	}
}

bool vulkan14_geometrybank::source(gfx::geometry_handle const &Geometry, chunk_source &Source)
{
	if ((Geometry.chunk == 0) || (Geometry.chunk > m_chunks.size()))
	{
		return false;
	}
	if (m_layoutdirty)
	{
		build();
	}
	else if (m_chunksdirty)
	{
		update();
	}
	auto const index{Geometry.chunk - 1};
	if ((false == bool(m_vertices)) || (index >= m_places.size()))
	{
		return false;
	}
	auto const &place{m_places[index]};
	Source.vertices = m_vertices->handle();
	Source.userdata = m_userdata->handle();
	Source.topology = topology_of(m_chunks[index].type);
	if (place.indexcount > 0)
	{
		Source.indices = m_indices->handle();
		Source.count = place.indexcount;
		Source.first = place.indexfirst;
		Source.basevertex = static_cast<std::int32_t>(place.vertexfirst);
	}
	else
	{
		Source.indices = VK_NULL_HANDLE;
		Source.count = place.vertexcount;
		Source.first = place.vertexfirst;
		Source.basevertex = 0;
	}
	return Source.count > 0;
}

bool vulkan14_geometrybank::indirect_source_(gfx::geometry_handle const &Geometry, gfx::indirect_source &Source)
{
	m_last = chunk_source{};
	if (false == source(Geometry, m_last))
	{
		return false;
	}
	Source.count = m_last.count;
	Source.first = m_last.first;
	Source.basevertex = m_last.basevertex;
	Source.indexed = (m_last.indices != VK_NULL_HANDLE);
	Source.type = m_chunks[Geometry.chunk - 1].type;
	return true;
}

} // namespace vk14
