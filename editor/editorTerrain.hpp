/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <memory>
#include <vector>
#include <glm/glm.hpp>

#include "scene/heightmapterrain.h"
#include "utilities/Classes.h"      // material_handle
#include "interfaces/ITexture.h"    // null_handle
#include "rendering/geometrybank.h" // gfx::geometry_handle

namespace scene { class basic_section; }

// a chunk of the heightmap terrain held in memory: its data, and what's drawn of it.
//
// the data is shared with the worker threads making its meshes, so it's never changed in place: an edit gets a copy
// of its own when the current data is in use elsewhere. what's drawn is managed by the terrain streamer
class editor_terrain
{
  public:
	explicit editor_terrain(heightmap::chunk_ptr Data);

	heightmap::chunk_data const &data() const { return *m_data; }
	heightmap::chunk_ptr const &shared() const { return m_data; }
	// the data, to be changed. marks the chunk as edited; Shape: the heights changed, Paint: the materials changed
	heightmap::chunk_data &edit(bool const Shape, bool const Paint);
	// puts provided data in place of the current one; Grass: the grass changed
	void replace(heightmap::chunk_ptr Data, bool const Shape, bool const Paint, bool const Grass = false);
	// tells this chunk from any other, also from the same chunk loaded again
	std::uint64_t serial() const { return m_serial; }
	// changes with every change of what the grass of the chunk is made of: the ground, the holes and the grass itself
	std::uint64_t grass_version() const { return m_shapeversion + m_paintversion + m_grassversion; }

	heightmap::chunk_key key() const { return m_data->key; }
	// true if (X,Z) lies within the chunk
	bool contains(double X, double Z) const { return m_data->contains(X, Z); }
	// surface height at (X,Z), the way the full grid is triangulated
	double height_at(double X, double Z) const { return m_data->height_at(X, Z); }
	// the same, from heights taken from heights() earlier
	double height_in(std::vector<float> const &Heights, double X, double Z) const { return m_data->height_in(Heights, X, Z); }
	std::vector<float> const &heights() const { return m_data->heights; }
	int cells() const { return m_data->cells; }
	glm::dvec3 centre() const;
	float extent() const { return static_cast<float>(heightmap::chunk_size); }
	bool valid() const { return m_data != nullptr && m_data->valid(); }
	gfx::geometry_handle geometry() const { return m_geometry; }
	// rendered triangle count, and the count of the full grid
	std::size_t triangles() const { return m_triangles; }
	std::size_t full_triangles() const { return static_cast<std::size_t>(cells()) * cells() * 2; }
	// changed since it was loaded or saved
	bool modified() const { return m_modified; }
	void clear_modified() { m_modified = false; }

  private:
	friend class terrain_streamer;

	heightmap::chunk_ptr m_data;
	bool m_modified{false};
	std::uint64_t m_shapeversion{1}; // grows with every change of the heights
	std::uint64_t m_paintversion{1}; // grows with every change of the paint
	std::uint64_t m_grassversion{1}; // grows with every change of the grass
	std::uint64_t m_serial{0};

	// drawn state, managed by the streamer
	int m_slot{-1}; // geometry slot holding the mesh
	gfx::geometry_handle m_geometry{0, 0};
	std::size_t m_triangles{0};
	int m_stride{0}; // of the mesh in the slot, 0: none yet
	std::uint64_t m_builtversion{0}; // shape version the mesh was made from
	std::uint64_t m_paintedversion{0}; // paint version the material was made from
	bool m_neighbours{false}; // a neighbour came or changed, the edges of the mesh are to be made again
	bool m_building{false}; // a mesh is being made by a worker
	material_handle m_material{null_handle};
	scene::basic_section *m_section{nullptr}; // holding the shape, null: not drawn
	glm::dvec3 m_low{0.0}, m_high{0.0}; // bounds of the drawn mesh
};
