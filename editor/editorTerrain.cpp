/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTerrain.hpp"

heightmap::chunk_data &editor_terrain::edit(bool const Shape, bool const Paint)
{
	// a worker may still be making a mesh of the current data, so a copy is changed instead
	if (m_data.use_count() > 1)
		m_data = std::make_shared<heightmap::chunk_data>(*m_data);
	m_modified = true;
	if (Shape)
		++m_shapeversion;
	if (Paint)
		++m_paintversion;
	// the pointer is to const for everyone else; this chunk is its only holder now
	return const_cast<heightmap::chunk_data &>(*m_data);
}

void editor_terrain::replace(heightmap::chunk_ptr Data, bool const Shape, bool const Paint)
{
	m_data = std::move(Data);
	m_modified = true;
	if (Shape)
		++m_shapeversion;
	if (Paint)
		++m_paintversion;
}

glm::dvec3 editor_terrain::centre() const
{
	auto const low{m_data->corner()};
	auto const half{heightmap::chunk_size * 0.5};
	return {low.x + half, m_data->height_at(low.x + half, low.y + half), low.y + half};
}
