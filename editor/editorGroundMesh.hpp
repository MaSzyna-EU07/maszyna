/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>
#include <glm/glm.hpp>

// triangles of a single piece of the scenery - a shape or a model - sorted on a grid seen from above, so the ones
// overlapping an area can be picked without going through all of them.
// the editor asks about the ground a tile at a time, frame after frame, and a terrain model can hold a million
// triangles: walking it whole for every question is what used to bring the simulator down to a frame a second
class ground_mesh
{
  public:
	using triangle = std::array<glm::vec3, 3>; // relative to the origin of the mesh
	using world_triangle = std::array<glm::dvec3, 3>;

	// takes over provided triangles. Origin: the point their vertices are relative to
	void assign(std::vector<triangle> Triangles, glm::dvec3 const &Origin)
	{
		m_triangles = std::move(Triangles);
		m_origin = Origin;
		m_starts.clear();
		m_items.clear();
		m_seen.assign(m_triangles.size(), 0);
		m_query = 0;
		if (m_triangles.empty())
			return;

		glm::vec2 low{m_triangles.front()[0].x, m_triangles.front()[0].z};
		glm::vec2 high{low};
		for (auto const &entry : m_triangles)
		{
			for (auto const &vertex : entry)
			{
				low = glm::min(low, glm::vec2{vertex.x, vertex.z});
				high = glm::max(high, glm::vec2{vertex.x, vertex.z});
			}
		}
		m_low = low;
		auto const extent{glm::max(high - low, glm::vec2{1.f})};
		// cells of a few metres, larger for a large mesh so the grid stays small
		m_cellsize = std::max(8.f, std::max(extent.x, extent.y) / 96.f);
		m_columns = static_cast<int>(extent.x / m_cellsize) + 1;
		m_rows = static_cast<int>(extent.y / m_cellsize) + 1;

		// a triangle is listed in every cell its bounds reach. the lists are kept one after another, with the place each starts at
		std::vector<std::uint32_t> counts(static_cast<std::size_t>(m_columns) * m_rows + 1, 0);
		for (auto const &entry : m_triangles)
		{
			auto const span{cells(entry)};
			for (int row = span[1]; row <= span[3]; ++row)
				for (int column = span[0]; column <= span[2]; ++column)
					++counts[static_cast<std::size_t>(row) * m_columns + column + 1];
		}
		for (std::size_t cell = 1; cell < counts.size(); ++cell)
			counts[cell] += counts[cell - 1];
		m_starts = counts;
		m_items.resize(m_starts.back());
		for (std::uint32_t index = 0; index < m_triangles.size(); ++index)
		{
			auto const span{cells(m_triangles[index])};
			for (int row = span[1]; row <= span[3]; ++row)
				for (int column = span[0]; column <= span[2]; ++column)
					m_items[counts[static_cast<std::size_t>(row) * m_columns + column]++] = index;
		}
	}
	std::size_t size() const
	{
		return m_triangles.size();
	}
	// appends the triangles whose bounds overlap specified rectangle of the world, seen from above (x, z)
	void collect(glm::dvec2 const &Min, glm::dvec2 const &Max, std::vector<world_triangle> &Out) const
	{
		if (m_triangles.empty())
			return;
		// the rectangle as the mesh sees it, widened by what's lost turning the numbers into single precision
		glm::vec2 const low{static_cast<float>(Min.x - m_origin.x) - 0.001f, static_cast<float>(Min.y - m_origin.z) - 0.001f};
		glm::vec2 const high{static_cast<float>(Max.x - m_origin.x) + 0.001f, static_cast<float>(Max.y - m_origin.z) + 0.001f};
		auto const firstcolumn{column(low.x)};
		auto const lastcolumn{column(high.x)};
		auto const firstrow{row(low.y)};
		auto const lastrow{row(high.y)};
		if (++m_query == 0)
		{
			// the counter went all the way round
			std::fill(m_seen.begin(), m_seen.end(), 0);
			m_query = 1;
		}
		for (int row = firstrow; row <= lastrow; ++row)
		{
			for (int column = firstcolumn; column <= lastcolumn; ++column)
			{
				auto const cell{static_cast<std::size_t>(row) * m_columns + column};
				for (auto item = m_starts[cell]; item < m_starts[cell + 1]; ++item)
				{
					auto const index{m_items[item]};
					if (m_seen[index] == m_query)
						continue;
					m_seen[index] = m_query;
					auto const &entry{m_triangles[index]};
					if (std::max({entry[0].x, entry[1].x, entry[2].x}) < low.x || std::min({entry[0].x, entry[1].x, entry[2].x}) > high.x || std::max({entry[0].z, entry[1].z, entry[2].z}) < low.y ||
					    std::min({entry[0].z, entry[1].z, entry[2].z}) > high.y)
						continue;
					Out.push_back({m_origin + glm::dvec3{entry[0]}, m_origin + glm::dvec3{entry[1]}, m_origin + glm::dvec3{entry[2]}});
				}
			}
		}
	}

  private:
	int column(float const X) const
	{
		return std::clamp(static_cast<int>(std::floor((X - m_low.x) / m_cellsize)), 0, m_columns - 1);
	}
	int row(float const Z) const
	{
		return std::clamp(static_cast<int>(std::floor((Z - m_low.y) / m_cellsize)), 0, m_rows - 1);
	}
	// cells the bounds of a triangle reach: first column, first row, last column, last row
	std::array<int, 4> cells(triangle const &Triangle) const
	{
		return {column(std::min({Triangle[0].x, Triangle[1].x, Triangle[2].x})), row(std::min({Triangle[0].z, Triangle[1].z, Triangle[2].z})), column(std::max({Triangle[0].x, Triangle[1].x, Triangle[2].x})),
		        row(std::max({Triangle[0].z, Triangle[1].z, Triangle[2].z}))};
	}

	std::vector<triangle> m_triangles;
	glm::dvec3 m_origin{0.0};
	glm::vec2 m_low{0.f};
	float m_cellsize{8.f};
	int m_columns{1};
	int m_rows{1};
	std::vector<std::uint32_t> m_starts; // for each cell: where its list starts; one more entry closes the last list
	std::vector<std::uint32_t> m_items; // numbers of the triangles, cell after cell
	mutable std::vector<std::uint32_t> m_seen; // number of the last lookup each triangle was reported in
	mutable std::uint32_t m_query{0};
};
