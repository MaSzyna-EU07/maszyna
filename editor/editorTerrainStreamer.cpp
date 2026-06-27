/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/editorTerrainStreamer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

// simulation-level streamer instance (see header)
terrain_streamer EditorTerrain;

void terrain_streamer::configure(int Cells, float CellSize, int Radius, float BaseHeight, std::string const &Texture)
{
	m_cells = std::max(1, Cells);
	m_cellsize = std::max(0.1f, CellSize);
	m_radius = std::max(0, Radius);
	m_baseheight = BaseHeight;
	m_texture = Texture;
}

terrain_streamer::chunk_key terrain_streamer::key_at(double X, double Z) const
{
	double const size = chunk_world_size();
	return {static_cast<int>(std::floor(X / size)), static_cast<int>(std::floor(Z / size))};
}

glm::dvec3 terrain_streamer::chunk_centre(int Cx, int Cz) const
{
	double const size = chunk_world_size();
	return glm::dvec3((Cx + 0.5) * size, static_cast<double>(m_baseheight), (Cz + 0.5) * size);
}

std::string terrain_streamer::chunk_path(int Cx, int Cz) const
{
	return m_dir + "/chunk_" + std::to_string(Cx) + "_" + std::to_string(Cz) + ".etc";
}

bool terrain_streamer::chunk_on_disk(chunk_key const &Key)
{
	auto const it = m_known.find(Key);
	if (it != m_known.end())
		return (it->second & cf_exists_on_disk) != 0;

	std::error_code ec;
	bool const exists = std::filesystem::exists(chunk_path(Key.first, Key.second), ec);
	m_known[Key] = exists ? cf_exists_on_disk : 0;
	return exists;
}

// 16-bit chunk file: 'ETC1' | uint16 cells | float baseY | float step | (cells+1)^2 * uint16
// where worldY = baseY + raw * step (per-chunk auto-scaled to fit the height range losslessly-ish)
bool terrain_streamer::load_heights(int Cx, int Cz, std::vector<float> &Out) const
{
	std::ifstream f(chunk_path(Cx, Cz), std::ios::binary);
	if (!f)
		return false;

	char magic[4] = {};
	f.read(magic, 4);
	if (f.gcount() != 4 || magic[0] != 'E' || magic[1] != 'T' || magic[2] != 'C' || magic[3] != '1')
		return false;

	std::uint16_t cells = 0;
	float baseY = 0.0f, step = 0.0f;
	f.read(reinterpret_cast<char *>(&cells), sizeof(cells));
	f.read(reinterpret_cast<char *>(&baseY), sizeof(baseY));
	f.read(reinterpret_cast<char *>(&step), sizeof(step));
	if (!f || static_cast<int>(cells) != m_cells)
		return false; // resolution changed since save: ignore and regenerate

	std::size_t const n = static_cast<std::size_t>(cells + 1) * (cells + 1);
	Out.resize(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		std::uint16_t raw = 0;
		f.read(reinterpret_cast<char *>(&raw), sizeof(raw));
		if (!f)
			return false;
		Out[i] = baseY + static_cast<float>(raw) * step;
	}
	return true;
}

void terrain_streamer::save_heights(int Cx, int Cz, editor_terrain const &Terrain)
{
	auto const &h = Terrain.heights();
	if (h.empty())
		return;

	float minh = h[0], maxh = h[0];
	for (float const v : h)
	{
		minh = std::min(minh, v);
		maxh = std::max(maxh, v);
	}
	float const step = (maxh > minh) ? (maxh - minh) / 65535.0f : 0.0f;

	std::error_code ec;
	std::filesystem::create_directories(m_dir, ec);
	std::ofstream f(chunk_path(Cx, Cz), std::ios::binary | std::ios::trunc);
	if (!f)
		return;

	char const magic[4] = {'E', 'T', 'C', '1'};
	std::uint16_t const cells = static_cast<std::uint16_t>(Terrain.cells());
	f.write(magic, 4);
	f.write(reinterpret_cast<char const *>(&cells), sizeof(cells));
	f.write(reinterpret_cast<char const *>(&minh), sizeof(minh));
	f.write(reinterpret_cast<char const *>(&step), sizeof(step));
	for (float const v : h)
	{
		std::uint16_t const raw = (step > 0.0f)
		                              ? static_cast<std::uint16_t>(std::lround((v - minh) / step))
		                              : std::uint16_t{0};
		f.write(reinterpret_cast<char const *>(&raw), sizeof(raw));
	}
}

void terrain_streamer::update(glm::dvec3 const &CameraPos)
{
	if (!m_active)
		return;

	chunk_key const camera = key_at(CameraPos.x, CameraPos.z);

	// load any missing chunk inside the radius (one build per frame keeps the hitch small)
	bool built = false;
	for (int dz = -m_radius; dz <= m_radius && !built; ++dz)
		for (int dx = -m_radius; dx <= m_radius && !built; ++dx)
		{
			chunk_key const key{camera.first + dx, camera.second + dz};
			if (m_chunks.count(key))
				continue;

			// only stream chunks that were actually authored (exist on disk); empty space stays empty
			std::vector<float> loaded;
			if (!chunk_on_disk(key) || !load_heights(key.first, key.second, loaded))
				continue;

			glm::dvec3 const centre = chunk_centre(key.first, key.second);
			double const size = chunk_world_size();
			double const x0 = key.first * size, z0 = key.second * size; // chunk corner
			int const cells = m_cells;
			float const cs = m_cellsize;
			editor_terrain::height_sampler sampler =
			    [&loaded, x0, z0, cells, cs](double X, double Z, double &OutY) -> bool {
				int ix = static_cast<int>(std::lround((X - x0) / cs));
				int iz = static_cast<int>(std::lround((Z - z0) / cs));
				ix = std::clamp(ix, 0, cells);
				iz = std::clamp(iz, 0, cells);
				OutY = loaded[static_cast<std::size_t>(iz) * (cells + 1) + ix];
				return true;
			};

			auto terrain = std::make_unique<editor_terrain>();
			if (terrain->create(centre, m_cells, m_cellsize, m_texture, sampler))
			{
				if (m_auto_optimize)
					terrain->optimize(m_simplify_error);
				m_chunks.emplace(key, std::move(terrain));
				built = true; // amortise: at most one new chunk per frame
			}
		}

	// unload chunks that drifted outside the radius
	for (auto it = m_chunks.begin(); it != m_chunks.end();)
	{
		int const dx = it->first.first - camera.first;
		int const dz = it->first.second - camera.second;
		if (std::abs(dx) > m_radius || std::abs(dz) > m_radius)
		{
			if (it->second)
			{
				// persist edits before dropping the chunk, so they survive the round-trip
				if (m_persist && it->second->modified())
				{
					save_heights(it->first.first, it->first.second, *it->second);
					m_known[it->first] |= cf_exists_on_disk;
				}
				it->second->destroy();
			}
			it = m_chunks.erase(it);
		}
		else
			++it;
	}
}

void terrain_streamer::clear()
{
	for (auto &entry : m_chunks)
		if (entry.second)
		{
			if (m_persist && entry.second->modified())
			{
				save_heights(entry.first.first, entry.first.second, *entry.second);
				m_known[entry.first] |= cf_exists_on_disk;
			}
			entry.second->destroy();
		}
	m_chunks.clear();
}

void terrain_streamer::collect(std::vector<editor_terrain *> &Out) const
{
	for (auto const &entry : m_chunks)
		if (entry.second)
			Out.push_back(entry.second.get());
}

editor_terrain *terrain_streamer::terrain_at(double X, double Z) const
{
	auto const it = m_chunks.find(key_at(X, Z));
	if (it != m_chunks.end() && it->second && it->second->contains(X, Z))
		return it->second.get();
	return nullptr;
}

void terrain_streamer::save_chunk(int Cx, int Cz, editor_terrain const &Terrain)
{
	save_heights(Cx, Cz, Terrain);
	m_known[{Cx, Cz}] |= cf_exists_on_disk;
}

void terrain_streamer::add_chunk(int Cx, int Cz)
{
	chunk_key const key{Cx, Cz};
	if (m_chunks.count(key))
		return; // already resident

	glm::dvec3 const centre = chunk_centre(Cx, Cz);
	auto terrain = std::make_unique<editor_terrain>();
	if (!terrain->create(centre, m_cells, m_cellsize, m_texture)) // flat
		return;

	// persist immediately so it becomes part of the authored, streamable world
	save_heights(Cx, Cz, *terrain);
	m_known[key] |= cf_exists_on_disk;

	if (m_auto_optimize)
		terrain->optimize(m_simplify_error);
	m_chunks.emplace(key, std::move(terrain));
}

void terrain_streamer::reset()
{
	// do NOT call editor_terrain::destroy() here: on a scenery reload the sections those chunks
	// referenced are already gone. just drop our bookkeeping; the old region freed the shapes.
	m_chunks.clear();
	m_known.clear();
	m_active = false;
}

void terrain_streamer::flush()
{
	for (auto &entry : m_chunks)
		if (entry.second && entry.second->modified())
		{
			save_heights(entry.first.first, entry.first.second, *entry.second);
			m_known[entry.first] |= cf_exists_on_disk;
			entry.second->clear_modified();
		}
}

void terrain_streamer::remove_chunk(int Cx, int Cz)
{
	chunk_key const key{Cx, Cz};
	auto const it = m_chunks.find(key);
	if (it != m_chunks.end())
	{
		if (it->second)
			it->second->destroy();
		m_chunks.erase(it);
	}
	std::error_code ec;
	std::filesystem::remove(chunk_path(Cx, Cz), ec);
	m_known[key] = 0; // no longer on disk
}
