/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <map>
#include <memory>
#include <vector>
#include <string>
#include <glm/glm.hpp>

#include "editor/editorTerrain.hpp"

// Streams editable terrain chunks around the camera for an effectively unbounded world.
//
// Phase 1: keeps a ring of resident chunks (each an editor_terrain mesh) within a radius of the
// camera; chunks entering the radius are built, chunks leaving it are destroyed (the renderer's
// geometry GC reclaims their GPU memory). Heights are flat + a gentle procedural roll so streaming
// is observable. Phase 2 will add 16-bit on-disk chunk paging, per-chunk flag bits and persistence.
class terrain_streamer
{
  public:
	using chunk_key = std::pair<int, int>; // (cx, cz) chunk coordinate

	terrain_streamer() = default;

	void active(const bool State) { m_active = State; }
	bool active() const { return m_active; }

	// per-chunk configuration (applied to chunks built from now on)
	void configure(int Cells, float CellSize, int Radius, float BaseHeight, std::string const &Texture);
	void simplify(const bool Auto, const float Error) { m_auto_optimize = Auto; m_simplify_error = Error; }
	// safe to change while resident (only affects how far chunks load/unload)
	void radius(const int Radius) { m_radius = Radius < 0 ? 0 : Radius; }
	// persist edited chunks to disk on unload, and load them back instead of regenerating
	void persist(const bool Enable) { m_persist = Enable; }
	void directory(std::string const &Dir)
	{
		if (!Dir.empty() && Dir != m_dir)
		{
			m_dir = Dir;
			m_known.clear(); // existence cache is per-folder; drop it when the folder changes
		}
	}
	std::string const &directory() const { return m_dir; }

	// loads/unloads chunks so the resident set matches the radius around CameraPos
	void update(glm::dvec3 const &CameraPos);
	// drops all resident chunks (e.g. when streaming is switched off), saving edits first
	void clear();
	// hard reset used when a new scenery loads: drops everything WITHOUT touching scene sections
	// (the old region is being torn down, so its section pointers are already dangling)
	void reset();

	// appends raw pointers to every resident chunk (for sculpt / snap / optimize routing)
	void collect(std::vector<editor_terrain *> &Out) const;
	// resident chunk whose footprint covers (X,Z), or nullptr
	editor_terrain *terrain_at(double X, double Z) const;

	// authoring: create a flat chunk at (Cx,Cz), make it resident and persist it to disk
	void add_chunk(int Cx, int Cz);
	// authoring: delete the chunk at (Cx,Cz) from memory and disk
	void remove_chunk(int Cx, int Cz);
	// persist an externally-authored chunk (e.g. a manual grid chunk being handed over to streaming)
	void save_chunk(int Cx, int Cz, editor_terrain const &Terrain);
	// chunk coordinate covering (X,Z)
	chunk_key key_for(const double X, const double Z) const { return key_at(X, Z); }

	std::size_t resident() const { return m_chunks.size(); }
	float chunk_world_size() const { return m_cells * m_cellsize; }
	int cells() const { return m_cells; }
	float cellsize() const { return m_cellsize; }
	int radius() const { return m_radius; }

	// writes every resident, edited chunk to disk (without unloading) - used on save
	void flush();

  private:
	// per-chunk state bits cached so we don't stat the filesystem repeatedly
	enum chunk_flag : uint8_t
	{
		cf_exists_on_disk = 1u << 0, // a saved 16-bit chunk file exists
	};

	chunk_key key_at(double X, double Z) const;
	glm::dvec3 chunk_centre(int Cx, int Cz) const;
	std::string chunk_path(int Cx, int Cz) const;
	bool chunk_on_disk(chunk_key const &Key);                          // cached existence test
	bool load_heights(int Cx, int Cz, std::vector<float> &Out) const;  // reads a 16-bit chunk file
	void save_heights(int Cx, int Cz, editor_terrain const &Terrain);  // writes a 16-bit chunk file

	bool m_active{false};
	int m_cells{32};
	float m_cellsize{2.0f};
	int m_radius{2};            // chunks loaded around the camera (Chebyshev radius)
	float m_baseheight{0.0f};
	std::string m_texture;
	bool m_auto_optimize{true};
	float m_simplify_error{0.5f};
	bool m_persist{true};
	std::string m_dir{"editor_terrain"}; // folder for 16-bit chunk files

	std::map<chunk_key, std::unique_ptr<editor_terrain>> m_chunks;
	std::map<chunk_key, std::uint8_t> m_known; // cached per-chunk flag bits
};

// single, simulation-level streamer instance shared by the editor (authoring) and the scenery
// loader (so streamed terrain renders in every mode, including the driver). loaded from the
// `editorterrain` scenery directive and updated each frame with the active camera.
extern terrain_streamer EditorTerrain;
