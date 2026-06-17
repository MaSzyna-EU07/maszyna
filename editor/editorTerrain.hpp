/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <vector>
#include <string>
#include <functional>
#include <glm/glm.hpp>

#include "utilities/Classes.h"      // material_handle
#include "interfaces/ITexture.h"    // null_handle
#include "rendering/geometrybank.h" // gfx::geometry_handle / geometrybank_handle

namespace scene { class basic_section; }

// Editor-owned, editable terrain patch.
//
// The engine's scene::shape_node drops its CPU-side vertices the moment it uploads them to the GPU,
// so it can't be edited or raycast after load. This class keeps the authoritative, editable data
// (a regular grid heightmap) on the CPU, generates a shape_node purely for rendering, and answers
// height/raycast queries directly from the heightmap (fast and exact). Sculpting updates the
// heightmap and pushes the new vertex positions into the shape's existing geometry chunk.
class editor_terrain
{
  public:
	editor_terrain() = default;

	// builds an NxN-cell grid centred on Center, each quad CellSize metres across, using the
	// material fetched from TextureName (empty => untextured). when Sampler is supplied it provides
	// the starting height at each grid vertex (returns false to fall back to Center.y) - used to
	// capture existing terrain. returns false on failure.
	using height_sampler = std::function<bool(double X, double Z, double &OutY)>;
	bool create(glm::dvec3 const &Center, int Cells, float CellSize, std::string const &TextureName,
	            height_sampler const &Sampler = {});

	// true if (X,Z) lies within the terrain's horizontal footprint
	bool contains(double X, double Z) const;
	// surface height at (X,Z) (bilinear over the covering quad); only valid when contains() is true
	double height_at(double X, double Z) const;

	// raises/lowers vertices within Radius of (X,Z) by Strength (metres, signed), with a smooth
	// falloff; regenerates the rendered geometry (full resolution). returns true if anything changed.
	bool sculpt(double X, double Z, double Radius, double Strength);

	// rebuilds the rendered mesh, collapsing regions flatter than ErrorMetres into larger quads
	// (adaptive quadtree). the editable heightmap is untouched, so sculpting/raycast stay exact.
	void optimize(float ErrorMetres);
	// rebuilds the rendered mesh at full resolution (undoes optimize)
	void unoptimize();

	// horizontal centre and extent, handy for the UI / camera framing
	glm::dvec3 centre() const;
	float extent() const { return m_cells * m_cellsize; }
	bool valid() const { return m_cells > 0; }
	bool optimized() const { return m_simplify; }
	// rendered triangle count (drops after optimize)
	std::size_t triangles() const { return m_vertexcount / 3; }
	// full-resolution triangle count, for reference
	std::size_t full_triangles() const { return static_cast<std::size_t>(m_cells) * m_cells * 2; }

  private:
	int index(int Ix, int Iz) const { return Iz * (m_cells + 1) + Ix; }
	glm::dvec3 vertex_position(int Ix, int Iz) const;
	glm::vec3 vertex_normal(int Ix, int Iz) const;
	world_vertex make_vertex(int Ix, int Iz) const;
	// fills Out with the GL_TRIANGLES world-space vertex list; Simplify enables adaptive merging
	void build_vertices(std::vector<world_vertex> &Out, bool Simplify) const;
	// adaptive quadtree helpers (used when Simplify is on)
	bool block_flat(int X0, int Z0, int X1, int Z1, float Error) const;
	void emit_block(int X0, int Z0, int X1, int Z1, float Error, std::vector<world_vertex> &Out) const;
	void emit_quad(int X0, int Z0, int X1, int Z1, std::vector<world_vertex> &Out) const;
	// rebuilds and re-uploads the rendered geometry (Replace when the count is unchanged, otherwise
	// a fresh chunk whose handle is swapped into the shape)
	void regenerate(bool Simplify);

	int m_cells{0};            // quads per side; (m_cells+1)^2 grid vertices
	float m_cellsize{1.0f};    // metres per quad
	double m_x0{0.0}, m_z0{0.0}; // world position of grid corner (ix=0, iz=0)
	std::vector<float> m_heights; // per-vertex world Y, row-major (m_cells+1)^2

	material_handle m_material{null_handle};
	gfx::geometrybank_handle m_bank{0, 0}; // geometry bank owning the rendered chunk
	gfx::geometry_handle m_geometry{0, 0}; // rendered chunk
	std::size_t m_vertexcount{0};          // current chunk's vertex count (for Replace vs recreate)
	glm::dvec3 m_origin{0.0};   // origin the GPU vertices are stored relative to
	scene::basic_section *m_section{nullptr}; // section holding the shape, for handle swaps

	bool m_simplify{false};        // whether the rendered mesh is currently simplified
	float m_simplify_error{0.5f};  // flatness tolerance used by optimize()
};
