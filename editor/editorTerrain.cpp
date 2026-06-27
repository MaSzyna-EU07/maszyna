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

#include "scene/scene.h"
#include "scene/scenenode.h"
#include "simulation/simulation.h"
#include "rendering/renderer.h"
#include "model/vertex.h"

#include <glad/glad.h>
#include <algorithm>
#include <cmath>

namespace
{
	constexpr double kPi = 3.14159265358979323846;
}

bool editor_terrain::create(glm::dvec3 const &Center, int Cells, float CellSize, std::string const &TextureName,
                            height_sampler const &Sampler)
{
	if (Cells < 1 || CellSize <= 0.0f || simulation::Region == nullptr)
		return false;

	m_cells = Cells;
	m_cellsize = CellSize;
	double const half = 0.5 * static_cast<double>(Cells) * CellSize;
	m_x0 = Center.x - half;
	m_z0 = Center.z - half;
	m_heights.assign(static_cast<std::size_t>(Cells + 1) * (Cells + 1), static_cast<float>(Center.y));

	// optionally seed the grid by sampling whatever geometry is already there (terrain capture)
	if (Sampler)
	{
		for (int iz = 0; iz <= Cells; ++iz)
			for (int ix = 0; ix <= Cells; ++ix)
			{
				double const vx = m_x0 + static_cast<double>(ix) * CellSize;
				double const vz = m_z0 + static_cast<double>(iz) * CellSize;
				double y;
				if (Sampler(vx, vz, y))
					m_heights[index(ix, iz)] = static_cast<float>(y);
			}
	}

	m_material = TextureName.empty() ? null_handle : GfxRenderer->Fetch_Material(TextureName);

	// section-level shapes are rendered relative to the section centre, so that is our geometry origin
	scene::basic_section &sec = simulation::Region->section(Center);
	sec.create_geometry(); // ensure existing section geometry is already built (idempotent)
	m_origin = sec.m_area.center;
	m_section = &sec;

	std::vector<world_vertex> verts;
	build_vertices(verts, false);
	m_vertexcount = verts.size();

	scene::shape_node shape;
	shape.make_terrain(m_material, std::move(verts), m_origin);

	// upload to a dedicated bank; the renderer resolves draw calls by handle regardless of bank
	m_bank = GfxRenderer->Create_Bank();
	shape.create_geometry(m_bank); // sets the shape's geometry handle, clears its CPU vertices
	m_geometry = shape.data().geometry;

	glm::dvec3 const shapecenter = shape.data().area.center;
	float const shaperadius = shape.radius(); // cached inside make_terrain, vertices already gone

	sec.m_shapes.emplace_back(std::move(shape));
	// extend the section bounds so the new terrain isn't frustum-culled at its edges
	sec.m_area.radius = std::max(
	    sec.m_area.radius,
	    static_cast<float>(glm::length(sec.m_area.center - shapecenter) + shaperadius));

	return true;
}

glm::dvec3 editor_terrain::vertex_position(int Ix, int Iz) const
{
	return glm::dvec3(
	    m_x0 + static_cast<double>(Ix) * m_cellsize,
	    static_cast<double>(m_heights[index(Ix, Iz)]),
	    m_z0 + static_cast<double>(Iz) * m_cellsize);
}

glm::vec3 editor_terrain::vertex_normal(int Ix, int Iz) const
{
	// central differences on the heightfield; clamp to edges
	int const xl = std::max(0, Ix - 1), xr = std::min(m_cells, Ix + 1);
	int const zl = std::max(0, Iz - 1), zr = std::min(m_cells, Iz + 1);
	float const hl = m_heights[index(xl, Iz)], hr = m_heights[index(xr, Iz)];
	float const hd = m_heights[index(Ix, zl)], hu = m_heights[index(Ix, zr)];
	float const dx = static_cast<float>((xr - xl)) * m_cellsize;
	float const dz = static_cast<float>((zr - zl)) * m_cellsize;
	glm::vec3 n(-(hr - hl) / (dx > 0.f ? dx : 1.f), 1.0f, -(hu - hd) / (dz > 0.f ? dz : 1.f));
	return glm::normalize(n);
}

world_vertex editor_terrain::make_vertex(int Ix, int Iz) const
{
	world_vertex v;
	v.position = vertex_position(Ix, Iz);
	v.normal = vertex_normal(Ix, Iz);
	v.texture = glm::vec2(static_cast<float>(Ix), static_cast<float>(Iz));
	return v;
}

// emits one quad (two upward-facing triangles) spanning grid corners (X0,Z0)..(X1,Z1)
void editor_terrain::emit_quad(int X0, int Z0, int X1, int Z1, std::vector<world_vertex> &Out) const
{
	world_vertex const v00 = make_vertex(X0, Z0);
	world_vertex const v10 = make_vertex(X1, Z0);
	world_vertex const v01 = make_vertex(X0, Z1);
	world_vertex const v11 = make_vertex(X1, Z1);

	Out.push_back(v00);
	Out.push_back(v01);
	Out.push_back(v10);

	Out.push_back(v11);
	Out.push_back(v10);
	Out.push_back(v01);
}

// true if every grid vertex inside the block stays within Error of the bilinear plane of its corners
bool editor_terrain::block_flat(int X0, int Z0, int X1, int Z1, float Error) const
{
	float const h00 = m_heights[index(X0, Z0)];
	float const h10 = m_heights[index(X1, Z0)];
	float const h01 = m_heights[index(X0, Z1)];
	float const h11 = m_heights[index(X1, Z1)];
	double const wx = X1 - X0, wz = Z1 - Z0;

	for (int iz = Z0; iz <= Z1; ++iz)
		for (int ix = X0; ix <= X1; ++ix)
		{
			double const tx = (wx > 0.0) ? (ix - X0) / wx : 0.0;
			double const tz = (wz > 0.0) ? (iz - Z0) / wz : 0.0;
			double const top = h00 + tx * (h10 - h00);
			double const bot = h01 + tx * (h11 - h01);
			double const interp = top + tz * (bot - top);
			if (std::abs(static_cast<double>(m_heights[index(ix, iz)]) - interp) > Error)
				return false;
		}
	return true;
}

// adaptive quadtree: collapse flat blocks into a single quad, otherwise split into four
void editor_terrain::emit_block(int X0, int Z0, int X1, int Z1, float Error, std::vector<world_vertex> &Out) const
{
	bool const splitx = (X1 - X0) > 1;
	bool const splitz = (Z1 - Z0) > 1;

	if ((!splitx && !splitz) || block_flat(X0, Z0, X1, Z1, Error))
	{
		emit_quad(X0, Z0, X1, Z1, Out);
		return;
	}

	int const xm = splitx ? (X0 + X1) / 2 : X1;
	int const zm = splitz ? (Z0 + Z1) / 2 : Z1;

	emit_block(X0, Z0, xm, zm, Error, Out);
	if (splitx)
		emit_block(xm, Z0, X1, zm, Error, Out);
	if (splitz)
		emit_block(X0, zm, xm, Z1, Error, Out);
	if (splitx && splitz)
		emit_block(xm, zm, X1, Z1, Error, Out);
}

void editor_terrain::build_vertices(std::vector<world_vertex> &Out, bool Simplify) const
{
	Out.clear();
	Out.reserve(static_cast<std::size_t>(m_cells) * m_cells * 6);

	if (Simplify)
	{
		emit_block(0, 0, m_cells, m_cells, m_simplify_error, Out);
		return;
	}

	for (int iz = 0; iz < m_cells; ++iz)
		for (int ix = 0; ix < m_cells; ++ix)
			emit_quad(ix, iz, ix + 1, iz + 1, Out);
}

void editor_terrain::regenerate(bool Simplify)
{
	if (!valid())
		return;

	std::vector<world_vertex> verts;
	build_vertices(verts, Simplify);

	gfx::vertex_array gpuverts;
	gpuverts.reserve(verts.size());
	for (auto const &v : verts)
		gpuverts.emplace_back(gfx::basic_vertex::convert(v, m_origin));
	gfx::userdata_array nouserdata;

	// fast path: same vertex count -> in-place swap into the existing chunk
	if (gpuverts.size() == m_vertexcount && (m_geometry.bank != 0 || m_geometry.chunk != 0))
	{
		GfxRenderer->Replace(gpuverts, nouserdata, m_geometry, GL_TRIANGLES);
		return;
	}

	// count changed (optimize / un-optimize): upload a fresh chunk and point the shape at it
	gfx::geometry_handle const newhandle = GfxRenderer->Insert(gpuverts, nouserdata, m_bank, GL_TRIANGLES);
	if (m_section != nullptr)
	{
		for (auto &shape : m_section->m_shapes)
		{
			auto const h = shape.data().geometry;
			if (h.bank == m_geometry.bank && h.chunk == m_geometry.chunk)
			{
				shape.geometry(newhandle);
				break;
			}
		}
	}
	m_geometry = newhandle;
	m_vertexcount = gpuverts.size();
}

void editor_terrain::optimize(float ErrorMetres)
{
	m_simplify = true;
	m_simplify_error = (ErrorMetres > 0.0f ? ErrorMetres : 0.01f);
	m_dirty = false;
	regenerate(true);
}

void editor_terrain::unoptimize()
{
	m_simplify = false;
	regenerate(false);
}

void editor_terrain::destroy()
{
	if (m_section != nullptr)
	{
		// erase the shape whose geometry handle is ours; other shapes keep their handles (so they
		// keep rendering), and the geometry GC reclaims our now-undrawn chunk's GPU memory
		for (auto it = m_section->m_shapes.begin(); it != m_section->m_shapes.end(); ++it)
		{
			auto const h = it->data().geometry;
			if (h.bank == m_geometry.bank && h.chunk == m_geometry.chunk)
			{
				m_section->m_shapes.erase(it);
				break;
			}
		}
	}
	m_section = nullptr;
	m_geometry = gfx::geometry_handle{0, 0};
	m_cells = 0; // mark invalid
	m_heights.clear();
}

bool editor_terrain::contains(double X, double Z) const
{
	double const x1 = m_x0 + static_cast<double>(m_cells) * m_cellsize;
	double const z1 = m_z0 + static_cast<double>(m_cells) * m_cellsize;
	return (X >= m_x0 && X <= x1 && Z >= m_z0 && Z <= z1);
}

double editor_terrain::height_at(double X, double Z) const
{
	double const fx = (X - m_x0) / m_cellsize;
	double const fz = (Z - m_z0) / m_cellsize;
	int ix = static_cast<int>(std::floor(fx));
	int iz = static_cast<int>(std::floor(fz));
	ix = std::clamp(ix, 0, m_cells - 1);
	iz = std::clamp(iz, 0, m_cells - 1);
	double const tx = std::clamp(fx - ix, 0.0, 1.0);
	double const tz = std::clamp(fz - iz, 0.0, 1.0);

	double const h00 = m_heights[index(ix, iz)];
	double const h10 = m_heights[index(ix + 1, iz)];
	double const h01 = m_heights[index(ix, iz + 1)];
	double const h11 = m_heights[index(ix + 1, iz + 1)];

	// matches the triangulation in build_vertices
	if (tx + tz <= 1.0)
		return h00 + tx * (h10 - h00) + tz * (h01 - h00);
	return h11 + (1.0 - tx) * (h01 - h11) + (1.0 - tz) * (h10 - h11);
}

bool editor_terrain::sculpt(double X, double Z, double Radius, double Strength)
{
	if (!valid() || Radius <= 0.0)
		return false;

	bool changed = false;
	for (int iz = 0; iz <= m_cells; ++iz)
		for (int ix = 0; ix <= m_cells; ++ix)
		{
			double const vx = m_x0 + static_cast<double>(ix) * m_cellsize;
			double const vz = m_z0 + static_cast<double>(iz) * m_cellsize;
			double const d = std::sqrt((vx - X) * (vx - X) + (vz - Z) * (vz - Z));
			if (d > Radius)
				continue;
			// smooth cosine falloff: full strength at the centre, zero at the rim
			double const falloff = 0.5 * (std::cos(kPi * d / Radius) + 1.0);
			m_heights[index(ix, iz)] += static_cast<float>(Strength * falloff);
			changed = true;
		}

	if (changed)
	{
		// sculpting edits the full-resolution mesh (fixed vertex count => fast in-place update);
		// mark dirty so it can be auto-simplified once the stroke finishes, and modified for saving
		m_simplify = false;
		m_dirty = true;
		m_modified = true;
		regenerate(false);
	}
	return changed;
}

glm::dvec3 editor_terrain::centre() const
{
	double const c = 0.5 * static_cast<double>(m_cells) * m_cellsize;
	double y = 0.0;
	if (!m_heights.empty())
		y = m_heights[index(m_cells / 2, m_cells / 2)];
	return glm::dvec3(m_x0 + c, y, m_z0 + c);
}
