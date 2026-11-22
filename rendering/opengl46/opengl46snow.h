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
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <glm/glm.hpp>

#include "gl/query.h"
#include "gl/shader.h"
#include "gl/vao.h"

class opengl46_sky_occlusion;

// the snow lying around the camera for the opengl 4.6 renderer (environment/snowcover.h, Global.gfx_snow): the drifts are worked
// out on the gpu from the surface of the scenery the maps of the occlusion of the sky hold (snow_drift.comp, then
// snow_settle.comp) into a buffer of cells which every pass reads (gl::SNOW_SSBO_BINDING, snow.glsl): the shading puts the snow on
// the surfaces by it, and the surface of the drifts is drawn over the ground with it (snow_surface.*). without the maps (gfx.skyvisibility
// no) the snow lies evenly on the surfaces turned up, and there are no drifts.
// what the drifts are made of changes slowly: they're worked out again only when the maps, the snow (its depth, the drifting and
// its wind) or the place of the camera changed, and then a strip of the maps a frame, all of them in SNOW_STRIPS frames. the
// surface is drawn by blocks of the grid, the ones out of the view left out
class opengl46_snow
{
  public:
	opengl46_snow() = default;
	~opengl46_snow();
	opengl46_snow(opengl46_snow const &) = delete;
	opengl46_snow &operator=(opengl46_snow const &) = delete;

	// the programs and the stand-in of the cells
	void init();
	// the drifts, from the maps as they are after their update and the weather block of the frame, when they changed
	void update(opengl46_sky_occlusion const &Maps, glm::dvec3 const &Camera);
	// the cells (or the stand-in) bound for the passes
	void bind() const;
	// whether the surface of the drifts is to be drawn in this frame
	bool drawn() const;
	// draws the surface of the drifts seen through Projection; Modelview hands the modelview of the pass (View, the camera at the
	// origin) to the model block. returns the triangles at most
	std::size_t render(glm::mat4 const &View, glm::mat4 const &Projection, std::function<void(glm::mat4 const &)> const &Modelview);
	// line for the statistics of the debug panel, with the gpu times
	std::string info();

  private:
	// what the drifts are worked out of, besides the maps: the snow and the wind drifting it (the weather block of the frame)
	struct snow_key
	{
		float depth{-1.f};
		float drift{0.f};
		glm::vec2 wind{0.f}; // where the drifting wind blows to, m/s
	};

	void release();
	bool allocate(int const Side);
	// whether the drifts are to be worked out again for the maps of Revision, the camera at the texel Camera and the snow Key
	bool changed(std::uint32_t const Revision, glm::ivec2 const &Camera, snow_key const &Key) const;
	// the drifts of the rows from First, Rows of them (all of the maps: 0 and m_side)
	void work_out(opengl46_sky_occlusion const &Maps, glm::ivec2 const &Camera, int const First, int const Rows);

	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used
	std::unique_ptr<gl::program> m_drift;
	std::unique_ptr<gl::program> m_settle;
	std::unique_ptr<gl::program> m_surface;
	std::unique_ptr<gl::vao> m_vao; // without attributes, the grid is made of the numbers of its vertices
	GLuint m_cells{0}; // the buffer of the cells, four floats each
	GLuint m_standin{0}; // a single cell of nothing
	GLuint m_drifted{0}; // RG32F, what snow_drift.comp works out
	int m_side{0};
	bool m_active{false}; // the cells hold the drifts
	int m_across{0}; // vertices across the grid of the surface
	float m_texel{0.f}; // m of a texel of the maps
	// what the drifts in the cells were worked out of, and the strips of the maps still to be worked out for it (the next one first)
	bool m_valid{false};
	std::uint32_t m_revision{0};
	glm::ivec2 m_camera{0};
	snow_key m_key;
	int m_strip{0};
	int m_strips_left{0};
	// the blocks of the grid drawn in a frame
	std::vector<GLint> m_firsts;
	std::vector<GLsizei> m_counts;
	gl::gpu_timer m_worktimer;
	gl::gpu_timer m_drawtimer;
};
