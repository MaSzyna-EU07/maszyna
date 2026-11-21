/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <functional>
#include <memory>
#include <string>

#include <glm/glm.hpp>

#include "gl/shader.h"
#include "gl/vao.h"

class opengl46_sky_occlusion;

// the snow lying around the camera for the opengl 4.6 renderer (environment/snowcover.h, Global.gfx_snow): every frame the drifts are
// worked out on the gpu from the surface of the scenery the maps of the occlusion of the sky hold (snow_drift.comp, then
// snow_settle.comp) into a buffer of cells which every pass reads (gl::SNOW_SSBO_BINDING, snow.glsl): the shading puts the snow on
// the surfaces by it, and the surface of the drifts is drawn over the ground with it (snow_surface.*). without the maps (gfx.skyvisibility
// no) the snow lies evenly on the surfaces turned up, and there are no drifts
class opengl46_snow
{
  public:
	opengl46_snow() = default;
	~opengl46_snow();
	opengl46_snow(opengl46_snow const &) = delete;
	opengl46_snow &operator=(opengl46_snow const &) = delete;

	// the programs and the stand-in of the cells
	void init();
	// the drifts of the frame, from the maps as they are after their update and the weather block of the frame
	void update(opengl46_sky_occlusion const &Maps, glm::dvec3 const &Camera);
	// the cells (or the stand-in) bound for the passes
	void bind() const;
	// whether the surface of the drifts is to be drawn in this frame
	bool drawn() const;
	// draws the surface of the drifts; Modelview hands the modelview of the pass (the camera at the origin) to the model block.
	// returns the triangles at most
	std::size_t render(glm::mat4 const &View, std::function<void(glm::mat4 const &)> const &Modelview);
	// line for the statistics of the debug panel
	std::string info() const;

  private:
	void release();
	bool allocate(int const Side);

	int m_state{0}; // 0: not set up yet, 1: ready, -1: can't be used
	std::unique_ptr<gl::program> m_drift;
	std::unique_ptr<gl::program> m_settle;
	std::unique_ptr<gl::program> m_surface;
	std::unique_ptr<gl::vao> m_vao; // without attributes, the grid is made of the numbers of its vertices
	GLuint m_cells{0}; // the buffer of the cells, four floats each
	GLuint m_standin{0}; // a single cell of nothing
	GLuint m_drifted{0}; // RG32F, what snow_drift.comp works out
	int m_side{0};
	bool m_active{false}; // the cells hold the drifts of this frame
	int m_across{0}; // vertices across the grid of the surface
};
