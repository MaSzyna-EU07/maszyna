/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the screen space reflections of the main view (gfx.ssr), the part shared by the renderers: the matrices which take the points
// the rays hit in a frame to the place they had in the image of the frame kept for the reflections (the history: the finished
// image of the last frame the view was drawn in, with its mipmaps), and the parameters of the shaders (gl::scene_ubs::ssr_*).
//
// the rays go through the depth of the scene drawn before the colour (light_common.glsl, ssr.glsl). the world is taken to stand
// still; what lies in the box around the vehicle of the cab moves with the camera, and is taken to the history with the matrix of
// the cab. the other vehicles moving leave their reflections a frame behind

#pragma once

#include <glm/glm.hpp>

class TDynamicObject;

namespace gl
{
struct scene_ubs;
}

namespace gfx
{

class screen_reflections
{
  public:
	// the frame of the colour pass of the main view about to be drawn: Projection as it is drawn (with the offset of the temporal
	// anti-aliasing), View the rotation of the camera (the axes of the world to those of the view), Camera its place, Cab the vehicle
	// whose cab is drawn (or null), Size of the images of the pass, Levels the mipmaps of the history
	void begin_frame(glm::mat4 const &Projection, glm::dmat3 const &View, glm::dvec3 const &Camera, TDynamicObject const *Cab, glm::ivec2 const &Size, int const Levels);
	// the image of the frame begun last went to the history
	void end_frame();
	// the history is dropped (the renderer made its images anew, the view changed size)
	void reset()
	{
		m_history.valid = false;
	}
	// the history holds an image the frame begun last can take the reflections from
	bool history_valid() const
	{
		return m_historyvalid;
	}
	// the parameters of the shaders for the frame begun last: Steps of the rays (0: none), whether the ray tracing fills in what the
	// screen doesn't hold (Raytracing, the longest traced ray Raytracingrange) and whether the noise of the steps changes from frame
	// to frame (Temporal, with the temporal anti-aliasing). the rays of the screen are off when the history can't be used
	void setup(gl::scene_ubs &Scene, float const Steps, bool const Raytracing, float const Raytracingrange, bool const Temporal) const;
	// the parameters with the reflections off, for the other passes
	static void clear(gl::scene_ubs &Scene);
	// the steps of a ray at a level of gfx.ssr (1 low, 2 medium, 3 high), 0: off
	static float steps(int const Level);
	// the settings of the reflections in a debug window, with a line on the ray tracing (null: the renderer has none)
	void draw_debug_ui(char const *Raytracing) const;

  private:
	struct frame
	{
		glm::dmat4 projection{1.0}; // as drawn
		glm::dmat3 view{1.0};
		glm::dvec3 camera{0.0};
		glm::dmat4 cab{1.0}; // the cab to the world around the camera
		TDynamicObject const *vehicle{nullptr};
		glm::ivec2 size{0};
		bool valid{false};
	};

	frame m_history; // of the image in the history
	frame m_current; // of the frame begun last
	glm::mat4 m_reprojection{1.f};
	glm::mat4 m_reprojectioncab{1.f};
	glm::mat4 m_cabbox{1.f};
	bool m_hascab{false};
	int m_levels{1};
	unsigned int m_frame{0};
	bool m_historyvalid{false};
};

} // namespace gfx
