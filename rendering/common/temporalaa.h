/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the temporal anti-aliasing of the main view (gfx.antialiasing taa), the part shared by the renderers: the sub-pixel offset of the
// projection of every frame, and the matrices which take the pixels of a frame to the place they had in the last one.
//
// with it the velocities of the colour pass point to the last frame: the future of the draws (model_ubs.future) is the matrix of the
// view of the last frame instead of the next one, for the scenery and for the cab apart (the cab moves with the camera), and the
// moving vehicles and wheels take their motion backwards (scene_walk::m_motionhistory). the resolve pass (postfx_taa.frag,
// taa_resolve.glsl) reads the history with the velocities, and takes the pixels of the world standing still (and the sky, which has
// no velocity drawn) to the last frame with reprojection(); on those the thin things (rails, wires) are kept in the history for a few
// frames although the pixel of this frame misses them

#pragma once

#include <algorithm>
#include <cstdint>
#include <glm/glm.hpp>

class TDynamicObject;

namespace gfx
{

class temporal_aa
{
  public:
	// the frame of the colour pass of the main view: Projection without the offset, View the rotation of the camera (the axes of the
	// world to those of the view), Camera its place, Cab the vehicle whose cab is drawn (or null), Size of the images of the pass.
	// returns the projection with the offset of the frame
	glm::mat4 begin_frame(glm::mat4 const &Projection, glm::dmat3 const &View, glm::dvec3 const &Camera, TDynamicObject const *Cab, glm::ivec2 const &Size);
	// the next frame starts the history anew
	void reset()
	{
		m_previous.valid = false;
	}
	// the offsets go round Phases frames (at least 1); 8 for the temporal anti-aliasing, more when dlss upscales (8 for each pixel
	// of the window)
	void set_phases(std::uint32_t const Phases)
	{
		m_phases = std::max<std::uint32_t>(Phases, 1);
	}
	// the frame has a history: the last frame was drawn the same way
	bool history_valid() const
	{
		return m_historyvalid;
	}
	// model_ubs.future of the scenery, and of the cab with the interiors of its vehicle
	glm::mat4 const &world_future() const
	{
		return m_worldfuture;
	}
	glm::mat4 const &cab_future() const
	{
		return m_cabfuture;
	}
	// the clip space of the pixels of this frame (with their depth) to the clip space of the last frame, for the world standing still
	glm::mat4 const &reprojection() const
	{
		return m_reprojection;
	}
	// the image of the history written this frame (0 or 1); the other one holds the last frame
	int target() const
	{
		return m_target;
	}
	// the offset of the frame, in pixels
	glm::vec2 jitter() const
	{
		return m_jitter;
	}

  private:
	struct frame
	{
		glm::dmat4 projection{1.0}; // without the offset
		glm::dmat3 view{1.0};
		glm::dvec3 camera{0.0};
		glm::dmat4 cab{1.0}; // the cab to the world around the camera
		TDynamicObject const *vehicle{nullptr};
		glm::ivec2 size{0};
		bool valid{false};
	};

	frame m_previous;
	std::uint32_t m_phase{0};
	std::uint32_t m_phases{8};
	glm::vec2 m_jitter{0.f};
	glm::mat4 m_worldfuture{1.f};
	glm::mat4 m_cabfuture{1.f};
	glm::mat4 m_reprojection{1.f};
	int m_target{0};
	bool m_historyvalid{false};
};

} // namespace gfx
