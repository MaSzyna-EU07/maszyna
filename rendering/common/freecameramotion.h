/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// the motion blur of the camera with no vehicle of its own (freefly, the editor): the world is blurred by the movement of the
// camera, as it is by the movement of the vehicle the camera sits in. without it the ground racing under a low camera strobes in
// rings, at the distances its texture repeats. the movement from the frame before to this one stands for the one to come

#pragma once

#include <glm/glm.hpp>
#include <optional>

namespace gfx
{

class free_camera_motion
{
  public:
	// from this far in a frame on the camera jumped (a camera preset, another place of the scenery), which is no movement
	static constexpr double jump{50.0};
	// the longest movement of a frame the blur is given. it draws the scene where the camera will be: a faster camera (shift, ctrl)
	// passes the ground at the bottom of the screen within a frame, ends up behind it there, and its motion turns over into a
	// bright band. half a metre a frame still blurs metres with the shutter of the motion blur
	static constexpr float longest{0.5f};

	// the camera of the frame. gives its movement since the frame before, no longer than the longest one; none in the first frame
	// and after a jump
	glm::vec3 next_frame(glm::dvec3 const &Position);
	// the camera sits in a vehicle: the free camera starts anew the next time
	void reset()
	{
		m_previous.reset();
	}

  private:
	std::optional<glm::dvec3> m_previous;
};

} // namespace gfx
