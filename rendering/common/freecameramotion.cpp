/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "rendering/common/freecameramotion.h"

glm::vec3 gfx::free_camera_motion::next_frame(glm::dvec3 const &Position)
{
	glm::vec3 movement{0.f};
	if (m_previous)
	{
		auto const step{glm::vec3{Position - *m_previous}};
		// a position that isn't a number (a camera gone wrong) fails the comparison and gives no movement either
		if (auto const length{glm::length(step)}; length < static_cast<float>(jump))
		{
			movement = (length > longest) ? step * (longest / length) : step;
		}
	}
	m_previous = Position;
	return movement;
}
