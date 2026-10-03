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

#include <glm/glm.hpp>

namespace gfx
{

// flat coloured geometry the editor shows in the 3d view, drawn with the depth test over the translucent scenery
struct editor_overlay
{
	struct batch
	{
		glm::vec4 color{1.0f};
		unsigned int type{0}; // GL primitive
		bool offset{false}; // pulled towards the camera, to show over the coplanar surfaces
		std::vector<glm::vec3> points; // relative to the origin
	};
	glm::dvec3 origin{0.0};
	std::vector<batch> batches;
	unsigned int revision{0}; // changed with every new content

	void clear()
	{
		batches.clear();
		++revision;
	}
};

} // namespace gfx

inline gfx::editor_overlay EditorOverlay;
