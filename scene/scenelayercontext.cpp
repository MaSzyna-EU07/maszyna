/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/scenelayers.h"

namespace scene
{

// converts world space location to the one which produces it when loaded with this context in effect
glm::dvec3 layer_context::to_local(glm::dvec3 Location) const
{
	// reverse of the transformation done by the scenery loader: rotation around the vertical axis, then scale, then offset
	Location -= offset;
	Location /= glm::dvec3{scale};
	if (rotation != glm::vec3{0.f})
	{
		Location = glm::rotateY<double>(Location, -glm::radians(static_cast<double>(rotation.y)));
	}
	return Location;
}

glm::dvec3 layer_context::to_world(glm::dvec3 Location) const
{
	if (rotation != glm::vec3{0.f})
	{
		Location = glm::rotateY<double>(Location, glm::radians(static_cast<double>(rotation.y)));
	}
	Location *= glm::dvec3{scale};
	Location += offset;
	return Location;
}

bool layer_context::matches(layer_context const &Other) const
{
	return glm::all(glm::epsilonEqual(offset, Other.offset, 1e-4)) && glm::all(glm::epsilonEqual(rotation, Other.rotation, 1e-4f)) && glm::all(glm::epsilonEqual(scale, Other.scale, 1e-5f));
}

} // namespace scene
