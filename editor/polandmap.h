/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <glm/glm.hpp>

namespace editor
{

// the whole country's extent in EPSG:2180 metres, as the opening view of a location picker. from
// there the base map itself carries the detail, so nothing else has to be drawn on top of it
void poland_extent(glm::dvec2 &Min, glm::dvec2 &Max);

} // namespace editor
