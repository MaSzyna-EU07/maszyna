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

// projects geographic coordinates onto PUWG 1992 (EPSG:2180) - the frame the track layout library
// works in, and the one Geoportal serves its imagery in. latitude and longitude in degrees, result
// in metres as (easting, northing).
glm::dvec2 puwg1992(double const Latitude, double const Longitude);

} // namespace editor
