/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

// where a scenery made after the real land lies on the map (the sceneries of IRT and the ones the editor laid on the aerial photos):
// the origin of its world in PUWG 1992 (EPSG:2180). the world x runs to the west and z to the north of it, as the editor lays the
// orthophoto (editor_orthophoto::camera_tile()). a scenery without it isn't on the map, and the live weather leaves it alone

#pragma once

#include <optional>
#include <string>

#include <glm/glm.hpp>

namespace georeference
{

// the origin of the scenery loaded (Global.SceneryFile), PUWG 1992 metres (north, east): the place the editor keeps for it, else the
// "//$g <1992 system> <easting> <northing>" line Rainsted writes into the scenery file or a file it includes near its top (the IRT
// sceneries); nullopt for a scenery not laid on the map. read once a scenery
std::optional<glm::dvec2> scenery_origin();

// the origin out of the //$g line of a text, PUWG 1992 metres (north, east); kilometres as Rainsted writes them are taken too
std::optional<glm::dvec2> parse_line(std::string const &Line);

// degrees (latitude, longitude) of a World point of a scenery of the Origin (north, east)
glm::dvec2 to_degrees(glm::dvec2 const &Origin, glm::dvec3 const &World);

} // namespace georeference
