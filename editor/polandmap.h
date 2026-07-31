/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>
#include <vector>

#include <glm/glm.hpp>

namespace editor
{

// a named point to start a scenery from, in EPSG:2180 metres
struct map_place
{
	std::string name;
	glm::dvec2 position;
};

// larger cities, to put the view somewhere recognisable before picking the exact spot
std::vector<map_place> const &poland_places();

// the whole country's extent in EPSG:2180 metres, as the starting view of a location picker
void poland_extent(glm::dvec2 &Min, glm::dvec2 &Max);

} // namespace editor
