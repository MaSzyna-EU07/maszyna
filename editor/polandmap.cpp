/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/polandmap.h"

#include "editor/puwg1992.h"

namespace editor
{

namespace
{

// the country's corners in degrees, wide enough to hold all of it: the Odra in the west, the Bug in
// the east, the Tatras in the south and the coast in the north
double const kSouthLatitude = 49.00;
double const kNorthLatitude = 54.90;
double const kWestLongitude = 14.10;
double const kEastLongitude = 24.20;

} // namespace

void poland_extent(glm::dvec2 &Min, glm::dvec2 &Max)
{
	// the projection is not axis-aligned with the graticule, so the extremes come from projecting
	// all four corners and taking the outermost values
	auto const sw = puwg1992(kSouthLatitude, kWestLongitude);
	auto const se = puwg1992(kSouthLatitude, kEastLongitude);
	auto const nw = puwg1992(kNorthLatitude, kWestLongitude);
	auto const ne = puwg1992(kNorthLatitude, kEastLongitude);

	Min = {std::min({sw.x, se.x, nw.x, ne.x}), std::min({sw.y, se.y, nw.y, ne.y})};
	Max = {std::max({sw.x, se.x, nw.x, ne.x}), std::max({sw.y, se.y, nw.y, ne.y})};
}

} // namespace editor
