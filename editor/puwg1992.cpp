/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "editor/puwg1992.h"

namespace editor
{

namespace
{

// PUWG 1992: a single transverse Mercator zone over the whole country, on the GRS80 ellipsoid
double const kSemiMajor = 6378137.0;
double const kFlattening = 1.0 / 298.257222101;
double const kCentralMeridian = 19.0;  // degrees east
double const kScaleFactor = 0.9993;
double const kFalseEasting = 500000.0;
double const kFalseNorthing = -5300000.0;

} // namespace

glm::dvec2 puwg1992(double const Latitude, double const Longitude)
{
	auto const e2 = 2.0 * kFlattening - kFlattening * kFlattening;
	auto const ep2 = e2 / (1.0 - e2);

	auto const lat = glm::radians(Latitude);
	auto const dlon = glm::radians(Longitude - kCentralMeridian);

	auto const sinlat = std::sin(lat);
	auto const coslat = std::cos(lat);
	auto const tanlat = std::tan(lat);

	// radius of curvature in the prime vertical, and the series terms of the standard
	// transverse Mercator expansion (Snyder); good to millimetres over a zone this size
	auto const n = kSemiMajor / std::sqrt(1.0 - e2 * sinlat * sinlat);
	auto const t = tanlat * tanlat;
	auto const c = ep2 * coslat * coslat;
	auto const a = dlon * coslat;

	// meridional arc from the equator
	auto const m =
	    kSemiMajor * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * lat -
	                  (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(2.0 * lat) +
	                  (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * std::sin(4.0 * lat) -
	                  (35.0 * e2 * e2 * e2 / 3072.0) * std::sin(6.0 * lat));

	auto const a2 = a * a;
	auto const a3 = a2 * a;
	auto const a4 = a3 * a;
	auto const a5 = a4 * a;
	auto const a6 = a5 * a;

	auto const easting = kFalseEasting + kScaleFactor * n *
	                                         (a + (1.0 - t + c) * a3 / 6.0 +
	                                          (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * ep2) * a5 / 120.0);

	auto const northing = kFalseNorthing + kScaleFactor *
	                                           (m + n * tanlat *
	                                                    (a2 / 2.0 + (5.0 - t + 9.0 * c + 4.0 * c * c) * a4 / 24.0 +
	                                                     (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * ep2) * a6 / 720.0));

	return {easting, northing};
}

} // namespace editor
