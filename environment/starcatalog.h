/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstddef>
#include <cstdint>

// the stars of the night sky: the Yale Bright Star Catalogue, 5th revised edition (Hoffleit & Warren, 1991), every star with a
// position and a visual magnitude, about down to magnitude 6.5. positions for the equinox J2000
struct catalogue_star
{
	std::uint16_t right_ascension; // hundredths of a degree, 0-36000
	std::int16_t declination; // hundredths of a degree, -9000-9000
	std::int16_t magnitude; // visual magnitude, hundredths
	std::int16_t temperature; // colour temperature, tens of kelvins
};

namespace star_catalogue
{

catalogue_star const *stars();
std::size_t size();

} // namespace star_catalogue
