/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#include "stdafx.h"
#include "scene/eu7/eu7_section.h"

#include <algorithm>
#include <cmath>

namespace scene::eu7 {

std::pair<int, int>
section_row_column( glm::dvec3 const &world_location ) {
    auto const column {
        static_cast<int>( std::floor(
            world_location.x / static_cast<double>( kSectionSizeM ) +
            kRegionSideSectionCount / 2 ) ) };
    auto const row {
        static_cast<int>( std::floor(
            world_location.z / static_cast<double>( kSectionSizeM ) +
            kRegionSideSectionCount / 2 ) ) };

    return {
        std::clamp( row, 0, kRegionSideSectionCount - 1 ),
        std::clamp( column, 0, kRegionSideSectionCount - 1 ) };
}

std::size_t
section_index( glm::dvec3 const &world_location ) {
    auto const [row, column] { section_row_column( world_location ) };
    return section_index( row, column );
}

std::size_t
section_index( int const row, int const column ) {
    return (
        static_cast<std::size_t>( row ) * static_cast<std::size_t>( kRegionSideSectionCount ) +
        static_cast<std::size_t>( column ) );
}

} // namespace scene::eu7
