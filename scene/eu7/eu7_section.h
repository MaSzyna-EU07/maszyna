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
#include <utility>

#include <glm/glm.hpp>

namespace scene::eu7 {

// Musi byc zgodne z scene::EU07_SECTIONSIZE / EU07_REGIONSIDESECTIONCOUNT.
constexpr int kSectionSizeM { 1000 };
constexpr int kRegionSideSectionCount { 500 };

[[nodiscard]] std::pair<int, int>
section_row_column( glm::dvec3 const &world_location );

[[nodiscard]] std::size_t
section_index( glm::dvec3 const &world_location );

[[nodiscard]] std::size_t
section_index( int const row, int const column );

} // namespace scene::eu7
