/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scene/eu7/eu7_types.h"

#include <optional>
#include <string>
#include <vector>

namespace scene::eu7 {

[[nodiscard]] Eu7Module
read_module( std::string const &Path );

[[nodiscard]] bool
is_valid_eu7b_file( std::string const &Path );

// Szuka wpisu PIDX dla sekcji 1 km (row/column jak basic_region::section).
[[nodiscard]] std::optional<Eu7PackIndexEntry>
find_pack_entry( Eu7Module const &Module, int Row, int Column );

// Odczyt MODL z chunku PACK dla jednej sekcji (world-space, po bake).
[[nodiscard]] std::vector<Eu7Model>
read_pack_section( Eu7Module const &Module, int Row, int Column );

// Odczyt sekcji PACK: modele + precomputed UMES (v9).
[[nodiscard]] Eu7PackSectionLoad
read_pack_section_load( Eu7Module const &Module, int Row, int Column );

} // namespace scene::eu7
