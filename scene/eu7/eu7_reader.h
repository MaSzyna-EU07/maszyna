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

#include <functional>
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

// Odczyt sekcji PACK: modele + precomputed UMES (v9/v10).
[[nodiscard]] Eu7PackSectionLoad
read_pack_section_load( Eu7Module const &Module, int Row, int Column );

// Odczyt jednego sub-chunka sekcji PACK (v10 CHNK; v9/v7 = cala sekcja przy ChunkIndex 0).
[[nodiscard]] Eu7PackSectionChunkLoad
read_pack_section_chunk_load(
    Eu7Module const &Module,
    int Row,
    int Column,
    std::uint32_t ChunkIndex );

// Czysci thread-local cache odczytu PACK (otwarty plik + sparsowany naglowek sekcji).
void
reset_pack_section_read_cache();

// UMES z naglowka sekcji — bez kopiowania calego wektora.
void
for_each_pack_section_unique_mesh(
    Eu7Module const &Module,
    int Row,
    int Column,
    std::function<void( std::string const & )> const &Visit );

// UTEX z naglowka sekcji (v11+) — bez skanowania instancji.
void
for_each_pack_section_unique_texture(
    Eu7Module const &Module,
    int Row,
    int Column,
    std::function<void( std::string const & )> const &Visit );

} // namespace scene::eu7
