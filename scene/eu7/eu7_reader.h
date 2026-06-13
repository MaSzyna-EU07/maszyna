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

#include <istream>
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

// Seek do pack_offset z PIDX i parsuj naglowek sekcji (v7/v8).
void
seek_pack_section(
    Eu7Module const &Module,
    std::istream &Input,
    Eu7PackIndexEntry const &Entry,
    Eu7PackSectionCursor &Cursor );

// Odczyt do max_count modeli z biezacej pozycji strumienia (po seek_pack_section).
[[nodiscard]] std::vector<Eu7Model>
read_pack_models_chunk(
    Eu7Module const &Module,
    std::istream &Input,
    Eu7PackSectionCursor &Cursor,
    std::size_t MaxCount );

// Wznowienie odczytu sub-chunka (offset wzgledem pack_offset sekcji z PIDX).
void
resume_pack_section(
    Eu7Module const &Module,
    std::istream &Input,
    Eu7PackIndexEntry const &Entry,
    std::uint64_t ResumeByteOffset,
    Eu7PackSectionCursor ResumeCursor,
    Eu7PackSectionCursor &Cursor );

} // namespace scene::eu7
