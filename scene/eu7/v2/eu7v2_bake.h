/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <cstdint>
#include <vector>

namespace scene::eu7 {
struct Eu7Scene;
struct Eu7Module;
} // namespace scene::eu7

namespace eu7v2 {

// Bakes a parsed scene (output of the text parser) into a self-contained
// eu7v2 "sim" container: STRS + PROT/INST (deduplicated models) + MESH
// (terrain) + SHPE/LINE + simulation record chunks. This is the clean-slate
// replacement for the legacy EU7B emitter; it reuses the existing parser
// front-end and only changes the on-disk representation.
[[nodiscard]] std::vector<std::uint8_t>
bake_scene( scene::eu7::Eu7Scene const &scene );

// Lossless module bake: everything bake_scene() emits, plus INCL (includes
// for recursion) and META (flags, placement, first_init_count). This is the
// format the runtime loads as a module.
[[nodiscard]] std::vector<std::uint8_t>
bake_module( scene::eu7::Eu7Module const &module );

} // namespace eu7v2
