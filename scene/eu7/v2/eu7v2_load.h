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

// Reconstructs a parsed scene from an eu7v2 container produced by bake_scene()
// / bake_module(). The reconstructed Eu7Scene can be fed to the existing apply
// path, so switching to the new format only swaps serialization and reuses the
// proven world-construction code. Returns false on a malformed container (the
// scene is left in a partially-populated but safe state).
[[nodiscard]] bool
load_scene( std::uint8_t const *data, std::size_t size, scene::eu7::Eu7Scene &out );

// Reconstructs a full module (scene + includes + placement + flags) from an
// eu7v2 container. The runtime needs the includes to recurse into child
// modules. Returns false on a malformed container.
[[nodiscard]] bool
load_module( std::uint8_t const *data, std::size_t size, scene::eu7::Eu7Module &out );

} // namespace eu7v2
