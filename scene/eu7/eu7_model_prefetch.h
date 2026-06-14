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

#include <cstddef>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class TModel3d;

namespace scene::eu7 {

// Tylko warm_instanceable_cache dla juz zcache'owanych meshy (bez cold GetModel na workerze).
void
preload_pack_models( std::vector<Eu7Model> const &Models );

// Jak wyzej, ale iteruje precomputed UMES zamiast skanowac instancje.
void
preload_pack_models(
    std::vector<Eu7Model> const &Models,
    std::vector<std::string> const &UniqueMeshes );

// Main thread: Fetch_Material dla unikalnych texture_file z chunka przed apply.
// Zwraca liczbe unikalnych Fetch_Material w tym slice.
std::size_t
warm_pack_texture_paths_main(
    std::string const *paths,
    std::size_t count,
    double budget_ms = 0.0,
    std::size_t *processed_out = nullptr );

std::size_t
warm_pack_textures_main( Eu7Model const *models, std::size_t count );

void
reset_pack_texture_warm_cache();

// Thread-safe cold mesh load: session cache, then GetModel under g_pack_mesh_load_mutex.
[[nodiscard]] bool
ensure_pack_mesh_in_session_cache(
    std::string model_file,
    std::unordered_map<std::string, TModel3d *> &session_cache );

} // namespace scene::eu7
