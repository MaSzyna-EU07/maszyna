/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "scene/eu7/eu7_pack_mesh_loader.h"
#include "scene/eu7/eu7_types.h"
#include "vehicle/DynObj.h"

#include <cstddef>
#include <string>
#include <vector>

namespace scene::eu7 {

void
preload_pack_models( std::vector<Eu7Model> const &Models );

void
preload_pack_models(
    std::vector<Eu7Model> const &Models,
    std::vector<std::string> const &UniqueMeshes );

std::size_t
warm_pack_texture_paths_main(
    std::string const *paths,
    std::size_t count,
    double budget_ms = 0.0,
    std::size_t *processed_out = nullptr,
    Eu7Model const *models = nullptr,
    std::size_t model_count = 0 );

std::size_t
warm_pack_textures_main(
    Eu7Model const *models,
    std::size_t count,
    double budget_ms = 0.0 );

void
reset_pack_texture_warm_cache();

[[nodiscard]] bool
assign_pack_texture(
    material_data &material,
    std::string const &model_file,
    std::string const &texture_file,
    std::string const &resolved_texture = {},
    std::uint32_t textures_alpha = 0 );

} // namespace scene::eu7
