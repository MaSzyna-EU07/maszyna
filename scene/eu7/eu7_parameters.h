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

#include <string>
#include <vector>

namespace scene::eu7 {

void
bind_include_parameter( std::string &text, std::vector<std::string> const &parameters, bool to_lower = false );

void
apply_include_parameters_to_scene( Eu7Scene &scene, std::vector<std::string> const &parameters );

void
apply_include_parameters_to_models( std::vector<Eu7Model> &models, std::vector<std::string> const &parameters );

// Placement z chunku PLAC (indeksy pN z origin/rotate w .inc) + wartosci z INCL.
[[nodiscard]] Eu7TransformContext
placement_transform_from_include_parameters(
    Eu7IncludePlacement const &binding,
    std::vector<std::string> const &parameters );

void
apply_include_placement_to_scene(
    Eu7Scene &scene,
    Eu7IncludePlacement const &binding,
    std::vector<std::string> const &parameters );

void
apply_include_placement_to_models(
    std::vector<Eu7Model> &models,
    Eu7IncludePlacement const &binding,
    std::vector<std::string> const &parameters );

[[nodiscard]] std::string
module_load_key(
    std::string const &resolved_path,
    std::vector<std::string> const &parameters );

} // namespace scene::eu7
