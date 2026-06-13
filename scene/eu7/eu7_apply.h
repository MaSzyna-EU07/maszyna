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

namespace scene {
class scratch_data;
}

namespace simulation {
class state_serializer;
}

namespace scene::eu7 {

[[nodiscard]] bool
is_model_only_module( Eu7Module const &module );

void
compose_models_with_prefix( std::vector<Eu7Model> &models, Eu7TransformContext const &prefix );

// Wczytuje zawartosc Eu7Module do symulacji (bez ponownego INCL i bez petli include).
void
    apply_module(
        Eu7Module const &Module,
        simulation::state_serializer &Serializer,
        scene::scratch_data &Scratch );

} // namespace scene::eu7
