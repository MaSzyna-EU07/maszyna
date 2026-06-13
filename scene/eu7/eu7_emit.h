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

namespace scene::eu7 {

// Sciezka tekstowa dla fallbacku include (source_path albo .eu7 -> .inc).
[[nodiscard]] std::string
    include_text_path( Eu7Include const &Include );

// Emituje bloki "include ... end" dla parsera SCM.
[[nodiscard]] std::string
    emit_includes_text( std::vector<Eu7Include> const &Includes );

// Emituje zawartosc modulu (bez INCL i bez terrain_shapes) do tekstu SCM.
[[nodiscard]] std::string
    emit_module_text( Eu7Module const &Module );

// Torow, eventy, modele itd. — bez TERR/MESH/LINE (te ida bezposrednio z binarki).
[[nodiscard]] std::string
    emit_scene_objects_text( Eu7Module const &Module );

} // namespace scene::eu7
