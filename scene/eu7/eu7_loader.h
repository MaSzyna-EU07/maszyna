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

namespace simulation {
class state_serializer;
}

namespace scene {

class basic_region;
class shape_node;
class lines_node;

namespace eu7 {

// Buduje shape_node z rekordu EU7B (friend shape_node).
[[nodiscard]] shape_node
    build_shape_node( Eu7Shape const &Source );

// Buduje lines_node z rekordu EU7B (friend lines_node).
[[nodiscard]] lines_node
    build_lines_node( Eu7Lines const &Source );

// Pełna ścieżka względem Global.asCurrentSceneryPath (jak basic_region::deserialize).
[[nodiscard]] std::string
    resolve_scenery_path( std::string const &Reference );

// Zamienia .scm/.sbt/.inc/.scn na .eu7 przy tym samym stemie.
[[nodiscard]] std::string
    binary_path( std::string const &Reference );

// Jak cParser: mPath + include, z katalogiem pliku biezacego dla sciezek wzglednych.
[[nodiscard]] std::string
    resolve_parser_include_path(
        std::string const &ParserPath,
        std::string const &CurrentFile,
        std::string const &IncludeReference );

// Sciezka .eu7 obok include (ta sama lokalizacja co SCM/INC).
[[nodiscard]] std::string
    include_eu7_path(
        std::string const &ParserPath,
        std::string const &CurrentFile,
        std::string const &IncludeReference );

// Alias dla terenu (kompatybilnosc wsteczna).
[[nodiscard]] std::string
    terrain_binary_path( std::string const &TerrainReference );

// Czy plik istnieje i jest poprawnym EU7B (v4-v8).
[[nodiscard]] bool
    probe_file( std::string const &Path );

// Czy obok scenariusza (.scn/.scm) jest zbakowany plik <stem>.eu7.
[[nodiscard]] bool
    probe_baked_scenario( std::string const &ScenarioFile );

// Rozszerzenia modulow tekstowych (.scn/.scm/.inc/.sbt).
[[nodiscard]] bool
    is_text_module_extension( std::string const &Path );

// Dla .eu7: istniejacy plik tekstowy obok; dla referencji tekstowej: sciezka rozwiazana.
[[nodiscard]] std::string
    text_source_path( std::string const &Reference );

// Istniejacy, poprawny plik .eu7 obok referencji tekstowej.
[[nodiscard]] bool
should_use_binary_module( std::string const &Reference );

// Czy plik zawiera chunk TERR.
[[nodiscard]] bool
    probe_terrain_file( std::string const &Path );

// Czy obok scenariusza jest <stem>.eu7 z terenem (analogia do .sbt).
[[nodiscard]] bool
    is_scenario_terrain( std::string const &ScenarioFile );

// Ładuje <stem>.eu7 dla danego scenariusza. Zwraca false gdy brak pliku.
[[nodiscard]] bool
    try_load_scenario_terrain( basic_region &Region, std::string const &ScenarioFile );

// Wczytuje chunk TERR z podanej ścieżki .eu7 do regionu (szybka ścieżka).
[[nodiscard]] bool
    load_terrain( basic_region &Region, std::string const &Path );

// Wstawia kształty terenu z Eu7Module do regionu.
void
    insert_terrain_shapes( basic_region &Region, Eu7Module const &Module );

// Czyści sesje deduplikacji przed ladowaniem scenariusza EU7.
void
    begin_load_session();

// Czy modul juz zostal zaladowany w biezacej sesji (parser include nie laduje drugi raz).
[[nodiscard]] bool
    is_module_loaded( std::string const &Path );

// Pełne ładowanie modułu EU7B (wszystkie chunki oprócz STRS).
[[nodiscard]] bool
    load_module( std::string const &Path, simulation::state_serializer &Serializer );

// Root scenariusza ma chunk PACK — modele scenerii ida ze streamingu, nie z MODL w .inc.
[[nodiscard]] bool
    pack_scenery_active();

} // namespace eu7
} // namespace scene
