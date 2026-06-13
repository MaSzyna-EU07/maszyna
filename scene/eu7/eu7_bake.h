/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <string>

namespace scene::eu7 {

struct Eu7BakeOutcome {
    bool ok { false };
    bool regenerated { false };
    std::string message;
};

// Czy wejscie to tekst SCM/SCN bez gotowego .eu7 przy wlaczonym auto-bake.
[[nodiscard]] bool
scenario_needs_eu7_regen( std::string const &ScenarioFile );

// Przed ladowaniem mapy: uzyj istniejacego <stem>.eu7 lub wygeneruj cale drzewo include.
[[nodiscard]] Eu7BakeOutcome
ensure_scenario_eu7( std::string const &ScenarioFile );

} // namespace scene::eu7
