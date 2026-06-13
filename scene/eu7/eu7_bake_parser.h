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

namespace scene::eu7::bake_parser {

[[nodiscard]] bool
bake_scenario_tree(
    std::string const &TextScenarioPath,
    unsigned MaxThreads,
    std::string &ErrorOut );

} // namespace scene::eu7::bake_parser
