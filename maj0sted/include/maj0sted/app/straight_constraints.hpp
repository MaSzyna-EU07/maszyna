#pragma once

#include <vector>

#include "maj0sted/editor/editor.hpp"

namespace maj0sted::app {

void apply_straight_constraints(std::vector<maj0sted::editor::NiweletaSpec>& niwelety);
void align_gap_fits(maj0sted::editor::NiweletaSpec& niweleta);

}  // namespace maj0sted::app
