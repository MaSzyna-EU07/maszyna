#pragma once

#include "maj0sted/domain/geometry/horizontal_alignment.hpp"
#include "maj0sted/domain/map_project.hpp"
#include "maj0sted/render/scene.hpp"

namespace maj0sted::render {

/// Samples a plan into absolute polylines in the CRS. Straights are
/// authoritative for position; curves (arcs, clothoids) are laid out continuing
/// from the previous element's end and heading. This is the reusable bridge
/// between the domain model and any renderer.
[[nodiscard]] Scene sample(const maj0sted::domain::HorizontalAlignment& plan);

/// Samples the plans of every niweleta in the project into one scene.
[[nodiscard]] Scene sample(const maj0sted::domain::MapProject& project);

}  // namespace maj0sted::render
