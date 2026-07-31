#pragma once

#include <string>

#include "maj0sted/render/scene.hpp"

namespace maj0sted::io {

struct SvgOptions {
    double width_px = 900.0;   ///< target drawing width
    double margin_px = 24.0;   ///< padding around the drawing
    double stroke_px = 2.0;    ///< line width
};

/// Renders a scene to a standalone SVG document (north up). Straights, arcs and
/// transition curves are drawn in distinct colours. Dependency-free — just a
/// simple viewer; a real GUI would consume render::Scene directly.
[[nodiscard]] std::string to_svg(const render::Scene& scene, const SvgOptions& options = {});

}  // namespace maj0sted::io
