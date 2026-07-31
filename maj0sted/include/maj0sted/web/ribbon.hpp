#pragma once

#include <vector>

#include "maj0sted/web/solve.hpp"  // WebPoint, WebPolyline

namespace maj0sted::web {

/// One element of an alignment "ribbon" (element chain), defined intrinsically.
struct RibbonElement {
    int kind{0};            ///< 0 = line, 1 = arc, 2 = spiral (clothoid)
    double length{0.0};     ///< line / spiral length (arc length is derived)
    double radius{0.0};     ///< arc radius; spiral end radius (0 = straight / infinite)
    double angle{0.0};      ///< arc central angle (radians)
    double radius_start{0.0};  ///< spiral start radius (0 = infinite)
    int dir{1};             ///< +1 left, -1 right (arc / spiral)
};

/// A whole alignment defined as a start pose plus an ordered element chain,
/// laid out head-to-tail. Handles any arc angle (including 180°+, loops) because
/// nothing depends on tangent intersection.
struct RibbonRequest {
    double x0{0.0};
    double y0{0.0};
    double az0{0.0};  ///< start azimuth (radians, clockwise from north)
    std::vector<RibbonElement> elements;
};

/// Lays the ribbon out and returns one polyline per element, ready to draw.
[[nodiscard]] std::vector<WebPolyline> solve_ribbon(const RibbonRequest& request);

}  // namespace maj0sted::web
