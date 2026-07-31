#pragma once

#include <vector>

namespace maj0sted::web {

/// A request from the GUI: three draggable vertices (entry straight A->B, exit
/// straight B->C) plus the fit parameters. Plain doubles so it maps trivially
/// onto a JS object / WASM boundary. This layer is pure C++ — the emscripten
/// bindings are a thin wrapper around it, and it is unit-tested natively.
struct SolveRequest {
    double ax{0.0}, ay{0.0};  ///< vertex A (entry start)
    double bx{0.0}, by{0.0};  ///< vertex B (corner: entry end = exit start)
    double cx{0.0}, cy{0.0};  ///< vertex C (exit end)

    int mode{0};  ///< 0 = arc, 1 = arc + symmetric transitions, 2 = compound (łuk koszowy)

    double radius{0.0};      ///< modes 0/1: arc radius
    double transition{0.0};  ///< mode 1: symmetric transition-curve length

    // Mode 2 (compound):
    double r1{0.0};        ///< first arc radius
    double arc1_len{0.0};  ///< first arc length (split place)
    double between{0.0};   ///< transition length between the two arcs
    double r2{0.0};        ///< second arc radius
    double entry_t{0.0};   ///< entry transition length
    double exit_t{0.0};    ///< exit transition length
};

/// A point in the CRS.
struct WebPoint {
    double x{0.0};
    double y{0.0};
};

/// One polyline; @c kind is render::ElementKind (0 straight, 1 arc, 2 transition).
struct WebPolyline {
    int kind{0};
    /// Logical owner used by map hit-testing. Exactly one is non-negative.
    int straight_index{-1};
    int gap{-1};
    /// Position inside a fitted curve chain (arc/clothoid/arc...).
    int element_index{-1};
    double length{0.0};
    /// Zero denotes an infinite radius (straight end of a transition).
    double radius_start{0.0};
    double radius_end{0.0};
    std::vector<WebPoint> points;
};

/// Builds the niweleta, runs the requested fit (falling back to the bare
/// straights if the parameters do not admit a fit) and returns the sampled plan
/// as polylines ready to draw.
[[nodiscard]] std::vector<WebPolyline> solve_scene(const SolveRequest& request);

}  // namespace maj0sted::web
