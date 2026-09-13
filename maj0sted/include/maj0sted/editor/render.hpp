#pragma once

#include <vector>

#include "maj0sted/editor/solution.hpp"

namespace maj0sted::editor {

/// One drawable rail, tagged with what it came from so the host can colour it
/// and hit-test it. Exactly one of @c element / @c turnout is set.
struct PlanPolyline {
    Kind kind{Kind::Line};
    ElementId element{ElementId::none};
    TrackId track{TrackId::none};
    TurnoutId turnout{TurnoutId::none};
    /// For a turnout rail: which piece of the diverging path it is, and what part
    /// that piece plays (0 przediglicowy, 1 iglica, 2 łuk, 3 krzyżownicowa), so a
    /// drawing can say every element apart. -1 on a plain track rail.
    int part{-1};
    std::size_t piece{0};
    double length{0.0};
    double radius_start{0.0};  ///< 0 == infinite
    double radius_end{0.0};
    std::vector<PlanPoint> points;
};

/// Draws a solved layout as rails: two per element, a gauge apart. Turnout
/// diverging paths are drawn too, tagged with their turnout.
///
/// This is presentation only. It decides nothing about geometry, and the
/// exporter no longer reads it back — the axis it needs is in the Solution.
[[nodiscard]] std::vector<PlanPolyline> render_rails(const Solution& solution,
                                                     double gauge_metres = 1.5);

}  // namespace maj0sted::editor
