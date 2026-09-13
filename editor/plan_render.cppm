/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <vector>

export module eu07.editor.plan_render;
export import eu07.editor.plan_solution;

export {

namespace editor::plan {

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

}  // namespace editor::plan

}  // export
