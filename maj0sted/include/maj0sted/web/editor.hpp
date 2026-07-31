#pragma once

#include <string>
#include <vector>

#include "maj0sted/web/solve.hpp"  // WebPoint, WebPolyline

namespace maj0sted::web {

/// An anchored straight segment (two XY endpoints).
struct StraightSpec {
    double x1{0.0}, y1{0.0}, x2{0.0}, y2{0.0};
    /// Internal tangent guide used to solve a terminal curve. It participates
    /// in fitting but is not rendered as track or exposed as an editor handle.
    bool hidden{false};
};

/// One arc of a compound fit: its radius, its fixed length (ignored for the last
/// arc, which absorbs the remaining deflection) and an optional clothoid to the
/// next arc.
struct CompoundArcSpec {
    double radius{0.0};
    double length{0.0};
    double transition_to_next{0.0};
};

/// A fit in the gap between straight @c gap and straight @c gap + 1.
struct GapFit {
    int gap{0};
    int mode{0};  ///< 0 none, 1 arc, 2 legacy arc+clothoids, 3 compound, 4 U-turn

    double radius{0.0};      ///< plain arc (mode 1)
    double transition{0.0};  ///< legacy mode-2 symmetric clothoid

    // Unified compound (mode 3): end clothoids plus one or more arcs.
    double entry_t{0.0}, exit_t{0.0};
    std::vector<CompoundArcSpec> arcs{};

    // Legacy fixed two-arc fields, kept so old saved projects still deserialize;
    // superseded by @c arcs. Used only when @c arcs is empty.
    double r1{0.0}, arc1_len{0.0}, between{0.0}, r2{0.0};
};

/// A niweleta as edited in the GUI: independent anchored straights plus fits
/// connecting consecutive straights. Because the straights are independent, a
/// gap can be a normal corner (straights meet at an intersection) OR a 180°
/// reversal (parallel, offset straights) — no 180° limitation.
struct NiweletaSpec {
    std::string name;
    std::vector<StraightSpec> straights;
    std::vector<GapFit> fits;
};

struct NiweletaPolys {
    std::vector<WebPolyline> polylines;
    /// Centreline segments after adjacent fits trim the input straights. Kept
    /// index-aligned with NiweletaSpec::straights for editor hit handles.
    std::vector<StraightSpec> rendered_straights;
    /// The authoritative fit set after solving: exactly the requested fits that
    /// actually succeeded. A requested fit that could not be produced (bad
    /// parameters or geometry) is simply absent here — the library, not the host,
    /// decides that a failed fit becomes "no fit". The host mirrors this list.
    std::vector<GapFit> applied_fits;
};

/// Builds each niweleta, fits every requested gap, and returns the sampled plan
/// per niweleta. Never throws: gaps whose parameters don't admit a fit are left
/// unconnected (the two straights simply stay as drawn).
[[nodiscard]] std::vector<NiweletaPolys> solve_project(
    const std::vector<NiweletaSpec>& niwelety);

}  // namespace maj0sted::web
