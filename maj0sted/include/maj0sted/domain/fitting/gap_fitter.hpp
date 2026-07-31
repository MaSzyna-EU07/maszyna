#pragma once

#include <optional>
#include <vector>

#include "maj0sted/domain/fitting/fit_result.hpp"
#include "maj0sted/domain/geometry/plan_element.hpp"  // Straight
#include "maj0sted/domain/value_objects/length.hpp"
#include "maj0sted/domain/value_objects/radius.hpp"

namespace maj0sted::domain {

/// How the gap between two independent, finite straights should be connected.
/// A pure domain concept: the editor's numeric fit modes map onto it, but this
/// enum carries no UI knowledge. A plain circular arc (Arc) and a compound curve
/// (Compound) are the only kinds — a "łuk + klotoidy" is just a Compound with a
/// single arc eased by symmetric clothoids.
enum class GapConnectionMode { Arc, Compound };

/// One arc of a compound connection.
struct CompoundGapArc {
    double radius{0.0};              ///< arc radius (metres)
    double length{0.0};              ///< fixed arc length (metres); ignored for the last arc
    double transition_to_next{0.0};  ///< clothoid to the next arc (metres, 0 = none)
};

/// Extra parameters for a compound connection: entry/exit easing clothoids plus a
/// sequence of one or more arcs. A single arc yields "łuk + klotoidy"; two or
/// more yield a "łuk koszowy". The last arc absorbs the remaining deflection, so
/// its @c length / @c transition_to_next are ignored.
struct CompoundGapParameters {
    double entry_transition{0.0};
    double exit_transition{0.0};
    std::vector<CompoundGapArc> arcs{};
};

/// A request to connect two finite straights with a curve.
struct GapConnectionRequest {
    GapConnectionMode mode{GapConnectionMode::Arc};
    double radius{0.0};                ///< plain-arc radius (metres)
    CompoundGapParameters compound{};  ///< compound only
};

/// The connection actually produced.
struct GapConnection {
    FitResult fit;               ///< trimmed straights + spliced curve
    double applied_radius{0.0};  ///< radius truly used (may be reduced to fit)
};

/// Domain policy connecting two independent, finite straights for the editor.
///
/// This is the single source of truth for the fitting business rules:
///   * a 180° reversal is detected from the geometry and handled as a nawrót,
///     never surfaced as a user-chosen mode;
///   * a plain arc uses the requested radius verbatim whenever it fits; the
///     radius is NEVER enlarged. A tighter-than-needed arc fits by extending the
///     straights toward the corner; only a radius too large to land on the
///     straights is reduced, to the largest one that fits. No other curve type is
///     ever substituted;
///   * a compound curve (one arc eased by clothoids, or a multi-arc basket) is
///     built as specified;
///   * a tangent point may extend a straight toward the corner, but must not cross
///     behind its outer, anchored end.
///
/// Returns nullopt when nothing admissible fits (the caller then leaves the gap
/// unconnected). Never throws.
class GapFitter {
public:
    [[nodiscard]] static std::optional<GapConnection> connect(
        const Straight& entry, const Straight& exit,
        const GapConnectionRequest& request);
};

}  // namespace maj0sted::domain
