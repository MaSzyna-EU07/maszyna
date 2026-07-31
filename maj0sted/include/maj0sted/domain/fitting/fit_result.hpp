#pragma once

#include <vector>

#include "maj0sted/domain/geometry/plan_element.hpp"
#include "maj0sted/domain/value_objects/cartesian_position.hpp"

namespace maj0sted::domain {

/// Result of fitting a curve between two straights.
///
/// The two straights keep their identity but are trimmed back to the tangent
/// points; @c curve is the klotoida?/łuk/klotoida? to splice between them. The
/// caller (or an aggregate command) replaces the original entry/exit straights
/// with @c entry, then @c curve, then @c exit.
struct FitResult {
    Straight entry;                  ///< trimmed entry straight (ends at tangent_in)
    std::vector<PlanElement> curve;  ///< klotoida? / łuk / klotoida?
    Straight exit;                   ///< trimmed exit straight (starts at tangent_out)
    CartesianPosition tangent_in;    ///< entry -> curve tangent point
    CartesianPosition tangent_out;   ///< curve -> exit tangent point
    TurnDirection direction;         ///< derived turn direction
};

}  // namespace maj0sted::domain
