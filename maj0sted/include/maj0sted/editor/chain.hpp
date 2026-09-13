#pragma once

#include <vector>

#include "maj0sted/domain/geometry/segment_layout.hpp"
#include "maj0sted/editor/model.hpp"
#include "maj0sted/editor/solution.hpp"

namespace maj0sted::editor {

/// What laying one track produced, and everything that went wrong doing it.
struct TrackOutcome {
    SolvedTrack track;
    std::vector<Diagnostic> diagnostics;
};

/// Lays @p track from @p start.
///
/// Every element states its own length, so laying is a walk: each element starts
/// on the pose the one before it ended on, and nothing is solved for. A clothoid
/// takes its near-end curvature from where the chain has got to, which is the
/// only way one element depends on another.
///
/// Never throws. An element that cannot be laid — a non-positive length, an arc
/// without a radius or a hand, a clothoid that changes nothing — is reported and
/// ends the chain there; everything before it is still returned and still
/// drawable.
[[nodiscard]] TrackOutcome lay_track(const Document& document, const Track& track,
                                     const domain::geometry::Pose& start);

}  // namespace maj0sted::editor
