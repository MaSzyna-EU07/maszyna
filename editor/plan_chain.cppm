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

export module eu07.editor.plan_chain;
export import eu07.editor.plan_segment;
export import eu07.editor.plan_model;
export import eu07.editor.plan_solution;

export {

namespace editor::plan {

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
                                     const geometry::Pose& start);

}  // namespace editor::plan

}  // export
