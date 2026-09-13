/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
module;
#include <string>
#include <vector>

export module eu07.editor.plan_layout;
export import eu07.editor.plan_turnout_geometry;
export import eu07.editor.plan_model;
export import eu07.editor.plan_solution;

export {

namespace editor::plan {

/// Solves a whole document: lays every turnout on its through track, lays every
/// track from its anchor, and moves the tracks that are held parallel to another
/// track's element until they really lie at the offset they were given.
///
/// The order is not fixed in advance — it is worked out from what refers to
/// what. A branch anchored to a turnout's frog is laid after that turnout, which
/// is laid after its through track, however deep that goes: a turnout on the
/// branch of a turnout is nothing special. A reference that runs in a circle is
/// reported and its members are left unlaid, rather than being fed through a
/// fixed number of relaxation passes and whatever came out.
///
/// Nothing here writes to @p document. Never throws.
[[nodiscard]] Solution solve(const Document& document);

/// The catalogue type as the geometry layer wants it: the piece list with its
/// curvatures signed for @p hand (+1 diverges left). Public because the editor
/// dimensions a *type* with it — laying one from an identity pose is how the
/// catalogue numbers are checked before any instance stands on a track.
[[nodiscard]] plan::Turnout to_domain(const TurnoutType& type, int hand);

/// The construction points of a turnout laid from @p start: where its parts meet,
/// where its blade is dimensioned, and the points the drawing takes its lengths
/// between. Public for the same reason as to_domain — so a bare type is drawn from
/// the very same list the solver reports for a placement, rather than from a second
/// idea of what a construction point is.
[[nodiscard]] std::vector<TurnoutMark> turnout_marks(const plan::TurnoutGeometry& geometry,
                                                     const geometry::Pose& start);

}  // namespace editor::plan

}  // export
