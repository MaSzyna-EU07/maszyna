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

export module eu07.editor.plan_sketch;
export import eu07.editor.plan_model;
export import eu07.editor.plan_solution;

export {

namespace editor::plan {

/// Laying track by clicking along the ground.
///
/// The corner is worked out here, at the click, and written into the document as
/// ordinary elements with ordinary lengths — a straight shortened by the
/// tangent, an arc of the radius asked for, a straight running out to the point
/// clicked. Nothing is left pointing at the drawing, so the numbers in the
/// document are the numbers on the ground.

/// What rounding one corner costs.
struct Corner {
    bool ok{false};
    /// How far back from the corner the arc starts, and how far past it the arc
    /// ends: R·tan(|Δ|/2). Both straights lose this much.
    double tangent{0.0};
    double arc_length{0.0};   ///< R·|Δ|
    int hand{0};              ///< +1 left, -1 right
    double deflection{0.0};   ///< Δ, radians, signed, + = left
};

/// Rounds the corner between a heading in and a heading out with an arc of
/// @p radius. Both headings are unit vectors in the plan's frame.
///
/// Not ok for a corner that does not bend, one that doubles back on itself, or a
/// non-positive radius: none of those is an arc, and guessing one is how a fit
/// used to quietly lay geometry nobody asked for.
[[nodiscard]] Corner fit_corner(double in_hx, double in_hy, double out_hx, double out_hy,
                                double radius);

/// The three elements a straight-arc-straight corner is made of, ready to append
/// after @p incoming has been shortened by `corner.tangent`.
[[nodiscard]] Element make_arc(Document& document, double radius, int hand, double length);
[[nodiscard]] Element make_line(Document& document, double length);
[[nodiscard]] Element make_clothoid(Document& document, double end_radius, int hand,
                                    double length);

/// Carries a change of radius or hand across the joint it belongs to.
///
/// A transition curve's radius is the radius it ends on, so it belongs to
/// whatever comes after it: the arc it eases into, or nothing at all when a
/// straight follows and it runs out flat at R=0. That is one number, not two, so
/// editing either side of the joint carries the other with it — and the joint
/// only ever looks forward. The transition behind an arc is the one that eases
/// into it; the one in front of it eases into whatever is beyond, which is none
/// of that arc's business.
void sync_joint(Track& track, std::size_t at);

// ---------------------------------------------------------------------------
// Straights as lines, and the arcs between them
// ---------------------------------------------------------------------------

/// The line a straight lies on: a point on it and the direction of travel.
///
/// This is what a straight really is once it is drawn — where it lies and which
/// way it points. Its length is not part of it: that falls out of where the arcs
/// at either end come off it.
struct Support {
    double px{0.0}, py{0.0};
    double dx{1.0}, dy{0.0};
};

/// What a track hangs on: the line under every one of its straights, and where
/// the first and last of them begin and end.
///
/// The corners between two straights are not in here, because they do not have
/// to be: given the lines, each of them is worked out. A corner hanging off
/// either end of the chain — a track that starts or finishes mid-curve — is not
/// in here either, for the opposite reason: it has nothing on its far side to be
/// fitted against, so it is carried along exactly as it was authored.
struct Skeleton {
    std::vector<Support> lines;
    double start_x{0.0}, start_y{0.0};  ///< where the first straight begins
    double end_x{0.0}, end_y{0.0};      ///< where the last straight ends
    /// Where the chain has to begin, and whether that is a place anybody may
    /// move. A branch on a turnout's port may not: its first curve is laid from
    /// the frog and the line under the straight after it follows wherever that
    /// curve comes out, rather than the curve being slid along a line drawn
    /// before the edit.
    bool pinned{false};
    geometry::Pose begins{};
};

/// Pulls @p track's skeleton out of @p solved.
///
/// A chain must read as straights with corners between them — a corner being any
/// run of arcs and transition curves that turns one way, one arc or a basket of
/// them. Two straights may never touch: two straights end to end are one
/// straight. A track with no straight at all has no skeleton, and says so.
[[nodiscard]] bool skeleton_of(const Track& track, const SolvedTrack& solved,
                               Skeleton& out, std::string& why);

/// Rewrites @p track to run along @p skeleton, keeping every arc's radius and
/// hand, and every transition curve's length.
///
/// Each corner with a straight on both sides is fitted: the transitions turn what
/// they turn, the arcs share out what is left of the corner in the proportions
/// they were drawn with — so a basket curve stays that basket, tighter or wider —
/// and the whole corner is then slid along the line it leaves until it lands on
/// the line it joins. The turn is taken the way the arcs go, so a corner may run
/// past a half circle, up to a full one, and nothing shortens it to the way round
/// that happens to be shorter.
///
/// A corner hanging off either end of the chain keeps exactly what it was given:
/// there is no second line to fit it against, so a track ending mid-curve simply
/// ends where that curve takes it.
///
/// The skeleton's ends stay put and the anchor follows them — unless the track is
/// pinned to a turnout's port, which is not an anchor anyone may move: a change
/// that would drag the branch off its frog is refused instead.
///
/// False, with a reason, when the skeleton cannot carry the chain. Nothing is
/// written to @p track unless all of it works out.
[[nodiscard]] bool lay_along_skeleton(Track& track, const Skeleton& skeleton,
                                      std::string& why);


// ---------------------------------------------------------------------------
// Putting a joint into a chain, and taking a piece out of one
// ---------------------------------------------------------------------------

/// What putting a joint in worked out to.
struct JointOutcome {
    bool ok{false};
    /// Why it could not be put there, ready to show; empty when it was. Nothing is
    /// written to the document unless the whole thing works out.
    std::string why;
    /// The element that took the far half. It is a new element with a new id; the
    /// near half keeps the id the whole element had.
    ElementId added{ElementId::none};
};

/// Divides whichever element of @p track the station lands in, leaving one chain
/// of one more element and exactly the same geometry on the ground.
///
/// The chain is relative — every element starts where the one before it ends — so
/// there is nothing to recompute: the two halves carry the lengths they were given
/// and the shape they always had. A transition curve divided in half eases to the
/// radius it had reached, and the far half goes on to the radius it was always
/// going to; the curvature at the new joint is the one that was there all along, so
/// the fit still reads the chain the same way.
///
/// Refused on a straight: two straights end to end are one straight, which is what
/// the chain would then be saying twice, and the skeleton the fit works from will
/// not read it. Refused inside a turnout too — a turnout is one piece of geometry
/// on one element.
[[nodiscard]] JointOutcome insert_joint(Document& document, const Solution& solution,
                                        TrackId track, double station);

/// What taking a piece out worked out to.
struct CutOutcome {
    bool ok{false};
    std::string why;
    /// The track carrying what was beyond the piece. It keeps the elements it took,
    /// ids and all, so a hold pointing at one of them still resolves; a turnout
    /// standing past the cut goes with it, its station measured from the new start.
    TrackId second{TrackId::none};
};

/// Takes the piece between @p from and @p to out of @p track, leaving the two ends
/// standing where they stood. What was between them is gone, and the gap is simply
/// the ground between two tracks — there is no such thing here as a chain with a
/// hole in it, which is why this one does make a second track and @c insert_joint
/// does not.
///
/// Refused when a turnout stands in the piece, or when either end would land in the
/// middle of a transition curve: a chain always begins running straight, and there
/// is nowhere in the document to say otherwise.
[[nodiscard]] CutOutcome cut_track(Document& document, const Solution& solution, TrackId track,
                                   double from, double to);

}  // namespace editor::plan

}  // export
