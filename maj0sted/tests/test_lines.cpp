// Pulling a straight about by one of its ends. The arcs beside it re-fit, every
// other straight keeps its bearing, and an arc that turns past a half circle
// stays as far round as it was drawn — that last one is the whole reason the
// construction goes through the arc's centre and not through where the tangents
// would have crossed.

#include <cmath>
#include <numbers>
#include <string>
#include <vector>

#include "check.hpp"
#include "maj0sted/editor/layout.hpp"
#include "maj0sted/editor/sketch.hpp"

using namespace maj0sted::editor;

namespace {

bool near(double a, double b, double tolerance = 1e-6) {
    return std::abs(a - b) <= tolerance;
}

Element line(Document& document, double metres) {
    return Element{mint_element(document), Kind::Line, 0.0, 0, metres, Free{}};
}
Element arc(Document& document, double radius, int hand, double metres) {
    return Element{mint_element(document), Kind::Arc, radius, hand, metres, Free{}};
}

double azimuth(const SolvedElement& element) {
    return std::atan2(element.start.hx, element.start.hy);
}

/// The shape most of this is about: straight, corner, straight, corner, straight.
TrackId add_track(Document& document, std::vector<Element> elements) {
    Track track;
    track.id = mint_track(document);
    track.name = "tor 1";
    track.anchor = AtPose{0.0, 0.0, 0.0};
    track.elements = std::move(elements);
    document.tracks.push_back(track);
    return track.id;
}

// -------------------------------------------------------------------------

/// Taking a track apart into lines and putting it back together, without moving
/// anything, gives back exactly the track that went in.
void a_track_survives_being_taken_apart_and_put_back() {
    Document document;
    const TrackId id = add_track(document, {line(document, 300.0), arc(document, 400.0, 1, 210.0),
                                            line(document, 250.0), arc(document, 300.0, -1, 180.0),
                                            line(document, 200.0)});

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, id);
    CHECK(laid != nullptr && before.diagnostics.empty());

    Skeleton skeleton;
    std::string why;
    CHECK(skeleton_of(document.tracks[0], *laid, skeleton, why));
    CHECK(skeleton.lines.size() == 3);

    Track rebuilt = document.tracks[0];
    CHECK(lay_along_skeleton(rebuilt, skeleton, why));
    for (std::size_t i = 0; i < rebuilt.elements.size(); ++i) {
        CHECK(near(rebuilt.elements[i].length, document.tracks[0].elements[i].length, 1e-9));
        CHECK(rebuilt.elements[i].radius == document.tracks[0].elements[i].radius);
        CHECK(rebuilt.elements[i].hand == document.tracks[0].elements[i].hand);
    }
}

/// An arc of 240°, which has no tangent crossing to be found: the two straights
/// it joins meet at 120° the other way round. Nothing may quietly turn it into
/// the 120° arc that would fit between the same two lines.
void an_arc_past_a_half_circle_stays_where_it_was_drawn() {
    Document document;
    const double turn = 240.0 * std::numbers::pi / 180.0;
    const TrackId id = add_track(document, {line(document, 200.0),
                                            arc(document, 200.0, 1, 200.0 * turn),
                                            line(document, 200.0)});

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, id);
    CHECK(laid != nullptr && before.diagnostics.empty());

    Skeleton skeleton;
    std::string why;
    CHECK(skeleton_of(document.tracks[0], *laid, skeleton, why));

    Track rebuilt = document.tracks[0];
    CHECK(lay_along_skeleton(rebuilt, skeleton, why));
    CHECK(near(rebuilt.elements[1].length, 200.0 * turn, 1e-9));
    CHECK(near(rebuilt.elements[0].length, 200.0, 1e-9));
    CHECK(near(rebuilt.elements[2].length, 200.0, 1e-9));
}

/// Exactly a half circle: the two straights are antiparallel and never cross at
/// all. Tangency fixes their spacing and nothing else, so the arc stays put.
void a_half_circle_is_laid_and_not_refused() {
    Document document;
    const double turn = std::numbers::pi;
    const TrackId id = add_track(document, {line(document, 150.0),
                                            arc(document, 250.0, -1, 250.0 * turn),
                                            line(document, 400.0)});

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, id);
    CHECK(laid != nullptr && before.diagnostics.empty());

    Skeleton skeleton;
    std::string why;
    CHECK(skeleton_of(document.tracks[0], *laid, skeleton, why));

    Track rebuilt = document.tracks[0];
    const bool ok = lay_along_skeleton(rebuilt, skeleton, why);
    CHECK(ok);
    if (!ok) {
        std::printf("  why: %s\n", why.c_str());
        return;
    }
    CHECK(near(rebuilt.elements[1].length, 250.0 * turn, 1e-6));
    CHECK(near(rebuilt.elements[0].length, 150.0, 1e-6));
    CHECK(near(rebuilt.elements[2].length, 400.0, 1e-6));
}

/// Swinging the middle straight: the arcs beside it re-fit, keeping their radii,
/// and the straights that do not touch it keep their bearing to the last bit.
void swinging_one_straight_leaves_the_others_pointing_where_they_did() {
    Document document;
    const TrackId id = add_track(document, {line(document, 300.0), arc(document, 400.0, 1, 210.0),
                                            line(document, 250.0), arc(document, 300.0, -1, 180.0),
                                            line(document, 200.0)});

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, id);
    CHECK(laid != nullptr);
    const double first_azimuth = azimuth(laid->elements[0]);
    const double last_azimuth = azimuth(laid->elements[4]);

    Skeleton skeleton;
    std::string why;
    CHECK(skeleton_of(document.tracks[0], *laid, skeleton, why));

    // Swing the middle line about the point the straight starts on, by 4°.
    const double angle = 4.0 * std::numbers::pi / 180.0;
    Support& middle = skeleton.lines[1];
    middle.px = laid->elements[2].start.x;
    middle.py = laid->elements[2].start.y;
    const double dx = middle.dx * std::cos(angle) - middle.dy * std::sin(angle);
    const double dy = middle.dx * std::sin(angle) + middle.dy * std::cos(angle);
    middle.dx = dx;
    middle.dy = dy;

    Track moved = document.tracks[0];
    CHECK(lay_along_skeleton(moved, skeleton, why));
    document.tracks[0] = moved;

    const Solution after = solve(document);
    const SolvedTrack* again = find_track(after, id);
    CHECK(again != nullptr && after.diagnostics.empty());

    // The straights at either end never moved: same bearing, to the micro-radian.
    CHECK(near(azimuth(again->elements[0]), first_azimuth, 1e-12));
    CHECK(near(azimuth(again->elements[4]), last_azimuth, 1e-12));
    // The arcs kept their radii and changed only how far they run.
    CHECK(near(again->elements[1].k0, 1.0 / 400.0, 1e-12));
    CHECK(near(again->elements[3].k0, -1.0 / 300.0, 1e-12));
    CHECK(!near(again->elements[1].length, 210.0, 1e-6));
    // And the free ends are still exactly where they were.
    CHECK(near(again->start.x, laid->start.x, 1e-9));
    CHECK(near(again->end.y, laid->end.y, 1e-9));
}

/// What cannot be laid says why, and leaves the caller's track alone.
void a_move_that_cannot_be_laid_is_refused() {
    Document document;
    const TrackId id = add_track(document, {line(document, 300.0), arc(document, 400.0, 1, 210.0),
                                            line(document, 250.0), arc(document, 300.0, -1, 180.0),
                                            line(document, 200.0)});
    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, id);

    Skeleton skeleton;
    std::string why;
    CHECK(skeleton_of(document.tracks[0], *laid, skeleton, why));

    {   // Swing the middle straight far enough that the two arcs overlap on it.
        Skeleton bent = skeleton;
        const double angle = 60.0 * std::numbers::pi / 180.0;
        Support& middle = bent.lines[1];
        const double dx = middle.dx * std::cos(angle) - middle.dy * std::sin(angle);
        const double dy = middle.dx * std::sin(angle) + middle.dy * std::cos(angle);
        middle.dx = dx;
        middle.dy = dy;
        Track candidate = document.tracks[0];
        CHECK(!lay_along_skeleton(candidate, bent, why));
        CHECK(!why.empty());
        // Nothing was written: the caller's copy still reads as it did.
        CHECK(candidate.elements[0].length == document.tracks[0].elements[0].length);
    }
    {   // A transition that does not run out into the straight after it is a kink.
        Document other;
        Track track;
        track.id = mint_track(other);
        track.anchor = AtPose{0.0, 0.0, 0.0};
        track.elements = {line(other, 100.0), arc(other, 400.0, 1, 100.0),
                          Element{mint_element(other), Kind::Clothoid, 400.0, 1, 40.0, Free{}},
                          line(other, 100.0)};
        other.tracks.push_back(track);
        const Solution solution = solve(other);
        Skeleton none;
        CHECK(!skeleton_of(other.tracks[0], *find_track(solution, track.id), none, why));
    }
    {   // Two arcs turning opposite ways inside one corner is an S, not a basket.
        Document other;
        Track track;
        track.id = mint_track(other);
        track.anchor = AtPose{0.0, 0.0, 0.0};
        track.elements = {line(other, 100.0), arc(other, 400.0, 1, 100.0),
                          arc(other, 400.0, -1, 100.0), line(other, 100.0)};
        other.tracks.push_back(track);
        const Solution solution = solve(other);
        Skeleton none;
        CHECK(!skeleton_of(other.tracks[0], *find_track(solution, track.id), none, why));
    }
}

/// The everyday main-line corner: straight, transition, arc, transition, straight.
/// The transitions are authored, so they keep their lengths; the arc takes what
/// is left of the turn.
void a_corner_with_transitions_re_fits() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.anchor = AtPose{0.0, 0.0, 0.0};
    track.elements = {
        line(document, 400.0),
        Element{mint_element(document), Kind::Clothoid, 600.0, 1, 80.0, Free{}},
        arc(document, 600.0, 1, 300.0),
        Element{mint_element(document), Kind::Clothoid, 0.0, 1, 80.0, Free{}},
        line(document, 400.0)};
    document.tracks.push_back(track);

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, track.id);
    CHECK(laid != nullptr && before.diagnostics.empty());
    const double first_azimuth = azimuth(laid->elements[0]);

    Skeleton skeleton;
    std::string why;
    const bool got = skeleton_of(document.tracks[0], *laid, skeleton, why);
    CHECK(got);
    if (!got) {
        std::printf("  why: %s\n", why.c_str());
        return;
    }
    CHECK(skeleton.lines.size() == 2);

    // Put it back untouched: everything comes out as it went in.
    Track rebuilt = document.tracks[0];
    CHECK(lay_along_skeleton(rebuilt, skeleton, why));
    for (std::size_t i = 0; i < rebuilt.elements.size(); ++i) {
        CHECK(near(rebuilt.elements[i].length, document.tracks[0].elements[i].length, 1e-6));
    }

    // Now swing the second straight by 3°: the transitions keep their lengths,
    // the arc takes up the difference, and the first straight does not budge.
    const double angle = 3.0 * std::numbers::pi / 180.0;
    Support& second = skeleton.lines[1];
    const double dx = second.dx * std::cos(angle) - second.dy * std::sin(angle);
    const double dy = second.dx * std::sin(angle) + second.dy * std::cos(angle);
    second.dx = dx;
    second.dy = dy;

    Track moved = document.tracks[0];
    CHECK(lay_along_skeleton(moved, skeleton, why));
    CHECK(near(moved.elements[1].length, 80.0, 1e-12));
    CHECK(near(moved.elements[3].length, 80.0, 1e-12));
    CHECK(!near(moved.elements[2].length, 300.0, 1e-6));
    document.tracks[0] = moved;

    const Solution after = solve(document);
    CHECK(after.diagnostics.empty());
    const SolvedTrack* again = find_track(after, track.id);
    CHECK(again != nullptr);
    CHECK(near(azimuth(again->elements[0]), first_azimuth, 1e-12));
    // The transition still hands the arc exactly the curvature it eased to.
    CHECK(near(again->elements[1].k1, 1.0 / 600.0, 1e-12));
    CHECK(near(again->elements[2].k0, 1.0 / 600.0, 1e-12));
}

/// A basket curve — two arcs of different radii turning the same way. Re-fitting
/// shares the corner's turn out between them in the proportions they were drawn
/// with, so it stays the same basket, tighter or wider.
void a_basket_curve_keeps_its_proportions() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.anchor = AtPose{0.0, 0.0, 0.0};
    track.elements = {line(document, 400.0), arc(document, 800.0, 1, 160.0),
                      arc(document, 400.0, 1, 120.0), line(document, 400.0)};
    document.tracks.push_back(track);

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, track.id);
    CHECK(laid != nullptr && before.diagnostics.empty());
    const double wide = 160.0 / 800.0;   // what each arc turns, as drawn
    const double tight = 120.0 / 400.0;
    const double first_azimuth = azimuth(laid->elements[0]);

    Skeleton skeleton;
    std::string why;
    const bool got = skeleton_of(document.tracks[0], *laid, skeleton, why);
    CHECK(got);
    if (!got) {
        std::printf("  why: %s\n", why.c_str());
        return;
    }

    Track rebuilt = document.tracks[0];
    CHECK(lay_along_skeleton(rebuilt, skeleton, why));
    CHECK(near(rebuilt.elements[1].length, 160.0, 1e-9));
    CHECK(near(rebuilt.elements[2].length, 120.0, 1e-9));

    // Swing the far straight: both arcs give, in proportion, and neither radius
    // changes.
    const double angle = 5.0 * std::numbers::pi / 180.0;
    Support& second = skeleton.lines[1];
    const double dx = second.dx * std::cos(angle) - second.dy * std::sin(angle);
    const double dy = second.dx * std::sin(angle) + second.dy * std::cos(angle);
    second.dx = dx;
    second.dy = dy;

    Track moved = document.tracks[0];
    CHECK(lay_along_skeleton(moved, skeleton, why));
    CHECK(moved.elements[1].radius == 800.0);
    CHECK(moved.elements[2].radius == 400.0);
    const double now_wide = moved.elements[1].length / 800.0;
    const double now_tight = moved.elements[2].length / 400.0;
    CHECK(!near(now_wide, wide, 1e-9));
    CHECK(near(now_wide / now_tight, wide / tight, 1e-9));

    document.tracks[0] = moved;
    const Solution after = solve(document);
    CHECK(after.diagnostics.empty());
    CHECK(near(azimuth(find_track(after, track.id)->elements[0]), first_azimuth, 1e-12));
}

/// Changing a radius is the corner's business and nobody else's: the straights
/// on either side give, and nothing past them moves a millimetre.
void a_new_radius_is_absorbed_by_its_own_corner() {
    Document document;
    const TrackId id = add_track(document, {line(document, 300.0), arc(document, 400.0, 1, 210.0),
                                            line(document, 250.0), arc(document, 300.0, -1, 180.0),
                                            line(document, 200.0)});

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, id);
    CHECK(laid != nullptr);
    const double last_azimuth = azimuth(laid->elements[4]);
    const double end_x = laid->end.x;
    const double end_y = laid->end.y;
    const double second_arc_length = document.tracks[0].elements[3].length;

    // The lines are taken off the track as it stands, then the radius changes.
    Skeleton skeleton;
    std::string why;
    CHECK(skeleton_of(document.tracks[0], *laid, skeleton, why));
    document.tracks[0].elements[1].radius = 250.0;

    CHECK(lay_along_skeleton(document.tracks[0], skeleton, why));

    const Solution after = solve(document);
    const SolvedTrack* again = find_track(after, id);
    CHECK(again != nullptr && after.diagnostics.empty());

    // A tighter arc turns the same corner, so it is shorter and the straights on
    // either side of it grow. The corner beyond is untouched, and so is the end.
    CHECK(near(again->elements[1].k0, 1.0 / 250.0, 1e-12));
    CHECK(near(document.tracks[0].elements[3].length, second_arc_length, 1e-9));
    CHECK(near(azimuth(again->elements[4]), last_azimuth, 1e-12));
    CHECK(near(again->end.x, end_x, 1e-9));
    CHECK(near(again->end.y, end_y, 1e-9));
    CHECK(near(again->start.x, laid->start.x, 1e-9));
}

/// A track that finishes mid-curve — laid by clicking and then given an arc, so
/// there is no straight after it. The corner at the end has nothing on its far
/// side to be fitted against, so it keeps what it was given and the track simply
/// ends where it takes it. Turning it the other way is an ordinary edit, not a
/// reason to refuse the whole chain.
void a_chain_ending_on_a_curve_still_re_fits() {
    Document document;
    const TrackId id = add_track(document, {line(document, 300.0), arc(document, 400.0, 1, 210.0),
                                            line(document, 250.0), arc(document, 300.0, -1, 180.0)});

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, id);
    CHECK(laid != nullptr && before.diagnostics.empty());
    const double first_azimuth = azimuth(laid->elements[0]);

    Skeleton skeleton;
    std::string why;
    const bool got = skeleton_of(document.tracks[0], *laid, skeleton, why);
    CHECK(got);
    if (!got) {
        std::printf("  why: %s\n", why.c_str());
        return;
    }
    CHECK(skeleton.lines.size() == 2);

    // Turn the trailing arc the other way. Everything before it stays put; the
    // arc keeps the length it was given and takes the track off elsewhere.
    document.tracks[0].elements[3].hand = 1;
    CHECK(lay_along_skeleton(document.tracks[0], skeleton, why));
    CHECK(near(document.tracks[0].elements[3].length, 180.0, 1e-12));

    const Solution after = solve(document);
    const SolvedTrack* again = find_track(after, id);
    CHECK(again != nullptr && after.diagnostics.empty());
    CHECK(near(azimuth(again->elements[0]), first_azimuth, 1e-12));
    CHECK(near(again->elements[2].length, laid->elements[2].length, 1e-9));
    CHECK(near(again->elements[3].start.x, laid->elements[3].start.x, 1e-9));
    CHECK(near(again->elements[3].k0, 1.0 / 300.0, 1e-12));
}

/// And one that starts mid-curve: a branch leaving a frog on a curve. The corner
/// at the front rides along, and the anchor is worked back through it.
void a_chain_starting_on_a_curve_carries_its_head_along() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.anchor = AtPose{10.0, 20.0, 0.3};
    track.elements = {arc(document, 250.0, -1, 90.0), line(document, 300.0),
                      arc(document, 400.0, 1, 210.0), line(document, 250.0)};
    document.tracks.push_back(track);

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, track.id);
    CHECK(laid != nullptr && before.diagnostics.empty());

    Skeleton skeleton;
    std::string why;
    const bool got = skeleton_of(document.tracks[0], *laid, skeleton, why);
    CHECK(got);
    if (!got) {
        std::printf("  why: %s\n", why.c_str());
        return;
    }
    CHECK(skeleton.lines.size() == 2);

    // Untouched, it comes back exactly as it was — anchor included.
    Track rebuilt = document.tracks[0];
    CHECK(lay_along_skeleton(rebuilt, skeleton, why));
    CHECK(near(rebuilt.elements[0].length, 90.0, 1e-12));
    CHECK(near(std::get<AtPose>(rebuilt.anchor).x, 10.0, 1e-9));
    CHECK(near(std::get<AtPose>(rebuilt.anchor).y, 20.0, 1e-9));
    CHECK(near(std::get<AtPose>(rebuilt.anchor).az, 0.3, 1e-9));

    // Tighten the interior corner: the head keeps its length, and the straights
    // beside the corner give.
    document.tracks[0].elements[2].radius = 250.0;
    CHECK(lay_along_skeleton(document.tracks[0], skeleton, why));
    CHECK(near(document.tracks[0].elements[0].length, 90.0, 1e-12));
    const Solution after = solve(document);
    CHECK(after.diagnostics.empty());
    const SolvedTrack* again = find_track(after, track.id);
    CHECK(near(again->elements[1].start.x, laid->elements[1].start.x, 1e-9));
    CHECK(near(again->elements[2].k0, 1.0 / 250.0, 1e-12));
}

/// The same chain, but hanging off a turnout's frog. Now the head corner may not
/// be slid anywhere — it leaves the frog, and that is that — so the line under
/// the straight behind it follows wherever the curve comes out, and the straight
/// gives up the difference in its length. Which is the only thing here that can.
void a_branch_off_a_frog_lets_its_first_straight_give() {
    Document document;

    TurnoutType type;
    type.name = "Rz 1:9 R190";
    type.crossing_n = 9.0;
    type.length = 27.138;
    type.blade = BladeSpec{0.005, 0.070};
    type.pieces = {
        TurnoutPieceSpec{0, 3.3108, 0.0, 0.0, 0.0},
        TurnoutPieceSpec{1, 7.188, 190.0, 190.0, 0.0},
        TurnoutPieceSpec{2, 190.0 * std::atan(1.0 / 9.0) - 7.188, 190.0, 190.0, 0.0},
        TurnoutPieceSpec{3, 2.7808, 0.0, 0.0, 0.0},
    };
    document.turnout_types.push_back(type);

    const TrackId main = add_track(document, {line(document, 1000.0)});

    TurnoutPlacement placement;
    placement.id = mint_turnout(document);
    placement.type = type.name;
    placement.on = main;
    placement.station = 300.0;
    placement.hand = -1;
    document.turnouts.push_back(placement);

    Track branch;
    branch.id = mint_track(document);
    branch.name = "odnoga";
    branch.anchor = AtPort{placement.id, Port::Frog};
    // straight off the frog it is not: the branch carries on curving, then runs out
    branch.elements = {arc(document, 300.0, -1, 120.0), line(document, 400.0),
                       arc(document, 500.0, 1, 150.0), line(document, 300.0)};
    document.tracks.push_back(branch);

    const Solution before = solve(document);
    const SolvedTrack* laid = find_track(before, branch.id);
    const SolvedTurnout* turnout = find_turnout(before, placement.id);
    CHECK(laid != nullptr && turnout != nullptr && before.diagnostics.empty());
    if (laid == nullptr || turnout == nullptr) {
        return;
    }
    const double straight_before = document.tracks[1].elements[1].length;

    Skeleton skeleton;
    std::string why;
    const bool got = skeleton_of(document.tracks[1], *laid, skeleton, why);
    CHECK(got);
    if (!got) {
        std::printf("  why: %s\n", why.c_str());
        return;
    }
    CHECK(skeleton.pinned);

    // Open out the curve leaving the frog. This used to be refused outright.
    document.tracks[1].elements[0].radius = 400.0;
    const bool relaid = lay_along_skeleton(document.tracks[1], skeleton, why);
    CHECK(relaid);
    if (!relaid) {
        std::printf("  why: %s\n", why.c_str());
        return;
    }

    // The head kept its length — it is authored, not fitted — and the straight
    // after it took up what the wider curve cost.
    CHECK(near(document.tracks[1].elements[0].length, 120.0, 1e-12));
    CHECK(std::abs(document.tracks[1].elements[1].length - straight_before) > 1.0);

    const Solution after = solve(document);
    CHECK(after.diagnostics.empty());
    const SolvedTrack* again = find_track(after, branch.id);
    CHECK(again != nullptr);
    if (again == nullptr) {
        return;
    }
    // Still on the frog, and the far end still runs where it ran.
    CHECK(near(again->start.x, turnout->frog.x, 1e-9));
    CHECK(near(again->start.y, turnout->frog.y, 1e-9));
    CHECK(near(again->elements[3].start.hx, laid->elements[3].start.hx, 1e-9));
    CHECK(near(again->elements[3].start.hy, laid->elements[3].start.hy, 1e-9));
}

/// A transition curve's radius is the one it ends on, so it belongs to what
/// comes after it. The transition running out of an arc into a straight ends
/// flat, and no amount of editing the arc may put the arc's radius on it — that
/// would be a transition easing from R to R, which eases nothing.
void a_joint_only_ever_looks_forward() {
    Document document;
    Track track;
    track.id = mint_track(document);
    track.anchor = AtPose{0.0, 0.0, 0.0};
    track.elements = {
        line(document, 400.0),
        Element{mint_element(document), Kind::Clothoid, 600.0, 1, 80.0, Free{}},
        arc(document, 600.0, 1, 300.0),
        Element{mint_element(document), Kind::Clothoid, 0.0, 1, 80.0, Free{}},
        line(document, 400.0)};
    document.tracks.push_back(track);

    // Editing the arc carries its radius back onto the transition that eases into
    // it, and leaves the one running out of it flat.
    document.tracks[0].elements[2].radius = 450.0;
    sync_joint(document.tracks[0], 2);
    CHECK(document.tracks[0].elements[1].radius == 450.0);
    CHECK(document.tracks[0].elements[3].radius == 0.0);

    // Turning it the other way carries the hand back the same way.
    document.tracks[0].elements[2].hand = -1;
    sync_joint(document.tracks[0], 2);
    CHECK(document.tracks[0].elements[1].hand == -1);

    // And the chain still lays: nothing anywhere eases from a radius to itself.
    const Solution solution = solve(document);
    CHECK(solution.diagnostics.empty());

    // Editing the entry transition carries forward onto its arc.
    document.tracks[0].elements[1].radius = 700.0;
    sync_joint(document.tracks[0], 1);
    CHECK(document.tracks[0].elements[2].radius == 700.0);

    // A transition that ends flat has no joint to carry anywhere.
    document.tracks[0].elements[3].hand = 1;
    sync_joint(document.tracks[0], 3);
    CHECK(document.tracks[0].elements[4].kind == Kind::Line);
    CHECK(solve(document).diagnostics.empty());
}

/// Two straights end to end are one straight, and the solver says so.
void touching_straights_are_reported() {
    Document document;
    add_track(document, {line(document, 100.0), line(document, 150.0)});
    const Solution solution = solve(document);
    bool found = false;
    for (const auto& diagnostic : solution.diagnostics) {
        found = found || diagnostic.code == Code::TouchingStraights;
    }
    CHECK(found);
    // It is a rule about writing track down, not about the geometry: it still lays.
    CHECK(find_track(solution, document.tracks[0].id)->elements.size() == 2);
}

}  // namespace

int main() {
    RUN(a_track_survives_being_taken_apart_and_put_back);
    RUN(an_arc_past_a_half_circle_stays_where_it_was_drawn);
    RUN(a_half_circle_is_laid_and_not_refused);
    RUN(swinging_one_straight_leaves_the_others_pointing_where_they_did);
    RUN(a_move_that_cannot_be_laid_is_refused);
    RUN(a_corner_with_transitions_re_fits);
    RUN(a_basket_curve_keeps_its_proportions);
    RUN(a_new_radius_is_absorbed_by_its_own_corner);
    RUN(a_chain_ending_on_a_curve_still_re_fits);
    RUN(a_chain_starting_on_a_curve_carries_its_head_along);
    RUN(a_branch_off_a_frog_lets_its_first_straight_give);
    RUN(a_joint_only_ever_looks_forward);
    RUN(touching_straights_are_reported);
    return REPORT();
}
