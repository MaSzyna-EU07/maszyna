/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
#include <cmath>
#include <cstdio>
#include <numbers>
#include "check.hpp"
import eu07.editor.plan_layout;
import eu07.editor.plan_sketch;

// What one track keeps against another, and what clicking along the ground
// works out at the click. Both are the same idea from two ends: geometry is
// decided once, in numbers, and nothing is left pointing at a drawing.



using namespace editor::plan;

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

/// A straight due north from the origin, 1000 m of it.
TrackId add_main(Document& document, ElementId& out_element) {
    Track track;
    track.id = mint_track(document);
    track.name = "tor 1";
    track.anchor = AtPose{0.0, 0.0, 0.0};
    track.elements = {line(document, 1000.0)};
    out_element = track.elements.front().id;
    document.tracks.push_back(track);
    return track.id;
}

bool has(const Solution& solution, Code code) {
    for (const auto& diagnostic : solution.diagnostics) {
        if (diagnostic.code == code) {
            return true;
        }
    }
    return false;
}

// -------------------------------------------------------------------------

/// A corner rounded at the click: the tangent comes off both straights, and the
/// arc turns exactly as far as the corner bends.
void a_corner_is_rounded_with_a_tangent_arc() {
    // Due north, then due east: a 90° corner to the right.
    const Corner corner = fit_corner(0.0, 1.0, 1.0, 0.0, 300.0);
    CHECK(corner.ok);
    CHECK(corner.hand == -1);
    CHECK(near(corner.deflection, -std::numbers::pi / 2.0));
    CHECK(near(corner.tangent, 300.0));  // R·tan(45°)
    CHECK(near(corner.arc_length, 300.0 * std::numbers::pi / 2.0));

    // Straight on is not a corner, and neither is doubling back.
    CHECK(!fit_corner(0.0, 1.0, 0.0, 1.0, 300.0).ok);
    CHECK(!fit_corner(0.0, 1.0, 0.0, -1.0, 300.0).ok);
    CHECK(!fit_corner(0.0, 1.0, 1.0, 0.0, 0.0).ok);
}

/// The corner really does come out tangent: laying the three elements it
/// produces lands the track on the second leg, heading along it.
void a_rounded_corner_lands_on_the_next_leg() {
    Document document;
    const Corner corner = fit_corner(0.0, 1.0, 1.0, 0.0, 300.0);

    Track track;
    track.id = mint_track(document);
    track.anchor = AtPose{0.0, 0.0, 0.0};
    track.elements = {line(document, 1000.0 - corner.tangent),
                      arc(document, 300.0, corner.hand, corner.arc_length),
                      line(document, 800.0 - corner.tangent)};
    document.tracks.push_back(track);

    const Solution solution = solve(document);
    CHECK(solution.diagnostics.empty());
    const SolvedTrack* laid = find_track(solution, track.id);
    CHECK(laid != nullptr);
    // The corner was at (0, 1000); the second leg runs east from there to (800, 1000).
    CHECK(near(laid->end.x, 800.0, 1e-6));
    CHECK(near(laid->end.y, 1000.0, 1e-6));
    CHECK(near(laid->end.hx, 1.0, 1e-9));
    CHECK(near(laid->end.hy, 0.0, 1e-9));
}

/// Międzytorze, said once: the held track is moved bodily onto the offset, and
/// stays where it was put along it.
void a_track_held_parallel_is_moved_onto_the_offset() {
    Document document;
    ElementId reference{ElementId::none};
    add_main(document, reference);

    Track siding;
    siding.id = mint_track(document);
    siding.name = "tor 2";
    // Put it well off the offset, and part way up the main track.
    siding.anchor = AtPose{40.0, 300.0, 0.0};
    siding.elements = {line(document, 500.0)};
    siding.elements.front().hold = Parallel{reference, 4.75};
    document.tracks.push_back(siding);

    const Solution solution = solve(document);
    CHECK(solution.diagnostics.empty());

    const SolvedTrack* laid = find_track(solution, siding.id);
    CHECK(laid != nullptr && laid->elements.size() == 1);
    // The main track heads north, so left of its travel is -x.
    CHECK(near(laid->elements[0].start.x, -4.75, 1e-9));
    // Nothing slid it along: it kept the y it was anchored at.
    CHECK(near(laid->elements[0].start.y, 300.0, 1e-9));
    CHECK(near(laid->elements[0].start.hy, 1.0, 1e-9));
    CHECK(near(laid->length, 500.0));

    // The offset is signed: the other side is the other side.
    document.tracks.back().elements.front().hold = Parallel{reference, -4.75};
    const Solution mirrored = solve(document);
    const SolvedTrack* other = find_track(mirrored, document.tracks.back().id);
    CHECK(other != nullptr && near(other->elements[0].start.x, 4.75, 1e-9));
}

/// Parallel to an arc is concentric with it: same centre, R∓d.
void a_track_held_parallel_to_an_arc_is_concentric() {
    Document document;
    Track main;
    main.id = mint_track(document);
    main.name = "tor 1";
    main.anchor = AtPose{0.0, 0.0, 0.0};
    main.elements = {arc(document, 500.0, 1, 400.0)};
    const ElementId reference = main.elements.front().id;
    document.tracks.push_back(main);

    Track siding;
    siding.id = mint_track(document);
    siding.name = "tor 2";
    siding.anchor = AtPose{-60.0, 10.0, 0.0};
    // The radius and hand asked for here are beside the point: the reference
    // fixes the shape.
    siding.elements = {arc(document, 999.0, -1, 200.0)};
    siding.elements.front().hold = Parallel{reference, 50.0};
    document.tracks.push_back(siding);

    const Solution solution = solve(document);
    CHECK(solution.diagnostics.empty());

    const SolvedTrack* laid = find_track(solution, siding.id);
    CHECK(laid != nullptr && laid->elements.size() == 1);
    // Left of a left-hand curve is the inside: R - d.
    CHECK(near(laid->elements[0].k0, 1.0 / 450.0, 1e-12));
    // The main track's centre of curvature is 500 m to the left of its start.
    CHECK(near(std::hypot(laid->elements[0].start.x + 500.0, laid->elements[0].start.y),
               450.0, 1e-6));
    CHECK(near(laid->length, 200.0));
}

/// A hold is a reference like any other: it orders the solve, and a circle of
/// them is reported rather than relaxed at.
void holds_are_ordered_and_cycles_are_caught() {
    Document document;
    ElementId reference{ElementId::none};
    add_main(document, reference);

    // Held to a track that is itself held: laid third, and still exact.
    Track second;
    second.id = mint_track(document);
    second.name = "tor 2";
    second.anchor = AtPose{40.0, 0.0, 0.0};
    second.elements = {line(document, 400.0)};
    second.elements.front().hold = Parallel{reference, 4.75};
    const ElementId middle = second.elements.front().id;
    document.tracks.push_back(second);

    Track third;
    third.id = mint_track(document);
    third.name = "tor 3";
    third.anchor = AtPose{80.0, 0.0, 0.0};
    third.elements = {line(document, 400.0)};
    third.elements.front().hold = Parallel{middle, 4.75};
    document.tracks.push_back(third);

    const Solution solution = solve(document);
    CHECK(solution.diagnostics.empty());
    const SolvedTrack* laid = find_track(solution, third.id);
    CHECK(laid != nullptr && near(laid->elements[0].start.x, -9.5, 1e-9));

    // Now point the first one back at the last: a circle.
    document.tracks[0].elements[0].hold =
        Parallel{document.tracks[2].elements[0].id, 4.75};
    CHECK(has(solve(document), Code::CyclicDependency));
}

/// Two elements of one movable track cannot both decide where it goes.
void two_holds_on_one_movable_track_conflict() {
    Document document;
    ElementId reference{ElementId::none};
    add_main(document, reference);

    Track siding;
    siding.id = mint_track(document);
    siding.name = "tor 2";
    siding.anchor = AtPose{40.0, 0.0, 0.0};
    siding.elements = {line(document, 200.0), line(document, 200.0)};
    siding.elements[0].hold = Parallel{reference, 4.75};
    siding.elements[1].hold = Parallel{reference, 9.50};
    document.tracks.push_back(siding);

    CHECK(has(solve(document), Code::HoldConflict));
}

/// A branch pinned to a turnout's frog cannot be moved onto the offset, so the
/// corner between the frog and the held straight takes up the difference — the
/// same fit as pulling a straight about by hand, and no full turn anywhere.
void a_pinned_branch_fits_its_way_onto_the_offset() {
    Document document;

    TurnoutType type;
    type.name = "Rz 1:9 R190";
    type.crossing_n = 9.0;
    type.length = 27.138;  // the catalogue's a + b
    type.blade = BladeSpec{0.005, 0.070};
    type.pieces = {
        // przediglicowy, then blade and curve as one arc of the crossing angle, then
        // the frog rail: a = 13,834 and b = 13,304 with R*tan(alfa/2) = 10,5232
        TurnoutPieceSpec{0, 3.3108, 0.0, 0.0, 0.0},
        TurnoutPieceSpec{1, 7.188, 190.0, 190.0, 0.0},
        TurnoutPieceSpec{2, 190.0 * std::atan(1.0 / 9.0) - 7.188, 190.0, 190.0, 0.0},
        TurnoutPieceSpec{3, 2.7808, 0.0, 0.0, 0.0},
    };
    document.turnout_types.push_back(type);

    ElementId reference{ElementId::none};
    const TrackId main = add_main(document, reference);

    TurnoutPlacement placement;
    placement.id = mint_turnout(document);
    placement.type = type.name;
    placement.on = main;
    placement.station = 300.0;
    placement.hand = -1;  // diverging to the right of a northbound track
    document.turnouts.push_back(placement);

    // Off the frog: a straight, a corner, and the straight that is to run beside
    // the main track. Nothing about it is right to begin with.
    Track branch;
    branch.id = mint_track(document);
    branch.name = "bocznica";
    branch.anchor = AtPort{placement.id, Port::Frog};
    branch.elements = {line(document, 40.0), arc(document, 300.0, 1, 30.0),
                       line(document, 400.0)};
    branch.elements[2].hold = Parallel{reference, -4.75};
    document.tracks.push_back(branch);

    const Solution solution = solve(document);
    for (const auto& diagnostic : solution.diagnostics) {
        std::printf("  diag: %s\n", diagnostic.text.c_str());
    }
    CHECK(!has(solution, Code::OffsetNotHeld));

    const SolvedTrack* laid = find_track(solution, branch.id);
    CHECK(laid != nullptr && laid->elements.size() == 3);
    const SolvedTurnout* turnout = find_turnout(solution, placement.id);
    CHECK(turnout != nullptr && turnout->valid);

    // It still starts exactly on the frog: the fit moved lengths, not the branch.
    CHECK(near(laid->start.x, turnout->frog.x, 1e-6));
    CHECK(near(laid->start.y, turnout->frog.y, 1e-6));
    // And the held straight really lies 4.75 m to the right of the main track,
    // pointing the same way.
    CHECK(near(laid->elements[2].start.x, 4.75, 1e-6));
    CHECK(near(laid->elements[2].start.hy, 1.0, 1e-9));
    // The corner took up the difference without looping the long way round.
    CHECK(laid->elements[1].length < 300.0 * std::numbers::pi);
}

/// A reference that is not there is said so, and the track is still laid.
void a_missing_reference_is_reported() {
    Document document;
    Track siding;
    siding.id = mint_track(document);
    siding.name = "tor 2";
    siding.anchor = AtPose{0.0, 0.0, 0.0};
    siding.elements = {line(document, 100.0)};
    siding.elements.front().hold = Parallel{static_cast<ElementId>(9999), 4.75};
    document.tracks.push_back(siding);

    const Solution solution = solve(document);
    CHECK(has(solution, Code::UnknownReference));
    const SolvedTrack* laid = find_track(solution, siding.id);
    CHECK(laid != nullptr && laid->elements.size() == 1);
}

}  // namespace

int main() {
    RUN(a_corner_is_rounded_with_a_tangent_arc);
    RUN(a_rounded_corner_lands_on_the_next_leg);
    RUN(a_track_held_parallel_is_moved_onto_the_offset);
    RUN(a_track_held_parallel_to_an_arc_is_concentric);
    RUN(holds_are_ordered_and_cycles_are_caught);
    RUN(two_holds_on_one_movable_track_conflict);
    RUN(a_pinned_branch_fits_its_way_onto_the_offset);
    RUN(a_missing_reference_is_reported);
    return REPORT();
}
