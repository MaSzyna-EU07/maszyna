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
#include <string>
#include <variant>
#include <vector>
#include "check.hpp"
import eu07.editor.plan_layout;
import eu07.editor.plan_sketch;

// Putting a joint into a chain, and taking a piece out of one. The chain is
// relative, so both are arithmetic on lengths: what has to be checked is that the
// ground does not move, that the chain still reads the way the fit needs it to,
// that ids survive, and that what cannot be done is refused with a reason instead
// of quietly laid wrong.

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
Element clothoid(Document& document, double end_radius, int hand, double metres) {
    return Element{mint_element(document), Kind::Clothoid, end_radius, hand, metres, Free{}};
}

TrackId add_track(Document& document, const char* name, std::vector<Element> elements) {
    Track track;
    track.id = mint_track(document);
    track.name = name;
    track.anchor = AtPose{0.0, 0.0, 0.0};  // due north from the origin
    track.elements = std::move(elements);
    document.tracks.push_back(track);
    return track.id;
}

/// A plain Rz 1:9 R190, as in the graph tests.
void add_catalogue(Document& document) {
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
}

TurnoutId add_turnout(Document& document, TrackId on, double station) {
    TurnoutPlacement placement;
    placement.id = mint_turnout(document);
    placement.type = "Rz 1:9 R190";
    placement.on = on;
    placement.station = station;
    placement.hand = 1;
    document.turnouts.push_back(placement);
    return placement.id;
}

// -------------------------------------------------------------------------

/// An arc divided in two: one chain, one more element, and the same ground. Both
/// halves turn at the radius they always did, and the track ends exactly where it
/// ended before.
void an_arc_takes_a_joint() {
    Document document;
    const TrackId main = add_track(
        document, "tor 1",
        {line(document, 100.0), arc(document, 400.0, -1, 200.0), line(document, 100.0)});
    Solution solution = solve(document);
    const geometry::Pose end_before = find_track(solution, main)->end;
    const double length_before = find_track(solution, main)->length;

    const JointOutcome outcome = insert_joint(document, solution, main, 200.0);
    CHECK(outcome.ok);
    CHECK(outcome.why.empty());
    CHECK(outcome.added != ElementId::none);
    // one track, one more element - not two tracks
    CHECK(document.tracks.size() == 1);

    const Track* authored = find_track(document, main);
    CHECK(authored != nullptr && authored->elements.size() == 4);
    CHECK(authored->elements[1].kind == Kind::Arc && near(authored->elements[1].length, 100.0));
    CHECK(authored->elements[2].kind == Kind::Arc && near(authored->elements[2].length, 100.0));
    CHECK(authored->elements[2].id == outcome.added);
    CHECK(near(authored->elements[2].radius, 400.0) && authored->elements[2].hand == -1);

    solution = solve(document);
    const SolvedTrack* laid = find_track(solution, main);
    CHECK(laid != nullptr && laid->elements.size() == 4);
    CHECK(near(laid->length, length_before, 1e-6));
    CHECK(near(laid->end.x, end_before.x, 1e-6) && near(laid->end.y, end_before.y, 1e-6));
    CHECK(near(laid->end.hx, end_before.hx, 1e-9) && near(laid->end.hy, end_before.hy, 1e-9));
    for (const Diagnostic& diagnostic : solution.diagnostics) {
        CHECK(diagnostic.code != Code::TouchingStraights);
        CHECK(diagnostic.code != Code::DegenerateElement);
    }
}

/// A transition curve divided in two: the near half eases to the radius the curve
/// had reached, the far half goes on to where the whole one was going. The curvature
/// at the new joint is the one that was there all along.
void a_transition_takes_a_joint() {
    Document document;
    const TrackId main = add_track(document, "tor 1",
                                   {line(document, 100.0), clothoid(document, 400.0, 1, 80.0),
                                    arc(document, 400.0, 1, 100.0)});
    Solution solution = solve(document);
    const geometry::Pose end_before = find_track(solution, main)->end;

    const JointOutcome outcome = insert_joint(document, solution, main, 140.0);
    CHECK(outcome.ok);
    CHECK(document.tracks.size() == 1);

    const Track* authored = find_track(document, main);
    CHECK(authored != nullptr && authored->elements.size() == 4);
    CHECK(authored->elements[1].kind == Kind::Clothoid);
    CHECK(authored->elements[2].kind == Kind::Clothoid);
    // half way along a transition from straight to R=400 the radius is twice that
    CHECK(near(authored->elements[1].radius, 800.0, 1e-6));
    CHECK(authored->elements[1].hand == 1);
    CHECK(near(authored->elements[2].radius, 400.0));
    CHECK(near(authored->elements[1].length + authored->elements[2].length, 80.0, 1e-9));

    solution = solve(document);
    const SolvedTrack* laid = find_track(solution, main);
    CHECK(laid != nullptr);
    for (const Diagnostic& diagnostic : solution.diagnostics) {
        CHECK(diagnostic.code != Code::DegenerateElement);
    }
    // the sampling of two halves is not the sampling of the whole, so the far end moves
    // by the width of a rounding error and no more
    CHECK(near(laid->end.x, end_before.x, 1e-3) && near(laid->end.y, end_before.y, 1e-3));
    const SolvedElement* second = &laid->elements[2];
    CHECK(near(second->k0, laid->elements[1].k1, 1e-12));
}

/// A straight takes no joint: two straights end to end are one straight, and the
/// skeleton the fit works from will not read a chain that says so.
void a_straight_takes_no_joint() {
    Document document;
    const TrackId main = add_track(document, "tor 1", {line(document, 1000.0)});
    const Solution solution = solve(document);

    const JointOutcome outcome = insert_joint(document, solution, main, 400.0);
    CHECK(!outcome.ok);
    CHECK(outcome.why.find("proste") != std::string::npos);
    CHECK(find_track(document, main)->elements.size() == 1);
}

/// Where there already is a joint, there is nothing to put.
void a_joint_is_not_put_twice() {
    Document document;
    const TrackId main = add_track(
        document, "tor 1", {arc(document, 500.0, 1, 300.0), arc(document, 500.0, 1, 200.0)});
    const Solution solution = solve(document);

    CHECK(!insert_joint(document, solution, main, 300.0).ok);
    CHECK(!insert_joint(document, solution, main, 300.02).ok);
    CHECK(find_track(document, main)->elements.size() == 2);
}

/// And not at either end of the track.
void the_ends_take_no_joint() {
    Document document;
    const TrackId main = add_track(document, "tor 1", {arc(document, 500.0, 1, 500.0)});
    const Solution solution = solve(document);

    CHECK(!insert_joint(document, solution, main, 0.0).ok);
    CHECK(!insert_joint(document, solution, main, 500.0).ok);
    CHECK(!insert_joint(document, solution, main, 0.01).ok);
    CHECK(find_track(document, main)->elements.size() == 1);
}

/// A turnout is one piece of geometry on one element, so a joint under it is refused
/// - and the type is named, so it is clear which one is in the way.
void a_turnout_keeps_its_element_whole() {
    Document document;
    add_catalogue(document);
    const TrackId main = add_track(document, "tor 1", {arc(document, 2000.0, 1, 1000.0)});
    add_turnout(document, main, 200.0);
    Solution solution = solve(document);

    const JointOutcome inside = insert_joint(document, solution, main, 210.0);
    CHECK(!inside.ok);
    CHECK(inside.why.find("rozjazd") != std::string::npos);

    // past it, the joint goes in and the turnout carries on standing where it stood
    const JointOutcome beyond = insert_joint(document, solution, main, 400.0);
    CHECK(beyond.ok);
    const TurnoutPlacement* placement = &document.turnouts.front();
    CHECK(placement->on == main && near(placement->station, 200.0));
    solution = solve(document);
    const SolvedTurnout* laid = find_turnout(solution, placement->id);
    CHECK(laid != nullptr && laid->valid);
}

/// A hold belongs to the element that was authored with it, and that is the half
/// which keeps the id: the reference still resolves and the far half stands free.
void a_hold_stays_with_the_element_that_has_its_id() {
    Document document;
    const TrackId main =
        add_track(document, "tor 1", {line(document, 200.0), arc(document, 600.0, 1, 300.0)});
    const ElementId curve = find_track(document, main)->elements.back().id;

    Track siding;
    siding.id = mint_track(document);
    siding.name = "tor 2";
    siding.anchor = AtPose{50.0, 0.0, 0.0};
    siding.elements = {arc(document, 600.0, 1, 100.0)};
    siding.elements.front().hold = Parallel{curve, 4.75};
    document.tracks.push_back(siding);

    Solution solution = solve(document);
    const JointOutcome outcome = insert_joint(document, solution, main, 350.0);
    CHECK(outcome.ok);

    solution = solve(document);
    for (const Diagnostic& diagnostic : solution.diagnostics) {
        CHECK(diagnostic.code != Code::UnknownReference);
    }
    CHECK(find_element(solution, curve) != nullptr);
    const Track* authored = find_track(document, main);
    CHECK(authored->elements[1].id == curve);
    CHECK(std::holds_alternative<Free>(authored->elements[2].hold));
}

/// Taking a piece out leaves the two ends where they stood, with nothing between
/// them: a gap is the ground between two tracks, not a hole in a chain.
void a_piece_is_taken_out() {
    Document document;
    const TrackId main = add_track(document, "tor 1", {line(document, 1000.0)});
    Solution solution = solve(document);

    const CutOutcome outcome = cut_track(document, solution, main, 300.0, 700.0);
    CHECK(outcome.ok);
    CHECK(document.tracks.size() == 2);

    solution = solve(document);
    const SolvedTrack* first = find_track(solution, main);
    const SolvedTrack* beyond = find_track(solution, outcome.second);
    CHECK(first != nullptr && beyond != nullptr);
    CHECK(near(first->length, 300.0));
    CHECK(near(beyond->length, 300.0));
    // the piece that is gone is exactly the distance between the two ends
    CHECK(near(std::hypot(beyond->start.x - first->end.x, beyond->start.y - first->end.y), 400.0,
               1e-4));
}

/// A cut that would swallow a turnout is refused, and the drawing is left alone.
void a_cut_over_a_turnout_is_refused() {
    Document document;
    add_catalogue(document);
    const TrackId main = add_track(document, "tor 1", {line(document, 1000.0)});
    add_turnout(document, main, 500.0);
    const Solution solution = solve(document);

    const CutOutcome outcome = cut_track(document, solution, main, 400.0, 600.0);
    CHECK(!outcome.ok);
    CHECK(outcome.why.find("rozjazd") != std::string::npos);
    CHECK(document.tracks.size() == 1);
    CHECK(document.turnouts.size() == 1);
    CHECK(near(find_track(document, main)->elements.front().length, 1000.0));
}

/// Two points the same way round, and a piece with no length, are not cuts.
void a_cut_needs_two_different_points() {
    Document document;
    const TrackId main = add_track(document, "tor 1", {line(document, 1000.0)});
    const Solution solution = solve(document);

    CHECK(!cut_track(document, solution, main, 400.0, 400.0).ok);
    // the other way round is the same cut
    const CutOutcome reversed = cut_track(document, solution, main, 700.0, 300.0);
    CHECK(reversed.ok);
    CHECK(document.tracks.size() == 2);
}

}  // namespace

int main() {
    RUN(an_arc_takes_a_joint);
    RUN(a_transition_takes_a_joint);
    RUN(a_straight_takes_no_joint);
    RUN(a_joint_is_not_put_twice);
    RUN(the_ends_take_no_joint);
    RUN(a_turnout_keeps_its_element_whole);
    RUN(a_hold_stays_with_the_element_that_has_its_id);
    RUN(a_piece_is_taken_out);
    RUN(a_cut_over_a_turnout_is_refused);
    RUN(a_cut_needs_two_different_points);
    return REPORT();
}
