/*
This Source Code Form is subject to the
terms of the Mozilla Public License, v.
2.0. If a copy of the MPL was not
distributed with this file, You can
obtain one at
http://mozilla.org/MPL/2.0/.
*/
#include <cmath>
#include <numbers>
#include "check.hpp"
import eu07.editor.plan_layout;

// What refers to what decides the order things are laid in. A turnout on the
// branch of a turnout is not a special case, and a reference that runs in a
// circle is reported rather than relaxed at for a fixed number of passes.



using namespace editor::plan;

namespace {

bool near(double a, double b, double tolerance = 1e-6) {
    return std::abs(a - b) <= tolerance;
}

bool has(const Solution& solution, Code code) {
    for (const auto& diagnostic : solution.diagnostics) {
        if (diagnostic.code == code) {
            return true;
        }
    }
    return false;
}

/// A plain Rz 1:9 R190, the everyday Polish turnout.
void add_catalogue(Document& document) {
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
}

TurnoutId add_turnout(Document& document, TrackId on, double station, int hand) {
    TurnoutPlacement placement;
    placement.id = mint_turnout(document);
    placement.type = "Rz 1:9 R190";
    placement.on = on;
    placement.station = station;
    placement.hand = hand;
    document.turnouts.push_back(placement);
    return placement.id;
}

TrackId add_straight_track(Document& document, const char* name, double x, double y,
                           double azimuth, double length) {
    Track track;
    track.id = mint_track(document);
    track.name = name;
    track.anchor = AtPose{x, y, azimuth};
    track.elements = {
        Element{mint_element(document), Kind::Line, 0.0, 0, length, Free{}}};
    document.tracks.push_back(track);
    return track.id;
}

// -------------------------------------------------------------------------

/// A branch anchored to a frog is laid after the turnout, which is laid after
/// its through track — whatever order they appear in the document.
void a_branch_follows_its_frog() {
    Document document;
    add_catalogue(document);

    // The branch is declared BEFORE the turnout and the through track it needs.
    Track branch;
    branch.id = mint_track(document);
    branch.name = "odnoga";
    branch.elements = {
        Element{mint_element(document), Kind::Line, 0.0, 0, 100.0, Free{}}};
    document.tracks.push_back(branch);

    const TrackId through = add_straight_track(document, "zasadniczy", 0.0, 0.0, 0.0, 1000.0);
    const TurnoutId turnout = add_turnout(document, through, 300.0, 1);
    document.tracks[0].anchor = AtPort{turnout, Port::Frog};

    const Solution solution = solve(document);

    CHECK(solution.diagnostics.empty());
    const SolvedTurnout* solved_turnout = find_turnout(solution, turnout);
    CHECK(solved_turnout != nullptr && solved_turnout->valid);

    const SolvedTrack* solved_branch = find_track(solution, branch.id);
    CHECK(solved_branch != nullptr);
    CHECK(solved_branch->elements.size() == 1);
    // The branch starts exactly at the frog, on the frog's heading. It was never
    // anywhere else, so there is nothing to pin back.
    CHECK(near(solved_branch->start.x, solved_turnout->frog.x));
    CHECK(near(solved_branch->start.y, solved_turnout->frog.y));
    CHECK(near(solved_branch->start.hx, solved_turnout->frog.hx));
    CHECK(near(solved_branch->start.hy, solved_turnout->frog.hy));
    // It diverges: the frog is off the through track's straight line.
    CHECK(std::abs(solved_turnout->frog.x) > 0.5);
}

/// A turnout on the branch of a turnout. The old solver ran a fixed two passes,
/// so this depth had no way to settle.
void a_turnout_on_a_branch_of_a_turnout() {
    Document document;
    add_catalogue(document);

    const TrackId through = add_straight_track(document, "zasadniczy", 0.0, 0.0, 0.0, 2000.0);
    const TurnoutId first = add_turnout(document, through, 300.0, 1);

    Track branch;
    branch.id = mint_track(document);
    branch.name = "odnoga 1";
    branch.anchor = AtPort{first, Port::Frog};
    branch.elements = {
        Element{mint_element(document), Kind::Line, 0.0, 0, 400.0, Free{}}};
    document.tracks.push_back(branch);

    const TurnoutId second = add_turnout(document, branch.id, 150.0, -1);

    Track branch2;
    branch2.id = mint_track(document);
    branch2.name = "odnoga 2";
    branch2.anchor = AtPort{second, Port::Frog};
    branch2.elements = {
        Element{mint_element(document), Kind::Arc, 300.0, -1, 120.0, Free{}}};
    document.tracks.push_back(branch2);

    const Solution solution = solve(document);

    CHECK(solution.diagnostics.empty());
    CHECK(find_turnout(solution, second) != nullptr);
    CHECK(find_turnout(solution, second)->valid);

    const SolvedTrack* deep = find_track(solution, branch2.id);
    CHECK(deep != nullptr);
    CHECK(deep->elements.size() == 1);
    CHECK(near(deep->start.x, find_turnout(solution, second)->frog.x));
    CHECK(near(deep->start.y, find_turnout(solution, second)->frog.y));
    // Three levels deep, well away from the through track.
    CHECK(std::abs(deep->start.x) > 1.0);
}

/// References that run in a circle are named, not relaxed at.
void a_cycle_is_reported() {
    // Two tracks, each held parallel to the other's element.
    Document document;
    const TrackId first = add_straight_track(document, "tor 1", 0.0, 0.0, 0.0, 500.0);
    const TrackId second = add_straight_track(document, "tor 2", 20.0, 0.0, 0.0, 500.0);
    document.tracks[0].elements[0].hold =
        Parallel{document.tracks[1].elements[0].id, 4.75};
    document.tracks[1].elements[0].hold =
        Parallel{document.tracks[0].elements[0].id, 4.75};
    (void)first;
    (void)second;

    const Solution solution = solve(document);
    CHECK(has(solution, Code::CyclicDependency));
}

/// A turnout whose catalogue type is missing says so, and the branch that hangs
/// off it says why it could not be laid — neither is silently dropped.
void a_missing_type_is_reported() {
    Document document;
    const TrackId through = add_straight_track(document, "zasadniczy", 0.0, 0.0, 0.0, 500.0);
    const TurnoutId turnout = add_turnout(document, through, 100.0, 1);  // no catalogue

    Track branch;
    branch.id = mint_track(document);
    branch.name = "odnoga";
    branch.anchor = AtPort{turnout, Port::Frog};
    branch.elements = {
        Element{mint_element(document), Kind::Line, 0.0, 0, 50.0, Free{}}};
    document.tracks.push_back(branch);

    const Solution solution = solve(document);
    CHECK(has(solution, Code::UnknownReference));
    CHECK(solution.diagnostics.size() >= 2);
    // Every track still has an entry, so a host can index by id without holes.
    CHECK(find_track(solution, branch.id) != nullptr);
    CHECK(find_track(solution, through) != nullptr);
}

/// Solving twice gives the same answer, and solving does not touch the document.
void solving_leaves_the_document_alone() {
    Document document;
    add_catalogue(document);
    const TrackId through = add_straight_track(document, "zasadniczy", 0.0, 0.0, 0.0, 1000.0);
    const TurnoutId turnout = add_turnout(document, through, 300.0, 1);
    Track branch;
    branch.id = mint_track(document);
    branch.anchor = AtPort{turnout, Port::Frog};
    branch.elements = {
        Element{mint_element(document), Kind::Arc, 300.0, 1, 200.0, Free{}}};
    document.tracks.push_back(branch);

    const Document before = document;
    const Solution first = solve(document);
    const Solution second = solve(document);

    CHECK(document.tracks.size() == before.tracks.size());
    CHECK(document.next_id == before.next_id);
    CHECK(first.tracks.size() == second.tracks.size());
    for (std::size_t i = 0; i < first.tracks.size(); ++i) {
        CHECK(near(first.tracks[i].length, second.tracks[i].length));
        CHECK(near(first.tracks[i].end.x, second.tracks[i].end.x));
        CHECK(near(first.tracks[i].end.y, second.tracks[i].end.y));
    }
}

/// Przejście rozjazdowe: a turnout standing against another one is placed by that
/// one's frog, not by a station somebody typed. Move the near one and the far one
/// goes with it, which is what makes the two of them one piece of trackwork.
void a_paired_turnout_follows_the_one_it_stands_against() {
    Document document;
    add_catalogue(document);
    const double spacing = 4.75;
    const TrackId first = add_straight_track(document, "tor 1", 0.0, 0.0, std::numbers::pi / 2.0, 1000.0);
    const TrackId second = add_straight_track(document, "tor 2", 0.0, spacing, std::numbers::pi / 2.0, 1000.0);

    // the near one opens toward the other track; the far one is trailing and stands
    // against it, so where it stands is not authored at all
    const TurnoutId near_id = add_turnout(document, first, 400.0, 1);
    TurnoutPlacement against;
    against.id = mint_turnout(document);
    against.type = "Rz 1:9 R190";
    against.on = second;
    against.station = 0.0;  // never used while it is paired
    against.hand = 1;       // a trailing turnout is laid reversed: the same side, the other hand
    against.facing = false;
    against.opposite = near_id;
    document.turnouts.push_back(against);

    Solution solution = solve(document);
    CHECK(solution.diagnostics.empty());
    const SolvedTurnout* laid_near = find_turnout(solution, near_id);
    const SolvedTurnout* laid_far = find_turnout(solution, against.id);
    CHECK(laid_near != nullptr && laid_near->valid);
    CHECK(laid_far != nullptr && laid_far->valid);
    if (laid_near == nullptr || laid_far == nullptr || !laid_far->valid) {
        return;
    }
    // the two frogs face each other across the międzytorze, with the wstawka between them:
    // 1,811 m of it left over after the two turnouts, which at 1:9 is 16,405 m of straight
    const double gap = std::hypot(laid_far->frog.x - laid_near->frog.x,
                                  laid_far->frog.y - laid_near->frog.y);
    CHECK(near(gap, 16.405, 0.01));
    const double station_before = laid_far->station;
    CHECK(station_before > 0.0);

    // slide the near one a hundred metres along its track
    find_turnout(document, near_id)->station = 500.0;
    solution = solve(document);
    CHECK(solution.diagnostics.empty());
    laid_near = find_turnout(solution, near_id);
    laid_far = find_turnout(solution, against.id);
    CHECK(laid_far != nullptr && laid_far->valid);
    if (laid_far == nullptr || !laid_far->valid) {
        return;
    }
    CHECK(near(laid_far->station, station_before + 100.0, 1e-3));
    CHECK(near(std::hypot(laid_far->frog.x - laid_near->frog.x,
                          laid_far->frog.y - laid_near->frog.y),
               gap, 1e-6));

    // unpaired, it stands where it was authored to stand and stays there
    find_turnout(document, against.id)->opposite = TurnoutId::none;
    find_turnout(document, against.id)->station = 700.0;
    solution = solve(document);
    CHECK(near(find_turnout(solution, against.id)->station, 700.0));
}

/// Standing against a turnout that is not there is said, not guessed at.
void a_turnout_against_nothing_is_reported() {
    Document document;
    add_catalogue(document);
    const TrackId only = add_straight_track(document, "tor 1", 0.0, 0.0, std::numbers::pi / 2.0, 800.0);
    TurnoutPlacement placement;
    placement.id = mint_turnout(document);
    placement.type = "Rz 1:9 R190";
    placement.on = only;
    placement.station = 200.0;
    placement.hand = 1;
    placement.opposite = static_cast<TurnoutId>(9999);
    document.turnouts.push_back(placement);

    const Solution solution = solve(document);
    CHECK(has(solution, Code::UnknownReference));
    CHECK(solution.turnouts.size() == 1);
    CHECK(!solution.turnouts.front().valid);
}

}  // namespace

int main() {
    RUN(a_branch_follows_its_frog);
    RUN(a_turnout_on_a_branch_of_a_turnout);
    RUN(a_cycle_is_reported);
    RUN(a_missing_type_is_reported);
    RUN(a_paired_turnout_follows_the_one_it_stands_against);
    RUN(a_turnout_against_nothing_is_reported);
    RUN(solving_leaves_the_document_alone);
    return REPORT();
}
