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
#include <initializer_list>
#include <vector>
import eu07.editor.plan_turnout_geometry;

// A turnout is laid from the list its catalogue entry gives, piece by piece, and
// nothing in it is fitted: whether the list keeps the promises of the crossing
// mark and the catalogue length is reported, not corrected. The blade is
// dimensioned on top of that, the way Koc's rys. 5.9 dimensions an iglica styczna
// do opornicy o ściętym ostrzu: the curve is theoretically tangent to the stock
// rail at PR, but the blade only starts at a, and its planed nose runs on to A
// where the rail is u thick.



using namespace editor::plan;
using editor::plan::geometry::Pose;

namespace {

constexpr double kAlfa19 = 0.11065722117389563;  // atan(1/9)
constexpr double kStart[]{0.0, 0.0, 1.0, 0.0};

// Rz 49E1-190-1:9 as the drawings have it: u and d from Koc's tablica 5.1, the
// rear tangent from rys. 5.13 (27138 = 10523 + 16615, krzyżownica prosta). The
// blade ends where the gap to the stock rail takes the przekładka in its heel.
constexpr double kRadius = 190.0;
constexpr double kTipThickness = 0.0049;  // u
constexpr double kNose = 0.1250;          // d
constexpr double kRailtop = 0.070;        // w
constexpr double kHeelOffset = 0.136;
constexpr double kBackTangent = 16.615;  // b

// a: the odcinek przediglicowy, derived from u and d rather than copied — the gap
// opens on the blade's own curve, and the station it opens at is on the axis
const double kLead = blade_run_to_offset(kRadius, 0.0, kTipThickness) - kNose;
const double kBlade = blade_run_to_offset(kRadius, 0.0, kHeelOffset) - kLead;

// The gap @p pose has opened between the blade and the stock rail: the axis offset
// plus what half a gauge of rail swings out of the way as the path turns off.
double gap_at(const Pose& start, const Pose& pose) {
    const double offset = (pose.x - start.x) * -start.hy + (pose.y - start.y) * start.hx;
    const double heading = std::atan2(pose.hy * start.hx - pose.hx * start.hy,
                                      pose.hx * start.hx + pose.hy * start.hy);
    return offset + kHalfGauge * (1.0 - std::cos(heading));
}
const double kFrontTangent = kRadius * std::tan(0.5 * kAlfa19);
const double kFrogRail = kBackTangent - kFrontTangent;

Turnout rz_1_9_r190() {
    Turnout turnout{CrossingMark{9.0},
                    DivergeSide::Left,
                    {},
                    kFrontTangent + kBackTangent,
                    Blade{kTipThickness, kNose, kRailtop}};
    turnout.path.push_back(TurnoutPiece{TurnoutPart::PreBlade, kLead, kRadius, kRadius, 0.0});
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Blade, kBlade, kRadius, kRadius, 0.0});
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Curve,
                                        kRadius * kAlfa19 - kLead - kBlade, kRadius,
                                        kRadius, 0.0});
    turnout.path.push_back(TurnoutPiece{TurnoutPart::FrogRail, kFrogRail, 0.0, 0.0, 0.0});
    return turnout;
}

TurnoutGeometry laid(const Turnout& turnout) {
    return lay_turnout(Pose{kStart[0], kStart[1], kStart[2], kStart[3]}, turnout);
}

// Walks the laid path and returns the pose at the end of segment @p index.
Pose pose_after(const TurnoutGeometry& geometry, std::size_t index) {
    Pose pose{0.0, 0.0, 1.0, 0.0};
    for (std::size_t i = 0; i <= index; ++i) {
        const auto& segment = geometry.path[i];
        if (segment.turn_in != 0.0) {
            const double c = std::cos(segment.turn_in);
            const double s = std::sin(segment.turn_in);
            pose = Pose{pose.x, pose.y, pose.hx * c - pose.hy * s,
                        pose.hx * s + pose.hy * c};
        }
        pose = editor::plan::geometry::layout_segment(segment.k0, segment.k1, segment.length, pose,
                                        nullptr);
    }
    return pose;
}

void a_tangential_blade_never_breaks_away() {
    const auto geometry = laid(rz_1_9_r190());
    CHECK(geometry.valid);
    for (const auto& segment : geometry.path) {
        CHECK(segment.turn_in == 0.0);
    }
    // the curve is there from PR: the odcinek przediglicowy is already on it, the
    // vehicle is simply still on the through track
    CHECK(std::abs(geometry.path[0].k0 - 1.0 / kRadius) < 1e-12);
    CHECK(std::abs(geometry.path[1].k0 - 1.0 / kRadius) < 1e-12);
    // the blade begins where its piece does, a from PR
    CHECK(geometry.has_blade);
    CHECK(std::abs(geometry.blade_station - kLead) < 1e-12);
    CHECK(std::abs(geometry.blade_tip.x - kRadius * std::sin(kLead / kRadius)) < 1e-9);
    CHECK(std::abs(geometry.blade_tip.y -
                   kRadius * (1.0 - std::cos(kLead / kRadius))) < 1e-9);
}

void a_planed_nose_reaches_its_thickness_where_the_table_says() {
    const auto geometry = laid(rz_1_9_r190());
    CHECK(geometry.valid);

    const Pose start{kStart[0], kStart[1], kStart[2], kStart[3]};
    // A: where the blade stands u away from the stock rail, and where the tangent
    // there is the kąt przylegania — a couple of dozen minutes
    CHECK(std::abs(gap_at(start, geometry.blade_nose_end) - kTipThickness) < 1e-9);
    CHECK(std::abs(geometry.blade_angle - geometry.nose_end / kRadius) < 1e-9);
    CHECK(geometry.blade_angle * 180.0 / std::numbers::pi > 0.40);
    CHECK(geometry.blade_angle * 180.0 / std::numbers::pi < 0.42);
    // a was cut so that A lands on a + d, so a type carrying all three agrees with
    // the curve it is laid on and the residual is nothing
    CHECK(std::abs(geometry.nose_end - (kLead + kNose)) < 1e-12);
    CHECK(std::abs(geometry.blade_residual) < 1e-12);

    // the planing runs from the ostrze to full railhead width
    CHECK(std::abs(geometry.planing_end - blade_run_to_offset(kRadius, 0.0, kRailtop)) <
          1e-12);
    CHECK(std::abs(gap_at(start, geometry.blade_planed) - kRailtop) < 1e-9);
    CHECK(std::abs(geometry.planing_length - (geometry.planing_end - kLead)) < 1e-12);
    CHECK(geometry.planing_end > geometry.nose_end);
}

void a_lead_derived_from_u_and_d_lands_on_the_table() {
    // Koc, tablica 5.1: a, d and u for the iglice styczne do opornicy o ściętym
    // ostrzu. They obey a + d = the run to a gap of u, which is what lets a preset
    // carry only u and d. The table dimensions it on the rail, as a drawing does, so
    // the axis station is scaled back up by R1/R; the numbers then agree to a hair at
    // 1200 m and to millimetres at 190
    struct Row {
        double radius, a, d, u, tolerance;
    };
    constexpr Row rows[]{
        {190.0, 1.2483, 0.1250, 0.0049, 7e-3},   {300.0, 1.5000, 0.1250, 0.0044, 2e-3},
        {500.0, 2.1650, 0.1250, 0.0053, 1.4e-2}, {760.0, 2.6330, 0.1250, 0.0050, 2e-4},
        {1200.0, 3.3347, 0.1304, 0.0050, 1e-4},
    };
    for (const auto& row : rows) {
        const double rail = row.radius + kHalfGauge;
        const double drawn =
            blade_run_to_offset(row.radius, 0.0, row.u) * rail / row.radius;
        CHECK(std::abs(drawn - (row.a + row.d)) < row.tolerance);
    }
}

void a_straight_blade_is_planed_by_its_break() {
    // iglica prosta sieczna: the same two dimensions, taken off the break angle
    const double beta = 0.02;
    Turnout turnout{CrossingMark{9.0}, DivergeSide::Left, {}, 0.0,
                    Blade{0.005, 0.0, 0.070}};
    turnout.path.push_back(TurnoutPiece{TurnoutPart::PreBlade, 3.5, 0.0, 0.0, 0.0});
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Blade, 6.0, 0.0, 0.0, beta});
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Curve, 190.0 * (kAlfa19 - beta),
                                        190.0, 190.0, 0.0});

    const auto geometry = laid(turnout);
    CHECK(geometry.valid);
    // the lead runs along the through track and the blade tip sits at its end
    CHECK(geometry.blade_station == 3.5);
    CHECK(std::abs(geometry.blade_tip.x - 3.5) < 1e-9);
    CHECK(geometry.blade_tip.y == 0.0);
    CHECK(std::abs(geometry.nose_end - (3.5 + 0.005 / std::sin(beta))) < 1e-12);
    CHECK(std::abs(geometry.planing_length - 0.070 / std::sin(beta)) < 1e-12);
    // Templot says the same length along the stock rail rather than along the blade
    CHECK(std::abs(geometry.planing_length * std::cos(beta) - 0.070 / std::tan(beta)) <
          1e-12);
    // the break itself is the kąt przylegania here: the blade is straight
    CHECK(std::abs(geometry.blade_angle - beta) < 1e-12);
}

void a_blade_without_a_cut_tip_is_dimensioned_at_its_own_start() {
    auto turnout = rz_1_9_r190();
    turnout.blade = Blade{0.0, 0.0, 0.0};
    const auto geometry = laid(turnout);
    CHECK(geometry.valid);
    CHECK(geometry.has_blade);
    CHECK(std::abs(geometry.blade_station - kLead) < 1e-12);
    CHECK(geometry.nose_end == 0.0);
    CHECK(geometry.planing_length == 0.0);
    CHECK(geometry.blade_residual == 0.0);
}

void a_catalogue_list_closes_on_its_own_numbers() {
    const auto geometry = laid(rz_1_9_r190());
    CHECK(geometry.valid);
    CHECK(std::abs(geometry.angle_residual) < 1e-9);
    CHECK(std::abs(std::atan2(geometry.frog.hy, geometry.frog.hx) - kAlfa19) < 1e-9);
    // the two tangent legs are the drawing's a and b, and L is their sum
    CHECK(std::abs(geometry.tangent_front - 10.523) < 1e-3);
    CHECK(std::abs(geometry.tangent_back - 16.615) < 1e-3);
    CHECK(std::abs(geometry.tangent_front + geometry.tangent_back - 27.138) < 1e-3);
    CHECK(std::abs(geometry.length_residual) < 1e-9);
}

void the_switch_ends_on_the_straight_too() {
    const auto geometry = laid(rz_1_9_r190());
    CHECK(geometry.valid);
    // koniec rozjazdu as the straight sees it: the frog projected back onto it, which
    // is shorter than a + b and is the second point marked on the tor zasadniczy
    CHECK(std::abs(geometry.through_length -
                   (kRadius * std::sin(kAlfa19) + kFrogRail * std::cos(kAlfa19))) < 1e-9);
    CHECK(geometry.through_length < geometry.tangent_front + geometry.tangent_back);
    CHECK(std::abs(geometry.through_end.x - geometry.through_length) < 1e-12);
    CHECK(geometry.through_end.y == 0.0);
    CHECK(geometry.through_end.hx == 1.0);
}

void a_list_that_misses_the_mark_is_still_laid_and_said_out_loud() {
    auto turnout = rz_1_9_r190();
    turnout.path[2].length -= 2.0;  // two metres of curve short of the crossing angle
    const auto geometry = laid(turnout);
    CHECK(geometry.valid);  // it lays: the numbers are the author's business
    CHECK(std::abs(geometry.angle_residual + 2.0 / kRadius) < 1e-9);
    CHECK(geometry.length_residual != 0.0);
}

void a_degenerate_piece_is_refused() {
    auto turnout = rz_1_9_r190();
    turnout.path[2].length = 0.0;
    CHECK(!laid(turnout).valid);

    turnout = rz_1_9_r190();
    turnout.path[1].radius_start = -190.0;
    CHECK(!laid(turnout).valid);

    turnout = rz_1_9_r190();
    turnout.path.clear();
    CHECK(!laid(turnout).valid);
}

void a_right_hand_turnout_is_the_mirror_of_a_left_one() {
    const auto left = laid(rz_1_9_r190());
    auto turnout = rz_1_9_r190();
    turnout.side = DivergeSide::Right;
    const auto geometry = laid(turnout);
    CHECK(geometry.valid);
    CHECK(std::abs(geometry.path[1].k0 + 1.0 / kRadius) < 1e-12);
    CHECK(std::abs(std::atan2(geometry.frog.hy, geometry.frog.hx) + kAlfa19) < 1e-9);
    CHECK(std::abs(geometry.through_length - left.through_length) < 1e-9);
    // the residual is not signed by the hand: a right turnout closes just as well
    CHECK(std::abs(geometry.angle_residual) < 1e-9);
    // and the blade is dimensioned the same, on the other side
    CHECK(std::abs(geometry.nose_end - left.nose_end) < 1e-12);
    CHECK(std::abs(geometry.blade_angle - left.blade_angle) < 1e-12);
    CHECK(std::abs(geometry.blade_nose_end.y + left.blade_nose_end.y) < 1e-12);
}

void a_basket_is_three_pieces_and_nothing_special() {
    // łuk koszowy: 190 m eased to 300 m by a clothoid, ending on the same angle
    const double first = 8.0;
    const double transition = 6.0;
    // deflection: first/190 + transition*(1/190 + 1/300)/2 + rest/300 == alfa
    const double turned = first / 190.0 + 0.5 * transition * (1.0 / 190.0 + 1.0 / 300.0);
    const double rest = (kAlfa19 - turned) * 300.0;

    Turnout turnout{CrossingMark{9.0}, DivergeSide::Left, {}, 0.0,
                    Blade{0.0049, 0.125, 0.070}};
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Blade, first, 190.0, 190.0, 0.0});
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Curve, transition, 190.0, 300.0, 0.0});
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Curve, rest, 300.0, 300.0, 0.0});

    const auto geometry = laid(turnout);
    CHECK(geometry.valid);
    CHECK(std::abs(geometry.angle_residual) < 1e-9);
    CHECK(std::abs(std::atan2(geometry.frog.hy, geometry.frog.hx) - kAlfa19) < 1e-9);
    const Pose end = pose_after(geometry, geometry.path.size() - 1);
    CHECK(std::abs(end.x - geometry.frog.x) < 1e-9);
    CHECK(std::abs(end.y - geometry.frog.y) < 1e-9);
    // a blade starting at PR has no lead ahead of it, so a is nothing and the
    // residual is what the type's own d asks for against the curve
    CHECK(geometry.blade_station == 0.0);
    CHECK(std::abs(geometry.blade_residual - (0.125 - geometry.nose_end)) < 1e-12);
}

TurnoutGeometry laid_bent(const Turnout& turnout, double bend) {
    return lay_turnout(Pose{kStart[0], kStart[1], kStart[2], kStart[3]}, turnout, bend);
}

// The bend that turns the tor zasadniczy by @p beta over the turnout. Every leg of
// the triangle has the same tangent t, so a radius is t/tg(half its own angle).
double bend_for(double beta) { return std::tan(0.5 * beta) / kFrontTangent; }

void a_bend_of_nothing_is_the_turnout_it_was() {
    const auto plain = laid(rz_1_9_r190());
    const auto bent = laid_bent(rz_1_9_r190(), 0.0);
    CHECK(bent.valid);
    CHECK(bent.bend == 0.0);
    CHECK(bent.diverging_curvature == 0.0);
    CHECK(bent.path.size() == plain.path.size());
    for (std::size_t i = 0; i < plain.path.size(); ++i) {
        CHECK(bent.path[i].k0 == plain.path[i].k0);
        CHECK(bent.path[i].length == plain.path[i].length);
    }
    CHECK(bent.through_length == plain.through_length);
    CHECK(bent.nose_end == plain.nose_end);
}

void a_symmetric_bend_of_the_1_9_r190_is_the_catalogue_rls() {
    // Koc, 5.3.6: Rłs 49E1-380,292-1:9 z krzyżownicą prostą powstaje przez wygięcie
    // Rz 49E1-190-1:9. Symmetric means the two tracks split the crossing angle, so
    // the tor zasadniczy turns half of it the other way and both radii come out the
    // same — and the catalogue's own 380,292 is what they come out at
    const auto geometry = laid_bent(rz_1_9_r190(), bend_for(-0.5 * kAlfa19));
    CHECK(geometry.valid);
    CHECK(geometry.bend < 0.0);
    CHECK(geometry.diverging_curvature > 0.0);
    CHECK(std::abs(std::abs(1.0 / geometry.bend) - 380.292) < 0.02);
    CHECK(std::abs(std::abs(1.0 / geometry.diverging_curvature) - 380.292) < 0.02);
    CHECK(std::abs(geometry.bend_angle + 0.5 * kAlfa19) < 1e-12);
}

void a_bend_lands_on_kocs_own_formulas() {
    // (5.7) jednostronny and (5.8) dwustronny. The second is written here with the
    // sign our curvature carries openly: Koc's R - R1 denominator is the same figure
    // read off a drawing that has already decided which way the branch goes
    const double t2 = kFrontTangent * kFrontTangent;
    for (const double r1 : {320.0, 600.0, 2000.0}) {
        const auto same = laid_bent(rz_1_9_r190(), 1.0 / r1);
        CHECK(same.valid);
        CHECK(std::abs(1.0 / same.diverging_curvature -
                       (kRadius * r1 - t2) / (kRadius + r1)) < 1e-9);
        // bending with the branch makes it sharper, which is why R190 may not be
        CHECK(1.0 / same.diverging_curvature < kRadius);

        const auto against = laid_bent(rz_1_9_r190(), -1.0 / r1);
        CHECK(against.valid);
        CHECK(std::abs(1.0 / against.diverging_curvature -
                       (kRadius * r1 + t2) / (r1 - kRadius)) < 1e-9);
        CHECK(1.0 / against.diverging_curvature > kRadius);
    }
}

void a_bend_that_straightens_the_branch_still_lays() {
    // wygięcie dwustronne aż tor zwrotny wychodzi prosty: the triangle is all
    // tangent and no arc, so the path from PR is 2t of straight and the frog rail
    const auto geometry = laid_bent(rz_1_9_r190(), bend_for(-kAlfa19));
    CHECK(geometry.valid);
    CHECK(geometry.diverging_curvature == 0.0);
    CHECK(std::abs(geometry.diverging_length -
                   (2.0 * kFrontTangent + kFrogRail)) < 1e-9);
    CHECK(std::abs(geometry.frog.hy) < 1e-12);  // it runs dead straight
}

void bending_leaves_the_triangle_alone() {
    // what łukowanie does not touch: the crossing angle, the tangent legs, and so
    // the catalogue length. that is the whole point of rotating the triangle
    for (const double beta : {-0.5 * kAlfa19, -0.02, 0.01, 0.03}) {
        const auto geometry = laid_bent(rz_1_9_r190(), bend_for(beta));
        CHECK(geometry.valid);
        CHECK(std::abs(geometry.angle_residual) < 1e-9);
        CHECK(std::abs(geometry.tangent_front - kFrontTangent) < 1e-9);
        CHECK(std::abs(geometry.tangent_back - kBackTangent) < 1e-9);
        CHECK(std::abs(geometry.length_residual) < 1e-9);
        CHECK(std::abs(geometry.bend_angle - beta) < 1e-12);
    }
}

void a_bent_switch_keeps_its_blade() {
    // Koc: zwrotnica i krzyżownica pozostają bez zmian co do wymiarów, należy je
    // tylko wygiąć. The blade opens against an opornica that curves with it, so what
    // dimensions it is the difference of the two — and a + d still lands on A, to
    // well under the millimetre the rail is cut to
    const auto geometry = laid_bent(rz_1_9_r190(), bend_for(-0.5 * kAlfa19));
    CHECK(geometry.valid);
    CHECK(geometry.has_blade);
    // the rails are the same rails: a and the blade keep their lengths exactly
    CHECK(geometry.blade_station == kLead);
    CHECK(std::abs(geometry.path[1].length - kBlade) < 1e-12);
    CHECK(std::abs(geometry.nose_end - (kLead + kNose)) < 1e-3);
    CHECK(std::abs(geometry.blade_residual) < 1e-3);
    CHECK(geometry.blade_angle > 0.0);
    // and the tip sits on the bent path: still off to the left, but half as far as
    // it was, because the tor zasadniczy now runs away under it
    const auto plain = laid(rz_1_9_r190());
    CHECK(geometry.blade_tip.y > 0.0);
    CHECK(geometry.blade_tip.y < plain.blade_tip.y);
}

void the_bent_switch_ends_on_its_own_arc() {
    const double bend = bend_for(-0.5 * kAlfa19);
    const auto geometry = laid_bent(rz_1_9_r190(), bend);
    CHECK(geometry.valid);
    // koniec rozjazdu is the frog brought back onto the tor zasadniczy along the
    // radius, so it lands on the arc rather than on the straight PR set out along
    const double radius = 1.0 / bend;
    const double centre_y = radius;
    CHECK(std::abs(std::hypot(geometry.through_end.x,
                              geometry.through_end.y - centre_y) - std::abs(radius)) < 1e-9);
    CHECK(geometry.through_length > 0.0);
    CHECK(geometry.through_length < geometry.tangent_front + geometry.tangent_back);
    CHECK(std::abs(geometry.through_end.y) > 1e-6);  // the through track has left y=0
}

void a_path_with_no_one_radius_will_not_bend() {
    // a łuk koszowy has no single triangle to rotate about, and a bend guessed at
    // one of its radii would be a drawing of nothing
    const double first = 8.0;
    const double transition = 6.0;
    const double turned = first / 190.0 + 0.5 * transition * (1.0 / 190.0 + 1.0 / 300.0);
    Turnout turnout{CrossingMark{9.0}, DivergeSide::Left, {}, 0.0, Blade{0.0049, 0.125, 0.070}};
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Blade, first, 190.0, 190.0, 0.0});
    turnout.path.push_back(TurnoutPiece{TurnoutPart::Curve, transition, 190.0, 300.0, 0.0});
    turnout.path.push_back(
        TurnoutPiece{TurnoutPart::Curve, (kAlfa19 - turned) * 300.0, 300.0, 300.0, 0.0});
    CHECK(laid(turnout).valid);
    CHECK(!laid_bent(turnout, 1.0 / 600.0).valid);
}

void the_tangent_lengths_meet_where_the_tracks_would() {
    const auto geometry = laid(rz_1_9_r190());
    CHECK(geometry.valid);
    // the through tangent and the frog tangent meet at one point: walking either
    // way from the two ends has to land on it
    const double cx = geometry.tangent_front;
    CHECK(std::abs(geometry.frog.x - geometry.frog.hx * geometry.tangent_back - cx) < 1e-9);
    CHECK(std::abs(geometry.frog.y - geometry.frog.hy * geometry.tangent_back) < 1e-9);
}

}  // namespace

int main() {
    RUN(a_tangential_blade_never_breaks_away);
    RUN(a_planed_nose_reaches_its_thickness_where_the_table_says);
    RUN(a_lead_derived_from_u_and_d_lands_on_the_table);
    RUN(a_straight_blade_is_planed_by_its_break);
    RUN(a_blade_without_a_cut_tip_is_dimensioned_at_its_own_start);
    RUN(a_catalogue_list_closes_on_its_own_numbers);
    RUN(the_switch_ends_on_the_straight_too);
    RUN(a_list_that_misses_the_mark_is_still_laid_and_said_out_loud);
    RUN(a_degenerate_piece_is_refused);
    RUN(a_right_hand_turnout_is_the_mirror_of_a_left_one);
    RUN(a_basket_is_three_pieces_and_nothing_special);
    RUN(the_tangent_lengths_meet_where_the_tracks_would);
    RUN(a_bend_of_nothing_is_the_turnout_it_was);
    RUN(a_symmetric_bend_of_the_1_9_r190_is_the_catalogue_rls);
    RUN(a_bend_lands_on_kocs_own_formulas);
    RUN(a_bend_that_straightens_the_branch_still_lays);
    RUN(bending_leaves_the_triangle_alone);
    RUN(a_bent_switch_keeps_its_blade);
    RUN(the_bent_switch_ends_on_its_own_arc);
    RUN(a_path_with_no_one_radius_will_not_bend);
    return REPORT();
}
