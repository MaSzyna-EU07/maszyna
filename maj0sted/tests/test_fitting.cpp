#include <cmath>
#include <numbers>
#include <variant>

#include "check.hpp"
#include "maj0sted/maj0sted.hpp"

using namespace maj0sted::domain;

namespace {

// A 90-degree left corner: entry heads east into (100,0), exit leaves north.
Niweleta make_corner() {
    Niweleta n{NiweletaId{1}, "Corner"};
    n.add_plan_element(Straight{StraightId{1}, CartesianPosition{0.0, 0.0},
                                CartesianPosition{100.0, 0.0}});
    n.add_plan_element(Straight{StraightId{2}, CartesianPosition{100.0, 0.0},
                                CartesianPosition{100.0, 100.0}},
                       JointContinuity::AzimuthBreak);
    return n;
}

void fit_plain_arc() {
    const Niweleta corner = make_corner();
    const FitParameters params{.entry = StraightId{1},
                               .exit = StraightId{2},
                               .radius = Radius::from_metres(50.0),
                               .transition_length = std::nullopt};
    const FitResult fit = FittingService::fit_between_straights(corner, params);

    CHECK(fit.direction == TurnDirection::Left);

    // Tangent length = R*tan(45) = 50 -> tangent points at (50,0) and (100,50).
    CHECK(std::abs(fit.tangent_in.x() - 50.0) < 1e-6);
    CHECK(std::abs(fit.tangent_in.y() - 0.0) < 1e-6);
    CHECK(std::abs(fit.tangent_out.x() - 100.0) < 1e-6);
    CHECK(std::abs(fit.tangent_out.y() - 50.0) < 1e-6);

    // The trimmed straights keep their ids and outer endpoints.
    CHECK(fit.entry.id() == StraightId{1});
    CHECK(std::abs(fit.entry.end().x() - 50.0) < 1e-6);
    CHECK(fit.exit.id() == StraightId{2});
    CHECK(std::abs(fit.exit.start().y() - 50.0) < 1e-6);
    CHECK(std::abs(fit.exit.end().y() - 100.0) < 1e-6);

    // A single circular arc, length R*delta = 50*pi/2.
    CHECK(fit.curve.size() == 1);
    const auto* arc = std::get_if<CircularArc>(&fit.curve.front());
    CHECK(arc != nullptr);
    CHECK(arc->direction() == TurnDirection::Left);
    CHECK(arc->length().has_value());
    CHECK(std::abs(arc->length().value().metres() - 50.0 * std::numbers::pi / 2.0) < 1e-6);
}

void fit_with_symmetric_transitions() {
    const Niweleta corner = make_corner();
    const FitParameters params{.entry = StraightId{1},
                               .exit = StraightId{2},
                               .radius = Radius::from_metres(50.0),
                               .transition_length = Length::from_metres(10.0)};
    const FitResult fit = FittingService::fit_between_straights(corner, params);

    // klotoida -> łuk -> klotoida
    CHECK(fit.curve.size() == 3);
    CHECK(std::holds_alternative<TransitionCurve>(fit.curve[0]));
    CHECK(std::holds_alternative<CircularArc>(fit.curve[1]));
    CHECK(std::holds_alternative<TransitionCurve>(fit.curve[2]));

    // Entry transition runs from an infinite radius (straight) into R.
    const auto& entry_kp = std::get<TransitionCurve>(fit.curve[0]);
    CHECK(!entry_kp.start_radius().has_value());
    CHECK(entry_kp.end_radius().has_value());
    CHECK(entry_kp.length().metres() == 10.0);

    // Arc central angle = delta - 2*tau, tau = Lk/(2R) = 0.1 -> arc len = 50*(pi/2 - 0.2).
    const auto& arc = std::get<CircularArc>(fit.curve[1]);
    const double expected = 50.0 * (std::numbers::pi / 2.0 - 0.2);
    CHECK(arc.length().has_value());
    CHECK(std::abs(arc.length().value().metres() - expected) < 1e-6);

    // Entry tangent point from the exact clothoid layout (not the L^2/24R
    // approximation, which was ~1.8 mm off and caused a visible lateral gap).
    CHECK(std::abs(fit.tangent_in.x() - 44.9185491305) < 1e-6);
}

void invalid_inputs_are_rejected() {
    const Niweleta corner = make_corner();
    const Radius r = Radius::from_metres(50.0);

    // Same straight.
    CHECK_THROWS(FittingService::fit_between_straights(
        corner, FitParameters{StraightId{1}, StraightId{1}, r, std::nullopt}));
    // Unknown id.
    CHECK_THROWS(FittingService::fit_between_straights(
        corner, FitParameters{StraightId{1}, StraightId{99}, r, std::nullopt}));

    // Collinear straights -> nothing to fit.
    Niweleta line{NiweletaId{2}};
    line.add_plan_element(Straight{StraightId{1}, CartesianPosition{0.0, 0.0},
                                   CartesianPosition{50.0, 0.0}});
    line.add_plan_element(Straight{StraightId{2}, CartesianPosition{50.0, 0.0},
                                   CartesianPosition{100.0, 0.0}});
    CHECK_THROWS(FittingService::fit_between_straights(
        line, FitParameters{StraightId{1}, StraightId{2}, r, std::nullopt}));

    // Transitions too long for this radius/deflection.
    CHECK_THROWS(FittingService::fit_between_straights(
        corner, FitParameters{StraightId{1}, StraightId{2}, r,
                              Length::from_metres(200.0)}));
}

void fit_compound_curve_two_arcs() {
    const Niweleta corner = make_corner();  // 90-degree left corner, IP at (100,0)

    // First arc delta1 = 45 deg -> length = R1 * pi/4 = 60 * pi/4; last arc computed.
    const CompoundFitParameters params{
        .entry = StraightId{1},
        .exit = StraightId{2},
        .arcs = {CompoundArc{Radius::from_metres(60.0),
                             Length::from_metres(60.0 * std::numbers::pi / 4.0)},
                 CompoundArc{Radius::from_metres(40.0), std::nullopt}}};
    const FitResult fit = FittingService::fit_compound(corner, params);

    CHECK(fit.direction == TurnDirection::Left);
    CHECK(fit.curve.size() == 2);
    const auto& arc1 = std::get<CircularArc>(fit.curve[0]);
    const auto& arc2 = std::get<CircularArc>(fit.curve[1]);
    CHECK(arc1.radius().metres() == 60.0);
    CHECK(arc2.radius().metres() == 40.0);

    // Same hand-computed geometry as before (rigid-chain solver reproduces it).
    CHECK(std::abs(arc1.length().value().metres() - 60.0 * std::numbers::pi / 4.0) < 1e-6);
    CHECK(std::abs(arc2.length().value().metres() - 40.0 * std::numbers::pi / 4.0) < 1e-6);
    CHECK(std::abs(fit.tangent_in.x() - 45.85786) < 1e-4);
    CHECK(std::abs(fit.tangent_in.y() - 0.0) < 1e-4);
    CHECK(std::abs(fit.tangent_out.x() - 100.0) < 1e-4);
    CHECK(std::abs(fit.tangent_out.y() - 45.85786) < 1e-4);
}

void fit_compound_curve_three_arcs() {
    const Niweleta corner = make_corner();  // 90 deg; entry line y=0, exit line x=100

    // 30 + 30 + (remaining 30) degrees, radii 80 / 50 / 30.
    const CompoundFitParameters params{
        .entry = StraightId{1},
        .exit = StraightId{2},
        .arcs = {CompoundArc{Radius::from_metres(80.0),
                             Length::from_metres(80.0 * std::numbers::pi / 6.0)},
                 CompoundArc{Radius::from_metres(50.0),
                             Length::from_metres(50.0 * std::numbers::pi / 6.0)},
                 CompoundArc{Radius::from_metres(30.0), std::nullopt}}};
    const FitResult fit = FittingService::fit_compound(corner, params);

    CHECK(fit.curve.size() == 3);
    CHECK(std::get<CircularArc>(fit.curve[0]).radius().metres() == 80.0);
    CHECK(std::get<CircularArc>(fit.curve[1]).radius().metres() == 50.0);
    CHECK(std::get<CircularArc>(fit.curve[2]).radius().metres() == 30.0);

    // Last arc takes the remaining 30 deg -> length = 30 * pi/6.
    CHECK(std::abs(std::get<CircularArc>(fit.curve[2]).length().value().metres() -
                   30.0 * std::numbers::pi / 6.0) < 1e-6);

    // The chain closes onto both fixed lines: tangent_in on y=0, tangent_out on x=100.
    CHECK(std::abs(fit.tangent_in.y() - 0.0) < 1e-6);
    CHECK(std::abs(fit.tangent_out.x() - 100.0) < 1e-6);
    CHECK(fit.direction == TurnDirection::Left);
}

void fit_compound_with_transitions() {
    const Niweleta corner = make_corner();  // 90 deg; entry line y=0, exit line x=100

    const CompoundFitParameters params{
        .entry = StraightId{1},
        .exit = StraightId{2},
        .entry_transition = Length::from_metres(10.0),
        .exit_transition = Length::from_metres(10.0),
        .arcs = {CompoundArc{Radius::from_metres(60.0), Length::from_metres(20.0),
                             Length::from_metres(8.0)},  // arc(60), KP 60->40 of 8 m
                 CompoundArc{Radius::from_metres(40.0), std::nullopt, std::nullopt}}};
    const FitResult fit = FittingService::fit_compound(corner, params);

    // Sequence: KP -> arc(60) -> KP(60->40) -> arc(40) -> KP.
    CHECK(fit.curve.size() == 5);
    CHECK(std::holds_alternative<TransitionCurve>(fit.curve[0]));
    CHECK(std::holds_alternative<CircularArc>(fit.curve[1]));
    CHECK(std::holds_alternative<TransitionCurve>(fit.curve[2]));
    CHECK(std::holds_alternative<CircularArc>(fit.curve[3]));
    CHECK(std::holds_alternative<TransitionCurve>(fit.curve[4]));

    // Entry transition: straight (inf) -> R60.
    const auto& kp_in = std::get<TransitionCurve>(fit.curve[0]);
    CHECK(!kp_in.start_radius().has_value());
    CHECK(kp_in.end_radius().value().metres() == 60.0);
    CHECK(kp_in.length().metres() == 10.0);

    // Between transition: R60 -> R40.
    const auto& kp_mid = std::get<TransitionCurve>(fit.curve[2]);
    CHECK(kp_mid.start_radius().value().metres() == 60.0);
    CHECK(kp_mid.end_radius().value().metres() == 40.0);
    CHECK(kp_mid.length().metres() == 8.0);

    // Exit transition: R40 -> straight (inf).
    const auto& kp_out = std::get<TransitionCurve>(fit.curve[4]);
    CHECK(kp_out.start_radius().value().metres() == 40.0);
    CHECK(!kp_out.end_radius().has_value());

    // The chain still closes onto both fixed lines.
    CHECK(std::abs(fit.tangent_in.y() - 0.0) < 1e-6);
    CHECK(std::abs(fit.tangent_out.x() - 100.0) < 1e-6);
    CHECK(fit.direction == TurnDirection::Left);
}

void compound_invalid_inputs_are_rejected() {
    const Niweleta corner = make_corner();
    const auto arc = [](double r, double len) {
        return CompoundArc{Radius::from_metres(r), Length::from_metres(len)};
    };
    const auto last = [](double r) { return CompoundArc{Radius::from_metres(r)}; };

    // Equal neighbouring radii.
    CHECK_THROWS(FittingService::fit_compound(
        corner, CompoundFitParameters{.entry = StraightId{1},
                                      .exit = StraightId{2},
                                      .arcs = {arc(50.0, 10.0), last(50.0)}}));

    // Split lengths exceed the total deflection.
    CHECK_THROWS(FittingService::fit_compound(
        corner, CompoundFitParameters{.entry = StraightId{1},
                                      .exit = StraightId{2},
                                      .arcs = {arc(60.0, 1000.0), last(40.0)}}));

    // A non-last arc without a length.
    CHECK_THROWS(FittingService::fit_compound(
        corner, CompoundFitParameters{.entry = StraightId{1},
                                      .exit = StraightId{2},
                                      .arcs = {last(60.0), last(40.0)}}));
}

void fit_equal_radius_reverse_curve_between_parallel_straights() {
    const Straight entry{StraightId{1}, CartesianPosition{0.0, 0.0},
                         CartesianPosition{200.0, 0.0}};
    const Straight exit{StraightId{2}, CartesianPosition{0.0, 20.0},
                        CartesianPosition{200.0, 20.0}};

    const FitResult fit =
        FittingService::fit_reverse(entry, exit, Radius::from_metres(50.0));
    CHECK(fit.curve.size() == 2);
    const auto& first = std::get<CircularArc>(fit.curve[0]);
    const auto& second = std::get<CircularArc>(fit.curve[1]);
    CHECK(first.radius() == second.radius());
    CHECK(first.radius() == Radius::from_metres(50.0));
    CHECK(first.direction() != second.direction());
    CHECK(first.length().has_value());
    CHECK(second.length().has_value());
    CHECK(std::abs(first.length()->metres() - second.length()->metres()) < 1e-9);
    CHECK(std::abs(fit.tangent_in.y()) < 1e-9);
    CHECK(std::abs(fit.tangent_out.y() - 20.0) < 1e-6);
    CHECK(fit.tangent_in.x() > 0.0 && fit.tangent_in.x() <= 200.0);
    CHECK(fit.tangent_out.x() >= 0.0 && fit.tangent_out.x() < 200.0);
}

void reverse_curve_rejects_collinear_and_antiparallel_straights() {
    const Straight entry{StraightId{1}, CartesianPosition{0.0, 0.0},
                         CartesianPosition{100.0, 0.0}};
    const Straight collinear{StraightId{2}, CartesianPosition{0.0, 0.0},
                             CartesianPosition{100.0, 0.0}};
    const Straight antiparallel{StraightId{3}, CartesianPosition{100.0, 20.0},
                                CartesianPosition{0.0, 20.0}};
    const Radius radius = Radius::from_metres(30.0);

    CHECK_THROWS(FittingService::fit_reverse(entry, collinear, radius));
    CHECK_THROWS(FittingService::fit_reverse(entry, antiparallel, radius));
}

void fit_reverse_curve_between_nonparallel_straights() {
    const Straight entry{StraightId{1}, CartesianPosition{0.0, 0.0},
                         CartesianPosition{100.0, 0.0}};
    // This exit line is constructed around the end pose of a R=30 m chain:
    // left 60 degrees followed by right 30 degrees.
    const double heading = std::numbers::pi / 6.0;
    const double tangent_x = 86.9615242271;
    const double tangent_y = 25.9807621135;
    const double dx = std::cos(heading);
    const double dy = std::sin(heading);
    const Straight exit{
        StraightId{2},
        CartesianPosition{tangent_x - 30.0 * dx, tangent_y - 30.0 * dy},
        CartesianPosition{tangent_x + 70.0 * dx, tangent_y + 70.0 * dy}};

    const FitResult fit =
        FittingService::fit_reverse(entry, exit, Radius::from_metres(30.0));
    CHECK(fit.curve.size() == 2);
    const auto& first = std::get<CircularArc>(fit.curve[0]);
    const auto& second = std::get<CircularArc>(fit.curve[1]);
    CHECK(first.direction() != second.direction());
    CHECK(first.radius() == second.radius());
    CHECK(std::abs(fit.tangent_in.y()) < 1e-6);
    const double exit_cross =
        (fit.tangent_out.x() - exit.start().x()) * dy -
        (fit.tangent_out.y() - exit.start().y()) * dx;
    CHECK(std::abs(exit_cross) < 1e-5);
}

void apply_fit_replaces_straights_in_plan() {
    Niweleta corner = make_corner();  // straight(1) --break--> straight(2)
    const FitParameters params{.entry = StraightId{1},
                               .exit = StraightId{2},
                               .radius = Radius::from_metres(50.0),
                               .transition_length = std::nullopt};
    const FitResult fit = FittingService::fit_between_straights(corner, params);

    corner.apply_fit(fit);

    // Plan is now: straight(1) -> arc -> straight(2), tangent joints, no break.
    CHECK(corner.plan().size() == 3);
    CHECK(std::holds_alternative<Straight>(corner.plan().elements()[0]));
    CHECK(std::holds_alternative<CircularArc>(corner.plan().elements()[1]));
    CHECK(std::holds_alternative<Straight>(corner.plan().elements()[2]));
    CHECK(corner.plan().joint(0) == JointContinuity::Tangent);
    CHECK(corner.plan().joint(1) == JointContinuity::Tangent);

    // Straights keep their ids and are trimmed to the tangent points.
    const auto& first = std::get<Straight>(corner.plan().elements()[0]);
    const auto& last = std::get<Straight>(corner.plan().elements()[2]);
    CHECK(first.id() == StraightId{1});
    CHECK(last.id() == StraightId{2});
    CHECK(std::abs(first.end().x() - 50.0) < 1e-6);
    CHECK(std::abs(last.start().y() - 50.0) < 1e-6);
}

}  // namespace

int main() {
    RUN(fit_plain_arc);
    RUN(fit_with_symmetric_transitions);
    RUN(invalid_inputs_are_rejected);
    RUN(fit_compound_curve_two_arcs);
    RUN(fit_compound_curve_three_arcs);
    RUN(fit_compound_with_transitions);
    RUN(compound_invalid_inputs_are_rejected);
    RUN(fit_equal_radius_reverse_curve_between_parallel_straights);
    RUN(reverse_curve_rejects_collinear_and_antiparallel_straights);
    RUN(fit_reverse_curve_between_nonparallel_straights);
    RUN(apply_fit_replaces_straights_in_plan);
    return REPORT();
}
