#include <cmath>

#include "check.hpp"
#include "maj0sted/maj0sted.hpp"

using namespace maj0sted::domain;

namespace {

void straight_computes_azimuth_and_length() {
    // Going east (+x): azimuth = 90 deg (clockwise from north).
    const Straight east{CartesianPosition{0.0, 0.0}, CartesianPosition{100.0, 0.0}};
    CHECK(std::abs(east.length().metres() - 100.0) < 1e-9);
    CHECK(std::abs(east.azimuth().degrees() - 90.0) < 1e-9);

    // Going north (+y): azimuth = 0 deg.
    const Straight north{CartesianPosition{0.0, 0.0}, CartesianPosition{0.0, 50.0}};
    CHECK(std::abs(north.azimuth().degrees() - 0.0) < 1e-9);
    CHECK(std::abs(north.length().metres() - 50.0) < 1e-9);

    // A zero-length straight has no azimuth and is rejected.
    CHECK_THROWS(Straight(CartesianPosition{1.0, 1.0}, CartesianPosition{1.0, 1.0}));
}

void arc_carries_radius_direction_and_optional_length() {
    const CircularArc without_length{Radius::from_metres(300.0), TurnDirection::Left};
    CHECK(!without_length.length().has_value());

    const CircularArc with_length{Radius::from_metres(300.0), TurnDirection::Left,
                                  Length::from_metres(45.0)};
    CHECK(with_length.length().has_value());
    CHECK(with_length.length().value().metres() == 45.0);
}

void adjacent_same_arcs_are_rejected() {
    HorizontalAlignment same;
    same.append(CircularArc{Radius::from_metres(300.0), TurnDirection::Left});
    // same radius + same direction -> forbidden
    CHECK_THROWS(
        same.append(CircularArc{Radius::from_metres(300.0), TurnDirection::Left}));

    HorizontalAlignment opposite;
    opposite.append(CircularArc{Radius::from_metres(300.0), TurnDirection::Left});
    // same radius, different direction -> allowed (S-curve)
    opposite.append(CircularArc{Radius::from_metres(300.0), TurnDirection::Right});
    CHECK(opposite.size() == 2);

    HorizontalAlignment compound;
    compound.append(CircularArc{Radius::from_metres(300.0), TurnDirection::Left});
    // same direction, different radius -> allowed (compound curve)
    compound.append(CircularArc{Radius::from_metres(500.0), TurnDirection::Left});
    CHECK(compound.size() == 2);
}

void straight_to_straight_tangent_requires_equal_azimuth() {
    // Collinear straights, same azimuth, touching -> tangent OK.
    HorizontalAlignment collinear;
    collinear.append(
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    collinear.append(
        Straight{CartesianPosition{50.0, 0.0}, CartesianPosition{100.0, 0.0}});
    CHECK(collinear.size() == 2);
    CHECK(collinear.joint(0) == JointContinuity::Tangent);

    // Touching but different azimuth with a Tangent joint -> must be a break.
    HorizontalAlignment unmarked;
    unmarked.append(
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    CHECK_THROWS(unmarked.append(
        Straight{CartesianPosition{50.0, 0.0}, CartesianPosition{50.0, 50.0}}));

    // Non-touching tangent straights -> invalid.
    HorizontalAlignment gap;
    gap.append(Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    CHECK_THROWS(gap.append(
        Straight{CartesianPosition{60.0, 0.0}, CartesianPosition{100.0, 0.0}}));
}

void azimuth_break_between_straights_must_be_marked() {
    // Corner marked as AzimuthBreak -> OK.
    HorizontalAlignment kink;
    kink.append(Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    kink.append(Straight{CartesianPosition{50.0, 0.0}, CartesianPosition{50.0, 50.0}},
                JointContinuity::AzimuthBreak);
    CHECK(kink.size() == 2);
    CHECK(kink.joint(0) == JointContinuity::AzimuthBreak);

    // AzimuthBreak with equal azimuth is pointless -> invalid.
    HorizontalAlignment straight_break;
    straight_break.append(
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    CHECK_THROWS(straight_break.append(
        Straight{CartesianPosition{50.0, 0.0}, CartesianPosition{100.0, 0.0}},
        JointContinuity::AzimuthBreak));

    // AzimuthBreak is only allowed between two straights.
    HorizontalAlignment arc_break;
    arc_break.append(
        Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    CHECK_THROWS(arc_break.append(
        CircularArc{Radius::from_metres(300.0), TurnDirection::Left},
        JointContinuity::AzimuthBreak));
}

void straight_curve_joints_are_accepted_pending_fit() {
    // prosta -> klotoida -> łuk -> klotoida -> prosta
    HorizontalAlignment plan;
    plan.append(Straight{CartesianPosition{0.0, 0.0}, CartesianPosition{50.0, 0.0}});
    plan.append(TransitionCurve{Length::from_metres(30.0), std::nullopt,
                                Radius::from_metres(300.0), TurnDirection::Left});
    plan.append(CircularArc{Radius::from_metres(300.0), TurnDirection::Left,
                            Length::from_metres(40.0)});
    plan.append(TransitionCurve{Length::from_metres(30.0),
                                Radius::from_metres(300.0), std::nullopt,
                                TurnDirection::Left});
    plan.append(Straight{CartesianPosition{200.0, 0.0}, CartesianPosition{260.0, 0.0}});
    CHECK(plan.size() == 5);
    // Known lengths: 50 + 30 + 40 + 30 + 60.
    CHECK(plan.total_length().metres() == 210.0);
}

}  // namespace

int main() {
    RUN(straight_computes_azimuth_and_length);
    RUN(arc_carries_radius_direction_and_optional_length);
    RUN(adjacent_same_arcs_are_rejected);
    RUN(straight_to_straight_tangent_requires_equal_azimuth);
    RUN(azimuth_break_between_straights_must_be_marked);
    RUN(straight_curve_joints_are_accepted_pending_fit);
    return REPORT();
}
