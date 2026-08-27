#include <cmath>
#include <numbers>

#include "check.hpp"
#include "maj0sted/domain/geometry/turnout.hpp"

using namespace maj0sted::domain;
using maj0sted::domain::geometry::Pose;

namespace {

Turnout rz_1_9_r190() {
    Turnout turnout{CrossingMark{9.0}, DivergeSide::Left, DivergingCurve{}, 27.138};
    turnout.curve.arcs.push_back(TurnoutArc{190.0, 0.0, 0.0});
    return turnout;
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
        pose = geometry::layout_segment(segment.k0, segment.k1, segment.length, pose,
                                        nullptr);
    }
    return pose;
}

void pre_blade_runs_along_the_through_track() {
    auto turnout = rz_1_9_r190();
    turnout.pre_blade = 3.5;
    turnout.blade_angle = 0.02;
    turnout.blade_length = 6.0;

    const auto geometry = lay_turnout(Pose{0.0, 0.0, 1.0, 0.0}, turnout);
    CHECK(geometry.valid);

    // the lead is straight, on the through track, and the blade tip sits at its end
    const Pose tip = pose_after(geometry, 0);
    CHECK(std::abs(tip.x - 3.5) < 1e-9);
    CHECK(std::abs(tip.y) < 1e-12);
    CHECK(std::abs(tip.hy) < 1e-12);

    // the blade breaks away by beta exactly, and stays straight over its length
    const Pose heel = pose_after(geometry, 1);
    CHECK(std::abs(std::atan2(heel.hy, heel.hx) - 0.02) < 1e-9);
    CHECK(std::abs(std::hypot(heel.x - tip.x, heel.y - tip.y) - 6.0) < 1e-9);
}

void the_frog_still_lands_on_the_crossing_angle() {
    const double alfa = std::atan(1.0 / 9.0);

    auto tangential = rz_1_9_r190();
    auto bladed = rz_1_9_r190();
    bladed.pre_blade = 3.5;
    bladed.blade_angle = 0.02;
    bladed.blade_length = 6.0;

    for (const auto& turnout : {tangential, bladed}) {
        const auto geometry = lay_turnout(Pose{0.0, 0.0, 1.0, 0.0}, turnout);
        CHECK(geometry.valid);
        // the curve spends only alfa - beta, so the frog heading is alfa either way
        CHECK(std::abs(std::atan2(geometry.frog.hy, geometry.frog.hx) - alfa) < 1e-9);
        // and the catalogue length is still measured along the through track
        CHECK(std::abs(geometry.frog.x - 27.138) < 1e-6);
    }
}

void a_blade_reaching_the_crossing_angle_is_rejected() {
    auto turnout = rz_1_9_r190();
    turnout.blade_angle = std::atan(1.0 / 9.0);  // nothing left for the curve to turn
    CHECK(!lay_turnout(Pose{0.0, 0.0, 1.0, 0.0}, turnout).valid);

    turnout.blade_angle = -0.01;
    CHECK(!lay_turnout(Pose{0.0, 0.0, 1.0, 0.0}, turnout).valid);
}

void a_bladeless_curve_still_starts_tangent() {
    const auto geometry = lay_turnout(Pose{0.0, 0.0, 1.0, 0.0}, rz_1_9_r190());
    CHECK(geometry.valid);
    for (const auto& segment : geometry.path) CHECK(segment.turn_in == 0.0);
}

// With no blade length the break still stands: the curve simply starts at the tip.
void a_zero_length_blade_keeps_its_break() {
    auto turnout = rz_1_9_r190();
    turnout.pre_blade = 3.5;
    turnout.blade_angle = 0.02;

    const auto geometry = lay_turnout(Pose{0.0, 0.0, 1.0, 0.0}, turnout);
    CHECK(geometry.valid);
    CHECK(geometry.path.size() >= 2);
    CHECK(std::abs(geometry.path[1].turn_in - 0.02) < 1e-12);
    CHECK(std::abs(std::atan2(geometry.frog.hy, geometry.frog.hx) -
                   std::atan(1.0 / 9.0)) < 1e-9);
}

// A right-hand turnout is the mirror image: the blade breaks the other way.
void a_right_hand_blade_breaks_the_other_way() {
    auto turnout = rz_1_9_r190();
    turnout.side = DivergeSide::Right;
    turnout.pre_blade = 3.5;
    turnout.blade_angle = 0.02;
    turnout.blade_length = 6.0;

    const auto geometry = lay_turnout(Pose{0.0, 0.0, 1.0, 0.0}, turnout);
    CHECK(geometry.valid);
    CHECK(std::abs(std::atan2(pose_after(geometry, 1).hy, pose_after(geometry, 1).hx) +
                   0.02) < 1e-9);
    CHECK(std::abs(std::atan2(geometry.frog.hy, geometry.frog.hx) +
                   std::atan(1.0 / 9.0)) < 1e-9);
}

}  // namespace

int main() {
    RUN(pre_blade_runs_along_the_through_track);
    RUN(the_frog_still_lands_on_the_crossing_angle);
    RUN(a_blade_reaching_the_crossing_angle_is_rejected);
    RUN(a_bladeless_curve_still_starts_tangent);
    RUN(a_zero_length_blade_keeps_its_break);
    RUN(a_right_hand_blade_breaks_the_other_way);
    return REPORT();
}
