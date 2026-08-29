#include "maj0sted/domain/geometry/turnout.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

namespace maj0sted::domain {

using geometry::layout_segment;
using geometry::Pose;
using geometry::XY;

double CrossingMark::angle() const noexcept {
    return denominator_ > 0.0 ? std::atan(1.0 / denominator_) : 0.0;
}

namespace {

// The deflection (radians) a segment of linearly varying curvature turns through.
double deflection(const TurnoutSegment& segment) {
    return 0.5 * (segment.k0 + segment.k1) * segment.length;
}

// Turns @p pose in place by @p angle (a heading break, no travel).
Pose turn(const Pose& pose, double angle) {
    if (angle == 0.0) return pose;
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return Pose{pose.x, pose.y, pose.hx * c - pose.hy * s, pose.hx * s + pose.hy * c};
}

// Advances @p pose over @p segment (without sampling points; that is the
// renderer's job): the heading break first, then the travel. Returns the end pose.
Pose advance(const TurnoutSegment& segment, const Pose& pose) {
    return layout_segment(segment.k0, segment.k1, segment.length,
                          turn(pose, segment.turn_in), nullptr);
}

// Where the through tangent (through @p a along @p a heading) meets the diverging
// tangent (through @p b along @p b heading). Writes the two tangent lengths; both
// stay zero when the tangents are parallel (a straight-through, angle-less turnout).
void tangent_lengths(const Pose& a, const Pose& b, double& front, double& back) {
    const double denominator = a.hx * b.hy - a.hy * b.hx;
    if (std::abs(denominator) < 1e-9) return;
    const double u = ((b.x - a.x) * b.hy - (b.y - a.y) * b.hx) / denominator;
    const double cx = a.x + a.hx * u;
    const double cy = a.y + a.hy * u;
    front = std::hypot(cx - a.x, cy - a.y);
    back = std::hypot(cx - b.x, cy - b.y);
}

// The heading of @p pose off the through track that runs through @p start, signed
// so that it grows as the path leaves it.
double heading_off(const Pose& start, const Pose& pose, double sign) {
    return sign * std::atan2(pose.hy * start.hx - pose.hx * start.hy,
                             pose.hx * start.hx + pose.hy * start.hy);
}

// The pose @p station metres along @p path. Stations are the same whichever frame
// the path is measured in, because bending moves curvature and never length.
Pose pose_at_station(const std::vector<TurnoutSegment>& path, const Pose& start,
                     double station) {
    Pose pose = start;
    double walked = 0.0;
    for (const auto& segment : path) {
        const Pose from = turn(pose, segment.turn_in);
        if (station <= walked + segment.length) {
            return layout_segment(segment.k0, segment.k1, station - walked, from, nullptr);
        }
        walked += segment.length;
        pose = layout_segment(segment.k0, segment.k1, segment.length, from, nullptr);
    }
    return pose;
}

// Bends the laid path onto a through track of curvature @p bend, the way a rozjazd
// łukowy is made: the triangle the turnout is dimensioned by is rotated about its
// centre, so `t` and the crossing angle stand and every arc takes the radius that
// follows from them.
//
// What is re-cut and what is not comes straight from Koc: the zwrotnica and the
// krzyżownica keep their dimensions and are only bent, so the blade, the odcinek
// przediglicowy and the frog rail keep their lengths to the millimetre. The szyny
// łączące — the plain curve between them — take up the whole difference, which is
// small: the arc between two tangent points barely changes length when it is bent.
//
// False when there is no single arc radius to rotate about, nothing flexible to
// take up the change, or the bend is sharp enough to eat the connecting rails whole.
bool bend_path(std::vector<TurnoutSegment>& path, double alpha, double sign, double bend,
               double& tangent_out, double& curvature_out, double& beta_out) {
    double radius = 0.0;
    double cut = 0.0;       // what bending does not re-cut: switch and frog
    double flexible = 0.0;  // the connecting rails, which it does
    for (const auto& segment : path) {
        if (segment.k0 == 0.0 && segment.k1 == 0.0) continue;
        if (segment.k0 != segment.k1) return false;  // a clothoid has no one radius
        const double own = std::abs(1.0 / segment.k0);
        if (radius != 0.0 && std::abs(own - radius) > 1e-6 * radius) return false;
        radius = own;
        (segment.part == TurnoutPart::Curve ? flexible : cut) += segment.length;
    }
    if (radius <= 0.0 || flexible <= 0.0 || alpha <= 0.0) return false;

    const double tangent = radius * std::tan(0.5 * alpha);
    const double beta = 2.0 * std::atan(tangent * bend);
    const double turn_to = beta + sign * alpha;
    if (std::abs(turn_to) >= std::numbers::pi - 1e-9) return false;

    // t/tg(theta/2) is the radius of every leg of the triangle, the through track's
    // as much as the branch's. At theta = 0 the branch comes out straight, and its
    // two tangents are then all the length there is between PR and the crossing
    const double curvature =
        std::abs(turn_to) > 1e-12 ? std::tan(0.5 * turn_to) / tangent : 0.0;
    const double run = curvature != 0.0 ? turn_to / curvature : 2.0 * tangent;
    const double spare = run - cut;
    if (spare <= 0.0) return false;

    for (auto& segment : path) {
        if (segment.k0 == 0.0 && segment.k1 == 0.0) continue;
        segment.k0 = curvature;
        segment.k1 = curvature;
        if (segment.part == TurnoutPart::Curve) {
            segment.length *= spare / flexible;
        }
    }

    tangent_out = tangent;
    curvature_out = curvature;
    beta_out = beta;
    return true;
}

// Where along the laid path the blade first stands @p target away from the stock
// rail — the dimension a catalogue drawing gives its blade in (the tip is where the
// rail is u thick, the planing ends where it is a railhead wide). The gap is not the
// offset of the axis: it opens on the blade's own curve, half a gauge wider than the
// axis it is offset from (Koc, 5.2), so it is carried segment by segment and the one
// that crosses it is solved in place. The station is on the axis. Returns false when
// the path never gets that far.
bool station_at_offset(const std::vector<TurnoutSegment>& path, const Pose& start,
                       double target, double sign, double& station_out, Pose& pose_out) {
    if (target <= 0.0) return false;

    Pose pose = start;
    double station = 0.0;
    double gap = 0.0;
    for (const auto& segment : path) {
        const Pose from = turn(pose, segment.turn_in);
        const Pose to = advance(segment, pose);
        const double heading = heading_off(start, from, sign);
        const double radius = segment.k0 != 0.0 ? std::abs(1.0 / segment.k0) : 0.0;
        const double opened =
            radius > 0.0
                ? (radius + kHalfGauge) *
                      (std::cos(heading) - std::cos(heading_off(start, to, sign)))
                : segment.length * std::sin(heading);

        if (gap <= target && gap + opened >= target) {
            const double run = std::clamp(
                blade_run_to_offset(radius, heading, target - gap), 0.0, segment.length);
            station_out = station + run;
            pose_out = layout_segment(segment.k0, segment.k1, run, from, nullptr);
            return true;
        }
        gap += opened;
        station += segment.length;
        pose = to;
    }
    return false;
}

// Fills the blade dimensions: where its tip stands, how much of the curve runs ahead
// of it as the odcinek przediglicowy, where its planed nose ends and where it reaches
// full railhead width.
// @p relative is the path with the through track's own curvature taken out of it:
// the blade opens against the opornica, so what dimensions it is how the two part
// company, not how either of them runs across the ground. For a rozjazd zwyczajny
// the two paths are the same thing.
void dimension_blade(const Turnout& turnout, const Pose& start,
                     const std::vector<TurnoutSegment>& relative, TurnoutGeometry& geometry,
                     double sign) {
    double station = 0.0;
    Pose pose = start;
    for (const auto& segment : geometry.path) {
        if (segment.part != TurnoutPart::Blade) {
            station += segment.length;
            pose = advance(segment, pose);
            continue;
        }
        geometry.has_blade = true;
        // the blade begins where its piece does; everything ahead of it is the lead
        geometry.blade_station = station;
        geometry.blade_tip = turn(pose, segment.turn_in);

        double at{0.0};
        Pose at_pose{};
        if (station_at_offset(relative, start, turnout.blade.tip_thickness, sign, at, at_pose)) {
            geometry.nose_end = at;
            geometry.blade_nose_end = pose_at_station(geometry.path, start, at);
            geometry.blade_angle = heading_off(start, at_pose, sign);
            geometry.blade_residual = station + turnout.blade.nose - at;
        }
        if (station_at_offset(relative, start, turnout.blade.railtop_width, sign, at, at_pose)) {
            geometry.planing_end = at;
            geometry.blade_planed = pose_at_station(geometry.path, start, at);
            geometry.planing_length = at - geometry.blade_station;
        }
        return;
    }
}

}  // namespace

double blade_run_to_offset(double radius, double angle, double offset) {
    if (offset <= 0.0) return 0.0;
    if (radius > 0.0) {
        // the blade is an arc: the gap opens as `R1(cos beta - cos(beta + theta))`
        // on the rail's own curve, and the axis spends `R theta` getting there
        const double rail = radius + kHalfGauge;
        const double target = std::cos(angle) - offset / rail;
        if (target < -1.0) return 0.0;  // the offset is more than the arc ever reaches
        return radius * (std::acos(std::clamp(target, -1.0, 1.0)) - angle);
    }
    // a straight blade: the offset grows with the sine of the break
    const double s = std::sin(angle);
    return s > 1e-12 ? offset / s : 0.0;
}

TurnoutGeometry lay_turnout(const Pose& start, const Turnout& turnout, double bend) {
    TurnoutGeometry geometry;

    if (turnout.path.empty()) return geometry;
    const double sign = turnout.side == DivergeSide::Left ? 1.0 : -1.0;

    for (const auto& piece : turnout.path) {
        if (piece.length <= 0.0) return geometry;
        if (piece.radius_start < 0.0 || piece.radius_end < 0.0) return geometry;
        if (piece.turn_in < 0.0) return geometry;
        geometry.path.push_back(TurnoutSegment{
            sign * piece.turn_in, piece.radius_start > 0.0 ? sign / piece.radius_start : 0.0,
            piece.radius_end > 0.0 ? sign / piece.radius_end : 0.0, piece.length, piece.part});
    }

    double tangent = 0.0;
    double beta = 0.0;
    if (bend != 0.0) {
        double curvature = 0.0;
        if (!bend_path(geometry.path, turnout.mark.angle(), sign, bend, tangent, curvature,
                       beta)) {
            return geometry;
        }
        geometry.bend = bend;
        geometry.bend_angle = beta;
        geometry.diverging_curvature = curvature;
    }

    // the path is laid exactly as authored: what it works out to is then checked
    // against the catalogue numbers rather than fitted to them
    Pose pose = start;
    double deflected = 0.0;
    for (const auto& segment : geometry.path) {
        deflected += segment.turn_in + deflection(segment);
        pose = advance(segment, pose);
        geometry.diverging_length += segment.length;
    }

    geometry.valid = true;
    geometry.frog = pose;
    tangent_lengths(start, pose, geometry.tangent_front, geometry.tangent_back);

    // the blade is dimensioned against the opornica, which bends with the turnout
    std::vector<TurnoutSegment> relative = geometry.path;
    for (auto& segment : relative) {
        segment.k0 -= bend;
        segment.k1 -= bend;
    }
    dimension_blade(turnout, start, relative, geometry, sign);

    // deflected is signed by the hand, and it is the through track the crossing
    // angle is measured from — which turns by beta when the turnout is bent
    geometry.angle_residual = sign * (deflected - beta) - turnout.mark.angle();

    // where the switch ends on the tor zasadniczy: the frog, projected back onto it.
    // this is the second point a drawing marks on the through track, and on a bent
    // turnout the projection runs along the radius rather than square to a straight
    if (bend == 0.0) {
        geometry.through_length =
            (pose.x - start.x) * start.hx + (pose.y - start.y) * start.hy;
        geometry.through_end = Pose{start.x + start.hx * geometry.through_length,
                                    start.y + start.hy * geometry.through_length, start.hx,
                                    start.hy};
    } else {
        const double centre_x = start.x - start.hy / bend;
        const double centre_y = start.y + start.hx / bend;
        const double from = std::atan2(start.y - centre_y, start.x - centre_x);
        const double to = std::atan2(pose.y - centre_y, pose.x - centre_x);
        double swept = to - from;
        while (swept > std::numbers::pi) swept -= 2.0 * std::numbers::pi;
        while (swept < -std::numbers::pi) swept += 2.0 * std::numbers::pi;
        geometry.through_length = swept / bend;
        geometry.through_end =
            layout_segment(bend, bend, geometry.through_length, start, nullptr);
    }

    // the catalogue length is a + b, the two tangent legs of the drawing
    if (turnout.length > 0.0) {
        geometry.length_residual =
            geometry.tangent_front + geometry.tangent_back - turnout.length;
    }
    return geometry;
}

}  // namespace maj0sted::domain
