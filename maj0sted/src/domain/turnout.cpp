#include "maj0sted/domain/geometry/turnout.hpp"

#include <cmath>

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

// Builds the diverging curve as signed-curvature segments that together turn
// through exactly @p total_angle. Every part but the closing arc is fixed; the
// closing arc's length is chosen to spend whatever deflection is left. Returns
// false when a radius is non-positive or the fixed parts overshoot the angle.
bool build_curve(const DivergingCurve& curve, double total_angle, double sign,
                 std::vector<TurnoutSegment>& out) {
    if (curve.arcs.empty()) return false;
    const double first_radius = curve.arcs.front().radius;
    const double last_radius = curve.arcs.back().radius;
    if (first_radius <= 0.0 || last_radius <= 0.0) return false;

    double consumed = 0.0;  // deflection of everything before the closing arc

    if (curve.entry_transition > 0.0) {
        out.push_back({0.0, sign / first_radius, curve.entry_transition});
        consumed += std::abs(deflection(out.back()));
    }

    // every arc but the last contributes its fixed length, eased to the next
    for (std::size_t i = 0; i + 1 < curve.arcs.size(); ++i) {
        const double radius = curve.arcs[i].radius;
        if (radius <= 0.0) return false;
        out.push_back({sign / radius, sign / radius, curve.arcs[i].length});
        consumed += std::abs(deflection(out.back()));

        const double next_radius = curve.arcs[i + 1].radius;
        if (curve.arcs[i].transition_to_next > 0.0 && next_radius > 0.0) {
            out.push_back({sign / radius, sign / next_radius,
                           curve.arcs[i].transition_to_next});
            consumed += std::abs(deflection(out.back()));
        }
    }

    const double exit_deflection = 0.5 * curve.exit_transition / last_radius;
    const double closing = total_angle - consumed - exit_deflection;
    if (closing <= 0.0) return false;  // the fixed parts already reach the crossing angle

    out.push_back({sign / last_radius, sign / last_radius, closing * last_radius});
    if (curve.exit_transition > 0.0) {
        out.push_back({sign / last_radius, 0.0, curve.exit_transition});
    }
    return true;
}

// Advances @p pose along @p segment (without sampling points; that is the
// renderer's job) and returns the end pose.
Pose advance(const TurnoutSegment& segment, const Pose& pose) {
    return layout_segment(segment.k0, segment.k1, segment.length, pose, nullptr);
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

}  // namespace

TurnoutGeometry lay_turnout(const Pose& start, const Turnout& turnout) {
    TurnoutGeometry geometry;

    const double angle = turnout.mark.angle();
    if (angle <= 0.0) return geometry;
    const double sign = turnout.side == DivergeSide::Left ? 1.0 : -1.0;

    if (!build_curve(turnout.curve, angle, sign, geometry.path)) {
        geometry.path.clear();
        return geometry;
    }

    // walk the curve to the frog direction (the crossing angle)
    Pose pose = start;
    for (const auto& segment : geometry.path) {
        pose = advance(segment, pose);
    }

    // a turnout always runs on past the curve as a straight frog rail. its length
    // takes the switch to its catalogue length, measured along the through track;
    // without a catalogue length a tangent-length rail keeps a frog rail present.
    const double through_projection =
        (pose.x - start.x) * start.hx + (pose.y - start.y) * start.hy;
    const double cos_angle = pose.hx * start.hx + pose.hy * start.hy;
    double rail = turnout.length > 0.0 && cos_angle > 1e-6
                      ? (turnout.length - through_projection) / cos_angle
                      : turnout.curve.arcs.back().radius * std::tan(angle * 0.5);
    if (rail > 0.0) {
        const TurnoutSegment frog_rail{0.0, 0.0, rail};
        geometry.path.push_back(frog_rail);
        pose = advance(frog_rail, pose);
    }

    geometry.valid = true;
    geometry.frog = pose;
    for (const auto& segment : geometry.path) geometry.diverging_length += segment.length;
    tangent_lengths(start, pose, geometry.tangent_front, geometry.tangent_back);
    return geometry;
}

}  // namespace maj0sted::domain
