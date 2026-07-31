#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <vector>

namespace maj0sted::domain::geometry {

struct XY {
    double x;
    double y;
};

/// Position plus unit heading, in (east=x, north=y).
struct Pose {
    double x;
    double y;
    double hx;
    double hy;
};

/// Lays out one segment of linearly varying signed curvature, starting at
/// @p start: a circular arc when @p k0 == @p k1, a clothoid otherwise, a
/// straight when both are zero. When @p out is non-null the sampled points
/// (including the start point) are appended to it. Returns the segment's end
/// pose.
///
/// Both the fitting solver and the renderer call this so their geometry is
/// bit-for-bit identical — the fitted tangent points then coincide exactly with
/// the drawn curve, with no lateral gap even for tight arcs.
inline Pose layout_segment(double k0, double k1, double length, Pose start,
                           std::vector<XY>* out) {
    const double left_x = -start.hy;
    const double left_y = start.hx;
    const auto emit = [&](double local_x, double local_y) {
        if (out != nullptr) {
            out->push_back({start.x + local_x * start.hx + local_y * left_x,
                            start.y + local_x * start.hy + local_y * left_y});
        }
    };

    emit(0.0, 0.0);

    double end_x = 0.0;
    double end_y = 0.0;
    if (length > 0.0) {
        const double turned = std::abs(0.5 * (k0 + k1) * length);
        const int steps = std::clamp(
            static_cast<int>(std::ceil(std::max(
                length / 0.5, turned * 180.0 / std::numbers::pi / 2.0))),
            8, 512);
        const double ds = length / steps;

        if (std::abs(k1 - k0) < 1e-12) {
            const double k = k0;
            if (std::abs(k) < 1e-12) {
                end_x = length;  // straight
                emit(end_x, 0.0);
            } else {
                for (int i = 1; i <= steps; ++i) {  // circular arc, exact
                    const double s = ds * i;
                    end_x = std::sin(k * s) / k;
                    end_y = (1.0 - std::cos(k * s)) / k;
                    emit(end_x, end_y);
                }
            }
        } else {
            const double slope = (k1 - k0) / (2.0 * length);  // clothoid
            for (int i = 1; i <= steps; ++i) {
                const double s_mid = ds * (i - 0.5);
                const double theta = k0 * s_mid + slope * s_mid * s_mid;
                end_x += std::cos(theta) * ds;
                end_y += std::sin(theta) * ds;
                emit(end_x, end_y);
            }
        }
    }

    const double dtheta = 0.5 * (k0 + k1) * length;
    const double cos_t = std::cos(dtheta);
    const double sin_t = std::sin(dtheta);
    return Pose{start.x + end_x * start.hx + end_y * left_x,
                start.y + end_x * start.hy + end_y * left_y,
                start.hx * cos_t - start.hy * sin_t,
                start.hx * sin_t + start.hy * cos_t};
}

}  // namespace maj0sted::domain::geometry
