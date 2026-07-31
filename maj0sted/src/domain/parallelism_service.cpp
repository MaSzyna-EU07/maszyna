#include "maj0sted/domain/parallelism/parallelism_service.hpp"

#include <cmath>
#include <numbers>

namespace maj0sted::domain {
namespace {

// Unit left-normal of a straight's direction of travel, in (east=x, north=y).
// Direction d = (sin az, cos az); rotating it +90 deg (counter-clockwise) gives
// the left normal (-cos az, sin az).
struct Vec {
    double east;
    double north;
};

Vec left_normal(Azimuth azimuth) noexcept {
    return Vec{-std::cos(azimuth.radians()), std::sin(azimuth.radians())};
}

}  // namespace

TrackParallelism ParallelismService::mark_parallel(const Straight& origin,
                                                   const Straight& parallel,
                                                   TrackOffset offset) {
    // The invariants (both identified, distinct) are enforced by TrackParallelism.
    return TrackParallelism{origin.id(), parallel.id(), offset};
}

bool ParallelismService::are_parallel(const Straight& a, const Straight& b,
                                      double tolerance_rad) {
    const double difference = a.azimuth().angular_distance(b.azimuth());  // [0, pi]
    return difference <= tolerance_rad ||
           std::abs(difference - std::numbers::pi) <= tolerance_rad;
}

Straight ParallelismService::derive_parallel_line(const Straight& origin,
                                                  TrackOffset offset) {
    const Vec normal = left_normal(origin.azimuth());
    const double sign = (offset.side() == Side::Left) ? 1.0 : -1.0;
    const double shift = offset.distance().metres() * sign;

    const CartesianPosition start{origin.start().x() + normal.east * shift,
                                  origin.start().y() + normal.north * shift};
    const CartesianPosition end{origin.end().x() + normal.east * shift,
                                origin.end().y() + normal.north * shift};
    return Straight{start, end};
}

double ParallelismService::perpendicular_distance(const Straight& origin,
                                                  const Straight& other) {
    const Vec normal = left_normal(origin.azimuth());
    const double d_east = other.start().x() - origin.start().x();
    const double d_north = other.start().y() - origin.start().y();
    return std::abs(d_east * normal.east + d_north * normal.north);
}

}  // namespace maj0sted::domain
