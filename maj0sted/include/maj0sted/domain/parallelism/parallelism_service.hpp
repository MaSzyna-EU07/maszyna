#pragma once

#include "maj0sted/domain/geometry/plan_element.hpp"
#include "maj0sted/domain/parallelism/track_parallelism.hpp"
#include "maj0sted/domain/value_objects/track_offset.hpp"

namespace maj0sted::domain {

/// Domain Service: creates and reasons about parallelism between straight
/// segments.
class ParallelismService {
public:
    /// Marks @p origin as the origin of a parallel relation with @p parallel,
    /// separated by @p offset. Both straights must be identified.
    /// @throws std::invalid_argument if a straight has no id or the two are the
    ///         same straight.
    static TrackParallelism mark_parallel(const Straight& origin,
                                          const Straight& parallel,
                                          TrackOffset offset);

    /// Are two straights parallel as lines (same or opposite azimuth)?
    static bool are_parallel(const Straight& a, const Straight& b,
                             double tolerance_rad = 1e-9);

    /// The line the parallel straight lies on: @p origin shifted sideways by
    /// @p offset, perpendicular to its direction of travel. Same length and
    /// azimuth as the origin; the returned straight is anonymous.
    static Straight derive_parallel_line(const Straight& origin, TrackOffset offset);

    /// Perpendicular distance (>= 0) from @p other's start point to the
    /// (infinite) line carrying @p origin.
    static double perpendicular_distance(const Straight& origin, const Straight& other);
};

}  // namespace maj0sted::domain
