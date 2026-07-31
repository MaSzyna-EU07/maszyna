#pragma once

#include <stdexcept>

#include "maj0sted/domain/value_objects/straight_id.hpp"
#include "maj0sted/domain/value_objects/track_offset.hpp"

namespace maj0sted::domain {

/// A reference-based relation between two straight segments, identified by their
/// StraightId. @c origin is the authoritative reference track; @c parallel runs
/// alongside it at @c offset (a perpendicular distance on a given side).
///
/// Invariants: both straights must be identified (non-null id), and a straight
/// cannot be parallel to itself.
class TrackParallelism {
public:
    TrackParallelism(StraightId origin, StraightId parallel, TrackOffset offset)
        : origin_{origin}, parallel_{parallel}, offset_{offset} {
        if (origin.is_null() || parallel.is_null()) {
            throw std::invalid_argument{"Parallelism requires identified straights"};
        }
        if (origin == parallel) {
            throw std::invalid_argument{"A straight cannot be parallel to itself"};
        }
    }

    [[nodiscard]] StraightId origin() const noexcept { return origin_; }
    [[nodiscard]] StraightId parallel() const noexcept { return parallel_; }
    [[nodiscard]] TrackOffset offset() const noexcept { return offset_; }

private:
    StraightId origin_;
    StraightId parallel_;
    TrackOffset offset_;
};

}  // namespace maj0sted::domain
