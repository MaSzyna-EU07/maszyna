#include <cmath>

#include "check.hpp"
#include "maj0sted/maj0sted.hpp"

using namespace maj0sted::domain;

namespace {

void mark_parallel_references_two_straights() {
    // Two identified straights (typically from different niwelety).
    const Straight origin{StraightId{10}, CartesianPosition{0.0, 0.0},
                          CartesianPosition{100.0, 0.0}};
    const Straight parallel{StraightId{20}, CartesianPosition{0.0, 5.0},
                            CartesianPosition{100.0, 5.0}};

    const TrackOffset offset{Length::from_metres(5.0), Side::Left};
    const auto relation = ParallelismService::mark_parallel(origin, parallel, offset);

    CHECK(relation.origin() == StraightId{10});
    CHECK(relation.parallel() == StraightId{20});
    CHECK(relation.offset().distance().metres() == 5.0);
    CHECK(relation.offset().side() == Side::Left);
}

void marking_requires_identified_distinct_straights() {
    const Straight anonymous{CartesianPosition{0.0, 0.0}, CartesianPosition{100.0, 0.0}};
    const Straight a{StraightId{1}, CartesianPosition{0.0, 0.0},
                     CartesianPosition{100.0, 0.0}};
    const Straight b{StraightId{2}, CartesianPosition{0.0, 5.0},
                     CartesianPosition{100.0, 5.0}};
    const TrackOffset offset{Length::from_metres(5.0), Side::Left};

    // A straight without an id cannot be referenced.
    CHECK_THROWS(ParallelismService::mark_parallel(anonymous, b, offset));
    CHECK_THROWS(ParallelismService::mark_parallel(a, anonymous, offset));

    // A straight cannot be parallel to itself (same id).
    const Straight a_again{StraightId{1}, CartesianPosition{0.0, 10.0},
                           CartesianPosition{100.0, 10.0}};
    CHECK_THROWS(ParallelismService::mark_parallel(a, a_again, offset));
}

void offset_must_be_positive() {
    CHECK_THROWS(TrackOffset(Length::from_metres(0.0), Side::Left));
}

void parallelism_geometry() {
    const Straight origin{CartesianPosition{0.0, 0.0}, CartesianPosition{100.0, 0.0}};
    const Straight same_direction{CartesianPosition{0.0, 5.0},
                                  CartesianPosition{100.0, 5.0}};
    const Straight opposite{CartesianPosition{100.0, 5.0}, CartesianPosition{0.0, 5.0}};
    const Straight perpendicular{CartesianPosition{0.0, 0.0},
                                 CartesianPosition{0.0, 50.0}};

    CHECK(ParallelismService::are_parallel(origin, same_direction));
    CHECK(ParallelismService::are_parallel(origin, opposite));  // anti-parallel lines
    CHECK(!ParallelismService::are_parallel(origin, perpendicular));

    // Derive a line 5 m to the LEFT of an east-going origin -> to the north (y = +5).
    const Straight left = ParallelismService::derive_parallel_line(
        origin, TrackOffset{Length::from_metres(5.0), Side::Left});
    CHECK(std::abs(left.start().x() - 0.0) < 1e-9);
    CHECK(std::abs(left.start().y() - 5.0) < 1e-9);
    CHECK(std::abs(left.end().y() - 5.0) < 1e-9);
    CHECK(ParallelismService::are_parallel(origin, left));

    // To the RIGHT -> to the south (y = -5).
    const Straight right = ParallelismService::derive_parallel_line(
        origin, TrackOffset{Length::from_metres(5.0), Side::Right});
    CHECK(std::abs(right.start().y() + 5.0) < 1e-9);

    CHECK(std::abs(ParallelismService::perpendicular_distance(origin, same_direction) -
                   5.0) < 1e-9);
}

}  // namespace

int main() {
    RUN(mark_parallel_references_two_straights);
    RUN(marking_requires_identified_distinct_straights);
    RUN(offset_must_be_positive);
    RUN(parallelism_geometry);
    return REPORT();
}
