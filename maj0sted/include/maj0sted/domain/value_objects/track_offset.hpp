#pragma once

#include <stdexcept>

#include "maj0sted/domain/value_objects/length.hpp"

namespace maj0sted::domain {

/// Which side a parallel track runs on, relative to the origin straight's
/// direction of travel: left ("po lewej") or right ("po prawej").
enum class Side { Left, Right };

/// Value Object: the sideways offset of a parallel track from its origin — a
/// strictly positive perpendicular distance plus the side it lies on.
class TrackOffset {
public:
    TrackOffset(Length distance, Side side) : distance_{distance}, side_{side} {
        if (!(distance.metres() > 0.0)) {
            throw std::invalid_argument{"Track offset distance must be positive"};
        }
    }

    [[nodiscard]] Length distance() const noexcept { return distance_; }
    [[nodiscard]] Side side() const noexcept { return side_; }

private:
    Length distance_;
    Side side_;
};

}  // namespace maj0sted::domain
