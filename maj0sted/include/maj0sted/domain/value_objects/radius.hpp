#pragma once

#include <compare>
#include <stdexcept>

namespace maj0sted::domain {

/// Value Object: a radius of curvature, in metres. Always strictly positive —
/// the turn direction (plan) or curve sense (profile) is carried separately by
/// the geometric element, so the radius itself is just a magnitude.
class Radius {
public:
    static constexpr Radius from_metres(double metres) {
        if (!(metres > 0.0)) {
            throw std::invalid_argument{"Radius must be positive"};
        }
        return Radius{metres};
    }

    [[nodiscard]] constexpr double metres() const noexcept { return metres_; }

    friend constexpr auto operator<=>(const Radius&, const Radius&) noexcept = default;

private:
    explicit constexpr Radius(double metres) noexcept : metres_{metres} {}

    double metres_;
};

}  // namespace maj0sted::domain
