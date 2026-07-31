#pragma once

#include <compare>
#include <stdexcept>

namespace maj0sted::domain {

/// Value Object: a non-negative length, in metres (the unit of the default
/// EPSG:2180 CRS). Lengths add and compare by value.
class Length {
public:
    constexpr Length() noexcept = default;

    static constexpr Length from_metres(double metres) {
        if (metres < 0.0) {
            throw std::invalid_argument{"Length cannot be negative"};
        }
        return Length{metres};
    }

    static constexpr Length zero() noexcept { return Length{}; }

    [[nodiscard]] constexpr double metres() const noexcept { return metres_; }

    [[nodiscard]] constexpr Length operator+(Length other) const noexcept {
        return Length{metres_ + other.metres_};
    }

    friend constexpr auto operator<=>(const Length&, const Length&) noexcept = default;

private:
    explicit constexpr Length(double metres) noexcept : metres_{metres} {}

    double metres_{0.0};
};

}  // namespace maj0sted::domain
