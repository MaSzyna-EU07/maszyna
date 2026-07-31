#pragma once

#include <cmath>
#include <compare>
#include <numbers>
#include <stdexcept>

#include "maj0sted/domain/value_objects/cartesian_position.hpp"

namespace maj0sted::domain {

/// Value Object: a horizontal azimuth — the direction of travel in the plan,
/// measured clockwise from north (the +y / northing axis) and normalised to
/// [0, 2*pi) radians.
class Azimuth {
public:
    static Azimuth from_radians(double radians) noexcept {
        return Azimuth{normalise(radians)};
    }

    static Azimuth from_degrees(double degrees) noexcept {
        return from_radians(degrees * std::numbers::pi / 180.0);
    }

    /// Azimuth of the direction from @p from to @p to.
    /// @throws std::invalid_argument if the two points coincide.
    static Azimuth of_direction(CartesianPosition from, CartesianPosition to) {
        const double d_east = to.x() - from.x();
        const double d_north = to.y() - from.y();
        if (d_east == 0.0 && d_north == 0.0) {
            throw std::invalid_argument{"Azimuth is undefined for coincident points"};
        }
        return from_radians(std::atan2(d_east, d_north));
    }

    [[nodiscard]] double radians() const noexcept { return radians_; }
    [[nodiscard]] double degrees() const noexcept {
        return radians_ * 180.0 / std::numbers::pi;
    }

    /// Smallest absolute angular difference to @p other, in [0, pi].
    [[nodiscard]] double angular_distance(Azimuth other) const noexcept {
        double difference = std::fabs(radians_ - other.radians_);
        if (difference > std::numbers::pi) {
            difference = 2.0 * std::numbers::pi - difference;
        }
        return difference;
    }

    friend auto operator<=>(const Azimuth&, const Azimuth&) noexcept = default;

private:
    explicit Azimuth(double radians) noexcept : radians_{radians} {}

    static double normalise(double radians) noexcept {
        const double two_pi = 2.0 * std::numbers::pi;
        double result = std::fmod(radians, two_pi);
        if (result < 0.0) {
            result += two_pi;
        }
        return result;
    }

    double radians_;
};

}  // namespace maj0sted::domain
