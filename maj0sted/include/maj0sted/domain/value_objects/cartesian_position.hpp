#pragma once

#include <compare>

namespace maj0sted::domain {

/// Value Object: a position in a Cartesian coordinate reference system.
///
/// Coordinates are expressed in the units of the project's CRS. For the default
/// EPSG:2180 these are metres, with @c x as easting and @c y as northing.
class CartesianPosition {
public:
    constexpr CartesianPosition() noexcept = default;
    constexpr CartesianPosition(double x, double y) noexcept : x_{x}, y_{y} {}

    [[nodiscard]] constexpr double x() const noexcept { return x_; }
    [[nodiscard]] constexpr double y() const noexcept { return y_; }

    friend constexpr bool operator==(const CartesianPosition&,
                                     const CartesianPosition&) noexcept = default;

private:
    double x_{0.0};
    double y_{0.0};
};

}  // namespace maj0sted::domain
