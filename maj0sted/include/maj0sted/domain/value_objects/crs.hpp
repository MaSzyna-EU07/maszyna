#pragma once

#include <compare>

namespace maj0sted::domain {

/// Value Object: a Coordinate Reference System, identified by its EPSG code.
///
/// The default for a map project is EPSG:2180 (PUWG 1992 / Poland CS92), a
/// Cartesian, metre-based projected system.
class Crs {
public:
    // EPSG code of the default map-project CRS.
    static constexpr int kDefaultEpsg = 2180;

    explicit constexpr Crs(int epsg) noexcept : epsg_{epsg} {}

    /// The default CRS: EPSG:2180.
    static constexpr Crs default_crs() noexcept { return Crs{kDefaultEpsg}; }

    [[nodiscard]] constexpr int epsg() const noexcept { return epsg_; }

    friend constexpr auto operator<=>(const Crs&, const Crs&) noexcept = default;

private:
    int epsg_;
};

}  // namespace maj0sted::domain
