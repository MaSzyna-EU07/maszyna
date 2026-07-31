#pragma once

#include <vector>

#include "maj0sted/domain/niweleta.hpp"
#include "maj0sted/domain/value_objects/crs.hpp"

namespace maj0sted::domain {

/// The map project ("projekt mapy").
///
/// It defines the Cartesian coordinate reference system in which every position
/// in the project is expressed (by default EPSG:2180, metres) and owns the
/// niwelety that make up the layout.
class MapProject {
public:
    /// Creates a project using @p crs, or EPSG:2180 by default.
    explicit MapProject(Crs crs = Crs::default_crs());

    [[nodiscard]] Crs crs() const noexcept { return crs_; }

    void add_niweleta(Niweleta niweleta);
    [[nodiscard]] const std::vector<Niweleta>& niwelety() const noexcept {
        return niwelety_;
    }

private:
    Crs crs_;
    std::vector<Niweleta> niwelety_;
};

}  // namespace maj0sted::domain
