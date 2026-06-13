#pragma once

#include <eu07/geo/puwg1992.hpp>
#include <eu07/scene/node/types.hpp>

#include <cstddef>
#include <optional>

namespace eu07::nmt {

struct RouteChainageSample {
    double chainageM = 0.0;
    scene::Vec3 sim{};
    geo::PuwgPoint geo{};
    double trackHeight = 0.0;
    std::optional<double> nmtHeight;
    int routeIndex = 0;
};

struct ProfileNmtResult {
    std::size_t samplesWithNmt = 0;
    std::size_t tilesLoaded = 0;
};

} // namespace eu07::nmt
