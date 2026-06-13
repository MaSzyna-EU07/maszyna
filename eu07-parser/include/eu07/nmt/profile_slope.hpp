#pragma once

#include <eu07/nmt/profile_height.hpp>
#include <eu07/nmt/profile_types.hpp>

#include <cmath>
#include <cstddef>
#include <optional>
#include <vector>

namespace eu07::nmt {

[[nodiscard]] inline double horizontalDistanceXY(
    const RouteChainageSample& a,
    const RouteChainageSample& b) noexcept {
    const double dx = b.geo.east - a.geo.east;
    const double dy = b.geo.north - a.geo.north;
    return std::sqrt(dx * dx + dy * dy);
}

[[nodiscard]] inline std::optional<double> slopePermilleOverChainageSegment(
    const std::vector<RouteChainageSample>& samples,
    const std::size_t routeStart,
    const std::size_t routeEnd,
    const double segStartChainage,
    const double segEndChainage,
    std::optional<std::size_t>& outEndIndex) {
    std::optional<std::size_t> startIndex;
    std::optional<std::size_t> prevIndex;
    double pathLengthM = 0.0;

    for (std::size_t i = routeStart; i < routeEnd; ++i) {
        if (samples[i].chainageM + 1e-6 < segStartChainage) {
            continue;
        }
        if (samples[i].chainageM > segEndChainage + 1e-6) {
            break;
        }

        if (!startIndex) {
            startIndex = i;
        }

        if (prevIndex) {
            pathLengthM += horizontalDistanceXY(samples[*prevIndex], samples[i]);
        }
        prevIndex = i;
    }

    if (!startIndex || !prevIndex || *startIndex == *prevIndex || pathLengthM <= 1e-6) {
        outEndIndex = std::nullopt;
        return std::nullopt;
    }

    outEndIndex = prevIndex;
    const double dh =
        profileSampleHeightZ(samples[*prevIndex]) - profileSampleHeightZ(samples[*startIndex]);
    return dh / pathLengthM * 1000.0;
}

[[nodiscard]] inline std::vector<std::optional<double>> slopePromilleEvery50m(
    const std::vector<RouteChainageSample>& samples) {
    std::vector<std::optional<double>> slopes(samples.size());
    if (samples.empty()) {
        return slopes;
    }

    constexpr double kSlopeIntervalM = 50.0;

    for (std::size_t routeStart = 0; routeStart < samples.size();) {
        const int routeIndex = samples[routeStart].routeIndex;
        std::size_t routeEnd = routeStart;
        while (routeEnd < samples.size() && samples[routeEnd].routeIndex == routeIndex) {
            ++routeEnd;
        }

        const double maxChainage = samples[routeEnd - 1].chainageM;
        for (double segStart = 0.0; segStart <= maxChainage + 1e-6; segStart += kSlopeIntervalM) {
            const double segEndTarget = segStart + kSlopeIntervalM;
            const bool isLastSegment = segEndTarget >= maxChainage - 1e-6;
            const double segEndChainage = isLastSegment ? maxChainage : segEndTarget;

            std::optional<std::size_t> endIndex;
            const std::optional<double> slope = slopePermilleOverChainageSegment(
                samples,
                routeStart,
                routeEnd,
                segStart,
                segEndChainage,
                endIndex);
            if (slope && endIndex) {
                slopes[*endIndex] = *slope;
            }
        }

        routeStart = routeEnd;
    }

    return slopes;
}

} // namespace eu07::nmt
