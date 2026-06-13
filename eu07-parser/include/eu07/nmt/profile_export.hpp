#pragma once

#include <eu07/nmt/profile_height.hpp>
#include <eu07/nmt/profile_slope.hpp>
#include <eu07/nmt/profile_types.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <vector>

namespace eu07::nmt {

struct ProfileCsvOptions {
    bool includeRouteColumn = false;
};

inline void writeProfileCsv(
    const std::filesystem::path& outPath,
    const std::vector<RouteChainageSample>& samples,
    const ProfileCsvOptions& options = {}) {
    std::ofstream out(outPath, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac: " + outPath.string());
    }

    const std::vector<std::optional<double>> slopes = slopePromilleEvery50m(samples);

    out << std::fixed << std::setprecision(3);
    if (options.includeRouteColumn) {
        out << "route,chainage_m,east,north,height,slope_promille\n";
    } else {
        out << "chainage_m,east,north,height,slope_promille\n";
    }

    for (std::size_t i = 0; i < samples.size(); ++i) {
        const RouteChainageSample& s = samples[i];
        if (options.includeRouteColumn) {
            out << s.routeIndex << ',';
        }
        out << s.chainageM << ',' << s.geo.east << ',' << s.geo.north << ','
            << profileSampleHeightZ(s) << ',';
        if (slopes[i]) {
            out << *slopes[i];
        }
        out << '\n';
    }
}

inline void writeCombinedProfileCsv(
    const std::filesystem::path& outPath,
    const std::vector<RouteChainageSample>& samples) {
    writeProfileCsv(outPath, samples, {.includeRouteColumn = true});
}

inline void writeCombinedProfileXyz(
    const std::filesystem::path& outPath,
    const std::vector<RouteChainageSample>& samples) {
    std::ofstream out(outPath, std::ios::binary);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac: " + outPath.string());
    }

    out << std::fixed << std::setprecision(1);
    for (const RouteChainageSample& s : samples) {
        out << s.geo.east << ' ' << s.geo.north << ' ' << profileSampleHeightZ(s) << '\n';
    }
}

inline void writeRouteProfileReport(
    const std::filesystem::path& outPath,
    const std::vector<RouteChainageSample>& samples) {
    std::ofstream out(outPath);
    if (!out) {
        throw std::runtime_error("Nie mozna zapisac: " + outPath.string());
    }

    out << std::fixed << std::setprecision(3);
    out << "# chainage_m east north track_h nmt_h  (PUWG1992 / EPSG:2180)\n";
    for (const RouteChainageSample& s : samples) {
        out << s.chainageM << ' ' << s.geo.east << ' ' << s.geo.north << ' ' << s.trackHeight << ' ';
        if (s.nmtHeight) {
            out << *s.nmtHeight;
        } else {
            out << '-';
        }
        out << '\n';
    }
}

} // namespace eu07::nmt
