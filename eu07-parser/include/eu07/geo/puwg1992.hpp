#pragma once

// PUWG 1992 ↔ współrzędne sceny MaSzyny (jak terenAI GeoToSim).
// Master offset = pierwsza napotkana dyrektywa //$g w pliku sceny.

#include <eu07/parser.hpp>
#include <eu07/scene/node/types.hpp>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace eu07::geo {

struct PuwgMasterOffset {
    double eastM = 0.0;  // wschód [m]
    double northM = 0.0; // północ [m]
    std::size_t sourceLine = 0;
};

[[nodiscard]] inline double normalizePuwgKmToMeters(const double value) noexcept {
    return std::abs(value) < 10000.0 ? value * 1000.0 : value;
}

[[nodiscard]] inline std::optional<PuwgMasterOffset> parseGeoMapLine(std::string_view line) {
    std::string trimmed(line);
    while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n')) {
        trimmed.pop_back();
    }

    std::stringstream ss(trimmed);
    std::string tag;
    std::string system;
    double east = 0.0;
    double north = 0.0;
    if (!(ss >> tag >> system >> east >> north)) {
        return std::nullopt;
    }
    if (tag != "//$g") {
        return std::nullopt;
    }

    PuwgMasterOffset out;
    out.eastM = normalizePuwgKmToMeters(east);
    out.northM = normalizePuwgKmToMeters(north);
    return out;
}

[[nodiscard]] inline std::optional<PuwgMasterOffset> readFirstGeoMapFromFile(
    const std::filesystem::path& scnPath) {
    std::ifstream in(scnPath);
    if (!in) {
        return std::nullopt;
    }

    std::string line;
    for (std::size_t lineNo = 0; std::getline(in, line); ++lineNo) {
        if (line.find("//$g") == std::string::npos) {
            continue;
        }
        if (const std::optional<PuwgMasterOffset> parsed = parseGeoMapLine(line)) {
            PuwgMasterOffset out = *parsed;
            out.sourceLine = lineNo + 1;
            return out;
        }
    }
    return std::nullopt;
}

[[nodiscard]] inline std::optional<PuwgMasterOffset> readFirstGeoMap(
    const std::filesystem::path& scnPath,
    const std::vector<StarterDirective>& starters) {
    for (const StarterDirective& s : starters) {
        if (s.id != StarterId::GeoMap) {
            continue;
        }
        const std::string synthetic = std::string("//$g ") + s.value;
        if (const std::optional<PuwgMasterOffset> parsed = parseGeoMapLine(synthetic)) {
            PuwgMasterOffset out = *parsed;
            out.sourceLine = s.line + 1;
            return out;
        }
    }
    return readFirstGeoMapFromFile(scnPath);
}

struct PuwgPoint {
    double east = 0.0;
    double north = 0.0;
    double height = 0.0;
};

[[nodiscard]] inline scene::Vec3 geoToSim(
    const double north,
    const double east,
    const double height,
    const PuwgMasterOffset& master) noexcept {
    return {
        master.eastM - east,
        height,
        north - master.northM,
    };
}

[[nodiscard]] inline PuwgPoint simToGeo(const scene::Vec3& sim, const PuwgMasterOffset& master) noexcept {
    return {
        master.eastM - sim.x,
        sim.z + master.northM,
        sim.y,
    };
}

} // namespace eu07::geo
