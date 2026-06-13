#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <string_view>

namespace eu07::nmt {

struct TerrainProfileOptions {
    std::optional<std::filesystem::path> nmtDir;
    double profileStepM = 10.0;
    double nmtRadiusM = 1.0;
};

[[nodiscard]] inline TerrainProfileOptions parseTerrainProfileCli(int argc, char** argv) {
    TerrainProfileOptions opts;
    for (int i = 2; i < argc; ++i) {
        const std::string_view arg = argv[i];
        if (arg == "--nmt-dir" && i + 1 < argc) {
            opts.nmtDir = std::filesystem::path{argv[++i]};
        } else if (arg == "--step" && i + 1 < argc) {
            opts.profileStepM = std::stod(argv[++i]);
        } else if (arg == "--band" && i + 1 < argc) {
            opts.nmtRadiusM = std::stod(argv[++i]);
        } else if (arg == "--radius" && i + 1 < argc) {
            opts.nmtRadiusM = std::stod(argv[++i]);
        }
    }
    return opts;
}

} // namespace eu07::nmt
