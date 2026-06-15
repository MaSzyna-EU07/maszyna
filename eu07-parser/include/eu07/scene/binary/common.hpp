#pragma once

#include <eu07/scene/binary/io.hpp>

#include <array>
#include <cctype>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>

namespace eu07::scene::binary {

inline constexpr std::array<char, 4> kMagic{'E', 'U', '7', 'B'};
// Pole version w EU7B: 5 = TERR terrain-lite. 4 = slim node + packed mesh. 3 = float32. 2 = float64.
inline constexpr std::uint32_t kVersionLegacy = 1; // codec.hpp — nieobslugiwane w CLI
inline constexpr std::uint32_t kVersionRuntime = 6;
inline constexpr std::uint32_t kVersionRuntimeV7 = 7;
inline constexpr std::uint32_t kVersionRuntimeV8 = 8;
inline constexpr std::uint32_t kVersionRuntimeV9 = 9;
inline constexpr std::uint32_t kVersionRuntimeV5 = 5;
inline constexpr std::uint32_t kVersionRuntimeV4 = 4;
inline constexpr std::uint32_t kVersionRuntimeF32 = 3;
inline constexpr std::uint32_t kVersionRuntimeF64 = 2;

[[nodiscard]] inline bool isSupportedEu7bVersion(const std::uint32_t version) noexcept {
    return version == kVersionRuntimeV9 || version == kVersionRuntimeV8 ||
           version == kVersionRuntimeV7 || version == kVersionRuntime ||
           version == kVersionRuntimeV5 || version == kVersionRuntimeV4;
}

[[nodiscard]] inline std::string formatVersionName(const std::uint32_t version) {
    if (version == kVersionRuntimeV9) {
        return "RuntimeV9";
    }
    if (version == kVersionRuntimeV8) {
        return "RuntimeV8";
    }
    if (version == kVersionRuntimeV7) {
        return "RuntimeV7";
    }
    if (version == kVersionRuntime) {
        return "RuntimeV6";
    }
    if (version == kVersionRuntimeV5) {
        return "RuntimeV5";
    }
    if (version == kVersionRuntimeV4) {
        return "RuntimeV4";
    }
    if (version == kVersionRuntimeF32) {
        return "RuntimeF32";
    }
    if (version == kVersionRuntimeF64) {
        return "RuntimeF64";
    }
    if (version == kVersionLegacy) {
        return "Legacy";
    }
    return "Unknown";
}

[[nodiscard]] inline std::string formatVersionDescription(const std::uint32_t version) {
    if (version == kVersionRuntimeV9) {
        return "bake v9 + PROT v9 + PACK sekcji v13 (indices + cell sort)";
    }
    if (version == kVersionRuntimeV8) {
        return "bake v7 + PROT + PACK sekcji v12 (solo + compact inst)";
    }
    if (version == kVersionRuntimeV7) {
        return "bake v6 + MODL w chunkach PIDX/PACK per sekcja 1km (v7)";
    }
    if (version == kVersionRuntime) {
        return "bake + chunki STRS/INCL/TRAK/... + INCL site transform (v6)";
    }
    if (version == kVersionRuntimeV5) {
        return "bake + chunki STRS/INCL/TRAK/... + TERR batched per sekcja 1km (v5)";
    }
    if (version == kVersionRuntimeV4) {
        return "bake slim v4 — przebakeuj do v5 dla TERR";
    }
    if (version == kVersionRuntimeF32) {
        return "bake float32 v3 — przebakeuj do v4";
    }
    if (version == kVersionRuntimeF64) {
        return "bake float64 v2 — przebakeuj do v4";
    }
    if (version == kVersionLegacy) {
        return "legacy (nieobslugiwane w CLI)";
    }
    return "nieobslugiwana wersja " + std::to_string(version);
}

[[nodiscard]] inline bool isSceneBinaryPath(const std::filesystem::path& path) {
    const std::string ext = path.extension().string();
    if (ext.empty()) {
        return false;
    }
    std::string lower = ext;
    for (char& ch : lower) {
        ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
    }
    return lower == ".eu7";
}

[[nodiscard]] inline bool probeSceneBinaryMagic(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return false;
    }
    std::array<char, 4> magic{};
    in.read(magic.data(), 4);
    return in.good() && magic == kMagic;
}

[[nodiscard]] inline bool isSceneBinaryInput(const std::filesystem::path& path) {
    return isSceneBinaryPath(path) || probeSceneBinaryMagic(path);
}

[[nodiscard]] inline std::uint32_t peekSceneBinaryVersion(const std::filesystem::path& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        throw std::runtime_error("Nie mozna otworzyc: " + path.string());
    }
    std::array<char, 4> magic{};
    in.read(magic.data(), 4);
    if (!in || magic != kMagic) {
        throw std::runtime_error("EU7B: zly magic");
    }
    return io::readU32(in);
}

} // namespace eu07::scene::binary
