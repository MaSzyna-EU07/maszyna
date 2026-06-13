#pragma once

// Kafel ASC w RAM + cache między trasami (lazy load per kafel).

#include <eu07/geo/puwg1992.hpp>
#include <eu07/nmt/asc_index.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace eu07::nmt {

struct AscTileRam {
    AscHeader header{};
    std::shared_ptr<const std::vector<float>> heights;
    std::filesystem::path relativePath;

    [[nodiscard]] std::size_t index(const int row, const int col) const {
        return static_cast<std::size_t>(row) * static_cast<std::size_t>(header.ncols) + static_cast<std::size_t>(col);
    }

    [[nodiscard]] bool validAt(const int row, const int col) const {
        if (heights == nullptr || row < 0 || row >= header.nrows || col < 0 || col >= header.ncols) {
            return false;
        }
        const float z = (*heights)[index(row, col)];
        return !std::isnan(z);
    }

    [[nodiscard]] std::optional<double> heightAt(const int row, const int col) const {
        if (!validAt(row, col)) {
            return std::nullopt;
        }
        return static_cast<double>((*heights)[index(row, col)]);
    }
};

namespace detail {

inline std::shared_ptr<std::vector<float>> makeMutableHeightsBuffer(const AscHeader& header) {
    const std::size_t cellCount =
        static_cast<std::size_t>(header.nrows) * static_cast<std::size_t>(header.ncols);
    return std::make_shared<std::vector<float>>(
        cellCount, std::numeric_limits<float>::quiet_NaN());
}

inline std::vector<char> readAscFileBuffer(const std::filesystem::path& ascPath) {
    std::ifstream file(ascPath, std::ios::binary | std::ios::ate);
    if (!file) {
        return {};
    }
    const std::streamsize size = file.tellg();
    if (size <= 0) {
        return {};
    }
    file.seekg(0, std::ios::beg);
    std::vector<char> buffer(static_cast<std::size_t>(size) + 1);
    if (!file.read(buffer.data(), size)) {
        return {};
    }
    buffer[static_cast<std::size_t>(size)] = '\0';
    return buffer;
}

inline double fastParseDouble(char** ptr, char* endPtr) {
    char* p = *ptr;
    while (p < endPtr && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
        ++p;
    }
    if (p >= endPtr) {
        *ptr = p;
        return 0.0;
    }

    bool neg = false;
    if (*p == '-') {
        neg = true;
        ++p;
    } else if (*p == '+') {
        ++p;
    }

    double val = 0.0;
    while (p < endPtr && *p >= '0' && *p <= '9') {
        val = val * 10.0 + static_cast<double>(*p - '0');
        ++p;
    }
    if (p < endPtr && (*p == '.' || *p == ',')) {
        ++p;
        double frac = 1.0;
        while (p < endPtr && *p >= '0' && *p <= '9') {
            frac *= 0.1;
            val += static_cast<double>(*p - '0') * frac;
            ++p;
        }
    }

    *ptr = p;
    return neg ? -val : val;
}

inline void fastSkipToken(char** ptr, char* endPtr) {
    char* p = *ptr;
    while (p < endPtr && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
        ++p;
    }
    while (p < endPtr && !(*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) {
        ++p;
    }
    *ptr = p;
}

inline bool parseAscHeaderFromBuffer(char*& ptr, char* endPtr, AscHeader& header) {
    for (int h = 0; h < 6; ++h) {
        fastSkipToken(&ptr, endPtr);
        if (h == 0) {
            header.ncols = static_cast<int>(fastParseDouble(&ptr, endPtr));
        } else if (h == 1) {
            header.nrows = static_cast<int>(fastParseDouble(&ptr, endPtr));
        } else if (h == 2) {
            header.xll = fastParseDouble(&ptr, endPtr);
        } else if (h == 3) {
            header.yll = fastParseDouble(&ptr, endPtr);
        } else if (h == 4) {
            header.cellsize = fastParseDouble(&ptr, endPtr);
        } else {
            header.nodata = fastParseDouble(&ptr, endPtr);
        }
    }
    return header.ncols > 0 && header.nrows > 0 && header.cellsize > 0.0;
}

[[nodiscard]] inline std::optional<AscTileRam> loadAscTileRam(
    const AscIndexEntry& entry,
    const std::filesystem::path& ascPath) {
    std::vector<char> buffer = readAscFileBuffer(ascPath);
    if (buffer.empty()) {
        return std::nullopt;
    }

    char* ptr = buffer.data();
    char* endPtr = buffer.data() + buffer.size() - 1;

    AscTileRam tile;
    tile.relativePath = entry.relativePath;

    if (!parseAscHeaderFromBuffer(ptr, endPtr, tile.header)) {
        return std::nullopt;
    }
    if (tile.header.ncols > 20000 || tile.header.nrows > 20000) {
        return std::nullopt;
    }

    const std::shared_ptr<std::vector<float>> heights = makeMutableHeightsBuffer(tile.header);
    tile.heights = heights;

    for (int row = 0; row < tile.header.nrows; ++row) {
        for (int col = 0; col < tile.header.ncols; ++col) {
            const double value = fastParseDouble(&ptr, endPtr);
            const std::size_t idx =
                static_cast<std::size_t>(row) * static_cast<std::size_t>(tile.header.ncols) +
                static_cast<std::size_t>(col);
            if (std::abs(value - tile.header.nodata) <= 0.1) {
                (*heights)[idx] = std::numeric_limits<float>::quiet_NaN();
            } else {
                (*heights)[idx] = static_cast<float>(value);
            }
        }
    }

    return tile;
}

} // namespace detail

struct AscTileCache {
    std::unordered_map<std::string, std::shared_ptr<const std::vector<float>>> heightsByPath;
    std::unordered_map<std::string, AscHeader> headerByPath;
    std::size_t ramBytes = 0;
    std::mutex mutex;

    [[nodiscard]] bool attachHeights(
        AscTileRam& tile,
        const AscIndexEntry& entry,
        const std::filesystem::path& catalogRoot) {
        const std::string key = entry.relativePath.generic_string();
        {
            std::lock_guard lock(mutex);
            const auto headerIt = headerByPath.find(key);
            const auto heightsIt = heightsByPath.find(key);
            if (headerIt != headerByPath.end() && heightsIt != heightsByPath.end()) {
                tile.header = headerIt->second;
                tile.heights = heightsIt->second;
                tile.relativePath = entry.relativePath;
                return tile.header.ncols > 0 && tile.header.nrows > 0;
            }
        }

        if (const std::optional<AscTileRam> loaded =
                detail::loadAscTileRam(entry, catalogRoot / entry.relativePath)) {
            std::lock_guard lock(mutex);
            const std::string storeKey = loaded->relativePath.generic_string();
            if (const auto heightsIt = heightsByPath.find(storeKey); heightsIt != heightsByPath.end()) {
                tile.header = headerByPath[storeKey];
                tile.heights = heightsIt->second;
            } else {
                heightsByPath.emplace(storeKey, loaded->heights);
                headerByPath.emplace(storeKey, loaded->header);
                ramBytes += loaded->heights->size() * sizeof(float);
                tile.header = loaded->header;
                tile.heights = loaded->heights;
            }
            tile.relativePath = loaded->relativePath;
            return tile.header.ncols > 0 && tile.header.nrows > 0;
        }
        return false;
    }
};

} // namespace eu07::nmt
