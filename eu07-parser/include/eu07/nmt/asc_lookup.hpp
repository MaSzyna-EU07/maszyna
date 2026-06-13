#pragma once

// NMT1 (ASC): wyszukiwanie komorki w promieniu od probki profilu toru.

#include <eu07/geo/puwg1992.hpp>
#include <eu07/nmt/asc_index.hpp>
#include <eu07/nmt/asc_ram.hpp>
#include <eu07/nmt/profile_types.hpp>

#include <cmath>
#include <filesystem>
#include <optional>
#include <vector>

namespace eu07::nmt {

[[nodiscard]] inline const AscIndexEntry* findAscEntryForPoint(
    const AscIndex& index,
    const double north,
    const double east) {
    for (const AscIndexEntry& entry : index.entries) {
        if (north >= entry.northMin && north <= entry.northMax && east >= entry.eastMin &&
            east <= entry.eastMax) {
            return &entry;
        }
    }
    return nullptr;
}

namespace detail {

inline bool locateCell(
    const AscHeader& header,
    const double north,
    const double east,
    int& outRow,
    int& outCol) {
    const double northTop = header.yll + header.nrows * header.cellsize;
    if (north < header.yll || north > northTop || east < header.xll ||
        east > header.xll + header.ncols * header.cellsize) {
        return false;
    }

    outCol = static_cast<int>(std::floor((east - header.xll) / header.cellsize));
    outRow = static_cast<int>(std::floor((northTop - north) / header.cellsize));
    return outCol >= 0 && outCol < header.ncols && outRow >= 0 && outRow < header.nrows;
}

inline scene::Vec3 cellCenterSim(
    const AscHeader& header,
    const int row,
    const int col,
    const geo::PuwgMasterOffset& master) {
    const double northTop = header.yll + header.nrows * header.cellsize;
    const double north = northTop - (static_cast<double>(row) + 0.5) * header.cellsize;
    const double east = header.xll + (static_cast<double>(col) + 0.5) * header.cellsize;
    return {
        master.eastM - east,
        0.0,
        north - master.northM,
    };
}

} // namespace detail

inline void fillProfileNmtHeights(
    std::vector<RouteChainageSample>& samples,
    const AscIndex& index,
    const std::filesystem::path& catalogRoot,
    const geo::PuwgMasterOffset& master,
    const double radiusM,
    AscTileCache& cache) {
    if (radiusM <= 0.0) {
        return;
    }

    const double radiusSq = radiusM * radiusM;
    AscTileRam tile;

    for (RouteChainageSample& sample : samples) {
        const AscIndexEntry* entry =
            findAscEntryForPoint(index, sample.geo.north, sample.geo.east);
        if (entry == nullptr) {
            continue;
        }

        if (!cache.attachHeights(tile, *entry, catalogRoot)) {
            continue;
        }

        int row0 = 0;
        int col0 = 0;
        if (!detail::locateCell(tile.header, sample.geo.north, sample.geo.east, row0, col0)) {
            continue;
        }

        const double cell = tile.header.cellsize;
        const int cellWindow = static_cast<int>(std::ceil(radiusM / cell)) + 1;

        double bestDistSq = radiusSq + 1.0;
        std::optional<double> bestHeight;

        for (int dr = -cellWindow; dr <= cellWindow; ++dr) {
            for (int dc = -cellWindow; dc <= cellWindow; ++dc) {
                const int row = row0 + dr;
                const int col = col0 + dc;
                if (row < 0 || row >= tile.header.nrows || col < 0 || col >= tile.header.ncols) {
                    continue;
                }

                const std::optional<double> height = tile.heightAt(row, col);
                if (!height) {
                    continue;
                }

                const scene::Vec3 cellSim = detail::cellCenterSim(tile.header, row, col, master);
                const double dx = cellSim.x - sample.sim.x;
                const double dz = cellSim.z - sample.sim.z;
                const double distSq = dx * dx + dz * dz;
                if (distSq <= radiusSq && distSq < bestDistSq) {
                    bestDistSq = distSq;
                    bestHeight = height;
                }
            }
        }

        sample.nmtHeight = bestHeight;
    }
}

[[nodiscard]] inline ProfileNmtResult processRouteProfileNmt(
    const AscIndex& index,
    const std::filesystem::path& catalogRoot,
    const geo::PuwgMasterOffset& master,
    const double radiusM,
    std::vector<RouteChainageSample>& samples,
    AscTileCache& cache) {
    ProfileNmtResult result;
    const std::size_t tilesBefore = cache.heightsByPath.size();

    fillProfileNmtHeights(samples, index, catalogRoot, master, radiusM, cache);

    result.tilesLoaded = cache.heightsByPath.size() - tilesBefore;
    for (const RouteChainageSample& sample : samples) {
        if (sample.nmtHeight) {
            ++result.samplesWithNmt;
        }
    }
    return result;
}

} // namespace eu07::nmt
