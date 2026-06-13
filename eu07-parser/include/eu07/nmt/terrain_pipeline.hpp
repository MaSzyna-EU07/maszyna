#pragma once

// Pipeline: geometria toru -> profil co N m -> NMT z ASC -> eksport XYZ.

#include <eu07/geo/puwg1992.hpp>
#include <eu07/nmt/asc_index.hpp>
#include <eu07/nmt/asc_lookup.hpp>
#include <eu07/nmt/asc_ram.hpp>
#include <eu07/nmt/parallel.hpp>
#include <eu07/nmt/profile_export.hpp>
#include <eu07/nmt/profile_types.hpp>
#include <eu07/nmt/route_geometry.hpp>
#include <eu07/nmt/terrain_cli.hpp>
#include <eu07/scene/document.hpp>
#include <eu07/scene/track_routes.hpp>

#include <filesystem>
#include <iostream>
#include <print>
#include <vector>

namespace eu07::nmt {

inline void runTerrainProfile(
    const std::filesystem::path& stem,
    const scene::SceneDocument& doc,
    const scene::TrackRouteBuildResult& routes,
    const geo::PuwgMasterOffset& master,
    const std::filesystem::path& nmtDir,
    const TerrainProfileOptions& options) {
    const AscIndexResult loaded = loadOrBuildAscIndex(nmtDir);
    const std::filesystem::path catalogRoot = loaded.index.catalogRoot;

    std::println("\nNMT (ASC):");
    std::println("  indeks: {} plikow ({})",
        loaded.index.entries.size(),
        loaded.status == AscIndexStatus::Created       ? "utworzono"
        : loaded.status == AscIndexStatus::Refreshed ? "odswiezono"
                                                     : "wczytano");
    std::println("  profil: co {:.0f} m wzdloz osi toru", options.profileStepM);
    std::println("  NMT:    promien {:.1f} m od osi (najblizsza komorka ASC)", options.nmtRadiusM);
    std::println("  watki:  {}", workerThreadCount());
    std::cerr << std::flush;

    AscTileCache ascCache;
    std::vector<RouteChainageSample> allProfileSamples;
    allProfileSamples.reserve(routes.routes.size() * 512);

    const std::size_t routeCount = routes.routes.size();
    for (std::size_t routeIndex = 0; routeIndex < routeCount; ++routeIndex) {
        const scene::TrackRoute& route = routes.routes[routeIndex];
        std::cerr << "[NMT] trasa " << (routeIndex + 1) << '/' << routeCount << ": " << route.label
                  << " — geometria toru...\n"
                  << std::flush;

        const RouteGeometry geometry = buildRouteGeometry(doc, routes, routeIndex);
        if (geometry.segments.empty()) {
            std::println("  trasa {}: {} — brak geometrii, pomijam", routeIndex + 1, route.label);
            continue;
        }

        std::cerr << "[NMT] trasa " << (routeIndex + 1) << '/' << routeCount << " — profil co "
                  << options.profileStepM << " m + NMT r=" << options.nmtRadiusM << " m...\n"
                  << std::flush;

        std::vector<RouteChainageSample> samples =
            sampleRouteCenterlineFromGeometry(geometry, master, options.profileStepM);
        for (RouteChainageSample& sample : samples) {
            sample.routeIndex = static_cast<int>(routeIndex + 1);
        }

        const ProfileNmtResult nmt = processRouteProfileNmt(
            loaded.index,
            catalogRoot,
            master,
            options.nmtRadiusM,
            samples,
            ascCache);

        allProfileSamples.insert(allProfileSamples.end(), samples.begin(), samples.end());

        std::println(
            "  trasa {}: {} len={:.0f}m profil={} pkt (NMT {}) | cache {:.1f} MB",
            routeIndex + 1,
            route.label,
            geometry.totalLengthM,
            samples.size(),
            nmt.samplesWithNmt,
            static_cast<double>(ascCache.ramBytes) / (1024.0 * 1024.0));
    }

    if (!allProfileSamples.empty()) {
        const std::filesystem::path xyzPath = stem.string() + ".profil10m.txt";
        writeCombinedProfileXyz(xyzPath, allProfileSamples);
        std::println(
            "  profil XYZ: {} ({} pkt, ASCII: east north height, EPSG:2180)",
            xyzPath.string(),
            allProfileSamples.size());
    }
}

} // namespace eu07::nmt
